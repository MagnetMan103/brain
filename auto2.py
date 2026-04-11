#!/usr/bin/env python3
"""
autonomy.py - Autonomous Bottle Chase for Hexapod Robot
Integrates repo's existing Vision class with JetsonHexapod direct control.
"""

import time
import signal
import sys
import csv
from camera import Vision
from jetson_controller import JetsonHexapod
import grabber

# Import the separated IMU Logger
from imu_logger import IMULogger

# Global flag for graceful shutdown
running = True
grabbed = False

def signal_handler(sig, frame):
    global running
    print("\n\nShutdown signal received...")
    running = False

class Colors:
    GREEN = '\033[92m'
    BLUE = '\033[94m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    CYAN = '\033[96m'
    RESET = '\033[0m'

# ==========================================
# Discrete Action Logger
# ==========================================
class ActionLogger:
    def __init__(self, filename='action_log_1.csv'):
        self.filename = filename
        # Initialize file with headers
        with open(self.filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['timestamp', 'action_type', 'parameter'])

    def log(self, action_type, parameter=0.0):
        """Appends a single action event to the CSV."""
        with open(self.filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([time.time(), action_type, parameter])

# ==========================================
# Vision Helpers
# ==========================================
def calculate_bottle_angle(bottle_x, frame_width, camera_fov=60):
    """Calculate the angle to turn to face the bottle."""
    frame_center = frame_width / 2
    offset_fraction = (bottle_x - frame_center) / frame_center
    return offset_fraction * (camera_fov / 2)

def estimate_distance(bottle_width, frame_width):
    """Estimate relative distance to bottle based on its width."""
    width_ratio = bottle_width / frame_width
    
    if width_ratio > 0.255:
        return 'too_close'
    elif width_ratio > 0.24:
        return 'close'
    elif width_ratio > 0.18:
        return 'medium'
    else:
        return 'far'

# ==========================================
# Main Loop
# ==========================================
def main():
    global running, grabbed
    signal.signal(signal.SIGINT, signal_handler)
    
    print(f"{Colors.GREEN}{'='*70}{Colors.RESET}")
    print(f"{Colors.GREEN}Hexapod Bottle Chase Autonomy Loop{Colors.RESET}")
    print(f"{Colors.GREEN}{'='*70}{Colors.RESET}\n")
    
    # 1. Start Background Loggers
    print(f"{Colors.CYAN}Starting loggers...{Colors.RESET}")
    imu_thread = IMULogger(port='/dev/ttyACM0', filename='imu_log_1.csv')
    imu_thread.start()
    
    action_log = ActionLogger(filename='action_log_1.csv')
    action_log.log("SYSTEM_START", 0)
    
    try:
        print(f"{Colors.CYAN}Initializing Hexapod...{Colors.RESET}")
        hexapod = JetsonHexapod() 
        
        print(f"\n{Colors.CYAN}Initializing Vision (YOLOv8)...{Colors.RESET}")
        brain = Vision("usb", "yolov8x.pt", stream=True)
        
        camera_fov = 60 
        
        print(f"{Colors.CYAN}Starting control loop...{Colors.RESET}\n")
        
        frames_without_bottle = 0
        max_frames_without_bottle = 5
        
        while running:
            # 1. Capture and process frame via Vision class
            brain.update()
            
            if brain.frame is None:
                continue
                
            frame_width = brain.frame.shape[1] 
            
            # 2. Extract Data 
            if brain.bottles:
                best_bottle = max(brain.bottles, key=lambda b: b['conf'])
                
                x1, y1 = best_bottle['x1'], best_bottle['y1']
                x2, y2 = best_bottle['x2'], best_bottle['y2']
                
                center_x = (x1 + x2) / 2
                width = x2 - x1
                height = y2 - y1
                length = min(width, height)
                confidence = best_bottle['conf']
                
                angle_to_bottle = calculate_bottle_angle(center_x, frame_width, camera_fov)
                distance = estimate_distance(length, frame_width)
                
                timestamp = time.strftime("%H:%M:%S")
                print(f"\n[{timestamp}] {Colors.GREEN}BOTTLE DETECTED{Colors.RESET}")
                print(f"  Size: {width}px ({length/frame_width*100:.1f}% of frame)")
                print(f"  Angle: {angle_to_bottle:.1f} degrees")
                print(f"  Distance: {distance.upper()}")
                
                frames_without_bottle = 0
                x_position_ratio = center_x / frame_width
                is_drifting_left = x_position_ratio < 0.35
                is_drifting_right = x_position_ratio > 0.65
                needs_recentering = abs(angle_to_bottle) > 5 or is_drifting_left or is_drifting_right
                
                # 3. Actions
                if needs_recentering:
                    print(f"{Colors.CYAN}Recentering bottle in frame...{Colors.RESET}")
                    turn_swing = 30 if abs(angle_to_bottle) > 15 else 15
                    
                    if angle_to_bottle > 0:
                        print("Taking a step right...")
                        action_log.log("TURN_RIGHT", turn_swing)
                        hexapod.step_turn_right(turn_swing)
                    else:
                        print("Taking a step left...")
                        action_log.log("TURN_LEFT", turn_swing)
                        hexapod.step_turn_left(turn_swing)
                    
                    print(f"{Colors.GREEN}Step complete. Resampling vision...{Colors.RESET}\n")
                    continue
                
                if distance == 'too_close':
                    print(f"\n{Colors.YELLOW}Too close to target ({(length/frame_width*100):.1f}%). Backing up slightly...{Colors.RESET}")
                    action_log.log("BACKWARD", 15)
                    hexapod.backward(swing_angle=15) 
                    print(f"{Colors.GREEN}Move complete.{Colors.RESET}\n")
                    continue
                    
                elif distance == 'close':
                    print(f"\n{Colors.GREEN}TARGET REACHED! Bottle is in the sweet spot.{Colors.RESET}")
                    print(f"{Colors.YELLOW}Initiating grab sequence...{Colors.RESET}\n")
                    if not grabbed or True:
                        action_log.log("GRAB", 0)
                        grabber.grab()
                        grabber.away()
                        grabbed = True
                    time.sleep(5)
                    continue
                    
                else: 
                    print(f"{Colors.GREEN}Bottle centered! Moving forward...{Colors.RESET}")
                    swing_amp = 30 if distance == 'far' else 15
                    action_log.log("FORWARD", swing_amp)
                    hexapod.forward(swing_angle=swing_amp)
                    print(f"{Colors.GREEN}Move complete.{Colors.RESET}\n")
                    continue
                    
            else:
                frames_without_bottle += 1
                if frames_without_bottle == 1:
                    print(f"[{time.strftime('%H:%M:%S')}] No bottle detected. Searching...")
                if frames_without_bottle >= max_frames_without_bottle:
                    print(f"{Colors.YELLOW}Waiting for bottle...{Colors.RESET}")
                    frames_without_bottle = 0 
                time.sleep(0.1)

    except Exception as e:
        print(f"\n{Colors.RED}Error: {e}{Colors.RESET}")
        import traceback
        traceback.print_exc()
        
    finally:
        print(f"\n{Colors.YELLOW}Shutting down...{Colors.RESET}")
        
        action_log.log("SYSTEM_STOP", 0)
        
        print("Stopping IMU Logger...")
        if 'imu_thread' in locals():
            imu_thread.stop()
            imu_thread.join()
        
        if 'hexapod' in locals():
            hexapod.close()
        if 'brain' in locals() and brain.cap:
            brain.cap.release()
        print(f"{Colors.GREEN}Shutdown complete.{Colors.RESET}")

if __name__ == "__main__":
    main()
