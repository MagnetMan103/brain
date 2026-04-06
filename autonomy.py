#!/usr/bin/env python3
"""
autonomy.py - Autonomous Bottle Chase for Hexapod Robot
Integrates repo's existing Vision class with JetsonHexapod direct control.
"""

import time
import signal
import sys
from camera import Vision
from jetson_controller import JetsonHexapod

# Global flag for graceful shutdown
running = True

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

def calculate_bottle_angle(bottle_x, frame_width, camera_fov=60):
    """Calculate the angle to turn to face the bottle."""
    frame_center = frame_width / 2
    offset_fraction = (bottle_x - frame_center) / frame_center
    return offset_fraction * (camera_fov / 2)

def estimate_distance(bottle_width, frame_width):
    """Estimate relative distance to bottle based on its width."""
    width_ratio = bottle_width / frame_width
    if width_ratio > 0.15:
        return 'close'
    elif width_ratio > 0.08:
        return 'medium'
    else:
        return 'far'

def main():
    global running
    signal.signal(signal.SIGINT, signal_handler)
    
    print(f"{Colors.GREEN}{'='*70}{Colors.RESET}")
    print(f"{Colors.GREEN}Hexapod Bottle Chase Autonomy Loop{Colors.RESET}")
    print(f"{Colors.GREEN}{'='*70}{Colors.RESET}\n")
    
    try:
        print(f"{Colors.CYAN}Initializing Hexapod...{Colors.RESET}")
        hexapod = JetsonHexapod(imu_port='/dev/ttyACM0')
        
        print(f"\n{Colors.CYAN}Initializing Vision (YOLOv8)...{Colors.RESET}")
        # Initialize the repo's Vision system with Flask streaming
        brain = Vision("usb", "yolov8x.pt", stream=True)
        
        # Camera parameters (Updated for the 1280x720 USB camera)
        camera_fov = 60 # You may need to bump this up to ~75 if your USB cam is wide-angle
        target_close_threshold = 0.15
        
        print(f"{Colors.CYAN}Starting control loop...{Colors.RESET}\n")
        
        frames_without_bottle = 0
        max_frames_without_bottle = 5
        
        while running:
            # 1. Capture and process frame via Vision class
            brain.update()
            
            if brain.frame is None:
                continue
                
            frame_width = brain.frame.shape[1] # Dynamically gets 1280
            
            # 2. Extract Data 
            if brain.bottles:
                # Get most confident bottle from Vision
                best_bottle = max(brain.bottles, key=lambda b: b['conf'])
                
                # Convert the dict format from camera.py into autonomy metrics
                x1, y1 = best_bottle['x1'], best_bottle['y1']
                x2, y2 = best_bottle['x2'], best_bottle['y2']
                
                center_x = (x1 + x2) / 2
                width = x2 - x1
                confidence = best_bottle['conf']
                
                angle_to_bottle = calculate_bottle_angle(center_x, frame_width, camera_fov)
                distance = estimate_distance(width, frame_width)
                
                timestamp = time.strftime("%H:%M:%S")
                print(f"\n[{timestamp}] {Colors.GREEN}BOTTLE DETECTED{Colors.RESET}")
                print(f"  Size: {width}px ({width/frame_width*100:.1f}% of frame)")
                print(f"  Angle: {angle_to_bottle:.1f} degrees")
                print(f"  Confidence: {confidence*100:.1f}%")
                
                frames_without_bottle = 0
                
                # 3. Actions
                if distance == 'close':
                    print(f"\n{Colors.GREEN}TARGET REACHED! Bottle is close enough.{Colors.RESET}")
                    print(f"{Colors.YELLOW}Waiting 5 seconds before continuing search...{Colors.RESET}\n")
                    time.sleep(5)
                    continue
                
                x_position_ratio = center_x / frame_width
                is_drifting_left = x_position_ratio < 0.35
                is_drifting_right = x_position_ratio > 0.65
                
                needs_recentering = abs(angle_to_bottle) > 5 or is_drifting_left or is_drifting_right
                if needs_recentering:
                    print(f"{Colors.CYAN}Recentering bottle in frame...{Colors.RESET}")
                    
                    # angle_to_bottle is already positive for Right and negative for Left
                    hexapod.turn(angle_to_bottle, tolerance=5.0)
                    
                    print(f"{Colors.GREEN}Recenter complete.{Colors.RESET}\n")
                    continue
                else:
                    print(f"{Colors.GREEN}Bottle centered! Moving forward...{Colors.RESET}")
                    swing_amp = 30 if distance == 'far' else 15
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
        if 'hexapod' in locals():
            hexapod.close()
        if 'brain' in locals() and brain.cap:
            brain.cap.release()
        print(f"{Colors.GREEN}Shutdown complete.{Colors.RESET}")

if __name__ == "__main__":
    main()
