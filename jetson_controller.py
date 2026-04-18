#!/usr/bin/env python3
import time
import sys
import busio
import board
import serial
import threading
from adafruit_servokit import ServoKit

class JetsonHexapod:
    """Controls hexapod movement directly via I2C and tracks orientation via Serial IMU"""
    
    # Gait Constants
    STEP_DELAY = 0.5
    SETTLE_DELAY = 0.5
    KNEE_NEUTRAL = 110
    KNEE_LIFTED = 120
    KNEE_LOWERED = 95
    HIP_NEUTRAL = 90
    DEFAULT_SWING = 30
    
    CALIBRATION_DATA = {
        0: (100,550), 1: (180, 490),
        2: (80, 530), 3: (140, 460),
        4: (130, 580), 5: (130, 420),
        6: (80, 530),  7: (460, 150),
        8: (80, 530), 9: (460, 150),
        10: (570, 120), 11: (510, 200)
    }

    def __init__(self, imu_port='/dev/ttyACM0', baud_rate=9600):
        # IMU State
        self.current_yaw = 0.0
        self.yaw_lock = threading.Lock()
        self.imu_running = True
        
        # Start IMU Thread
        self.arduino_port = imu_port
        self.baud_rate = baud_rate
        self.imu_thread = threading.Thread(target=self._imu_reader_thread, daemon=True)
        self.imu_thread.start()
        
        # Setup I2C
        try:
            i2c = busio.I2C(board.SCL_1, board.SDA_1)
            self.kit = ServoKit(channels=16, i2c=i2c, address=0x41)
        except Exception as e:
            print(f"I2C Error: {e}. Check your SCL/SDA pins and 0x41 address.")
            sys.exit(1)
            
        # Apply Calibrations
        for pin, (val1, val2) in self.CALIBRATION_DATA.items():
            p_min = int(min(val1, val2) * 4.8828)
            p_max = int(max(val1, val2) * 4.8828)
            self.kit.servo[pin].set_pulse_width_range(p_min, p_max)
            
        # Initial Stance
        self.stand_up()
        
        # Wait for first IMU reading
        print("SYSTEM: Waiting for IMU data...")
        time.sleep(2)
        print(f"SYSTEM: IMU Initialized. Current yaw: {self.get_current_yaw():.1f}")

    def _imu_reader_thread(self):
        """Background thread to read YPR from Arduino."""
        return
        try:
            ser = serial.Serial(self.arduino_port, self.baud_rate, timeout=0.1)
            print(f"SYSTEM: Connected to Arduino IMU on {self.arduino_port}")
        except Exception as e:
            print(f"SYSTEM: IMU Serial connection failed: {e}")
            return

        while self.imu_running:
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("YPR:"):
                        parts = line.split("YPR:")[1].split(",")
                        with self.yaw_lock:
                            self.current_yaw = float(parts[0])
            except Exception:
                pass
            time.sleep(0.01)

    def get_current_yaw(self):
        with self.yaw_lock:
            return self.current_yaw

    def set_angle(self, servo_index, angle):
        angle = max(0, min(180, angle))
        val1, val2 = self.CALIBRATION_DATA[servo_index]
        if val1 > val2:
            actual_angle = 180 - angle
        else:
            actual_angle = angle
        self.kit.servo[servo_index].angle = actual_angle

    def stand_up(self):
        print("SYSTEM: Initializing Stance...")
        for p in [0, 2, 4]: self.set_angle(p, self.KNEE_LOWERED)
        time.sleep(1)
        for p in [6, 8, 10]: self.set_angle(p, self.KNEE_LOWERED)
        time.sleep(1)
        for pin in [1, 3, 5, 7, 9, 11]:
            self.set_angle(pin, self.HIP_NEUTRAL)
        time.sleep(0.5)

    def backward(self, swing_angle=DEFAULT_SWING):
        fwd = self.HIP_NEUTRAL - swing_angle
        # GROUP A
        for p in [2, 6, 10]: self.set_angle(p, self.KNEE_LOWERED)
        for p in [0, 4, 8]: self.set_angle(p, self.KNEE_LIFTED)
        time.sleep(self.STEP_DELAY)
        for p in [1, 5, 9]: self.set_angle(p, fwd)
        time.sleep(self.STEP_DELAY)
        for p in [0, 4, 8]: self.set_angle(p, self.KNEE_LOWERED - 5)
        time.sleep(self.STEP_DELAY)
        for p in [1, 5, 9]: self.set_angle(p, self.HIP_NEUTRAL)
        time.sleep(self.SETTLE_DELAY)

        # GROUP B
        for p in [2, 6, 10]: self.set_angle(p, self.KNEE_LIFTED)
        time.sleep(self.STEP_DELAY)
        for p in [3, 7, 11]: self.set_angle(p, fwd)
        time.sleep(self.STEP_DELAY)
        for p in [2, 6, 10]: self.set_angle(p, self.KNEE_LOWERED - 5)
        time.sleep(self.STEP_DELAY)
        for p in [3, 7, 11]: self.set_angle(p, self.HIP_NEUTRAL)
        time.sleep(self.SETTLE_DELAY)

    def forward(self, swing_angle=DEFAULT_SWING):
        back = self.HIP_NEUTRAL + swing_angle
        # GROUP A
        for p in [0, 4, 8]: self.set_angle(p, self.KNEE_LIFTED)
        time.sleep(self.STEP_DELAY)
        for p in [1, 5, 9]: self.set_angle(p, back)
        time.sleep(self.STEP_DELAY)
        for p in [0, 4, 8]: self.set_angle(p, self.KNEE_LOWERED)
        time.sleep(self.STEP_DELAY)
        for p in [1, 5, 9]: self.set_angle(p, self.HIP_NEUTRAL)
        time.sleep(self.SETTLE_DELAY)

        # GROUP B
        for p in [2, 6, 10]: self.set_angle(p, self.KNEE_LIFTED)
        time.sleep(self.STEP_DELAY)
        for p in [3, 7, 11]: self.set_angle(p, back)
        time.sleep(self.STEP_DELAY)
        for p in [2, 6, 10]: self.set_angle(p, self.KNEE_LOWERED - 3)
        time.sleep(self.STEP_DELAY)
        for p in [3, 7, 11]: self.set_angle(p, self.HIP_NEUTRAL)
        time.sleep(self.SETTLE_DELAY)

    def step_turn_right(self, swing_angle):
        fwd = self.HIP_NEUTRAL - swing_angle
        back = self.HIP_NEUTRAL + swing_angle
        # GROUP A
        for p in [2, 6, 10]: self.set_angle(p, self.KNEE_LOWERED)
        for p in [0, 4, 8]: self.set_angle(p, self.KNEE_LIFTED)
        time.sleep(self.STEP_DELAY)
        self.set_angle(1, fwd); self.set_angle(5, fwd); self.set_angle(9, back)
        time.sleep(self.STEP_DELAY)
        for p in [0, 4, 8]: self.set_angle(p, self.KNEE_LOWERED)
        time.sleep(self.STEP_DELAY)
        for p in [1, 5, 9]: self.set_angle(p, self.HIP_NEUTRAL)
        time.sleep(self.SETTLE_DELAY)

        # GROUP B
        for p in [2, 6, 10]: self.set_angle(p, self.KNEE_LIFTED)
        time.sleep(self.STEP_DELAY)
        self.set_angle(3, fwd); self.set_angle(7, back); self.set_angle(11, back)
        time.sleep(self.STEP_DELAY)
        for p in [2, 6, 10]: self.set_angle(p, self.KNEE_LOWERED - 5)
        time.sleep(self.STEP_DELAY)
        for p in [3, 7, 11]: self.set_angle(p, self.HIP_NEUTRAL)
        time.sleep(self.SETTLE_DELAY)

    def step_turn_left(self, swing_angle):
        fwd = self.HIP_NEUTRAL - swing_angle
        back = self.HIP_NEUTRAL + swing_angle
        # GROUP A
        for p in [2, 6, 10]: self.set_angle(p, self.KNEE_LOWERED)
        for p in [0, 4, 8]: self.set_angle(p, self.KNEE_LIFTED)
        time.sleep(self.STEP_DELAY)
        self.set_angle(1, back); self.set_angle(5, back); self.set_angle(9, fwd)
        time.sleep(self.STEP_DELAY)
        for p in [0, 4, 8]: self.set_angle(p, self.KNEE_LOWERED)
        time.sleep(self.STEP_DELAY)
        for p in [1, 5, 9]: self.set_angle(p, self.HIP_NEUTRAL)
        time.sleep(self.SETTLE_DELAY)

        # GROUP B
        for p in [2, 6, 10]: self.set_angle(p, self.KNEE_LIFTED)
        time.sleep(self.STEP_DELAY)
        self.set_angle(3, back); self.set_angle(7, fwd); self.set_angle(11, fwd)
        time.sleep(self.STEP_DELAY)
        for p in [2, 6, 10]: self.set_angle(p, self.KNEE_LOWERED)
        time.sleep(self.STEP_DELAY)
        for p in [3, 7, 11]: self.set_angle(p, self.HIP_NEUTRAL)
        time.sleep(self.SETTLE_DELAY)
    def normalize_angle(self, angle_norm):
            """Normalize angle to -180 to 180 range"""
            return (angle_norm + 180) % 360 - 180
    def turn(self, target_degrees_right, tolerance=8.0, max_iterations=15):
        """
        Turns relative to starting yaw.
        Positive degrees = Turn Right.
        Negative degrees = Turn Left.
        """
        start_yaw = self.get_current_yaw()
        print(f"SYSTEM: Initiating relative turn by {target_degrees_right:.1f} degrees.")
        
        swing_angle = self.DEFAULT_SWING
        iteration = 0
        previous_error = None
        
        while iteration < max_iterations:
            current_yaw = self.get_current_yaw()
            
            # 1. Calculate raw IMU change (handles the 180 to -180 wrap-around)
            measured_change = self.normalize_angle(current_yaw - start_yaw)
            
            # 2. Standard IMUs decrease yaw when turning right.
            # We flip the sign so turning Right = Positive degrees turned (matches camera)
            # NOTE: If your robot spins in circles forever, your IMU is mounted upside down.
            # If that happens, just change this to: degrees_turned_right = measured_change
            degrees_turned_right = -measured_change 
            
            # 3. Calculate remaining error
            error = target_degrees_right - degrees_turned_right
            
            if abs(error) <= tolerance:
                print(f"SYSTEM: Turn complete! Final yaw: {current_yaw:.1f}")
                return True
                
            # Overshoot Adaptive Control
            if previous_error is not None:
                if (previous_error > 0 and error < 0) or (previous_error < 0 and error > 0):
                    swing_angle = max(swing_angle // 2, 5)
                    print(f"SYSTEM: Overshoot. Reducing swing to {swing_angle}")
            
            # If error > 0, we haven't turned right enough.
            if error > 0:
                self.step_turn_right(swing_angle)
            else:
                self.step_turn_left(swing_angle)
                
            previous_error = error
            iteration += 1
            
        print(f"SYSTEM: WARNING: Turn timeout. Final yaw: {self.get_current_yaw():.1f}")
        return False

    def close(self):
        self.imu_running = False
