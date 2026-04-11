import time
import sys
import busio
import board
from adafruit_servokit import ServoKit

# ============================================================================
# Setup I2C and ServoKit
# ============================================================================
# Initializing I2C on Bus 1 of the Jetson Orin 40-pin header
try:
    i2c = busio.I2C(board.SCL_1, board.SDA_1)
    kit = ServoKit(channels=16, i2c=i2c, address=0x41)
except Exception as e:
    print(f"I2C Error: {e}. Check your SCL/SDA pins and 0x41 address.")
    sys.exit(1)

# ============================================================================
# Calibration & Constants
# ============================================================================
def count_to_us(count):
    """Converts PCA9685 register counts to microseconds."""
    return int(count * 4.8828)

# Updated Hip Calibrations and Original Knee Calibrations
CALIBRATION_DATA = {
        12: (150, 600), 14: (150, 600), 15: (150, 600)  # Reversed
}

# Apply calibration and determine orientation
for pin, (val1, val2) in CALIBRATION_DATA.items():
    p_min = count_to_us(min(val1, val2))
    p_max = count_to_us(max(val1, val2))
    kit.servo[pin].set_pulse_width_range(p_min, p_max)

# ============================================================================
# Core Movement Logic
# ============================================================================

def set_angle(servo_index, angle):
    """Sets angle while respecting reversed calibrations (val1 > val2)"""
    angle = max(0, min(180, angle))
    val1, val2 = CALIBRATION_DATA[servo_index]
    
    # Logic for reversed servos: if the first value is larger, 
    # then the movement direction is inverted relative to standard servos.
    if val1 > val2:
        actual_angle = 180 - angle
    else:
        actual_angle = angle
        
    kit.servo[servo_index].angle = actual_angle

def smooth_move(servo_index, start_angle, end_angle, speed=0.02):
    """
    Moves a servo slowly from start_angle to end_angle.
    'speed' is the delay between each 1-degree step.
    """
    step = 1 if end_angle > start_angle else -1
    
    for angle in range(start_angle, end_angle + step, step):
        set_angle(servo_index, angle)
        time.sleep(speed)

joint1_end = 20
joint2_end = 50
joint1_grab = 120
joint2_grab = 110
def calibrate():
    smooth_move(15, 0, 180)
    smooth_move(15, 180, 0)

def grab():
    smooth_move(15, joint2_end, joint2_grab)
    smooth_move(12,joint1_end, joint1_grab)
    time.sleep(1.5)
    set_angle(14, 160)
def away():
    smooth_move(12, joint1_grab, joint1_end)
    smooth_move(15, joint2_grab, joint2_end)
    time.sleep(3)
    set_angle(14, 120)
