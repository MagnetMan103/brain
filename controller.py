"""
Hexapod Controller — runs on the Jetson, sends servo commands to the Arduino.

Usage:
    from hexapod import Hexapod

    bot = Hexapod("/dev/ttyACM0")
    bot.forward(amplitude=60, steps=3)
    bot.close()

"""

import time
import serial
import sys


# ============================================================================
# Leg / Joint Layout
# ============================================================================
# Each leg has two servos: a knee (even index) and a hip (odd index).
#
#   Leg 1: knee=0,  hip=1     (front-left)
#   Leg 2: knee=2,  hip=3     (front-right)
#   Leg 3: knee=4,  hip=5     (mid-left)
#   Leg 4: knee=6,  hip=7     (mid-right)
#   Leg 5: knee=8,  hip=9     (rear-left)
#   Leg 6: knee=10, hip=11    (rear-right)

class Leg:
    def __init__(self, name, knee, hip):
        self.name = name
        self.knee = knee
        self.hip = hip

LEGS = [
    Leg("front_left",  knee=0,  hip=1),
    Leg("front_right", knee=2,  hip=3),
    Leg("mid_left",    knee=4,  hip=5),
    Leg("mid_right",   knee=6,  hip=7),
    Leg("rear_left",   knee=8,  hip=9),
    Leg("rear_right",  knee=10, hip=11),
]

# Tripod groups
TRIPOD_A = [LEGS[0], LEGS[2], LEGS[4]]  # front-left, mid-left, rear-left
TRIPOD_B = [LEGS[1], LEGS[3], LEGS[5]]  # front-right, mid-right, rear-right

# For turning, we need to know which side each leg is on
LEFT_LEGS  = [LEGS[0], LEGS[2], LEGS[4]]
RIGHT_LEGS = [LEGS[1], LEGS[3], LEGS[5]]

# ============================================================================
# Default Angles
# ============================================================================
KNEE_NEUTRAL = 120
KNEE_LIFTED  = 150
HIP_NEUTRAL  = 90

NUM_SERVOS = 12


# ============================================================================
# Low-Level Serial Interface
# ============================================================================
class ServoDriver:
    """Talks to the Arduino over serial."""

    def __init__(self, port, baud=115200, timeout=1.0):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2)  # wait for Arduino reset
        self._drain_startup_messages()

    def _drain_startup_messages(self):
        """Read and discard the boot messages (SYSTEM:Ready, etc.)."""
        deadline = time.time() + 2
        while time.time() < deadline:
            if self.ser.in_waiting:
                line = self.ser.readline().decode(errors="replace").strip()
                print(f"  [arduino] {line}")
            else:
                time.sleep(0.05)

    def set_servo(self, servo, angle):
        cmd = f"S,{servo},{angle}\n"
        self.ser.write(cmd.encode())
        self.ser.readline()  # consume "OK"

    def set_all(self, angles):
        """Send all 12 angles in one packet. Much faster than 12 singles."""
        body = ",".join(str(a) for a in angles)
        cmd = f"B,{body}\n"
        self.ser.write(cmd.encode())
        self.ser.readline()  # consume "OK"

    def read_imu(self):
        """Request a single IMU reading. Returns (yaw, pitch, roll) or None."""
        self.ser.write(b"I\n")
        line = self.ser.readline().decode(errors="replace").strip()
        if line.startswith("IMU:"):
            parts = line[4:].split(",")
            return (float(parts[0]), float(parts[1]), float(parts[2]))
        return None

    def close(self):
        self.ser.close()


# ============================================================================
# High-Level Hexapod Controller
# ============================================================================
class Hexapod:
    """
    Manages a 12-servo hexapod through the Arduino servo driver.

    All positions are tracked in a local array so we can use batch updates
    and do smooth interpolation.
    """

    def __init__(self, port="/dev/ttyACM0", step_time=0.25):
        self.driver = ServoDriver(port)
        self.step_time = step_time

        # Current target angle for every servo
        self.angles = [0] * NUM_SERVOS
        self._set_neutral()
        self.commit()

    # ── Angle Bookkeeping ────────────────────────────────────────────
    def _set_neutral(self):
        for leg in LEGS:
            self.angles[leg.knee] = KNEE_NEUTRAL
            self.angles[leg.hip]  = HIP_NEUTRAL

    def _set_knees(self, legs, angle):
        for leg in legs:
            self.angles[leg.knee] = angle

    def _set_hips(self, legs, angle):
        for leg in legs:
            self.angles[leg.hip] = angle

    def commit(self):
        """Push the current angle array to the Arduino in one batch."""
        self.driver.set_all(self.angles)

    def commit_and_wait(self, duration=None):
        """Commit, then wait for the servos to reach position."""
        self.commit()
        time.sleep(duration if duration is not None else self.step_time)

    # ── Amplitude Helper ─────────────────────────────────────────────
    @staticmethod
    def _swing(amplitude):
        """Convert amplitude (10-100) to hip swing offset in degrees."""
        return int(10 + (amplitude - 10) * (50 - 10) / (100 - 10))

    # ── Basic Gaits ──────────────────────────────────────────────────
    def forward(self, amplitude=50, steps=1):
        swing = self._swing(amplitude)
        hip_forward = HIP_NEUTRAL - swing

        for _ in range(steps):
            self._tripod_step(TRIPOD_A, TRIPOD_B, hip_forward)
            self._tripod_step(TRIPOD_B, TRIPOD_A, hip_forward)

    def backward(self, amplitude=50, steps=1):
        swing = self._swing(amplitude)
        hip_backward = HIP_NEUTRAL + swing

        for _ in range(steps):
            self._tripod_step(TRIPOD_A, TRIPOD_B, hip_backward)
            self._tripod_step(TRIPOD_B, TRIPOD_A, hip_backward)

    def _tripod_step(self, swing_group, stance_group, hip_target):
        """One half of a tripod gait cycle: lift -> swing -> lower -> push."""
        # Lift
        self._set_knees(swing_group, KNEE_LIFTED)
        self.commit_and_wait()

        # Swing to target
        self._set_hips(swing_group, hip_target)
        self.commit_and_wait()

        # Lower
        self._set_knees(swing_group, KNEE_NEUTRAL)
        self.commit_and_wait()

        # Return hips to neutral (power stroke)
        self._set_hips(swing_group, HIP_NEUTRAL)
        # put knees above
        self._set_knees(swing_group, KNEE_NEUTRAL + 10)
        self.commit_and_wait()

    def turn_left(self, amplitude=50, steps=1):
        """Rotate counter-clockwise: left legs back, right legs forward."""
        self._turn(amplitude, steps, left_target="backward", right_target="forward")

    def turn_right(self, amplitude=50, steps=1):
        """Rotate clockwise: left legs forward, right legs back."""
        self._turn(amplitude, steps, left_target="forward", right_target="backward")

    def _turn(self, amplitude, steps, left_target, right_target):
        swing = self._swing(amplitude)
        targets = {
            "forward":  HIP_NEUTRAL - swing,
            "backward": HIP_NEUTRAL + swing,
        }

        for _ in range(steps):
            for swing_group, stance_group in [(TRIPOD_A, TRIPOD_B),
                                              (TRIPOD_B, TRIPOD_A)]:
                # Lift
                self._set_knees(swing_group, KNEE_LIFTED)
                self.commit_and_wait()

                # Differential swing
                for leg in swing_group:
                    if leg in LEFT_LEGS:
                        self.angles[leg.hip] = targets[left_target]
                    else:
                        self.angles[leg.hip] = targets[right_target]
                self.commit_and_wait()

                # Lower
                self._set_knees(swing_group, KNEE_NEUTRAL)
                self.commit_and_wait()

                # Power stroke back to neutral
                self._set_hips(swing_group, HIP_NEUTRAL)
                self._set_knees(swing_group, KNEE_NEUTRAL + 10)
                self.commit_and_wait()

    # ── Poses ────────────────────────────────────────────────────────
    def stand(self):
        self._set_neutral()
        self.commit_and_wait()

    def sit(self):
        for leg in LEGS:
            self.angles[leg.knee] = 60
            self.angles[leg.hip]  = HIP_NEUTRAL
        self.commit_and_wait(0.5)

    # ── Smooth Motion ────────────────────────────────────────────────
    def interpolate_to(self, target_angles, duration=0.5, fps=50):
        """
        Smoothly move from current angles to target_angles over `duration`
        seconds, updating at `fps` frames per second.
        """
        start_angles = list(self.angles)
        num_frames = max(int(duration * fps), 1)
        frame_delay = duration / num_frames

        for frame in range(1, num_frames + 1):
            t = frame / num_frames
            for i in range(NUM_SERVOS):
                self.angles[i] = int(
                    start_angles[i] + (target_angles[i] - start_angles[i]) * t
                )
            self.commit()
            time.sleep(frame_delay)

    # ── IMU ──────────────────────────────────────────────────────────
    def read_imu(self):
        return self.driver.read_imu()

    # ── Cleanup ──────────────────────────────────────────────────────
    def close(self):
        self.driver.close()


# ============================================================================
# Demo — run this file directly to test basic gaits
# ============================================================================
if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    print(f"Connecting to Arduino on {port}...")

    bot = Hexapod(port, step_time=0.25)
    print("Standing up...")
    bot.stand()
    time.sleep(1)

    print("Walking forward (3 steps)...")
    bot.forward(amplitude=60, steps=3)
    time.sleep(0.5)

    print("Turning right...")
    bot.turn_right(amplitude=50, steps=2)
    time.sleep(0.5)

    print("Walking backward (2 steps)...")
    bot.backward(amplitude=50, steps=2)
    time.sleep(0.5)

    print("Turning left...")
    bot.turn_left(amplitude=50, steps=2)
    time.sleep(0.5)

    # Smooth interpolation demo
    print("Smooth sit...")
    sit_pose = [120, 90] * 6
    bot.interpolate_to(sit_pose, duration=1.0)
    time.sleep(1)

    print("Smooth stand...")
    stand_pose = [KNEE_NEUTRAL, HIP_NEUTRAL] * 6
    bot.interpolate_to(stand_pose, duration=1.0)
    time.sleep(1)

    # Read IMU
    imu = bot.read_imu()
    if imu:
        yaw, pitch, roll = imu
        print(f"IMU: yaw={yaw:.1f}  pitch={pitch:.1f}  roll={roll:.1f}")

    print("Done!")
    bot.close()
