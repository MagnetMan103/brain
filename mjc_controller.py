"""
MuJoCo Hexapod Controller — simulates the hexapod gaits in MuJoCo.

Mirrors the Hexapod class API from controller.py but drives MuJoCo position
actuators instead of serial commands.  Angles are tracked internally in
servo-degrees (same convention as the real controller) and converted to
radians with left/right hip mirroring when written to ctrl.

Usage:
    mjpython mjc_controller.py
"""

import os
import time
import math
import mujoco
import mujoco.viewer


# ============================================================================
# Leg / Joint Layout  (mirrors controller.py servo indices)
# ============================================================================
#
# Real controller servo layout:
#   Leg 1: knee=0,  hip=1     (front-left)
#   Leg 2: knee=2,  hip=3     (front-right)
#   Leg 3: knee=4,  hip=5     (mid-left)
#   Leg 4: knee=6,  hip=7     (mid-right)
#   Leg 5: knee=8,  hip=9     (rear-left)
#   Leg 6: knee=10, hip=11    (rear-right)
#
# MuJoCo joint mapping (from robot.xml body positions):
#   Left side  (low x):  hip1/knee1 (rear), hip2/knee2 (mid), hip3/knee3 (front)
#   Right side (high x): hip4/knee4 (front), hip5/knee5 (mid), hip6/knee6 (rear)
#
# Hip sign: left and right hips rotate in opposite directions.
#           left = +1, right = -1  (FLIP THESE IF DIRECTION IS WRONG)

class Leg:
    def __init__(self, name, knee, hip, mj_hip, mj_knee, hip_sign=1):
        self.name = name
        self.knee = knee          # index in the 12-element angles array
        self.hip = hip
        self.mj_hip = mj_hip     # MuJoCo actuator name for hip
        self.mj_knee = mj_knee   # MuJoCo actuator name for knee
        self.hip_sign = hip_sign  # +1 or -1 for left/right mirroring


LEGS = [
    Leg("front_left",  knee=0,  hip=1,  mj_hip="hip3",  mj_knee="knee3",  hip_sign=+1),
    Leg("front_right", knee=2,  hip=3,  mj_hip="hip4",  mj_knee="knee4",  hip_sign=-1),
    Leg("mid_left",    knee=4,  hip=5,  mj_hip="hip2",  mj_knee="knee2",  hip_sign=+1),
    Leg("mid_right",   knee=6,  hip=7,  mj_hip="hip5",  mj_knee="knee5",  hip_sign=-1),
    Leg("rear_left",   knee=8,  hip=9,  mj_hip="hip1",  mj_knee="knee1",  hip_sign=+1),
    Leg("rear_right",  knee=10, hip=11, mj_hip="hip6",  mj_knee="knee6",  hip_sign=-1),
]

TRIPOD_A = [LEGS[0], LEGS[3], LEGS[4]]  # front-left, mid-right, rear-left
TRIPOD_B = [LEGS[1], LEGS[2], LEGS[5]]  # front-right, mid-left, rear-right

LEFT_LEGS  = [LEGS[0], LEGS[2], LEGS[4]]
RIGHT_LEGS = [LEGS[1], LEGS[3], LEGS[5]]

# Default angles (servo degrees — same as real controller)
KNEE_NEUTRAL = 120
KNEE_LIFTED  = 150
HIP_NEUTRAL  = 90
NUM_SERVOS   = 12


# ============================================================================
# MuJoCo Hexapod Controller
# ============================================================================
class MjcHexapod:
    """
    Simulated hexapod for MuJoCo.  Same high-level API as the real Hexapod
    class (stand, sit, forward, backward, turn_left, turn_right, interpolate_to).
    """

    def __init__(self, model_path="hexmk2/scene.xml", step_time=0.25):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.step_time = step_time

        # Actuator name → ctrl index
        self._act_id = {}
        for i in range(self.model.nu):
            self._act_id[self.model.actuator(i).name] = i

        # Internal angles array (servo degrees, same as real controller)
        self.angles = [0] * NUM_SERVOS
        self._set_neutral()

        # Start the body above the floor so legs can settle down
        self.data.qpos[2] = 0.15

        # Write neutral ctrl and compute forward dynamics
        self._apply_ctrl()
        mujoco.mj_forward(self.model, self.data)

        # Launch passive viewer (renders in its own thread)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        # Let the robot drop and settle onto the ground
        self._sim_seconds(2.0)

    # ── Angle Conversion ─────────────────────────────────────────────

    @staticmethod
    def _servo_to_rad(servo_deg, neutral_deg, sign=1):
        """Servo degrees → MuJoCo radians.  Joint angle 0 = neutral."""
        return sign * math.radians(servo_deg - neutral_deg)

    def _apply_ctrl(self):
        """Push self.angles into data.ctrl (converted to radians)."""
        for leg in LEGS:
            hip_rad  = self._servo_to_rad(self.angles[leg.hip],  HIP_NEUTRAL,  leg.hip_sign)
            knee_rad = self._servo_to_rad(self.angles[leg.knee], KNEE_NEUTRAL)
            self.data.ctrl[self._act_id[leg.mj_hip]]  = hip_rad
            self.data.ctrl[self._act_id[leg.mj_knee]] = knee_rad

    # ── Simulation ───────────────────────────────────────────────────

    def _sim_seconds(self, duration):
        """Step the sim forward by *duration* seconds, rendering at ~60 fps."""
        n_steps = int(duration / self.model.opt.timestep)
        steps_per_frame = max(1, int((1.0 / 60.0) / self.model.opt.timestep))
        wall_start = time.time()

        for i in range(n_steps):
            if not self.viewer.is_running():
                return
            mujoco.mj_step(self.model, self.data)

            if (i + 1) % steps_per_frame == 0:
                self.viewer.sync()
                # real-time pacing
                elapsed_sim = (i + 1) * self.model.opt.timestep
                elapsed_wall = time.time() - wall_start
                wait = elapsed_sim - elapsed_wall
                if wait > 0:
                    time.sleep(wait)

        self.viewer.sync()

    # ── Angle Bookkeeping (identical to controller.py) ───────────────

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
        """Push current angle array into MuJoCo ctrl."""
        self._apply_ctrl()

    def commit_and_wait(self, duration=None):
        """Commit angles and simulate for the given duration."""
        self.commit()
        self._sim_seconds(duration if duration is not None else self.step_time)

    # ── Amplitude Helper ─────────────────────────────────────────────

    @staticmethod
    def _swing(amplitude):
        """Convert amplitude (10-100) to hip swing offset in degrees."""
        return int(10 + (amplitude - 10) * (50 - 10) / (100 - 10))

    # ── Basic Gaits (logic identical to controller.py) ───────────────

    def forward(self, amplitude=50, steps=1):
        swing = self._swing(amplitude)
        hip_fwd = HIP_NEUTRAL - swing
        for _ in range(steps):
            self._tripod_step(TRIPOD_A, TRIPOD_B, hip_fwd)
            self._tripod_step(TRIPOD_B, TRIPOD_A, hip_fwd)

    def backward(self, amplitude=50, steps=1):
        swing = self._swing(amplitude)
        hip_bwd = HIP_NEUTRAL + swing
        for _ in range(steps):
            self._tripod_step(TRIPOD_A, TRIPOD_B, hip_bwd)
            self._tripod_step(TRIPOD_B, TRIPOD_A, hip_bwd)

    def _tripod_step(self, swing_group, stance_group, hip_target):
        """One half of a tripod gait cycle: lift → swing → lower → push."""
        # Lift
        self._set_knees(swing_group, KNEE_LIFTED)
        self.commit_and_wait()
        # Swing to target
        self._set_hips(swing_group, hip_target)
        self.commit_and_wait()
        # Lower
        self._set_knees(swing_group, KNEE_NEUTRAL)
        self.commit_and_wait()
        # Power stroke back to neutral
        self._set_hips(swing_group, HIP_NEUTRAL)
        self._set_knees(swing_group, KNEE_NEUTRAL + 10)
        self.commit_and_wait()

    def turn_left(self, amplitude=50, steps=1):
        """Rotate counter-clockwise."""
        self._turn(amplitude, steps, left_target="backward", right_target="forward")

    def turn_right(self, amplitude=50, steps=1):
        """Rotate clockwise."""
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
                self._set_knees(swing_group, KNEE_LIFTED)
                self.commit_and_wait()

                for leg in swing_group:
                    if leg in LEFT_LEGS:
                        self.angles[leg.hip] = targets[left_target]
                    else:
                        self.angles[leg.hip] = targets[right_target]
                self.commit_and_wait()

                self._set_knees(swing_group, KNEE_NEUTRAL)
                self.commit_and_wait()

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
        """Smoothly move from current angles to target over duration seconds."""
        start = list(self.angles)
        n_frames = max(int(duration * fps), 1)
        dt = duration / n_frames
        for f in range(1, n_frames + 1):
            t = f / n_frames
            for i in range(NUM_SERVOS):
                self.angles[i] = int(start[i] + (target_angles[i] - start[i]) * t)
            self.commit()
            self._sim_seconds(dt)

    # ── IMU (from simulation state) ──────────────────────────────────

    def read_imu(self):
        """Return (yaw, pitch, roll) in degrees from the body quaternion."""
        qw, qx, qy, qz = self.data.qpos[3:7]

        sinr = 2 * (qw * qx + qy * qz)
        cosr = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr, cosr)

        sinp = 2 * (qw * qy - qz * qx)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))

        siny = 2 * (qw * qz + qx * qy)
        cosy = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny, cosy)

        return (math.degrees(yaw), math.degrees(pitch), math.degrees(roll))

    # ── Cleanup ──────────────────────────────────────────────────────

    def close(self):
        if self.viewer.is_running():
            self.viewer.close()


# ============================================================================
# Demo — same sequence as controller.py
# ============================================================================
if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, "hexmk2", "scene.xml")

    print("Loading MuJoCo hexapod...")
    bot = MjcHexapod(model_path, step_time=0.25)

    print("Standing...")
    bot.stand()
    bot._sim_seconds(1.0)

    print("Walking forward (3 steps)...")
    bot.forward(amplitude=80, steps=20)
    bot._sim_seconds(0.5)

    print("Turning right (2 steps)...")
    bot.turn_right(amplitude=50, steps=2)
    bot._sim_seconds(0.5)

    print("Walking backward (2 steps)...")
    bot.backward(amplitude=50, steps=2)
    bot._sim_seconds(0.5)

    print("Turning left (2 steps)...")
    bot.turn_left(amplitude=50, steps=2)
    bot._sim_seconds(0.5)

    print("Smooth sit...")
    bot.interpolate_to([60, 90] * 6, duration=1.0)
    bot._sim_seconds(1.0)

    print("Smooth stand...")
    bot.interpolate_to([KNEE_NEUTRAL, HIP_NEUTRAL] * 6, duration=1.0)
    bot._sim_seconds(1.0)

    imu = bot.read_imu()
    if imu:
        yaw, pitch, roll = imu
        print(f"IMU: yaw={yaw:.1f}  pitch={pitch:.1f}  roll={roll:.1f}")

    print("Done! Close the viewer window to exit.")
    while bot.viewer.is_running():
        time.sleep(0.1)

    bot.close()
