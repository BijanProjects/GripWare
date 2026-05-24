"""Python control bridge for the GripWare 6-DOF arm.

Wire format (must match Robot_Kol_Arduino_Kod firmware):
    "root armA elbow wristA wristB gripper\n"   six ints 0..180

Joint index map: 0=root, 1=armA(shoulder), 2=elbow, 3=wristA, 4=wristB, 5=gripper.

Typical usage:
    from arm import Arm
    from poses import HOME, HOVER

    with Arm("COM10") as arm:
        arm.move_to(HOME)
        arm.wait_for_settle()
        arm.move_to(HOVER)
        arm.wait_for_settle()
        arm.gripper_close()
"""

import time

import serial

BAUD = 115200
BOOT_DELAY_S = 2.0           # Arduino auto-resets on serial open
NUM_JOINTS = 6
SLEW_DEG_PER_S = 60.0        # matches firmware SLEW_DEG_PER_TICK / TICK_MS
JOINT_MIN = (0, 0, 0, 0, 0, 45)
JOINT_MAX = (180, 180, 180, 180, 180, 110)
BOOT_POSE = (90, 90, 90, 90, 90, 77)  # firmware init pose


class Arm:
    def __init__(self, port: str, baud: int = BAUD, boot_delay: float = BOOT_DELAY_S):
        self._ser = serial.Serial(port, baud, timeout=1)
        time.sleep(boot_delay)
        self._prev = None
        self._last = None

    def move_to(self, pose) -> None:
        """Send a 6-int pose packet. Values are clamped to per-joint limits."""
        if len(pose) != NUM_JOINTS:
            raise ValueError(f"pose must have {NUM_JOINTS} ints; got {len(pose)}")
        clamped = tuple(
            max(JOINT_MIN[i], min(JOINT_MAX[i], int(v))) for i, v in enumerate(pose)
        )
        line = " ".join(str(v) for v in clamped) + "\n"
        self._ser.write(line.encode("ascii"))
        self._ser.flush()
        self._prev = self._last if self._last is not None else BOOT_POSE
        self._last = clamped

    def set_joints(self, root, armA, elbow, wristA, wristB, gripper) -> None:
        """Send a pose given as six positional arguments."""
        self.move_to((root, armA, elbow, wristA, wristB, gripper))

    def home(self) -> None:
        from poses import HOME
        self.move_to(HOME)

    def gripper(self, value: int) -> None:
        """Set gripper angle while holding other joints at their last commanded values."""
        if self._last is None:
            raise RuntimeError("no prior pose; call move_to() or home() first")
        pose = list(self._last)
        pose[5] = int(value)
        self.move_to(pose)

    def gripper_open(self) -> None:
        self.gripper(JOINT_MAX[5])

    def gripper_close(self) -> None:
        self.gripper(JOINT_MIN[5])

    def wait_for_settle(self, deg_per_s: float = SLEW_DEG_PER_S, extra_s: float = 0.3) -> None:
        """Block until the firmware slew should have finished, based on max joint delta."""
        if self._prev is None or self._last is None:
            time.sleep(BOOT_DELAY_S)
            return
        max_delta = max(abs(a - b) for a, b in zip(self._last, self._prev))
        time.sleep(max_delta / deg_per_s + extra_s)

    @property
    def last_pose(self):
        return self._last

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
