"""M04 acceptance test.

Drives the arm to HOVER from 10 different starting poses. Acceptance:
the arm visibly ends in the same physical pose every time.

Usage:
    python m04_named_poses.py --port COM10
"""

import argparse
import sys

from arm import Arm
from poses import HOME, READY, HOVER

# Ten varied starting poses inside per-joint limits. Each is a noticeable
# deviation from HOVER so the move is visible.
STARTS = (
    HOME,
    READY,
    (90,  80, 110,  80, 90,  77),  # shoulder back, elbow forward
    (90, 120,  60, 110, 90,  90),  # shoulder forward, elbow back
    ( 60, 90,  90,  90, 90,  77),  # base CCW
    (120, 90,  90,  90, 90,  77),  # base CW
    (90,  90,  90,  90, 60,  77),  # wrist roll CCW
    (90,  90,  90,  90,120,  77),  # wrist roll CW
    (90,  90,  90,  90, 90,  45),  # gripper closed
    (90,  90,  90,  90, 90, 110),  # gripper fully open
)


def run(port: str) -> None:
    with Arm(port) as arm:
        for i, start in enumerate(STARTS, 1):
            print(f"[{i:2d}/10] start={start}  ->  HOVER")
            arm.move_to(start)
            arm.wait_for_settle()
            arm.move_to(HOVER)
            arm.wait_for_settle()
        print("\nDone. M04 passes if the arm ended in the SAME visible pose all 10 trials.")
        arm.move_to(HOME)
        arm.wait_for_settle()


def main() -> int:
    parser = argparse.ArgumentParser(description="M04 named-poses acceptance test.")
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM10")
    args = parser.parse_args()
    run(args.port)
    return 0


if __name__ == "__main__":
    sys.exit(main())
