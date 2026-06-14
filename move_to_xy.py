"""move_to_xy.py — drive the arm to a world (x, y, z) via IK, with a
three-axis straight-line demo.

This is the M11 calibration / demonstration driver: it ties kinematics.py
(the IK) to arm.py (the serial bridge). A target (x, y, z) in millimetres
(table frame: origin at the base axis, Z up from the table) is turned into a
servo command by `kinematics.ik()` and sent to the arm.

Straight lines in CARTESIAN space
---------------------------------
Interpolating in joint space would make the gripper tip bow along an arc.
To move the TIP in a straight line we sample the line in world space and
solve IK at each small step, so every intermediate pose lands on the line.
The on-MCU slew limiter then smooths the motion between samples.

The whole path is planned and reachability-checked BEFORE any motion, so an
unreachable point aborts cleanly with nothing sent (fail-closed).

Examples
--------
    # See the plan without hardware (also validates reachability):
    python move_to_xy.py --demo --dry-run
    python move_to_xy.py --goto 180 0 60 --dry-run

    # Drive the real arm (give your serial port):
    python move_to_xy.py --port COM10 --goto 180 0 60
    python move_to_xy.py --port COM10 --demo

NOTE: until the per-joint servo offsets in kinematics.JointMap are calibrated
(M11), commands may hit the 0/180 clamps and the physical path will not be
metrically straight. The geometry is correct; calibration makes it accurate.
Use --dry-run to watch which joints clamp, then tune the JointMaps.
"""

from __future__ import annotations

import argparse
import os
import time

from kinematics import ARM, ArmModel, Unreachable, ik, reach_limits


def plan_segment(p0, p1, steps, model: ArmModel = ARM, elbow_up: bool = True):
    """Linear Cartesian interpolation p0->p1; IK at each of steps+1 samples.

    Returns a list of ((x, y, z), servo_tuple). Raises Unreachable (naming the
    offending point) if any sample is outside the workspace.
    """
    out = []
    for i in range(steps + 1):
        t = i / steps if steps else 0.0
        x = p0[0] + (p1[0] - p0[0]) * t
        y = p0[1] + (p1[1] - p0[1]) * t
        z = p0[2] + (p1[2] - p0[2]) * t
        try:
            servo = ik(x, y, z, model, elbow_up)
        except Unreachable as exc:
            raise Unreachable(f"point ({x:.0f}, {y:.0f}, {z:.0f}) mm: {exc}") from None
        out.append(((x, y, z), servo))
    return out


def plan_path(keypoints, steps, model: ArmModel = ARM, elbow_up: bool = True):
    """Chain straight segments through a list of world keypoints."""
    plan = []
    for a, b in zip(keypoints, keypoints[1:]):
        seg = plan_segment(a, b, steps, model, elbow_up)
        if plan:                       # drop the shared join point
            seg = seg[1:]
        plan.extend(seg)
    return plan


def three_axis_keypoints(center, span):
    """Center, then a straight line along X, along Y, along Z (each 2*span
    long and passing through the center), returning to center between lines.
    """
    cx, cy, cz = center
    return [
        (cx, cy, cz),
        (cx - span, cy, cz), (cx + span, cy, cz),   # ---- X line ----
        (cx, cy, cz),
        (cx, cy - span, cz), (cx, cy + span, cz),   # ---- Y line ----
        (cx, cy, cz),
        (cx, cy, cz - span), (cx, cy, cz + span),   # ---- Z line ----
        (cx, cy, cz),
    ]


def run_plan(arm, plan, slew_deg_per_s, min_dwell: float = 0.05,
             margin: float = 0.05):
    """Send each pose, pacing by the firmware slew time so the tip actually
    reaches each sample before the next is sent (accurate line tracing).
    """
    # First pose: move and wait for a full settle from the boot pose.
    arm.move_to(plan[0][1])
    arm.wait_for_settle()
    prev = plan[0][1]
    for xyz, servo in plan[1:]:
        arm.move_to(servo)
        max_delta = max(abs(a - b) for a, b in zip(servo, prev))
        time.sleep(max(min_dwell, max_delta / slew_deg_per_s + margin))
        prev = servo


def print_plan(plan):
    print(f"planned {len(plan)} waypoints  (root armA elbow wristA wristB grip)")
    print("   idx |    x     y     z (mm) ->  servos")
    for i, (xyz, servo) in enumerate(plan):
        clamp = " <clamp>" if servo[1] in (0, 180) or servo[2] in (0, 180) \
            or servo[3] in (0, 180) else ""
        print(f"   {i:3d} | {xyz[0]:5.0f} {xyz[1]:5.0f} {xyz[2]:5.0f}  ->  "
              f"{servo}{clamp}")


def build_args():
    p = argparse.ArgumentParser(description="Drive the arm to world XYZ via IK.")
    p.add_argument("--port", default=os.environ.get("ARM_PORT", "COM10"),
                   help="serial port (default $ARM_PORT or COM10)")
    p.add_argument("--dry-run", action="store_true",
                   help="plan + reachability check only; do not open the serial port")
    p.add_argument("--goto", nargs=3, type=float, metavar=("X", "Y", "Z"),
                   help="move to a single world point (mm)")
    p.add_argument("--demo", action="store_true",
                   help="three orthogonal straight lines through the center")
    p.add_argument("--center", nargs=3, type=float, default=(170.0, 0.0, 120.0),
                   metavar=("X", "Y", "Z"), help="demo center (mm)")
    p.add_argument("--span", type=float, default=40.0,
                   help="half-length of each demo line (mm)")
    p.add_argument("--steps", type=int, default=20,
                   help="interpolation samples per segment")
    p.add_argument("--elbow-down", action="store_true",
                   help="use the elbow-down IK branch (default is elbow-up)")
    return p.parse_args()


def main() -> int:
    args = build_args()
    elbow_up = not args.elbow_down

    if not args.goto and not args.demo:
        print("nothing to do: pass --goto X Y Z or --demo")
        return 2

    try:
        if args.goto:
            plan = plan_segment(tuple(args.goto), tuple(args.goto), 0, ARM, elbow_up)
            band = reach_limits(args.goto[2])
            if band:
                print(f"reach band at z={args.goto[2]:.0f}: "
                      f"{band[0]:.0f}..{band[1]:.0f} mm radius")
        else:
            kp = three_axis_keypoints(tuple(args.center), args.span)
            plan = plan_path(kp, args.steps, ARM, elbow_up)
    except Unreachable as exc:
        print(f"ABORT (fail-closed): {exc}")
        return 1

    print_plan(plan)

    if args.dry_run:
        print("\n--dry-run: no serial port opened, nothing sent.")
        return 0

    from arm import Arm, SLEW_DEG_PER_S   # lazy: only needed for a real run
    print(f"\nopening {args.port} ...")
    with Arm(args.port) as arm:
        run_plan(arm, plan, SLEW_DEG_PER_S)
        arm.wait_for_settle()
    print("done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
