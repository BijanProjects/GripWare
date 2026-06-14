"""kinematics.py — forward + inverse kinematics for the GripWare 6-DOF arm.

Implements M10 (forward kinematics) and M12 (2-link planar IK) from ROADMAP.md.

The pick decomposition
----------------------
The arm is 6-DOF, but for a tabletop pick we hold the gripper pointing
straight DOWN. That collapses the 6-DOF problem into three independent,
closed-form pieces (no general IK solver, no singularity handling):

    base yaw          1-D rotation, atan2 of the target
    shoulder + elbow  closed-form 2-link planar IK (law of cosines)
    wrist pitch        chosen so the gripper stays vertical

Coordinate frames
-----------------
World / table frame (millimetres):
    X, Y   on the table plane; origin at the base rotation axis.
    Z      up from the table surface (Z = 0 is the table top).

The base yaw selects a vertical working plane; inside it we use (r, z):
    r = sqrt(X**2 + Y**2)      radial distance from the base axis
    z = height above the table

Joint-angle conventions (geometric, degrees)
-------------------------------------------
    shoulder : elevation of the lower-arm link above horizontal.
               0 = pointing horizontally outward (+r), +90 = straight up.
    elbow    : RELATIVE bend of the upper arm w.r.t. the lower arm.
               0 = arm fully straight; positive = upper arm rotates CCW
               (in the r-z plane) relative to the lower arm.
    wrist_a  : RELATIVE pitch of the gripper w.r.t. the upper arm, set so
               the gripper points straight down (-Z).

These geometric angles are turned into 0..180 servo commands by a per-joint
affine map (offset + sign), see `JointMap`. Those offsets are the M11
calibration knobs; the defaults below are placeholders.

Link lengths (mm)
-----------------
L1/L2 come from the STL shaft-boss centres found by stl_analyze.py (the
rotation axes, not the part extents) -- see STL_REPORT.md. L3 and the
shoulder height depend on how the gripper/base are assembled, so they are
placeholders to MEASURE (ruler/caliper or AprilTag) during M10/M11. Edit the
constants in `ArmModel` (or pass your own model) once you have real values.

Run the built-in self-test (no hardware needed):
    python kinematics.py
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, replace
from pathlib import Path


class Unreachable(ValueError):
    """Raised when a requested (r, z) lies outside the arm's reach."""


@dataclass(frozen=True)
class JointMap:
    """Affine map between a geometric joint angle (deg) and a 0..180 servo value.

        servo = clamp(servo_at_zero + servo_per_deg * angle_deg, lo, hi)
        angle = (servo - servo_at_zero) / servo_per_deg

    `servo_at_zero` is the servo value when the joint is at geometric 0;
    `servo_per_deg` carries both sign (direction) and scale (servo units per
    degree, in case of gearing or the mirrored shoulder). Defaults (90, 1.0)
    are an uncalibrated 1:1 placeholder. Calibrate with the dashboard panel ->
    calibration.json -> load_calibration().
    """

    servo_at_zero: float = 90.0
    servo_per_deg: float = 1.0
    lo: float = 0.0
    hi: float = 180.0

    def to_servo(self, angle_deg: float) -> int:
        v = self.servo_at_zero + self.servo_per_deg * angle_deg
        return int(round(max(self.lo, min(self.hi, v))))

    def to_angle(self, servo: float) -> float:
        return (servo - self.servo_at_zero) / self.servo_per_deg

    @staticmethod
    def from_two_points(s1, a1, s2, a2, lo=0.0, hi=180.0) -> "JointMap":
        """Fit from two (servo, geometric-angle) captures."""
        if s2 == s1:
            raise ValueError("two captures share the same servo value")
        m = (a2 - a1) / (s2 - s1)          # angle = m*servo + c
        k = 1.0 / m                         # servo = k*angle + b
        b = s1 - k * a1
        return JointMap(servo_at_zero=b, servo_per_deg=k, lo=lo, hi=hi)


@dataclass(frozen=True)
class ArmModel:
    """Geometry + servo calibration for the arm. All lengths in mm.

    Defaults are FIRST ESTIMATES (STL bounding boxes). Tune later.
    """

    # --- link lengths (pivot-to-pivot, mm) -------------------------------
    # L1/L2 measured from the STL shaft-boss centres (see stl_analyze.py /
    # STL_REPORT.md): the rotation axes, not the part extents.
    L1: float = 199.0        # shoulder -> elbow   (Alt_Kol shaft bosses Z=-105..+94)
    L2: float = 141.0        # elbow    -> wrist    (On_Kol  shaft bosses X=-141..0)
    # L3 and H_SHOULDER depend on assembly, not a single part -- MEASURE these
    # (ruler/caliper or AprilTag) before trusting metric output. Placeholders:
    L3: float = 100.0        # wrist pivot -> gripper tip (Bilek+roll+El+Parmak stack)
    H_SHOULDER: float = 200.0  # table -> shoulder pivot height

    # --- per-joint servo calibration (M11) -------------------------------
    base_map: JointMap = field(default_factory=JointMap)
    shoulder_map: JointMap = field(default_factory=JointMap)
    elbow_map: JointMap = field(default_factory=JointMap)
    wrist_a_map: JointMap = field(default_factory=JointMap)

    # wrist roll + gripper are not part of the position solve
    WRIST_B_NEUTRAL: int = 90
    GRIPPER_OPEN: int = 110   # matches firmware JOINT_MAX[5]
    GRIPPER_CLOSED: int = 45  # matches firmware JOINT_MIN[5]


ARM = ArmModel()


# Maps calibration.json joint keys (the firmware/dashboard names) to the
# ArmModel field that holds that joint's servo map.
_CAL_KEY_TO_FIELD = {
    "root": "base_map",
    "armA": "shoulder_map",
    "armB": "elbow_map",
    "wristA": "wrist_a_map",
}


def load_calibration(path="calibration.json", base: ArmModel = ARM) -> ArmModel:
    """Build an ArmModel from a calibration.json exported by the dashboard
    Calibration panel. Missing fields fall back to `base`. See the panel's
    export for the schema (joints[*].servo_at_zero/servo_per_deg, L3_mm,
    H_shoulder_mm, gripper_open/closed).
    """
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    joints = data.get("joints", {})
    overrides = {}
    for key, field_name in _CAL_KEY_TO_FIELD.items():
        d = joints.get(key)
        if not d or "servo_at_zero" not in d or "servo_per_deg" not in d:
            continue
        overrides[field_name] = JointMap(
            servo_at_zero=float(d["servo_at_zero"]),
            servo_per_deg=float(d["servo_per_deg"]),
            lo=float(d.get("lo", 0.0)), hi=float(d.get("hi", 180.0)))
    if "L3_mm" in data:
        overrides["L3"] = float(data["L3_mm"])
    if "H_shoulder_mm" in data:
        overrides["H_SHOULDER"] = float(data["H_shoulder_mm"])
    if "gripper_open" in data:
        overrides["GRIPPER_OPEN"] = int(data["gripper_open"])
    if "gripper_closed" in data:
        overrides["GRIPPER_CLOSED"] = int(data["gripper_closed"])
    return replace(base, **overrides)


# ---------------------------------------------------------------------------
# Forward kinematics
# ---------------------------------------------------------------------------

def fk_planar(shoulder_deg: float, elbow_deg: float, model: ArmModel = ARM):
    """Wrist-pivot position in the vertical plane, relative to the SHOULDER
    pivot. Returns (r, z) in mm where r is radial, z is up.
    """
    t1 = math.radians(shoulder_deg)               # link 1 absolute angle
    t2 = math.radians(shoulder_deg + elbow_deg)    # link 2 absolute angle
    r = model.L1 * math.cos(t1) + model.L2 * math.cos(t2)
    z = model.L1 * math.sin(t1) + model.L2 * math.sin(t2)
    return r, z


def fk_tip(shoulder_deg: float, elbow_deg: float, model: ArmModel = ARM):
    """Gripper-tip position in the WORLD (table) frame, assuming the gripper
    points straight down. Returns (r, z) in mm: r radial from the base axis,
    z height above the table.
    """
    r, z_rel = fk_planar(shoulder_deg, elbow_deg, model)
    z_world = model.H_SHOULDER + z_rel - model.L3   # tip hangs L3 below wrist
    return r, z_world


# ---------------------------------------------------------------------------
# Inverse kinematics
# ---------------------------------------------------------------------------

def ik_planar(r: float, z_world: float, model: ArmModel = ARM,
              elbow_up: bool = True):
    """2-link planar IK. Place the gripper tip at (r, z_world) with the
    gripper pointing straight down.

    Returns geometric angles (shoulder_deg, elbow_deg, wrist_a_deg).
    Raises `Unreachable` if (r, z_world) is outside the workspace.

    `elbow_up=True` keeps the elbow above the shoulder->wrist line (the safe
    choice for a tabletop pick so the lower arm does not dive into the table).
    """
    L1, L2 = model.L1, model.L2

    # Wrist pivot sits L3 directly above the tip (gripper points down), and we
    # work relative to the shoulder pivot height.
    wr = r
    wz = (z_world - model.H_SHOULDER) + model.L3

    d2 = wr * wr + wz * wz
    d = math.sqrt(d2)
    if d > L1 + L2 + 1e-9:
        raise Unreachable(f"target too far: reach {d:.1f} mm > {L1 + L2:.1f} mm")
    if d < abs(L1 - L2) - 1e-9:
        raise Unreachable(f"target too close: reach {d:.1f} mm < {abs(L1 - L2):.1f} mm")

    # Relative angle between the two links (law of cosines). theta_rel = 0
    # means the arm is straight.
    cos_rel = (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    cos_rel = max(-1.0, min(1.0, cos_rel))     # guard FP drift at the limits
    theta_rel = math.acos(cos_rel)             # in [0, pi]
    if elbow_up:
        theta_rel = -theta_rel                 # elbow bends the other way

    # Shoulder elevation.
    t1 = math.atan2(wz, wr) - math.atan2(L2 * math.sin(theta_rel),
                                         L1 + L2 * math.cos(theta_rel))
    t2 = t1 + theta_rel                        # upper-arm absolute angle

    shoulder_deg = math.degrees(t1)
    elbow_deg = math.degrees(theta_rel)

    # Wrist pitch keeps the gripper vertical: gripper must point at -90 deg
    # (straight down) in the plane, so its relative angle to link 2 is
    # (-90 - link2_absolute).
    wrist_a_deg = -90.0 - math.degrees(t2)

    return shoulder_deg, elbow_deg, wrist_a_deg


def ik(x: float, y: float, z: float, model: ArmModel = ARM,
       elbow_up: bool = True, gripper: int | None = None):
    """Full IK from a world target (x, y, z) mm to a 6-int servo command
    tuple (root, armA, elbow, wristA, wristB, gripper) for arm.Arm.move_to().

    Raises `Unreachable` if the target cannot be reached.
    """
    yaw_deg = math.degrees(math.atan2(y, x))
    r = math.hypot(x, y)

    shoulder_deg, elbow_deg, wrist_a_deg = ik_planar(r, z, model, elbow_up)

    root = model.base_map.to_servo(yaw_deg)
    arm_a = model.shoulder_map.to_servo(shoulder_deg)
    elbow = model.elbow_map.to_servo(elbow_deg)
    wrist_a = model.wrist_a_map.to_servo(wrist_a_deg)
    wrist_b = model.WRIST_B_NEUTRAL
    grip = model.GRIPPER_OPEN if gripper is None else int(gripper)

    return (root, arm_a, elbow, wrist_a, wrist_b, grip)


def reach_limits(z_world: float, model: ArmModel = ARM):
    """Return (r_min, r_max) reachable radii at a given table height z_world,
    or None if the height itself is unreachable at any radius.
    """
    wz = (z_world - model.H_SHOULDER) + model.L3
    L1, L2 = model.L1, model.L2
    rmax2 = (L1 + L2) ** 2 - wz * wz
    rmin2 = (abs(L1 - L2)) ** 2 - wz * wz
    if rmax2 < 0:
        return None
    r_max = math.sqrt(rmax2)
    r_min = math.sqrt(rmin2) if rmin2 > 0 else 0.0
    return r_min, r_max


# ---------------------------------------------------------------------------
# Self-test (M10/M12 acceptance in software, no hardware required)
# ---------------------------------------------------------------------------

def _selftest() -> int:
    print("kinematics self-test")
    print(f"  model: L1={ARM.L1} L2={ARM.L2} L3={ARM.L3} "
          f"H_shoulder={ARM.H_SHOULDER} mm  (max reach {ARM.L1 + ARM.L2} mm)\n")

    # 1) FK->IK->FK round-trip over a grid of reachable poses. This validates
    #    the math regardless of the (uncalibrated) link lengths.
    max_err = 0.0
    n = 0
    for shoulder in range(-20, 91, 10):
        for elbow in range(-120, 1, 10):    # elbow_up region: theta_rel <= 0
            r, z = fk_tip(shoulder, elbow)
            try:
                s2, e2, _ = ik_planar(r, z, elbow_up=True)
            except Unreachable:
                continue
            r2, z2 = fk_tip(s2, e2)
            err = math.hypot(r2 - r, z2 - z)
            max_err = max(max_err, err)
            n += 1
    print(f"  [1] round-trip over {n} poses: max tip error = {max_err:.6f} mm")
    rt_ok = max_err < 1e-6

    # 2) Reachability sweep at a typical pick height (just above the table).
    z_pick = 40.0
    lim = reach_limits(z_pick)
    if lim:
        print(f"  [2] at z={z_pick} mm the reachable radius band is "
              f"{lim[0]:.0f} .. {lim[1]:.0f} mm")
    else:
        print(f"  [2] z={z_pick} mm is unreachable at any radius")

    # 3) A couple of worked examples: a reachable target and an absurd one.
    print("  [3] example IK solves (world mm -> servo tuple):")
    for tgt in [(150.0, 0.0, 40.0), (120.0, 120.0, 60.0), (0.0, 0.0, 40.0)]:
        try:
            cmd = ik(*tgt)
            # confirm the solve actually lands on the target
            s, e, _ = ik_planar(math.hypot(tgt[0], tgt[1]), tgt[2])
            r_chk, z_chk = fk_tip(s, e)
            back = (round(r_chk, 1), round(z_chk, 1))
            print(f"        target {tgt} -> servos {cmd}   (tip check r,z={back})")
        except Unreachable as exc:
            print(f"        target {tgt} -> UNREACHABLE ({exc})")

    print()
    if rt_ok:
        print("  PASS: FK/IK are mutually consistent to < 1e-6 mm.")
        return 0
    print("  FAIL: round-trip error too large -- check the math.")
    return 1


if __name__ == "__main__":
    raise SystemExit(_selftest())
