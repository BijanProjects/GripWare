"""Named poses for the GripWare arm.

Each pose is (root, armA, elbow, wristA, wristB, gripper) in degrees.

HOME    Arm centered, all joints at 90 except gripper at 77 (midpoint of [45, 110]).
        Used as the safe starting/parking pose.
READY   Slight forward lean from HOME, gripper open. Used as the pre-motion pose.
HOVER   Hand-tuned pose putting the gripper above the default workspace at a
        reachable height. Placeholder values — to be refined analytically in M12
        once 2-link IK is in place.
"""

HOME  = (90,  90,  90,  90, 90,  77)
READY = (90, 100,  85,  95, 90, 100)
HOVER = (90, 110,  70, 100, 90, 100)
