""""Live joint-angle detector using AprilTags.

Setup
-----
1. Print two tag36h11 AprilTags at the same physical size (default 50 mm):
     - reference tag: ID 0  -> place flat on the table next to the arm
     - joint tag:     ID 1  -> mount on the joint, face it toward the camera
   Tag PNGs live in the AprilRobotics/apriltag-imgs repo, folder tag36h11.

2. Mount a USB webcam looking straight down (top view) so both tags appear
   roughly parallel to the image plane.

3. Config lives in a .env file at the repo root (gitignored). Defaults are
   baked in as fallbacks, so the script also runs with no .env present.
   Available keys (defaults shown):
       CAM_INDEX=0
       CAM_WIDTH=1280
       CAM_HEIGHT=720
       REF_TAG_ID=0
       JOINT_TAG_ID=1
       TAG_SIZE_MM=50
       SHOW_FPS=1

4. Install deps:   pip install -r requirements.txt

Run:    python AprilTag/joint_angle.py
Quit:   press Q (or Esc) in the video window.

What gets reported
------------------
For each frame in which both the reference and joint tags are detected,
the script computes the joint tag's full 6DOF pose expressed in the
reference tag's frame, then reports:

    Rx   rotation about the reference X axis (deg)
    Ry   rotation about the reference Y axis (deg)
    Rz   rotation about the reference Z axis (deg, the primary joint rot)
    dist 3D euclidean distance between the two tag centers (mm)

Euler angles use XYZ-intrinsic convention. With a top-down camera and
both tags lying flat, Rz is the dominant value; Rx and Ry stay near zero
unless the joint tilts out of the reference's plane.

The angle math relies on pose estimation, which needs a camera matrix
and the tag's physical size. The matrix here is synthesized from frame
dimensions (~60 deg horizontal FOV), so the readings are useful but not
metrically precise -- Rx/Ry and dist in particular will drift a bit.
Run a checkerboard calibration if you need sub-degree or mm accuracy.

"Overlay legend
--------------
On every detected tag, the script draws the tag's local coordinate frame:
    red   = +X axis (the direction whose image angle becomes the joint angle)
    green = +Y axis
    blue  = +Z axis (out of the tag, toward the camera)
On the tracked ref + joint tags, a wireframe cube also pops up out of the
tag surface so you can see the tag's orientation in 3D. Because we lack a
real camera calibration, the cube will look slightly skewed off-axis -- the
2D angle reading itself does not depend on this.
"""

import math
import os
import time
from pathlib import Path

import cv2
import numpy as np
from dotenv import load_dotenv
from pupil_apriltags import Detector

load_dotenv(Path(__file__).resolve().parent.parent / ".env")


def _env_int(name: str, default: int) -> int:
    return int(os.environ.get(name, default))


def _env_float(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def _camera_matrix(width: int, height: int):
    """Synthetic intrinsics from frame size: assumes ~60 deg horizontal FOV.

    Good enough to visualize tag orientation; not metrically accurate.
    Run a proper checkerboard calibration if you need real-world distances.
    """
    fx = fy = float(width)
    cx = width / 2.0
    cy = height / 2.0
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    dist = np.zeros(5, dtype=np.float64)
    return K, dist, (fx, fy, cx, cy)


def _project(points_3d, R, t, K, dist):
    rvec, _ = cv2.Rodrigues(R)
    img_pts, _ = cv2.projectPoints(np.asarray(points_3d, dtype=np.float64),
                                   rvec, t, K, dist)
    return img_pts.reshape(-1, 2).astype(int)


def draw_tag_axes(frame, det, K, dist, length):
    """Red/green/blue X/Y/Z axes from the tag center."""
    pts = _project(
        [(0, 0, 0), (length, 0, 0), (0, length, 0), (0, 0, length)],
        det.pose_R, det.pose_t, K, dist,
    )
    origin = tuple(pts[0])
    cv2.line(frame, origin, tuple(pts[1]), (0, 0, 255), 3)   # X red
    cv2.line(frame, origin, tuple(pts[2]), (0, 255, 0), 3)   # Y green
    cv2.line(frame, origin, tuple(pts[3]), (255, 0, 0), 3)   # Z blue


def draw_tag_cube(frame, det, K, dist, size, color):
    """Wireframe cube sitting on the tag, extruding toward the camera."""
    s = size / 2.0
    verts = [
        (-s, -s, 0), (s, -s, 0), (s, s, 0), (-s, s, 0),
        (-s, -s, size), (s, -s, size), (s, s, size), (-s, s, size),
    ]
    p = _project(verts, det.pose_R, det.pose_t, K, dist)
    edges = [(0, 1), (1, 2), (2, 3), (3, 0),
             (4, 5), (5, 6), (6, 7), (7, 4),
             (0, 4), (1, 5), (2, 6), (3, 7)]
    for i, j in edges:
        cv2.line(frame, tuple(p[i]), tuple(p[j]), color, 2)


def relative_pose(ref_det, joint_det):
    """Joint pose expressed in the reference tag's frame.

    Returns (rx_deg, ry_deg, rz_deg, dist_mm) where the Euler angles use
    XYZ intrinsic convention (rotate about X, then the new Y, then the new
    Z) and dist_mm is the 3D euclidean distance between tag centers.

    For a top-down camera with both tags lying flat, Rz is the dominant
    "joint rotation" and Rx/Ry should hover near zero. Rx/Ry growing means
    the joint tag is tilting out of the reference's plane.
    """
    R_ref = ref_det.pose_R
    t_ref = ref_det.pose_t.reshape(3)
    R_joint = joint_det.pose_R
    t_joint = joint_det.pose_t.reshape(3)

    R_rel = R_ref.T @ R_joint
    t_rel = R_ref.T @ (t_joint - t_ref)

    sy = math.sqrt(R_rel[0, 0] ** 2 + R_rel[1, 0] ** 2)
    if sy > 1e-6:
        rx = math.atan2(R_rel[2, 1], R_rel[2, 2])
        ry = math.atan2(-R_rel[2, 0], sy)
        rz = math.atan2(R_rel[1, 0], R_rel[0, 0])
    else:
        rx = math.atan2(-R_rel[1, 2], R_rel[1, 1])
        ry = math.atan2(-R_rel[2, 0], sy)
        rz = 0.0

    return (math.degrees(rx), math.degrees(ry), math.degrees(rz),
            float(np.linalg.norm(t_rel)))


def main() -> None:
    cam_index = _env_int("CAM_INDEX", 0)
    cam_width = _env_int("CAM_WIDTH", 1280)
    cam_height = _env_int("CAM_HEIGHT", 720)
    ref_id = _env_int("REF_TAG_ID", 0)
    joint_id = _env_int("JOINT_TAG_ID", 1)
    tag_size_mm = _env_float("TAG_SIZE_MM", 50.0)
    show_fps = _env_int("SHOW_FPS", 1)

    # DSHOW opens far faster than the default MSMF backend on Windows.
    cap = cv2.VideoCapture(cam_index, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_height)
    if not cap.isOpened():
        raise SystemExit(f"could not open camera index {cam_index}")

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    K, dist, cam_params = _camera_matrix(actual_w, actual_h)
    print(f"camera {cam_index}: {actual_w}x{actual_h}")
    print(f"looking for ref id={ref_id}, joint id={joint_id} (tag36h11)")
    print(f"tag size: {tag_size_mm} mm")
    print("press Q in the window to quit\n")

    detector = Detector(
        families="tag36h11",
        nthreads=2,
        quad_decimate=1.0,
        refine_edges=True,
    )

    last_print = 0.0
    fps_t0 = time.time()
    fps_frames = 0
    fps = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=cam_params,
            tag_size=tag_size_mm,
        )

        ref_det = None
        joint_det = None
        for d in detections:
            if d.tag_id == ref_id:
                ref_det = d
            elif d.tag_id == joint_id:
                joint_det = d

        for d in detections:
            pts = d.corners.astype(int).reshape(-1, 1, 2)
            tracked = d.tag_id in (ref_id, joint_id)
            color = (0, 255, 0) if tracked else (0, 165, 255)
            cv2.polylines(frame, [pts], True, color, 2)
            cx = int(d.center[0])
            bottom_y = int(max(p[1] for p in d.corners))
            cv2.putText(frame, f"id={d.tag_id}", (cx - 24, bottom_y + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            draw_tag_axes(frame, d, K, dist, tag_size_mm * 0.75)
            if tracked:
                draw_tag_cube(frame, d, K, dist, tag_size_mm, color)

        if ref_det is not None and joint_det is not None:
            rx, ry, rz, dist_mm = relative_pose(ref_det, joint_det)
            status_lines = [
                f"Rx: {rx:+7.2f} deg",
                f"Ry: {ry:+7.2f} deg",
                f"Rz: {rz:+7.2f} deg",
                f"dist: {dist_mm:7.1f} mm",
            ]
            now = time.time()
            if now - last_print > 0.1:
                print(f"\rRx:{rx:+7.2f} Ry:{ry:+7.2f} Rz:{rz:+7.2f}  "
                      f"d:{dist_mm:7.1f}mm  ", end="", flush=True)
                last_print = now
        else:
            missing = []
            if ref_det is None:
                missing.append(f"ref({ref_id})")
            if joint_det is None:
                missing.append(f"joint({joint_id})")
            status_lines = ["missing: " + ", ".join(missing)]

        for i, line in enumerate(status_lines):
            cv2.putText(frame, line, (12, 34 + i * 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        fps_frames += 1
        if fps_frames >= 30:
            now = time.time()
            fps = fps_frames / (now - fps_t0)
            fps_frames = 0
            fps_t0 = now
        if show_fps:
            cv2.putText(frame, f"{fps:5.1f} fps", (actual_w - 110, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)

        cv2.imshow("AprilTag joint angle", frame)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break

    cap.release()
    cv2.destroyAllWindows()
    print()


if __name__ == "__main__":
    main()
