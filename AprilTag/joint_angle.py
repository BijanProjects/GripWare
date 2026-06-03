"""Live joint-angle detector using AprilTags.

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

How the angle is computed
-------------------------
For a top-down camera with both tags lying flat, joint rotation about the
camera's optical axis appears as a pure in-plane rotation in the image.
Each tag detection gives four image-space corners ordered counter-clockwise
starting at the tag's bottom-left. The vector from corner[0] -> corner[1]
is the tag's +x edge; its image-space angle is the tag's orientation.
The reported joint angle is (joint_angle - reference_angle), wrapped to
[-180, 180].

This 2D method needs no intrinsic calibration. If the joint later needs to
tilt out of the image plane, calibrate the camera and switch to a solvePnP-
based 3D pose estimate.

Overlay legend
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


def tag_image_angle_deg(corners) -> float:
    """Image-space angle of the tag's bottom edge (corner[0] -> corner[1]).

    Image y grows downward, so we negate dy to get a math-convention angle
    where counter-clockwise is positive.
    """
    x0, y0 = corners[0]
    x1, y1 = corners[1]
    return math.degrees(math.atan2(-(y1 - y0), x1 - x0))


def wrap_180(deg: float) -> float:
    return ((deg + 180.0) % 360.0) - 180.0


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
            a_ref = tag_image_angle_deg(ref_det.corners)
            a_joint = tag_image_angle_deg(joint_det.corners)
            delta = wrap_180(a_joint - a_ref)
            status = f"joint: {delta:+7.2f} deg"

            now = time.time()
            if now - last_print > 0.1:
                print(f"\r{status}     ", end="", flush=True)
                last_print = now
        else:
            missing = []
            if ref_det is None:
                missing.append(f"ref({ref_id})")
            if joint_det is None:
                missing.append(f"joint({joint_id})")
            status = "missing: " + ", ".join(missing)

        cv2.putText(frame, status, (12, 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        fps_frames += 1
        if fps_frames >= 30:
            now = time.time()
            fps = fps_frames / (now - fps_t0)
            fps_frames = 0
            fps_t0 = now
        if show_fps:
            cv2.putText(frame, f"{fps:5.1f} fps", (12, 66),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)

        cv2.imshow("AprilTag joint angle", frame)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break

    cap.release()
    cv2.destroyAllWindows()
    print()


if __name__ == "__main__":
    main()
