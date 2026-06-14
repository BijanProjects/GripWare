# Roadmap: Voice-Controlled Pick-and-Place on the 6-DOF Arm

## Context

You have a working 6-DOF servo arm controlled from a browser dashboard over USB serial. The target demo is: **speak "grab the red cube" → arm picks the red cube from among several colored cubes and holds it up.** Industry's mainstream answer is Vision-Language-Action models trained via imitation learning — but you don't have two identical arms or the data pipeline to train one. The pragmatic alternative chosen here is to **use a cloud VLM (Claude/GPT-4V) as a *target selector* on top of a classical CV + scripted-motion stack**, not as a motion planner. The LLM decides *what* to grab; deterministic Python decides *how* to move.

### Industry best-practice note (you asked)

For a positioned joint, industry uses **position servos or steppers/BLDCs with encoders**. The base on your arm is an MG996R **continuous-rotation servo** — it accepts velocity only, has no position feedback, and its "stop" point drifts with supply voltage. Pure visual-servoing on a CR base is a research curiosity, not standard practice. The recommended industry path is: position-controlled base + camera for *fine* alignment only. We will keep visual-servoing as the priority branch (your request), and treat the hardware swap as the cleaner long-term answer, runnable in parallel.

### Current state (reconnaissance summary)

- Firmware: [Robot_Kol_Arduino_Kod.ino](Robot_Kol_Arduino_Kod/Robot_Kol_Arduino_Kod.ino) — accepts `"root armA armB wristA wristB gripper\n"` (six ints 0–180) over USB serial at **115200 baud**, on-MCU slew limiter ~60°/s. **Root is now a position-controlled servo** (hardware swap from the original continuous-rotation unit) and is slewed like the other position joints.
- Dashboard: [inference.html](inference.html) — vanilla JS + Web Serial + Three.js, direct browser-to-Arduino, no backend.
- Shoulder is **dual-servo mirrored** (D4 + D5, `armA` writes both) — couples two physical servos to one command. Note for later, don't touch now.
- Python control bridge (`arm.py`, `poses.py`) now exists. Vision/ML below is still greenfield.

---

## Progress & revisions (as of 2026-06-14)

This section is the live status overlay on the layered plan below. Mini-project bodies are kept intact for reference; read this first.

**Done / changed:**
- **M01 ✅** `m01_hello_serial.py` — per-joint sweep, arm moves, no serial errors.
- **M02 ✅** 115200 baud across firmware, dashboard, and Python.
- **M03 ⏭️ skipped (justified).** The heartbeat watchdog existed to stop a *runaway continuous-rotation base* if Python crashed. The base is now a **position servo** that holds its last commanded angle when packets stop, so that failure mode no longer exists. (If a watchdog is ever wanted for a different reason, re-open it — but it is not load-bearing anymore.)
- **M04 ✅** `arm.py` exposes `move_to`/`set_joints`/`home`/`gripper`/`gripper_open`/`gripper_close`/`wait_for_settle`; `poses.py` defines named poses.
- **Base hardware swap DONE.** This collapses Layer D — see the revised Layer D note below. The "Optional parallel branch — Position-servo base" at the bottom is now **completed**, not optional.

**New technique adopted — AprilTags (tag36h11):**
- `AprilTag/joint_angle.py` reads the 6-DOF pose of a *joint* tag relative to a *reference* tag and reports Rx/Ry/Rz + center distance. Deps added to `requirements.txt` (`pupil-apriltags`, `opencv-python`, `python-dotenv`).
- **Decision:** AprilTag replaces ArUco for the gripper-tip marker (M09) — more robust decoding, lower false-positive rate.
- **Decision:** multi-joint tags are adopted as a **calibration / ground-truth instrument**, NOT as a live per-joint pose loop on the top-down camera. Rationale: a planar tag measures in-plane rotation well and out-of-plane tilt poorly; the shoulder/elbow/wrist-pitch joints rotate in the *vertical* plane, which is the *out-of-plane* (noisy) direction for a top-down camera, and per-link tags reintroduce the occlusion problem the plan deliberately avoided. The right use is: tags on the upper-arm + forearm links viewed by the **side camera** (edge-on to the vertical plane → joint rotation becomes the well-measured in-plane Rz) to measure actual-vs-commanded joint angles. This feeds M10/M11 directly and quantifies the shoulder dual-servo coupling and droop.
- **Prerequisite:** `joint_angle.py` currently synthesizes camera intrinsics from frame size (~60° FOV guess), so its mm/degree readings are non-metric. Real ChArUco calibration (**M07**) must land before any AprilTag measurement is trusted quantitatively.

---

## Critical decisions baked into this plan

1. **Bump serial baud from 9600 to 115200.** 9600 caps the control loop at ~35 Hz, which is too tight for visual servoing. One-line firmware change + matching Python.
2. **Single top-down homography only.** Two-camera stereo triangulation is the wrong tool at this scope — the workspace is a flat table, Z is constant at the grab plane. Side camera is used only for (a) gripper-height verification, (b) extra LLM context.
3. **LLM never plans motion.** It returns a structured JSON target (or asks for clarification). Motion is deterministic Python.
4. **Heartbeat watchdog before any closed-loop work.** A runaway CR base with no encoder is the scariest failure mode in this whole project. Mandatory early.
5. **Browser dashboard is NOT migrated.** Keep it untouched as a debug fallback. Python opens a separate serial session.
6. **No full 6-DOF inverse kinematics.** We decompose the pick into constrained sub-motions so the math collapses to a 1-D base rotation + a closed-form 2-link planar IK. (See "Technical approach" below.)
7. **Every mini-project below has a single binary acceptance test.** If a test fails, you don't move forward. This is the defense against the "everything is broken and nobody knows why" failure mode.

---

## Technical approach (the hard part, explained)

You correctly identified the three problems that have to be solved end-to-end: **(1) know where the gripper is, (2) know where the cube is, (3) plan a motion from one to the other.** Here is how each is handled, with the simplifying assumptions that make them tractable for a hobby setup.

### (1) Robot pose estimation — "where is the gripper?"

We use a **hybrid: proprioception (joint angles) for all position-controlled joints + one vision marker on the gripper tip + AprilTags as an offline calibration instrument.**

- **All six joints (base included) are now position servos.** Commanded angle ≈ actual angle within a few degrees (slew + slop). Forward kinematics (FK) from the link lengths already encoded in [inference.html:1101-1260](inference.html#L1101-L1260) gives the gripper position — including the base angle, which we now command directly (the CR-base unknown is gone).
- **Gripper-tip marker**: print a small **AprilTag (tag36h11, ~2 cm)** and stick it on the gripper tip (AprilTag chosen over ArUco for more robust decoding / fewer false positives). The top-down camera + a calibrated **pixel ↔ table-plane homography** gives the gripper tip's world (X, Y) at ~30 Hz.
- **Result**: gripper pose = FK + visually-measured gripper-tip XY in the world frame. We do **not** triangulate from two cameras. We do **not** read a tag on every link in the live loop (occlusion + the out-of-plane accuracy problem below). One tip marker is enough for grasping.
- **AprilTags as a calibration instrument (not a live per-joint loop).** A planar tag measures *in-plane* rotation accurately and *out-of-plane* tilt poorly. Shoulder/elbow/wrist-pitch rotate in the **vertical** plane — which is the out-of-plane (noisy) direction for a *top-down* camera. So per-joint tags are read with the **side camera** (edge-on to the vertical plane → the joint rotation becomes the well-measured in-plane angle, `Rz` in `AprilTag/joint_angle.py`), used **offline** to measure actual-vs-commanded joint angles. This produces the per-joint offset table for M11 and quantifies the shoulder dual-servo coupling and droop. Requires real intrinsics from M07 first — `joint_angle.py`'s synthetic intrinsics are not metric.
- **What if FK is wrong?** Servos have backlash, droop, and load-dependent error. We absorb most of it two ways: (a) AprilTag calibration (above) folds per-joint offsets into FK before runtime; (b) the gripper-tip marker still gives the *measured* tip XY, so DESCEND/GRIP can be checked against ground truth. If FK says radius 12 cm but the tip marker says 10.5 cm, we trust the marker.

### (2) Cube localization — "where is the cube?"

We exploit a critical simplification: **the cube sits on the table, so its Z is known.** We don't need 3-D — we need 2-D world coordinates on the table plane.

- One **top-down USB camera** mounted directly above the workspace, fixed.
- ChArUco board calibration once → camera intrinsics. Then a single calibration step using a marker placed at the arm's base origin gives the **pixel ↔ table-plane (X, Y) homography**. From then on, any pixel `(u, v)` on the table maps to a unique world point `(X, Y, Z_table)`.
- Detect cubes by **HSV color thresholding** → centroid in pixels → world (X, Y) via the homography.
- Accuracy: 720p webcam mounted 60–80 cm above a 30×30 cm workspace → ~±2–3 mm positional accuracy. Plenty for a 4 cm cube and a parallel-jaw gripper.
- The side camera is **only** used for: (a) verifying the cube is upright (not stacked or tipped), (b) extra LLM context for ambiguity resolution. No stereo math.

### (3) Motion planning — "how do we get there?"

This is where most hobby projects collapse. The trick is to **avoid full 6-DOF inverse kinematics**. We decompose every pick into a sequence of constrained sub-motions, each of which is either a closed-form math problem or a 1-D control loop:

```
1. HOVER    Arm at READY pose: gripper at fixed height H_hover above the table,
            at fixed default radius R_default from the base axis. Base wherever.
2. ALIGN    Keep H and R constant. Rotate BASE ONLY (closed loop on vision) until
            measured gripper XY lies on the line from the base origin to the cube XY.
3. REACH    Keep base angle and H constant. Adjust SHOULDER + ELBOW so the gripper
            radius matches cube radius R = sqrt(cube_X^2 + cube_Y^2).
            This is closed-form 2-link planar IK (law of cosines, three lines of math).
4. DESCEND  Keep base/radius constant. Lower the arm by a known fixed delta in Z
            (re-solve 2-link IK at lower Z). Open loop.
5. GRIP     Close the gripper.
6. LIFT     Reverse of DESCEND.
```

The full motion is now **1-D feedback control on the base + parametric 2-link IK on the radius + a fixed Z trajectory.** No 6-DOF solver, no collision planner, no singularity handling.

**Why this works for the demo**: cubes are isolated on a flat table within a roughly annular reachable zone around the arm. No obstacles. No stacked cubes. No tight clearances. Everything else (full 6-DOF IK, RRT/PRM planners, MoveIt-style stacks) would itself be three months of work and is wildly overkill.

**Why this is safe**: every sub-motion has a "fail closed" exit. If ALIGN doesn't converge in 5 s → abort, return to HOME. If REACH would put the gripper outside the reachable workspace → reject the target before any motion starts. If DESCEND hits unexpected resistance (servo current spike — a future enhancement) → retract.

---

## Mini-projects (the actual work, sliced for verifiability)

Each mini-project below is small enough to do in **half a day to two days**. Each has exactly one **deliverable** and one **binary acceptance test**. If a test fails, stop and debug — do not move to the next one. The mini-projects are grouped into five layers; each layer is a fail-safe checkpoint where the system is observably working before more complexity is added.

> **Notation**: "M-NN" is the mini-project ID. Use these as the task IDs in any progress tracker.

---

### Layer A — Reliable programmatic control (no vision yet)

Goal: prove you can drive the arm from Python the same way the browser does, safely. If this layer fails, nothing downstream matters.

- **M01 — Hello Python serial (½ day)**
  *Deliverable*: a 30-line Python script that opens `COM*`, sends one packet `"90 110 95 90 90 80\n"`, the arm moves.
  *Test*: Run script → gripper visibly moves to the commanded pose.

- **M02 — Baud bump to 115200 (½ day)**
  *Deliverable*: firmware [Robot_Kol_Arduino_Kod.ino:120](Robot_Kol_Arduino_Kod/Robot_Kol_Arduino_Kod.ino#L120) changed; [inference.html](inference.html) updated for parity; Python uses 115200.
  *Test*: Both Python and the existing browser dashboard control the arm at the new baud. No regressions.

- **M03 — Heartbeat watchdog (½ day, mandatory)**
  *Deliverable*: Python sends a keepalive every 100 ms; firmware auto-commands `root=90` and holds position joints if no packet arrives for 500 ms.
  *Test*: Start the base rotating, then yank the USB cable — base stops within 500 ms.

- **M04 — Joint API + named poses (1 day)**
  *Deliverable*: `arm.py` exposes `set_joints()`, `home()`, `gripper(open|close)`. `poses.py` defines `HOME`, `READY`, `HOVER` (gripper directly above default reach point at hover height).
  *Test*: `arm.move_to(HOVER)` reaches the same physical pose from any starting position, 10/10 times.

- **M05 — FastAPI control server (1 day, optional polish)**
  *Deliverable*: `server.py` exposes the arm over WebSocket so notebooks and CLIs can share the connection.
  *Test*: Two Python clients can take turns commanding the arm without restart.

**Layer A checkpoint**: you can move the arm reliably from Python, it will not run away if Python crashes, and you have HOME/READY/HOVER poses.

---

### Layer B — Vision foundation (camera knows where things are)

Goal: prove the camera can give you world coordinates of anything on the table to within ±3 mm. Independent of the arm — pure vision plumbing.

- **M06 — Top-down camera capture (½ day)**
  *Deliverable*: `vision.py` opens the top-down USB cam, displays a live 30 Hz feed in a window.
  *Test*: A live window shows the workspace, no dropped frames over 60 s.

- **M07 — Camera intrinsics via ChArUco (1 day)**
  *Deliverable*: Print a ChArUco board, capture ~20 views, run OpenCV's calibration, save `K` and `dist` to a JSON. Add `undistort(frame)` helper.
  *Test*: Side-by-side raw vs. undistorted feed — straight edges (table edges) become straight after undistort.

- **M08 — Table-plane homography (1 day)**
  *Deliverable*: Lay 4 known-coordinate fiducials on the table (or one ArUco at the arm base + 3 others at measured offsets). Compute `H` such that `pixel_to_world(u,v) -> (X,Y)` on the table plane. Save `H` to JSON. Add `world_to_pixel(X,Y)` for overlay rendering.
  *Test*: Place a ruler on the table. Click 5 points in the live feed. Predicted world XYs match ruler within ±3 mm.

- **M09 — Gripper-tip ArUco marker (½ day)**
  *Deliverable*: Print a 2 cm ArUco marker, attach to the gripper jaw. Add `detect_gripper_tip(frame) -> world_xy` using M07 + M08.
  *Test*: Jog the arm by hand sliders; the live overlay shows a dot at the gripper marker's world XY, tracking continuously without dropouts.

**Layer B checkpoint**: from any pixel in the top-down feed you can recover a world (X, Y), and you can measure the gripper's world XY at 30 Hz.

---

### Layer C — Open-loop motion math (the arm "knows itself")

Goal: prove FK + the 2-link planar IK work to within tolerance, on the position joints. Still no closed loop.

- **M10 — Forward kinematics (1 day)**
  *Deliverable*: `kinematics.py` with `fk(shoulder, elbow, wrist_a) -> (radius, height)` in the arm's vertical plane. Use the link lengths in [inference.html:1101-1260](inference.html#L1101-L1260). Ignore base; that's separate.
  *Test*: Command 5 different (shoulder, elbow, wrist_a) poses. For each, FK predicts (R, Z). Measure R using M09 (compute radius from gripper world XY) — error < ±15 mm. Log discrepancies; these will be calibrated out in M11.

- **M11 — Servo angle calibration (1 day)**
  *Deliverable*: Identify the offset between commanded and actual angles per joint (servos rarely zero at the same place as the model assumes). Update FK with per-joint offsets.
  *Test*: Re-run M10's test, now error < ±5 mm.

- **M12 — 2-link planar IK (1 day)**
  *Deliverable*: `ik_2link(radius, height) -> (shoulder, elbow, wrist_a)`. Closed-form law of cosines. The wrist pitch is chosen to keep the gripper pointing down at the table.
  *Test*: Sweep `R` from 8 cm → 18 cm at fixed `Z = hover_height`. Command each. Measure with M09. Achieved radius error < ±5 mm across the range.

- **M13 — Reachable workspace mask (1.5 days)**
  *Deliverable*: For each (R, base_angle) grid point, jog the arm there and record reachability + collision-free status. Store as a polygon in world coords. Render as an overlay in the live feed.
  *Test*: Visualization clearly shows the reachable annulus. Hand-pick 10 points (5 in, 5 out) — every "in" point can be reached without collision; every "out" point is correctly rejected.

**Layer C checkpoint**: given a world (X, Y) inside the mask, the arm can position the gripper above it within ±5 mm — *if the base angle is correct*. The base is the only remaining unknown.

---

### Layer D — Base alignment

> **REVISED (2026-06-14): the base is now a position servo.** M14–M16 were written to tame a *continuous-rotation* base with no feedback. With the hardware swap done, ALIGN collapses to a one-line **command the absolute base angle** = `atan2(cube_Y, cube_X)` minus the servo offset (from M11). M14 (dead-band cal) and M15 (velocity characterization) are **obsolete**; M16 (closed-loop P-controller) is replaced by an open-loop absolute command, optionally fine-tuned by one vision correction step using the gripper-tip marker. The original M14–M16 text below is retained only as a record of the abandoned CR-base approach.

Goal (original, CR-base): rotate the base so the gripper tip ends up over an arbitrary world XY target, using vision feedback.

- **M14 — Base dead-band calibration (½ day)**
  *Deliverable*: At session start, sweep base PWM 85 → 95 in 1-unit steps, hold each for 300 ms, measure gripper XY motion via M09. Record `BASE_STOP_MIN`, `BASE_STOP_MAX` (the range of PWM where the base is stationary).
  *Test*: Re-run with different battery levels — dead-band changes, script adapts. Stop point is always within ±2 PWM units of true zero.

- **M15 — Base velocity characterization (1 day)**
  *Deliverable*: Pulse base CW/CCW at PWM offsets {±5, ±10, ±20, ±40} from the dead-band center for 200 ms each. Record degrees-of-base-rotation-per-pulse from the gripper marker arc. Build an empirical PWM ↔ angular-velocity curve.
  *Test*: Print the curve. Should be roughly monotonic. Both directions characterized.

- **M16 — Closed-loop base align (2 days)**
  *Deliverable*: `align_base_to(target_world_xy)` runs a P-controller using M09 gripper position and M15 velocity model. NO integral term (wind-up will oscillate). Hard 5 s timeout → abort. Explicit stop command on exit.
  *Test*: From 10 random starting base angles, command `align_base_to(target)` for 10 random reachable targets. Convergence within ±10 px and stop within 5 s, 9/10 trials.

**Layer D checkpoint**: the arm can now point itself at any world XY on the table. Combined with Layer C, you can position the gripper directly above any reachable point.

---

### Layer E — Object detection, picking, language, voice

Goal: glue the previous layers into the actual demo.

- **M17 — HSV cube detection (1 day)**
  *Deliverable*: `detect.py` with `find_cubes(frame) -> [{id, color, world_xy, pixel_xy, bbox}]` for {red, green, blue, yellow}. HSV ranges calibrated under your lighting.
  *Test*: Scatter 4 cubes. All detected, correct colors, world XY within ±5 mm of hand-measured ground truth. Light a desk lamp on the workspace — still works.

- **M18 — Scripted pick at given XY (1.5 days)**
  *Deliverable*: `pick(world_xy)` runs HOVER → ALIGN (M16) → REACH (M12) → DESCEND → GRIP → LIFT, with a fail-closed abort path.
  *Test*: Place a cube at 10 hand-marked positions inside the reachable mask. `pick(world_xy)` succeeds 9/10 times. Failed trials must terminate cleanly at HOME, not in an unsafe pose.

- **M19 — End-to-end pick by hardcoded color (½ day)**
  *Deliverable*: `pick_demo.py` calls M17, filters for `"red"`, calls `pick(red_cube.world_xy)`.
  *Test*: 4 cubes in random positions → script picks the red one. 8/10 trials.

- **M20 — LLM target selector (1.5 days)**
  *Deliverable*: `brain.py` sends `(top_image, side_image, detected_cubes_list, task_text)` to Claude with strict-JSON output:
  ```json
  { "action": "pick" | "clarify",
    "target_id": "<id from detected list>" | null,
    "clarify_question": "..." | null,
    "reasoning": "..." }
  ```
  *Test*: With hardcoded task strings:
  - `"grab the red cube"` → returns `action=pick, target_id=<red>`.
  - `"grab the cube"` with two reds visible → returns `action=clarify`.
  - `"grab the purple cube"` with no purple → returns `action=clarify` (not a hallucinated pick).

- **M21 — Voice input (½ day)**
  *Deliverable*: Mic capture + Whisper API → text → M20.
  *Test*: Speak each of the three M20 phrases → correct transcription and correct M20 result.

- **M22 — Full demo + polish (½ day)**
  *Deliverable*: Wire M21 → M20 → M19 with TTS or printed feedback for `clarify` responses.
  *Test*: 4 random-color cubes, voice command. Picks correctly 8/10 runs. Ambiguous prompts get a spoken clarification, not a wrong action.

---

## ✅ COMPLETED branch — Position-servo base

**Done 2026-05-24.** This was the recommended path and it eliminated Layer D's risk entirely. The base is now position-controlled and slewed in firmware; M14–M16 are superseded (see the revised Layer D note above).

- **H1 — Procure & install position MG996R + 4:1 gear reduction (1 week incl. ordering)**
  Mechanical retrofit on the base. CR can spin freely; position is 0–180°, so a gear reduction is needed to keep enough effective range while staying inside servo limits.
- **H2 — Firmware: root as position (½ day)**
  Move `root` into the slewed-position array in [Robot_Kol_Arduino_Kod.ino:165-182](Robot_Kol_Arduino_Kod/Robot_Kol_Arduino_Kod.ino#L165-L182). Update bounds.
- **H3 — Re-run M08 + M13 (½ day)**
  Recalibrate homography and reach mask in the new base-angle frame.
- **Effect**: M14–M16 collapse to a one-line "command absolute base angle"; the visual servo loop is no longer required. The vision-servo branch can still be built later as a robustness layer.

---

## Critical path & timeline summary

Layer-by-layer, sequential. Each layer is a hard gate.

```
Layer A   Layer B    Layer C       Layer D            Layer E
M01-M05   M06-M09    M10-M13       M14-M16            M17-M22
 ~3 d      ~3 d       ~4.5 d         ~3.5 d            ~5 d
            calendar overhead, debugging, hardware downtime: 2-3x
```

- **Best case (everything works first time)**: ~19 working days ≈ 4 calendar weeks.
- **Realistic estimate** (with debugging, lighting issues, servo retuning): **~8–10 calendar weeks** end-to-end.
- **With hardware swap (H1–H3 instead of M14–M16)**: subtract ~2 calendar weeks of Layer D risk; add ~1 week of hardware procurement and integration → **~7–9 weeks** total.

**Key principle: do not skip a layer's checkpoint.** If Layer B's M08 homography is off by 1 cm, M16 will never converge — and you will spend a week debugging the wrong thing. Each layer's checkpoint is the cheapest place to catch a problem.

---

## Critical files

**Modify (existing)**
- [Robot_Kol_Arduino_Kod/Robot_Kol_Arduino_Kod.ino](Robot_Kol_Arduino_Kod/Robot_Kol_Arduino_Kod.ino) — baud → 115200 (M02); heartbeat timeout (M03); root → position-controlled (only if doing H1–H3)
- [inference.html](inference.html) — match new baud (M02). Do **not** migrate to talk to Python. Keep as debug fallback.

**Create (new, all under project root)**
- `arm.py` — serial bridge with the existing 6-int packet format (M01, M04)
- `server.py` — FastAPI/WebSocket interface (M05, optional)
- `poses.py` — named poses HOME / READY / HOVER (M04)
- `kinematics.py` — FK + 2-link planar IK + servo offset calibration (M10–M12)
- `vision.py` — capture, intrinsics calibration, table-plane homography, reach mask (M06–M09, M13)
- `detect.py` — `find_cubes()` + `detect_gripper_tip()` (M09, M17)
- `controllers.py` — `align_base_to()`, `pick(world_xy)` motion sequence (M16, M18)
- `pick_demo.py` — hardcoded-color end-to-end demo (M19)
- `brain.py` — voice → Whisper → LLM → target selection (M20, M21)
- `requirements.txt` — `pyserial`, `opencv-python`, `opencv-contrib-python` (for ArUco/ChArUco), `numpy`, `fastapi`, `uvicorn`, `anthropic`, `openai-whisper` (or `anthropic`'s audio API)
- `calibration/` — folder for saved intrinsics JSON, homography JSON, servo-offset JSON, reach-mask PNG, dead-band JSON. Treat these as data, regenerated by named scripts.

---

## Verification strategy

The plan is structured so that **each mini-project's acceptance test is the cheapest possible check that the previous layer is still healthy**. There is no "big bang" integration step — every mini-project verifies a single piece of the stack against ground truth.

**Per-mini-project**: each M-NN has a single binary test (see above). Pass = move on. Fail = stop and debug. Do not "fix it later".

**Per-layer checkpoints** (the explicit gates that catch upstream errors before they amplify):

- **End of Layer A (after M04)**: Reliable, safe, scriptable arm control. If this gate fails, none of the vision work matters.
- **End of Layer B (after M09)**: World coords from pixels are accurate to ±3 mm and the gripper-tip XY is observed live. If this gate fails, every subsequent layer is built on sand.
- **End of Layer C (after M13)**: The arm can position itself to a known XY (assuming the base angle is correct), and you know which XYs are reachable. If this gate fails, the closed loop in Layer D will appear to "not converge" but the bug is upstream.
- **End of Layer D (after M16)**: The arm can autonomously align over any reachable XY. From here, the LLM and voice are just plumbing.
- **End of Layer E (after M22)**: Demo complete.

**End-to-end demo acceptance** (after M22):
1. Lay 4 cubes (red, green, blue, yellow) at random positions inside the reachable mask.
2. Speak "grab the red cube." Expect: arm picks the red cube, lifts it above the table within 15 s. Repeat 10×, expect ≥ 8 successes.
3. Speak "grab the cube" (ambiguous). Expect: spoken or printed clarification question — never a wrong pick.
4. Place a cube outside the reachable mask. Speak "grab the red cube." Expect: a graceful "out of reach" error, no collision attempt.

---

## Risks I'm watching, mapped to mitigations

- **CR base dead-band drift with battery voltage** — calibrated at session start by M14, not hard-coded.
- **Gripper self-occlusion** during descent — closed-loop on XY only (M16); descent (M18) is open-loop in Z.
- **Reachable workspace ≠ camera FOV** — M13 produces an explicit reach mask; M20 is told about it so the LLM cannot return unreachable targets.
- **FK error from servo backlash / link-length mismatch** — calibrated out in M11 before it propagates.
- **Lighting changes break HSV** — M17 acceptance test explicitly varies lighting; if it fails there, consider a small YOLO model instead.
- **Shoulder dual-servo coupling** (D4+D5 mirrored) — flagged for any future posture-optimization work, out of scope for this demo.
- **LLM latency** (~2–4 s per call with vision) — acceptable for a one-shot voice command; do not call inside the servo loop.
- **Forgetting the watchdog** — M03 is mandatory and must pass before any work in Layer D begins.

---

## What this plan deliberately does NOT include

- True 3-D stereo / depth sensing
- General 6-DOF inverse kinematics
- Imitation learning / training a VLA
- Trajectory smoothing / collision planners (MoveIt-class)
- Replacing the gripper, shoulder, elbow, or wrist hardware
- Browser dashboard migration
- Anything beyond rigid, single-layer, colored cubes on a flat table

---

