# Team Tracker — Robotic Arm AI Roadmap

**Team size**: 3
**Cadence**: twice weekly — **Tuesday + Friday**
**Kickoff**: Tue 2026-05-26
**Demo target**: Fri 2026-07-31 (~10 calendar weeks)
**Plan reference**: see the full plan with technical reasoning at `C:\Users\bijan\.claude\plans\okay-so-we-have-jolly-teacup.md`

**How to read the table**:
- **Description** combines the *deliverable* (what gets built) and the *acceptance test* (how you know it works). A task is "done" only when its acceptance test passes — never on partial implementation.
- **Due** is the meeting at which the task should be *demoed and signed off*, not just claimed done.
- Layer checkpoints (in **bold rows**) are hard gates. If a checkpoint fails, the team stops and debugs before any later task starts.

---

## Mini-project schedule

| Task | Description (deliverable + acceptance test) | Due |
|---|---|---|
| **M01 — Hello Python serial** | Python script opens the Arduino COM port at 9600 baud and sends one home packet (`"90 90 90 90 90 90\n"`). **Accept** when the arm visibly moves to or holds the home pose with no errors. | **Tue 5/26** |
| **M02 — Baud bump to 115200** | Update firmware (`Serial.begin(115200)`), browser dashboard, and Python to 115200 baud. **Accept** when both Python and the existing browser dashboard control the arm at the new baud with no regressions. | Tue 5/26 |
| **M03 — Heartbeat watchdog** | Python sends a keepalive packet every 100 ms; firmware auto-commands `root=90` and holds position servos if no packet arrives for 500 ms. **Accept** when yanking the USB cable mid-rotation stops the base within 500 ms. **Mandatory before any closed-loop work.** | **Fri 5/29** |
| M04 — Joint API + named poses | `arm.py` exposes `set_joints()`, `home()`, `gripper(open/close)`. `poses.py` defines HOME, READY, HOVER. **Accept** when `arm.move_to(HOVER)` reaches the same pose from any starting state, 10/10 times. | Tue 6/2 |
| M05 — FastAPI control server (optional polish) | `server.py` exposes the arm over WebSocket so notebooks and CLIs share one connection. **Accept** when two Python clients can take turns commanding the arm without restart. | Fri 6/5 |
| **🏁 Layer A checkpoint — reliable, safe scriptable arm** | Arm is driven from Python, watchdog confirmed working, named poses reproducible. If this gate fails, all vision work is blocked. | **Fri 6/5** |
| M06 — Top-down camera capture | OpenCV opens the top-down USB cam and shows a live 30 Hz feed. **Accept** when no dropped frames over 60 s. | Tue 6/9 |
| M07 — Camera intrinsics via ChArUco | Print a ChArUco board, capture ~20 views, run OpenCV calibration, save `K` and `dist` to JSON, add `undistort(frame)`. **Accept** when undistorted feed shows straight table edges that were curved before. | Fri 6/12 |
| M08 — Table-plane homography | Lay 4 fiducials at measured world coordinates; compute the pixel↔world homography; expose `pixel_to_world(u,v)` and `world_to_pixel(X,Y)`. **Accept** when 5 clicked points on the live feed predict world XY within ±3 mm of a ruler. | Tue 6/16 |
| M09 — Gripper-tip ArUco marker | Print a ~2 cm ArUco, attach to gripper, add `detect_gripper_tip(frame) -> world_xy`. **Accept** when the live overlay tracks the gripper marker continuously while you jog the arm. | Fri 6/19 |
| **🏁 Layer B checkpoint — vision knows world coordinates** | Pixel ↔ world mapping is ±3 mm; gripper-tip XY is measurable at 30 Hz. If this gate fails, every later layer is built on sand. | **Fri 6/19** |
| M10 — Forward kinematics | `kinematics.py` with `fk(shoulder, elbow, wrist_a) -> (radius, height)` from the link lengths in `inference.html` (lines 1101–1260). **Accept** when 5 commanded poses predict radius within ±15 mm of M09's measurement. | Tue 6/23 |
| M11 — Servo angle offset calibration | Identify per-joint commanded-vs-actual offsets; update FK to apply them. **Accept** when M10's test now passes within ±5 mm. | Fri 6/26 |
| M12 — 2-link planar IK | `ik_2link(radius, height) -> (shoulder, elbow, wrist_a)` via closed-form law of cosines; wrist pitch keeps the gripper pointing down. **Accept** when sweeping R = 8 → 18 cm at fixed Z hits the radius within ±5 mm across the whole range. | Tue 6/30 |
| M13 — Reachable workspace mask | Jog the arm to a grid of world XY points; record reachability + collision-free status; save as a polygon and render as a live overlay. **Accept** when 5-in / 5-out hand-picked points are correctly classified. | Fri 7/3 |
| **🏁 Layer C checkpoint — open-loop math works** | Arm reaches any commanded XY within ±5 mm assuming the base angle is correct. If this gate fails, Layer D's closed loop will look broken but the bug is here. | **Fri 7/3** |
| M14 — CR base dead-band calibration | At session start, sweep base PWM 85 → 95, hold each value 300 ms, measure gripper XY motion to find the "stop" PWM range. **Accept** when re-running with a different battery level produces a different dead-band that the script adapts to. | Tue 7/7 |
| M15 — Base velocity characterization | Pulse base CW/CCW at PWM offsets {±5, ±10, ±20, ±40} for 200 ms each; record degrees-of-rotation per pulse. **Accept** when the resulting PWM ↔ angular-velocity curve is roughly monotonic in both directions. | Tue 7/7 |
| M16 — Closed-loop base alignment | `align_base_to(target_world_xy)` runs a P-controller using M09 + M15. NO integral term. Hard 5 s timeout + explicit stop on exit. **Accept** when 9/10 trials from random starting base angles converge within ±10 px and 5 s. | Fri 7/10 |
| **🏁 Layer D checkpoint — arm autonomously points at any XY** | Combined with Layer C, the arm can place the gripper directly above any reachable world point. The remainder is plumbing. | **Fri 7/10** |
| M17 — HSV cube detection | `find_cubes(frame) -> [{id, color, world_xy, pixel_xy, bbox}]` for red/green/blue/yellow under your lighting. **Accept** when 4 scattered cubes are all detected, correct colors, world XY within ±5 mm — and the test still passes with a desk lamp on. | Tue 7/14 |
| M18 — Scripted pick(world_xy) | `pick(world_xy)` runs HOVER → ALIGN → REACH → DESCEND → GRIP → LIFT with a fail-closed abort path. **Accept** when 10 hand-marked positions inside the reach mask yield 9/10 successful picks; failed trials terminate cleanly at HOME. | Fri 7/17 |
| M19 — Pick by hardcoded color | `pick_demo.py` calls M17, filters for `"red"`, calls `pick()`. **Accept** when 4 cubes in random positions yield the red one being picked 8/10 trials. | Tue 7/21 |
| M20 — LLM target selector | `brain.py` sends `(top_image, side_image, detected_cubes, task_text)` to Claude with strict-JSON output (`action`, `target_id`, `clarify_question`). **Accept** when (a) "grab the red cube" → pick, (b) "grab the cube" with two reds → clarify, (c) "grab the purple cube" with no purple → clarify, never a hallucinated pick. | Fri 7/24 |
| M21 — Voice input via Whisper | Microphone capture + Whisper API → text → M20. **Accept** when each of M20's three test phrases is correctly transcribed and routed. | Tue 7/28 |
| **M22 — Full demo + polish** | Wire M21 → M20 → M19 with spoken or printed feedback for `clarify` responses. **Accept** when 4 random-color cubes + a voice command yield 8/10 correct picks, and ambiguous prompts get a clarification — not a wrong action. | **Fri 7/31 — DEMO DAY** |

---

## Suggested team split (3 people)

Most tasks have a primary owner but pair work is fine. The plan is sequenced so multiple layers don't run in parallel — this is intentional, to keep failures localized.

- **Person F (firmware / motion)**: M02, M03, M10, M11, M12, M13, M14, M15, M16, M18
- **Person V (vision)**: M06, M07, M08, M09, M13 (with F), M17
- **Person B (brain / glue)**: M01, M04, M05, M19, M20, M21, M22

Layer-A tasks (M01–M05) are early enough that anyone can pick them up; the split matters more from Layer B onward.

---

## Meeting agenda template (15–30 min, twice weekly)

1. **Live demo** of each task due today — pass / fail against its acceptance test. No "almost done".
2. **Failed test triage**: if a task failed, before moving on, identify whether the bug is in the current task or in an earlier-layer assumption.
3. **Next-meeting assignments**: confirm owner per upcoming task.
4. **Risk check**: is the upcoming layer checkpoint date still realistic? If two consecutive meetings slip a task, replan rather than compress later layers.

---

## Out-of-band: the optional hardware swap

If you decide to swap the base CR servo for a position servo + 4:1 gear reduction (sub-tasks H1–H3 in the plan), insert these between Layer C and Layer D and **skip M14–M16**. The track shortens by ~1.5 calendar weeks.
