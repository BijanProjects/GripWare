# GripWare

A browser-based control system for a 3D-printed 6-DOF robotic arm with real-time 3D visualization.

![License](https://img.shields.io/badge/license-Apache%202.0-blue)

## The Robot Arm

GripWare controls a 6-degree-of-freedom (6-DOF) articulated robotic arm. The arm is 3D-printed and driven by a combination of high-torque MG996R and compact MG90S servo motors, connected to an Arduino over serial.

### Degrees of Freedom

The arm has six independently controlled joints, chained from base to gripper:

```
[Base] → [Shoulder] → [Elbow] → [Wrist Pitch] → [Wrist Roll] → [Gripper]
```

1. **Base (Root)** — Rotates the entire arm horizontally. Uses a continuous rotation MG996R servo, meaning it spins freely in either direction at a controllable speed rather than moving to a fixed angle. This allows unlimited rotation for positioning the arm around its workspace.

2. **Shoulder (Arm A)** — Lifts the lower arm up and down. This is the highest-torque joint, using **two mirrored MG996R servos** (pins D4 and D5) working in opposition to double the lifting force. When one servo writes angle X, the other writes 180-X, so they push together. This joint carries the weight of the entire arm above it.

3. **Elbow (Arm B)** — Bends the upper arm relative to the lower arm, like a human elbow. Single MG996R servo (pin D6). Together with the shoulder, these two joints control the arm's reach and height.

4. **Wrist Pitch (Wrist A)** — Tilts the end-effector up and down. MG90S servo (pin D9). This allows the gripper to angle toward objects on a table or reach upward.

5. **Wrist Roll (Wrist B)** — Rotates the end-effector around the arm's axis. MG90S servo (pin D10). Useful for reorienting the gripper to approach objects from different angles.

6. **Gripper** — A parallel-jaw gripper with two flat plates that slide together and apart. MG90S servo (pin D11). The plates move symmetrically to grip objects between their inner faces.

### Joint Types

The arm uses two types of servo control:

| Type | Behavior | Servos | Used For |
|------|----------|--------|----------|
| **Continuous Rotation (CR)** | Spins at variable speed; 90 = stop, 0 = full CW, 180 = full CCW | MG996R | Base rotation |
| **Position** | Moves to a precise angle between 0-180 degrees | MG996R / MG90S | All other joints |

All position servos default to 90 degrees (center) on startup.

### Physical Construction

The arm is 3D-printed from the [Robotic Arm with Servo & Arduino](https://www.thingiverse.com/) design, consisting of:

- **Alt_Govde / Alt_Kasa / Alt_Kapak** — Base housing (cylindrical pedestal)
- **Alt_Kol** — Lower arm link (shoulder to elbow)
- **On_Kol** — Upper arm link (elbow to wrist)
- **Bilek** — Wrist joint housing
- **El / El_Ust** — Gripper body and top plate
- **Parmak / Parmak_2** — Gripper finger halves
- **Disli / Mil / Servo_Disli** — Gears, shafts, and servo adapters

### How It Moves

When a command is sent (e.g., `90 120 60 90 90 45`), the Arduino writes each value to its corresponding servo:

1. The **base** servo receives 90 → stops rotating (center = no movement for CR servos)
2. The **shoulder** servos receive 120 → both pivot the lower arm forward by 30 degrees from center. The second shoulder servo receives 180-120=60 to mirror the motion.
3. The **elbow** servo receives 60 → bends the upper arm backward by 30 degrees from center
4. **Wrist pitch** receives 90 → stays level
5. **Wrist roll** receives 90 → stays aligned
6. **Gripper** receives 45 → closes partially

The arm moves all joints simultaneously, reaching the target pose in the time it takes the slowest servo to arrive.

### Workspace and Limits

- **Base rotation**: Unlimited (continuous rotation)
- **All position joints**: 0-180 degrees mechanical range
- **Payload**: Limited by the MG996R torque (~10 kg-cm) at the shoulder
- **Reach**: Determined by the combined length of the lower and upper arm links

## Wiring

```
Arduino Uno / Mega
    ├── D3  → Base servo (MG996R, continuous rotation)
    ├── D4  → Shoulder servo A1 (MG996R)
    ├── D5  → Shoulder servo A2 (MG996R, mirrored)
    ├── D6  → Elbow servo (MG996R)
    ├── D9  → Wrist pitch servo (MG90S)
    ├── D10 → Wrist roll servo (MG90S)
    ├── D11 → Gripper servo (MG90S)
    └── USB → Host computer (power + serial data)

External 5V PSU
    ├── VCC → All servo power rails
    └── GND → All servo GND + Arduino GND (shared ground)
```

**Important**: Never power 7 servos from the Arduino 5V pin. Use an external 5V supply rated for at least 5A. Share ground between the supply and Arduino.

## Serial Protocol

Communication runs at **9600 baud**. The host sends 6 space-separated integers (0-180) followed by a newline:

```
root armA armB wristA wristB gripper\n
```

The Arduino responds with:
- `OK <values>` — command accepted and applied
- `ERR` — malformed input (rejected)
- `READY` — sent once on boot

The dashboard uses an ACK handshake: one packet in flight at a time, with a 400ms timeout fallback to prevent stalling.

## Control Interface

Open `inference.html` in **Chrome** or **Edge** (requires Web Serial API). The interface provides:

- **Servo sliders and jog buttons** for all 6 joints
- **Real-time 3D preview** (Three.js) that mirrors the arm's pose
- **Direct 3D manipulation** — double-click to focus on a joint, drag to control it
- **Emergency stop** — halts all servos instantly
- **Serial log** — see every TX/RX packet for debugging

The 3D viewer stays visible in a sticky panel while scrolling through controls.

## Getting Started

1. **Print the arm** — Slice and print the STL files
2. **Assemble** — Mount servos, connect gears and shafts
3. **Wire** — Connect servos to Arduino pins and external power
4. **Flash** — Upload `robotic_arm_code/robotic_arm_code.ino` via Arduino IDE
5. **Open** — Load `inference.html` in Chrome/Edge
6. **Connect** — Click "Connect Arduino", select the port, and start controlling

## Project Structure

```
GripWare/
  inference.html    # Dashboard + 3D viewer (single HTML file, no build step)
  README.md
  LICENSE           # Apache 2.0
```

## License

Apache 2.0 — see [LICENSE](LICENSE) for details.
