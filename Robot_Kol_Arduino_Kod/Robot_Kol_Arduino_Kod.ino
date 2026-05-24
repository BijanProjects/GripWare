// Emre Kalem | Eskisehir, Turkiye | 2025
// Robust firmware update for GripWare — December 2026
//
// What changed vs the original sketch and why:
//
// 1) Line-buffered serial parsing.
//    The original used Serial.parseInt() guarded only by
//    `Serial.available() >= 6`. That check counts BYTES, not ints —
//    so on partial packets parseInt() would block on its 1 s timeout
//    and return 0, slamming a servo to 0 degrees mid-motion.
//    That is the "drop under load" symptom: a transient bad parse
//    commands a position the arm can't hold, and gravity wins.
//    Now we accumulate characters until '\n', then parse the whole
//    line in one shot and only apply it if all 6 ints parsed cleanly.
//
// 2) Per-servo slew limiter inside the Arduino.
//    Even with clean packets, if the host commands a large step the
//    servo lunges and can droop. We slew each position-controlled
//    servo toward its target at SLEW_DEG_PER_TICK every TICK_MS,
//    independent of how fast the host sends. The host can send a
//    far-away target — the arm will glide to it smoothly.
//
// 3) Per-joint range clamping.
//    Every commanded angle is clamped to per-joint mechanical limits
//    (see JOINT_MIN / JOINT_MAX) before slew so garbled data can never
//    drive a servo past its safe range. Gripper is [45, 110]; the rest
//    are full [0, 180].
//
// 4) Root is now a position-controlled servo (hardware swap from the
//    earlier continuous-rotation unit). Slewed like the other position
//    servos so large host steps don't lunge the base.
//
// Wire format (unchanged, backward-compatible with both dashboards):
//   "root armA elbow wristA wristB gripper\n"  e.g. "90 110 95 90 90 80\n"
//   - 6 ints, space separated, terminated with '\n'.
//   - all six = absolute angle in degrees (0..180), home = 90

#include <Servo.h>

Servo servo1; // Pin 3  — Root (position, 0..180)
Servo servo2; // Pin 4  — Arm A1
Servo servo3; // Pin 5  — Arm A2 (mirrored from A1)
Servo servo4; // Pin 6  — Arm B (elbow)
Servo servo5; // Pin 9  — Wrist A
Servo servo6; // Pin 10 — Wrist B
Servo servo7; // Pin 11 — Gripper

const uint8_t SERVO_PINS[] = { 3, 4, 5, 6, 9, 10, 11 };

// Index 0=root, 1=armA, 2=elbow, 3=wristA, 4=wristB, 5=gripper
int   target[6]  = { 90, 90, 90, 90, 90, 77 }; // latest commanded values from host
float current[6] = { 90, 90, 90, 90, 90, 77 }; // smoothed values actually written

// Per-joint mechanical limits. Anything outside is clamped before slew.
// Gripper has a hard mechanical range of [45, 110]; everything else is full 0..180.
const int JOINT_MIN[6] = {  0,  0,  0,  0,  0,  45 };
const int JOINT_MAX[6] = { 180, 180, 180, 180, 180, 110 };

// Slew tuning. The position servos can typically do ~300 deg/s no-load,
// but under load that drops sharply. 60 deg/s leaves a comfortable margin
// so the servo never falls behind the commanded position (which is what
// causes droop).
const float         SLEW_DEG_PER_TICK = 0.6f;   // -> 60 deg/s at 10 ms tick
const unsigned long TICK_MS           = 10;     // servo update period

// Serial line buffer.
char     lineBuf[80];
uint8_t  lineLen = 0;

unsigned long lastTick = 0;

// ---------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------

static inline int clampJoint(int v, uint8_t i) {
  if (v < JOINT_MIN[i]) return JOINT_MIN[i];
  if (v > JOINT_MAX[i]) return JOINT_MAX[i];
  return v;
}

// Hand-rolled int parser: faster and more deterministic than sscanf on AVR.
// Returns true if exactly 6 ints were parsed from `s`.
bool parseSixInts(char* s, int out[6]) {
  uint8_t n = 0;
  char* p = s;
  while (*p && n < 6) {
    while (*p == ' ' || *p == '\t' || *p == ',') p++;
    if (!*p) break;

    int sign = 1;
    if (*p == '-') { sign = -1; p++; }
    else if (*p == '+') { p++; }

    if (*p < '0' || *p > '9') return false; // garbage token
    int v = 0;
    while (*p >= '0' && *p <= '9') {
      v = v * 10 + (*p - '0');
      p++;
      if (v > 10000) return false; // overflow guard
    }
    out[n++] = sign * v;
  }
  return (n == 6);
}

void writeServos() {
  // Root: absolute angle.
  servo1.write((int)current[0]);

  // Shoulder pair: write A1 and its mirror to A2.
  int armA = (int)current[1];
  servo2.write(armA);
  servo3.write(180 - armA);

  servo4.write((int)current[2]); // elbow
  servo5.write((int)current[3]); // wrist A
  servo6.write((int)current[4]); // wrist B
  servo7.write((int)current[5]); // gripper
}

// ---------------------------------------------------------------------
// Arduino setup / loop
// ---------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < 7; i++) pinMode(SERVO_PINS[i], OUTPUT);

  servo1.attach(SERVO_PINS[0]);
  servo2.attach(SERVO_PINS[1]);
  servo3.attach(SERVO_PINS[2]);
  servo4.attach(SERVO_PINS[3]);
  servo5.attach(SERVO_PINS[4]);
  servo6.attach(SERVO_PINS[5]);
  servo7.attach(SERVO_PINS[6]);

  digitalWrite(13, LOW);

  // Apply initial pose so servos hold from boot.
  writeServos();
}

void loop() {
  // ---- 1. Drain serial into the line buffer ----
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        int parsed[6];
        if (parseSixInts(lineBuf, parsed)) {
          // All six ints valid — clamp and accept as new target.
          for (uint8_t i = 0; i < 6; i++) {
            target[i] = clampJoint(parsed[i], i);
          }
        }
        // If parse failed, we silently drop the packet — current target
        // is retained, so the arm holds its last known good position.
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      // Overflow — drop the line, wait for the next newline to resync.
      lineLen = 0;
    }
  }

  // ---- 2. Slew each servo toward its target every TICK_MS ----
  unsigned long now = millis();
  if (now - lastTick >= TICK_MS) {
    lastTick = now;

    // All six joints are position-controlled — rate-limited approach to target.
    for (uint8_t i = 0; i < 6; i++) {
      float diff = (float)target[i] - current[i];
      if (diff >  SLEW_DEG_PER_TICK)      current[i] += SLEW_DEG_PER_TICK;
      else if (diff < -SLEW_DEG_PER_TICK) current[i] -= SLEW_DEG_PER_TICK;
      else                                current[i]  = (float)target[i];
    }

    writeServos();
  }
}
