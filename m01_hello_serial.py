"""M01 - Hello Python serial.

Sends a minimal per-joint sweep around the home pose to verify Python
serial control and that every servo responds to its index in the packet.

Home pose: (90, 90, 90, 90, 90, 77)
  - Five non-gripper joints at the middle of [0, 180].
  - Gripper at the middle of its mechanical range [45, 110] -> 77.

Sweep: for each joint in turn, command home, home+10, home, home-10, home.
A 60 deg/s firmware slew + a 0.6 s host pause leaves plenty of margin.

Usage:
    python m01_hello_serial.py             # list ports
    python m01_hello_serial.py --port COM10
"""

import argparse
import sys
import time

import serial
from serial.tools import list_ports

BAUD = 9600  # bumped to 115200 in M02
HOME = (90, 90, 90, 90, 90, 77)
JOINT_NAMES = ("root/base", "shoulder", "elbow", "wrist_a", "wrist_b", "gripper")
SWEEP_OFFSETS = (+10, 0, -10, 0)  # small symmetric excursion, return to home
SETTLE_S = 0.6                    # > time for firmware slew across 10 deg
INITIAL_BOOT_S = 2.0              # Arduino auto-resets on serial open


def list_available_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports detected.")
        return
    print("Available serial ports:")
    for p in ports:
        print(f"  {p.device:10s}  {p.description}")
    print("\nRe-run with --port COM<N>")


def send(ser: serial.Serial, pose: tuple) -> None:
    line = " ".join(str(int(v)) for v in pose) + "\n"
    ser.write(line.encode("ascii"))
    ser.flush()


def sweep(port: str) -> None:
    print(f"Opening {port} at {BAUD} baud...")
    with serial.Serial(port, BAUD, timeout=1) as ser:
        time.sleep(INITIAL_BOOT_S)

        print(f"Home: {HOME}")
        send(ser, HOME)
        time.sleep(1.0)

        for i, name in enumerate(JOINT_NAMES):
            print(f"  Sweeping {name} (index {i})...")
            for off in SWEEP_OFFSETS:
                pose = list(HOME)
                pose[i] = HOME[i] + off
                send(ser, pose)
                time.sleep(SETTLE_S)

        print("Returning to home.")
        send(ser, HOME)
        time.sleep(1.0)
    print("Done. M01 passes if every joint visibly moved twice and returned to home.")


def main() -> int:
    parser = argparse.ArgumentParser(description="M01 sweep test.")
    parser.add_argument("--port", help="Serial port, e.g. COM10")
    args = parser.parse_args()

    if not args.port:
        list_available_ports()
        return 0

    try:
        sweep(args.port)
    except serial.SerialException as exc:
        print(f"Serial error: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
