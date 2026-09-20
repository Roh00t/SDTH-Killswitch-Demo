"""ESP32 link bring-up — flash the firmware, then run this.

Verifies the safety interlocks on real hardware before any of them matters.

Usage:
    python -m tools.serial_probe --list
    python -m tools.serial_probe --port /dev/cu.usbserial-0001
    python -m tools.serial_probe --port <p> --interactive
"""
from __future__ import annotations

import argparse
import logging
import sys
import time

from helper.hardware.actuator import ActuatorError, SerialActuator
from helper.hardware.protocol import BAUD_RATE


def list_ports() -> int:
    from serial.tools import list_ports as lp

    ports = list(lp.comports())
    if not ports:
        print("No serial ports found. Check the cable and the CP2102/CH340 driver.")
        return 1
    print(f"{'device':<28} {'description'}")
    print("-" * 70)
    for p in ports:
        print(f"{p.device:<28} {p.description}")
    print("\nPick the UART bridge (usbserial/wchusbserial/SLAB), NOT the native")
    print("USB-CDC port — the bridge stays enumerated across ESP32 resets.")
    return 0


def run_checks(port: str) -> int:
    """Exercise every firmware interlock and report pass/fail."""
    actuator = SerialActuator(port, baud=BAUD_RATE)
    passed, failed = 0, 0

    def check(label: str, condition: bool, detail: str = "") -> None:
        nonlocal passed, failed
        if condition:
            passed += 1
            print(f"  PASS  {label}")
        else:
            failed += 1
            print(f"  FAIL  {label} {detail}")

    try:
        print(f"\nConnecting on {port} at {BAUD_RATE}...")
        actuator.connect()
        status = actuator.last_status()
        print(f"  firmware: {status}\n")

        print("Boot state")
        check("effector de-energised at boot", status is not None and not status.laser_on)
        check("disarmed at boot", status is not None and not status.armed)

        print("\nGimbal (watch the servos move)")
        for pan, tilt in ((60.0, 70.0), (120.0, 110.0), (90.0, 90.0)):
            actuator.set_angles(pan, tilt)
            time.sleep(0.8)
        s = actuator.last_status()
        check("gimbal returned to stow", s is not None and abs(s.pan - 90.0) < 6.0)

        print("\nBounds (commands are clamped, not wrapped)")
        actuator.set_angles(999.0, -999.0)
        time.sleep(1.2)
        s = actuator.last_status()
        check("pan clamped to 180", s is not None and s.pan <= 180.5, f"got {s.pan if s else '?'}")
        check("tilt clamped to 45", s is not None and s.tilt >= 44.5, f"got {s.tilt if s else '?'}")
        actuator.set_angles(90.0, 90.0)
        time.sleep(0.8)

        print("\nArming interlock")
        actuator.set_effector(True)  # still latched from connect(): must be refused
        time.sleep(0.3)
        s = actuator.last_status()
        check("fire refused while disarmed", s is not None and not s.laser_on)

        actuator.arm()
        time.sleep(0.2)
        check("arm accepted", (actuator.last_status() or s).armed)

        print("\nEffector (LED should light ~0.5s)")
        actuator.set_effector(True)
        time.sleep(0.3)
        check("effector energised when armed", (actuator.last_status() or s).laser_on)
        actuator.set_effector(False)
        time.sleep(0.2)
        check("effector de-energised on command", actuator.confirm_effector_off())

        print("\nBurn ceiling (firmware cuts at 2.0s even if host does not)")
        actuator.arm()
        actuator.set_effector(True)
        time.sleep(2.6)
        check("firmware cut the burn at its ceiling", actuator.confirm_effector_off())

        print("\nDeadman (heartbeat suppressed for 0.6s)")
        actuator.arm()
        actuator.set_effector(True)
        time.sleep(0.15)
        actuator._running.clear()          # stop the heartbeat, simulating a dead host
        time.sleep(0.6)
        s = actuator.last_status()
        check("deadman cut the effector", s is not None and not s.laser_on)

        print(f"\n{passed} passed, {failed} failed")
        if failed == 0:
            print("Link is good. Record the port in config and move on.")
        return 0 if failed == 0 else 1

    except ActuatorError as exc:
        print(f"\nACTUATOR ERROR: {exc}")
        return 1
    finally:
        actuator.close()


def interactive(port: str) -> int:
    """Manual command shell. Type raw protocol bodies; 'q' exits."""
    actuator = SerialActuator(port, baud=BAUD_RATE)
    try:
        actuator.connect()
        print("\nCommands: a <pan> <tilt> | arm | disarm | on | off | z | s | q\n")
        while True:
            try:
                parts = input("> ").strip().split()
            except (EOFError, KeyboardInterrupt):
                break
            if not parts:
                continue
            verb = parts[0].lower()
            try:
                if verb == "q":
                    break
                elif verb == "a" and len(parts) == 3:
                    actuator.set_angles(float(parts[1]), float(parts[2]))
                elif verb == "arm":
                    actuator.arm()
                elif verb == "disarm":
                    actuator.disarm()
                elif verb == "on":
                    actuator.set_effector(True)
                elif verb == "off":
                    actuator.set_effector(False)
                elif verb == "z":
                    actuator.emergency_stop()
                elif verb == "s":
                    print(f"  {actuator.last_status()}")
                else:
                    print("  ?")
            except (ActuatorError, ValueError) as exc:
                print(f"  error: {exc}")
        return 0
    finally:
        actuator.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Serial device path")
    parser.add_argument("--list", action="store_true", help="List serial ports")
    parser.add_argument("--interactive", action="store_true", help="Manual shell")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")

    if args.list or not args.port:
        return list_ports()
    return interactive(args.port) if args.interactive else run_checks(args.port)


if __name__ == "__main__":
    sys.exit(main())
