"""ESP32 link bring-up — flash the firmware, then run this.

Verifies the safety interlocks on real hardware before any of them matters.

Usage:
    python -m tools.serial_probe --list
    python -m tools.serial_probe --servo-sweep         # port auto-detected
    python -m tools.serial_probe --port COM3           # Windows
    python -m tools.serial_probe --port /dev/cu.usbserial-130   # macOS
    python -m tools.serial_probe --interactive
"""
from __future__ import annotations

import argparse
import logging
import sys
import time

from helper.hardware.actuator import ActuatorError, SerialActuator, resolve_port
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

        print("\nGimbal (COMMANDED angle only — SG90s have no position feedback,")
        print("       so this passes even with no servos attached. Use")
        print("       'serial_probe --servo-sweep' to confirm physical motion.)")
        for pan, tilt in ((60.0, 70.0), (120.0, 110.0), (90.0, 90.0)):
            actuator.set_angles(pan, tilt)
            time.sleep(0.8)
        s = actuator.last_status()
        check("firmware echoed the stow command", s is not None and abs(s.pan - 90.0) < 6.0)

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

        print("\nDeadman (heartbeat suppressed for 0.6s — LED should die on its own)")
        actuator.arm()
        actuator.set_effector(True)
        time.sleep(0.15)
        lit = actuator.last_status()
        check("effector lit before suspending heartbeat",
              lit is not None and lit.laser_on)
        # Suspend ONLY the heartbeat. Stopping _running would also stop the
        # reader, so the firmware would cut the effector and we would never see
        # the status frame proving it.
        actuator.suspend_heartbeat()
        time.sleep(0.6)
        s = actuator.last_status()
        check("deadman cut the effector", s is not None and not s.laser_on,
              f"(status {s})")
        check("deadman also disarmed", s is not None and not s.armed)
        actuator.resume_heartbeat()
        time.sleep(0.2)

        print(f"\n{passed} passed, {failed} failed")
        if failed == 0:
            print("Link is good. Record the port in config and move on.")
        return 0 if failed == 0 else 1

    except ActuatorError as exc:
        print(f"\nACTUATOR ERROR: {exc}")
        return 1
    finally:
        actuator.close()


def servo_sweep(port: str) -> int:
    """Large, slow, obvious movements. YOU are the sensor here.

    SG90s report nothing, so the only way to confirm the gimbal physically
    moves is to watch it. This isolates each axis so a wiring or power fault
    points at one servo rather than the whole rig.
    """
    actuator = SerialActuator(port, baud=BAUD_RATE)
    try:
        actuator.connect()
        print("\nWatch the gimbal. Each move is deliberately large and slow.\n")

        def move(label: str, pan: float, tilt: float, hold: float = 1.6) -> None:
            print(f"  {label:<38}", end="", flush=True)
            actuator.set_angles(pan, tilt)
            time.sleep(hold)
            s = actuator.last_status()
            print(f"commanded pan={s.pan:5.1f} tilt={s.tilt:5.1f}" if s else "no status")

        print("PAN axis (GPIO 5) — should swing left/right")
        move("centre", 90.0, 90.0)
        move("pan hard left  (0 deg)", 5.0, 90.0, 2.5)
        move("pan hard right (180 deg)", 175.0, 90.0, 3.0)
        move("pan centre", 90.0, 90.0, 2.5)

        print("\nTILT axis (GPIO 6) — should tip up/down")
        move("tilt down (45 deg)", 90.0, 48.0, 2.5)
        move("tilt up   (135 deg)", 90.0, 132.0, 3.0)
        move("tilt centre", 90.0, 90.0, 2.5)

        print("\nBOTH axes together")
        move("diagonal A", 40.0, 60.0, 2.5)
        move("diagonal B", 140.0, 120.0, 3.0)
        move("stow", 90.0, 90.0, 2.5)

        print("""
--- What you just saw tells you where the fault is ---

BOTH axes moved smoothly
    Gimbal is good. Nothing to fix.

NOTHING moved, no sound at all
    No power reaching the servos. This is the usual cause.
    - Servo V+ (red) must go to a SEPARATE 5V supply, not the ESP32 3V3 pin.
    - That supply's GND must be tied to an ESP32 GND pin. Without a common
      ground the PWM signal has no reference and the servo ignores it.
    - Check the supply is switched on and the barrel/USB connector is seated.

NOTHING moved but you hear buzzing or feel the horn straining
    Power is present but sagging. Two SG90s stall-draw ~700mA each.
    - A phone charger rated under 2A will brown out under load.
    - Check for a loose ground or thin jumper wire on the V+ run.

ONE axis moved, the other did not
    That axis alone is at fault: signal wire, the servo itself, or the pin.
    - Pan signal (orange/yellow) -> GPIO 5, Tilt -> GPIO 6.
    - Swap the two signal wires. If the fault follows the wire it is wiring;
      if it stays on the same axis it is that servo or that GPIO.

Movement is jerky or it jumps to an end stop and sticks
    Likely a pulse-width mismatch. Adjust SERVO_MIN_US / SERVO_MAX_US in
    firmware/esp32_actuator/esp32_actuator.ino (currently 500-2400us) and
    reflash.
""")
        return 0
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
        print("\nCommands:")
        print("  a <pan> <tilt>  absolute angles      d   firmware PWM diagnostics")
        print("  arm / disarm    arming interlock     t   RAW sweep (bypasses slew")
        print("  on / off        effector                 limiting and deadband)")
        print("  z               e-stop               s   status      q  quit\n")
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
                elif verb == "d":
                    actuator.send_raw("D")
                    time.sleep(0.4)   # let the reader surface the DIAG line
                elif verb == "t":
                    print("  raw sweep — watch the gimbal for ~2s")
                    actuator.send_raw("T")
                    time.sleep(2.5)
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
    parser.add_argument("--servo-sweep", action="store_true",
                        help="Large slow movements to confirm the gimbal physically moves")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")

    if args.list:
        return list_ports()

    # Auto-resolve when --port is omitted. Previously any command without
    # --port silently fell through to list_ports(), so `--servo-sweep` alone
    # printed a port table and moved no servos — looking exactly like a dead
    # gimbal. Reuse the same auto-detection the node uses.
    port = args.port
    if not port:
        try:
            port = resolve_port("auto")
        except ActuatorError as exc:
            print(f"\n{exc}\n")
            return list_ports()

    if args.servo_sweep:
        return servo_sweep(port)
    return interactive(port) if args.interactive else run_checks(port)


if __name__ == "__main__":
    sys.exit(main())
