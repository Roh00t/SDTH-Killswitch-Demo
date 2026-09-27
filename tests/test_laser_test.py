"""Bench laser tool: toggle, off, firmware ceiling, shutdown. No hardware."""
from __future__ import annotations

from helper.hardware.actuator import MockActuator
from tools.laser_test import LaserBench


def _bench():
    act = MockActuator()
    act.connect()                  # boots e-stop latched, like the firmware
    lines = []
    return act, LaserBench(act, say=lines.append), lines


def test_toggle_arms_fires_then_turns_off():
    act, bench, _ = _bench()
    bench.toggle()
    assert act.effector_on and bench.is_on
    bench.toggle()
    assert not act.effector_on and not bench.is_on


def test_off_is_safe_when_already_off():
    act, bench, lines = _bench()
    bench.off()
    assert not act.effector_on and lines[-1] == "  laser off"


def test_poll_notices_the_firmware_cutting_the_burn():
    act, bench, lines = _bench()
    bench.on()
    act.set_effector(False)        # what the 2 s ceiling does, host uninvolved
    bench.poll()
    assert not bench.is_on and "ceiling" in lines[-1]


def test_shutdown_leaves_it_off_and_disarmed():
    act, bench, _ = _bench()
    bench.on()
    bench.shutdown()
    assert not act.effector_on and not act.armed
