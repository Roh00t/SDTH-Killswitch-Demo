"""Actuator safety tests. No serial port, no hardware."""
from __future__ import annotations

import pytest

from helper.hardware.actuator import ActuatorError, MockActuator
from helper.hardware.protocol import (
    PAN_MAX_DEG, PAN_MIN_DEG, TILT_MAX_DEG, TILT_MIN_DEG,
    checksum, clamp_pan, clamp_tilt, frame,
)


@pytest.fixture
def actuator() -> MockActuator:
    a = MockActuator()
    a.connect()
    a.clear_estop()
    return a


class TestProtocolFraming:
    def test_checksum_matches_firmware_xor(self):
        assert checksum("L1") == "7D"   # 0x4C ^ 0x31
        assert checksum("M1") == "7C"   # 0x4D ^ 0x31

    def test_hazard_increasing_commands_are_checksummed(self):
        assert frame("L1", require_checksum=True) == b"L1*7D\n"
        assert frame("M1", require_checksum=True) == b"M1*7C\n"

    def test_hazard_decreasing_commands_are_not(self):
        """A corrupted byte must never be able to block a shutdown."""
        assert frame("L0") == b"L0\n"
        assert frame("M0") == b"M0\n"
        assert frame("Z") == b"Z\n"


class TestBoundsClamping:
    @pytest.mark.parametrize(
        "raw,expected",
        [(-20.0, PAN_MIN_DEG), (0.0, PAN_MIN_DEG), (90.0, 90.0), (180.0, PAN_MAX_DEG), (190.0, PAN_MAX_DEG)],
    )
    def test_pan_clamped_never_wrapped(self, raw, expected):
        assert clamp_pan(raw) == expected

    @pytest.mark.parametrize(
        "raw,expected",
        [(0.0, TILT_MIN_DEG), (45.0, TILT_MIN_DEG), (90.0, 90.0), (135.0, TILT_MAX_DEG), (200.0, TILT_MAX_DEG)],
    )
    def test_tilt_clamped_to_gimbal_geometry(self, raw, expected):
        assert clamp_tilt(raw) == expected

    def test_out_of_range_command_is_clamped_not_rejected(self, actuator):
        actuator.set_angles(999.0, -999.0)
        assert (actuator.pan, actuator.tilt) == (PAN_MAX_DEG, TILT_MIN_DEG)


class TestArmingInterlock:
    def test_fire_while_disarmed_is_rejected(self, actuator):
        with pytest.raises(ActuatorError, match="not armed"):
            actuator.set_effector(True)
        assert actuator.effector_on is False

    def test_two_key_sequence_required_to_fire(self, actuator):
        actuator.arm()
        actuator.set_effector(True)
        assert actuator.effector_on is True

    def test_disarm_also_de_energises(self, actuator):
        actuator.arm()
        actuator.set_effector(True)
        actuator.disarm()
        assert actuator.effector_on is False
        assert actuator.armed is False

    def test_de_energise_always_accepted_even_disarmed(self, actuator):
        actuator.set_effector(False)  # must not raise
        assert actuator.effector_on is False


class TestEmergencyStop:
    def test_estop_kills_and_latches(self, actuator):
        actuator.arm()
        actuator.set_effector(True)
        actuator.emergency_stop()
        assert actuator.effector_on is False
        assert actuator.armed is False
        assert actuator.estop_latched is True

    def test_latched_estop_blocks_rearm(self, actuator):
        actuator.emergency_stop()
        with pytest.raises(ActuatorError, match="E08"):
            actuator.arm()

    def test_latched_estop_blocks_fire(self, actuator):
        actuator.arm()
        actuator.emergency_stop()
        with pytest.raises(ActuatorError, match="E08"):
            actuator.set_effector(True)

    def test_estop_never_raises(self):
        """E-stop must work on every path, including an unconnected actuator."""
        MockActuator().emergency_stop()


class TestSafeStateInvariants:
    def test_construction_is_de_energised(self):
        a = MockActuator()
        assert a.effector_on is False
        assert a.armed is False

    def test_connect_forces_safe_state(self):
        a = MockActuator()
        a.connect()
        assert a.effector_on is False
        assert a.estop_latched is True

    def test_close_de_energises(self, actuator):
        actuator.arm()
        actuator.set_effector(True)
        actuator.close()
        assert actuator.effector_on is False
        assert actuator.confirm_effector_off() is True

    def test_confirm_effector_off_reports_truthfully(self, actuator):
        actuator.arm()
        actuator.set_effector(True)
        assert actuator.confirm_effector_off() is False
        actuator.set_effector(False)
        assert actuator.confirm_effector_off() is True


class TestAuditTrail:
    def test_commands_are_recorded_in_order(self, actuator):
        actuator.set_angles(120.0, 70.0)
        actuator.arm()
        actuator.set_effector(True)
        verbs = [entry for _, entry in actuator.history]
        assert "A120.0,70.0" in verbs
        assert verbs.index("M1") < verbs.index("L1")

    def test_rejections_are_recorded(self, actuator):
        actuator.set_effector(False)
        with pytest.raises(ActuatorError):
            actuator.set_effector(True)
        assert "fire while disarmed" in actuator.rejections
