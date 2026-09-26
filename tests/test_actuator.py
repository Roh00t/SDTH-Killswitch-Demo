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

    def test_checksummed_arm_clears_the_soft_latch(self, actuator):
        """Mirrors firmware: a checksummed M1 sets estopLatched = false.

        The soft latch stops a booted-or-faulted board firing until something
        deliberately arms it. It is not the human barrier — that is the physical
        interlock, which no software path can clear.
        """
        actuator.emergency_stop()
        assert actuator.estop_latched is True
        actuator.arm()
        assert actuator.estop_latched is False and actuator.armed is True

    def test_latched_estop_blocks_fire_without_an_intervening_arm(self, actuator):
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


class TestPortResolution:
    """macOS renumbers /dev/cu.usbserial-<n> between plugs, so a hardcoded
    port is a guaranteed failure at the worst moment."""

    def test_explicit_path_is_passed_through_untouched(self):
        from helper.hardware.actuator import resolve_port

        assert resolve_port("/dev/cu.usbserial-130") == "/dev/cu.usbserial-130"
        assert resolve_port("COM3") == "COM3"

    def test_auto_picks_the_single_bridge(self, monkeypatch):
        from helper.hardware import actuator as mod

        class FakePort:
            def __init__(self, device, description):
                self.device, self.description = device, description

        fake = [
            FakePort("/dev/cu.Bluetooth-Incoming-Port", "n/a"),
            FakePort("/dev/cu.usbserial-130", "CP2102N USB to UART Bridge Controller"),
        ]
        monkeypatch.setattr(
            "serial.tools.list_ports.comports", lambda: fake, raising=False
        )
        assert mod.resolve_port("auto") == "/dev/cu.usbserial-130"

    def test_auto_raises_when_no_bridge_present(self, monkeypatch):
        from helper.hardware import actuator as mod

        class FakePort:
            def __init__(self, device, description):
                self.device, self.description = device, description

        monkeypatch.setattr(
            "serial.tools.list_ports.comports",
            lambda: [FakePort("/dev/cu.debug-console", "n/a")], raising=False,
        )
        with pytest.raises(ActuatorError, match="no USB-UART bridge"):
            mod.resolve_port("auto")

    def test_auto_refuses_to_guess_between_two_boards(self, monkeypatch):
        from helper.hardware import actuator as mod

        class FakePort:
            def __init__(self, device, description):
                self.device, self.description = device, description

        monkeypatch.setattr(
            "serial.tools.list_ports.comports",
            lambda: [
                FakePort("/dev/cu.usbserial-130", "CP2102N USB to UART Bridge"),
                FakePort("/dev/cu.wchusbserial-20", "CH340 USB to UART"),
            ], raising=False,
        )
        with pytest.raises(ActuatorError, match="ambiguous"):
            mod.resolve_port("auto")


class TestPortOpenError:
    """Windows says "Access is denied" for a port another program holds."""

    def test_a_held_port_names_the_fix(self, monkeypatch):
        import serial

        from helper.hardware.actuator import SerialActuator

        def taken(*args, **kwargs):
            raise serial.SerialException(
                "could not open port 'COM3': PermissionError(13, 'Access is denied.', None, 5)")

        monkeypatch.setattr(serial, "Serial", taken)
        with pytest.raises(ActuatorError) as raised:
            SerialActuator("COM3").connect()
        message = str(raised.value)
        assert "Access is denied" in message                  # the original text survives
        assert "Another program has COM3 open" in message
        assert "Arduino Serial Monitor" in message

    def test_a_missing_port_gets_no_misleading_hint(self):
        from helper.hardware.actuator import port_open_error

        message = port_open_error("COM9", FileNotFoundError("could not open port 'COM9'"))
        assert message == "Could not open COM9: could not open port 'COM9'"

    def test_interactive_prints_one_line_instead_of_a_traceback(self, monkeypatch, capsys):
        import tools.serial_probe as probe
        from helper.hardware.actuator import SerialActuator

        def taken(self):
            raise ActuatorError("Could not open COM3: Access is denied. Another program has COM3 open")

        monkeypatch.setattr(SerialActuator, "connect", taken)
        assert probe.interactive("COM3") == 1
        assert "ACTUATOR ERROR: Could not open COM3" in capsys.readouterr().out


class TestPanReversed:
    """The rig's pan servo turns anticlockwise; the driver mirrors it both ways."""

    class _Port:
        def __init__(self, actuator, lines=()):
            self.actuator, self.lines, self.written = actuator, list(lines), []

        def write(self, payload):
            self.written.append(payload)

        def readline(self):
            if self.lines:
                return self.lines.pop(0)
            self.actuator._running.clear()
            return b""

    def _actuator(self, reversed_, lines=()):
        from helper.hardware.actuator import SerialActuator

        act = SerialActuator("COM3", pan_reversed=reversed_)
        act._serial = self._Port(act, lines)
        return act

    def test_mirror_keeps_stow_and_bounds(self):
        from helper.hardware.actuator import mirror_pan

        assert mirror_pan(45.0) == 135.0
        assert mirror_pan(90.0) == 90.0
        assert (mirror_pan(PAN_MIN_DEG), mirror_pan(PAN_MAX_DEG)) == (PAN_MAX_DEG, PAN_MIN_DEG)

    def test_commands_go_out_mirrored_only_when_reversed(self):
        straight, mirrored = self._actuator(False), self._actuator(True)
        straight.set_angles(45.0, 97.0)
        mirrored.set_angles(45.0, 97.0)
        assert b"A045.0,097.0" in straight._serial.written[0]
        assert b"A135.0,097.0" in mirrored._serial.written[0]

    def test_out_of_range_is_clamped_before_mirroring(self):
        act = self._actuator(True)
        act.set_angles(999.0, -999.0)
        assert b"A000.0,045.0" in act._serial.written[0]

    def test_status_comes_back_in_the_hosts_sense(self):
        act = self._actuator(True, [b"ST 135.0,97.0,0,0,1234\n"])
        act._running.set()
        act._reader_loop()
        status = act.last_status()
        assert (status.pan, status.tilt) == (45.0, 97.0)

    def test_serial_probe_reads_the_same_setting(self, tmp_path):
        from tools.serial_probe import load_pan_reversed

        on = tmp_path / "on.yaml"
        on.write_text("actuator:\n  pan_reversed: true\n")
        off = tmp_path / "off.yaml"
        off.write_text("actuator:\n  port: COM3\n")
        assert load_pan_reversed(str(on)) is True
        assert load_pan_reversed(str(off)) is False
        assert load_pan_reversed(str(tmp_path / "missing.yaml")) is False
        assert load_pan_reversed("config/fallback.yaml") is True
