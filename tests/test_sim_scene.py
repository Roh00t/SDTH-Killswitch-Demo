"""SceneDetector — the closed-loop synthetic target behind `main.py --sim-target`.

The property that matters most is the first one: a simulated target must never
be able to steer a real effector. The rest check that the scene behaves like a
camera would — the target is visible only when the gimbal faces it, the loop is
genuinely closed through the actuator's commanded angles, and a reset issues a
fresh identity the way the real tracker does.
"""
from __future__ import annotations

import numpy as np
import pytest

from helper.hardware.actuator import MockActuator, SerialActuator
from tools.simulator import (
    SIM_TARGET_PAN_DEG,
    SIM_TARGET_TILT_DEG,
    SceneDetector,
    SimulatedTarget,
)

FRAME = np.zeros((720, 1280, 3), dtype=np.uint8)


class FakeClock:
    """Manually advanced monotonic clock, so tests never sleep."""

    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


def make_scene(actuator=None, target=None, class_name="bird"):
    act = actuator or MockActuator()
    if isinstance(act, MockActuator):
        act.connect()
    clock = FakeClock()
    scene = SceneDetector(
        act, class_name=class_name, frame_width=1280, frame_height=720,
        horizontal_fov_deg=65.0, target=target, clock=clock,
    )
    return scene, act, clock


def run_for(scene, clock, seconds, step=0.05):
    """Call detect() on a fixed cadence; return the last result."""
    out = []
    for _ in range(int(round(seconds / step))):
        clock.t += step
        out = scene.detect(FRAME)
    return out


class TestSafety:
    def test_refuses_a_real_actuator(self):
        with pytest.raises(TypeError, match="never steer a real effector"):
            SceneDetector(
                SerialActuator("COM9"), class_name="bird",
                frame_width=1280, frame_height=720, horizontal_fov_deg=65.0,
            )

    def test_sim_flag_forces_the_mock_actuator(self):
        from main import KillswitchNode, load_config

        # The demo profile, with no mock_actuator requested: the sim flag alone
        # must force it, or --sim-target would drive the real ESP32.
        node = KillswitchNode(
            load_config("config/fallback.yaml"), sim_target=True, mock_c2=True
        )
        assert node._mock_actuator is True
        assert node._mock_camera is True


class TestClosedLoop:
    def test_target_out_of_frame_from_stow(self):
        # Stow faces pan 90; the default target sits at pan 45, which is outside
        # the +/-32.5 deg half-FOV. A real camera would not see it either.
        scene, _, clock = make_scene()
        assert run_for(scene, clock, 0.5) == []

    def test_target_acquired_once_the_gimbal_slews_onto_it(self):
        scene, act, clock = make_scene()
        act.set_angles(SIM_TARGET_PAN_DEG, SIM_TARGET_TILT_DEG)
        seen = run_for(scene, clock, 0.5)  # 45 deg at 250 deg/s is 0.18 s
        assert len(seen) == 1
        # Commanded straight at it, so it lands near the image centre.
        assert abs(seen[0].x - 640.0) < 60.0
        assert abs(seen[0].y - 360.0) < 60.0

    def test_commanding_the_gimbal_moves_the_target_in_frame(self):
        target = SimulatedTarget(azimuth_deg=90.0, elevation_deg=90.0)
        scene, act, clock = make_scene(target=target)
        centred = run_for(scene, clock, 0.3)[0]
        act.set_angles(95.0, 90.0)  # pan 5 deg right of the target
        shifted = run_for(scene, clock, 0.3)[0]
        # Target now 5 deg left of boresight: ~98 px at 1280 px / 65 deg.
        assert shifted.x < centred.x - 80.0

    def test_emits_the_configured_class_name(self):
        scene, act, clock = make_scene(class_name="frisbee")
        act.set_angles(SIM_TARGET_PAN_DEG, SIM_TARGET_TILT_DEG)
        assert run_for(scene, clock, 0.5)[0].class_name == "frisbee"


class TestIdentity:
    def test_reset_issues_a_fresh_track_id(self):
        scene, act, clock = make_scene()
        act.set_angles(SIM_TARGET_PAN_DEG, SIM_TARGET_TILT_DEG)
        first = run_for(scene, clock, 0.5)[0].track_id
        scene.reset()
        second = run_for(scene, clock, 0.1)[0].track_id
        assert second == first + 1
