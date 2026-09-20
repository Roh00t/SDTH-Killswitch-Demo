"""Targeting math tests. No camera, no model, no hardware, no numpy."""
from __future__ import annotations

import pytest

from helper.vision.aimpoint import (
    AimpointSolver,
    error_magnitude,
    pixel_error,
    select_priority_target,
)
from helper.vision.types import Detection


def make_det(x=320.0, y=240.0, w=100.0, h=80.0, track_id=1) -> Detection:
    return Detection(
        x=x, y=y, w=w, h=h, confidence=0.9,
        class_id=0, class_name="drone", track_id=track_id,
    )


class TestDetectionGeometry:
    def test_xyxy_converts_centre_to_corner_format(self):
        det = make_det(x=100.0, y=100.0, w=40.0, h=20.0)
        assert det.xyxy == (80.0, 90.0, 120.0, 110.0)

    def test_min_dimension_picks_the_smaller_side(self):
        assert make_det(w=100.0, h=30.0).min_dimension == 30.0
        assert make_det(w=20.0, h=90.0).min_dimension == 20.0


class TestAimpointOffset:
    def test_zero_offset_is_centre_of_mass(self):
        aim = AimpointSolver().solve(make_det(x=320.0, y=240.0))
        assert (aim.x, aim.y) == (320.0, 240.0)
        assert aim.downgraded is False

    def test_rotor_hub_offset_biases_up_and_left(self):
        # -0.35 of a 200x160 box: -70px horizontally, -56px vertically.
        solver = AimpointSolver(offset_x=-0.35, offset_y=-0.35)
        aim = solver.solve(make_det(x=320.0, y=240.0, w=200.0, h=160.0))
        assert aim.x == pytest.approx(250.0)
        assert aim.y == pytest.approx(184.0)
        assert aim.downgraded is False

    def test_payload_offset_biases_down_only(self):
        solver = AimpointSolver(offset_x=0.0, offset_y=0.4)
        aim = solver.solve(make_det(x=320.0, y=240.0, w=200.0, h=160.0))
        assert aim.x == pytest.approx(320.0)
        assert aim.y == pytest.approx(304.0)

    def test_offset_scales_with_apparent_size(self):
        """Same physical aimpoint at different ranges -> offset scales in pixels."""
        solver = AimpointSolver(offset_x=-0.35, offset_y=0.0)
        near = solver.solve(make_det(x=320.0, y=240.0, w=200.0, h=200.0))
        far = solver.solve(make_det(x=320.0, y=240.0, w=100.0, h=100.0))
        assert (320.0 - near.x) == pytest.approx(70.0)
        assert (320.0 - far.x) == pytest.approx(35.0)


class TestOffsetClamping:
    def test_offset_beyond_half_is_clamped(self):
        solver = AimpointSolver(offset_x=0.9, offset_y=-1.5)
        assert solver.configured_offset == (0.5, -0.5)

    def test_clamped_aimpoint_stays_on_the_box_edge(self):
        solver = AimpointSolver(offset_x=5.0, offset_y=0.0)
        det = make_det(x=320.0, y=240.0, w=100.0, h=100.0)
        aim = solver.solve(det)
        x1, _, x2, _ = det.xyxy
        assert x1 <= aim.x <= x2
        assert aim.x == pytest.approx(370.0)


class TestResolutionGate:
    def test_small_box_downgrades_to_centre_of_mass(self):
        solver = AimpointSolver(offset_x=-0.35, offset_y=-0.35, min_box_px=40.0)
        aim = solver.solve(make_det(x=320.0, y=240.0, w=10.0, h=10.0))
        assert (aim.x, aim.y) == (320.0, 240.0)
        assert aim.downgraded is True
        assert aim.offset_applied == (0.0, 0.0)
        assert "resolution gate" in aim.reason

    def test_large_box_applies_offset(self):
        solver = AimpointSolver(offset_x=-0.35, offset_y=-0.35, min_box_px=40.0)
        aim = solver.solve(make_det(x=320.0, y=240.0, w=100.0, h=100.0))
        assert aim.downgraded is False

    def test_gate_uses_min_dimension_not_area(self):
        """A wide, flat box is still unresolved in its short axis."""
        solver = AimpointSolver(offset_x=-0.35, offset_y=-0.35, min_box_px=40.0)
        aim = solver.solve(make_det(w=400.0, h=12.0))
        assert aim.downgraded is True

    def test_boundary_is_inclusive_at_threshold(self):
        solver = AimpointSolver(offset_x=0.25, min_box_px=40.0)
        assert solver.solve(make_det(w=40.0, h=40.0)).downgraded is False
        assert solver.solve(make_det(w=39.9, h=39.9)).downgraded is True

    def test_rejects_non_positive_threshold(self):
        with pytest.raises(ValueError):
            AimpointSolver(min_box_px=0.0)


class TestTargetSelection:
    def test_returns_none_for_empty_not_a_sentinel(self):
        """Legacy get_closest_coords returned (9999, 9999) here."""
        assert select_priority_target([], (320.0, 240.0)) is None

    def test_picks_nearest_to_frame_centre(self):
        near = make_det(x=330.0, y=250.0, track_id=1)
        far = make_det(x=600.0, y=50.0, track_id=2)
        assert select_priority_target([far, near], (320.0, 240.0)).track_id == 1

    def test_selects_beyond_the_legacy_999_distance_ceiling(self):
        """Legacy code hardcoded a 999^2 ceiling; a 4K frame exceeds it."""
        distant = make_det(x=3800.0, y=2000.0, track_id=7)
        assert select_priority_target([distant], (1920.0, 1080.0)).track_id == 7


class TestPointingError:
    def test_signed_error_directions(self):
        aim = AimpointSolver().solve(make_det(x=400.0, y=300.0))
        ex, ey = pixel_error(aim, (320.0, 240.0))
        assert (ex, ey) == (80.0, 60.0)

    def test_magnitude_is_euclidean(self):
        aim = AimpointSolver().solve(make_det(x=323.0, y=244.0))
        assert error_magnitude(aim, (320.0, 240.0)) == pytest.approx(5.0)

    def test_centred_target_has_zero_error(self):
        aim = AimpointSolver().solve(make_det(x=320.0, y=240.0))
        assert error_magnitude(aim, (320.0, 240.0)) == pytest.approx(0.0)


class TestAuthorisationBinding:
    def test_aimpoint_carries_track_id_for_auth_binding(self):
        """Auth binds to track_id; a solution for 7 must not fire on 9."""
        aim = AimpointSolver().solve(make_det(track_id=7))
        assert aim.track_id == 7

    def test_untracked_detection_yields_none_track_id(self):
        aim = AimpointSolver().solve(make_det(track_id=None))
        assert aim.track_id is None


class TestDegenerateBoxRejection:
    """Real model output contained a 44x0 box clipped at the frame edge."""

    def test_zero_height_box_is_gated_to_centre_of_mass(self):
        solver = AimpointSolver(offset_x=-0.35, offset_y=-0.35, min_box_px=40.0)
        aim = solver.solve(make_det(x=1258.0, y=0.0, w=44.0, h=0.0))
        assert aim.downgraded is True
        assert (aim.x, aim.y) == (1258.0, 0.0)

    def test_zero_area_box_has_zero_min_dimension(self):
        assert make_det(w=44.0, h=0.0).min_dimension == 0.0
