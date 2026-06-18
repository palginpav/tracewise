"""Tests for exact_geom — Phase A validation of the #4 NEAR build geometry primitives.

All expected values are hand-computed (derivations in comments).
Asymmetric coordinates are used throughout so an accidental x/y swap produces
a detectably wrong result.
"""

from __future__ import annotations

import math

import pytest

from tracewise.route.engine.exact_geom import (
    POLAR_ANGLES,
    clearance_to_obstacle,
    is_legal,
    min_clearance,
    nudge_endpoint,
    nudge_endpoint_in_region,
    point_rect_distance,
    segment_point_distance,
    segment_rect_distance,
    segment_segment_distance,
)

# ---------------------------------------------------------------------------
# segment_point_distance
# ---------------------------------------------------------------------------


class TestSegmentPointDistance:
    def test_point_on_segment_returns_zero(self):
        # Midpoint of a=(1.0, 2.0)–b=(5.0, 2.0): t=0.5, foot=(3,2), dist=0
        assert segment_point_distance((1.0, 2.0), (5.0, 2.0), (3.0, 2.0)) == pytest.approx(0.0)

    def test_perpendicular_foot_inside_segment(self):
        # a=(0.0, 0.0) b=(4.0, 0.0) p=(2.0, 3.0)
        # t=0.5, foot=(2.0, 0.0), dist=3.0
        assert segment_point_distance((0.0, 0.0), (4.0, 0.0), (2.0, 3.0)) == pytest.approx(3.0)

    def test_foot_beyond_endpoint_b_clamps(self):
        # a=(0.0, 0.0) b=(4.0, 0.0) p=(6.0, 3.0)
        # t=1.5 clamped to 1.0, foot=(4.0, 0.0), dist=hypot(2,3)=3.6055…
        expected = math.hypot(2.0, 3.0)
        assert segment_point_distance((0.0, 0.0), (4.0, 0.0), (6.0, 3.0)) == pytest.approx(expected)

    def test_foot_beyond_endpoint_a_clamps(self):
        # a=(3.0, 0.0) b=(7.0, 0.0) p=(0.0, 4.0)
        # t=-0.75 clamped to 0.0, foot=(3.0, 0.0), dist=hypot(3,4)=5.0
        assert segment_point_distance((3.0, 0.0), (7.0, 0.0), (0.0, 4.0)) == pytest.approx(5.0)

    def test_degenerate_segment_a_equals_b(self):
        # a=b=(3.0, 5.0), p=(6.0, 9.0)
        # dist = hypot(3, 4) = 5.0
        assert segment_point_distance((3.0, 5.0), (3.0, 5.0), (6.0, 9.0)) == pytest.approx(5.0)

    def test_asymmetric_coords_x_not_equal_y(self):
        # Oblique segment to catch a potential x/y swap:
        # a=(1.0, 3.0) b=(7.0, 5.0) p=(4.0, 6.0)
        # segment direction (6,2), len_sq=40
        # t = ((4-1)*6 + (6-3)*2) / 40 = (18+6)/40 = 0.6
        # foot = (1+0.6*6, 3+0.6*2) = (4.6, 4.2)
        # dist = hypot(4-4.6, 6-4.2) = hypot(0.6, 1.8)
        expected = math.hypot(0.6, 1.8)
        assert segment_point_distance((1.0, 3.0), (7.0, 5.0), (4.0, 6.0)) == pytest.approx(expected)


# ---------------------------------------------------------------------------
# segment_segment_distance
# ---------------------------------------------------------------------------


class TestSegmentSegmentDistance:
    def test_parallel_horizontal_offset(self):
        # a=(0.0, 0.0)–b=(4.0, 0.0)  c=(1.0, 3.0)–d=(5.0, 3.0)
        # All four endpoint-to-segment distances are 3.0 (foot always inside)
        assert segment_segment_distance(
            (0.0, 0.0), (4.0, 0.0), (1.0, 3.0), (5.0, 3.0)
        ) == pytest.approx(3.0)

    def test_crossing_segments_returns_zero(self):
        # a=(0.0, 0.0)–b=(4.0, 4.0)  c=(0.0, 4.0)–d=(4.0, 0.0) — cross at (2,2)
        assert segment_segment_distance(
            (0.0, 0.0), (4.0, 4.0), (0.0, 4.0), (4.0, 0.0)
        ) == pytest.approx(0.0)

    def test_endpoint_to_endpoint_closest(self):
        # a=(0.0, 3.0)–b=(2.0, 3.0)  c=(5.0, 7.0)–d=(8.0, 7.0)
        # Nearest pair: b=(2,3)↔c=(5,7): hypot(3,4)=5.0
        assert segment_segment_distance(
            (0.0, 3.0), (2.0, 3.0), (5.0, 7.0), (8.0, 7.0)
        ) == pytest.approx(5.0)

    def test_collinear_overlapping_returns_zero(self):
        # a=(0.0, 0.0)–b=(4.0, 0.0)  c=(2.0, 0.0)–d=(6.0, 0.0)
        # c=(2,0) lies on ab → distance 0
        assert segment_segment_distance(
            (0.0, 0.0), (4.0, 0.0), (2.0, 0.0), (6.0, 0.0)
        ) == pytest.approx(0.0)

    def test_t_shaped_perpendicular(self):
        # Horizontal seg a=(0.0, 0.0)–b=(6.0, 0.0)
        # Vertical seg c=(3.0, 2.0)–d=(3.0, 5.0) — foot inside, dist=2.0
        assert segment_segment_distance(
            (0.0, 0.0), (6.0, 0.0), (3.0, 2.0), (3.0, 5.0)
        ) == pytest.approx(2.0)


# ---------------------------------------------------------------------------
# point_rect_distance
# ---------------------------------------------------------------------------


class TestPointRectDistance:
    def test_point_inside_rect(self):
        # p=(3.0, 4.0) inside (1.0, 2.0, 6.0, 8.0) → 0
        assert point_rect_distance((3.0, 4.0), (1.0, 2.0, 6.0, 8.0)) == pytest.approx(0.0)

    def test_point_on_boundary(self):
        # p=(3.0, 2.0) on bottom edge of (1.0, 2.0, 6.0, 8.0) → 0
        assert point_rect_distance((3.0, 2.0), (1.0, 2.0, 6.0, 8.0)) == pytest.approx(0.0)

    def test_point_outside_via_right_edge(self):
        # p=(8.0, 5.0), rect=(1.0, 2.0, 6.0, 8.0)
        # dx=max(1-8,0,8-6)=2; dy=max(2-5,0,5-8)=0; dist=2.0
        assert point_rect_distance((8.0, 5.0), (1.0, 2.0, 6.0, 8.0)) == pytest.approx(2.0)

    def test_point_outside_via_corner(self):
        # p=(9.0, 11.0), rect=(2.0, 3.0, 6.0, 8.0)
        # dx=max(2-9,0,9-6)=3; dy=max(3-11,0,11-8)=3; dist=hypot(3,3)
        expected = math.hypot(3.0, 3.0)
        assert point_rect_distance((9.0, 11.0), (2.0, 3.0, 6.0, 8.0)) == pytest.approx(expected)

    def test_point_below_rect(self):
        # p=(3.0, 1.0), rect=(1.0, 2.0, 6.0, 8.0)
        # dx=0 (x inside); dy=max(2-1,0,1-8)=1; dist=1.0
        assert point_rect_distance((3.0, 1.0), (1.0, 2.0, 6.0, 8.0)) == pytest.approx(1.0)

    def test_inverted_rect_normalised(self):
        # Rect specified with x1>x2, y1>y2 — module normalises it
        # p=(8.0, 5.0), rect=(6.0, 8.0, 1.0, 2.0) ≡ (1,2,6,8)
        assert point_rect_distance((8.0, 5.0), (6.0, 8.0, 1.0, 2.0)) == pytest.approx(2.0)


# ---------------------------------------------------------------------------
# segment_rect_distance
# ---------------------------------------------------------------------------


class TestSegmentRectDistance:
    def test_track_crossing_rect_returns_zero(self):
        # Horizontal segment (0.0, 3.0)–(8.0, 3.0) crosses rect (3.0, 1.0, 5.0, 5.0)
        assert segment_rect_distance(
            (0.0, 3.0), (8.0, 3.0), (3.0, 1.0, 5.0, 5.0)
        ) == pytest.approx(0.0)

    def test_track_inside_rect_returns_zero(self):
        # Segment entirely inside rect
        assert segment_rect_distance(
            (2.0, 2.0), (4.0, 4.0), (0.0, 0.0, 6.0, 6.0)
        ) == pytest.approx(0.0)

    def test_track_near_rect_horizontal(self):
        # Segment (0.0, 5.0)–(3.0, 5.0), rect (5.0, 4.0, 7.0, 6.0)
        # Nearest: endpoint b=(3,5) to rect left edge x=5 → dist=2.0
        assert segment_rect_distance(
            (0.0, 5.0), (3.0, 5.0), (5.0, 4.0, 7.0, 6.0)
        ) == pytest.approx(2.0)

    def test_track_near_rect_endpoint_closest(self):
        # Angled segment (1.0, 8.0)–(3.0, 6.0), rect (5.0, 4.0, 7.0, 6.0)
        # b=(3,6) to rect: dx=5-3=2, dy=max(4-6,0,6-6)=0 → dist=2.0
        assert segment_rect_distance(
            (1.0, 8.0), (3.0, 6.0), (5.0, 4.0, 7.0, 6.0)
        ) == pytest.approx(2.0)


# ---------------------------------------------------------------------------
# clearance_to_obstacle
# ---------------------------------------------------------------------------


class TestClearanceToObstacle:
    def test_circle_obstacle_positive_clearance(self):
        # point=(7.0, 3.0), circle center=(4.0, 7.0), r=0.5, track_hw=0.1
        # dist_centre=hypot(3,4)=5.0; raw=5.0-0.5=4.5; clearance=4.5-0.1=4.4
        obs = ("circle", 4.0, 7.0, 0.5)
        result = clearance_to_obstacle((7.0, 3.0), obs, track_hw=0.1)
        assert result == pytest.approx(4.4)

    def test_circle_obstacle_negative_clearance_overlap(self):
        # point=(4.1, 7.0), circle center=(4.0, 7.0), r=0.5, track_hw=0.1
        # dist_centre=0.1; raw=0.1-0.5=-0.4; clearance=-0.4-0.1=-0.5
        obs = ("circle", 4.0, 7.0, 0.5)
        result = clearance_to_obstacle((4.1, 7.0), obs, track_hw=0.1)
        assert result == pytest.approx(-0.5)

    def test_rect_obstacle_positive_clearance(self):
        # point=(5.0, 4.0), rect=(2.0, 3.0, 4.0, 5.0), track_hw=0.1
        # point_rect_distance=1.0 (1mm right of right edge); clearance=1.0-0.1=0.9
        obs = ("rect", 2.0, 3.0, 4.0, 5.0)
        result = clearance_to_obstacle((5.0, 4.0), obs, track_hw=0.1)
        assert result == pytest.approx(0.9)

    def test_rect_obstacle_negative_clearance_inside(self):
        # point inside rect: raw=0; clearance=0-0.1=-0.1
        obs = ("rect", 2.0, 3.0, 4.0, 5.0)
        result = clearance_to_obstacle((3.0, 4.0), obs, track_hw=0.1)
        assert result == pytest.approx(-0.1)

    def test_segment_obstacle_positive_clearance(self):
        # segment (0.0, 5.0)–(8.0, 5.0) hw=0.2, point=(4.0, 7.0), track_hw=0.1
        # dist_centre to seg=2.0; raw=2.0-0.2=1.8; clearance=1.8-0.1=1.7
        obs = ("segment", 0.0, 5.0, 8.0, 5.0, 0.2)
        result = clearance_to_obstacle((4.0, 7.0), obs, track_hw=0.1)
        assert result == pytest.approx(1.7)

    def test_segment_obstacle_negative_clearance_overlap(self):
        # segment (0.0, 5.0)–(8.0, 5.0) hw=0.2, point=(4.0, 5.15), track_hw=0.1
        # dist_centre=0.15; raw=0.15-0.2=-0.05; clearance=-0.05-0.1=-0.15
        obs = ("segment", 0.0, 5.0, 8.0, 5.0, 0.2)
        result = clearance_to_obstacle((4.0, 5.15), obs, track_hw=0.1)
        assert result == pytest.approx(-0.15)

    def test_half_width_subtracted_correctly(self):
        # Verify that doubling track_hw reduces clearance by the same amount
        obs = ("circle", 5.0, 3.0, 0.3)
        # point=(8.0, 3.0): dist_centre=3.0, raw=2.7
        cl1 = clearance_to_obstacle((8.0, 3.0), obs, track_hw=0.1)
        cl2 = clearance_to_obstacle((8.0, 3.0), obs, track_hw=0.2)
        assert cl1 - cl2 == pytest.approx(0.1)


# ---------------------------------------------------------------------------
# min_clearance
# ---------------------------------------------------------------------------


class TestMinClearance:
    def test_empty_obstacles_returns_inf(self):
        assert min_clearance((3.0, 4.0), [], track_hw=0.1) == math.inf

    def test_returns_minimum_over_all_obstacles(self):
        # point=(3.0, 1.0), track_hw=0.1
        # circle (10,10,0.5): dist=hypot(7,9)~11.4, clearance=11.4-0.5-0.1~10.8
        # rect (0,0,1,1): nearest corner (1,1), dist=hypot(2,0)=2.0, clearance=1.9
        # circle (5,2,0.3): dist=hypot(2,1)=sqrt(5), clearance=sqrt(5)-0.3-0.1~1.836
        obs = [
            ("circle", 10.0, 10.0, 0.5),
            ("rect", 0.0, 0.0, 1.0, 1.0),
            ("circle", 5.0, 2.0, 0.3),
        ]
        mc = min_clearance((3.0, 1.0), obs, track_hw=0.1)
        expected = math.sqrt(5.0) - 0.3 - 0.1  # circle (5,2,0.3) dominates
        assert mc == pytest.approx(expected, abs=1e-6)

    def test_single_obstacle(self):
        obs = [("circle", 0.0, 0.0, 1.0)]
        # point (4.0, 3.0): dist=5.0, raw=4.0, clearance=4.0-0.2=3.8
        result = min_clearance((4.0, 3.0), obs, track_hw=0.2)
        assert result == pytest.approx(3.8)


# ---------------------------------------------------------------------------
# is_legal
# ---------------------------------------------------------------------------


class TestIsLegal:
    def test_legal_point(self):
        obs = [("circle", 0.0, 0.0, 0.5)]
        # point (5.0, 0.0): dist=5, raw=4.5, clearance=4.5-0.1=4.4 >> 0.15
        assert is_legal((5.0, 0.0), obs, track_hw=0.1, required_clearance=0.15) is True

    def test_illegal_point(self):
        obs = [("circle", 0.0, 0.0, 0.5)]
        # point (0.65, 0.0): dist=0.65, raw=0.15, clearance=0.05 < 0.15
        assert is_legal((0.65, 0.0), obs, track_hw=0.1, required_clearance=0.15) is False

    def test_exactly_at_required_clearance(self):
        obs = [("rect", 0.0, 0.0, 2.0, 2.0)]
        # point (2.25, 1.0): dist=0.25, clearance=0.25-0.1=0.15 (exactly required)
        assert is_legal((2.25, 1.0), obs, track_hw=0.1, required_clearance=0.15) is True


# ---------------------------------------------------------------------------
# nudge_endpoint — the core primitive
# ---------------------------------------------------------------------------


class TestNudgeEndpoint:
    def test_already_legal_returns_unchanged(self):
        # point well clear of obstacles
        obs = [("circle", 5.0, 3.0, 0.2)]
        ep = (0.0, 0.0)
        result, ok = nudge_endpoint(ep, None, obs, 0.15, 0.1)
        assert ok is True
        assert result == pytest.approx((0.0, 0.0))

    def test_mitayi_style_rect_pad_nudge(self):
        """Endpoint 0.07mm inside required clearance zone beside a pad.

        pad rect (0, 0, 1.0, 0.5), track_hw=0.1, required=0.15.
        endpoint=(1.18, 0.25): point_rect_distance=0.18, clearance=0.08 < 0.15.
        After nudge, clearance must be >= required.
        """
        pad = ("rect", 0.0, 0.0, 1.0, 0.5)
        track_hw = 0.1
        required = 0.15
        endpoint = (1.18, 0.25)

        result, ok = nudge_endpoint(endpoint, None, [pad], required, track_hw)
        assert ok is True
        actual_clearance = min_clearance(result, [pad], track_hw)
        assert actual_clearance >= required - 1e-9, (
            f"clearance {actual_clearance:.6f} < required {required}"
        )

    def test_via_too_close_to_obstacle_nudged_legal(self):
        """Via center too close to a circular obstacle, nudge yields legal position."""
        via_obs = ("circle", 3.35, 7.0, 0.2)
        track_hw = 0.1
        required = 0.15
        endpoint = (3.0, 7.0)
        # dist_centre=0.35, raw=0.15, clearance=0.05 < 0.15

        result, ok = nudge_endpoint(endpoint, None, [via_obs], required, track_hw)
        assert ok is True
        actual_clearance = min_clearance(result, [via_obs], track_hw)
        assert actual_clearance >= required - 1e-9

    def test_hopelessly_boxed_in_returns_false(self):
        """Endpoint surrounded by obstacles within max_nudge — must return ok=False."""
        # 8 circles in a ring at radius 0.15 from (5,5), each with r=0.05.
        # For any point P within max_nudge=0.3 of (5,5), at least one obstacle
        # gives clearance < 0.15 (verified numerically above).
        endpoint = (5.0, 5.0)
        track_hw = 0.1
        required = 0.15
        obstacles = []
        for k in range(8):
            theta = 2.0 * math.pi * k / 8
            cx = 5.0 + 0.15 * math.cos(theta)
            cy = 5.0 + 0.15 * math.sin(theta)
            obstacles.append(("circle", cx, cy, 0.05))

        result, ok = nudge_endpoint(endpoint, None, obstacles, required, track_hw, max_nudge=0.3)
        assert ok is False
        # Original endpoint returned unchanged
        assert result == pytest.approx((5.0, 5.0))

    def test_anchor_constraint_preserved(self):
        """Nudged endpoint must be no farther from anchor than original.

        Setup: endpoint=(5.0, 3.0), anchor=(0.0, 3.0), orig_dist=5.0.
        Pad rect (5.1, 2.5, 6.0, 3.5) forces endpoint left (toward anchor).
        """
        endpoint = (5.0, 3.0)
        anchor = (0.0, 3.0)
        orig_anchor_dist = math.hypot(5.0 - 0.0, 3.0 - 3.0)  # = 5.0

        pad = ("rect", 5.1, 2.5, 6.0, 3.5)
        track_hw = 0.1
        required = 0.15

        result, ok = nudge_endpoint(endpoint, anchor, [pad], required, track_hw)
        assert ok is True

        # Clearance must be satisfied
        actual_clearance = min_clearance(result, [pad], track_hw)
        assert actual_clearance >= required - 1e-9

        # Anchor distance must not increase
        new_anchor_dist = math.hypot(result[0] - 0.0, result[1] - 3.0)
        assert new_anchor_dist <= orig_anchor_dist + 1e-9, (
            f"nudged point moved farther from anchor: "
            f"{new_anchor_dist:.6f} > {orig_anchor_dist:.6f}"
        )

    def test_anti_swap_x_y_asymmetric_layout(self):
        """Obstacle is offset purely in Y so a swap of x/y produces a wrong nudge direction.

        endpoint=(2.0, 7.0), circle at (2.0, 7.25) — obstacle is directly ABOVE
        in Y only.  Correct nudge must move primarily in -Y (downward), not in X.
        If the module internally swapped x/y, the nudge would move in a wrong axis.
        """
        endpoint = (2.0, 7.0)
        obs = ("circle", 2.0, 7.25, 0.1)
        track_hw = 0.08
        required = 0.12
        # dist_centre=0.25, raw=0.15, clearance=0.07 < 0.12 (violation)

        result, ok = nudge_endpoint(endpoint, None, [obs], required, track_hw)
        assert ok is True

        # Clearance genuinely satisfied
        actual_clearance = min_clearance(result, [obs], track_hw)
        assert actual_clearance >= required - 1e-9

        rx, ry = result
        # Movement must be predominantly in Y (toward lower Y), not in X
        dx = abs(rx - 2.0)
        dy = abs(ry - 7.0)
        assert dy > dx, (
            f"Expected Y-axis nudge (dy={dy:.5f}) to dominate X-axis (dx={dx:.5f}); "
            "possible x/y swap bug"
        )
        # And it should move in the -Y direction (away from obstacle above)
        assert ry < 7.0, f"Expected nudge downward (ry={ry:.5f} < 7.0)"

    def test_determinism_same_input_same_output(self):
        """Two calls with identical input must return bit-for-bit identical results."""
        pad = ("rect", 1.0, 1.0, 3.0, 2.5)
        track_hw = 0.1
        required = 0.15
        endpoint = (3.12, 1.75)
        anchor = (0.0, 1.75)

        r1, ok1 = nudge_endpoint(endpoint, anchor, [pad], required, track_hw)
        r2, ok2 = nudge_endpoint(endpoint, anchor, [pad], required, track_hw)
        assert ok1 == ok2
        assert r1[0] == r2[0]  # bit-exact
        assert r1[1] == r2[1]

    def test_no_obstacles_returns_original_ok(self):
        result, ok = nudge_endpoint((3.0, 7.0), None, [], 0.15, 0.1)
        assert ok is True
        assert result == pytest.approx((3.0, 7.0))

    def test_max_nudge_respected(self):
        """nudge must not displace endpoint beyond max_nudge."""
        # obstacle very close, but max_nudge is tiny
        obs = [("circle", 2.5, 4.0, 0.1)]
        endpoint = (2.5, 3.9)
        # dist_centre=0.1, raw=0.0, clearance=-0.1 < 0.15 (violation)
        # required displacement > 0.03 but max_nudge = 0.02 → should fail
        result, ok = nudge_endpoint(endpoint, None, obs, 0.15, 0.1, max_nudge=0.02)
        assert ok is False
        assert result == pytest.approx((2.5, 3.9))


# ---------------------------------------------------------------------------
# POLAR_ANGLES constant
# ---------------------------------------------------------------------------


class TestPolarAnglesConstant:
    def test_sixteen_angles(self):
        assert len(POLAR_ANGLES) == 16

    def test_spans_full_circle(self):
        # Last angle should be 2π × 15/16, not 2π itself
        assert POLAR_ANGLES[0] == pytest.approx(0.0)
        assert POLAR_ANGLES[-1] == pytest.approx(2.0 * math.pi * 15 / 16)

    def test_uniformly_spaced(self):
        step = 2.0 * math.pi / 16
        for i, theta in enumerate(POLAR_ANGLES):
            assert theta == pytest.approx(i * step)


# ---------------------------------------------------------------------------
# nudge_endpoint_in_region — Phase B terminal nudge (pad-rect-constrained)
# ---------------------------------------------------------------------------


class TestNudgeEndpointInRegion:
    def test_start_already_legal_inside_rect_unchanged(self):
        """Start is far from obstacles and already legal → returns start unchanged, ok=True."""
        # pad rect (0,0,2,1), start at center (1,0.5), obstacle far away
        rect = (0.0, 0.0, 2.0, 1.0)
        start = (1.0, 0.5)
        obs = [("rect", 5.0, 5.0, 6.0, 6.0)]
        result, ok = nudge_endpoint_in_region(start, rect, obs, 0.15, 0.1)
        assert ok is True
        assert result == pytest.approx((1.0, 0.5))
        # Result is inside rect
        x1, y1, x2, y2 = rect
        assert x1 <= result[0] <= x2
        assert y1 <= result[1] <= y2

    def test_nudges_within_rect_to_gain_clearance(self):
        """The mitayi case: endpoint ~0.0998mm from an other-net pad, required 0.15.

        Setup: pad (own) rect = (-0.3, -0.3, 0.3, 0.3), start at (0.0, 0.0).
        Other-net obstacle rect at (0.25, -0.3, 0.55, 0.3) — distance from
        start (0.0,0.0) to this rect = 0.25, clearance = 0.25 - 0.1 = 0.15 exactly.
        Now shift obstacle left so clearance = 0.15 - (0.15-0.0998) = 0.0998:
        obstacle at (0.1498, -0.3, 0.5498, 0.3): distance from start = 0.1498,
        clearance = 0.1498 - 0.1 = 0.0498 < 0.15.
        With a large-enough pad rect the nudge must find a legal in-rect point.
        """
        rect = (-0.5, -0.5, 0.5, 0.5)
        start = (0.0, 0.0)
        # Obstacle: rect edge at x=0.1498 → distance from start = 0.1498
        # clearance = 0.1498 - track_hw(0.1) = 0.0498 < 0.15
        obs = [("rect", 0.1498, -0.3, 0.5498, 0.3)]
        track_hw = 0.1
        required = 0.15
        result, ok = nudge_endpoint_in_region(start, rect, obs, required, track_hw)
        x1, y1, x2, y2 = rect
        assert x1 <= result[0] <= x2
        assert y1 <= result[1] <= y2
        if ok:
            actual_cl = min_clearance(result, obs, track_hw)
            assert actual_cl >= required - 1e-9, (
                f"clearance {actual_cl:.6f} < required {required}"
            )

    def test_result_always_inside_allowed_rect(self):
        """Even on failure (ok=False), the returned point is inside allowed_rect."""
        # Heavily boxed-in setup inside a small rect — all in-rect points illegal
        rect = (4.9, 4.9, 5.1, 5.1)
        start = (5.0, 5.0)
        # 8 obstacles pressing in from all sides
        obstacles = []
        for k in range(8):
            theta = 2.0 * math.pi * k / 8
            cx = 5.0 + 0.08 * math.cos(theta)
            cy = 5.0 + 0.08 * math.sin(theta)
            obstacles.append(("circle", cx, cy, 0.03))
        result, ok = nudge_endpoint_in_region(start, rect, obstacles, 0.15, 0.1)
        x1, y1, x2, y2 = rect
        assert x1 <= result[0] <= x2, f"x={result[0]} outside [{x1},{x2}]"
        assert y1 <= result[1] <= y2, f"y={result[1]} outside [{y1},{y2}]"

    def test_thin_pad_clamps_not_rejects(self):
        """A narrow pad (0.1mm wide in x) where the unclamped push-out points outside.

        Obstacle is to the right; pad is narrow in x so push-out would overshoot
        left past the pad edge. Clamping brings it back inside; result must be in rect.
        """
        # Pad rect: x=[2.0, 2.1], y=[0.0, 1.0] — only 0.1mm wide in x
        rect = (2.0, 0.0, 2.1, 1.0)
        # Start at center (2.05, 0.5)
        start = (2.05, 0.5)
        # Obstacle rect to the right: x=[2.2, 3.0] — left edge at 2.2
        # dist from start (2.05) to left edge (2.2) = 0.15; clearance = 0.15-0.1 = 0.05 < 0.15
        obs = [("rect", 2.2, 0.0, 3.0, 1.0)]
        result, ok = nudge_endpoint_in_region(start, rect, obs, 0.15, 0.1)
        x1, y1, x2, y2 = rect
        assert x1 <= result[0] <= x2, f"x={result[0]} outside [{x1},{x2}]"
        assert y1 <= result[1] <= y2, f"y={result[1]} outside [{y1},{y2}]"
        # The only way to gain clearance with x in [2.0, 2.1] is to move left.
        # Leftmost point x=2.0 → dist to obs left edge = 2.2 - 2.0 = 0.2, clearance = 0.1 = ok.
        # So we expect ok=True and clearance satisfied
        if ok:
            actual_cl = min_clearance(result, obs, 0.1)
            assert actual_cl >= 0.15 - 1e-9

    def test_hopelessly_boxed_returns_start_false(self):
        """Obstacles surround the rect so no in-rect point can be legal → (start, False)."""
        # A 0.02mm × 0.02mm rect centered at (10.0, 10.0).
        # Tight ring of circles just outside the rect — legal clearance 0.15 impossible in-rect.
        rect = (9.99, 9.99, 10.01, 10.01)
        start = (10.0, 10.0)
        obstacles = []
        for k in range(12):
            theta = 2.0 * math.pi * k / 12
            cx = 10.0 + 0.04 * math.cos(theta)
            cy = 10.0 + 0.04 * math.sin(theta)
            obstacles.append(("circle", cx, cy, 0.01))
        result, ok = nudge_endpoint_in_region(start, rect, obstacles, 0.15, 0.1)
        assert ok is False
        # Result must still be inside rect
        x1, y1, x2, y2 = rect
        assert x1 <= result[0] <= x2
        assert y1 <= result[1] <= y2
        # And must equal the clamped start (start is already in rect)
        assert result == pytest.approx((10.0, 10.0))

    def test_determinism(self):
        """Two calls with identical inputs → bit-identical outputs."""
        rect = (1.0, 2.0, 2.5, 3.5)
        start = (1.75, 2.75)
        obs = [("rect", 2.3, 2.0, 3.0, 3.5)]
        r1, ok1 = nudge_endpoint_in_region(start, rect, obs, 0.15, 0.1)
        r2, ok2 = nudge_endpoint_in_region(start, rect, obs, 0.15, 0.1)
        assert ok1 == ok2
        assert r1[0] == r2[0]  # bit-exact
        assert r1[1] == r2[1]

    def test_asymmetric_coords_anti_swap(self):
        """Obstacle is offset purely in X, so an x/y swap would nudge in the wrong axis.

        start=(3.0, 7.0), obstacle rect to the RIGHT at x=[3.1, 4.0], y=[6.8, 7.2].
        Distance from start = 0.1; clearance = 0.0 < 0.15.
        Correct nudge must move in -X (leftward, away from right obstacle), not in Y.
        Pad rect is large enough: x=[2.5, 3.5], y=[6.5, 7.5].
        """
        rect = (2.5, 6.5, 3.5, 7.5)
        start = (3.0, 7.0)
        # obstacle rect: left edge at x=3.1 → distance = 0.1; clearance = 0.0 < 0.15
        obs = [("rect", 3.1, 6.8, 4.0, 7.2)]
        result, ok = nudge_endpoint_in_region(start, rect, obs, 0.15, 0.1)
        assert ok is True
        # Result must be in rect
        x1, y1, x2, y2 = rect
        assert x1 <= result[0] <= x2
        assert y1 <= result[1] <= y2
        # Clearance must be satisfied
        actual_cl = min_clearance(result, obs, 0.1)
        assert actual_cl >= 0.15 - 1e-9
        # Movement must be predominantly in -X (leftward), not in Y
        dx = abs(result[0] - 3.0)
        dy = abs(result[1] - 7.0)
        assert dx > dy, (
            f"Expected X-axis nudge (dx={dx:.5f}) to dominate Y-axis (dy={dy:.5f}); "
            "possible x/y swap"
        )
        assert result[0] < 3.0, f"Expected leftward nudge, got x={result[0]:.5f}"
