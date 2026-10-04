"""Measured heading releases the leader without waiting for follower assembly."""

import math

import pytest

from astar_planner.adaptive_structure import wrap
from astar_planner.formation_motion import team_motion_free
from astar_planner.peer_safety import peer_motion_free
from astar_planner.leader_follower import Polyline
from astar_planner.turnaround import RollingTurnMotion, TurnaroundPlanner, needs_turnaround
from test_fast_formation_switch import TRIANGLE, grid_with_boxes


def planner(grid):
    def route(a, b, peers):
        if grid.segment_free(a, b) and all(peer_motion_free(a, b, p, p, preferred=0.38) for p in peers):
            return [a, b]
        return None
    return TurnaroundPlanner(grid.segment_free, grid.clearance, route, (2, 1, 0))


@pytest.mark.parametrize('angle', [math.pi, math.pi / 2, -math.pi / 2])
def test_open_turn_prefers_rolling_motion(angle):
    grid = grid_with_boxes([])
    route = [(0.0, 0.0), (3.0 * math.cos(angle), 3.0 * math.sin(angle))]
    assert needs_turnaround(TRIANGLE, route, (2, 1, 0))
    motion = planner(grid).build(TRIANGLE, route)
    assert isinstance(motion, RollingTurnMotion)
    rows = motion.rows(TRIANGLE, 0.1, 8, 0.15)
    assert all(math.dist(row[2][:2], TRIANGLE[2][:2]) < 1e-8 for row in rows)
    assert abs(wrap(rows[0][2][2] - angle)) < 1e-6
    aligned = (*TRIANGLE[:2], (*TRIANGLE[2][:2], angle))
    rows = motion.rows(aligned, 0.1, 8, 0.15)
    assert math.dist(rows[0][2][:2], aligned[2][:2]) > 0.005
    assert team_motion_free(aligned, rows[0], grid.segment_free)
    assert all(team_motion_free(a, b, grid.segment_free) for a, b in zip(rows, rows[1:]))


def test_followers_move_during_leader_alignment():
    grid = grid_with_boxes([])
    motion = planner(grid).build(TRIANGLE, [(0.0, 0.0), (0.0, 3.0)])
    assert isinstance(motion, RollingTurnMotion)
    rows = motion.rows(TRIANGLE, 0.1, 8, 0.15)
    assert any(math.dist(rows[0][i][:2], TRIANGLE[i][:2]) > 0.005 for i in (0, 1))
    assert rows[0][2][:2] == TRIANGLE[2][:2]


def test_nearby_bend_is_handled_before_rigid_slot_rotation_slows_the_leader():
    assert needs_turnaround(TRIANGLE, [(0.0, 0.0), (0.35, 0.0), (0.35, 2.0)], (2, 1, 0))
    assert not needs_turnaround(TRIANGLE, [(0.0, 0.0), (3.0, 0.0)], (2, 1, 0))


def test_yaw_release_uses_measured_error_without_a_nominal_stage_timer():
    grid = grid_with_boxes([])
    motion = planner(grid).build(TRIANGLE, [(0.0, 0.0), (-3.0, 0.0)])
    almost = (*TRIANGLE[:2], (0.0, 0.0, math.pi - 0.25))
    rows = motion.rows(almost, 0.1, 8, 0.15)
    assert rows[0][2][:2] == almost[2][:2]
    aligned = (*TRIANGLE[:2], (0.0, 0.0, math.pi - 0.15))
    rows = motion.rows(aligned, 0.1, 8, 0.15)
    assert math.dist(rows[0][2][:2], aligned[2][:2]) > 0.005


def test_live_sweeps_hold_the_leader_if_a_follower_moves_into_its_route():
    grid = grid_with_boxes([])
    motion = planner(grid).build(TRIANGLE, [(0.0, 0.0), (-3.0, 0.0)])
    actual = ((-0.7, -0.5, 0.0), (-0.39, 0.0, 0.0), (0.0, 0.0, math.pi))
    rows = motion.rows(actual, 0.1, 8, 0.15)
    # A displaced follower invalidates its track; hold/replan instead of
    # advancing on a nominal plan that is no longer supported by measurements.
    assert motion.failure_reason
    assert rows == [actual] * 9


def test_rolling_plan_does_not_accept_a_route_through_a_wall():
    grid = grid_with_boxes([(-1.0, -0.2, -0.3, 0.2)])
    motion = planner(grid).build(TRIANGLE, [(0.0, 0.0), (-3.0, 0.0)])
    assert motion is None


def test_live_sweeps_limit_progress_when_a_delayed_follower_blocks_the_route():
    grid = grid_with_boxes([])
    motion = planner(grid).build(TRIANGLE, [(0.0, 0.0), (-3.0, 0.0)])
    motion.delays[1] = 0.7
    actual = (TRIANGLE[0], (-0.7, 0.21, math.pi), (-0.25, 0.0, math.pi))
    rows = motion.rows(actual, 0.1, 8, 0.15)
    assert not motion.failure_reason
    assert all(row[1][:2] == actual[1][:2] for row in rows)
    assert max(math.dist(row[2][:2], actual[2][:2]) for row in rows) < 0.14
    assert team_motion_free(actual, rows[0], grid.segment_free)
    assert all(team_motion_free(a, b, grid.segment_free) for a, b in zip(rows, rows[1:]))


def test_reference_horizon_preserves_a_short_route_corner():
    grid = grid_with_boxes([])
    actual = (*TRIANGLE[:2], (0.0, 0.0, math.pi / 2))
    lines = [Polyline([p[:2]]) for p in actual]
    lines[2] = Polyline([(0.0, 0.0), (0.03, 0.0), (0.03, 1.0)])
    targets = (*actual[:2], (0.03, 1.0, math.pi / 2))
    motion = RollingTurnMotion(actual, lines, targets, [0.0] * 3, (2, 1, 0),
                               grid.segment_free, grid.clearance)
    rows = motion.rows(actual, 0.1, 8, 0.15)
    assert any(math.dist(row[2][:2], (0.03, 0.0)) < 1e-8 for row in rows)
