"""Doorway regressions: immediate folding, stable reopening and safe concurrency."""

import math

from astar_planner.adaptive_leader import AdaptiveLeaderFormation
from astar_planner.formation_motion import assemble_followers, team_motion_free
from astar_planner.grid_clearance import GridClearance
from astar_planner.leader_follower import Polyline


TRIANGLE = ((-0.7, -0.5, 0.0), (-0.7, 0.5, 0.0), (0.0, 0.0, 0.0))
COLUMN = ((-1.4, 0.0, 0.0), (-0.7, 0.0, 0.0), (0.0, 0.0, 0.0))


def grid_with_boxes(boxes):
    width, height, size = 200, 120, 0.05
    occupied = []
    for y in range(height):
        for x in range(width):
            px, py = -4.0 + (x + 0.5) * size, -3.0 + (y + 0.5) * size
            occupied.append(any(left <= px <= right and bottom <= py <= top
                                for left, bottom, right, top in boxes))
    return GridClearance(width, height, size, (-4.0, -3.0, 0.0), occupied, 0.18)


def doorway():
    return grid_with_boxes([(0.8, 0.65, 1.6, 2.0), (0.8, -2.0, 1.6, -0.65)])


def formation(room, **options):
    # The mutable room lets a new obstacle arrive during an expansion.
    segment = lambda a, b: room[0].segment_free(a, b)
    clearance = lambda p: room[0].clearance(p)
    return AdaptiveLeaderFormation(
        [(0.0, 0.0), (4.0, 0.0)], [(-1.4, 0.0), (0.0, 0.0)], [0.0, 0.7, 0.0],
        (2, 1, 0), segment, clearance, segment, lambda a, b, p: [a, b], **options)


def test_doorway_folds_directly_even_during_decision_cooldown():
    f = formation([doorway()])
    f.state, f.width, f.next_decision = 'triangle', 0.5, 100.0
    assert f.observe(TRIANGLE)
    assert f.wide_safe(0.4, entering=True), 'A smaller triangle fits this doorway'
    assert not f.wide_safe(0.5)
    f.rows(TRIANGLE, 0.1, 8, 0.15)
    assert f.state == 'transition' and f.transition_state == 'column'
    assert f.width == 0.0, 'Skip the intermediate stopped triangle assemblies'
    assert f.transition.times[-1] <= math.hypot(0.7, 0.5) / 0.18 + 0.11
    assert all(f.transition_free(a, b) for a, b in zip(f.transition.rows, f.transition.rows[1:]))


def test_initial_route_with_doorway_prefers_column_over_compressed_triangle():
    f = formation([doorway()])
    assert f.observe(TRIANGLE)
    f.rows(TRIANGLE, 0.1, 8, 0.15)
    assert f.transition_state == 'column'


def test_leader_clearing_doorway_does_not_expand_over_followers_still_inside():
    f = formation([doorway()])
    f.state, f.width, f.elapsed = 'column', 0.0, 10.0
    f.column.progress[2] = 1.8
    f.column.trail = Polyline([(-1.4, 0.0), (1.8, 0.0)])
    f.poses = tuple((x + 1.8, y, yaw) for x, y, yaw in COLUMN)
    f.open_candidate, f.open_since = 0.5, 0.0
    f.decide()
    assert f.state == 'column' and f.open_candidate is None


def test_expansion_requires_continuous_open_space_and_resets_at_doorway():
    room = [grid_with_boxes([])]
    f = formation(room, open_hold=0.6)
    f.state, f.poses = 'column', COLUMN
    for seconds in (0.0, 0.3):
        f.elapsed = seconds
        f.decide()
        assert f.state == 'column'
    room[0] = doorway()
    f.elapsed = 0.6
    f.decide()
    assert f.open_candidate is None and f.state == 'column'
    room[0] = grid_with_boxes([])
    for seconds in (0.9, 1.2, 1.4):
        f.elapsed = seconds
        f.decide()
        assert f.state == 'column'
    f.elapsed = 1.5
    f.decide()
    assert f.state == 'transition' and f.transition_state == 'triangle'


def test_new_doorway_interrupts_expansion_from_measured_poses():
    room = [grid_with_boxes([])]
    f = formation(room)
    f.state, f.poses = 'column', COLUMN
    assert f.begin_transition('triangle', 0.5)
    f.poses = f.transition.sample(0.25 * f.transition.times[-1])
    room[0] = doorway()
    f.decide()
    assert f.state == 'transition' and f.transition_state == 'column'
    assert f.transition.rows[0] == f.poses


def test_followers_use_simultaneous_routes_when_direct_motion_hits_walls():
    grid = grid_with_boxes([(-0.2, 0.3, 0.2, 0.7), (-0.2, -0.7, 0.2, -0.3)])
    actual = ((-0.9, -0.5, 0.0), (-0.9, 0.5, 0.0), (0.0, 2.5, 0.0))
    target = ((0.9, -0.5, 0.0), (0.9, 0.5, 0.0), actual[2])
    def routed(a, b, peers):
        side = 1.0 if a[1] > 0.0 else -1.0
        return [a, (-0.5, side), (0.5, side), b]
    assert not team_motion_free(actual, target, grid.segment_free)
    rows = assemble_followers(actual, target, (2, 1, 0), grid.segment_free, routed, max_distance=2.0)
    assert rows is not None
    assert all(team_motion_free(a, b, grid.segment_free) for a, b in zip(rows, rows[1:]))
    assert all(row[2] == actual[2] for row in rows)
    duration = sum(max(math.dist(a[:2], b[:2]) for a, b in zip(p, q))
                   for p, q in zip(rows, rows[1:]))
    assert duration <= Polyline(routed(actual[0], target[0], [])).length + 1e-8
    assert any(all(math.dist(p[i][:2], q[i][:2]) > 0.01 for i in (0, 1))
               for p, q in zip(rows, rows[1:]))


def test_crossing_routes_fall_back_to_sequential_motion():
    grid = grid_with_boxes([])
    actual = ((-1.0, -0.5, 0.0), (-1.0, 0.5, 0.0), (0.0, 2.5, 0.0))
    target = ((1.0, 0.5, 0.0), (1.0, -0.5, 0.0), actual[2])
    rows = assemble_followers(actual, target, (2, 1, 0), grid.segment_free,
                              lambda a, b, p: [a, b], max_distance=2.5)
    assert rows is not None
    assert all(team_motion_free(a, b, grid.segment_free) for a, b in zip(rows, rows[1:]))
    assert not any(all(math.dist(p[i][:2], q[i][:2]) > 0.01 for i in (0, 1))
                   for p, q in zip(rows, rows[1:]))
