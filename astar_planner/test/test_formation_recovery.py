"""Regressions for measured peer deadlocks and adaptive formation references."""

import math
from pathlib import Path

from astar_planner.adaptive_leader import AdaptiveLeaderFormation
from astar_planner.formation_motion import GatedTeamMotion, team_motion_free
from astar_planner.grid_clearance import GridClearance
from astar_planner.peer_safety import minimum_team_distance, peer_motion_free
from astar_planner.separation_recovery import SeparationRecoveryPlanner


STUCK_POSES = (
    (1.342922376101486, -0.39201541689536523, 3.06508),
    (0.7355350449196757, -0.2807558357677953, 2.043857),
    (0.4241935441299629, -0.14782539739406084, -0.875016),
)


def maze_grid(radius=0.18):
    path = Path(__file__).resolve().parents[2] / 'ros2_omni_robot_sim/map/my_maze.pgm'
    with path.open('rb') as stream:
        tokens = []
        while len(tokens) < 4:
            line = stream.readline()
            if line.startswith(b'#'):
                continue
            tokens.extend(line.split())
        assert tokens[0] == b'P5' and tokens[3] == b'255'
        width, height = int(tokens[1]), int(tokens[2])
        pixels = stream.read()
    assert len(pixels) == width * height
    occupied = [pixels[(height - y - 1) * width + x] < 128
                for y in range(height) for x in range(width)]
    return GridClearance(width, height, 0.05, (-4.95, -4.975, 0.0), occupied, radius)


def test_close_pair_can_separate_but_cannot_approach():
    a, b = (0.0, 0.0), (0.339, 0.0)
    assert peer_motion_free(a, (-0.05, 0.0), b, b)
    assert peer_motion_free(a, a, b, b)
    assert not peer_motion_free(a, (0.01, 0.0), b, b)
    assert not peer_motion_free(a, (0.0, 0.0), (0.31, 0.0), (0.4, 0.0))
    assert not peer_motion_free(a, (0.10, 0.0), (0.40, 0.0), (0.40, 0.0))
    assert not peer_motion_free(a, (0.05, 0.0), (0.40, 0.0), (0.40, 0.0), physical=0.36)


def test_logged_maze_positions_have_a_checked_peer_escape():
    grid = maze_grid()
    planner = SeparationRecoveryPlanner(grid.segment_free, grid.clearance, (2, 1, 0))
    motion = planner.build(STUCK_POSES)
    assert motion is not None, planner.failure_reason
    rows = motion.stages[0][1]
    assert minimum_team_distance(rows[-1]) >= 0.46
    assert rows[-1][2] == STUCK_POSES[2], 'Prefer follower recovery; leader should hold'
    for before, after in zip(rows, rows[1:]):
        assert team_motion_free(before, after, grid.segment_free)
        assert sum(math.dist(a[:2], b[:2]) > 1e-6 for a, b in zip(before, after)) == 1


def test_recovery_rejects_escape_through_walls():
    blocked_motion = lambda a, b: math.dist(a[:2], b[:2]) < 1e-10
    planner = SeparationRecoveryPlanner(blocked_motion, lambda p: 0.5, (2, 1, 0))
    assert planner.build(STUCK_POSES) is None


def test_turn_stage_waits_for_measured_yaw_before_passing():
    parked = ((-0.7, -0.7, 0.0), (-0.7, 0.7, 0.0), (0.0, 0.0, 0.0))
    turned = (*parked[:2], (0.0, 0.0, math.pi))
    passed = (*parked[:2], (-0.5, 0.0, math.pi))
    motion = GatedTeamMotion([('turn', [parked, turned], (2,)), ('pass', [turned, passed], ())])
    motion.phase = motion.timelines[0][-1]
    motion.rows(parked, 0.1, 8, 0.15)
    assert motion.stage_index == 0
    motion.rows(turned, 0.1, 8, 0.15)
    assert motion.stage_index == 1


def test_triangle_initialization_does_not_require_a_long_trail():
    poses = ((-0.7, -0.5, 0.0), (-0.7, 0.5, 0.0), (0.0, 0.0, 0.0))
    formation = AdaptiveLeaderFormation(
        [(0.0, 0.0), (2.0, 0.0)], [(0.0, 0.0)], [0.0] * 3, (2, 1, 0),
        lambda a, b: True, lambda p: 2.0, lambda a, b: True, lambda a, b, p: [a, b])
    assert formation.observe(poses)
    formation.rows(poses, 0.1, 8, 0.15)
    assert formation.transition_state == 'triangle'


def test_triangle_reference_respects_follower_speed_on_a_bend():
    formation = AdaptiveLeaderFormation(
        [(0.0, 0.0), (0.3, 0.0), (0.3, 0.8)], [(0.0, 0.0)], [0.0] * 3, (2, 1, 0),
        lambda a, b: True, lambda p: 2.0, lambda a, b: True, lambda a, b, p: [a, b])
    formation.state, formation.width = 'triangle', 0.5
    formation.column.progress[2] = 0.15
    poses = formation.configuration(0.0, 0.5)
    formation.poses = poses
    assert formation.wide_safe(0.5), 'A clear bend should retain the triangle'
    rows = formation.triangle_rows(0.1, 8, 0.15)
    for before, after in zip(rows, rows[1:]):
        assert max(math.dist(a[:2], b[:2]) for a, b in zip(before, after)) <= 0.22 * 0.15 + 1e-6
