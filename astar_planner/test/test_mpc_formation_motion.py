"""Run the production MPC against kinematic feedback without publishing to ROS."""

import math
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
from astar_planner.three_robot_dstar_mpc import DistributedRobotMPC, ThreeRobotDStarMPC
from astar_planner.adaptive_leader import AdaptiveLeaderFormation
from astar_planner.grid_clearance import GridClearance
from astar_planner.peer_safety import minimum_team_distance, peer_motion_free
from astar_planner.separation_recovery import SeparationRecoveryPlanner
from astar_planner.turnaround import RollingTurnMotion, TurnaroundPlanner
from sensor_msgs.msg import LaserScan
from rclpy.time import Time

from test_formation_recovery import STUCK_POSES, maze_grid
from test_fast_formation_switch import TRIANGLE, doorway, formation


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages[:] = [message]


def controller(index, grid, clock):
    # No Node constructor, executor, network publisher or robot command topic.
    node = DistributedRobotMPC.__new__(DistributedRobotMPC)
    node.robot_index = index
    node.robot_radius, node.inflation_radius = 0.15, 0.18
    node.preferred_clearance, node.peer_preferred_distance = 0.22, 0.46
    node.max_linear_speed, node.max_angular_speed = 0.28, 1.0
    node.mpc_horizon, node.mpc_dt = 8, 0.15
    node.motion_enabled = True
    node.blocked = list(grid.occupied)
    node.hard_collision_grid = grid
    node.reference = []
    node.reference_time = node.state_time = 0.0
    node.peer_predictions = [None] * 3
    node.peer_prediction_times = [-float('inf')] * 3
    node.scan_points = []
    node.last_command = (0.0, 0.0, 0.0)
    node.last_control_time = None
    node.cmd_pub, node.prediction_pub = Capture(), Capture()
    node.now_seconds = lambda: clock[0]
    return node


def step_team(nodes, poses, rows, now):
    old_predictions = [getattr(node, 'prediction', None) for node in nodes]
    for i, node in enumerate(nodes):
        node.own_pose, node.peer_poses = poses[i], poses
        node.reference, node.reference_time, node.state_time = rows, now, now
        node.peer_predictions = old_predictions.copy()
        node.peer_prediction_times = [now - 0.1] * 3
        node.control_tick()
        node.prediction = node.poses_from(node.prediction_pub.messages[-1])
    result = []
    for pose, node in zip(poses, nodes):
        vx, vy, spin = node.last_command
        yaw = pose[2] + 0.5 * spin * 0.1
        result.append((pose[0] + 0.1 * (vx * math.cos(yaw) - vy * math.sin(yaw)),
                       pose[1] + 0.1 * (vx * math.sin(yaw) + vy * math.cos(yaw)),
                       pose[2] + 0.1 * spin))
    return tuple(result)


def coordinator(poses, grid):
    node = ThreeRobotDStarMPC.__new__(ThreeRobotDStarMPC)
    node.__dict__.update(
        robot_names=['robot1', 'robot2', 'robot3'], convoy_order=(2, 1, 0), leader_index=2,
        robot_radius=0.15, inflation_radius=0.18, occupied_threshold=50, unknown_is_obstacle=True,
        width=grid.width, height=grid.height, resolution=grid.resolution,
        origin_x=grid.origin_x, origin_y=grid.origin_y, origin_yaw=0.0,
        routing_peers=[], dynamic_obstacles=[], assembly_route_cache={}, dstar=None,
        goal_xy=(0.835641, -2.78159), map_msg=object(), adaptive_formation=True,
        failed_turn_position=None, follow_gap=0.70, triangle_half_width=0.50, formation_lookahead=1.20,
        transition_speed=0.18, formation_open_hold=0.60,
        max_linear_speed=0.28, max_angular_speed=1.0,
        robot_state=[(*p, True) for p in poses], structure=None, convoy=None, maneuver=None,
        recovering_peers=False, robot_paths=[None] * 3, replan_required=True,
        last_formation_mode=None, last_convoy_display=-float('inf'))
    node.now_seconds = lambda: 100.0
    node.poses_ready = lambda: True
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=100.0))
    node.get_parameter = lambda name: SimpleNamespace(value={'formation_speed': 0.16, 'narrow_speed': 0.08}[name])
    node.diagnostics = []
    logger = SimpleNamespace(info=lambda text, **kw: node.diagnostics.append(text),
                             warning=lambda text, **kw: node.diagnostics.append(text))
    node.get_logger = lambda: logger
    node.path_pub, node.reference_pub = Capture(), Capture()
    node.path_pubs = [Capture() for _ in range(3)]
    node.blocked = node.make_inflated_grid([100 if blocked else 0 for blocked in grid.occupied])
    return node


def test_mpc_escapes_the_logged_close_pair_on_the_real_maze():
    nominal, hard = maze_grid(), maze_grid(0.15)
    planner = SeparationRecoveryPlanner(nominal.segment_free, nominal.clearance, (2, 1, 0))
    motion = planner.build(STUCK_POSES)
    assert motion is not None
    clock, poses = [0.0], STUCK_POSES
    nodes = [controller(i, hard, clock) for i in range(3)]
    lowest = minimum_team_distance(poses)
    moved = False
    for k in range(100):
        clock[0] = k * 0.1
        rows = motion.rows(poses, 0.1, 8, 0.15)
        after = step_team(nodes, poses, rows, clock[0])
        assert all(hard.segment_free(a, b) for a, b in zip(poses, after))
        assert all(peer_motion_free(poses[i], after[i], poses[j], after[j])
                   for i, j in ((0, 1), (0, 2), (1, 2)))
        assert minimum_team_distance(after) >= lowest - 1e-8
        moved |= any(math.dist(a[:2], b[:2]) > 0.001 for a, b in zip(poses, after))
        poses = after
        if motion.finished:
            break
    assert moved, 'MPC must produce a separating command instead of only zero commands'
    assert motion.finished, 'Measured feedback must complete the recovery'
    assert minimum_team_distance(poses) > 0.40
    replanned = coordinator(poses, nominal)
    replanned.plan()
    assert replanned.convoy is not None, replanned.diagnostics
    assert not replanned.recovering_peers
    print(f'Logged-state recovery: {lowest:.4f} -> {minimum_team_distance(poses):.4f} m '
          f'in {clock[0]:.1f} simulated seconds; goal route replanned.')


def test_mpc_tracks_a_triangle_around_an_open_bend():
    hard = GridClearance(240, 240, 0.05, (-6.0, -6.0, 0.0), [False] * (240 * 240), 0.15)
    formation = AdaptiveLeaderFormation(
        [(0.0, 0.0), (1.2, 0.0), (1.2, 1.5)], [(0.0, 0.0)], [0.0] * 3,
        (2, 1, 0), hard.segment_free, hard.clearance, hard.segment_free, lambda a, b, p: [a, b])
    poses = ((-0.7, -0.5, 0.0), (-0.7, 0.5, 0.0), (0.0, 0.0, 0.0))
    clock = [0.0]
    nodes = [controller(i, hard, clock) for i in range(3)]
    errors = []
    for k in range(140):
        clock[0] = k * 0.1
        assert formation.observe(poses), formation.failure_reason
        rows = formation.rows(poses, 0.1, 8, 0.15)
        poses = step_team(nodes, poses, rows, clock[0])
        assert minimum_team_distance(poses) > 0.40
        if formation.state == 'triangle':
            target = formation.configuration(0.0, formation.width)
            errors.append(max(math.dist(a[:2], b[:2]) for a, b in zip(poses, target)))
        assert formation.state != 'column', 'A clear bend must not force single file'
    assert poses[2][0] > 1.0, 'Leader should reach the bend without a formation deadlock'
    assert errors and max(errors) < 0.13
    assert sum(errors) / len(errors) < 0.06
    print(f'Open-bend triangle: peak error {max(errors):.4f} m; mean {sum(errors) / len(errors):.4f} m.')


def test_faster_fold_completes_with_mpc_feedback_and_clearance():
    elapsed = []
    for speed in (0.08, 0.18):
        grid = doorway()
        f = formation([grid], transition_speed=speed)
        poses, clock = TRIANGLE, [0.0]
        f.state, f.width, f.poses = 'triangle', 0.5, poses
        assert f.begin_transition('column', 0.0)
        nodes = [controller(i, grid, clock) for i in range(3)]
        for k in range(160):
            clock[0] = k * 0.1
            assert f.observe(poses), f.failure_reason
            rows = f.rows(poses, 0.1, 8, 0.15)
            after = step_team(nodes, poses, rows, clock[0])
            assert all(grid.segment_free(a, b) for a, b in zip(poses, after))
            assert minimum_team_distance(after) > 0.40
            assert math.dist(after[2][:2], TRIANGLE[2][:2]) < 0.04
            poses = after
            if f.state == 'column':
                elapsed.append(clock[0])
                break
        else:
            pytest.fail(f'Fold did not finish at reference speed {speed}')
    assert elapsed[1] < 0.65 * elapsed[0], 'Faster references must shorten measured completion time'
    print(f'MPC fold feedback: {elapsed[0]:.1f} s at 0.08 m/s -> {elapsed[1]:.1f} s at 0.18 m/s.')


@pytest.mark.parametrize('route', [
    [(0.0, 0.0), (-3.0, 0.0)],
    [(0.0, 0.0), (0.0, 3.0)],
    [(0.0, 0.0), (0.35, 0.0), (0.35, 2.5)],
], ids=['reverse', 'right_angle', 'bend_ahead'])
def test_rolling_turn_tracks_with_mpc_feedback(route):
    grid = GridClearance(240, 240, 0.05, (-6.0, -6.0, 0.0), [False] * (240 * 240), 0.18)
    router = coordinator(TRIANGLE, grid)
    planner = TurnaroundPlanner(grid.segment_free, grid.clearance, router.route_between, (2, 1, 0))
    motion = planner.build(TRIANGLE, route)
    assert isinstance(motion, RollingTurnMotion)
    clock, poses, departure = [0.0], TRIANGLE, None
    nodes = [controller(i, grid, clock) for i in range(3)]
    minimum_spacing = minimum_team_distance(poses)
    for k in range(250):
        clock[0] = k * 0.1
        rows = motion.rows(poses, 0.1, 8, 0.15)
        assert not motion.failure_reason
        after = step_team(nodes, poses, rows, clock[0])
        assert all(grid.segment_free(a, b) for a, b in zip(poses, after))
        assert all(peer_motion_free(poses[i], after[i], poses[j], after[j])
                   for i, j in ((0, 1), (0, 2), (1, 2)))
        minimum_spacing = min(minimum_spacing, minimum_team_distance(after))
        if departure is None and math.dist(after[2][:2], TRIANGLE[2][:2]) > 0.05:
            departure = clock[0]
        poses = after
        if motion.finished:
            break
    assert motion.finished, f'Turn stalled at {poses}; progress={motion.progress}'
    assert departure is not None and departure < 5.0
    assert minimum_spacing > 0.37
    print(f'Rolling turn {route}: leader departed at {departure:.1f} s, complete at {clock[0]:.1f} s; '
          f'minimum peer spacing {minimum_spacing:.3f} m.')


def test_rolling_reverse_on_my_maze_from_spawn():
    nominal, hard = maze_grid(), maze_grid(0.15)
    poses = ((4.0, -2.9, 1.57), (3.0, -2.9, 1.57), (3.5, -2.2, 1.57))
    router = coordinator(poses, nominal)
    route = router.route_between(poses[2], (3.5, -3.8), [])
    assert route is not None
    planner = TurnaroundPlanner(nominal.segment_free, nominal.clearance, router.route_between, (2, 1, 0))
    motion = planner.build(poses, route)
    assert isinstance(motion, RollingTurnMotion)
    clock, departure = [0.0], None
    initial = poses
    nodes = [controller(i, hard, clock) for i in range(3)]
    for k in range(250):
        clock[0] = k * 0.1
        rows = motion.rows(poses, 0.1, 8, 0.15)
        assert not motion.failure_reason
        after = step_team(nodes, poses, rows, clock[0])
        assert all(hard.segment_free(a, b) for a, b in zip(poses, after))
        assert all(peer_motion_free(poses[i], after[i], poses[j], after[j])
                   for i, j in ((0, 1), (0, 2), (1, 2)))
        if departure is None and math.dist(after[2][:2], initial[2][:2]) > 0.05:
            departure = clock[0]
        poses = after
        if motion.finished:
            break
    assert motion.finished, f'Maze reverse stalled at {poses}'
    assert departure is not None and departure < 5.0
    print(f'my_maze reverse: leader departed at {departure:.1f} s, complete at {clock[0]:.1f} s.')


def test_coordinator_recovers_before_attempting_the_blocked_connector():
    node = coordinator(STUCK_POSES, maze_grid())
    node.plan()
    assert node.recovering_peers and node.maneuver is not None
    assert not node.replan_required
    assert any('Peer separation recovery' in line for line in node.diagnostics)


def test_lidar_keeps_new_obstacles_without_duplicating_mapped_walls():
    raw = [False] * (24 * 24)
    raw[12 * 24 + 22] = True  # Occupied box starts at world (1.0, 0.0).
    grid = GridClearance(24, 24, 0.1, (-1.2, -1.2, 0.0), raw, 0.15)
    node = controller(0, grid, [2.0])
    node.own_pose = (0.0, 0.0, 0.0)
    node.peer_poses = [node.own_pose, (3.0, 3.0, 0.0), (-3.0, -3.0, 0.0)]
    node.state_time, node.last_scan_time = 2.0, -float('inf')
    scan = LaserScan()
    scan.header.stamp.sec = 2
    scan.angle_min, scan.angle_increment = 0.0, math.pi / 2
    scan.range_min, scan.range_max = 0.05, 5.0
    scan.ranges = [1.0, 0.5]
    node.scan_callback(scan)
    assert len(node.scan_points) == 1
    assert math.dist(node.scan_points[0][:2], (0.0, 0.5)) < 1e-6


def test_lidar_keeps_a_new_obstacle_just_outside_a_mapped_wall():
    raw = [False] * (24 * 24)
    raw[12 * 24 + 22] = True
    grid = GridClearance(24, 24, 0.1, (-1.2, -1.2, 0.0), raw, 0.15)
    node = controller(0, grid, [2.0])
    node.own_pose = (0.0, 0.0, 0.0)
    node.peer_poses = [node.own_pose, (3.0, 3.0, 0.0), (-3.0, -3.0, 0.0)]
    node.state_time, node.last_scan_time = 2.0, -float('inf')
    scan = LaserScan()
    scan.header.stamp.sec = 2
    scan.range_min, scan.range_max = 0.05, 5.0
    scan.ranges = [0.94]
    node.scan_callback(scan)
    assert len(node.scan_points) == 1
    assert math.dist(node.scan_points[0][:2], (0.94, 0.0)) < 1e-6
