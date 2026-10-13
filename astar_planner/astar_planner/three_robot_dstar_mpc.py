#!/usr/bin/env python3
"""D* Lite, adaptive leader-first triangle/column references and distributed MPC."""

import bisect
import heapq
import json
import math
import time

import rclpy
from geometry_msgs.msg import (
    Point, PointStamped, Pose, PoseArray, PoseStamped, TransformStamped, Twist,
)
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .adaptive_structure import interpolate, swept_separation
from .moving_formation import MovingAdaptiveFormation
from .grid_clearance import GridClearance
from .leader_follower import ClearanceRecovery
from .turnaround import TurnaroundPlanner, leader_route_blockers
from .peer_safety import minimum_team_distance, peer_motion_free
from .separation_recovery import SeparationRecoveryPlanner
from .social_observation import ObservationSweep


INF = float('inf')


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def set_pose_yaw(pose, yaw):
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = math.sin(yaw * 0.5)
    pose.orientation.w = math.cos(yaw * 0.5)


class DStarLiteGrid:
    """Incremental eight-connected D* Lite search on an occupancy grid."""

    MOVES = (
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
    )

    def __init__(self, width, height, blocked, start, goal):
        self.width = width
        self.height = height
        self.blocked = blocked
        self.start = start
        self.last_start = start
        self.goal = goal
        self.km = 0.0
        self.g = {}
        self.rhs = {goal: 0.0}
        self.open_heap = []
        self.open_entries = {}
        self.sequence = 0
        self._push(goal)

    def _value(self, values, cell):
        return values.get(cell, INF)

    @staticmethod
    def _heuristic(a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)

    def _key(self, cell):
        best = min(self._value(self.g, cell), self._value(self.rhs, cell))
        return (
            best + self._heuristic(self.start, cell) + self.km,
            best,
        )

    def _push(self, cell):
        key = self._key(cell)
        self.sequence += 1
        entry = (key[0], key[1], self.sequence)
        self.open_entries[cell] = entry
        heapq.heappush(self.open_heap, (key[0], key[1], self.sequence, cell))

    def _top_key(self):
        while self.open_heap:
            k1, k2, sequence, cell = self.open_heap[0]
            if self.open_entries.get(cell) == (k1, k2, sequence):
                return k1, k2
            heapq.heappop(self.open_heap)
        return INF, INF

    def _pop(self):
        while self.open_heap:
            k1, k2, sequence, cell = heapq.heappop(self.open_heap)
            if self.open_entries.get(cell) == (k1, k2, sequence):
                del self.open_entries[cell]
                return (k1, k2), cell
        return (INF, INF), None

    def _inside(self, cell):
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def neighbors(self, cell):
        # D* predecessors include blocked vertices too. Edge costs, not graph
        # adjacency, determine traversability when an obstacle changes.
        x, y = cell
        for dx, dy, distance in self.MOVES:
            candidate = (x + dx, y + dy)
            if self._inside(candidate):
                yield candidate, distance

    @staticmethod
    def consistent(a, b):
        return a == b or abs(a - b) < 1e-9

    def _edge_cost(self, source, target, distance):
        if not self._inside(source) or not self._inside(target):
            return INF
        if self.blocked[source[1] * self.width + source[0]]:
            return INF
        if self.blocked[target[1] * self.width + target[0]]:
            return INF
        dx = target[0] - source[0]
        dy = target[1] - source[1]
        if dx and dy:
            if self.blocked[source[1] * self.width + target[0]]:
                return INF
            if self.blocked[target[1] * self.width + source[0]]:
                return INF
        return distance

    def update_vertex(self, cell):
        if cell != self.goal:
            best = INF
            for neighbor, distance in self.neighbors(cell):
                candidate = self._edge_cost(cell, neighbor, distance) + self._value(
                    self.g, neighbor
                )
                best = min(best, candidate)
            self.rhs[cell] = best

        self.open_entries.pop(cell, None)
        if not self.consistent(self._value(self.g, cell), self._value(self.rhs, cell)):
            self._push(cell)

    def move_start(self, new_start):
        if new_start != self.start:
            self.km += self._heuristic(self.last_start, new_start)
            self.last_start = new_start
            self.start = new_start

    def update_blocked(self, blocked):
        if len(blocked) != len(self.blocked):
            return False
        changed = [
            index for index, (old, new) in enumerate(zip(self.blocked, blocked))
            if old != new
        ]
        if not changed:
            return False

        self.blocked = blocked
        affected = set()
        for index in changed:
            x = index % self.width
            y = index // self.width
            affected.add((x, y))
            for dx, dy, _ in self.MOVES:
                cell = (x + dx, y + dy)
                if self._inside(cell):
                    affected.add(cell)
        for cell in affected:
            self.update_vertex(cell)
        return True

    def compute_shortest_path(self):
        limit = max(1000, self.width * self.height * 8)
        iterations = 0
        while (
            self._top_key() < self._key(self.start)
            or not self.consistent(self._value(self.rhs, self.start), self._value(self.g, self.start))
        ):
            old_key, current = self._pop()
            if current is None:
                break
            new_key = self._key(current)
            if old_key < new_key:
                self._push(current)
            elif self._value(self.g, current) > self._value(self.rhs, current):
                self.g[current] = self._value(self.rhs, current)
                for predecessor, _ in self.neighbors(current):
                    self.update_vertex(predecessor)
            else:
                self.g[current] = INF
                self.update_vertex(current)
                for predecessor, _ in self.neighbors(current):
                    self.update_vertex(predecessor)
            iterations += 1
            if iterations >= limit:
                break
        return iterations < limit and self.consistent(
            self._value(self.rhs, self.start), self._value(self.g, self.start))

    def extract_path(self):
        if not self._inside(self.start) or not self._inside(self.goal):
            return None
        if (self.blocked[self.start[1] * self.width + self.start[0]]
                or self.blocked[self.goal[1] * self.width + self.goal[0]]):
            return None
        if not math.isfinite(self._value(self.g, self.start)):
            return None
        path = [self.start]
        visited = {self.start}
        current = self.start
        limit = self.width * self.height
        while current != self.goal and len(path) <= limit:
            best_cell = None
            best_cost = INF
            for neighbor, distance in self.neighbors(current):
                candidate = self._edge_cost(current, neighbor, distance) + self._value(
                    self.g, neighbor
                )
                if candidate < best_cost:
                    best_cost = candidate
                    best_cell = neighbor
            if best_cell is None or not math.isfinite(best_cost) or best_cell in visited:
                return None
            path.append(best_cell)
            visited.add(best_cell)
            current = best_cell
        return path if current == self.goal else None


class ThreeRobotDStarMPC(Node):
    """Plan a shared route and control three omnidirectional robots."""

    def __init__(self):
        super().__init__('three_robot_dstar_mpc')

        self.declare_parameter('robot_names', ['robot1', 'robot2', 'robot3'])
        self.declare_parameter(
            'spawn_poses',
            [4.0, -2.9, 1.57, 3.0, -2.9, 1.57, 3.5, -2.2, 1.57],
        )
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('unknown_is_obstacle', True)
        self.declare_parameter('inflation_radius', 0.18)
        self.declare_parameter('robot_radius', 0.15)
        self.declare_parameter('leader_robot', 'robot3')
        self.declare_parameter('formation_follow_gap', 0.70)
        self.declare_parameter('adaptive_formation', True)
        self.declare_parameter('formation_triangle_half_width', 0.50)
        self.declare_parameter('formation_lookahead', 1.20)
        self.declare_parameter('formation_transition_speed', 0.22)
        self.declare_parameter('formation_follower_speed', 0.22)
        self.declare_parameter('formation_narrow_follower_speed', 0.12)
        self.declare_parameter('formation_open_hold', 0.60)
        self.declare_parameter('narrow_speed', 0.08)
        self.declare_parameter('require_gazebo_pose', True)
        self.declare_parameter('formation_speed', 0.16)
        self.declare_parameter('max_linear_speed', 0.28)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('mpc_horizon', 8)
        self.declare_parameter('mpc_dt', 0.15)
        self.declare_parameter('social_observation_enabled', False)
        self.declare_parameter('observation_angular_speed', 0.75)
        self.declare_parameter('observation_dwell', 0.8)
        self.declare_parameter('observation_analysis_time', 0.0)

        self.robot_names = list(self.get_parameter('robot_names').value)
        spawn_values = list(self.get_parameter('spawn_poses').value)
        if len(set(self.robot_names)) != 3 or len(self.robot_names) != 3 or len(spawn_values) != 9:
            raise ValueError('This node requires exactly three robot names and three (x,y,yaw) spawn poses')
        self.spawn_poses = [
            tuple(float(value) for value in spawn_values[i:i + 3])
            for i in range(0, 9, 3)
        ]
        self.require_gazebo_pose = bool(self.get_parameter('require_gazebo_pose').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.unknown_is_obstacle = bool(self.get_parameter('unknown_is_obstacle').value)
        self.inflation_radius = float(self.get_parameter('inflation_radius').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.leader_index = self.robot_names.index(str(self.get_parameter('leader_robot').value))
        followers = sorted((i for i in range(3) if i != self.leader_index),
                           key=lambda i: (round(math.dist(self.spawn_poses[i][:2],
                                                         self.spawn_poses[self.leader_index][:2]), 6), -i))
        self.convoy_order = (self.leader_index, *followers)
        self.follow_gap = float(self.get_parameter('formation_follow_gap').value)
        self.adaptive_formation = bool(self.get_parameter('adaptive_formation').value)
        self.triangle_half_width = float(self.get_parameter('formation_triangle_half_width').value)
        self.formation_lookahead = float(self.get_parameter('formation_lookahead').value)
        self.transition_speed = float(self.get_parameter('formation_transition_speed').value)
        self.follower_speed = float(self.get_parameter('formation_follower_speed').value)
        self.narrow_follower_speed = float(self.get_parameter('formation_narrow_follower_speed').value)
        self.formation_open_hold = float(self.get_parameter('formation_open_hold').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.mpc_horizon = max(2, int(self.get_parameter('mpc_horizon').value))
        self.mpc_dt = float(self.get_parameter('mpc_dt').value)
        self.observation_enabled = bool(self.get_parameter('social_observation_enabled').value)
        self.observation_speed = float(self.get_parameter('observation_angular_speed').value)
        self.observation_dwell = float(self.get_parameter('observation_dwell').value)
        self.observation_analysis_time = float(self.get_parameter('observation_analysis_time').value)
        if not 0.0 <= self.observation_analysis_time <= 35.0:
            raise ValueError('Require observation analysis time between 0 and 35 seconds')
        if not (0 < self.observation_speed <= self.max_angular_speed and self.observation_dwell >= 0.4):
            raise ValueError('Require observation speed within angular limit and dwell >= 0.4 s')
        self.observation = None
        self.observation_search_id = None
        self.observation_wall_deadline = INF
        self.last_observation = -INF
        self.observation_frame_time = self.observation_frame_wall = -INF
        if not 0.0 < self.robot_radius <= self.inflation_radius:
            raise ValueError('Require 0 < robot_radius <= inflation_radius')
        if self.follow_gap < max(0.40, 2.0 * self.robot_radius + 0.04):
            raise ValueError('formation_follow_gap is too small for inter-robot clearance')
        if not (0.0 < float(self.get_parameter('narrow_speed').value)
                <= float(self.get_parameter('formation_speed').value) <= self.max_linear_speed):
            raise ValueError('Require 0 < narrow_speed <= formation_speed <= max_linear_speed')
        if self.mpc_dt <= 0.0:
            raise ValueError('mpc_dt must be positive')
        if not 0.0 < self.transition_speed <= self.max_linear_speed or self.formation_open_hold < 0.0:
            raise ValueError('Require 0 < formation_transition_speed <= max_linear_speed and open hold >= 0')
        if not 0.0 < self.follower_speed <= self.max_linear_speed:
            raise ValueError('Require 0 < formation_follower_speed <= max_linear_speed')
        if not (float(self.get_parameter('narrow_speed').value)
                <= self.narrow_follower_speed <= self.max_linear_speed):
            raise ValueError('Require narrow_speed <= formation_narrow_follower_speed <= max_linear_speed')
        if self.triangle_half_width < 0.35 or self.formation_lookahead < 0.5:
            raise ValueError('Require triangle half-width >= 0.35 m and formation lookahead >= 0.5 m')

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10
        )
        self.clicked_point_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.clicked_point_callback, 10
        )

        self.path_pub = self.create_publisher(Path, '/dstar_lite/path', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/three_robot_dstar_mpc/markers', 10
        )
        self.team_state_pub = self.create_publisher(
            PoseArray, '/formation/robot_states', 10
        )
        self.path_pubs = [
            self.create_publisher(Path, f'/{name}/desired_path', 10)
            for name in self.robot_names
        ]

        self.reference_pub = self.create_publisher(PoseArray, '/formation/reference', 1)
        self.observation_pub = self.create_publisher(String, '/social_navigation/observation_status', 1)
        self.search_status_pub = self.create_publisher(String, '/social_navigation/search_status', 10)
        self.create_subscription(String, '/social_navigation/search', self.search_callback, 10)
        self.create_subscription(PointStamped, '/social_navigation/look_at', self.look_at_callback, 1)
        self.create_subscription(Header, '/leader/people/observation_stamp', self.observation_frame_callback, 1)
        self.create_service(Trigger, '/social_navigation/observe', self.observe_callback)
        self.world_pose_sub = self.create_subscription(
            TFMessage, '/gazebo/world_poses', self.world_pose_callback, 1)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_times = [-INF, -INF, -INF]
        self.pose_wall_times = [-INF, -INF, -INF]
        self.odometry = [None, None, None]
        self.structure = None
        self.convoy = None
        self.maneuver = None
        self.recovering_peers = False
        self.last_turn_attempt = -INF
        self.failed_turn_position = None
        self.failed_turn_peers = None
        self.last_convoy_display = -INF
        self.last_formation_mode = None
        self.phase = 0.0
        self.last_tick = None
        self.last_plan_attempt = -INF
        self.replan_required = False
        self.robot_paths = [None, None, None]
        self.robot_state = [None, None, None]
        self.gazebo_pose_seen = [False, False, False]
        self.trails = [[] for _ in self.robot_names]
        self.scan_points = [[] for _ in self.robot_names]
        self.last_scan_stamp = [0.0 for _ in self.robot_names]

        for index, robot_name in enumerate(self.robot_names):
            self.create_subscription(
                Odometry,
                f'/{robot_name}/odom',
                lambda msg, robot_index=index: self.odom_callback(msg, robot_index),
                10,
            )
            self.create_subscription(
                LaserScan,
                f'/{robot_name}/scan',
                lambda msg, robot_index=index: self.scan_callback(msg, robot_index),
                qos_profile_sensor_data,
            )

        self.map_msg = None
        self.width = 0
        self.height = 0
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_yaw = 0.0
        self.base_blocked = []
        self.blocked = []
        self.collision_grid = None
        self.hard_collision_grid = None
        self.dynamic_obstacles = []
        self.routing_peers = []
        self.assembly_route_cache = {}
        self.dstar = None
        self.map_geometry = None
        self.goal_xy = None
        self.goal_cell = None
        self.last_planned_start = None
        self.last_map_signature = None
        self.last_dynamic_update = 0.0
        self.last_marker_update = 0.0

        # All planning, reference and sensor timing uses the same ROS clock.
        self.create_timer(0.1, self.control_tick)
        self.get_logger().info(
            'D* Lite + leader/follower MPC ready; order: '
            + ' -> '.join(self.robot_names[i] for i in self.convoy_order) + '; '
            'waiting for /map and an RViz 2D Goal Pose.'
        )

    def get_robot_pose(self, index):
        state = self.robot_state[index]
        if state is not None:
            return state
        x, y, yaw = self.spawn_poses[index]
        return x, y, yaw, False

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def poses_ready(self):
        now = self.now_seconds()
        return all(
            self.robot_state[i] is not None
            and 0.0 <= now - self.pose_times[i] < 0.5
            and time.monotonic() - self.pose_wall_times[i] < 5.0
            and (self.gazebo_pose_seen[i] or not self.require_gazebo_pose)
            for i in range(3)
        )

    def team_center(self):
        return self.get_robot_pose(self.leader_index)[:3]

    def record_pose(self, index, x, y, yaw):
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            return
        self.robot_state[index] = (x, y, yaw, True)
        self.pose_times[index] = self.now_seconds()
        self.pose_wall_times[index] = time.monotonic()
        trail = self.trails[index]
        if not trail or math.hypot(x - trail[-1][0], y - trail[-1][1]) > 0.025:
            trail.append((x, y))
            del trail[:-400]

    def odom_callback(self, msg, index):
        self.odometry[index] = msg
        if not self.gazebo_pose_seen[index] and not self.require_gazebo_pose:
            # Kinematics integrates in the IMU's initial (spawn-relative) frame.
            sx, sy, syaw = self.spawn_poses[index]
            p = msg.pose.pose.position
            c, sn = math.cos(syaw), math.sin(syaw)
            self.record_pose(index, sx + c * p.x - sn * p.y,
                             sy + sn * p.x + c * p.y,
                             normalize_angle(syaw + quaternion_yaw(msg.pose.pose.orientation)))
        self.publish_map_odom(index)

    def world_pose_callback(self, msg):
        # SceneBroadcaster includes links as well: only exact top-level model
        # names are world poses. Never publish its entire unscoped stream to TF.
        for transform in msg.transforms:
            name = transform.child_frame_id
            if name not in self.robot_names:
                continue
            index = self.robot_names.index(name)
            p = transform.transform.translation
            yaw = quaternion_yaw(transform.transform.rotation)
            if not self.gazebo_pose_seen[index]:
                self.get_logger().info(f'{name}: using Gazebo world pose in map.')
                self.trails[index].clear()
            self.gazebo_pose_seen[index] = True
            self.record_pose(index, p.x, p.y, yaw)

    def publish_map_odom(self, index):
        odom = self.odometry[index]
        if odom is None or self.robot_state[index] is None:
            return
        if abs(self.now_seconds() - self.pose_times[index]) > 0.5:
            return
        x, y, yaw, _ = self.robot_state[index]
        local = odom.pose.pose
        rotation = normalize_angle(yaw - quaternion_yaw(local.orientation))
        c, sn = math.cos(rotation), math.sin(rotation)
        msg = TransformStamped()
        msg.header.stamp = odom.header.stamp
        msg.header.frame_id = 'map'
        msg.child_frame_id = f'{self.robot_names[index]}/odom'
        msg.transform.translation.x = x - c * local.position.x + sn * local.position.y
        msg.transform.translation.y = y - sn * local.position.x - c * local.position.y
        msg.transform.rotation.z = math.sin(rotation * 0.5)
        msg.transform.rotation.w = math.cos(rotation * 0.5)
        self.tf_broadcaster.sendTransform(msg)

    def scan_callback(self, msg, index):
        now = self.now_seconds()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not self.poses_ready() or abs(now - stamp) > 0.3:
            return
        if now - self.last_scan_stamp[index] < 0.12:
            return
        self.last_scan_stamp[index] = now
        x, y, yaw, _ = self.get_robot_pose(index)
        ranges = msg.ranges
        stride = max(1, len(ranges) // 90)
        points = []
        max_range = min(float(msg.range_max), 3.5)
        for beam in range(0, len(ranges), stride):
            distance = float(ranges[beam])
            if not math.isfinite(distance) or distance <= max(0.12, msg.range_min):
                continue
            if distance >= max_range:
                continue
            angle = yaw + msg.angle_min + beam * msg.angle_increment
            px, py = x + distance * math.cos(angle), y + distance * math.sin(angle)
            if any(math.hypot(px - p[0], py - p[1]) < 0.26 for p in self.robot_state):
                continue
            points.append((px, py))
        self.scan_points[index] = [(px, py, now + 0.65) for px, py in points]

    def map_callback(self, msg):
        info = msg.info
        geometry = (
            info.width,
            info.height,
            round(info.resolution, 9),
            round(info.origin.position.x, 6),
            round(info.origin.position.y, 6),
            round(quaternion_yaw(info.origin.orientation), 6),
        )
        signature = (
            *geometry,
            tuple(msg.data),
        )
        if signature == self.last_map_signature:
            return
        self.last_map_signature = signature
        self.map_msg = msg
        self.width = int(info.width)
        self.height = int(info.height)
        self.resolution = float(info.resolution)
        self.origin_x = float(info.origin.position.x)
        self.origin_y = float(info.origin.position.y)
        self.origin_yaw = quaternion_yaw(info.origin.orientation)
        self.base_blocked = self.make_inflated_grid(msg.data)
        self.blocked = self.base_blocked.copy()
        self.dynamic_obstacles = []
        self.assembly_route_cache = {}

        geometry_changed = self.map_geometry != geometry
        if geometry_changed:
            self.dstar = None
            self.map_geometry = geometry
        self.get_logger().info(
            f'Map received: {self.width} x {self.height}, '
            f'{self.resolution:.3f} m/cell.', throttle_duration_sec=5.0
        )
        if geometry_changed:
            if self.observation is not None:
                self.finish_observation('aborted: map coordinate geometry changed')
            self.clear_paths()
            self.replan_required = self.goal_xy is not None
        # Retain measured trail/progress when an unrelated social region changes.
        # Reapply live LiDAR returns before checking the route against this map.
        self.update_dynamic_layer(force=True)

    def make_inflated_grid(self, occupancy):
        count = self.width * self.height
        raw = [False] * count
        for index, value in enumerate(occupancy):
            raw[index] = (
                value < 0 and self.unknown_is_obstacle
            ) or value >= self.occupied_threshold

        self.collision_grid = GridClearance(
            self.width, self.height, self.resolution,
            (self.origin_x, self.origin_y, self.origin_yaw), raw, self.inflation_radius)
        self.hard_collision_grid = GridClearance(
            self.width, self.height, self.resolution,
            (self.origin_x, self.origin_y, self.origin_yaw), raw, self.robot_radius)

        if self.inflation_radius <= 0.0:
            return raw
        radius = int(math.ceil(self.inflation_radius / self.resolution + 0.5))
        inflated = raw.copy()
        # Measure from each candidate cell center to the occupied cell BOX,
        # matching GridClearance. A center-to-center disk alone missed diagonal
        # corner cells which later failed continuous footprint validation.
        disk = [
            (dx, dy)
            for dy in range(-radius, radius + 1)
            for dx in range(-radius, radius + 1)
            if ((max(0.0, abs(dx) - 0.5) * self.resolution) ** 2
                + (max(0.0, abs(dy) - 0.5) * self.resolution) ** 2
                < self.inflation_radius ** 2 - 1e-12)
        ]
        for index, is_blocked in enumerate(raw):
            if not is_blocked:
                continue
            x = index % self.width
            y = index // self.width
            for dx, dy in disk:
                nx = x + dx
                ny = y + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    inflated[ny * self.width + nx] = True
        for y in range(self.height):
            for x in range(self.width):
                if min(x + 0.5, y + 0.5, self.width - x - 0.5,
                       self.height - y - 0.5) * self.resolution < self.inflation_radius:
                    inflated[y * self.width + x] = True
        return inflated

    def world_to_grid(self, x, y):
        dx = x - self.origin_x
        dy = y - self.origin_y
        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        return (
            int(math.floor(local_x / self.resolution)),
            int(math.floor(local_y / self.resolution)),
        )

    def grid_to_world(self, gx, gy):
        local_x = (gx + 0.5) * self.resolution
        local_y = (gy + 0.5) * self.resolution
        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)
        return (
            self.origin_x + c * local_x - s * local_y,
            self.origin_y + s * local_x + c * local_y,
        )

    def is_free_cell(self, cell):
        gx, gy = cell
        if not (0 <= gx < self.width and 0 <= gy < self.height):
            return False
        return not self.blocked[gy * self.width + gx]

    def is_free_world(self, x, y):
        return (self.collision_grid is not None
                and self.collision_grid.point_free((x, y))
                and all(math.hypot(x - ox, y - oy) >= self.inflation_radius
                        for ox, oy in self.dynamic_obstacles)
                and all(math.hypot(x - p[0], y - p[1]) >= 0.38
                        for p in self.routing_peers))

    def peer_connector_free(self, point, center):
        """Leave a peer's preferred buffer without getting closer to that peer.

        MPC prefers 0.34 m, while routing prefers 0.38 m. A measured
        endpoint inside the preferred buffer is legal if its entire connector
        preserves its current peer distance and reaches the normal search grid.
        """
        if (self.collision_grid is None or not self.collision_grid.segment_free(point, center)
                or any(swept_separation(point, center, p, p) < self.inflation_radius
                       for p in self.dynamic_obstacles)):
            return False
        for peer in self.routing_peers:
            if not peer_motion_free(point, center, peer, peer, preferred=0.38,
                                    physical=2.0 * self.robot_radius + 0.02):
                return False
        return True

    def connect_to_grid(self, point, maximum_distance=0.30, measured_endpoint=False):
        """Join a continuous free pose to the conservative D* grid.

        Inflation rounds to whole cells for search. A physically valid pose can
        lie in one of those boundary cells. Require full footprint clearance on
        the connector; never snap a pose through a wall or discard the connector.
        """
        connector_free = self.peer_connector_free if measured_endpoint else self.segment_is_free
        if not connector_free(point, point):
            return None
        gx, gy = self.world_to_grid(*point[:2])
        radius = int(math.ceil(maximum_distance / self.resolution))
        candidates = []
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                cell = (gx + dx, gy + dy)
                if not self.is_free_cell(cell):
                    continue
                center = self.grid_to_world(*cell)
                if not self.is_free_world(*center):
                    continue
                distance = math.hypot(point[0] - center[0], point[1] - center[1])
                if distance <= maximum_distance:
                    candidates.append((distance, cell, center))
        for _, cell, center in sorted(candidates):
            if connector_free(point, center):
                return cell
        return None

    def path_with_connectors(self, start, cells, goal, measured_goal=False):
        """Keep endpoint escape connectors separate from normal route smoothing."""
        centers = self.simplify_path([self.grid_to_world(*cell) for cell in cells])
        if not centers or not self.peer_connector_free(start, centers[0]):
            return None
        goal_free = self.peer_connector_free if measured_goal else self.segment_is_free
        if not goal_free(goal, centers[-1]):
            return None
        if not all(self.segment_is_free(a, b) for a, b in zip(centers, centers[1:])):
            return None
        points = [tuple(start[:2]), *centers, tuple(goal[:2])]
        # Remove tiny cell-centre turns at departure/arrival when a checked
        # direct segment fits. Keep measured escape connectors separate when
        # an endpoint lies inside a preferred peer buffer.
        if self.segment_is_free(start, start) and self.segment_is_free(goal, goal):
            return self.simplify_path(points)
        return points

    def connector_diagnostic(self, label, point, measured_endpoint=False):
        near = [(self.robot_names[i], math.dist(point[:2], self.get_robot_pose(i)[:2]))
                for i in self.convoy_order[1:]]
        distances = ', '.join(f'{name}={distance:.3f} m' for name, distance in near)
        return (f'{label} ({point[0]:.3f}, {point[1]:.3f}) has no safe grid connector; '
                f'obstacle clearance={self.clearance_world(point):.3f} m; peers: {distances}. '
                + ('Measured start needs a connector that does not reduce peer clearance.'
                   if measured_endpoint else 'Choose a goal clear of walls and current teammates.'))

    def goal_callback(self, msg):
        if msg.header.frame_id not in ('', 'map'):
            self.get_logger().error('Goal must be in map coordinates; goal rejected.')
            return
        self.set_goal(float(msg.pose.position.x), float(msg.pose.position.y))

    def clicked_point_callback(self, msg):
        if msg.header.frame_id not in ('', 'map'):
            self.get_logger().error('Clicked point must be in map coordinates.')
            return
        self.set_goal(float(msg.point.x), float(msg.point.y))

    def set_goal(self, x, y):
        if not math.isfinite(x) or not math.isfinite(y):
            return
        self.goal_xy = (x, y)
        self.failed_turn_position = None
        self.failed_turn_peers = None
        if self.observation is not None:
            self.finish_observation('cancelled: new goal')
        self.clear_paths()
        self.replan_required = True
        self.plan()

    def start_observation(self, point=None, search_kind=None, search_id=None):
        now = self.now_seconds()
        priority_screen = search_kind == 'find_screen'
        previous = self.observation
        previous_wall_deadline = self.observation_wall_deadline
        if not self.observation_enabled:
            return False, 'Observation is disabled in this launch.'
        if previous is not None and (not priority_screen or previous.kind == 'find_screen'):
            return False, 'An observation sweep is already active.'
        if not priority_screen and now - self.last_observation < 5.0:
            return False, 'Waiting for the observation cooldown.'
        if not self.poses_ready() or self.map_msg is None:
            return False, 'Waiting for fresh robot poses and the planning map.'
        if (not 0 <= now - self.observation_frame_time < 1.0
                or time.monotonic() - self.observation_frame_wall > 2.0):
            return False, 'Waiting for fresh processed camera frames.'
        actual = tuple(self.get_robot_pose(i)[:3] for i in range(3))
        if (self.structure is not None or self.maneuver is not None or self.recovering_peers
                or minimum_team_distance(actual) < 0.40
                or not all(self.is_free_world(*p[:2]) for p in actual)):
            return False, 'Recovery / obstacle clearance takes priority over observation.'
        bearing = None if point is None else math.atan2(
            point[1] - actual[self.leader_index][1], point[0] - actual[self.leader_index][0])
        if previous is not None:
            self.finish_observation('cancelled: prioritizing measured viewing direction')
        self.clear_paths()
        self.observation = ObservationSweep(actual, self.leader_index, now,
                                            self.observation_speed, self.observation_dwell, bearing,
                                            self.observation_analysis_time, search_kind)
        self.observation_wall_deadline = (time.monotonic() + ObservationSweep.AUTOMATIC_TIMEOUT
                                          if self.observation.kind != 'full' else INF)
        if previous is not None and previous.kind != 'full':
            self.observation.deadline = min(self.observation.deadline, previous.deadline)
            self.observation_wall_deadline = min(self.observation_wall_deadline, previous_wall_deadline)
        self.observation_search_id = search_id
        self.last_observation = now
        self.observation_pub.publish(String(data='scanning: ' + self.observation.kind))
        self.get_logger().info('Leader observing; followers hold their measured positions.')
        return True, 'Observation started; leader rotates through MPC, followers hold position.'

    def search_status(self, ident, state, reason):
        self.search_status_pub.publish(String(data=json.dumps(dict(
            version=1, id=ident, state=state, reason=reason))))

    def search_callback(self, msg):
        """Correlated, timestamped search requests; all motion remains under MPC."""
        try:
            data = json.loads(msg.data)
            ident, command = data['id'], data['command']
            if (data.get('version') != 1 or data.get('frame_id') != 'map'
                    or not isinstance(ident, str) or not 1 <= len(ident) <= 100
                    or not 0 <= self.now_seconds() - float(data['stamp']) < 1.0):
                return
            if command in ('start', 'focus'):
                point = tuple(float(v) for v in data['point'])
                if len(point) != 2 or not all(math.isfinite(v) for v in point):
                    return
            if command == 'start':
                kind = data['kind']
                if kind not in ('inspect_person', 'find_viewers', 'find_screen'):
                    return
                if self.observation is not None and self.observation_search_id == ident:
                    self.search_status(ident, 'scanning', 'Search is already active.')
                    return
                accepted, reason = self.start_observation(point, kind, ident)
                self.search_status(ident, 'scanning' if accepted else 'rejected', reason)
            elif self.observation is not None and self.observation_search_id == ident:
                if command == 'focus':
                    pose = self.get_robot_pose(self.leader_index)
                    accepted = self.observation.focus_on(math.atan2(point[1] - pose[1], point[0] - pose[0]))
                    self.search_status(ident, 'focusing' if accepted else 'focus_rejected',
                                       'Hold a measured person view within the original search deadline.')
                elif command == 'complete':
                    self.finish_observation('completed: social evidence acquired')
                elif command == 'cancel':
                    self.finish_observation('cancelled: social search reset')
        except (ValueError, KeyError, TypeError, OverflowError, AttributeError):
            return

    def look_at_callback(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        point = (msg.point.x, msg.point.y)
        if (msg.header.frame_id != 'map' or not all(math.isfinite(v) for v in point)
                or not 0 <= self.now_seconds() - stamp < 1.0):
            return
        accepted, reason = self.start_observation(point)
        if not accepted:
            self.observation_pub.publish(String(data='rejected: ' + reason))

    def observation_frame_callback(self, msg):
        stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        if msg.frame_id == 'map' and 0 <= self.now_seconds() - stamp < 1.0:
            self.observation_frame_time = stamp
            self.observation_frame_wall = time.monotonic()

    def observe_callback(self, request, response):
        response.success, response.message = self.start_observation()
        return response

    def finish_observation(self, reason):
        if self.observation_search_id is not None:
            self.search_status(self.observation_search_id, reason.split(':')[0], reason)
        self.observation_search_id = None
        self.observation = None
        self.observation_wall_deadline = INF
        self.clear_paths()
        self.replan_required = self.goal_xy is not None
        self.last_plan_attempt = -INF
        self.observation_pub.publish(String(data=reason))
        self.get_logger().info('Leader observation ' + reason + '; refreshing route from measured poses.')

    def turn_space_changed(self, actual):
        return (self.failed_turn_position is None
                or math.dist(actual[self.leader_index][:2], self.failed_turn_position) > 0.12
                or (getattr(self, 'failed_turn_peers', None) is not None
                    and any(math.dist(actual[i][:2], self.failed_turn_peers[i][:2]) > 0.12
                            for i in self.convoy_order[1:])))

    def clearance_world(self, point):
        clearance = self.hard_collision_grid.clearance(point)
        for obstacle in self.dynamic_obstacles:
            clearance = min(clearance, math.dist(point[:2], obstacle))
        return clearance

    def hard_segment_is_free(self, a, b):
        return (self.hard_collision_grid is not None
                and self.hard_collision_grid.segment_free(a, b)
                and all(swept_separation(a, b, p, p) >= self.robot_radius
                        for p in self.dynamic_obstacles))

    def start_peer_recovery(self, actual):
        planner = SeparationRecoveryPlanner(
            self.segment_is_free, self.clearance_world, self.convoy_order,
            physical=2.0 * self.robot_radius + 0.02, obstacle_margin=self.inflation_radius)
        motion = planner.build(actual)
        if motion is None:
            self.get_logger().warning(planner.failure_reason + '; team held.',
                                      throttle_duration_sec=3.0)
            return False
        self.maneuver = motion
        self.structure = self.convoy = None
        self.recovering_peers = True
        self.replan_required = False
        self.last_tick = self.now_seconds()
        self.publish_formation_paths()
        self.get_logger().info(
            f'Peer separation recovery: spacing {minimum_team_distance(actual):.3f} m; '
            'moving one robot at 0.05 m/s toward 0.46 m clearance.')
        return True

    def start_clearance_recovery(self, actual):
        rows = [actual]
        current = list(actual)
        for index in self.convoy_order:
            start = current[index]
            if self.is_free_world(*start[:2]):
                continue
            initial_clearance = self.clearance_world(start)
            if initial_clearance < self.robot_radius:
                self.get_logger().warning(
                    f'{self.robot_names[index]} clearance {initial_clearance:.3f} m is below '
                    f'the {self.robot_radius:.2f} m footprint limit; motion held.',
                    throttle_duration_sec=3.0)
                return False
            gx, gy = self.world_to_grid(*start[:2])
            radius = int(math.ceil(0.40 / self.resolution))
            candidates = []
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    cell = (gx + dx, gy + dy)
                    if not self.is_free_cell(cell):
                        continue
                    point = self.grid_to_world(*cell)
                    distance = math.dist(start[:2], point)
                    clearance = self.clearance_world(point)
                    if not 0.01 <= distance <= 0.40 or clearance < self.inflation_radius + 0.015:
                        continue
                    if not self.hard_segment_is_free(start, point):
                        continue
                    if (self.hard_collision_grid.segment_clearance(
                            start, point, initial_clearance) < initial_clearance - 1e-9
                            or any(swept_separation(start, point, p, p) < initial_clearance - 1e-9
                                   for p in self.dynamic_obstacles)):
                        continue
                    peers = [p for j, p in enumerate(current) if j != index]
                    if any(not peer_motion_free(start, point, peer, peer,
                                                physical=2.0 * self.robot_radius + 0.02)
                           for peer in peers):
                        continue
                    # Recovery may leave the preferred buffer, but must never
                    # approach an obstacle more closely than the measured start.
                    samples = max(2, int(math.ceil(distance / 0.01)))
                    previous = initial_clearance
                    valid = True
                    for k in range(1, samples + 1):
                        position = (start[0] + k / samples * (point[0] - start[0]),
                                    start[1] + k / samples * (point[1] - start[1]))
                        clearance_at_step = self.clearance_world(position)
                        if clearance_at_step + 1e-5 < previous:
                            valid = False
                            break
                        previous = clearance_at_step
                    if valid:
                        # Prefer enough margin to tolerate measured tracking error.
                        candidates.append((distance + 2.0 * max(0.0, 0.24 - clearance), point))
            if not candidates:
                self.get_logger().warning(
                    f'{self.robot_names[index]}: no short escape with increasing clearance.',
                    throttle_duration_sec=3.0)
                return False
            _, point = min(candidates)
            current[index] = (point[0], point[1], start[2])
            rows.append(tuple(current))
        if len(rows) < 2:
            return False
        self.structure = ClearanceRecovery(rows, self.hard_segment_is_free)
        self.convoy = None
        self.maneuver = None
        self.recovering_peers = False
        self.phase = 0.0
        self.last_tick = self.now_seconds()
        self.replan_required = False
        self.publish_formation_paths()
        self.get_logger().info('Clearance recovery: moving one robot away from the wall at 0.05 m/s.')
        return True

    def mask_peers(self, peers, radius=0.40):
        occupied = self.blocked.copy()
        cells = int(math.ceil(radius / self.resolution)) + 1
        for pose in peers:
            gx, gy = self.world_to_grid(*pose[:2])
            for dy in range(-cells, cells + 1):
                for dx in range(-cells, cells + 1):
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        # Peer poses need not coincide with cell centers.
                        # Rasterize about the measured position, not its cell.
                        if math.dist(self.grid_to_world(nx, ny), pose[:2]) < radius:
                            occupied[ny * self.width + nx] = True
        return occupied

    def plan(self):
        self.last_plan_attempt = self.now_seconds()
        self.assembly_route_cache = {}
        if self.map_msg is None or self.goal_xy is None:
            return
        if not self.poses_ready():
            self.get_logger().warning('Waiting for fresh measured poses of all three robots.',
                                      throttle_duration_sec=3.0)
            return
        if not self.is_free_world(*self.goal_xy):
            self.get_logger().warning('Goal lacks obstacle clearance; choose a free goal.',
                                      throttle_duration_sec=3.0)
            return
        actual = tuple(self.get_robot_pose(i)[:3] for i in range(3))
        if any(not self.is_free_world(*pose[:2]) for pose in actual):
            self.start_clearance_recovery(actual)
            return
        if minimum_team_distance(actual) < 0.38:
            self.start_peer_recovery(actual)
            return

        # Keep the incremental D* graph obstacle-only. First find the leader's
        # route; teammates become routing obstacles only if yielding fails.
        saved, saved_peers = self.blocked, self.routing_peers
        self.routing_peers = []
        try:
            start = self.connect_to_grid(actual[self.leader_index], measured_endpoint=True)
            goal = self.connect_to_grid(self.goal_xy)
            if start is None:
                self.get_logger().warning(self.connector_diagnostic(
                    'Leader start', actual[self.leader_index], measured_endpoint=True),
                    throttle_duration_sec=3.0)
                return
            if goal is None:
                self.get_logger().warning(self.connector_diagnostic('Goal', self.goal_xy),
                                          throttle_duration_sec=3.0)
                return
            if self.dstar is None or self.dstar.goal != goal:
                self.dstar = DStarLiteGrid(self.width, self.height, self.blocked.copy(), start, goal)
            else:
                self.dstar.update_blocked(self.blocked.copy())
                self.dstar.move_start(start)
            converged = self.dstar.compute_shortest_path()
            cells = self.dstar.extract_path() if converged else None
            if not cells:
                self.get_logger().warning('D* Lite: no route for the leader with current obstacles.',
                                          throttle_duration_sec=3.0)
                return
            points = self.path_with_connectors(actual[self.leader_index], cells, self.goal_xy)
            if points is None:
                self.get_logger().warning('Leader route failed swept-footprint clearance.',
                                          throttle_duration_sec=3.0)
                return
        finally:
            self.blocked, self.routing_peers = saved, saved_peers

        blockers = leader_route_blockers(actual, points, self.convoy_order)
        if blockers and self.turn_space_changed(actual):
            self.last_turn_attempt = self.now_seconds()
            planner = TurnaroundPlanner(
                self.segment_is_free, self.clearance_world, self.route_between,
                self.convoy_order, self.follow_gap, self.triangle_half_width,
                self.inflation_radius,
                speed=float(self.get_parameter('formation_speed').value),
                follower_speed=self.transition_speed,
                narrow_speed=float(self.get_parameter('narrow_speed').value),
                angular_speed=self.max_angular_speed)
            maneuver = planner.yield_followers(actual, points)
            if maneuver is not None:
                self.failed_turn_position = self.failed_turn_peers = None
                self.maneuver = maneuver
                self.recovering_peers = False
                self.convoy = self.structure = None
                self.last_tick = self.now_seconds()
                self.replan_required = False
                headings, _ = self.path_geometry(points)
                self.publish_global_path(points, headings)
                self.publish_formation_paths()
                self.get_logger().info('Follower yield ready: ' + maneuver.mode)
                return
            self.failed_turn_position = actual[self.leader_index][:2]
            self.failed_turn_peers = actual
            self.get_logger().info(
                'No reachable passing bay for the blocking follower; routing around teammates.',
                throttle_duration_sec=5.0)
        if blockers:
            peers = [actual[i] for i in self.convoy_order[1:]]
            points = self.route_between(actual[self.leader_index], self.goal_xy, peers)
            if points is None:
                self.get_logger().warning(
                    'Leader corridor is blocked and no safe yield/detour is available here.',
                    throttle_duration_sec=3.0)
                return

        # Seed the breadcrumb trail from the actual tail, through the middle
        # robot, to the leader. This needs no lateral formation assembly.
        leader, middle, tail = self.convoy_order
        trail = [actual[tail][:2]]
        progress = [0.0] * 3
        length = 0.0
        for source, target in ((tail, middle), (middle, leader)):
            other = [actual[i] for i in range(3) if i not in (source, target)]
            connector = self.route_between(actual[source], actual[target], other)
            if connector is None:
                self.get_logger().warning(
                    f'No safe trail connector: {self.robot_names[source]} -> {self.robot_names[target]}.',
                    throttle_duration_sec=3.0)
                return
            for point in connector[1:]:
                step = math.dist(trail[-1], point)
                if step > 1e-6:
                    length += step
                    trail.append(tuple(point))
            progress[target] = length
        points = self.resample_path(points, max(self.resolution, 0.04))
        arguments = (points, trail, progress, self.convoy_order,
                     self.hard_segment_is_free, self.clearance_world)
        options = dict(gap=self.follow_gap,
                       speed=float(self.get_parameter('formation_speed').value),
                       narrow_speed=float(self.get_parameter('narrow_speed').value))
        self.convoy = MovingAdaptiveFormation(
            *arguments, self.segment_is_free, self.route_between, **options,
            half_width=self.triangle_half_width, lookahead=self.formation_lookahead,
            planning_radius=self.inflation_radius, transition_speed=self.transition_speed,
            open_hold=self.formation_open_hold, follower_speed=self.follower_speed,
            narrow_follower_speed=self.narrow_follower_speed,
            allow_triangle=self.adaptive_formation)
        self.structure = None
        self.maneuver = None
        self.recovering_peers = False
        self.phase = 0.0
        self.last_tick = self.now_seconds()
        self.replan_required = False
        headings, _ = self.path_geometry(points)
        self.publish_global_path(points, headings)
        self.publish_formation_paths()
        self.get_logger().info(
            'Leader/follower ready: ' + ' -> '.join(self.robot_names[i] for i in self.convoy_order)
            + f'; gap {self.follow_gap:.2f} m; adaptive triangle/column={self.adaptive_formation}.')

    def publish_empty_reference(self):
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.reference_pub.publish(msg)

    def clear_paths(self):
        self.structure = None
        self.convoy = None
        self.maneuver = None
        self.recovering_peers = False
        self.last_formation_mode = None
        self.robot_paths = [None, None, None]
        empty_path = Path()
        empty_path.header.frame_id = 'map'
        empty_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(empty_path)
        for publisher in self.path_pubs:
            publisher.publish(empty_path)
        self.publish_empty_reference()

    def route_between(self, start, goal, peers):
        key = (tuple(start[:2]), tuple(goal[:2]), tuple(sorted(tuple(p[:2]) for p in peers)))
        if key not in self.assembly_route_cache:
            if len(self.assembly_route_cache) >= 128:
                self.assembly_route_cache.clear()
            self.assembly_route_cache[key] = self._route_between(start, goal, peers)
        return self.assembly_route_cache[key]

    def _route_between(self, start, goal, peers):
        # Most joins and passing-bay moves are short, clear segments. Avoid a
        # grid search and cell-centre detour when the continuous sweep is safe.
        if (self.segment_is_free(start, goal)
                and all(peer_motion_free(start, goal, peer, peer, preferred=0.38,
                                         physical=2.0 * self.robot_radius + 0.02)
                        for peer in peers)):
            return [tuple(start[:2]), tuple(goal[:2])]
        # Seed the initial trail around stationary teammates, with checked
        # connectors from their measured positions into the search grid.
        saved = self.blocked
        saved_peers = self.routing_peers
        occupied = self.mask_peers(peers)
        self.blocked = occupied
        self.routing_peers = peers
        try:
            first = self.connect_to_grid(start, measured_endpoint=True)
            last = self.connect_to_grid(goal, measured_endpoint=True)
            if first is None or last is None:
                return None
            planner = DStarLiteGrid(self.width, self.height, occupied, first, last)
            if not planner.compute_shortest_path():
                return None
            cells = planner.extract_path()
            if not cells:
                return None
            return self.path_with_connectors(start, cells, goal, measured_goal=True)
        finally:
            self.blocked = saved
            self.routing_peers = saved_peers

    def simplify_path(self, points):
        if len(points) <= 2:
            return points
        result = [points[0]]
        anchor = 0
        while anchor < len(points) - 1:
            furthest = anchor + 1
            for candidate in range(anchor + 2, len(points)):
                if not self.segment_is_free(points[anchor], points[candidate]):
                    break
                furthest = candidate
            result.append(points[furthest])
            anchor = furthest
        return result

    @staticmethod
    def resample_path(points, spacing):
        if len(points) < 2:
            return points
        result = [points[0]]
        for start, end in zip(points, points[1:]):
            distance = math.hypot(end[0] - start[0], end[1] - start[1])
            steps = max(1, int(math.ceil(distance / spacing)))
            for step in range(1, steps + 1):
                ratio = step / steps
                result.append((
                    start[0] + ratio * (end[0] - start[0]),
                    start[1] + ratio * (end[1] - start[1]),
                ))
        return result

    def segment_is_free(self, a, b):
        return (self.collision_grid is not None and self.collision_grid.segment_free(a, b)
                and all(swept_separation(a, b, p, p) >= self.inflation_radius
                        for p in self.dynamic_obstacles)
                and all(swept_separation(a, b, p, p) >= 0.38 for p in self.routing_peers))

    @staticmethod
    def path_geometry(points):
        headings = []
        cumulative = [0.0]
        for index, point in enumerate(points):
            previous = points[max(0, index - 1)]
            following = points[min(len(points) - 1, index + 1)]
            headings.append(math.atan2(following[1] - previous[1], following[0] - previous[0]))
            if index:
                cumulative.append(
                    cumulative[-1] + math.hypot(
                        point[0] - points[index - 1][0],
                        point[1] - points[index - 1][1],
                    )
                )
        return headings, cumulative

    @staticmethod
    def sample_path(points, headings, cumulative, distance):
        if distance <= 0.0:
            return points[0][0], points[0][1], headings[0]
        if distance >= cumulative[-1]:
            return points[-1][0], points[-1][1], headings[-1]
        index = bisect.bisect_left(cumulative, distance)
        index = max(1, index)
        span = cumulative[index] - cumulative[index - 1]
        ratio = 0.0 if span <= 1e-9 else (distance - cumulative[index - 1]) / span
        x = points[index - 1][0] + ratio * (points[index][0] - points[index - 1][0])
        y = points[index - 1][1] + ratio * (points[index][1] - points[index - 1][1])
        yaw_delta = normalize_angle(headings[index] - headings[index - 1])
        yaw = normalize_angle(headings[index - 1] + ratio * yaw_delta)
        return x, y, yaw

    def publish_global_path(self, points, headings):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for (x, y), yaw in zip(points, headings):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            set_pose_yaw(pose.pose, yaw)
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def publish_formation_paths(self):
        stamp = self.get_clock().now().to_msg()
        if self.maneuver is not None:
            paths = self.maneuver.display_paths()
        elif self.convoy is not None:
            paths = self.convoy.display_paths()
        elif self.structure is not None:
            paths = [[row[i] for row in self.structure.rows] for i in range(3)]
        else:
            return
        for index, points in enumerate(paths):
            msg = Path()
            msg.header.frame_id = 'map'
            msg.header.stamp = stamp
            for point in points:
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose.position.x, pose.pose.position.y = point[:2]
                set_pose_yaw(pose.pose, point[2] if len(point) > 2 else 0.0)
                msg.poses.append(pose)
            self.robot_paths[index] = msg
            self.path_pubs[index].publish(msg)
        self.last_convoy_display = self.now_seconds()

    def publish_reference(self, dt):
        if not self.poses_ready():
            if self.observation is not None:
                self.finish_observation('aborted: stale robot poses')
            self.publish_empty_reference()
            return
        actual = tuple(self.get_robot_pose(i)[:3] for i in range(3))
        if self.observation is not None:
            if time.monotonic() >= self.observation_wall_deadline:
                self.finish_observation('aborted: brief camera check reached its wall-time limit')
                return
            if (not 0 <= self.now_seconds() - self.observation_frame_time < 2.0
                    or time.monotonic() - self.observation_frame_wall > 3.0):
                self.finish_observation('aborted: camera inference is stale')
                return
            if (minimum_team_distance(actual) < 0.38
                    or not all(self.is_free_world(*p[:2]) for p in actual)):
                self.finish_observation('aborted: clearance changed')
                return
            rows = self.observation.rows(actual, self.now_seconds(), self.mpc_horizon, self.mpc_dt)
            if rows is None:
                self.finish_observation(self.observation.reason)
                return
            self.publish_reference_rows(rows)
            return
        if self.structure is None and (self.convoy is not None or self.maneuver is not None):
            # MPC tracking error can put a robot inside the planning buffer
            # while its physical footprint is still clear. A whole-team sweep
            # then rejects every row, even steps of an unobstructed leader.
            # Recover the measured pose now; waiting for a new goal deadlocks.
            outside_buffer = [i for i, pose in enumerate(actual)
                              if not self.is_free_world(*pose[:2])]
            if outside_buffer:
                details = ', '.join(
                    f'{self.robot_names[i]}={self.clearance_world(actual[i]):.3f} m'
                    for i in outside_buffer)
                self.get_logger().warning(
                    f'Measured obstacle clearance below {self.inflation_radius:.2f} m '
                    f'({details}); recovering wall clearance before resuming the route.',
                    throttle_duration_sec=3.0)
                self.clear_paths()
                self.replan_required = True
                self.last_plan_attempt = self.now_seconds()
                if not self.start_clearance_recovery(actual):
                    return
        if (not self.recovering_peers and self.structure is None
                and minimum_team_distance(actual) < 0.37
                and all(self.is_free_world(*pose[:2]) for pose in actual)):
            self.clear_paths()
            self.replan_required = True
            if not self.start_peer_recovery(actual):
                self.publish_empty_reference()
                return
        if self.maneuver is not None:
            rows = self.maneuver.rows(actual, dt, self.mpc_horizon, self.mpc_dt)
            if getattr(self.maneuver, 'failure_reason', ''):
                self.get_logger().warning(self.maneuver.failure_reason + '; replanning.',
                                          throttle_duration_sec=3.0)
                self.clear_paths()
                self.replan_required = True
                return
            if self.maneuver.finished:
                was_recovery = self.recovering_peers
                if not was_recovery:
                    leader, middle, tail = self.convoy_order
                    if (math.dist(actual[tail][:2], actual[leader][:2]) + 0.15
                            < math.dist(actual[middle][:2], actual[leader][:2])):
                        self.convoy_order = (leader, tail, middle)
                        self.get_logger().info(
                            'Follower order adapted after passing: ' +
                            ' -> '.join(self.robot_names[i] for i in self.convoy_order))
                self.clear_paths()
                self.replan_required = True
                self.last_plan_attempt = -INF
                self.get_logger().info(
                    'Peer spacing restored; replanning.' if was_recovery else
                    'Leader passed followers; resuming adaptive following.')
                return
            if self.maneuver.mode != self.last_formation_mode:
                self.last_formation_mode = self.maneuver.mode
                self.get_logger().info('Formation: ' + self.maneuver.mode)
            if self.now_seconds() - self.last_convoy_display > 0.5:
                self.publish_formation_paths()
        elif self.convoy is not None:
            if not self.convoy.observe(actual):
                self.get_logger().warning(self.convoy.failure_reason + '; replanning.',
                                          throttle_duration_sec=3.0)
                self.clear_paths()
                self.replan_required = True
                return
            if (self.convoy.state in ('column', 'triangle')
                    and self.now_seconds() - self.last_turn_attempt > 0.35
                    and self.turn_space_changed(actual)
                    and leader_route_blockers(actual, self.convoy.remaining_leader_route(), self.convoy_order)):
                self.last_turn_attempt = self.now_seconds()
                self.clear_paths()
                self.replan_required = True
                self.last_plan_attempt = -INF
                self.get_logger().info('Follower intersects leader route; preparing a local yield.')
                return
            rows = self.convoy.rows(actual, dt, self.mpc_horizon, self.mpc_dt)
            if self.convoy.failure_reason:
                self.get_logger().warning(self.convoy.failure_reason + '; replanning.',
                                          throttle_duration_sec=3.0)
                self.clear_paths()
                self.replan_required = True
                return
            if self.convoy.mode != self.last_formation_mode:
                self.last_formation_mode = self.convoy.mode
                self.get_logger().info('Formation: ' + self.convoy.mode)
            if self.now_seconds() - self.last_convoy_display > 0.5:
                self.publish_formation_paths()
        elif self.structure is not None:
            target = self.structure.sample(self.phase)
            error = max(math.dist(p[:2], q[:2]) for p, q in zip(actual, target))
            rate = clamp((0.10 - error) / 0.07, 0.0, 1.0)
            self.phase = min(self.structure.times[-1], self.phase + dt * rate)
            rows = [self.structure.sample(self.phase + rate * k * self.mpc_dt)
                    for k in range(self.mpc_horizon + 1)]
            if (self.phase >= self.structure.times[-1] and error < 0.025
                    and all(self.is_free_world(*p[:2]) for p in actual)):
                self.clear_paths()
                self.replan_required = True
                self.last_plan_attempt = -INF
                self.get_logger().info('Clearance restored; preparing leader/follower route.')
                return
        else:
            self.publish_empty_reference()
            return
        self.publish_reference_rows(rows)

    def publish_reference_rows(self, rows):
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for row in rows:
            for x, y, yaw in row:
                pose = Pose()
                pose.position.x, pose.position.y = x, y
                set_pose_yaw(pose, yaw)
                msg.poses.append(pose)
        self.reference_pub.publish(msg)

    def update_dynamic_layer(self, force=False):
        now = self.now_seconds()
        if self.map_msg is None or (not force and now - self.last_dynamic_update < 0.5):
            return
        self.last_dynamic_update = now
        dynamic = set()
        dynamic_points = []
        radius = max(0, int(math.ceil(self.inflation_radius / self.resolution)))
        disk = [
            (dx, dy)
            for dy in range(-radius, radius + 1)
            for dx in range(-radius, radius + 1)
            if dx * dx + dy * dy <= radius * radius
        ]
        for robot_points in self.scan_points:
            fresh = []
            for x, y, expiry in robot_points:
                if expiry <= now:
                    continue
                fresh.append((x, y, expiry))
                gx, gy = self.world_to_grid(x, y)
                if not (0 <= gx < self.width and 0 <= gy < self.height):
                    continue
                # Static wall returns are already inflated in the base map.
                if self.base_blocked[gy * self.width + gx]:
                    continue
                dynamic_points.append((x, y))
                for dx, dy in disk:
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        index = ny * self.width + nx
                        if not self.base_blocked[index]:
                            dynamic.add(index)
            robot_points[:] = fresh

        new_blocked = self.base_blocked.copy()
        for index in dynamic:
            new_blocked[index] = True
        geometry_changed = dynamic_points != self.dynamic_obstacles
        self.dynamic_obstacles = dynamic_points
        if force or new_blocked != self.blocked or geometry_changed:
            self.assembly_route_cache = {}
            self.blocked = new_blocked
            if self.dstar is not None:
                self.dstar.update_blocked(new_blocked.copy())
            self.invalidate_blocked_motion()

    def invalidate_blocked_motion(self):
        """Only changed obstacles intersecting remaining motion discard progress."""
        blocked = False
        for motion in (self.maneuver, self.convoy):
            if motion is not None and any(not self.hard_segment_is_free(a, b)
                                          for a, b in motion.remaining_segments()):
                blocked = True
        if self.convoy is not None and self.poses_ready():
            actual = tuple(self.get_robot_pose(i)[:3] for i in range(3))
            blocked |= any(not self.hard_segment_is_free(p, p) for p in actual)
            if self.convoy.last_rows:
                blocked |= any(not self.hard_segment_is_free(a, b)
                               for a, b in zip(actual, self.convoy.last_rows[0]))
        # Recovery segments have their own monotonic clearance checks.
        if self.structure is not None:
            first = max(0, bisect.bisect_left(self.structure.times, self.phase) - 1)
            rows = self.structure.rows[first:]
            blocked |= any(not self.structure.transition_free(a, b) for a, b in zip(rows, rows[1:]))
        if blocked:
            self.clear_paths()
            self.replan_required = self.goal_xy is not None

    def control_tick(self):
        now = self.now_seconds()
        dt = 0.0 if self.last_tick is None else clamp(now - self.last_tick, 0.0, 0.2)
        if self.last_tick is not None and now < self.last_tick:
            if self.observation is not None:
                self.finish_observation('aborted: clock reset')
            self.last_observation = -INF
            self.observation_frame_time = self.observation_frame_wall = -INF
            self.clear_paths()
            self.robot_state = [None, None, None]
            self.pose_times = [-INF, -INF, -INF]
            self.scan_points = [[], [], []]
            self.last_dynamic_update = self.last_marker_update = -INF
            self.last_scan_stamp = [-INF] * 3
            self.last_plan_attempt = -INF
            self.last_turn_attempt = -INF
            self.failed_turn_position = None
            self.failed_turn_peers = None
            self.replan_required = self.goal_xy is not None
        self.last_tick = now
        self.update_dynamic_layer()
        if self.observation is None and self.replan_required and now - self.last_plan_attempt > 1.0:
            self.plan()
        self.publish_team_states()
        self.publish_reference(dt)
        self.publish_markers()

    def publish_team_states(self):
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.poses_ready():
            # Stamp freshness at pose reception, not at marker publication.
            stamp_ns = int(min(self.pose_times) * 1e9)
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1000000000)
            for index in range(3):
                x, y, yaw, _ = self.get_robot_pose(index)
                pose = Pose()
                pose.position.x, pose.position.y = x, y
                set_pose_yaw(pose, yaw)
                msg.poses.append(pose)
        self.team_state_pub.publish(msg)

    def publish_markers(self):
        now = self.now_seconds()
        if now - self.last_marker_update < 0.08:
            return
        self.last_marker_update = now
        markers = MarkerArray()
        colors = (
            (0.95, 0.18, 0.16),
            (0.15, 0.65, 1.0),
            (0.20, 0.85, 0.35),
        )
        stamp = self.get_clock().now().to_msg()
        for index, name in enumerate(self.robot_names):
            x, y, yaw, localized = self.get_robot_pose(index)
            red, green, blue = colors[index]

            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.header.stamp = stamp
            arrow.ns = 'robot_positions'
            arrow.id = index
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = x
            arrow.pose.position.y = y
            arrow.pose.position.z = 0.08
            set_pose_yaw(arrow.pose, yaw)
            arrow.scale.x = 0.42
            arrow.scale.y = 0.14
            arrow.scale.z = 0.14
            arrow.color.r = red
            arrow.color.g = green
            arrow.color.b = blue
            arrow.color.a = 1.0
            arrow.lifetime.sec = 0
            markers.markers.append(arrow)

            label = Marker()
            label.header = arrow.header
            label.ns = 'robot_labels'
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.42
            label.pose.orientation.w = 1.0
            label.scale.z = 0.20
            label.color.r = red
            label.color.g = green
            label.color.b = blue
            label.color.a = 1.0
            if not localized:
                label.text = f'{name} (spawn estimate)'
            elif now - self.pose_times[index] >= 0.5:
                label.text = f'{name} (stale pose)'
            else:
                rank = self.convoy_order.index(index)
                label.text = f'{name} (leader)' if rank == 0 else f'{name} (follower {rank})'
                if rank == 0:
                    if self.observation is not None:
                        label.text += f'\nobserving ({self.observation.index + 1}/{len(self.observation.targets)})'
                    elif self.maneuver is not None:
                        label.text += '\n' + self.maneuver.mode
                    elif self.convoy is not None:
                        label.text += '\n' + self.convoy.mode
            markers.markers.append(label)

            trail = Marker()
            trail.header = arrow.header
            trail.ns = 'robot_trails'
            trail.id = index
            trail.type = Marker.LINE_STRIP
            trail.action = Marker.ADD
            trail.pose.orientation.w = 1.0
            trail.scale.x = 0.035
            trail.color.r = red
            trail.color.g = green
            trail.color.b = blue
            trail.color.a = 0.85
            for tx, ty in self.trails[index]:
                point = Point()
                point.x = tx
                point.y = ty
                point.z = 0.035
                trail.points.append(point)
            if len(trail.points) >= 2:
                markers.markers.append(trail)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ThreeRobotDStarMPC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_empty_reference()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class DistributedRobotMPC(Node):
    """Local finite-control-set MPC using the team's common timed reference."""

    def __init__(self):
        super().__init__('distributed_robot_mpc')
        for name, default in (
            ('robot_name', 'robot1'), ('robot_names', ['robot1', 'robot2', 'robot3']),
            ('motion_enabled', True), ('occupied_threshold', 50),
            ('unknown_is_obstacle', True), ('inflation_radius', 0.18),
            ('robot_radius', 0.15), ('preferred_clearance', 0.22),
            ('peer_preferred_distance', 0.46),
            ('max_linear_speed', 0.28), ('max_angular_speed', 1.0),
            ('mpc_horizon', 8), ('mpc_dt', 0.15),
        ):
            self.declare_parameter(name, default)
        self.robot_name = str(self.get_parameter('robot_name').value)
        self.robot_names = list(self.get_parameter('robot_names').value)
        self.robot_index = self.robot_names.index(self.robot_name)
        self.motion_enabled = bool(self.get_parameter('motion_enabled').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.unknown_is_obstacle = bool(self.get_parameter('unknown_is_obstacle').value)
        self.inflation_radius = float(self.get_parameter('inflation_radius').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.preferred_clearance = float(self.get_parameter('preferred_clearance').value)
        self.peer_preferred_distance = float(self.get_parameter('peer_preferred_distance').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.mpc_horizon = max(2, int(self.get_parameter('mpc_horizon').value))
        self.mpc_dt = float(self.get_parameter('mpc_dt').value)
        if len(self.robot_names) != 3 or len(set(self.robot_names)) != 3:
            raise ValueError('MPC requires exactly three distinct robot names')
        if not 0.0 < self.robot_radius <= self.inflation_radius <= self.preferred_clearance:
            raise ValueError('Require 0 < robot_radius <= inflation_radius <= preferred_clearance')
        if self.mpc_dt <= 0.0:
            raise ValueError('mpc_dt must be positive')
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.create_subscription(PoseArray, '/formation/robot_states', self.team_state_callback, 1)
        self.create_subscription(PoseArray, '/formation/reference', self.reference_callback, 1)
        self.create_subscription(LaserScan, f'/{self.robot_name}/scan', self.scan_callback,
                                 qos_profile_sensor_data)
        for index, name in enumerate(self.robot_names):
            if index != self.robot_index:
                self.create_subscription(
                    PoseArray, f'/{name}/mpc_prediction',
                    lambda msg, i=index: self.prediction_callback(msg, i), 1)
        self.cmd_pub = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 1)
        self.prediction_pub = self.create_publisher(
            PoseArray, f'/{self.robot_name}/mpc_prediction', 1)
        self.width = self.height = 0
        self.resolution = self.origin_x = self.origin_y = self.origin_yaw = 0.0
        self.blocked = []
        self.own_pose = None
        self.collision_grid = None
        self.hard_collision_grid = None
        self.peer_poses = [None] * 3
        self.peer_predictions = [None] * 3
        self.peer_prediction_times = [-INF] * 3
        self.state_time = self.reference_time = -INF
        self.state_wall = self.reference_wall = -INF
        self.reference = []
        self.scan_points = []
        self.last_scan_time = -INF
        self.last_command = (0.0, 0.0, 0.0)
        self.last_control_time = None
        self.create_timer(0.1, self.control_tick)
        # This watchdog also works when /clock stops or the coordinator exits.
        self.create_timer(0.2, self.watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'{self.robot_name}: MPC waiting for measured team poses and common reference.')

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def stamp_seconds(header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    @staticmethod
    def poses_from(msg):
        return [(p.position.x, p.position.y, quaternion_yaw(p.orientation)) for p in msg.poses]

    def map_callback(self, msg):
        info = msg.info
        self.width, self.height = int(info.width), int(info.height)
        self.resolution = float(info.resolution)
        self.origin_x, self.origin_y = info.origin.position.x, info.origin.position.y
        self.origin_yaw = quaternion_yaw(info.origin.orientation)
        self.blocked = ThreeRobotDStarMPC.make_inflated_grid(self, msg.data)

    def world_to_grid(self, x, y):
        return ThreeRobotDStarMPC.world_to_grid(self, x, y)

    def is_free(self, x, y):
        return self.hard_collision_grid is not None and self.hard_collision_grid.point_free((x, y))

    def segment_free(self, a, b, minimum_clearance=None):
        if self.hard_collision_grid is None:
            return False
        if minimum_clearance is None:
            minimum_clearance = max(self.robot_radius, min(
                self.inflation_radius,
                self.hard_collision_grid.clearance(a, self.inflation_radius)))
        return (self.hard_collision_grid.segment_clearance(a, b, minimum_clearance)
                >= minimum_clearance - 1e-9)

    def team_state_callback(self, msg):
        if msg.header.frame_id != 'map' or len(msg.poses) != 3:
            self.own_pose = None
            self.publish_stop()
            return
        self.peer_poses = self.poses_from(msg)
        self.own_pose = self.peer_poses[self.robot_index]
        self.state_time = self.stamp_seconds(msg.header)
        self.state_wall = time.monotonic()

    def reference_callback(self, msg):
        if msg.header.frame_id != 'map' or len(msg.poses) != 3 * (self.mpc_horizon + 1):
            self.reference = []
            self.publish_stop()
            return
        poses = self.poses_from(msg)
        self.reference = [poses[i:i + 3] for i in range(0, len(poses), 3)]
        self.reference_time = self.stamp_seconds(msg.header)
        self.reference_wall = time.monotonic()

    def prediction_callback(self, msg, index):
        if msg.header.frame_id == 'map' and len(msg.poses) == self.mpc_horizon + 1:
            self.peer_predictions[index] = self.poses_from(msg)
            self.peer_prediction_times[index] = self.stamp_seconds(msg.header)

    def scan_callback(self, msg):
        now = self.now_seconds()
        if self.own_pose is None or now - self.state_time > 0.3:
            return
        if now - self.last_scan_time < 0.1 or abs(now - self.stamp_seconds(msg.header)) > 0.3:
            return
        self.last_scan_time = now
        x, y, yaw = self.own_pose
        points = []
        for beam in range(0, len(msg.ranges), max(1, len(msg.ranges) // 120)):
            distance = float(msg.ranges[beam])
            if not math.isfinite(distance) or not max(0.12, msg.range_min) < distance < min(msg.range_max, 3.5):
                continue
            angle = yaw + msg.angle_min + beam * msg.angle_increment
            px, py = x + distance * math.cos(angle), y + distance * math.sin(angle)
            # Teammates are handled by the exchanged MPC predictions.
            if any(p is not None and math.hypot(px - p[0], py - p[1]) < 0.26
                   for p in self.peer_poses):
                continue
            # The map already checks known static surfaces with exact swept
            # geometry. Keeping every wall ray duplicated those constraints
            # thousands of times in the beam search. Preserve returns in free
            # map space, where they represent newly observed obstacles.
            if (self.hard_collision_grid is not None
                    and self.hard_collision_grid.clearance((px, py), 0.08) <= 1e-9):
                continue
            points.append((px, py, now + 0.6))
        self.scan_points = points

    def reference_at(self, seconds):
        value = clamp((seconds - self.reference_time) / self.mpc_dt,
                      0.0, len(self.reference) - 1.0)
        index = min(int(value), len(self.reference) - 2)
        return tuple(interpolate(a, b, value - index)
                     for a, b in zip(self.reference[index], self.reference[index + 1]))

    def peer_at(self, index, seconds, now):
        prediction = self.peer_predictions[index]
        stamp = self.peer_prediction_times[index]
        if prediction and 0.0 <= now - stamp < 0.4:
            value = clamp((seconds - stamp) / self.mpc_dt, 0.0, len(prediction) - 1.0)
            k = min(int(value), len(prediction) - 2)
            return interpolate(prediction[k], prediction[k + 1], value - k)
        # A silent peer is stationary, never assumed to keep moving at max speed.
        return self.peer_poses[index]

    def watchdog(self):
        if time.monotonic() - min(self.state_wall, self.reference_wall) > 2.0:
            self.publish_stop()

    def control_tick(self):
        now = self.now_seconds()
        if (not self.motion_enabled or self.own_pose is None or not self.blocked
                or not self.reference or not 0.0 <= now - self.state_time < 0.5
                or not 0.0 <= now - self.reference_time < 0.4):
            self.publish_stop()
            return
        if self.last_control_time is not None and now < self.last_control_time:
            self.reference = []
            self.peer_predictions = [None] * 3
            self.scan_points = []
            self.publish_stop()
            self.last_control_time = now
            return
        command_dt = (0.1 if self.last_control_time is None
                      else clamp(now - self.last_control_time, 0.02, 0.2))
        self.last_control_time = now
        x, y, yaw = self.own_pose
        obstacles = [(px, py) for px, py, expiry in self.scan_points
                     if expiry > now and math.hypot(px - x, py - y) < 0.8]
        peers = [i for i in range(3) if i != self.robot_index]
        close_context = (self.hard_collision_grid.clearance(self.own_pose, 0.32) < 0.32
                         or min(math.dist(self.own_pose[:2], self.peer_poses[i][:2]) for i in peers) < 0.50)
        references = [self.reference_at(now + k * self.mpc_dt)
                      for k in range(self.mpc_horizon + 1)]
        predictions = [{i: self.peer_at(i, now + k * self.mpc_dt, now) for i in peers}
                       for k in range(self.mpc_horizon + 1)]
        # State: x, y, yaw, accumulated cost, first command, last command, rollout.
        beam = [(x, y, yaw, 0.0, None, self.last_command, [self.own_pose])]
        for step in range(1, self.mpc_horizon + 1):
            tx, ty, tyaw = references[step][self.robot_index]
            previous_ref = references[step - 1][self.robot_index]
            feed_x, feed_y = (tx - previous_ref[0]) / self.mpc_dt, (ty - previous_ref[1]) / self.mpc_dt
            expanded = []
            for px, py, pyaw, old_cost, first, previous, trajectory in beam:
                # Use the coordinator's planning margin during normal motion.
                # If tracking/noisy observations already put the measured
                # robot inside that margin, permit only sweeps that preserve
                # or increase its clearance, never crossing the footprint.
                clearance_before = min(
                    [self.hard_collision_grid.clearance((px, py), self.inflation_radius)]
                    + [math.hypot(px - ox, py - oy) for ox, oy in obstacles])
                minimum_clearance = max(self.robot_radius, min(
                    self.inflation_radius, clearance_before))
                # Continuous tracking candidates remove the old full-speed/zero
                # quantization that caused oscillation near formation slots.
                wx, wy = feed_x + 2.0 * (tx - px), feed_y + 2.0 * (ty - py)
                c, sn = math.cos(pyaw), math.sin(pyaw)
                bx, by = c * wx + sn * wy, -sn * wx + c * wy
                omega = clamp(2.0 * normalize_angle(tyaw - pyaw),
                              -self.max_angular_speed, self.max_angular_speed)
                proposals = [(0.0, 0.0, 0.0), previous]
                adjustments = [(0.5, 0.0), (1.0, 0.0), (1.3, 0.0), (1.0, -0.5), (1.0, 0.5)]
                if close_context:
                    adjustments.extend(((0.7, -1.0), (0.7, 1.0)))
                for scale, turn in adjustments:
                    vx = scale * (math.cos(turn) * bx - math.sin(turn) * by)
                    vy = scale * (math.sin(turn) * bx + math.cos(turn) * by)
                    for spin in (omega, 0.0):
                        proposals.append((vx, vy, spin))
                seen_commands = set()
                for vx, vy, spin in proposals:
                    speed = math.hypot(vx, vy)
                    speed_limit = self.max_linear_speed
                    if step == 1 and any(math.dist(self.own_pose[:2], self.peer_poses[i][:2]) < 0.38
                                         for i in peers):
                        speed_limit = min(speed_limit, 0.06)
                    if speed > speed_limit:
                        vx, vy = vx * speed_limit / speed, vy * speed_limit / speed
                    horizon_dt = command_dt if step == 1 else self.mpc_dt
                    dvx, dvy = vx - previous[0], vy - previous[1]
                    dv = math.hypot(dvx, dvy)
                    if dv > 0.8 * horizon_dt:
                        vx = previous[0] + dvx * 0.8 * horizon_dt / dv
                        vy = previous[1] + dvy * 0.8 * horizon_dt / dv
                    spin = clamp(spin, previous[2] - 2.0 * horizon_dt,
                                 previous[2] + 2.0 * horizon_dt)
                    command = (vx, vy, spin)
                    command_key = tuple(round(v, 5) for v in command)
                    if command_key in seen_commands:
                        continue
                    seen_commands.add(command_key)
                    mid_yaw = pyaw + 0.5 * spin * self.mpc_dt
                    nx = px + (vx * math.cos(mid_yaw) - vy * math.sin(mid_yaw)) * self.mpc_dt
                    ny = py + (vx * math.sin(mid_yaw) + vy * math.cos(mid_yaw)) * self.mpc_dt
                    nyaw = normalize_angle(pyaw + spin * self.mpc_dt)
                    before, after = (px, py), (nx, ny)
                    if not self.segment_free(before, after, minimum_clearance):
                        continue
                    if any(swept_separation(before, after, o, o) < minimum_clearance - 1e-9
                           for o in obstacles):
                        continue
                    if any(not peer_motion_free(before, after, predictions[step - 1][i],
                                                predictions[step][i],
                                                physical=2.0 * self.robot_radius + 0.02) for i in peers):
                        continue
                    if step == 1 and any(not peer_motion_free(
                            before, after, self.peer_poses[i], self.peer_poses[i], preferred=0.38,
                            physical=2.0 * self.robot_radius + 0.02) for i in peers):
                        continue
                    position_error = (nx - tx) ** 2 + (ny - ty) ** 2
                    cross_error = -(nx - tx) * math.sin(tyaw) + (ny - ty) * math.cos(tyaw)
                    # Convoy spacing is enforced along the measured trail by
                    # the coordinator. A rigid XY formation cost at a bend
                    # would pull followers across the inside wall.
                    clearance = self.hard_collision_grid.clearance(after, self.preferred_clearance)
                    clearance = min([clearance] + [math.dist(after, o) for o in obstacles])
                    wall_error = max(0.0, self.preferred_clearance - clearance)
                    peer_error = sum(max(0.0, self.peer_preferred_distance
                                         - math.dist(after, predictions[step][i][:2])) ** 2 for i in peers)
                    change = ((vx - previous[0]) ** 2 + (vy - previous[1]) ** 2
                              + 0.10 * (spin - previous[2]) ** 2)
                    cost = (old_cost + 12.0 * position_error + 6.0 * cross_error ** 2
                            + 6.0 * wall_error ** 2 + 8.0 * peer_error
                            + 0.08 * normalize_angle(tyaw - nyaw) ** 2
                            + 0.10 * change + 0.015 * (vx * vx + vy * vy + 0.1 * spin * spin))
                    expanded.append((nx, ny, nyaw, cost, command if first is None else first,
                                     command, trajectory + [(nx, ny, nyaw)]))
            if not expanded:
                self.publish_stop()
                return
            unique_states = {}
            for state in expanded:
                key = tuple(round(v, 4) for v in (*state[:3], *state[4], *state[5]))
                if key not in unique_states or state[3] < unique_states[key][3]:
                    unique_states[key] = state
            beam = heapq.nsmallest(12, unique_states.values(), key=lambda state: state[3])
        tx, ty, _ = references[-1][self.robot_index]
        best = min(beam, key=lambda q: q[3] + 20.0 * ((q[0] - tx) ** 2 + (q[1] - ty) ** 2))
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = best[4]
        self.cmd_pub.publish(message)
        self.last_command = best[4]
        self.publish_prediction(best[6], now)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())
        self.last_command = (0.0, 0.0, 0.0)
        if self.own_pose is not None:
            self.publish_prediction([self.own_pose] * (self.mpc_horizon + 1), self.now_seconds())

    def publish_prediction(self, trajectory, stamp):
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(int(stamp * 1e9), 1000000000)
        for x, y, yaw in trajectory:
            pose = Pose()
            pose.position.x, pose.position.y = x, y
            set_pose_yaw(pose, yaw)
            msg.poses.append(pose)
        self.prediction_pub.publish(msg)


def robot_mpc_main(args=None):
    rclpy.init(args=args)
    node = DistributedRobotMPC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
