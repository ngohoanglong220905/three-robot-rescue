#!/usr/bin/env python3
"""D* Lite, adaptive virtual structure and receding-horizon MPC for 3 robots."""

import bisect
import heapq
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
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .adaptive_structure import AdaptiveStructure, interpolate, swept_separation


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
            [3.5, -3.8, 1.57, 3.5, -3.0, 1.57, 3.5, -2.2, 1.57],
        )
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('unknown_is_obstacle', True)
        self.declare_parameter('inflation_radius', 0.18)
        # Spawn geometry places robot1 / robot3 0.8 m to either side of robot2.
        self.declare_parameter('formation_half_width', 0.8)
        self.declare_parameter('formation_follow_gap', 0.65)
        self.declare_parameter('require_gazebo_pose', True)
        self.declare_parameter('formation_speed', 0.16)
        self.declare_parameter('max_linear_speed', 0.28)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('mpc_horizon', 8)
        self.declare_parameter('mpc_dt', 0.15)

        self.robot_names = list(self.get_parameter('robot_names').value)
        spawn_values = list(self.get_parameter('spawn_poses').value)
        if len(self.robot_names) != 3 or len(spawn_values) != 9:
            raise ValueError('This node requires exactly three robot names and three (x,y,yaw) spawn poses')
        self.spawn_poses = [
            tuple(float(value) for value in spawn_values[i:i + 3])
            for i in range(0, 9, 3)
        ]
        self.require_gazebo_pose = bool(self.get_parameter('require_gazebo_pose').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.unknown_is_obstacle = bool(self.get_parameter('unknown_is_obstacle').value)
        self.inflation_radius = float(self.get_parameter('inflation_radius').value)
        self.nominal_width = float(self.get_parameter('formation_half_width').value)
        self.follow_gap = float(self.get_parameter('formation_follow_gap').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.mpc_horizon = max(2, int(self.get_parameter('mpc_horizon').value))
        self.mpc_dt = float(self.get_parameter('mpc_dt').value)

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
        self.world_pose_sub = self.create_subscription(
            TFMessage, '/gazebo/world_poses', self.world_pose_callback, 1)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_times = [-INF, -INF, -INF]
        self.pose_wall_times = [-INF, -INF, -INF]
        self.odometry = [None, None, None]
        self.structure = None
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
            'D* Lite + adaptive virtual structure + omni MPC ready; '
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
        # Robot2 anchors the structure even after the flank robots queue behind.
        return self.get_robot_pose(1)[:3]

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

        if self.map_geometry != geometry:
            self.dstar = None
            self.map_geometry = geometry
        elif self.dstar is not None:
            self.dstar.update_blocked(self.blocked)
        self.get_logger().info(
            f'Map received: {self.width} x {self.height}, '
            f'{self.resolution:.3f} m/cell.'
        )
        if self.goal_xy is not None:
            self.clear_paths()
            self.replan_required = True

    def make_inflated_grid(self, occupancy):
        count = self.width * self.height
        raw = [False] * count
        for index, value in enumerate(occupancy):
            raw[index] = (
                value < 0 and self.unknown_is_obstacle
            ) or value >= self.occupied_threshold

        radius = max(0, int(math.ceil(self.inflation_radius / self.resolution)))
        if radius == 0:
            return raw
        inflated = raw.copy()
        disk = [
            (dx, dy)
            for dy in range(-radius, radius + 1)
            for dx in range(-radius, radius + 1)
            if dx * dx + dy * dy <= radius * radius
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
        return self.is_free_cell(self.world_to_grid(x, y))

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
        self.clear_paths()
        self.replan_required = True
        self.plan()

    def plan(self):
        self.last_plan_attempt = self.now_seconds()
        if self.map_msg is None or self.goal_xy is None:
            return
        if not self.poses_ready():
            self.get_logger().warning(
                'Waiting for fresh positions of all three robots from /gazebo/world_poses.',
                throttle_duration_sec=3.0)
            return

        center_x, center_y, _ = self.team_center()
        start = self.world_to_grid(center_x, center_y)
        goal = self.world_to_grid(*self.goal_xy)
        # Do not silently move a blocked goal through a wall or overwrite a
        # snapped free endpoint with the original occupied coordinate.
        if not self.is_free_cell(start) or not self.is_free_cell(goal):
            self.clear_paths()
            self.get_logger().warning(
                f'D* Lite endpoint blocked/outside map: robot2=({center_x:.2f}, '
                f'{center_y:.2f}) cell={start}, goal={self.goal_xy} cell={goal}.',
                throttle_duration_sec=3.0)
            return
        if self.dstar is None or self.dstar.goal != goal:
            self.dstar = DStarLiteGrid(self.width, self.height,
                                      self.blocked.copy(), start, goal)
        else:
            self.dstar.update_blocked(self.blocked.copy())
            self.dstar.move_start(start)
        self.goal_cell = goal
        self.last_planned_start = start
        converged = self.dstar.compute_shortest_path()
        cells = self.dstar.extract_path() if converged else None
        if not cells:
            self.clear_paths()
            reason = 'search did not converge' if not converged else 'free cells are disconnected'
            self.get_logger().warning(f'D* Lite: {reason}; start={start}, goal={goal}.',
                                      throttle_duration_sec=3.0)
            return
        points = [self.grid_to_world(gx, gy) for gx, gy in cells]
        # Include exact endpoints as extra vertices so even one-cell routes
        # retain both the robot position and the user's goal.
        points = [(center_x, center_y)] + points + [self.goal_xy]
        points = self.simplify_path(points)
        points = self.resample_path(points, max(self.resolution, 0.04))
        headings, cumulative = self.path_geometry(points)
        actual = tuple(self.get_robot_pose(i)[:3] for i in range(3))
        structure = AdaptiveStructure(
            self.segment_is_free, self.route_between,
            width=self.nominal_width, gap=self.follow_gap,
            speed=float(self.get_parameter('formation_speed').value))
        if not structure.build(points, headings, cumulative, actual, self.sample_path):
            self.clear_paths()
            self.get_logger().warning(
                'D* Lite found a route, but no collision-free formation transition '
                'fits this route and the current robot positions.', throttle_duration_sec=3.0)
            return
        self.structure = structure
        self.phase = 0.0
        self.last_tick = self.now_seconds()
        self.replan_required = False
        self.publish_global_path(points, headings)
        self.publish_formation_paths()
        self.get_logger().info(
            f'D* Lite: {len(points)} poses; synchronized formation, '
            f'half-width {min(structure.widths):.2f}..{max(structure.widths):.2f} m.')

    def publish_empty_reference(self):
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.reference_pub.publish(msg)

    def clear_paths(self):
        self.structure = None
        self.robot_paths = [None, None, None]
        empty_path = Path()
        empty_path.header.frame_id = 'map'
        empty_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(empty_path)
        for publisher in self.path_pubs:
            publisher.publish(empty_path)
        self.publish_empty_reference()

    def route_between(self, start, goal, peers):
        # Used only to assemble the initial structure; stationary teammates
        # occupy disks of the required center-to-center clearance.
        saved = self.blocked
        occupied = saved.copy()
        radius = int(math.ceil(0.46 / self.resolution))
        for pose in peers:
            gx, gy = self.world_to_grid(*pose[:2])
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    nx, ny = gx + dx, gy + dy
                    if dx * dx + dy * dy <= radius * radius and 0 <= nx < self.width and 0 <= ny < self.height:
                        occupied[ny * self.width + nx] = True
        self.blocked = occupied
        try:
            first, last = self.world_to_grid(*start[:2]), self.world_to_grid(*goal[:2])
            if not self.is_free_cell(first) or not self.is_free_cell(last):
                return None
            planner = DStarLiteGrid(self.width, self.height, occupied, first, last)
            if not planner.compute_shortest_path():
                return None
            cells = planner.extract_path()
            if not cells:
                return None
            points = [start[:2]] + [self.grid_to_world(*cell) for cell in cells] + [goal[:2]]
            route = self.simplify_path(points)
            if not all(self.segment_is_free(a, b) for a, b in zip(route, route[1:])):
                return None
            return route
        finally:
            self.blocked = saved

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
        distance = math.hypot(b[0] - a[0], b[1] - a[1])
        steps = max(1, int(math.ceil(distance / max(self.resolution * 0.5, 0.01))))
        for i in range(steps + 1):
            t = i / steps
            if not self.is_free_world(
                a[0] + t * (b[0] - a[0]),
                a[1] + t * (b[1] - a[1]),
            ):
                return False
        return True

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
        # These displayed paths are exactly the geometry used by the shared
        # reference, including collision-checked initial assembly movements.
        stamp = self.get_clock().now().to_msg()
        for index in range(3):
            msg = Path()
            msg.header.frame_id = 'map'
            msg.header.stamp = stamp
            for row in self.structure.rows:
                x, y, yaw = row[index]
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                set_pose_yaw(pose.pose, yaw)
                msg.poses.append(pose)
            self.robot_paths[index] = msg
            self.path_pubs[index].publish(msg)

    def publish_reference(self, dt):
        if self.structure is None or not self.poses_ready():
            self.publish_empty_reference()
            return
        target = self.structure.sample(self.phase)
        error = max(math.hypot(p[0] - q[0], p[1] - q[1])
                    for p, q in zip(self.robot_state, target))
        # One slow robot slows the common virtual clock for *all* robots.
        rate = clamp((0.16 - error) / 0.10, 0.0, 1.0)
        self.phase = min(self.structure.times[-1], self.phase + dt * rate)
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        # Atomic time-major layout: [r1,r2,r3] at t=0, then t=dt, ... t=H*dt.
        # All MPC nodes receive the identical reference generation and phase.
        for step in range(self.mpc_horizon + 1):
            row = self.structure.sample(self.phase + rate * step * self.mpc_dt)
            for x, y, yaw in row:
                pose = Pose()
                pose.position.x, pose.position.y = x, y
                set_pose_yaw(pose, yaw)
                msg.poses.append(pose)
        self.reference_pub.publish(msg)

    def update_dynamic_layer(self):
        now = self.now_seconds()
        if self.map_msg is None or now - self.last_dynamic_update < 0.5:
            return
        self.last_dynamic_update = now
        dynamic = set()
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
        if new_blocked != self.blocked:
            self.blocked = new_blocked
            if self.dstar is not None:
                self.dstar.update_blocked(new_blocked.copy())
            # Preserve the current structure and common phase when a changed
            # cell does not obstruct any remaining swept formation movement.
            if self.structure is not None:
                first = max(0, bisect.bisect_left(self.structure.times, self.phase) - 1)
                rows = self.structure.rows[first:]
                if any(not self.structure.transition_free(a, b)
                       for a, b in zip(rows, rows[1:])):
                    self.clear_paths()
                    self.replan_required = self.goal_xy is not None

    def control_tick(self):
        now = self.now_seconds()
        dt = 0.0 if self.last_tick is None else clamp(now - self.last_tick, 0.0, 0.2)
        if self.last_tick is not None and now < self.last_tick:
            self.clear_paths()
            self.robot_state = [None, None, None]
            self.pose_times = [-INF, -INF, -INF]
            self.scan_points = [[], [], []]
            self.last_dynamic_update = self.last_marker_update = -INF
            self.last_scan_stamp = [-INF] * 3
            self.last_plan_attempt = -INF
            self.replan_required = self.goal_xy is not None
        self.last_tick = now
        self.update_dynamic_layer()
        if self.replan_required and now - self.last_plan_attempt > 1.0:
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
                label.text = name
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
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.mpc_horizon = max(2, int(self.get_parameter('mpc_horizon').value))
        self.mpc_dt = float(self.get_parameter('mpc_dt').value)
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
        if not self.blocked or self.resolution <= 0.0:
            return False
        gx, gy = self.world_to_grid(x, y)
        return (0 <= gx < self.width and 0 <= gy < self.height
                and not self.blocked[gy * self.width + gx])

    def segment_free(self, a, b):
        count = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1])
                                    / max(0.01, self.resolution * 0.5))))
        return all(self.is_free(a[0] + k / count * (b[0] - a[0]),
                                a[1] + k / count * (b[1] - a[1]))
                   for k in range(count + 1))

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
                # Continuous tracking candidates remove the old full-speed/zero
                # quantization that caused oscillation near formation slots.
                wx, wy = feed_x + 2.0 * (tx - px), feed_y + 2.0 * (ty - py)
                c, sn = math.cos(pyaw), math.sin(pyaw)
                bx, by = c * wx + sn * wy, -sn * wx + c * wy
                omega = clamp(2.0 * normalize_angle(tyaw - pyaw),
                              -self.max_angular_speed, self.max_angular_speed)
                proposals = [(0.0, 0.0, 0.0), previous]
                for scale, turn in ((0.5, 0.0), (1.0, 0.0), (1.3, 0.0),
                                    (1.0, -0.5), (1.0, 0.5)):
                    vx = scale * (math.cos(turn) * bx - math.sin(turn) * by)
                    vy = scale * (math.sin(turn) * bx + math.cos(turn) * by)
                    for spin in (omega, 0.0):
                        proposals.append((vx, vy, spin))
                for vx, vy, spin in proposals:
                    speed = math.hypot(vx, vy)
                    if speed > self.max_linear_speed:
                        vx, vy = vx * self.max_linear_speed / speed, vy * self.max_linear_speed / speed
                    horizon_dt = command_dt if step == 1 else self.mpc_dt
                    dvx, dvy = vx - previous[0], vy - previous[1]
                    dv = math.hypot(dvx, dvy)
                    if dv > 0.8 * horizon_dt:
                        vx = previous[0] + dvx * 0.8 * horizon_dt / dv
                        vy = previous[1] + dvy * 0.8 * horizon_dt / dv
                    spin = clamp(spin, previous[2] - 2.0 * horizon_dt,
                                 previous[2] + 2.0 * horizon_dt)
                    command = (vx, vy, spin)
                    mid_yaw = pyaw + 0.5 * spin * self.mpc_dt
                    nx = px + (vx * math.cos(mid_yaw) - vy * math.sin(mid_yaw)) * self.mpc_dt
                    ny = py + (vx * math.sin(mid_yaw) + vy * math.cos(mid_yaw)) * self.mpc_dt
                    nyaw = normalize_angle(pyaw + spin * self.mpc_dt)
                    before, after = (px, py), (nx, ny)
                    if not self.segment_free(before, after):
                        continue
                    if any(swept_separation(before, after, o, o) < self.inflation_radius for o in obstacles):
                        continue
                    if any(swept_separation(before, after, predictions[step - 1][i],
                                            predictions[step][i]) < 0.34 for i in peers):
                        continue
                    position_error = (nx - tx) ** 2 + (ny - ty) ** 2
                    formation_error = sum(
                        ((nx - predictions[step][i][0]) - (tx - references[step][i][0])) ** 2
                        + ((ny - predictions[step][i][1]) - (ty - references[step][i][1])) ** 2
                        for i in peers)
                    change = ((vx - previous[0]) ** 2 + (vy - previous[1]) ** 2
                              + 0.10 * (spin - previous[2]) ** 2)
                    cost = (old_cost + 12.0 * position_error + 2.0 * formation_error
                            + 0.12 * normalize_angle(tyaw - nyaw) ** 2
                            + 0.10 * change + 0.015 * (vx * vx + vy * vy + 0.1 * spin * spin))
                    expanded.append((nx, ny, nyaw, cost, command if first is None else first,
                                     command, trajectory + [(nx, ny, nyaw)]))
            if not expanded:
                self.publish_stop()
                return
            beam = heapq.nsmallest(12, expanded, key=lambda state: state[3])
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
