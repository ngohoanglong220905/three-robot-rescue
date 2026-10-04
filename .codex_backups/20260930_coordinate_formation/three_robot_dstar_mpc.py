#!/usr/bin/env python3
"""D* Lite, adaptive virtual structure and receding-horizon MPC for 3 robots."""

import bisect
import heapq
import math
import time

import rclpy
from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


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
        x, y = cell
        for dx, dy, distance in self.MOVES:
            candidate = (x + dx, y + dy)
            if not self._inside(candidate) or self.blocked[candidate[1] * self.width + candidate[0]]:
                continue
            if dx and dy:
                side_a = (y * self.width + x + dx)
                side_b = ((y + dy) * self.width + x)
                if self.blocked[side_a] or self.blocked[side_b]:
                    continue
            yield candidate, distance

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
        if self._value(self.g, cell) != self._value(self.rhs, cell):
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
            or self._value(self.rhs, self.start) != self._value(self.g, self.start)
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
        return iterations < limit

    def extract_path(self):
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
            [3.5, -3.5, 1.57, 3.5, -2.7, 1.57, 3.5, -2.1, 1.57],
        )
        self.declare_parameter('motion_enabled', False)
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('unknown_is_obstacle', True)
        self.declare_parameter('inflation_radius', 0.18)
        # Spawn geometry places robot1 / robot3 0.8 m to either side of robot2.
        self.declare_parameter('formation_half_width', 0.8)
        self.declare_parameter('formation_min_width', 0.35)
        self.declare_parameter('formation_follow_gap', 0.45)
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
        self.motion_enabled = bool(self.get_parameter('motion_enabled').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.unknown_is_obstacle = bool(self.get_parameter('unknown_is_obstacle').value)
        self.inflation_radius = float(self.get_parameter('inflation_radius').value)
        self.nominal_width = float(self.get_parameter('formation_half_width').value)
        self.minimum_width = float(self.get_parameter('formation_min_width').value)
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

        self.robot_paths = [None, None, None]
        self.robot_path_data = [None, None, None]
        self.cmd_pubs = []
        self.robot_state = [None, None, None]
        self.gazebo_pose_seen = [False, False, False]
        self.last_cmd = [(0.0, 0.0, 0.0) for _ in self.robot_names]
        self.path_indices = [0, 0, 0]
        self.trails = [[] for _ in self.robot_names]
        self.scan_points = [[] for _ in self.robot_names]
        self.last_scan_stamp = [0.0 for _ in self.robot_names]

        for index, robot_name in enumerate(self.robot_names):
            self.cmd_pubs.append(
                self.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
            )
            self.create_subscription(
                Odometry,
                f'/{robot_name}/odom',
                lambda msg, robot_index=index: self.odom_callback(msg, robot_index),
                10,
            )
            self.create_subscription(
                Pose,
                f'/{robot_name}/gazebo_pose',
                lambda msg, robot_index=index: self.gazebo_pose_callback(msg, robot_index),
                10,
            )
            self.create_subscription(
                LaserScan,
                f'/{robot_name}/scan',
                lambda msg, robot_index=index: self.scan_callback(msg, robot_index),
                10,
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
        self.last_path_log = 0.0
        self.formation_mode = 'not planned'
        self.formation_side_signs = None

        # Wall time is deliberate: position markers remain visible without /clock.
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

    def team_center(self):
        poses = [self.get_robot_pose(i) for i in range(3)]
        x = sum(pose[0] for pose in poses) / 3.0
        y = sum(pose[1] for pose in poses) / 3.0
        yaw = poses[0][2]
        return x, y, yaw

    def odom_callback(self, msg, index):
        # Keep odometry as a fallback while Gazebo's world pose is unavailable.
        if self.gazebo_pose_seen[index]:
            return
        # The kinematics node integrates displacement from a zeroed odom origin.
        # Anchor that local displacement at the matching Gazebo spawn in map/world.
        spawn_x, spawn_y, _ = self.spawn_poses[index]
        local_x = msg.pose.pose.position.x
        local_y = msg.pose.pose.position.y
        x = spawn_x + local_x
        y = spawn_y + local_y
        yaw = quaternion_yaw(msg.pose.pose.orientation)
        self.robot_state[index] = (x, y, yaw, True)

        trail = self.trails[index]
        if not trail or math.hypot(x - trail[-1][0], y - trail[-1][1]) > 0.025:
            trail.append((x, y))
            del trail[:-400]

    def gazebo_pose_callback(self, msg, index):
        x = float(msg.position.x)
        y = float(msg.position.y)
        yaw = quaternion_yaw(msg.orientation)
        self.gazebo_pose_seen[index] = True
        self.robot_state[index] = (x, y, yaw, True)

        trail = self.trails[index]
        if not trail or math.hypot(x - trail[-1][0], y - trail[-1][1]) > 0.025:
            trail.append((x, y))
            del trail[:-400]

    def scan_callback(self, msg, index):
        now = time.monotonic()
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
            points.append((x + distance * math.cos(angle), y + distance * math.sin(angle)))
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
            self.plan(force=True)

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

    def nearest_free(self, cell, maximum_radius=24):
        if self.is_free_cell(cell):
            return cell
        cx, cy = cell
        for radius in range(1, maximum_radius + 1):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    candidate = (cx + dx, cy + dy)
                    if self.is_free_cell(candidate):
                        return candidate
        return None

    def goal_callback(self, msg):
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        if msg.header.frame_id not in ('', 'map'):
            self.get_logger().warning(
                f'Goal frame is {msg.header.frame_id}; interpreting its coordinates in map.'
            )
        self.goal_xy = (x, y)
        self.formation_side_signs = None
        self.plan(force=True)

    def clicked_point_callback(self, msg):
        if msg.header.frame_id not in ('', 'map'):
            self.get_logger().warning(
                f'Clicked point frame is {msg.header.frame_id}; interpreting it in map.'
            )
        self.goal_xy = (float(msg.point.x), float(msg.point.y))
        self.formation_side_signs = None
        self.plan(force=True)

    def plan(self, force=False):
        if self.map_msg is None or self.goal_xy is None:
            return

        center_x, center_y, _ = self.team_center()
        start = self.nearest_free(self.world_to_grid(center_x, center_y))
        goal = self.nearest_free(self.world_to_grid(*self.goal_xy))
        if start is None:
            self.get_logger().error('Team start is outside the free map area; no path published.')
            self.clear_paths()
            return
        if goal is None:
            self.get_logger().error('Goal is outside the free map area or fully blocked.')
            self.clear_paths()
            return

        if self.dstar is None or self.dstar.goal != goal:
            self.dstar = DStarLiteGrid(
                self.width, self.height, self.blocked.copy(), start, goal
            )
            force = True
        else:
            self.dstar.update_blocked(self.blocked.copy())
            self.dstar.move_start(start)
        self.goal_cell = goal
        self.last_planned_start = start
        self.dstar.compute_shortest_path()
        cells = self.dstar.extract_path()
        if not cells:
            self.get_logger().warning('D* Lite could not connect the team to the goal.')
            self.clear_paths()
            return

        points = [self.grid_to_world(gx, gy) for gx, gy in cells]
        points[0] = (center_x, center_y)
        points[-1] = self.goal_xy
        points = self.simplify_path(points)
        points = self.resample_path(points, max(self.resolution, 0.03))
        headings, cumulative = self.path_geometry(points)
        self.publish_global_path(points, headings)
        self.publish_formation_paths(points, headings, cumulative)

        now = time.monotonic()
        if force or now - self.last_path_log > 2.0:
            self.get_logger().info(
                f'D* Lite path: {len(points)} poses; '
                f'adaptive formation: {self.formation_mode}.'
            )
            self.last_path_log = now

    def clear_paths(self):
        self.robot_paths = [None, None, None]
        self.robot_path_data = [None, None, None]
        empty_path = Path()
        empty_path.header.frame_id = 'map'
        empty_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(empty_path)
        for index, publisher in enumerate(self.path_pubs):
            publisher.publish(empty_path)
            self.publish_stop(index)

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

    def formation_slot(self, robot_index, width):
        """Return (path lag, side sign) for the configured spawn ordering.

        robot2 starts in the center. The two side signs are chosen from the
        robots' actual positions projected onto the first path normal. As width
        shrinks, robot2 leads and the two flank robots queue behind.
        """
        if robot_index == 1:
            return 0.0, 0.0

        if self.formation_side_signs is None:
            raise RuntimeError('Formation side assignment has not been initialized')
        compact = clamp(
            1.0 - width / max(self.nominal_width, 1e-6), 0.0, 1.0
        )
        if robot_index == 0:
            return self.follow_gap * compact, self.formation_side_signs[0]
        return 2.0 * self.follow_gap * compact, self.formation_side_signs[2]

    def assign_formation_sides(self, first_heading):
        center_x, center_y, _ = self.team_center()
        normal_x = -math.sin(first_heading)
        normal_y = math.cos(first_heading)
        signs = [None, 0.0, None]
        for index in (0, 2):
            x, y, _, _ = self.get_robot_pose(index)
            projection = (x - center_x) * normal_x + (y - center_y) * normal_y
            if abs(projection) > 0.05:
                signs[index] = 1.0 if projection > 0.0 else -1.0

        if signs[0] is None and signs[2] is None:
            signs[0], signs[2] = 1.0, -1.0
        elif signs[0] is None:
            signs[0] = -signs[2]
        elif signs[2] is None:
            signs[2] = -signs[0]
        elif signs[0] == signs[2]:
            signs[0], signs[2] = 1.0, -1.0
        self.formation_side_signs = signs

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

    def publish_formation_paths(self, points, headings, cumulative):
        if self.formation_side_signs is None:
            self.assign_formation_sides(headings[0])
        candidate_widths = [
            self.nominal_width,
            max(self.minimum_width, self.nominal_width * 0.8),
            max(self.minimum_width, self.nominal_width * 0.6),
            max(self.minimum_width, self.nominal_width * 0.4),
            self.minimum_width,
            0.0,
        ]
        raw_widths = []
        for distance, yaw in zip(cumulative, headings):
            chosen = 0.0
            for width in candidate_widths:
                if self.formation_sample_is_free(
                    distance, yaw, width, points, headings, cumulative
                ):
                    chosen = width
                    break
            raw_widths.append(chosen)

        # Fold before the leader reaches a gap; keep the convoy until the last
        # robot has cleared it, then restore the lateral shape.
        folding_distance = 0.35
        release_distance = 2.0 * self.follow_gap + 0.20
        adaptive_widths = []
        for index, distance in enumerate(cumulative):
            first = bisect.bisect_left(
                cumulative, max(0.0, distance - release_distance)
            )
            last = bisect.bisect_right(cumulative, distance + folding_distance)
            adaptive_widths.append(min(raw_widths[first:last] or [raw_widths[index]]))

        for robot_index, robot_name in enumerate(self.robot_names):
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_points = []
            path_headings = []
            for index, distance in enumerate(cumulative):
                width = adaptive_widths[index]
                lag, lateral_sign = self.formation_slot(robot_index, width)

                sample_distance = max(0.0, distance - lag)
                x, y, yaw = self.sample_path(
                    points, headings, cumulative, sample_distance
                )
                x += lateral_sign * width * -math.sin(yaw)
                y += lateral_sign * width * math.cos(yaw)
                path_points.append((x, y))
                path_headings.append(yaw)

                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                set_pose_yaw(pose.pose, yaw)
                path_msg.poses.append(pose)

            self.robot_paths[robot_index] = path_msg
            _, robot_cumulative = self.path_geometry(path_points)
            self.robot_path_data[robot_index] = (
                path_points, path_headings, robot_cumulative
            )
            self.path_indices[robot_index] = 0
            self.get_publisher_for_robot(robot_index).publish(path_msg)

        width_values = set(round(width, 2) for width in adaptive_widths)
        if width_values == {round(self.nominal_width, 2)}:
            self.formation_mode = f'wide (half-width {self.nominal_width:.2f} m)'
        elif min(adaptive_widths) == 0.0:
            self.formation_mode = 'column through narrow section, reopening adaptively'
        else:
            self.formation_mode = (
                f'compressed (minimum half-width {min(adaptive_widths):.2f} m)'
            )

    def get_publisher_for_robot(self, index):
        # Keep publishers in creation order; only command publishers are stored there.
        return self.path_pubs[index]

    def formation_sample_is_free(self, distance, yaw, width, points, headings, cumulative):
        del yaw
        for index in (0, 1, 2):
            lag, lateral_sign = self.formation_slot(index, width)
            x, y, tangent = self.sample_path(
                points, headings, cumulative, max(0.0, distance - lag)
            )
            x += lateral_sign * width * -math.sin(tangent)
            y += lateral_sign * width * math.cos(tangent)
            if not self.is_free_world(x, y):
                return False
        return True

    def scan_dynamic_points(self):
        now = time.monotonic()
        points = []
        for robot_points in self.scan_points:
            points.extend((x, y) for x, y, expiry in robot_points if expiry > now)
        return points

    def update_dynamic_layer(self):
        now = time.monotonic()
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
                self.plan(force=True)

    def control_tick(self):
        self.update_dynamic_layer()
        if self.dstar is not None and self.goal_xy is not None:
            start = self.nearest_free(self.world_to_grid(*self.team_center()[:2]))
            if start is not None and start != self.last_planned_start:
                self.plan()

        if self.goal_xy is not None:
            for index in range(3):
                self.control_robot(index)
        self.publish_team_states()
        self.publish_markers()

    def publish_team_states(self):
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for index in range(3):
            x, y, yaw, _ = self.get_robot_pose(index)
            pose = PoseStamped().pose
            pose.position.x = x
            pose.position.y = y
            set_pose_yaw(pose, yaw)
            msg.poses.append(pose)
        self.team_state_pub.publish(msg)

    def control_robot(self, index):
        path_data = self.robot_path_data[index]
        state = self.robot_state[index]
        if not self.motion_enabled:
            return
        if path_data is None:
            self.publish_stop(index)
            return
        if state is None:
            return
        points, headings, cumulative = path_data
        if len(points) < 2:
            self.publish_stop(index)
            return

        x, y, yaw, _ = state
        previous_index = self.path_indices[index]
        first = max(0, previous_index - 4)
        last = min(len(points), previous_index + 80)
        nearest_index = min(
            range(first, max(first + 1, last)),
            key=lambda i: (points[i][0] - x) ** 2 + (points[i][1] - y) ** 2,
        )
        self.path_indices[index] = nearest_index
        progress = cumulative[nearest_index]
        goal_distance = math.hypot(points[-1][0] - x, points[-1][1] - y)
        if goal_distance < 0.10:
            self.publish_stop(index)
            return

        dynamic_points = [
            (ox, oy)
            for ox, oy in self.scan_dynamic_points()
            if (ox - x) ** 2 + (oy - y) ** 2 < 1.0 ** 2
        ]
        previous = self.last_cmd[index]
        linear = self.max_linear_speed
        angular = self.max_angular_speed
        diagonal = linear / math.sqrt(2.0)
        translations = (
            (0.0, 0.0),
            (linear, 0.0), (-linear, 0.0),
            (0.0, linear), (0.0, -linear),
            (diagonal, diagonal), (diagonal, -diagonal),
            (-diagonal, diagonal), (-diagonal, -diagonal),
        )
        controls = [
            (vx, vy, omega_value)
            for vx, vy in translations
            for omega_value in (-angular, 0.0, angular)
        ]

        # Beam search over changing control sequences: each rollout is a
        # short-horizon omni-robot model, and only its first command is applied.
        # The next timer tick solves the horizon again (receding-horizon MPC).
        beam = [(x, y, yaw, 0.0, None, previous)]
        beam_width = 24
        for step in range(1, self.mpc_horizon + 1):
            expanded = []
            target_distance = min(
                cumulative[-1],
                progress + min(0.50, linear * self.mpc_dt * step),
            )
            tx, ty, tyaw = self.sample_path(
                points, headings, cumulative, target_distance
            )
            for px, py, pyaw, cost_so_far, first_command, last_command in beam:
                for command in controls:
                    vx, vy, omega = command
                    next_x = px + (
                        vx * math.cos(pyaw) - vy * math.sin(pyaw)
                    ) * self.mpc_dt
                    next_y = py + (
                        vx * math.sin(pyaw) + vy * math.cos(pyaw)
                    ) * self.mpc_dt
                    next_yaw = normalize_angle(pyaw + omega * self.mpc_dt)
                    if not self.is_free_world(next_x, next_y):
                        continue
                    if any(
                        (next_x - ox) ** 2 + (next_y - oy) ** 2 < 0.27 ** 2
                        for ox, oy in dynamic_points
                    ):
                        continue

                    position_error = (next_x - tx) ** 2 + (next_y - ty) ** 2
                    yaw_error = normalize_angle(tyaw - next_yaw)
                    command_change = (
                        (vx - last_command[0]) ** 2
                        + (vy - last_command[1]) ** 2
                        + 0.15 * (omega - last_command[2]) ** 2
                    )
                    effort = vx * vx + vy * vy + 0.12 * omega * omega
                    score = cost_so_far
                    score += (5.0 + 0.30 * step) * position_error
                    score += 0.10 * yaw_error * yaw_error
                    score += 0.30 * command_change + 0.04 * effort
                    expanded.append((
                        next_x,
                        next_y,
                        next_yaw,
                        score,
                        command if first_command is None else first_command,
                        command,
                    ))
            if not expanded:
                self.publish_stop(index)
                return
            expanded.sort(key=lambda candidate: candidate[3])
            beam = expanded[:beam_width]

        terminal_distance = min(
            cumulative[-1],
            progress + min(0.50, linear * self.mpc_dt * self.mpc_horizon),
        )
        terminal_x, terminal_y, _ = self.sample_path(
            points, headings, cumulative, terminal_distance
        )
        best = min(
            beam,
            key=lambda candidate: candidate[3]
            + 7.0 * ((candidate[0] - terminal_x) ** 2 + (candidate[1] - terminal_y) ** 2),
        )
        best_cmd = best[4]
        if best_cmd is None:
            self.publish_stop(index)
            return
        command = Twist()
        command.linear.x = best_cmd[0]
        command.linear.y = best_cmd[1]
        command.angular.z = best_cmd[2]
        self.cmd_pubs[index].publish(command)
        self.last_cmd[index] = best_cmd

    def publish_stop(self, index):
        self.cmd_pubs[index].publish(Twist())
        self.last_cmd[index] = (0.0, 0.0, 0.0)

    def publish_markers(self):
        now = time.monotonic()
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
            label.text = name if localized else f'{name} (spawn estimate)'
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
        for index in range(3):
            node.publish_stop(index)
        node.destroy_node()
        rclpy.shutdown()


class DistributedRobotMPC(Node):
    """One robot's independent receding-horizon controller."""

    def __init__(self):
        super().__init__('distributed_robot_mpc')
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('robot_names', ['robot1', 'robot2', 'robot3'])
        self.declare_parameter('spawn_pose', [3.5, -3.8, 1.57])
        self.declare_parameter('motion_enabled', True)
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('unknown_is_obstacle', True)
        self.declare_parameter('inflation_radius', 0.18)
        self.declare_parameter('max_linear_speed', 0.28)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('mpc_horizon', 8)
        self.declare_parameter('mpc_dt', 0.15)

        self.robot_name = str(self.get_parameter('robot_name').value)
        self.robot_names = list(self.get_parameter('robot_names').value)
        self.robot_index = self.robot_names.index(self.robot_name)
        spawn = list(self.get_parameter('spawn_pose').value)
        self.spawn_pose = tuple(float(value) for value in spawn)
        self.motion_enabled = bool(self.get_parameter('motion_enabled').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.unknown_is_obstacle = bool(self.get_parameter('unknown_is_obstacle').value)
        self.inflation_radius = float(self.get_parameter('inflation_radius').value)
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
        self.state_sub = self.create_subscription(
            PoseArray, '/formation/robot_states', self.team_state_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_name}/odom',
            self.odom_callback,
            10,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            f'/{self.robot_name}/scan',
            self.scan_callback,
            10,
        )
        self.path_subs = [
            self.create_subscription(
                Path,
                f'/{name}/desired_path',
                lambda msg, index=index: self.path_callback(msg, index),
                10,
            )
            for index, name in enumerate(self.robot_names)
        ]
        self.prediction_subs = [
            self.create_subscription(
                PoseArray,
                f'/{name}/mpc_prediction',
                lambda msg, index=index: self.prediction_callback(msg, index),
                10,
            )
            for index, name in enumerate(self.robot_names)
        ]
        self.cmd_pub = self.create_publisher(
            Twist, f'/{self.robot_name}/cmd_vel', 10
        )
        self.prediction_pub = self.create_publisher(
            PoseArray, f'/{self.robot_name}/mpc_prediction', 10
        )

        self.width = 0
        self.height = 0
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_yaw = 0.0
        self.blocked = []
        self.own_pose = None
        self.team_state_received = False
        self.peer_poses = [None, None, None]
        self.peer_predictions = [None, None, None]
        self.peer_prediction_times = [0.0, 0.0, 0.0]
        self.path_data = [None, None, None]
        self.path_index = 0
        self.scan_points = []
        self.last_scan_time = 0.0
        self.last_command = (0.0, 0.0, 0.0)
        self.create_timer(0.1, self.control_tick)
        self.get_logger().info(
            f'{self.robot_name}: independent distributed MPC controller ready.'
        )

    def map_callback(self, msg):
        info = msg.info
        self.width = int(info.width)
        self.height = int(info.height)
        self.resolution = float(info.resolution)
        self.origin_x = float(info.origin.position.x)
        self.origin_y = float(info.origin.position.y)
        self.origin_yaw = quaternion_yaw(info.origin.orientation)
        raw = [
            (value < 0 and self.unknown_is_obstacle)
            or value >= self.occupied_threshold
            for value in msg.data
        ]
        radius = max(0, int(math.ceil(self.inflation_radius / self.resolution)))
        self.blocked = raw.copy()
        if radius:
            disk = [
                (dx, dy)
                for dy in range(-radius, radius + 1)
                for dx in range(-radius, radius + 1)
                if dx * dx + dy * dy <= radius * radius
            ]
            for index, occupied in enumerate(raw):
                if not occupied:
                    continue
                gx, gy = index % self.width, index // self.width
                for dx, dy in disk:
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        self.blocked[ny * self.width + nx] = True

    def world_to_grid(self, x, y):
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return (
            int(math.floor((c * dx + s * dy) / self.resolution)),
            int(math.floor((-s * dx + c * dy) / self.resolution)),
        )

    def is_free(self, x, y):
        if not self.blocked or self.resolution <= 0.0:
            return False
        gx, gy = self.world_to_grid(x, y)
        if not (0 <= gx < self.width and 0 <= gy < self.height):
            return False
        return not self.blocked[gy * self.width + gx]

    def path_callback(self, msg, index):
        if not msg.poses:
            self.path_data[index] = None
            if index == self.robot_index:
                self.publish_stop()
            return
        points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]
        headings, cumulative = ThreeRobotDStarMPC.path_geometry(points)
        self.path_data[index] = (points, headings, cumulative)
        if index == self.robot_index:
            if self.own_pose is None:
                self.path_index = 0
            else:
                x, y, _ = self.own_pose
                self.path_index = min(
                    range(len(points)),
                    key=lambda i: (points[i][0] - x) ** 2
                    + (points[i][1] - y) ** 2,
                )

    def odom_callback(self, msg):
        if not self.team_state_received:
            self.own_pose = (
                self.spawn_pose[0] + msg.pose.pose.position.x,
                self.spawn_pose[1] + msg.pose.pose.position.y,
                quaternion_yaw(msg.pose.pose.orientation),
            )

    def team_state_callback(self, msg):
        for index, pose in enumerate(msg.poses[:3]):
            self.peer_poses[index] = (
                pose.position.x,
                pose.position.y,
                quaternion_yaw(pose.orientation),
            )
        if len(msg.poses) > self.robot_index:
            self.own_pose = self.peer_poses[self.robot_index]
            self.team_state_received = True

    def prediction_callback(self, msg, index):
        self.peer_predictions[index] = [
            (pose.position.x, pose.position.y, quaternion_yaw(pose.orientation))
            for pose in msg.poses
        ]
        self.peer_prediction_times[index] = time.monotonic()

    def scan_callback(self, msg):
        now = time.monotonic()
        if now - self.last_scan_time < 0.12:
            return
        self.last_scan_time = now
        pose = self.own_pose or self.spawn_pose
        x, y, yaw = pose
        stride = max(1, len(msg.ranges) // 90)
        points = []
        max_range = min(float(msg.range_max), 3.5)
        for beam in range(0, len(msg.ranges), stride):
            distance = float(msg.ranges[beam])
            if not math.isfinite(distance) or distance <= max(0.12, msg.range_min):
                continue
            if distance >= max_range:
                continue
            angle = yaw + msg.angle_min + beam * msg.angle_increment
            points.append((
                x + distance * math.cos(angle),
                y + distance * math.sin(angle),
                now + 0.65,
            ))
        self.scan_points = points

    def peer_context(self):
        context = []
        for index in range(3):
            if index == self.robot_index:
                continue
            pose = self.peer_poses[index]
            if pose is None:
                pose = self.spawn_pose_for(index)
            data = self.path_data[index]
            progress = 0.0
            if data is not None:
                points, _, cumulative = data
                nearest = min(
                    range(len(points)),
                    key=lambda i: (points[i][0] - pose[0]) ** 2
                    + (points[i][1] - pose[1]) ** 2,
                )
                progress = cumulative[nearest]
            context.append((index, pose, data, progress))
        return context

    def spawn_pose_for(self, index):
        defaults = (
            (3.5, -3.8, 1.57),
            (3.5, -3.0, 1.57),
            (3.5, -2.2, 1.57),
        )
        return defaults[index]

    def peer_prediction(self, context, step):
        predictions = []
        travel = self.max_linear_speed * self.mpc_dt * step
        for index, pose, data, progress in context:
            communicated = self.peer_predictions[index]
            if (
                communicated
                and time.monotonic() - self.peer_prediction_times[index] < 0.4
                and len(communicated) >= step
            ):
                predictions.append(communicated[step - 1][:2])
                continue
            x, y, _ = pose
            if data is not None:
                points, headings, cumulative = data
                target_s = min(cumulative[-1], progress + min(0.35, travel))
                tx, ty, _ = ThreeRobotDStarMPC.sample_path(
                    points, headings, cumulative, target_s
                )
                distance = math.hypot(tx - x, ty - y)
                if distance > 1e-6:
                    moved = min(distance, travel)
                    x += (tx - x) * moved / distance
                    y += (ty - y) * moved / distance
            predictions.append((x, y))
        return predictions

    def control_tick(self):
        if not self.motion_enabled:
            return
        if self.own_pose is None:
            return
        if self.width == 0 or not self.path_data[self.robot_index]:
            self.publish_stop()
            return

        now = time.monotonic()
        self.scan_points = [point for point in self.scan_points if point[2] > now]
        x, y, yaw = self.own_pose
        points, headings, cumulative = self.path_data[self.robot_index]
        if len(points) < 2:
            self.publish_stop()
            return
        start = max(0, self.path_index - 4)
        end = min(len(points), self.path_index + 80)
        self.path_index = min(
            range(start, max(start + 1, end)),
            key=lambda i: (points[i][0] - x) ** 2 + (points[i][1] - y) ** 2,
        )
        progress = cumulative[self.path_index]
        if math.hypot(points[-1][0] - x, points[-1][1] - y) < 0.10:
            self.publish_stop()
            return

        local_obstacles = [
            (ox, oy)
            for ox, oy, _ in self.scan_points
            if (ox - x) ** 2 + (oy - y) ** 2 < 0.8 ** 2
        ]
        peers = self.peer_context()
        linear = self.max_linear_speed
        angular = self.max_angular_speed
        diagonal = linear / math.sqrt(2.0)
        translations = (
            (0.0, 0.0),
            (linear, 0.0), (-linear, 0.0),
            (0.0, linear), (0.0, -linear),
            (diagonal, diagonal), (diagonal, -diagonal),
            (-diagonal, diagonal), (-diagonal, -diagonal),
        )
        controls = [
            (vx, vy, omega)
            for vx, vy in translations
            for omega in (-angular, 0.0, angular)
        ]
        beam = [(x, y, yaw, 0.0, None, self.last_command, [])]
        beam_width = 24

        for step in range(1, self.mpc_horizon + 1):
            target_s = min(
                cumulative[-1],
                progress + min(0.50, linear * self.mpc_dt * step),
            )
            tx, ty, tyaw = ThreeRobotDStarMPC.sample_path(
                points, headings, cumulative, target_s
            )
            peer_positions = self.peer_prediction(peers, step)
            expanded = []
            for px, py, pyaw, old_cost, first_command, last_command, trajectory in beam:
                for command in controls:
                    vx, vy, omega = command
                    nx = px + (vx * math.cos(pyaw) - vy * math.sin(pyaw)) * self.mpc_dt
                    ny = py + (vx * math.sin(pyaw) + vy * math.cos(pyaw)) * self.mpc_dt
                    nyaw = normalize_angle(pyaw + omega * self.mpc_dt)
                    if not self.is_free(nx, ny):
                        continue
                    if any((nx - ox) ** 2 + (ny - oy) ** 2 < 0.27 ** 2 for ox, oy in local_obstacles):
                        continue
                    if any((nx - ox) ** 2 + (ny - oy) ** 2 < 0.30 ** 2 for ox, oy in peer_positions):
                        continue

                    position_error = (nx - tx) ** 2 + (ny - ty) ** 2
                    yaw_error = normalize_angle(tyaw - nyaw)
                    change = (
                        (vx - last_command[0]) ** 2
                        + (vy - last_command[1]) ** 2
                        + 0.15 * (omega - last_command[2]) ** 2
                    )
                    effort = vx * vx + vy * vy + 0.12 * omega * omega
                    score = old_cost
                    score += (5.0 + 0.30 * step) * position_error
                    score += 0.10 * yaw_error * yaw_error
                    score += 0.30 * change + 0.04 * effort
                    expanded.append((
                        nx,
                        ny,
                        nyaw,
                        score,
                        command if first_command is None else first_command,
                        command,
                        trajectory + [(nx, ny, nyaw)],
                    ))
            if not expanded:
                self.publish_stop()
                return
            expanded.sort(key=lambda state: state[3])
            beam = expanded[:beam_width]

        terminal_s = min(
            cumulative[-1],
            progress + min(0.50, linear * self.mpc_dt * self.mpc_horizon),
        )
        tx, ty, _ = ThreeRobotDStarMPC.sample_path(
            points, headings, cumulative, terminal_s
        )
        best = min(
            beam,
            key=lambda state: state[3] + 7.0 * ((state[0] - tx) ** 2 + (state[1] - ty) ** 2),
        )
        command = best[4]
        if command is None:
            self.publish_stop()
            return
        message = Twist()
        message.linear.x = command[0]
        message.linear.y = command[1]
        message.angular.z = command[2]
        self.cmd_pub.publish(message)
        self.last_command = command
        self.publish_prediction(best[6])

    def publish_stop(self):
        self.cmd_pub.publish(Twist())
        self.last_command = (0.0, 0.0, 0.0)
        pose = self.own_pose or self.spawn_pose
        self.publish_prediction([pose for _ in range(self.mpc_horizon)])

    def publish_prediction(self, trajectory):
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in trajectory:
            pose = PoseStamped().pose
            pose.position.x = x
            pose.position.y = y
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
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
