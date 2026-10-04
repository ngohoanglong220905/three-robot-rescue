
#!/usr/bin/env python3

import math
import heapq

import rclpy
from rclpy.node import Node

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PointStamped, PoseStamped

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


class AStarPlanner(Node):

    def __init__(self):
        super().__init__('astar_planner')

        # =====================================================
        # PARAMETERS
        # =====================================================

        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('inflation_radius', 0.10)
        self.declare_parameter('unknown_is_obstacle', True)

        self.occupied_threshold = int(
            self.get_parameter('occupied_threshold').value
        )

        self.inflation_radius = float(
            self.get_parameter('inflation_radius').value
        )

        self.unknown_is_obstacle = bool(
            self.get_parameter('unknown_is_obstacle').value
        )

        # =====================================================
        # MAP QoS
        # map_server publishes static map with TRANSIENT_LOCAL
        # =====================================================

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # =====================================================
        # ROS SUBSCRIBERS / PUBLISHER
        # =====================================================

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos,
        )

        self.clicked_point_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10,
        )

        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10,
        )

        self.path_pub = self.create_publisher(
            Path,
            '/astar_path',
            10,
        )

        # =====================================================
        # TF
        # =====================================================

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # =====================================================
        # INTERNAL DATA
        # =====================================================

        self.map_msg = None
        self.grid = None
        self.last_goal = None

        self.last_map_log_time = self.get_clock().now()

        self.get_logger().info('A* planner started.')
        self.get_logger().info(
            'Waiting for /map and a goal...'
        )

    # =========================================================
    # MAP CALLBACK
    # =========================================================

    def map_callback(self, msg):

        self.map_msg = msg

        now = self.get_clock().now()

        # Avoid printing "Map received" continuously
        if (
            now - self.last_map_log_time
        ).nanoseconds >= 5_000_000_000:

            self.get_logger().info(
                f'Map received: '
                f'{msg.info.width} x {msg.info.height}, '
                f'resolution={msg.info.resolution:.3f}'
            )

            self.last_map_log_time = now

        self.build_inflated_grid()

        if self.last_goal is not None:
            self.plan_to_goal(
                self.last_goal[0],
                self.last_goal[1]
            )

    # =========================================================
    # GOAL CALLBACKS
    # =========================================================

    def clicked_point_callback(self, msg):

        if msg.header.frame_id not in ('', 'map'):

            self.get_logger().warn(
                f'Clicked point frame is '
                f'{msg.header.frame_id}, expected map'
            )

        self.plan_to_goal(
            msg.point.x,
            msg.point.y
        )

    def goal_pose_callback(self, msg):

        if msg.header.frame_id not in ('', 'map'):

            self.get_logger().warn(
                f'Goal frame is '
                f'{msg.header.frame_id}, expected map'
            )

        self.plan_to_goal(
            msg.pose.position.x,
            msg.pose.position.y
        )

    # =========================================================
    # BUILD INFLATED GRID
    # =========================================================

    def build_inflated_grid(self):

        if self.map_msg is None:
            return

        width = self.map_msg.info.width
        height = self.map_msg.info.height
        data = self.map_msg.data

        occupied = [False] * (width * height)

        for i, value in enumerate(data):

            if value < 0:

                occupied[i] = self.unknown_is_obstacle

            elif value >= self.occupied_threshold:

                occupied[i] = True

        radius_cells = int(
            math.ceil(
                self.inflation_radius /
                self.map_msg.info.resolution
            )
        )

        inflated = occupied.copy()

        for y in range(height):

            for x in range(width):

                index = y * width + x

                if not occupied[index]:
                    continue

                for dy in range(
                    -radius_cells,
                    radius_cells + 1
                ):

                    for dx in range(
                        -radius_cells,
                        radius_cells + 1
                    ):

                        if (
                            dx * dx + dy * dy
                            > radius_cells * radius_cells
                        ):
                            continue

                        nx = x + dx
                        ny = y + dy

                        if (
                            0 <= nx < width and
                            0 <= ny < height
                        ):

                            inflated[
                                ny * width + nx
                            ] = True

        self.grid = inflated

    # =========================================================
    # WORLD -> GRID
    # =========================================================

    def world_to_grid(self, x, y):

        info = self.map_msg.info

        gx = int(
            math.floor(
                (
                    x - info.origin.position.x
                ) / info.resolution
            )
        )

        gy = int(
            math.floor(
                (
                    y - info.origin.position.y
                ) / info.resolution
            )
        )

        return gx, gy

    # =========================================================
    # GRID -> WORLD
    # =========================================================

    def grid_to_world(self, gx, gy):

        info = self.map_msg.info

        x = (
            info.origin.position.x
            + (gx + 0.5) * info.resolution
        )

        y = (
            info.origin.position.y
            + (gy + 0.5) * info.resolution
        )

        return x, y

    # =========================================================
    # CHECK FREE CELL
    # =========================================================

    def is_free(self, x, y):

        if self.grid is None:
            return False

        width = self.map_msg.info.width
        height = self.map_msg.info.height

        if not (
            0 <= x < width
            and 0 <= y < height
        ):
            return False

        return not self.grid[
            y * width + x
        ]

    # =========================================================
    # FIND NEAREST FREE CELL
    # =========================================================

    def find_nearest_free(
        self,
        cell,
        max_radius=15
    ):

        cx, cy = cell

        if self.is_free(cx, cy):
            return cell

        for radius in range(
            1,
            max_radius + 1
        ):

            for dy in range(
                -radius,
                radius + 1
            ):

                for dx in range(
                    -radius,
                    radius + 1
                ):

                    if max(
                        abs(dx),
                        abs(dy)
                    ) != radius:
                        continue

                    nx = cx + dx
                    ny = cy + dy

                    if self.is_free(nx, ny):
                        return nx, ny

        return None

    # =========================================================
    # ROBOT POSITION
    # =========================================================

    def get_robot_position(self):

        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    'map',
                    'base_link',
                    rclpy.time.Time()
                )
            )

            return (
                transform.transform.translation.x,
                transform.transform.translation.y
            )

        except TransformException:

            try:

                transform = (
                    self.tf_buffer.lookup_transform(
                        'map',
                        'base_footprint',
                        rclpy.time.Time()
                    )
                )

                return (
                    transform.transform.translation.x,
                    transform.transform.translation.y
                )

            except TransformException as exc:

                self.get_logger().error(
                    f'Cannot get robot pose: {exc}'
                )

                return None

    # =========================================================
    # HEURISTIC
    # =========================================================

    def heuristic(self, a, b):

        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])

        return (
            dx
            + dy
            + (math.sqrt(2.0) - 2.0)
            * min(dx, dy)
        )

    # =========================================================
    # A*
    # =========================================================

    def astar(self, start, goal):

        if not self.is_free(*start):

            self.get_logger().error(
                f'Start cell {start} is occupied.'
            )

            return None

        if not self.is_free(*goal):

            self.get_logger().error(
                f'Goal cell {goal} is occupied.'
            )

            return None

        neighbors = [
            (-1,  0, 1.0),
            ( 1,  0, 1.0),
            ( 0, -1, 1.0),
            ( 0,  1, 1.0),

            (-1, -1, math.sqrt(2.0)),
            (-1,  1, math.sqrt(2.0)),
            ( 1, -1, math.sqrt(2.0)),
            ( 1,  1, math.sqrt(2.0)),
        ]

        open_set = []

        heapq.heappush(
            open_set,
            (
                self.heuristic(start, goal),
                0.0,
                start
            )
        )

        came_from = {}
        g_score = {start: 0.0}
        closed = set()

        while open_set:

            _, current_cost, current = (
                heapq.heappop(open_set)
            )

            if current in closed:
                continue

            closed.add(current)

            if current == goal:

                path = [current]

                while current in came_from:

                    current = came_from[current]
                    path.append(current)

                path.reverse()

                return path

            cx, cy = current

            for dx, dy, move_cost in neighbors:

                nx = cx + dx
                ny = cy + dy

                if not self.is_free(nx, ny):
                    continue

                # Prevent diagonal corner cutting
                if dx != 0 and dy != 0:

                    if not self.is_free(
                        cx + dx,
                        cy
                    ):
                        continue

                    if not self.is_free(
                        cx,
                        cy + dy
                    ):
                        continue

                neighbor = (nx, ny)

                tentative_g = (
                    current_cost + move_cost
                )

                if (
                    neighbor not in g_score
                    or tentative_g
                    < g_score[neighbor]
                ):

                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g

                    f_score = (
                        tentative_g
                        + self.heuristic(
                            neighbor,
                            goal
                        )
                    )

                    heapq.heappush(
                        open_set,
                        (
                            f_score,
                            tentative_g,
                            neighbor
                        )
                    )

        return None

    # =========================================================
    # PLAN
    # =========================================================

    def plan_to_goal(
        self,
        goal_x,
        goal_y
    ):

        if self.map_msg is None:

            self.get_logger().warn(
                'No map received yet.'
            )

            return

        if self.grid is None:

            self.get_logger().warn(
                'Inflated grid not ready.'
            )

            return

        robot = self.get_robot_position()

        if robot is None:
            return

        start_cell = self.world_to_grid(
            robot[0],
            robot[1]
        )

        goal_cell = self.world_to_grid(
            goal_x,
            goal_y
        )

        # Move goal to nearest free cell if needed
        if not self.is_free(*goal_cell):

            new_goal = self.find_nearest_free(
                goal_cell
            )

            if new_goal is None:

                self.get_logger().error(
                    f'No free cell near goal '
                    f'{goal_cell}'
                )

                return

            self.get_logger().warn(
                f'Goal {goal_cell} is occupied. '
                f'Using nearest free cell '
                f'{new_goal}.'
            )

            goal_cell = new_goal

        # Move start to nearest free cell if needed
        if not self.is_free(*start_cell):

            new_start = self.find_nearest_free(
                start_cell
            )

            if new_start is None:

                self.get_logger().error(
                    f'No free cell near start '
                    f'{start_cell}'
                )

                return

            self.get_logger().warn(
                f'Start {start_cell} is occupied. '
                f'Using nearest free cell '
                f'{new_start}.'
            )

            start_cell = new_start

        self.get_logger().info(
            f'A* start={start_cell}, '
            f'goal={goal_cell}'
        )

        path_cells = self.astar(
            start_cell,
            goal_cell
        )

        if path_cells is None:

            self.get_logger().error(
                'A* could not find a path.'
            )

            return

        self.publish_path(path_cells)

        self.last_goal = (
            goal_x,
            goal_y
        )

        self.get_logger().info(
            f'A* path found: '
            f'{len(path_cells)} cells'
        )

    # =========================================================
    # PUBLISH PATH
    # =========================================================

    def publish_path(self, path_cells):

        path_msg = Path()

        path_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        path_msg.header.frame_id = 'map'

        for i, (gx, gy) in enumerate(
            path_cells
        ):

            x, y = self.grid_to_world(
                gx,
                gy
            )

            pose = PoseStamped()

            pose.header = path_msg.header

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            if i < len(path_cells) - 1:

                next_x, next_y = (
                    self.grid_to_world(
                        path_cells[i + 1][0],
                        path_cells[i + 1][1]
                    )
                )

                yaw = math.atan2(
                    next_y - y,
                    next_x - x
                )

                pose.pose.orientation.z = (
                    math.sin(yaw / 2.0)
                )

                pose.pose.orientation.w = (
                    math.cos(yaw / 2.0)
                )

            else:

                pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)


def main(args=None):

    rclpy.init(args=args)

    node = AStarPlanner()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
