#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import heapq
import math

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('a_star_planner')

        # Subscriptions
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.start_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.start_callback, 10)

        # Publisher
        self.path_pub = self.create_publisher(Path, '/a_star_path', 10)

        self.map_data = None
        # Vị trí xuất phát mặc định (trùng vị trí spawn robot ở góc dưới bên phải)
        self.start_pose = (3.5, -3.5)
        self.goal_pose = None

        self.get_logger().info("A* Path Planner Node đã sẵn sàng!")

    def map_callback(self, msg):
        self.map_data = msg

    def start_callback(self, msg):
        self.start_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info(f"Vị trí bắt đầu mới: ({self.start_pose[0]:.2f}, {self.start_pose[1]:.2f})")
        self.plan_path()

    def goal_callback(self, msg):
        self.goal_pose = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"Nhận điểm đích mới: ({self.goal_pose[0]:.2f}, {self.goal_pose[1]:.2f})")
        self.plan_path()

    def world_to_grid(self, wx, wy):
        if not self.map_data:
            return None
        res = self.map_data.info.resolution
        ox = self.map_data.info.origin.position.x
        oy = self.map_data.info.origin.position.y

        gx = int((wx - ox) / res)
        gy = int((wy - oy) / res)
        return (gx, gy)

    def grid_to_world(self, gx, gy):
        res = self.map_data.info.resolution
        ox = self.map_data.info.origin.position.x
        oy = self.map_data.info.origin.position.y

        wx = ox + (gx + 0.5) * res
        wy = oy + (gy + 0.5) * res
        return (wx, wy)

    def is_valid(self, gx, gy, inflation_cells=2):
        """Kiểm tra ô có thuộc bản đồ và có va chạm vật cản (bao gồm vùng an toàn) không."""
        width = self.map_data.info.width
        height = self.map_data.info.height

        if gx < 0 or gx >= width or gy < 0 or gy >= height:
            return False

        # Kiểm tra bán kính mở rộng vật cản (Inflation) để tránh cọ tường/ngách hẹp
        for dx in range(-inflation_cells, inflation_cells + 1):
            for dy in range(-inflation_cells, inflation_cells + 1):
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < width and 0 <= ny < height:
                    idx = ny * width + nx
                    val = self.map_data.data[idx]
                    if val > 50 or val == -1:  # Vật cản (>50) hoặc vùng chưa quét (-1)
                        return False
        return True

    def heuristic(self, a, b):
        # Euclidean distance
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def plan_path(self):
        if self.map_data is None:
            self.get_logger().warn("Chưa nhận được bản đồ từ Topic /map!")
            return
        if self.goal_pose is None:
            return

        start_grid = self.world_to_grid(self.start_pose[0], self.start_pose[1])
        goal_grid = self.world_to_grid(self.goal_pose[0], self.goal_pose[1])

        if not start_grid or not goal_grid:
            self.get_logger().error("Tọa độ nằm ngoài phạm vi bản đồ!")
            return

        # 8 hướng di chuyển (Ngang/Dọc: 1.0, Đường chéo: ~1.414)
        motion = [
            (1, 0, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414), (1, -1, 1.414)
        ]

        open_set = []
        heapq.heappush(open_set, (0, start_grid))
        came_from = {}
        g_score = {start_grid: 0}

        found = False

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal_grid:
                found = True
                break

            for dx, dy, cost in motion:
                neighbor = (current[0] + dx, current[1] + dy)

                if not self.is_valid(neighbor[0], neighbor[1]):
                    continue

                tentative_g = g_score[current] + cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f_score, neighbor))

        if found:
            # Truy vết ngược lại đường đi từ Đích -> Gốc
            curr = goal_grid
            grid_path = []
            while curr in came_from:
                grid_path.append(curr)
                curr = came_from[curr]
            grid_path.append(start_grid)
            grid_path.reverse()

            # Tạo message nav_msgs/Path
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp = self.get_clock().now().to_msg()

            for gx, gy in grid_path:
                wx, wy = self.grid_to_world(gx, gy)
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.position.x = wx
                pose.position.y = wy
                pose.position.z = 0.0
                pose.orientation.w = 1.0
                path_msg.poses.append(pose)

            self.path_pub.publish(path_msg)
            self.get_logger().info(f"Đã quy hoạch đường đi A* thành công! Số nút: {len(grid_path)}")
        else:
            self.get_logger().error("Không tìm thấy đường đi A* tới điểm đích!")

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