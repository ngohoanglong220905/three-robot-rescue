import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class VirtualStructureNode(Node):

    def __init__(self):
        super().__init__("virtual_structure_node")

        self.declare_parameter("spacing", 0.60)

        self.spacing = float(
            self.get_parameter("spacing").value
        )

        self.subscription = self.create_subscription(
            Path,
            "/astar_path",
            self.path_callback,
            10
        )

        self.robot1_pub = self.create_publisher(
            Path,
            "/robot1/desired_path",
            10
        )

        self.robot2_pub = self.create_publisher(
            Path,
            "/robot2/desired_path",
            10
        )

        self.robot3_pub = self.create_publisher(
            Path,
            "/robot3/desired_path",
            10
        )

        self.get_logger().info(
            "Virtual Structure 3 robots started"
        )

        self.get_logger().info(
            f"Formation spacing = {self.spacing:.2f} m"
        )

    # =====================================================
    # A* PATH CALLBACK
    # =====================================================

    def path_callback(self, msg):

        if len(msg.poses) < 2:
            self.get_logger().warn(
                "A* path has less than 2 poses"
            )
            return

        path1 = Path()
        path2 = Path()
        path3 = Path()

        path1.header = msg.header
        path2.header = msg.header
        path3.header = msg.header

        path1.header.frame_id = "map"
        path2.header.frame_id = "map"
        path3.header.frame_id = "map"

        for i in range(len(msg.poses)):

            current = msg.poses[i]

            # ---------------------------------------------
            # Tính hướng tiếp tuyến của đường A*
            # ---------------------------------------------

            if i == 0:

                nxt = msg.poses[i + 1]

                dx = (
                    nxt.pose.position.x
                    - current.pose.position.x
                )

                dy = (
                    nxt.pose.position.y
                    - current.pose.position.y
                )

            elif i == len(msg.poses) - 1:

                prev = msg.poses[i - 1]

                dx = (
                    current.pose.position.x
                    - prev.pose.position.x
                )

                dy = (
                    current.pose.position.y
                    - prev.pose.position.y
                )

            else:

                prev = msg.poses[i - 1]
                nxt = msg.poses[i + 1]

                dx = (
                    nxt.pose.position.x
                    - prev.pose.position.x
                )

                dy = (
                    nxt.pose.position.y
                    - prev.pose.position.y
                )

            length = math.hypot(dx, dy)

            if length < 1e-6:
                continue

            # Vector tiếp tuyến
            tx = dx / length
            ty = dy / length

            # Vector pháp tuyến
            nx = -ty
            ny = tx

            x = current.pose.position.x
            y = current.pose.position.y

            # ---------------------------------------------
            # Robot 1: trái
            # ---------------------------------------------

            r1_x = x + self.spacing * nx
            r1_y = y + self.spacing * ny

            # ---------------------------------------------
            # Robot 2: leader / center
            # ---------------------------------------------

            r2_x = x
            r2_y = y

            # ---------------------------------------------
            # Robot 3: phải
            # ---------------------------------------------

            r3_x = x - self.spacing * nx
            r3_y = y - self.spacing * ny

            # ---------------------------------------------
            # Tạo pose
            # ---------------------------------------------

            pose1 = self.create_pose(
                msg.header,
                r1_x,
                r1_y,
                tx,
                ty
            )

            pose2 = self.create_pose(
                msg.header,
                r2_x,
                r2_y,
                tx,
                ty
            )

            pose3 = self.create_pose(
                msg.header,
                r3_x,
                r3_y,
                tx,
                ty
            )

            path1.poses.append(pose1)
            path2.poses.append(pose2)
            path3.poses.append(pose3)

        # ---------------------------------------------
        # Publish
        # ---------------------------------------------

        self.robot1_pub.publish(path1)
        self.robot2_pub.publish(path2)
        self.robot3_pub.publish(path3)

        self.get_logger().info(
            f"Published formation paths: {len(path1.poses)} points"
        )

    # =====================================================
    # CREATE POSE
    # =====================================================

    def create_pose(
        self,
        header,
        x,
        y,
        tx,
        ty
    ):

        pose = PoseStamped()

        pose.header = header
        pose.header.frame_id = "map"

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        yaw = math.atan2(ty, tx)

        pose.pose.orientation.z = math.sin(
            yaw / 2.0
        )

        pose.pose.orientation.w = math.cos(
            yaw / 2.0
        )

        return pose


def main(args=None):

    rclpy.init(args=args)

    node = VirtualStructureNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
