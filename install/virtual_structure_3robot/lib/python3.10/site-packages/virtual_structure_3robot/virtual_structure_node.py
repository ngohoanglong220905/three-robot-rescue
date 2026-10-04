
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class VirtualStructure3Robot(Node):

    def __init__(self):
        super().__init__("virtual_structure_3robot")

        # =====================================================
        # PARAMETERS
        # =====================================================

        self.declare_parameter("formation_spacing", 0.60)

        self.spacing = self.get_parameter(
            "formation_spacing"
        ).value

        # =====================================================
        # SUBSCRIBER
        # =====================================================

        self.path_sub = self.create_subscription(
            Path,
            "/astar_path",
            self.path_callback,
            10
        )

        # =====================================================
        # PUBLISHERS
        # =====================================================

        self.robot1_path_pub = self.create_publisher(
            Path,
            "/robot1/desired_path",
            10
        )

        self.robot2_path_pub = self.create_publisher(
            Path,
            "/robot2/desired_path",
            10
        )

        self.robot3_path_pub = self.create_publisher(
            Path,
            "/robot3/desired_path",
            10
        )

        self.robot1_pose_pub = self.create_publisher(
            PoseStamped,
            "/robot1/desired_pose",
            10
        )

        self.robot2_pose_pub = self.create_publisher(
            PoseStamped,
            "/robot2/desired_pose",
            10
        )

        self.robot3_pose_pub = self.create_publisher(
            PoseStamped,
            "/robot3/desired_pose",
            10
        )

        self.get_logger().info(
            "Virtual Structure 3 Robot started"
        )

        self.get_logger().info(
            f"Formation spacing = {self.spacing:.2f} m"
        )

    # =========================================================
    # PATH CALLBACK
    # =========================================================

    def path_callback(self, msg: Path):

        if len(msg.poses) < 2:
            self.get_logger().warn(
                "A* path too short"
            )
            return

        robot1_path = Path()
        robot2_path = Path()
        robot3_path = Path()

        robot1_path.header = msg.header
        robot2_path.header = msg.header
        robot3_path.header = msg.header

        robot1_path.header.frame_id = "map"
        robot2_path.header.frame_id = "map"
        robot3_path.header.frame_id = "map"

        # =====================================================
        # GENERATE 3 FORMATION PATHS
        # =====================================================

        for i in range(len(msg.poses)):

            current_pose = msg.poses[i]

            # -------------------------------------------------
            # Find tangent direction
            # -------------------------------------------------

            if i == 0:
                next_pose = msg.poses[i + 1]

                dx = (
                    next_pose.pose.position.x
                    - current_pose.pose.position.x
                )

                dy = (
                    next_pose.pose.position.y
                    - current_pose.pose.position.y
                )

            elif i == len(msg.poses) - 1:

                prev_pose = msg.poses[i - 1]

                dx = (
                    current_pose.pose.position.x
                    - prev_pose.pose.position.x
                )

                dy = (
                    current_pose.pose.position.y
                    - prev_pose.pose.position.y
                )

            else:

                prev_pose = msg.poses[i - 1]
                next_pose = msg.poses[i + 1]

                dx = (
                    next_pose.pose.position.x
                    - prev_pose.pose.position.x
                )

                dy = (
                    next_pose.pose.position.y
                    - prev_pose.pose.position.y
                )

            # -------------------------------------------------
            # Normalize tangent
            # -------------------------------------------------

            length = math.sqrt(
                dx * dx + dy * dy
            )

            if length < 1e-6:
                continue

            tx = dx / length
            ty = dy / length

            # -------------------------------------------------
            # Normal vector
            #
            # n = [-ty, tx]
            # -------------------------------------------------

            nx = -ty
            ny = tx

            # -------------------------------------------------
            # Center robot
            # -------------------------------------------------

            center_x = current_pose.pose.position.x
            center_y = current_pose.pose.position.y

            # -------------------------------------------------
            # Robot 1 = LEFT
            # -------------------------------------------------

            robot1_x = (
                center_x
                + self.spacing * nx
            )

            robot1_y = (
                center_y
                + self.spacing * ny
            )

            # -------------------------------------------------
            # Robot 2 = CENTER
            # -------------------------------------------------

            robot2_x = center_x
            robot2_y = center_y

            # -------------------------------------------------
            # Robot 3 = RIGHT
            # -------------------------------------------------

            robot3_x = (
                center_x
                - self.spacing * nx
            )

            robot3_y = (
                center_y
                - self.spacing * ny
            )

            # -------------------------------------------------
            # Desired poses
            # -------------------------------------------------

            pose1 = self.make_pose(
                current_pose,
                robot1_x,
                robot1_y,
                tx,
                ty
            )

            pose2 = self.make_pose(
                current_pose,
                robot2_x,
                robot2_y,
                tx,
                ty
            )

            pose3 = self.make_pose(
                current_pose,
                robot3_x,
                robot3_y,
                tx,
                ty
            )

            robot1_path.poses.append(pose1)
            robot2_path.poses.append(pose2)
            robot3_path.poses.append(pose3)

        # =====================================================
        # PUBLISH PATHS
        # =====================================================

        self.robot1_path_pub.publish(
            robot1_path
        )

        self.robot2_path_pub.publish(
            robot2_path
        )

        self.robot3_path_pub.publish(
            robot3_path
        )

        # =====================================================
        # PUBLISH FIRST DESIRED POSE
        # =====================================================

        if len(robot1_path.poses) > 0:

            self.robot1_pose_pub.publish(
                robot1_path.poses[0]
            )

            self.robot2_pose_pub.publish(
                robot2_path.poses[0]
            )

            self.robot3_pose_pub.publish(
                robot3_path.poses[0]
            )

        self.get_logger().info(
            f"Virtual Structure generated "
            f"{len(robot1_path.poses)} points"
        )

    # =========================================================
    # CREATE POSE
    # =========================================================

    def make_pose(
        self,
        source_pose,
        x,
        y,
        tx,
        ty
    ):

        pose = PoseStamped()

        pose.header = source_pose.header

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = (
            source_pose.pose.position.z
        )

        yaw = math.atan2(
            ty,
            tx
        )

        pose.pose.orientation.z = math.sin(
            yaw / 2.0
        )

        pose.pose.orientation.w = math.cos(
            yaw / 2.0
        )

        return pose


def main(args=None):

    rclpy.init(args=args)

    node = VirtualStructure3Robot()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

