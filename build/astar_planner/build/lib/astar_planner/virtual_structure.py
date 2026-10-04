#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class VirtualStructure(Node):

    def __init__(self):
        super().__init__('virtual_structure')

        # ==========================================
        # Parameters
        # ==========================================

        self.declare_parameter('robot_spacing', 0.60)

        self.spacing = float(
            self.get_parameter('robot_spacing').value
        )

        # ==========================================
        # Publishers
        # ==========================================

        self.robot1_pub = self.create_publisher(
            PoseStamped,
            '/robot1/desired_pose',
            10
        )

        self.robot2_pub = self.create_publisher(
            PoseStamped,
            '/robot2/desired_pose',
            10
        )

        self.robot3_pub = self.create_publisher(
            PoseStamped,
            '/robot3/desired_pose',
            10
        )

        # ==========================================
        # Subscriber
        # ==========================================

        self.path_sub = self.create_subscription(
            Path,
            '/astar_path',
            self.path_callback,
            10
        )

        self.get_logger().info(
            'Virtual Structure started.'
        )

        self.get_logger().info(
            f'Robot spacing = {self.spacing:.2f} m'
        )

    # =====================================================
    # PATH CALLBACK
    # =====================================================

    def path_callback(self, path):

        if len(path.poses) < 2:
            self.get_logger().warn(
                'A* path has too few points.'
            )
            return

        # -------------------------------------------------
        # Choose a representative point on the path.
        # For testing, use the first point near the robot.
        # -------------------------------------------------

        center_pose = path.poses[0]

        center_x = center_pose.pose.position.x
        center_y = center_pose.pose.position.y

        # -------------------------------------------------
        # Estimate path heading
        # -------------------------------------------------

        next_pose = path.poses[1]

        dx = (
            next_pose.pose.position.x
            - center_x
        )

        dy = (
            next_pose.pose.position.y
            - center_y
        )

        yaw = math.atan2(dy, dx)

        # -------------------------------------------------
        # Normal vector to the path
        #
        # left normal:
        # nx = -sin(yaw)
        # ny =  cos(yaw)
        # -------------------------------------------------

        nx = -math.sin(yaw)
        ny = math.cos(yaw)

        # -------------------------------------------------
        # Three robot desired positions
        # -------------------------------------------------

        robot1_x = center_x + nx * self.spacing
        robot1_y = center_y + ny * self.spacing

        robot2_x = center_x
        robot2_y = center_y

        robot3_x = center_x - nx * self.spacing
        robot3_y = center_y - ny * self.spacing

        # -------------------------------------------------
        # Create poses
        # -------------------------------------------------

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        robot1_pose = PoseStamped()
        robot2_pose = PoseStamped()
        robot3_pose = PoseStamped()

        robot1_pose.header = path.header
        robot2_pose.header = path.header
        robot3_pose.header = path.header

        robot1_pose.pose.position.x = robot1_x
        robot1_pose.pose.position.y = robot1_y
        robot1_pose.pose.orientation.z = qz
        robot1_pose.pose.orientation.w = qw

        robot2_pose.pose.position.x = robot2_x
        robot2_pose.pose.position.y = robot2_y
        robot2_pose.pose.orientation.z = qz
        robot2_pose.pose.orientation.w = qw

        robot3_pose.pose.position.x = robot3_x
        robot3_pose.pose.position.y = robot3_y
        robot3_pose.pose.orientation.z = qz
        robot3_pose.pose.orientation.w = qw

        # -------------------------------------------------
        # Publish
        # -------------------------------------------------

        self.robot1_pub.publish(robot1_pose)
        self.robot2_pub.publish(robot2_pose)
        self.robot3_pub.publish(robot3_pose)

        self.get_logger().info(
            f'Formation: '
            f'R1=({robot1_x:.2f},{robot1_y:.2f}), '
            f'R2=({robot2_x:.2f},{robot2_y:.2f}), '
            f'R3=({robot3_x:.2f},{robot3_y:.2f})'
        )


def main(args=None):

    rclpy.init(args=args)

    node = VirtualStructure()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()