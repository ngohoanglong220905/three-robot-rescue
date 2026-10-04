#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, TransformException


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


class PID:
    def __init__(self, kp, ki, kd, output_limit=None, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_limit = output_limit
        self.integral_limit = integral_limit

        self.integral = 0.0
        self.prev_error = 0.0
        self.initialized = False

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.initialized = False

    def compute(self, error, dt):
        if dt <= 0.0:
            return 0.0

        self.integral += error * dt

        if self.integral_limit is not None:
            self.integral = max(
                -self.integral_limit,
                min(self.integral, self.integral_limit)
            )

        if not self.initialized:
            derivative = 0.0
            self.initialized = True
        else:
            derivative = (error - self.prev_error) / dt

        self.prev_error = error

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        if self.output_limit is not None:
            output = max(
                -self.output_limit,
                min(output, self.output_limit)
            )

        return output


class PIDPathFollower(Node):

    def __init__(self):
        super().__init__('pid_path_follower')

        # ==========================================
        # Parameters
        # ==========================================

        self.declare_parameter('kp_x', 1)
        self.declare_parameter('ki_x', 0.0)
        self.declare_parameter('kd_x', 0.05)

        self.declare_parameter('kp_y', 1)
        self.declare_parameter('ki_y', 0.0)
        self.declare_parameter('kd_y', 0.05)

        self.declare_parameter('kp_theta', 1.2)
        self.declare_parameter('ki_theta', 0.01)
        self.declare_parameter('kd_theta', 0.09)

        self.declare_parameter('max_vx', 0.35)
        self.declare_parameter('max_vy', 0.35)
        self.declare_parameter('max_omega', 1.0)

        self.declare_parameter('position_tolerance', 0.08)
        self.declare_parameter('goal_tolerance', 0.10)

        self.declare_parameter('lookahead_distance', 0.20)

        # ==========================================
        # PID controllers
        # ==========================================

        self.pid_x = PID(
            self.get_parameter('kp_x').value,
            self.get_parameter('ki_x').value,
            self.get_parameter('kd_x').value,
            output_limit=self.get_parameter('max_vx').value,
            integral_limit=1.0
        )

        self.pid_y = PID(
            self.get_parameter('kp_y').value,
            self.get_parameter('ki_y').value,
            self.get_parameter('kd_y').value,
            output_limit=self.get_parameter('max_vy').value,
            integral_limit=1.0
        )

        self.pid_theta = PID(
            self.get_parameter('kp_theta').value,
            self.get_parameter('ki_theta').value,
            self.get_parameter('kd_theta').value,
            output_limit=self.get_parameter('max_omega').value,
            integral_limit=1.0
        )

        # ==========================================
        # ROS
        # ==========================================

        self.path_sub = self.create_subscription(
            Path,
            '/astar_path',
            self.path_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # ==========================================
        # TF
        # ==========================================

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ==========================================
        # State
        # ==========================================

        self.path = None
        self.target_index = 0

        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(
            0.02,
            self.control_loop
        )

        self.get_logger().info(
            'PID path follower started.'
        )

    # ==========================================
    # PATH CALLBACK
    # ==========================================

    def path_callback(self, msg):

        if len(msg.poses) < 2:
            self.path = None
            self.stop_robot()
            return

        self.path = msg
        self.target_index = 0

        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_theta.reset()

        self.get_logger().info(
            f'Received A* path with '
            f'{len(msg.poses)} poses'
        )

    # ==========================================
    # GET ROBOT POSE
    # ==========================================

    def get_robot_pose(self):

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            q = transform.transform.rotation

            siny_cosp = (
                2.0 * (q.w * q.z + q.x * q.y)
            )

            cosy_cosp = (
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )

            yaw = math.atan2(
                siny_cosp,
                cosy_cosp
            )

            return x, y, yaw

        except TransformException:

            try:
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    'base_footprint',
                    rclpy.time.Time()
                )

                x = transform.transform.translation.x
                y = transform.transform.translation.y

                q = transform.transform.rotation

                siny_cosp = (
                    2.0 * (q.w * q.z + q.x * q.y)
                )

                cosy_cosp = (
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                )

                yaw = math.atan2(
                    siny_cosp,
                    cosy_cosp
                )

                return x, y, yaw

            except TransformException as exc:

                self.get_logger().warn(
                    f'Cannot get robot pose: {exc}'
                )

                return None

    # ==========================================
    # FIND LOOKAHEAD TARGET
    # ==========================================

    def find_target(self, robot_x, robot_y):

        if self.path is None:
            return None

        poses = self.path.poses

        # Find closest point
        closest_index = self.target_index
        closest_distance = float('inf')

        for i in range(
            self.target_index,
            len(poses)
        ):

            px = poses[i].pose.position.x
            py = poses[i].pose.position.y

            distance = math.hypot(
                px - robot_x,
                py - robot_y
            )

            if distance < closest_distance:
                closest_distance = distance
                closest_index = i

        # Find lookahead point
        lookahead = self.get_parameter(
            'lookahead_distance'
        ).value

        target_index = closest_index

        for i in range(
            closest_index,
            len(poses)
        ):

            px = poses[i].pose.position.x
            py = poses[i].pose.position.y

            distance = math.hypot(
                px - robot_x,
                py - robot_y
            )

            if distance >= lookahead:
                target_index = i
                break

        self.target_index = target_index

        return poses[target_index]

    # ==========================================
    # CONTROL LOOP
    # ==========================================

    def control_loop(self):

        if self.path is None:
            return

        pose = self.get_robot_pose()

        if pose is None:
            self.stop_robot()
            return

        robot_x, robot_y, robot_yaw = pose

        # ======================================
        # Goal
        # ======================================

        final_pose = self.path.poses[-1].pose

        goal_dx = (
            final_pose.position.x - robot_x
        )

        goal_dy = (
            final_pose.position.y - robot_y
        )

        goal_distance = math.hypot(
            goal_dx,
            goal_dy
        )

        goal_tolerance = self.get_parameter(
            'goal_tolerance'
        ).value

        if goal_distance < goal_tolerance:

            self.stop_robot()

            self.get_logger().info(
                'Goal reached.'
            )

            self.path = None

            return

        # ======================================
        # Target
        # ======================================

        target_pose = self.find_target(
            robot_x,
            robot_y
        )

        if target_pose is None:
            self.stop_robot()
            return

        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y

        q = target_pose.pose.orientation

        target_yaw = math.atan2(
            2.0 * (q.w * q.z),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        # ======================================
        # Position error in WORLD
        # ======================================

        error_x = target_x - robot_x
        error_y = target_y - robot_y

        error_theta = normalize_angle(
            target_yaw - robot_yaw
        )

        # ======================================
        # Time
        # ======================================

        now = self.get_clock().now()

        dt = (
            now - self.last_time
        ).nanoseconds / 1e9

        self.last_time = now

        if dt <= 0.0 or dt > 0.5:
            dt = 0.02

        # ======================================
        # PID in WORLD FRAME
        # ======================================

        vx_world = self.pid_x.compute(
            error_x,
            dt
        )

        vy_world = self.pid_y.compute(
            error_y,
            dt
        )

        omega = self.pid_theta.compute(
            error_theta,
            dt
        )

        # ======================================
        # WORLD → ROBOT FRAME
        # ======================================

        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)

        vx_body = (
            cos_yaw * vx_world
            + sin_yaw * vy_world
        )

        vy_body = (
            -sin_yaw * vx_world
            + cos_yaw * vy_world
        )

        # ======================================
        # Publish
        # ======================================

        cmd = Twist()

        cmd.linear.x = vx_body
        cmd.linear.y = vy_body

        cmd.angular.z = omega

        self.cmd_pub.publish(cmd)

    # ==========================================
    # STOP
    # ==========================================

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = PIDPathFollower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
