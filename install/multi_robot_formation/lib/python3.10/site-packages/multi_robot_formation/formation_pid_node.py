import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class RobotPID:
    def __init__(self, kp_x, kd_x, kp_y, kd_y, kp_theta, kd_theta):
        self.kp_x = kp_x
        self.kd_x = kd_x

        self.kp_y = kp_y
        self.kd_y = kd_y

        self.kp_theta = kp_theta
        self.kd_theta = kd_theta

        self.prev_ex = 0.0
        self.prev_ey = 0.0
        self.prev_eth = 0.0

    def reset(self):
        self.prev_ex = 0.0
        self.prev_ey = 0.0
        self.prev_eth = 0.0


class FormationPIDNode(Node):

    def __init__(self):
        super().__init__("formation_pid_node")

        # =================================================
        # PARAMETERS
        # =================================================

        self.declare_parameter("kp_x", 0.8)
        self.declare_parameter("kd_x", 0.05)

        self.declare_parameter("kp_y", 0.8)
        self.declare_parameter("kd_y", 0.05)

        self.declare_parameter("kp_theta", 1.2)
        self.declare_parameter("kd_theta", 0.05)

        self.declare_parameter("max_vx", 0.35)
        self.declare_parameter("max_vy", 0.35)
        self.declare_parameter("max_omega", 1.0)

        self.declare_parameter("goal_tolerance", 0.10)

        kp_x = float(self.get_parameter("kp_x").value)
        kd_x = float(self.get_parameter("kd_x").value)

        kp_y = float(self.get_parameter("kp_y").value)
        kd_y = float(self.get_parameter("kd_y").value)

        kp_theta = float(
            self.get_parameter("kp_theta").value
        )

        kd_theta = float(
            self.get_parameter("kd_theta").value
        )

        self.max_vx = float(
            self.get_parameter("max_vx").value
        )

        self.max_vy = float(
            self.get_parameter("max_vy").value
        )

        self.max_omega = float(
            self.get_parameter("max_omega").value
        )

        self.goal_tolerance = float(
            self.get_parameter("goal_tolerance").value
        )

        # =================================================
        # PID CONTROLLERS
        # =================================================

        self.pid = {
            "robot1": RobotPID(
                kp_x, kd_x,
                kp_y, kd_y,
                kp_theta, kd_theta
            ),

            "robot2": RobotPID(
                kp_x, kd_x,
                kp_y, kd_y,
                kp_theta, kd_theta
            ),

            "robot3": RobotPID(
                kp_x, kd_x,
                kp_y, kd_y,
                kp_theta, kd_theta
            )
        }

        # =================================================
        # TF
        # =================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # =================================================
        # PATH STORAGE
        # =================================================

        self.paths = {
            "robot1": None,
            "robot2": None,
            "robot3": None
        }

        # =================================================
        # SUBSCRIBERS
        # =================================================

        self.create_subscription(
            Path,
            "/robot1/desired_path",
            lambda msg: self.path_callback(
                "robot1",
                msg
            ),
            10
        )

        self.create_subscription(
            Path,
            "/robot2/desired_path",
            lambda msg: self.path_callback(
                "robot2",
                msg
            ),
            10
        )

        self.create_subscription(
            Path,
            "/robot3/desired_path",
            lambda msg: self.path_callback(
                "robot3",
                msg
            ),
            10
        )

        # =================================================
        # PUBLISHERS
        # =================================================

        self.cmd_publishers = {
            "robot1": self.create_publisher(
                Twist,
                "/robot1/cmd_vel",
                10
            ),

            "robot2": self.create_publisher(
                Twist,
                "/robot2/cmd_vel",
                10
            ),

            "robot3": self.create_publisher(
                Twist,
                "/robot3/cmd_vel",
                10
            )
        }

        # =================================================
        # CONTROL TIMER
        # =================================================

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            "3 Robot PID Formation Controller started"
        )

    # =====================================================
    # PATH CALLBACK
    # =====================================================

    def path_callback(self, robot_name, msg):
        self.paths[robot_name] = msg

    # =====================================================
    # CONTROL LOOP
    # =====================================================

    def control_loop(self):

        for robot_name in [
            "robot1",
            "robot2",
            "robot3"
        ]:

            path = self.paths[robot_name]

            if path is None:
                self.stop_robot(robot_name)
                continue

            if len(path.poses) == 0:
                self.stop_robot(robot_name)
                continue

            self.control_robot(
                robot_name,
                path
            )

    # =====================================================
    # CONTROL ONE ROBOT
    # =====================================================

    def control_robot(
        self,
        robot_name,
        path
    ):

        frame = f"{robot_name}/base_footprint"

        try:

            transform = self.tf_buffer.lookup_transform(
                "map",
                frame,
                rclpy.time.Time()
            )

        except (
            LookupException,
            ConnectivityException,
            ExtrapolationException
        ):

            self.stop_robot(robot_name)

            return

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        robot_yaw = self.quaternion_to_yaw(
            qx,
            qy,
            qz,
            qw
        )

        # =================================================
        # FIND LOOKAHEAD TARGET
        # =================================================

        target = self.find_target(
            robot_x,
            robot_y,
            path
        )

        if target is None:
            self.stop_robot(robot_name)
            return

        tx = target.pose.position.x
        ty = target.pose.position.y

        target_yaw = self.quaternion_to_yaw(
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w
        )

        # =================================================
        # ERROR
        # =================================================

        ex_world = tx - robot_x
        ey_world = ty - robot_y

        eth = self.normalize_angle(
            target_yaw - robot_yaw
        )

        # =================================================
        # WORLD -> ROBOT FRAME
        # =================================================

        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)

        ex = (
            cos_yaw * ex_world
            + sin_yaw * ey_world
        )

        ey = (
            -sin_yaw * ex_world
            + cos_yaw * ey_world
        )

        # =================================================
        # PID
        # =================================================

        pid = self.pid[robot_name]

        dt = 0.05

        dex = (
            ex - pid.prev_ex
        ) / dt

        dey = (
            ey - pid.prev_ey
        ) / dt

        deth = (
            eth - pid.prev_eth
        ) / dt

        vx = (
            pid.kp_x * ex
            + pid.kd_x * dex
        )

        vy = (
            pid.kp_y * ey
            + pid.kd_y * dey
        )

        omega = (
            pid.kp_theta * eth
            + pid.kd_theta * deth
        )

        pid.prev_ex = ex
        pid.prev_ey = ey
        pid.prev_eth = eth

        # =================================================
        # LIMIT
        # =================================================

        vx = max(
            -self.max_vx,
            min(self.max_vx, vx)
        )

        vy = max(
            -self.max_vy,
            min(self.max_vy, vy)
        )

        omega = max(
            -self.max_omega,
            min(self.max_omega, omega)
        )

        # =================================================
        # GOAL STOP
        # =================================================

        distance = math.hypot(
            ex_world,
            ey_world
        )

        if (
            distance < self.goal_tolerance
            and
            len(path.poses) <= 2
        ):

            vx = 0.0
            vy = 0.0
            omega = 0.0

            pid.reset()

        # =================================================
        # PUBLISH CMD_VEL
        # =================================================

        cmd = Twist()

        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = omega

        self.cmd_publishers[
            robot_name
        ].publish(cmd)

    # =====================================================
    # LOOKAHEAD
    # =====================================================

    def find_target(
        self,
        robot_x,
        robot_y,
        path
    ):

        lookahead = 0.20

        best = None

        for pose in path.poses:

            dx = (
                pose.pose.position.x
                - robot_x
            )

            dy = (
                pose.pose.position.y
                - robot_y
            )

            distance = math.hypot(
                dx,
                dy
            )

            if distance >= lookahead:
                best = pose
                break

        if best is None:
            best = path.poses[-1]

        return best

    # =====================================================
    # QUATERNION -> YAW
    # =====================================================

    @staticmethod
    def quaternion_to_yaw(
        x,
        y,
        z,
        w
    ):

        siny_cosp = (
            2.0 * (
                w * z
                +
                x * y
            )
        )

        cosy_cosp = (
            1.0
            -
            2.0 * (
                y * y
                +
                z * z
            )
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

    # =====================================================
    # NORMALIZE ANGLE
    # =====================================================

    @staticmethod
    def normalize_angle(angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    # =====================================================
    # STOP
    # =====================================================

    def stop_robot(self, robot_name):

        cmd = Twist()

        self.cmd_publishers[
            robot_name
        ].publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = FormationPIDNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

