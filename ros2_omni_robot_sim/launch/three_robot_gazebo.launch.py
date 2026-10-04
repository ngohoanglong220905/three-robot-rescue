import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources.python_launch_file_utilities import (
    load_python_launch_file_as_module,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node


PACKAGE_NAME = "ros2_omni_robot_sim"

_graphics = load_python_launch_file_as_module(
    os.path.join(os.path.dirname(__file__), '_graphics_backend.py')
)


def launch_gazebo(context, gazebo_launch_path, world_file):
    rendering, environment, diagnostic = _graphics.graphics_environment(
        LaunchConfiguration('gazebo_rendering').perform(context)
    )
    actions = [LogInfo(msg=f'Gazebo rendering: {rendering}')]
    if diagnostic:
        actions.append(LogInfo(msg=diagnostic))
    actions.append(GroupAction(
        scoped=True,
        actions=[
            *[SetEnvironmentVariable(name=key, value=value)
              for key, value in environment.items()],
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch_path),
                launch_arguments={
                    'gz_args': [world_file, ' -r -v 4'],
                    # A GUI failure can make the combined Gazebo command exit 0.
                    # Stop the launch even then, instead of waiting for absent
                    # world services and leaving RViz with no robots.
                    'on_exit_shutdown': 'true',
                }.items(),
            ),
        ],
    ))
    return actions


# ============================================================
# TẠO NODE CHO 1 ROBOT
# ============================================================

def generate_robot_nodes(
    package_path,
    robot_name,
    x,
    y,
    yaw,
):

    # --------------------------------------------------------
    # XACRO
    # --------------------------------------------------------

    xacro_file = os.path.join(
        package_path,
        "urdf",
        "3w_v2_multi",
        "main.urdf.xacro",
    )

    # QUAN TRỌNG:
    # Phải dùng Command() để robot_state_publisher nhận
    # chuỗi robot_description sau khi chạy xacro.
    robot_description = Command([
        "xacro ",
        xacro_file,
        " robot_prefix:=",
        robot_name,
    ])

    # --------------------------------------------------------
    # ROBOT STATE PUBLISHER
    # --------------------------------------------------------

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",

        namespace=robot_name,

        name="robot_state_publisher",

        output="screen",

        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
                "frame_prefix": robot_name + "/",
            }
        ],
    )

    # --------------------------------------------------------
    # SPAWN ROBOT
    # --------------------------------------------------------

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",

        arguments=[
            "-world",
            "my_maze",

            "-topic",
            "/" + robot_name + "/robot_description",

            "-name",
            robot_name,

            "-x",
            str(x) if isinstance(x, (int, float)) else x,

            "-y",
            str(y) if isinstance(y, (int, float)) else y,

            "-z",
            "0.1",

            "-Y",
            str(yaw) if isinstance(yaw, (int, float)) else yaw,
        ],

        output="screen",
    )

    # --------------------------------------------------------
    # CONTROLLER SPAWNER
    # --------------------------------------------------------

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",

        namespace=robot_name,

        # Đặt tên riêng để không còn cảnh báo
        # duplicate node name.
        name=robot_name + "_controller_spawner",

        arguments=[
            "joint_state_broadcaster",

            "wheel1_controller",
            "wheel2_controller",
            "wheel3_controller",

            "--controller-manager",
            "/" + robot_name + "/controller_manager",

            "--controller-manager-timeout",
            "60",
        ],

        output="screen",
    )

    # --------------------------------------------------------
    # OMNI KINEMATICS
    # --------------------------------------------------------

    kinematics = Node(
        package=PACKAGE_NAME,
        executable="kinematics",

        namespace=robot_name,

        name="omni_kinematics",

        parameters=[
            {
                "use_sim_time": True,
            }
        ],

        output="screen",
    )

    return (
        robot_state_publisher,
        spawn_robot,
        controller_spawner,
        kinematics,
    )


# ============================================================
# MAIN
# ============================================================

def generate_launch_description():

    package_path = get_package_share_directory(
        PACKAGE_NAME
    )

    # --------------------------------------------------------
    # ARGUMENT
    # --------------------------------------------------------

    world = LaunchConfiguration("world")

    # --------------------------------------------------------
    # WORLD FILE
    # --------------------------------------------------------

    world_file = os.path.join(
        package_path,
        "worlds",
        "my_maze.sdf",
    )

    # --------------------------------------------------------
    # GAZEBO RESOURCE PATH
    # --------------------------------------------------------

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",

        value=[
            package_path,
            ":",
            os.path.join(
                package_path,
                "worlds",
            ),
        ],
    )

    # --------------------------------------------------------
    # GAZEBO LAUNCH FILE
    # --------------------------------------------------------

    gazebo_launch_path = PathJoinSubstitution([
        get_package_share_directory("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py",
    ])

    gazebo = OpaqueFunction(
        function=launch_gazebo,
        args=[gazebo_launch_path, world_file],
    )

    # --------------------------------------------------------
    # BRIDGE
    # --------------------------------------------------------

    bridge_params = os.path.join(
        package_path,
        "config",
        "gz_bridge",
        "gz_bridge_3robots.yaml",
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",

        name="ros_gz_bridge",

        parameters=[
            {
                "config_file": bridge_params,
            }
        ],

        output="screen",
    )

    # ========================================================
    # ROBOT 1
    # ========================================================

    (
        robot1_state_publisher,
        robot1_spawn,
        robot1_controller_spawner,
        robot1_kinematics,
    ) = generate_robot_nodes(
        package_path,
        "robot1",
        LaunchConfiguration('robot1_x'),
        LaunchConfiguration('robot1_y'),
        LaunchConfiguration('robot1_yaw'),
    )

    # ========================================================
    # ROBOT 2
    # ========================================================

    (
        robot2_state_publisher,
        robot2_spawn,
        robot2_controller_spawner,
        robot2_kinematics,
    ) = generate_robot_nodes(
        package_path,
        "robot2",
        LaunchConfiguration('robot2_x'),
        LaunchConfiguration('robot2_y'),
        LaunchConfiguration('robot2_yaw'),
    )

    # ========================================================
    # ROBOT 3
    # ========================================================

    (
        robot3_state_publisher,
        robot3_spawn,
        robot3_controller_spawner,
        robot3_kinematics,
    ) = generate_robot_nodes(
        package_path,
        "robot3",
        LaunchConfiguration('robot3_x'),
        LaunchConfiguration('robot3_y'),
        LaunchConfiguration('robot3_yaw'),
    )

    # ========================================================
    # LAUNCH DESCRIPTION
    # ========================================================

    ld = LaunchDescription()
    for name, pose in (
        ('robot1', (3.5, -3.8, 1.57)),
        ('robot2', (3.5, -3.0, 1.57)),
        ('robot3', (3.5, -2.2, 1.57)),
    ):
        for axis, value in zip(('x', 'y', 'yaw'), pose):
            ld.add_action(DeclareLaunchArgument(f'{name}_{axis}', default_value=str(value)))

    # --------------------------------------------------------
    # ARGUMENTS
    # --------------------------------------------------------

    ld.add_action(
        DeclareLaunchArgument(
            "world",
            default_value="my_maze",
            description="Gazebo world",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            'gazebo_rendering',
            default_value='auto',
            choices=['auto', 'native', 'intel', 'software'],
            description='Graphics backend for Gazebo GUI and sensor rendering.',
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        )
    )

    # --------------------------------------------------------
    # ENVIRONMENT
    # --------------------------------------------------------

    ld.add_action(
        gz_resource_path
    )

    # --------------------------------------------------------
    # GAZEBO
    # --------------------------------------------------------

    ld.add_action(
        gazebo
    )

    # --------------------------------------------------------
    # Start descriptions before spawning so each create process can subscribe
    # to a live, namespaced robot_description topic.
    ld.add_action(
        TimerAction(
            period=1.0,
            actions=[
                ros_gz_bridge,
                robot1_state_publisher,
                robot2_state_publisher,
                robot3_state_publisher,
            ],
        )
    )

    def spawn_exit_handler(robot_name, follow_up_actions):
        def on_exit(event, context):
            if event.returncode == 0:
                return follow_up_actions
            reason = f'{robot_name} failed to spawn (exit code {event.returncode}).'
            return [
                LogInfo(msg=f'ERROR: {reason} Shutting down this launch.'),
                EmitEvent(event=Shutdown(reason=reason)),
            ]

        return on_exit

    # Spawn one model at a time. The next spawn and each robot's controllers
    # start only after Gazebo confirms the previous model was created.
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=robot1_spawn,
                on_exit=spawn_exit_handler(
                    'robot1',
                    [
                        robot1_kinematics,
                        robot1_controller_spawner,
                        TimerAction(period=1.0, actions=[robot2_spawn]),
                    ],
                ),
            )
        )
    )
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=robot2_spawn,
                on_exit=spawn_exit_handler(
                    'robot2',
                    [
                        robot2_kinematics,
                        robot2_controller_spawner,
                        TimerAction(period=1.0, actions=[robot3_spawn]),
                    ],
                ),
            )
        )
    )
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=robot3_spawn,
                on_exit=spawn_exit_handler(
                    'robot3',
                    [robot3_kinematics, robot3_controller_spawner],
                ),
            )
        )
    )

    # Give Gazebo a short startup window, then spawn robot1. Further robots are
    # launched by the successful-exit handlers above rather than fixed timers.
    ld.add_action(TimerAction(period=4.0, actions=[robot1_spawn]))

    return ld
