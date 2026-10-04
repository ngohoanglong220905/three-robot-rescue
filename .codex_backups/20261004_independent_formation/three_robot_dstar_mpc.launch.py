"""Run my_maze with three 3w_v2 robots and D* Lite / leader-follower MPC."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources.python_launch_file_utilities import (
    load_python_launch_file_as_module,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_graphics = load_python_launch_file_as_module(
    os.path.join(os.path.dirname(__file__), '_graphics_backend.py')
)


def _launch_rviz(context, rviz_config):
    """Select a GLX backend for RViz without changing Gazebo's environment."""
    rendering, environment, diagnostic = _graphics.graphics_environment(
        LaunchConfiguration('rviz_rendering').perform(context)
    )
    actions = []
    if diagnostic:
        actions.append(LogInfo(msg=diagnostic))

    actions.extend([
        LogInfo(msg=f'RViz rendering: {rendering}'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='three_robot_dstar_mpc_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': ParameterValue(
                LaunchConfiguration('simulation'), value_type=bool)}],
            additional_env=environment,
        ),
    ])
    return actions


def generate_launch_description():
    sim_share = get_package_share_directory('ros2_omni_robot_sim')
    sim_launch = os.path.join(
        sim_share, 'launch', 'three_robot_gazebo.launch.py'
    )
    map_yaml = os.path.join(sim_share, 'map', 'my_maze.yaml')
    rviz_config = os.path.join(
        sim_share, 'rviz', 'three_robot_dstar_mpc.rviz'
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'yaml_filename': map_yaml,
                'frame_id': 'map',
                'use_sim_time': ParameterValue(LaunchConfiguration('simulation'), value_type=bool),
            }
        ],
    )

    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='map_lifecycle_manager',
        output='screen',
        parameters=[
            {
                'autostart': True,
                'node_names': ['map_server'],
                'use_sim_time': ParameterValue(LaunchConfiguration('simulation'), value_type=bool),
            }
        ],
    )

    controller = Node(
        package='astar_planner',
        executable='three_robot_dstar_mpc',
        name='three_robot_dstar_mpc',
        output='screen',
        parameters=[
            {
                'use_sim_time': ParameterValue(LaunchConfiguration('simulation'), value_type=bool),
                'require_gazebo_pose': True,
                'robot_names': ['robot1', 'robot2', 'robot3'],
                'spawn_poses': [
                    4.0, -2.9, 1.57,
                    3.0, -2.9, 1.57,
                    3.5, -2.2, 1.57,
                ],
                'leader_robot': LaunchConfiguration('leader_robot'),
                'adaptive_formation': ParameterValue(
                    LaunchConfiguration('adaptive_formation'), value_type=bool),
                'formation_triangle_half_width': 0.50,
                'formation_lookahead': 1.20,
                'formation_transition_speed': ParameterValue(
                    LaunchConfiguration('formation_transition_speed'), value_type=float),
                'formation_open_hold': ParameterValue(
                    LaunchConfiguration('formation_open_hold'), value_type=float),
                'formation_follow_gap': 0.70,
                'formation_speed': 0.16,
                'narrow_speed': 0.08,
                'robot_radius': 0.15,
            }
        ],
    )

    robot_controllers = [
        Node(
            package='astar_planner',
            executable='robot_distributed_mpc',
            name=f'{robot_name}_distributed_mpc',
            output='screen',
            parameters=[
                {
                    'use_sim_time': ParameterValue(LaunchConfiguration('simulation'), value_type=bool),
                    'robot_name': robot_name,
                    'robot_names': ['robot1', 'robot2', 'robot3'],
                    'robot_radius': 0.15,
                    'preferred_clearance': 0.22,
                    'motion_enabled': ParameterValue(
                        LaunchConfiguration('motion_enabled'), value_type=bool
                    ),
                }
            ],
        )
        for robot_name in ('robot1', 'robot2', 'robot3')
    ]

    rviz = OpaqueFunction(
        function=_launch_rviz,
        args=[rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'adaptive_formation',
                default_value='true',
                description='Use triangle formation in open space and a column in narrow passages.',
            ),
            DeclareLaunchArgument(
                'leader_robot',
                default_value='robot3',
                description='Leader robot; robot3 starts at the front of the spawned triangle.',
            ),
            DeclareLaunchArgument(
                'formation_transition_speed', default_value='0.18',
                description='Reference speed in m/s while folding or opening the formation.',
            ),
            DeclareLaunchArgument(
                'formation_open_hold', default_value='0.60',
                description='Seconds of clear space before expanding from a column to a triangle.',
            ),
            DeclareLaunchArgument(
                'simulation',
                default_value='true',
                description='Launch Gazebo and spawn the three robots.',
            ),
            DeclareLaunchArgument(
                'motion_enabled',
                default_value='true',
                description='Send MPC velocity commands to all three robots.',
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='true',
                description='Open RViz with the map, paths, and robot position markers.',
            ),
            DeclareLaunchArgument(
                'rviz_rendering',
                default_value='auto',
                choices=['auto', 'native', 'intel', 'software'],
                description=(
                    'RViz GLX backend. Auto uses Intel, or software rendering, '
                    'if NVIDIA reports a driver/library version mismatch.'
                ),
            ),
            DeclareLaunchArgument(
                'gazebo_rendering',
                default_value='auto',
                choices=['auto', 'native', 'intel', 'software'],
                description='Graphics backend for both Gazebo GUI and sensor rendering.',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                condition=IfCondition(LaunchConfiguration('simulation')),
                launch_arguments={
                    'gazebo_rendering': LaunchConfiguration('gazebo_rendering'),
                    'robot1_x': '4.0', 'robot1_y': '-2.9', 'robot1_yaw': '1.57',
                    'robot2_x': '3.0', 'robot2_y': '-2.9', 'robot2_yaw': '1.57',
                    'robot3_x': '3.5', 'robot3_y': '-2.2', 'robot3_yaw': '1.57',
                }.items(),
            ),
            map_server,
            map_lifecycle,
            controller,
            *robot_controllers,
            rviz,
        ]
    )
