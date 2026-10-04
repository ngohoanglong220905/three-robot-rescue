"""Run my_maze with three 3w_v2 robots and D* Lite / leader-follower MPC."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
                    3.5, -3.8, 1.57,
                    3.5, -3.0, 1.57,
                    3.5, -2.2, 1.57,
                ],
                'leader_robot': LaunchConfiguration('leader_robot'),
                'adaptive_formation': ParameterValue(
                    LaunchConfiguration('adaptive_formation'), value_type=bool),
                'formation_triangle_half_width': 0.50,
                'formation_lookahead': 1.20,
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

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='three_robot_dstar_mpc_rviz',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('simulation'), value_type=bool)}],
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
                description='Leader robot; robot3 starts at the front of the spawned column.',
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                condition=IfCondition(LaunchConfiguration('simulation')),
            ),
            map_server,
            map_lifecycle,
            controller,
            *robot_controllers,
            rviz,
        ]
    )
