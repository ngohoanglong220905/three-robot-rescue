"""Run my_maze with three 3w_v2 robots and D* Lite / MPC formation control."""

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
                'use_sim_time': False,
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
                'use_sim_time': False,
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
                # Independent wall-clock timers keep fallback markers alive if
                # Gazebo or /clock has not started.
                'use_sim_time': False,
                # Each robot's separate ROS node owns its local MPC loop.
                'motion_enabled': False,
                'robot_names': ['robot1', 'robot2', 'robot3'],
                'spawn_poses': [
                    3.5, -3.8, 1.57,
                    3.5, -3.0, 1.57,
                    3.5, -2.2, 1.57,
                ],
                'formation_half_width': 0.8,
                'formation_min_width': 0.35,
            }
        ],
    )

    spawn_poses = [
        ('robot1', [3.5, -3.8, 1.57]),
        ('robot2', [3.5, -3.0, 1.57]),
        ('robot3', [3.5, -2.2, 1.57]),
    ]
    robot_controllers = [
        Node(
            package='astar_planner',
            executable='robot_distributed_mpc',
            name=f'{robot_name}_distributed_mpc',
            output='screen',
            parameters=[
                {
                    'use_sim_time': False,
                    'robot_name': robot_name,
                    'robot_names': ['robot1', 'robot2', 'robot3'],
                    'spawn_pose': spawn_pose,
                    'motion_enabled': ParameterValue(
                        LaunchConfiguration('motion_enabled'), value_type=bool
                    ),
                }
            ],
        )
        for robot_name, spawn_pose in spawn_poses
    ]

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='three_robot_dstar_mpc_rviz',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(
        [
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
