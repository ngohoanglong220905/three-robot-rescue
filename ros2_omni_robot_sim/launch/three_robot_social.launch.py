"""Open social activity experiment, with the previous room scene selectable."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('ros2_omni_robot_sim')
    world = PythonExpression(["'social_people' if '", LaunchConfiguration('social_scene'),
                              "' == 'people' else 'social_rescue'"])

    def asset(directory, people_name, rooms_name):
        return PythonExpression([repr(os.path.join(share, directory, people_name)), " if '",
                                 LaunchConfiguration('social_scene'), "' == 'people' else ",
                                 repr(os.path.join(share, directory, rooms_name))])

    return LaunchDescription([
        DeclareLaunchArgument('social_scene', default_value='people', choices=['people', 'rooms']),
        DeclareLaunchArgument('scenario_annotations', default_value='true',
                              choices=['true', 'false'],
                              description='Publish authored scene markers, not camera detections.'),
        DeclareLaunchArgument('simulation', default_value='true'),
        DeclareLaunchArgument('motion_enabled', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('leader_robot', default_value='robot3', choices=['robot1', 'robot2', 'robot3']),
        DeclareLaunchArgument('leader_camera', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('person_detection', default_value='true', choices=['true', 'false'],
                              description='Detect people from leader RGB and locate them using depth/TF.'),
        DeclareLaunchArgument('person_confidence', default_value='0.35'),
        DeclareLaunchArgument('person_detection_rate', default_value='4.0',
                              description='Maximum inference frequency in wall time; latest frame only.'),
        DeclareLaunchArgument('person_input_size', default_value='416',
                              description='DNN input size; multiple of 32, e.g. 320, 416, 608.'),
        DeclareLaunchArgument('person_cpu_threads', default_value='2'),
        DeclareLaunchArgument('head_pose_enabled', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('body_pose_enabled', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('body_input_size', default_value='256'),
        DeclareLaunchArgument('body_max_people', default_value='3'),
        DeclareLaunchArgument('social_analysis_window', default_value='10.0'),
        DeclareLaunchArgument('leg_detection', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('social_avoidance', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('social_zone_memory', default_value='60.0'),
        DeclareLaunchArgument('social_protect_uncertain', default_value='false', choices=['true', 'false'],
                              description='Deprecated: proximity alone no longer creates social regions.'),
        DeclareLaunchArgument('social_provisional_memory', default_value='20.0'),
        DeclareLaunchArgument('social_group_distance', default_value='2.5',
                              description='Nearby peer distance for one-way attention and shared viewing (metres).'),
        DeclareLaunchArgument('social_group_margin', default_value='0.45'),
        DeclareLaunchArgument('social_viewing_length', default_value='3.2'),
        DeclareLaunchArgument('social_tv_distance', default_value='4.5',
                              description='Maximum distance from each viewer to the observed TV/colour target.'),
        DeclareLaunchArgument('social_color_attention', default_value='true', choices=['true', 'false'],
                              description='Protect common directions towards a nearby RGB-D TV or multicolour object.'),
        DeclareLaunchArgument('social_color_person_memory', default_value='75.0'),
        DeclareLaunchArgument('social_color_object_memory', default_value='75.0'),
        DeclareLaunchArgument('social_color_zone_memory', default_value='90.0'),
        DeclareLaunchArgument('social_observation', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('social_auto_observe', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('social_observe_cooldown', default_value='30.0'),
        DeclareLaunchArgument('social_prediction_horizon', default_value='1.5'),
        DeclareLaunchArgument('social_inferred_focus', default_value='false', choices=['true', 'false'],
                              description='Deprecated: viewing requires an observed multicolour target.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'three_robot_dstar_mpc.launch.py')),
            launch_arguments={
                'world': world,
                'map_yaml': asset('map', 'social_people.yaml', 'social_rescue.yaml'),
                'bridge_config': asset('config/gz_bridge', 'gz_bridge_social_people.yaml', 'gz_bridge_social.yaml'),
                'rviz_config': os.path.join(share, 'rviz', 'three_robot_social.rviz'),
                'simulation': LaunchConfiguration('simulation'),
                'motion_enabled': LaunchConfiguration('motion_enabled'),
                'leader_robot': LaunchConfiguration('leader_robot'),
                'leader_camera': LaunchConfiguration('leader_camera'),
                'observation_analysis_time': PythonExpression([
                    "float('", LaunchConfiguration('social_analysis_window'), "') + 2.0 if '",
                    LaunchConfiguration('body_pose_enabled'), "' == 'true' else 0.0",
                ]),
                'social_observation_enabled': PythonExpression([
                    "str('", LaunchConfiguration('social_observation'), "' == 'true' and '",
                    LaunchConfiguration('social_avoidance'), "' == 'true' and '",
                    LaunchConfiguration('person_detection'), "' == 'true' and '",
                    LaunchConfiguration('leader_camera'), "' == 'true' and '",
                    LaunchConfiguration('motion_enabled'), "' == 'true').lower()",
                ]),
                'planning_map_topic': PythonExpression([
                    "'/social_navigation/map' if '", LaunchConfiguration('social_avoidance'),
                    "' == 'true' else '/map'",
                ]),
            }.items(),
        ),
        Node(
            package='ros2_omni_robot_sim', executable='leader_person_detector.py',
            name='leader_person_detector', output='screen',
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('person_detection'), "' == 'true' and '",
                LaunchConfiguration('leader_camera'), "' == 'true'",
            ])),
            parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('simulation'), value_type=bool),
                'confidence': ParameterValue(LaunchConfiguration('person_confidence'), value_type=float),
                'inference_rate': ParameterValue(LaunchConfiguration('person_detection_rate'), value_type=float),
                'input_size': ParameterValue(LaunchConfiguration('person_input_size'), value_type=int),
                'cpu_threads': ParameterValue(LaunchConfiguration('person_cpu_threads'), value_type=int),
                'head_pose_enabled': ParameterValue(LaunchConfiguration('head_pose_enabled'), value_type=bool),
                'body_pose_enabled': ParameterValue(LaunchConfiguration('body_pose_enabled'), value_type=bool),
                'body_input_size': ParameterValue(LaunchConfiguration('body_input_size'), value_type=int),
                'body_max_people': ParameterValue(LaunchConfiguration('body_max_people'), value_type=int),
                'colorful_targets_enabled': ParameterValue(LaunchConfiguration('social_color_attention'), value_type=bool),
            }],
        ),
        Node(
            package='ros2_omni_robot_sim', executable='human_activity_analysis.py',
            name='human_activity_analysis', output='screen',
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('body_pose_enabled'), "' == 'true' and '",
                LaunchConfiguration('person_detection'), "' == 'true' and '",
                LaunchConfiguration('leader_camera'), "' == 'true'",
            ])),
            parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('simulation'), value_type=bool),
                'analysis_window': ParameterValue(LaunchConfiguration('social_analysis_window'), value_type=float),
            }],
        ),
        Node(
            package='ros2_omni_robot_sim', executable='leader_leg_candidates.py',
            name='leader_leg_candidates', output='screen',
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('leg_detection'), "' == 'true' and '",
                LaunchConfiguration('leader_camera'), "' == 'true'",
            ])),
            parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('simulation'), value_type=bool),
                'leader_robot': LaunchConfiguration('leader_robot'),
            }],
        ),
        Node(
            package='ros2_omni_robot_sim', executable='social_navigation.py',
            name='social_navigation', output='screen',
            condition=IfCondition(LaunchConfiguration('social_avoidance')),
            parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('simulation'), value_type=bool),
                'zone_memory': ParameterValue(LaunchConfiguration('social_zone_memory'), value_type=float),
                'protect_uncertain_groups': ParameterValue(LaunchConfiguration('social_protect_uncertain'), value_type=bool),
                'provisional_memory': ParameterValue(LaunchConfiguration('social_provisional_memory'), value_type=float),
                'nearby_group_distance': ParameterValue(LaunchConfiguration('social_group_distance'), value_type=float),
                'group_margin': ParameterValue(LaunchConfiguration('social_group_margin'), value_type=float),
                'viewing_length': ParameterValue(LaunchConfiguration('social_viewing_length'), value_type=float),
                'nearby_tv_distance': ParameterValue(LaunchConfiguration('social_tv_distance'), value_type=float),
                'color_attention_enabled': ParameterValue(LaunchConfiguration('social_color_attention'), value_type=bool),
                'color_person_memory': ParameterValue(LaunchConfiguration('social_color_person_memory'), value_type=float),
                'color_object_memory': ParameterValue(LaunchConfiguration('social_color_object_memory'), value_type=float),
                'color_zone_memory': ParameterValue(LaunchConfiguration('social_color_zone_memory'), value_type=float),
                'allow_inferred_focus': ParameterValue(LaunchConfiguration('social_inferred_focus'), value_type=bool),
                'leader_robot': LaunchConfiguration('leader_robot'),
                'require_activity_window': ParameterValue(LaunchConfiguration('body_pose_enabled'), value_type=bool),
                'auto_observe': ParameterValue(PythonExpression([
                    "'", LaunchConfiguration('social_auto_observe'), "' == 'true' and '",
                    LaunchConfiguration('social_observation'), "' == 'true' and '",
                    LaunchConfiguration('person_detection'), "' == 'true' and '",
                    LaunchConfiguration('leader_camera'), "' == 'true' and '",
                    LaunchConfiguration('motion_enabled'), "' == 'true'",
                ]), value_type=bool),
                'observe_cooldown': ParameterValue(LaunchConfiguration('social_observe_cooldown'), value_type=float),
                'prediction_horizon': ParameterValue(LaunchConfiguration('social_prediction_horizon'), value_type=float),
            }],
        ),
        Node(
            package='ros2_omni_robot_sim', executable='social_scene_markers.py',
            name='social_scene_markers', output='screen',
            condition=IfCondition(LaunchConfiguration('scenario_annotations')),
            parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('simulation'), value_type=bool),
                'scene_file': asset('config', 'social_people_scene.json', 'social_rescue_scene.json'),
            }],
        ),
    ])
