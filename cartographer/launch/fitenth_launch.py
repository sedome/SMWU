from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 기본 런치 인자 설정
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    f1tenth_cartographer_prefix = get_package_share_directory('f1tenth_cartographer')
    cartographer_config_dir = LaunchConfiguration(
        'cartographer_config_dir',
        default=os.path.join(f1tenth_cartographer_prefix, 'config')
    )
    configuration_basename = LaunchConfiguration(
        'configuration_basename',
        default='f1tenth_2d.lua'
    )
    rviz_config_file = os.path.join(f1tenth_cartographer_prefix, 'rviz', 'f110_cartographer.rviz')

    # URDF 로딩
    urdf_file = os.path.join(f1tenth_cartographer_prefix, 'urdf', 'F1_hokuyo.urdf')
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz'),
        DeclareLaunchArgument('cartographer_config_dir', default_value=cartographer_config_dir, description='Path to Cartographer config directory'),
        DeclareLaunchArgument('configuration_basename', default_value=configuration_basename, description='Lua config file for Cartographer'),
        DeclareLaunchArgument('resolution', default_value=resolution, description='Grid resolution'),
        DeclareLaunchArgument('publish_period_sec', default_value=publish_period_sec, description='Map publish period'),

        # Cartographer SLAM Node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ],
        ),

        # Occupancy Grid Node
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec],
            remappings=[
            ],
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen'
        ),
    ])
