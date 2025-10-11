from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Config and args
    localize_config = os.path.join(
        get_package_share_directory('particle_filter'),
        'config',
        'localize.yaml'
    )
    localize_config_dict = yaml.safe_load(open(localize_config, 'r'))
    map_name = localize_config_dict['map_server']['ros__parameters']['map']
    
    # RViz config file path (using your pf.rviz)
    rviz_config_file = os.path.join(
        get_package_share_directory('particle_filter'),
        'rviz',
        'pf.rviz'
    )
    
    localize_la = DeclareLaunchArgument(
        'localize_config',
        default_value=localize_config,
        description='Localization configs'
    )
    
    ld = LaunchDescription([localize_la])

    # Nodes
    pf_node = Node(
        package='particle_filter',
        executable='particle_filter',
        #executable='particle_filter_imu',
        name='particle_filter',
        parameters=[LaunchConfiguration('localize_config')]
    )
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': os.path.join(get_package_share_directory('particle_filter'), 'maps', map_name + '.yaml')},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': True}]
    )

    
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    # Add actions to LaunchDescription
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(pf_node)
    ld.add_action(rviz_node)

    return ld

