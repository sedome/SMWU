# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License")
#
# Author: Darby Lim

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    # --- Launch Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    f1tenth_cartographer_prefix = get_package_share_directory('f1tenth_cartographer')
    cartographer_config_dir = LaunchConfiguration(
        'cartographer_config_dir',
        default=os.path.join(f1tenth_cartographer_prefix, 'config')
    )
    configuration_basename = LaunchConfiguration(
        'configuration_basename',
        default='f1tenth_2d.lua'
    )
    
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    
    rviz_config_file = os.path.join(f1tenth_cartographer_prefix, 'rviz', 'f110_cartographer.rviz')

    # --- Launch Description ---
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true', description='Launch RViz'
        ),
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to Cartographer config directory'
        ),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Lua configuration file for Cartographer'
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of occupancy grid'
        ),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'
        ),

        # Cartographer Node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('/odom', '/ego_racecar/odom'),
                ('/scan', '/ego_racecar/laser') # Corrected remapping: Cartographer expects '/scan', but your robot publishes on '/ego_racecar/laser'
            ],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ]
        ),

        # Occupancy Grid Node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'resolution': resolution,
                'publish_period_sec': publish_period_sec
            }.items()
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
