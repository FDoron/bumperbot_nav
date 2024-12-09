#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    bumperbot_navigation_pkg = get_package_share_directory('bumperbot_navigation')
    bumperbot_controller_pkg = get_package_share_directory('bumperbot_controller')
    
    # use_sim_time = LaunchConfiguration("use_sim_time")

    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # Fields2Cover Path Generation Node
    f2c_path_planner_node = Node(
        package='bumperbot_navigation',
        executable='f2c_path_planner',
        name='f2c_path_planner'
    )
    
    # Path Converter Node
    path_converter_node = Node(
        package='bumperbot_navigation',
        executable='path_converter',
        name='path_converter'
    )
    path_executor = Node(
            package='bumperbot_navigation',
            executable='path_executor',
            name='path_executor'
    )
    frame_verification = Node(
            package='bumperbot_navigation',
            executable='frame_verification',
            name='frame_verification'
    )
    
    # NAV2 twist relay node
    # nav2_twist_relay_node = Node(
    #     package='bumperbot_navigation',
    #     executable='nav2_twist_relay',
    #     name='nav2_twist_relay',
    #     parameters=[{"use_sim_time": True}],
    #     # remappings=[
    #     #     ("/cmd_vel", "/nav_vel")  # Relay Nav2's /cmd_vel to /nav_vel for twist_mux
    #     # ]
    # )

    return LaunchDescription([
        static_tf_publisher,
        # f2c_path_planner_node,
        # path_converter_node,
        # frame_verification,
        # path_executor,
        # nav2_twist_relay_node,
    ])