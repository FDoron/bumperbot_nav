import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Fields2Cover Path Generation Node
        Node(
            package='bumperbot_navigation',
            executable='f2c_path_planner',  # Remove .py extension
            name='f2c_path_planner'
        ),
        
        # Path Converter Node
        Node(
            package='bumperbot_navigation',
            executable='path_converter',    # Remove .py extension
            name='path_converter'
        ),
    ])
        # Nav2 Integration (when ready)
        # Node(
        #     package='nav2_bt_navigator',
        #     executable='bt_navigator',
        #     name='bt_navigator',
        #     parameters=[os.path.join(nav_pkg_dir, 'config', 'nav2_params.yaml')]
        # )
    # ])