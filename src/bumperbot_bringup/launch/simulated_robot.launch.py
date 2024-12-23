import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    # Initial pose arguments
    x_pose = LaunchConfiguration('x_pose', default='5.0')
    y_pose = LaunchConfiguration('y_pose', default='5.0')
    z_pose = LaunchConfiguration('z_pose', default='0.1')
    yaw_pose = LaunchConfiguration('yaw_pose', default='-1.57')

    # Pose arguments declaration
    declare_x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')
    declare_z_pose_arg = DeclareLaunchArgument('z_pose', default_value='0.0')
    declare_yaw_pose_arg = DeclareLaunchArgument('yaw_pose', default_value='0.0')

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_description"),
            "launch",
            "gazebo.launch.py"
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose,
            'yaw_pose': yaw_pose
        }.items()
    )
    
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )

    
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    safety_stop = Node(
        package="bumperbot_utils",
        executable="safety_stop",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    rviz_localization = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("bumperbot_localization"),
                "rviz",
                "global_localization.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=UnlessCondition(use_slam)
    )

    rviz_slam = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("bumperbot_mapping"),
                "rviz",
                "slam.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_slam)
    )

     # Static transforms for map to odom and base_link to base_footprint
    static_transforms = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['5.0', '5.0', '0', '0', '0', '0', 'map', 'odom'],
            name='map_to_odom_tf'
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        #     # arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        #     name='base_link_to_footprint_tf'
        # )
    ]


    coverage_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("opennav_coverage_demo"), 
                "launch", 
                "coverage_demo_launch.py"
            )
        )#,
        # launch_arguments={
        #     'x_pose': x_pose,
        #     'y_pose': y_pose,
        #     'z_pose': z_pose,
        #     'yaw_pose': yaw_pose
        # }.items()
    )
    
    

    return LaunchDescription([
        use_slam_arg,
        declare_x_pose_arg,
        declare_y_pose_arg,
        declare_z_pose_arg,
        declare_yaw_pose_arg,
        gazebo,
        controller,
        joystick,
        safety_stop,
        localization,
        slam,
        # rviz_localization,
        rviz_slam,
        *static_transforms,
        coverage_demo
    ])