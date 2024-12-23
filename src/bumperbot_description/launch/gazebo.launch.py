import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bumperbot_description = get_package_share_directory("bumperbot_description")

    # Declare pose arguments in the top-level launch file
    declare_x_pose_arg = DeclareLaunchArgument("x_pose", default_value="0.0", description="Initial X position")
    declare_y_pose_arg = DeclareLaunchArgument("y_pose", default_value="0.0", description="Initial Y position")
    declare_z_pose_arg = DeclareLaunchArgument("z_pose", default_value="0.0", description="Initial Z position")
    declare_yaw_pose_arg = DeclareLaunchArgument("yaw_pose", default_value="0.0", description="Initial yaw angle")


    model_arg = DeclareLaunchArgument(
        name="model", default_value=os.path.join(
                bumperbot_description, "urdf", "bumperbot.urdf.xacro"
            ),
        description="Absolute path to robot urdf file"
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")

    world_path = PathJoinSubstitution([
            bumperbot_description,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]
    )

    model_path = str(Path(bumperbot_description).parent.resolve())
    model_path += pathsep + os.path.join(get_package_share_directory("bumperbot_description"), 'models')

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path
        )

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments={
                    "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
                }.items()
             )

    # gz_spawn_entity = Node(
    #     package="ros_gz_sim",
    #     executable="create",
    #     output="screen",
    #     arguments=["-topic", "robot_description",
    #                "-name", "bumperbot"],
    # )

    # Added initial position for integration with opennav_coverage
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "bumperbot",
            "-x", LaunchConfiguration("x_pose"),
            "-y", LaunchConfiguration("y_pose"),
            "-z", LaunchConfiguration("z_pose"),
            "-Y", LaunchConfiguration("yaw_pose"),
        ],
    )



    # ROS-Gazebo Bridge Node with Config File
    bridge_params = os.path.join(get_package_share_directory("bumperbot_description"),'config','gz_bridge.yaml')
    ros_gz_bridge_with_yaml = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    # Additional ROS-Gazebo Bridge Topics

    ros_gz_additional_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        ],
        remappings=[
            ('/imu', '/imu/out')
        ]
    )
    

    # Image Bridge for Camera
    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )



    return LaunchDescription([
        model_arg,
        # Declare pose arguments in the top-level launch file
        declare_x_pose_arg,
        declare_y_pose_arg,
        declare_z_pose_arg,
        declare_yaw_pose_arg,
        world_name_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        # gz_ros2_bridge,
        ros_gz_bridge_with_yaml,
        ros_gz_additional_bridge,
        ros_gz_image_bridge,
    ])
