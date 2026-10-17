import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    Command,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory(
        "robot_description"
    )
    sim_share = get_package_share_directory(
        "robot_sim"
    )
    ros_gz_sim_share = get_package_share_directory(
        "ros_gz_sim"
    )

    default_model = os.path.join(
        description_share,
        "urdf",
        "two_wheel_robot.urdf.xacro",
    )
    world = os.path.join(
        sim_share,
        "worlds",
        "empty.sdf",
    )

    model_argument = DeclareLaunchArgument(
        "model",
        default_value=default_model,
        description="IRIS robot Xacro file.",
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                LaunchConfiguration("model"),
            ]
        ),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": f"-r {world}",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "two_wheel_robot",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
        remappings=[
            ("/imu", "/imu/data"),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            model_argument,
            robot_state_publisher,
            gazebo,
            spawn_robot,
            bridge,
        ]
    )
