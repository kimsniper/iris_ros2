import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory(
        "robot_bringup"
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_share,
                "launch",
                "sim.launch.py",
            )
        )
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    wheel_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "wheel_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    estimator = Node(
        package="robot_estimation",
        executable="robot_estimator",
        parameters=[
            {
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    observation = Node(
        package="robot_control",
        executable="observation_node",
        parameters=[
            {
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    control_share = get_package_share_directory(
        "robot_control"
    )

    balance = Node(
        package="robot_control",
        executable="balance_node",
        parameters=[
            os.path.join(
                control_share,
                "config",
                "balance_params.yaml",
            ),
            {
                "use_sim_time": True,
            },
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            simulation,
            joint_state_broadcaster,
            wheel_controller,
            estimator,
            observation,
            balance,
        ]
    )
