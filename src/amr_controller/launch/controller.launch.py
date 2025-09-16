import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    amr_wheel_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["amr_controller",
                "--controller-manager",
                "/controller_manager"
        ]
    )


    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            amr_wheel_controller,
            
        ]
    )