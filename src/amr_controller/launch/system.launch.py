from launch import LaunchDescription
import os
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("amr_description"),
            "launch",
            "amr_gazebo.launch.py"
        )
    )

    controller = IncludeLaunchDescription(
        os.path.join(
        get_package_share_directory("amr_controller"),
        "launch",
        "controller.launch.py"
        )
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
        get_package_share_directory("amr_controller"),
        "launch",
        "joystick_teleop.launch.py"
        )
    )


    return LaunchDescription([
        gazebo,
        controller,
        joystick
    ])