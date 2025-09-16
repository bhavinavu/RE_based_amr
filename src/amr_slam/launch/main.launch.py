import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("amr_description"),
            "launch",
            "amr_gazebo.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("amr_controller"),
            "launch",
            "controller.launch.py"
        ),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("amr_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("amr_slam"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("amr_slam"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", ""],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    
    return LaunchDescription([
        use_slam_arg,
        gazebo,
        controller,
        joystick,
        localization,
        slam,
        rviz_node
    ])
