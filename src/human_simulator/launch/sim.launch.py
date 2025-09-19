from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Start Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ]),
        launch_arguments={'gz_args': '-v 4 -r empty.sdf'}.items(),
    )

    # Random walker ROS node

    model_path = os.path.join(
        get_package_share_directory('human_simulator'),
        'human_model', 'actor.sdf'
    )

    # Spawn the human actor
    spawn_human = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-file', model_path,
             '-name', 'random_human'],
        output='screen'
    )

    # ROS ↔ Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/human/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen'
    )
    

    return LaunchDescription([
        gazebo,
        spawn_human,
        bridge
    ])