import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from os import pathsep
import launch
import yaml
import json
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue

configFilepath = os.path.join(
    get_package_share_directory("amr_drl_navigation"), 'config',
    'main_params.yaml'
    )
with open(configFilepath, 'r') as file:
    configParams = yaml.safe_load(file)['main_node']['ros__parameters']

# Fetching Goals and Poses
goals_path = os.path.join(
    get_package_share_directory("amr_drl_navigation"), 
    'goal_and_poses', 
    configParams['data_path']
    )

goal_and_poses = json.load(open(goals_path,'r'))
robot_pose, goal_pose = goal_and_poses["initial_pose"], goal_and_poses["goals"][0]

x_rob = '-x '+str(robot_pose[0])
y_rob = '-y '+str(robot_pose[1])
z_rob = '-z '+str(0.3)
yaw_rob = '-Y ' +str(robot_pose[2])

x_goal = '-x '+str(goal_pose[0])
y_goal = '-y '+str(goal_pose[1])
# z_goal = '-z 0.01'


goal_entity = os.path.join(get_package_share_directory("amr_drl_navigation"), 'models', 
            'goal_box', 'model.sdf')

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time', default_value='true', description='Use simulation clock if true'
    )
    
    amr_description = get_package_share_directory("amr_description")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        amr_description, "urdf", "amr.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file"
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="small_warehouse")

    world_path = PathJoinSubstitution([
            amr_description,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]
    )

    model_path = str(Path(amr_description).parent.resolve())
    model_path += pathsep + os.path.join(get_package_share_directory("amr_description"), 'models')


    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path 
        )
    
    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model")
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"
        ]),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_path, " -v 4 '"])
        }.items()
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "amr",
            "-x", str(robot_pose[0]),
            "-y", str(robot_pose[1]),
            "-z", "0.3",
            "-Y", str(robot_pose[2])
        ]
    )

    spawn_goal = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-file", goal_entity,
            "-name", "goal",
            "-x", str(goal_pose[0]),
            "-y", str(goal_pose[1]),
            "-z", "0.1"
        ]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/model/amr/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            "/world/default/create@ros_gz_interfaces/srv/SpawnEntity",
            "/world/default/remove@ros_gz_interfaces/srv/DeleteEntity",
            "/world/default/control@ros_gz_interfaces/srv/ControlWorld",
            "/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"]
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("amr_controller"),
            "launch",
            "controller.launch.py"
        ),
    )

    message_converter_node = Node(
        package="amr_drl_navigation",
        executable="message_converter.py",
        parameters= [{'use_sim_time': LaunchConfiguration("use_sim_time")}]

    )
    
    
    return launch.LaunchDescription([
        use_sim_time_arg,
        model_arg,
        world_name_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        spawn_robot,
        spawn_goal,
        gz_ros2_bridge,
        controller,
        message_converter_node
    ])