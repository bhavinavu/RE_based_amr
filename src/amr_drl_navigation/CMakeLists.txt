cmake_minimum_required(VERSION 3.8)
project(amr_drl_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(robot_localization REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(xacro REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch config goals_and_poses models 
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/go_to_pose_lidar.py
  ${PROJECT_NAME}/actor_critic_networks.py
  ${PROJECT_NAME}/env_utils.py
  ${PROJECT_NAME}/get_replay_buffer.py
  ${PROJECT_NAME}/initialize_logger.py
  ${PROJECT_NAME}/launch_utils.py
  ${PROJECT_NAME}/nav_metrics.py
  ${PROJECT_NAME}/normalizer.py
  ${PROJECT_NAME}/on_policy_trainer.py
  ${PROJECT_NAME}/periodic_ops.py
  ${PROJECT_NAME}/amr4rl_environment_lidar.py
  ${PROJECT_NAME}/amr4rl_lidar.py
  ${PROJECT_NAME}/policy_base.py
  ${PROJECT_NAME}/prepare_output_dir.py
  ${PROJECT_NAME}/sac.py
  ${PROJECT_NAME}/sensors.py
  ${PROJECT_NAME}/target_update_ops.py
  ${PROJECT_NAME}/tf2rl_env_utils.py
  ${PROJECT_NAME}/tfp_convmix_gaussian_actor.py
  ${PROJECT_NAME}/tfp_gaussian_actor.py
  ${PROJECT_NAME}/trainer.py
  ${PROJECT_NAME}/utils.py
  ${PROJECT_NAME}/message_converter.py
  ${PROJECT_NAME}/discount_cumsum.py
  DESTINATION lib/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
