# 🤖 MOBILE_ROBOT: Autonomous Mobile Robot (AMR) System

This repository provides the complete software stack for an **Autonomous Mobile Robot (AMR)**, featuring advanced modules for **Perception**, **SLAM**, and **Deep Reinforcement Learning (DRL)-based Navigation**.  
The system is developed and tested in **ROS 2 Jazzy Jalisco** with **Gazebo Harmonic**.

---

## ✨ Features

The project is organized into modular ROS 2 packages:

| Package | Functionality | Key Technologies |
| :--- | :--- | :--- |
| `amr_perception` | Real-time object detection & recognition | **YOLOv8 (Ultralytics)** integrated with Gazebo |
| `amr_slam` | Mapping and localization | **ROS 2 SLAM Toolbox** |
| `amr_drl_navigation` | Training & deployment of navigation policy | **Deep Reinforcement Learning (SAC)** with **TensorFlow** & **Gymnasium** |
| `amr_controller` | Low-level robot control & ROS 2 interfaces | **ROS 2 Control** |
| `amr_description` | Robot URDF/XACRO description | Defines physical structure, sensors, kinematics |
| `human_simulator` | (Optional) Human-robot interaction simulation | Human models for dynamic obstacle scenarios *(work in progress: random walk & trajectories)* |

---

## 🚀 Getting Started

### ✅ Prerequisites

1. **Operating System:** Ubuntu 24.04 (Noble Numbat)  
2. **ROS Distribution:** ROS 2 Jazzy Jalisco  
3. **Simulation Environment:** Gazebo Harmonic (`ros_gz_sim`)  
4. **Python Libraries:**  
   - `ultralytics` (YOLOv8)  
   - `tensorflow`  
   - `gymnasium`  
   - `numpy`, `matplotlib`, `opencv`  

---

### ⚙️ Installation

1 Clone the Repository
   ```bash
   git clone https://github.com/bhavinavu/RE_based_amr.git
   cd RE_based_amr

2 Install ROS 2 Dependencies

    sudo apt install ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
     ros-jazzy-ros2-control ros-jazzy-ros-gz-sim \
     ros-jazzy-robot-state-publisher ros-jazzy-rviz2 \
     ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro \
     ros-jazzy-key-teleop ros-jazzy-twist-mux \
     ros-jazzy-joy-teleop ros-jazzy-joy ros-jazzy-cv-bridge

3 Install Python Dependencies

    pip install ultralytics tensorflow gymnasium numpy matplotlib opencv-python

4 Build the Workspace

    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install
    source install/setup.bash


🛠 Usage

1. Run Simulation with Manual Control

    # Start the robot in Gazebo and control it via keyboard.
    ros2 launch amr_controller system.launch.py
    
    #In another terminal:
    ros2 run key_teleop key_teleop --ros-args -r key_vel:=input_joy/cmd_vel_stamped -p twist_stamped_enabled:=True

2. SLAM: Create or Use a Map

    ros2 launch amr_slam main.launch.py use_slam:=true   # generate new map
    ros2 launch amr_slam main.launch.py use_slam:=false  # AMCL localization with existing map

    #Control the robot:
    ros2 run key_teleop key_teleop --ros-args -r key_vel:=input_joy/cmd_vel_stamped -p twist_stamped_enabled:=True

3. Object Detection with YOLOv8

    ros2 launch amr_controller system.launch.py world_name:=small_house

    #In another terminal:
    ros2 run key_teleop key_teleop --ros-args -r key_vel:=input_joy/cmd_vel_stamped -p twist_stamped_enabled:=True

    #In another terminal:
    ros2 run amr_perception amr_yolo.py   # loads pretrained YOLOv8 weights

4. Train DRL Navigation Policy (SAC)

    #Simulation with DRL training:
    ros2 launch amr_drl_navigation simulation.launch.py world_name:=small_warehouse

    #In another terminal:
    ros2 launch amr_drl_navigation amr_trainer.launch.py


    #Training parameters can be modified in:
    amr_drl_navigation/config/


📂 Project Directory Structure

The repository follows the standard ROS 2 workspace layout:


RE_based_amr/
├── src/
│   ├── amr_controller/
│   │   ├── amr_controller/
│   │   ├── config/
│   │   ├── include/
│   │   ├── launch/
│   │   ├── src/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── amr_description/
│   │   ├── include/
│   │   ├── launch/
│   │   ├── meshes/
│   │   ├── models/
│   │   ├── photos/
│   │   ├── rviz/
│   │   ├── src/
│   │   ├── urdf/
│   │   ├── worlds/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── amr_drl_navigation/
│   │   ├── amr_drl_navigation/
│   │   ├── config/
│   │   ├── goals_and_poses/training/
│   │   ├── include/
│   │   ├── launch/
│   │   ├── models/
│   │   ├── src/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── amr_perception/
│   │   ├── amr_perception/
│   │   ├── config/
│   │   ├── include/
│   │   ├── launch/
│   │   ├── src/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── amr_slam/
│   │   ├── amr_slam/
│   │   ├── config/
│   │   ├── include/
│   │   ├── launch/
│   │   ├── maps/
│   │   ├── src/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── human_simulator/
│   │   ├── config/
│   │   ├── human_model/
│   │   ├── human_simulator/
│   │   ├── include/
│   │   ├── launch/
│   │   ├── src/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── CMakeLists.txt   # Workspace-level CMake
│
├── README.md
├── .gitignore
├── yolov8n.pt            # YOLO model weights
