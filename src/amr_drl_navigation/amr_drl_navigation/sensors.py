#!/usr/bin/env python3
import os
import time
import math
import rclpy
import numpy as np

import matplotlib.pyplot as plt
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu


from amr_drl_navigation.env_utils import quat_to_euler, tf_decompose, euler_from_quaternion

import yaml


class LaserScanSensor:
    def __init__(
        self,
        max_dist,
        num_points,
        robot_type,
        robot_radius,
        robot_size,
        collision_tolerance,
    ):
        self.max_dist = max_dist
        self.num_points = num_points
        self.laser_info = [
            0.0,
            6.283199787139893,
            0.01750195026397705,
        ]  # min angle, max angle, increment

        self.get_collision_vector(
            robot_type, robot_radius, robot_size, collision_tolerance
        )

    def get_collision_vector(
        self, robot_type, robot_radius, robot_size, collision_tolerance
    ):
        """ """
        if robot_type == "circular":
            self.collision_vector = np.full(360, robot_radius + collision_tolerance)

        elif robot_type == "rectangular":
            sL = robot_size[0] / 2 + collision_tolerance  # Rover lenght semiaxis
            sW = robot_size[1] / 2 + collision_tolerance  # Rover width semiaxis

            degrees = np.arange(0, math.pi * 2, math.pi / 180)
            vec1 = sL / np.cos(degrees)
            vec2 = sW / np.sin(degrees + 0.0001)

            self.collision_vector = np.minimum(np.abs(vec1), np.abs(vec2))

    def process_data(self, points):
        # There are some outliers (0 or nan values, they all are set to 0) that will not be passed to the DRL agent
        # Correct data:
        scan_range = []

        min_dist_point = self.max_dist
        points = np.nan_to_num(
            points[:], nan=self.max_dist, posinf=self.max_dist, neginf=self.max_dist
        )
        points[points < 0.05] = self.max_dist
        raw_data = points
        collision = np.any(points < self.collision_vector)

        min_obstacle_distance = min(points)
        # max_obstacle_distance = max(points)
        min_obstacle_angle = np.argmin(points)

        points = self.add_noise(points)
        points = np.clip(points, 0.05, self.max_dist)
        # Takes only num_points
        div = int(360 / self.num_points)

        # Takes only sensed measurements
        scan_range = np.minimum.reduceat(points, np.arange(0, len(points), div))
        # print('min obstacle distance: ', min_obstacle_distance)
        # print('min obstacle angle :', min_obstacle_angle)
        # print(len(scan_range))
        # self.plot_points(points)

        return scan_range, min_obstacle_distance, collision 

    def add_noise(self, points):
        noise = np.random.normal(loc=0.0, scale=0.05, size=points.shape)
        noisy_points = points + noise
        # print('360 noisy lidar points: ', noisy_points)
        return noisy_points

    def plot_points(self, points):
        x_coord = []
        y_coord = []

        x_collision = []
        y_collision = []
        degrees = np.arange(0, math.pi * 2, math.pi / 180)
        for i in range(len(degrees)):
            angle = degrees[i]
            x = points[i] * math.cos(-angle)
            y = points[i] * math.sin(-angle)

            x_c = self.collision_vector[i] * math.cos(-angle)
            y_c = self.collision_vector[i] * math.sin(-angle)

            x_coord.append(x)
            y_coord.append(y)
            x_collision.append(x_c)
            y_collision.append(y_c)

        plt.scatter(y_coord, x_coord, label="lidar")
        plt.scatter(y_collision, x_collision, label="collision")
        plt.ylabel("x [m]")
        plt.xlabel("y [m]")
        plt.title("Lidar points")
        plt.legend()
        plt.grid(True)
        plt.show(block=True)
        plt.pause(1)
        plt.close()


class OdomSensor:
    def __init__(self):
        pass

    def process_data(self, data, vel=False):
        pos_x = data.pose.pose.position.x
        pos_y = data.pose.pose.position.y
        zqr = data.pose.pose.orientation.z
        wqr = data.pose.pose.orientation.w
        zr = quat_to_euler(zqr, wqr)
        # _,_,yaw = euler_from_quaternion(data.pose.pose.orientation)

        if vel:
            vx = data.twist.twist.linear.x
            wz = data.twist.twist.angular.z

            return [pos_x, pos_y, zr], [vx, wz]

        else:
            return [pos_x, pos_y, zr]



class Sensors:
    def __init__(self, node):
        # super().__init__("generic_sensors_node")
        self.param = self.get_param(node)

        self.odom_data = None
        self.laser_data = None

        self.laser_sub = None
        self.odom_sub = None

        self.laser_process = None
        self.odom_process = None
        self.node = node
        self.sensor_msg = {}
        self.sensors = self.activate_sensors()

    def get_param(self, node):
        configFilepath = node.main_params_path

        # Load the topic parameters
        with open(configFilepath, "r") as file:
            configParams = yaml.safe_load(file)["main_node"]["ros__parameters"]

        return configParams

    def activate_sensors(self):

            
        if self.param["lidar_enabled"] == "true":
            self.node.get_logger().debug("Laser scan subscription done")
            self.laser_sub = self.node.create_subscription(
                LaserScan,
                self.param["sensors_topic"]["laser_topic"],
                self.laser_scan_cb,
                1,
            )
            self.laser_process = LaserScanSensor(
                self.param["laser_param"]["max_distance"],
                self.param["laser_param"]["num_points"],
                self.param["robot_type"],
                self.param["robot_radius"],
                self.param["robot_size"],
                self.param["collision_tolerance"],
            )
            self.sensor_msg["scan"] = "None"

        self.node.get_logger().debug("Odometry subscription done")
        self.odom_sub = self.node.create_subscription(
            Odometry, self.param["sensors_topic"]["odom_topic"], self.odometry_cb, 1
        )
        self.odom_process = OdomSensor()
        self.sensor_msg["odom"] = "None"



    def laser_scan_cb(self, msg):
        self.laser_data = msg.ranges
        self.sensor_msg["scan"] = msg

    def odometry_cb(self, msg):
        self.odom_data = msg
        self.sensor_msg["odom"] = msg

    def get_odom(self, vel=False):
        if self.odom_sub is None:
            self.node.get_logger().warn("NO Odometry subscription")
            return None
        if self.odom_data is None:
            self.node.get_logger().warn("NO Odometry data")
            return None

        if vel:
            data, velocities = self.odom_process.process_data(self.odom_data, vel)
            return data, velocities
        else:
            data = self.odom_process.process_data(self.odom_data)
            return data


    def get_laser(self, min_obstacle_distance=False):
        if self.laser_sub is None:
            self.node.get_logger().warn("NO laser subscription")
            return None, False
        if self.laser_data is None:
            self.node.get_logger().warn("NO laser data")
            return None, False
        processed_data, min_obstacle_distance_v, collision = self.laser_process.process_data(
            self.laser_data
        )
        if min_obstacle_distance:
            return processed_data, min_obstacle_distance_v, collision
        return processed_data, collision
