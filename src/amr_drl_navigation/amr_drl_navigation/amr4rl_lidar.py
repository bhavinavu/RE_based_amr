#!/usr/bin/env python3

import json
import numpy as np
import random
import yaml
import sys
import time
import math
import os
import traceback

# os.environ["CUDA_VISIBLE_DEVICES"]="-1"
import tensorflow as tf

import rclpy
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity, ControlWorld, SetEntityPose
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_srvs.srv import Empty
from geometry_msgs.msg import Twist

import gymnasium as gym
from gymnasium import spaces

from amr_drl_navigation.sac import SAC
from amr_drl_navigation.trainer import Trainer
from amr_drl_navigation.on_policy_trainer import OnPolicyTrainer
from amr_drl_navigation.amr4rl_environment_lidar import Pic4rlEnvironmentLidar
from ament_index_python.packages import get_package_share_directory

from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import ExternalShutdownException


class Pic4rlLidar(Pic4rlEnvironmentLidar):
    def __init__(self):
        super().__init__()
        self.log_check()
        train_params = self.parameters_declaration()
        if self.tflite_flag:
            self.actor_fp16 = tf.lite.Interpreter(
                model_path="~/inference/actor_fp16.tflite"
            )
            self.actor_fp16.allocate_tensors()
            self.input_index_image = self.actor_fp16.get_input_details()[0]["index"]
            self.input_index_state = self.actor_fp16.get_input_details()[1]["index"]
            self.output_index = self.actor_fp16.get_output_details()[0]["index"]
            self.commands = [0.0, 0.0]
            self.step_counter = 0
            self.done = False

        else:
            self.set_parser_list(train_params)
            self.trainer = self.instantiate_agent()

    def instantiate_agent(self):
        """
        ACTION AND OBSERVATION SPACES settings
        """
        action = [
            [self.min_lin_vel, self.max_lin_vel],  # x_speed
            # [self.min_lin_vel, self.max_lin_vel], # y_speed
            [self.min_ang_vel, self.max_ang_vel],  # w_speed
        ]

        low_action = []
        high_action = []
        for i in range(len(action)):
            low_action.append(action[i][0])
            high_action.append(action[i][1])

        low_action = np.array(low_action, dtype=np.float32)
        high_action = np.array(high_action, dtype=np.float32)

        self.action_space = spaces.Box(
            low=low_action, high=high_action, shape=(2,), dtype=np.float32
        )

        state = [
            [0.0, 15.0],  # goal_distance
            [-math.pi, math.pi],  # goal angle or yaw
        ]


        if len(state) > 0:
            low_state = []
            high_state = []
            for i in range(len(state)):
                low_state.append(state[i][0])
                high_state.append(state[i][1])

            self.low_state = np.array(low_state, dtype=np.float32)
            self.high_state = np.array(high_state, dtype=np.float32)

        self.observation_space = spaces.Box(
            low=self.low_state, high=self.high_state, dtype=np.float32
        )

        # Set Epsilon-greedy starting value for exploration policy (minimum 0.05)
        epsilon = 0.6

        self.print_log()
        if self.mode == "testing":
            self.epsilon = 0.0
        else:
            self.epsilon = epsilon

        # OFF-POLICY ALGORITHM TRAINER
        if self.policy_trainer == "off-policy":
            parser = Trainer.get_argument()
            if self.train_policy == "SAC":
                self.get_logger().debug("Parsing SAC parameters...")
                parser = SAC.get_argument(parser)
                args = parser.parse_args(self.parser_list)
                policy = SAC(
                    state_shape=self.observation_space.shape,
                    action_dim=self.action_space.high.size,
                    max_action=self.action_space.high,
                    min_action=self.action_space.low,
                    lr=2e-4,
                    lr_alpha=3e-4,
                    actor_units=(256, 256),
                    critic_units=(256, 256),
                    tau=5e-3,
                    alpha=0.2,
                    auto_alpha=False,
                    init_temperature=None,
                    gpu=self.gpu,
                    batch_size=self.batch_size,
                    n_warmup=self.n_warmup,
                    memory_capacity=self.memory_capacity,
                    epsilon=self.epsilon,
                    epsilon_decay=0.996,
                    epsilon_min=0.05,
                    log_level=self.log_level,
                )
                self.get_logger().info("Instantiate SAC agent...")

            trainer = Trainer(policy, self, args, test_env=None)
            # self.get_logger().info('Starting process...')
            # trainer()

        return trainer

    def set_parser_list(self, params):
        """ """
        self.parser_list = []
        for k, v in params.items():
            if v is not None:
                kv = k + "=" 
                if k == "--logdir":
                    kv += self.logdir
                    self.get_logger().info(f"logdir set to: {kv}")
                elif k == "--model-dir":
                    kv += self.model_path
                    self.get_logger().info(f"model path set to: {kv}")
                elif k == "--rb-path-save":
                    kv += self.logdir + '/' + v
                    self.get_logger().info(f"rb path save set to: {kv}")
                elif k == "--rb-path-load":
                    kv += self.rb_path_load
                    self.get_logger().info(f"rb path load set to: {kv}")
                else:
                    kv += str(v)
                self.parser_list.append(kv)
            else:
                self.parser_list.append(k)

    def threadFunc(self):
        """ """
        try:
            self.trainer()
        except Exception:
            self.get_logger().error(
                f"Error in starting trainer:\n {traceback.format_exc()}"
            )
            return

    def threadFunc_tflite(self):
        while True:
            if self.step_counter == 0:
                observation = self.reset(self.step_counter)
            else:
                observation, reward, done, info = self.step(self.commands)
                self.done = done
            if self.done:
                self.done = False
                self.step_counter = 0
                observation = self.reset(self.step_counter)

            self.actor_fp16.set_tensor(self.input_index_state, observation[1])
            self.actor_fp16.set_tensor(self.input_index_image, observation[0])

            self.actor_fp16.invoke()
            self.commands = self.actor_fp16.get_tensor(self.output_index)[0, :]

            self.step_counter += 1

    def log_check(self):
        """
        Select the ROS2 log level.
        """
        try:
            self.log_level = int(os.environ["LOG_LEVEL"])
        except:
            self.log_level = 20
            self.get_logger().info("LOG_LEVEL not defined, setting default: INFO")

        self.get_logger().set_level(self.log_level)

    def print_log(self):
        """ """
        for i in range(len(self.log_dict)):
            self.get_logger().info(
                f"{list(self.log_dict)[i]}: {self.log_dict[list(self.log_dict)[i]]}"
            )

        self.get_logger().info(f"action space shape: {self.action_space.high.size}")
        self.get_logger().info(
            f"observation space size: {self.observation_space.high.size}\n"
        )

    def parameters_declaration(self):
        """ """
        
        self.package_name = self.get_parameter(
            "package_name").get_parameter_value().string_value
        train_params_path = self.get_parameter(
            "training_params_path").get_parameter_value().string_value

        with open(train_params_path, "r") as train_param_file:
            train_params = yaml.safe_load(train_param_file)["training_params"]

        self.declare_parameters(
            namespace="",
            parameters=[
                ("max_lin_vel", rclpy.Parameter.Type.DOUBLE),
                ("min_lin_vel", rclpy.Parameter.Type.DOUBLE),
                ("max_ang_vel", rclpy.Parameter.Type.DOUBLE),
                ("min_ang_vel", rclpy.Parameter.Type.DOUBLE),
            ],
        )

        self.train_policy = train_params["--policy"]
        self.policy_trainer = train_params["--policy_trainer"]
        self.min_ang_vel = self.get_parameter("min_ang_vel").get_parameter_value().double_value
        self.min_lin_vel = self.get_parameter("min_lin_vel").get_parameter_value().double_value
        self.max_ang_vel = self.get_parameter("max_ang_vel").get_parameter_value().double_value
        self.max_lin_vel = self.get_parameter("max_lin_vel").get_parameter_value().double_value
        self.sensor_type = self.get_parameter("sensor").get_parameter_value().string_value
        self.gpu = train_params["--gpu"]
        self.batch_size = train_params["--batch-size"]
        self.n_warmup = train_params["--n-warmup"]
        self.tflite_flag = train_params["--tflite_flag"]
        self.tflite_model_path = train_params["--tflite_model_path"]

        if self.train_policy == "PPO":
            self.horizon = int(train_params["--horizon"])
            self.normalize_adv = bool(train_params["--normalize-adv"])
            self.enable_gae = bool(train_params["--enable-gae"])
        else:
            self.memory_capacity = int(train_params["--memory-capacity"])

        self.log_dict = {
            "policy": train_params["--policy"],
            "max_steps": train_params["--max-steps"],
            "max_episode_steps": train_params["--episode-max-steps"],
            "sensor": self.sensor_type,
            "gpu": train_params["--gpu"],
        }

        return train_params