#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
import math
from geometry_msgs.msg import Twist

class RandomWalker(Node):
    def __init__(self):
        super().__init__('random_walker')
        self.publisher = self.create_publisher(Twist, '/human/cmd_vel', 10)
        self.timer = self.create_timer(2.0, self.change_direction)
        self.vel_msg = Twist()

    def change_direction(self):
        # Random walk: new heading every 2s
        speed = 1.0  # m/s walking speed
        angle = random.uniform(0, 2 * math.pi)

        self.vel_msg.linear.x = speed * math.cos(angle)
        self.vel_msg.linear.y = speed * math.sin(angle)
        self.vel_msg.angular.z = 0.0  # keep facing forward

        self.publisher.publish(self.vel_msg)
        self.get_logger().info(f"Walking at vx={self.vel_msg.linear.x:.2f}, vy={self.vel_msg.linear.y:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = RandomWalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()