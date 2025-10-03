#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistBridge(Node):
    def __init__(self):
        super().__init__('twist_to_twiststamped_bridge')

        # Subscriber: listens to plain Twist
        self.sub = self.create_subscription(
            Twist,
            '/amr_controller/cmd_vel_unstamped',   # <-- your topic publishing Twist
            self.twist_callback,
            10
        )

        # Publisher: publishes TwistStamped
        self.pub = self.create_publisher(
            TwistStamped,
            'amr_controller/cmd_vel',  # <-- what your robot listens to
            10
        )

    def twist_callback(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.twist = msg
        self.pub.publish(stamped)

def main():
    rclpy.init()
    node = TwistBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()