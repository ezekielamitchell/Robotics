#!/usr/bin/env python3.12

import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__("python_test")
        self.counter_ = 0
        self.get_logger().info("Hello @__init__")
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.counter_ += 1
        self.get_logger().info("Hello @timer_callback : counter = " + str(self.counter_))


def main(args=None):
    rclpy.init(args=args)

    node = MyNode()

    rclpy.spin(node=node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
