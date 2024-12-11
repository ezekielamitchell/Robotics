#!/usr/bin/env python3.12

import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)

    node = Node("python_test")
    node.get_logger().info("Hello, Ezekiel -- FROM ROS2")

    rclpy.spin(node=node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
