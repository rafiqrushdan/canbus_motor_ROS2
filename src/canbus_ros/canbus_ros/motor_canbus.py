#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MotorCanbus(Node):

    def __init__(self):
        super().__init__("motor_canbus")
        example_param = self.declare_parameter("example_param", "default_value").value
        self.get_logger().info(f"Declared parameter 'example_param'. Value: {example_param}")
        self.get_logger().info("Hello world from the Python node motor_canbus")


def main(args=None):
    rclpy.init(args=args)

    motor_canbus = MotorCanbus()

    try:
        rclpy.spin(motor_canbus)
    except KeyboardInterrupt:
        pass

    motor_canbus.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
