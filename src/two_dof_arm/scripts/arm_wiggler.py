#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class ArmWiggler(Node):

    def __init__(self):
        super().__init__('arm_wiggler')

        # Publisher to the arm controller
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/arm_controller/commands',
            10
        )

        # Timer → publish at 10 Hz
        self.timer = self.create_timer(0.1, self.wiggle)

        self.t = 0.0
        self.get_logger().info("Arm wiggler started")

    def wiggle(self):
        msg = Float64MultiArray()

        # Wiggle joints with sine waves
        joint1 = 0.5 * math.sin(self.t)
        joint2 = 1.0 * math.sin(self.t * 0.5)

        msg.data = [joint1, joint2]

        self.pub.publish(msg)

        self.t += 0.1


def main(args=None):
    rclpy.init(args=args)

    node = ArmWiggler()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
