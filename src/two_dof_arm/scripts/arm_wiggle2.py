#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point


class ArmWigglerFK(Node):

    def __init__(self):
        super().__init__('arm_wiggler_fk')

        # Joint command publisher
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/arm_controller/commands',
            10
        )

        # End-effector position publisher
        self.ee_pub = self.create_publisher(
            Point,
            '/end_effector_position',
            10
        )

        self.timer = self.create_timer(0.1, self.update)

        self.t = 0.0

        # Link lengths (meters)
        self.L1 = 0.3
        self.L2 = 0.3

    def update(self):

        # ---- Joint motion ----
        joint1 = 0.5 * math.sin(self.t)
        joint2 = 1.0 * math.sin(self.t * 0.5)

        cmd_msg = Float64MultiArray()
        cmd_msg.data = [joint1, joint2]
        self.cmd_pub.publish(cmd_msg)

        # ---- Forward kinematics ----
#        x = self.L1 * math.cos(joint1) + \
 #           self.L2 * math.cos(joint1 + joint2)

  #      z = self.L1 * math.sin(joint1) + \
   #         self.L2 * math.sin(joint1 + joint2)

  # ---- Der var byttet rundt på x og z. IDK why
        x = self.L2 * math.sin(joint1 + joint2) + self.L1 * math.sin(joint1)
        z = self.L2 * math.cos(joint1 + joint2) + self.L1 * math.cos(joint1) + 0.2 # base box = 0.2

        ee_msg = Point()
        ee_msg.x = x
        ee_msg.y = 0.0
        ee_msg.z = z

        self.ee_pub.publish(ee_msg)

        self.get_logger().info(
            f"EE position → x: {x:.3f}, z: {z:.3f}"
        )

        self.t += 0.1


def main(args=None):
    rclpy.init(args=args)
    node = ArmWigglerFK()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

