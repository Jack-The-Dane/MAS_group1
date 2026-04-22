#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String


class TuckTestNode(Node):
    def __init__(self):
        super().__init__('tuck_test_node')

        self.arm_pub = self.create_publisher(
            Float64MultiArray,
            '/arm_controller/commands',
            10
        )

        self.gimbal_mode_pub = self.create_publisher(
            String,
            '/gimbal_mode',
            10
        )

        self.declare_parameter('mode', 'tuck')
        self.declare_parameter('arm_q1_deg', 0.0)
        self.declare_parameter('arm_q2_deg', 0.0)

        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.timer_callback)

    # in case we dont have numpy on the drone
    def deg2rad(self, deg):
        return deg * math.pi / 180.0

    def timer_callback(self):
        mode = self.get_parameter('mode').value
        arm_q1_deg = self.get_parameter('arm_q1_deg').value
        arm_q2_deg = self.get_parameter('arm_q2_deg').value

        mode_msg = String()
        mode_msg.data = mode
        self.gimbal_mode_pub.publish(mode_msg)

        arm_msg = Float64MultiArray()
        arm_msg.data = [
            self.deg2rad(arm_q1_deg),
            self.deg2rad(arm_q2_deg)
        ]
        self.arm_pub.publish(arm_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TuckTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()