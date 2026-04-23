#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3Stamped, Vector3
from nav_msgs.msg import Odometry
from mavros.base import SENSOR_QOS
import numpy as np



class TargetRelay(Node):
    def __init__(self):
        super().__init__('target_relay_node')
        self.target_1_sub = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/target_1/pose',
            self.target_1_callback,
            SENSOR_QOS
        )
        self.target_2_sub = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/target_2/pose',
            self.target_2_callback,
            SENSOR_QOS
        )
        self.target_3_sub = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/target_3/pose',
            self.target_3_callback,
            SENSOR_QOS
        )
        self.target_4_sub = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/target_4/pose',
            self.target_4_callback,
            SENSOR_QOS
        )
        self.end_effector_sub = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/end_effector_1/pose',
            self.end_effector_callback,
            SENSOR_QOS
        )

        self.target_1_pub = self.create_publisher(Odometry, '/target1/pose', 10)
        self.target_2_pub = self.create_publisher(Odometry, '/target2/pose', 10)
        self.target_3_pub = self.create_publisher(Odometry, '/target3/pose', 10)
        self.target_4_pub = self.create_publisher(Odometry, '/target4/pose', 10)
        self.end_effector_pub = self.create_publisher(Odometry, '/end_effector/pose', 10)

    def target_1_callback(self, msg):
        odom = Odometry()
        odom.header = msg.header
        odom.pose.pose = msg.pose
        self.target_1_pub.publish(odom)

    def target_2_callback(self, msg):
        odom = Odometry()
        odom.header = msg.header
        odom.pose.pose = msg.pose
        self.target_2_pub.publish(odom)

    def target_3_callback(self, msg):
        odom = Odometry()
        odom.header = msg.header
        odom.pose.pose = msg.pose
        self.target_3_pub.publish(odom)

    def target_4_callback(self, msg):
        odom = Odometry()
        odom.header = msg.header
        odom.pose.pose = msg.pose
        self.target_4_pub.publish(odom)

    def end_effector_callback(self, msg):
        odom = Odometry()
        odom.header = msg.header
        odom.pose.pose = msg.pose
        self.end_effector_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = TargetRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
