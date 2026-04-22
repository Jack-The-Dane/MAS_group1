#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class GimbalNode(Node):
    def __init__(self):
        super().__init__('gimbal_node')

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/gimbal_controller/commands',
            10
        )

        self.dt = 0.02  # 50 Hz
        self.timer = self.create_timer(self.dt, self.timer_callback)

        ####### IMPORTANT #############
        # Make sure Aksel have not messed up and sends these to the correct motors
        # Current command sent to motors
        self.gimbal_pitch = 0.0
        self.gimbal_roll = 0.0

        # IMU stabilization values (Dynamic)
        self.stabilize_pitch = 0.0
        self.stabilize_roll = 0.0

        ####### IMPORTANT #############
        # Test these and find the correct values
        # Hardcoded tuck values
        self.tuck_pitch = 0.0
        self.tuck_roll = 0.0
        self.declare_parameter('gimbal1_deg', 0.0)
        self.declare_parameter('gimbal2_deg', 0.0)

        # Modes: "stabilize" or "tuck"
        self.mode = "tuck"


        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            qos_profile
        )

        self.mode_subscription = self.create_subscription(
            String,
            '/gimbal_mode',
            self.mode_callback,
            10
        )

        self.get_logger().info('Gimbal node started in tuck mode')

    def deg2rad(self, deg):
        return deg * math.pi / 180.0


    # Check for gimbal mode
    def mode_callback(self, msg: String):
        new_mode = msg.data

        if new_mode in ['stabilize', 'tuck']:
            if new_mode != self.mode:
                self.mode = new_mode
                self.get_logger().info(f'Gimbal mode: {self.mode}')

    # Set gimbal commands based on mode
    def timer_callback(self):
        self.tuck_pitch = self.deg2rad(self.get_parameter('gimbal1_deg').value)
        self.tuck_roll = self.deg2rad(self.get_parameter('gimbal2_deg').value)


        if self.mode == 'stabilize':
            self.gimbal_pitch = self.stabilize_pitch
            self.gimbal_roll = self.stabilize_roll
        elif self.mode == 'tuck':
            self.gimbal_pitch = self.tuck_pitch
            self.gimbal_roll = self.tuck_roll

        msg = Float64MultiArray()
        msg.data = [self.gimbal_pitch, self.gimbal_roll]
        self.publisher_.publish(msg)

    def quaternion_to_euler_angle(self, w, x, y, z):
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def imu_callback(self, msg: Imu):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        roll, pitch, yaw = self.quaternion_to_euler_angle(qw, qx, qy, qz)

        # Normal gimbal stabilization mode
        self.stabilize_pitch = pitch
        self.stabilize_roll = -roll


def main(args=None):
    rclpy.init(args=args)
    node = GimbalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()