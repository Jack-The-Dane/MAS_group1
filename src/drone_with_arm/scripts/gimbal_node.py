#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('gimbal_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/gimbal_controller/commands', 10)

        self.dt = 0.02  # 50 Hz (smooth)
        self.t = 0.0
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.gimbal_pitch = 0.0
        self.gimbal_roll = 0.0

        self.max_pitch = 0.0
        self.max_roll = 0.0

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # subsribe to IMU
        self.subscription = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            qos_profile
        )

    
    def timer_callback(self):
    
        msg = Float64MultiArray()
        msg.data = [self.gimbal_pitch, self.gimbal_roll]
        self.publisher_.publish(msg)

        self.t += 0.01  # speed (bigger = faster)
        
    def quaternion_to_euler_angle(self, w, x, y, z):
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = math.atan2(t3, t4)

        return X, Y, Z



    def imu_callback(self, msg: Imu):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        q = [qx, qy, qz, qw]
        roll, pitch, yaw = self.quaternion_to_euler_angle(qw, qx, qy, qz)

        # roll, pitch, yaw = euler_from_quaternion(q)
        self.gimbal_pitch = pitch
        self.gimbal_roll = -roll 

        self.max_pitch = max(pitch, self.max_pitch)
        self.max_roll = max(roll, self.max_roll)
        print("max pitch: ", self.max_pitch)
        print("max roll: ", self.max_roll)


    

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
