#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('move_arm_in_circle')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'arm_controller/commands', 10)

        self.dt = 0.02  # 50 Hz (smooth)
        self.t = 0.0
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def timer_callback(self):
    
        x_des = 0.2+0.1*math.cos(self.t)
        z_des = 0.2+0.1*math.sin(self.t)
        
        l1 = 0.3
        l2 = 0.3
        
        costheta2 = ( x_des*x_des + z_des*z_des - l1*l1 - l2*l2 ) / (2*l1*l2)
    
        theta2 = math.atan2(math.sqrt(1-costheta2*costheta2),costheta2)
        k1 = l1 + l2 * costheta2
        k2 = l2 * math.sin(theta2)
        
        theta1 = math.atan2(z_des, x_des) - math.atan2(k2, k1)

        msg = Float64MultiArray()
        msg.data = [theta1, theta2]
        print("q1: ", round(theta1,4))
        print("q2: ", round(theta2,4), "\n")
        self.publisher_.publish(msg)

        self.t += 0.01  # speed (bigger = faster)
        
        # Forward kinematics for testing/print purposes
        fk_x = l2 * math.sin(theta1 + theta2) + l1 * math.sin(theta1)
        fk_y = l2 * math.cos(theta1 + theta2) + l1 * math.cos(theta1) + 0.2 # base box = 0.2

        print("fk_x: ", round(fk_x,4))
        print("fk_z: ", round(fk_y,4), "\n")


def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
