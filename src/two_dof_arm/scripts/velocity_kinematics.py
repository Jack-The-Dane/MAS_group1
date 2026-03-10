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
        
        self.q1 = 0.1
        self.q2 = 0.1

    def timer_callback(self):
    
        x_dot_des = -0.1*math.sin(self.t)
        z_dot_des = 0.1*math.cos(self.t)
        
        l1 = 0.3
        l2 = 0.3
        
        # Jacobian elements
        J11 = -l1*math.sin(self.q1) - l2*math.sin(self.q1+self.q2)
        J12 = -l2*math.sin(self.q1+self.q2)
        J21 =  l1*math.cos(self.q1) + l2*math.cos(self.q1+self.q2)
        J22 =  l2*math.cos(self.q1+self.q2)
        
        detJ = J11*J22 - J12*J21
        
        if abs(detJ) < 1e-4:
            print("Near singularity!")
            
            return
        
        invJ11 =  J22 / detJ
        invJ12 = -J12 / detJ
        invJ21 = -J21 / detJ
        invJ22 =  J11 / detJ
        
        q1dot = invJ11*x_dot_des + invJ12*z_dot_des
        q2dot = invJ21*x_dot_des + invJ22*z_dot_des
        
        self.q1 += q1dot * self.dt
        self.q2 += q2dot * self.dt
        
        msg = Float64MultiArray()
        msg.data = [self.q1, self.q2]
        print("q1: ", round(self.q1,4))
        print("q2: ", round(self.q2,4), "\n")
        self.publisher_.publish(msg)

        self.t += 0.01  # speed (bigger = faster)
        
        # Forward kinematics for testing/print purposes
        #fk_x = l2 * math.sin(theta1 + theta2) + l1 * math.sin(theta1)
        #fk_y = l2 * math.cos(theta1 + theta2) + l1 * math.cos(theta1) + 0.2 # base box = 0.2

        #print("fk_x: ", round(fk_x,4))
        #print("fk_z: ", round(fk_y,4), "\n")


def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
