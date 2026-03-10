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
    
        print("virker ikke")
        # sidste step: forward kinematics til at korrigere for drift
        # uden så integreres små fejl op og den drifter væk fra banen
        l1 = 0.3
        l2 = 0.3
        fk_x = l2 * math.sin(self.q1 + self.q2) + l1 * math.sin(self.q1)
        fk_z = l2 * math.cos(self.q1 + self.q2) + l1 * math.cos(self.q1) 
        
        x_des = 0.1*math.cos(self.t)
        z_des = 0.1*math.sin(self.t)

        error_x = x_des - fk_x
        error_z = z_des - fk_z
        Kp = 0.1
        
        # vel kin
    
        x_dot_des = -0.1*math.sin(self.t) + Kp*error_x
        z_dot_des = 0.1*math.cos(self.t) + Kp*error_z
        
        l1 = 0.3
        l2 = 0.3
        
        # Jacobian elements
        J11 = -l1*math.sin(self.q1) - l2*math.sin(self.q1+self.q2)
        J12 = -l2*math.sin(self.q1+self.q2)
        J21 =  l1*math.cos(self.q1) + l2*math.cos(self.q1+self.q2)
        J22 =  l2*math.cos(self.q1+self.q2)
        
        # Moore-Penrose with damping J+ = J' (J * J' + lambda²*I)^-1
        
        lambda_sq = 0.01
        
        # J * J'
        
        a = J11*J11 + J12*J12
        b = J11*J21 + J12*J22
        c = J11*J21 + J12*J22
        d = J21*J21 + J22*J22
        
        # Add damping
        a += lambda_sq
        d += lambda_sq
        
        det = a*d - b*c
        
        inv11 = d/det
        inv12 = -b/det
        inv21 = -c/det
        inv22 = a/det
        
        if abs(det) < 1e-4:
            print("Near singularity!")    
            #return
        
        # velocities
        
        v1 = inv11*x_dot_des + inv12*z_dot_des
        v2 = inv21*x_dot_des + inv22*z_dot_des
        
        # multiply by J'
        
        q1dot = J11*v1 + J21*v2
        q2dot = J12*v1 + J22*v2
        
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
