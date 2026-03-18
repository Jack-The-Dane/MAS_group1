#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('move_arm_in_circle')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'arm_controller/commands', 10)
        self.pos_publisher_ = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)


        self.dt = 0.02  # 50 Hz (smooth)
        self.t = 0.0
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        # self.q1 = np.deg2rad(85)
        # self.q2 = np.deg2rad(-170)
        # self.q1 = np.deg2rad(-70)
        # self.q2 = np.deg2rad(100)
        self.q1 = np.deg2rad(0) # init 
        self.q2 = np.deg2rad(0) # init

        self.null_q1 = np.deg2rad(-60) # target/nullspace joint angles
        self.null_q2 = np.deg2rad(110) 


        self.psi = 0.0

        self.x = 0.0
        self.y = 0.0
        self.z = 1.0
        self.yaw = 0.0

        self.x_ee = 0.0
        self.y_ee = 0.0
        self.z_ee = 0.0

        self.mode = ""
        self.armed = False


        self.start_pos = np.array([0.0, 0.0, 1.5])
        self.targets = [None, None, None, None]
        self.current_target = 0

        self.state = "WAIT"
        self.state_start_time = self.get_clock().now().seconds_nanoseconds()[0]

        # subsribe to end-effector pose
        self.ee_subscription = self.create_subscription(
            Odometry,
            '/end_effector/pose',
            self.ee_pose_callback,
            10
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # subsribe to IMU
        self.imu_subscription = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            qos_profile
        )
        self.mode_subscription = self.create_subscription(
            State,
            '/mavros/state',
            self.mode_callback,
            10
        )

        self.target_subscription = self.create_subscription(
            Odometry,
            '/target1/pose',
            self.target1_callback,
            10
        )

        self.target_subscription = self.create_subscription(
            Odometry,
            '/target2/pose',
            self.target2_callback,
            10
        )
        self.target_subscription = self.create_subscription(
            Odometry,
            '/target3/pose',
            self.target3_callback,
            10
        )
        self.target_subscription = self.create_subscription(
            Odometry,
            '/target4/pose',
            self.target4_callback,
            10
        )


    def ee_pose_callback(self, msg: Odometry):
        self.x_ee = msg.pose.pose.position.x
        self.y_ee = msg.pose.pose.position.y
        self.z_ee = msg.pose.pose.position.z
        
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

        _, _, self.yaw = self.quaternion_to_euler_angle(qw, qx, qy, qz)

    def target1_callback(self, msg: Odometry):
        self.targets[0] = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
    def target2_callback(self, msg: Odometry):
        self.targets[1] = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
    def target3_callback(self, msg: Odometry):
        self.targets[2] = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
    def target4_callback(self, msg: Odometry):
        self.targets[3] = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])


    def mode_callback(self, msg: State):
        self.mode = msg.mode
        self.armed = msg.armed

    def timer_callback(self):
        # print("implement wait for OFFBOARD mode in topic /mavros/state/mode")
        # print("Fix link1 rotates around center!!!!!!!!")
        # print("mode", self.mode)
        # print("armed", self.armed)
        if self.mode != "OFFBOARD" or not self.armed:
            msg = PoseStamped()
            msg.pose.position.x = 0.0
            msg.pose.position.y = 0.0
            msg.pose.position.z = 0.0
            self.pos_publisher_.publish(msg)
            # print("0,0,0")
            msg = Float64MultiArray()
            msg.data = [self.q1, self.q2]
            
            self.publisher_.publish(msg)
            return
        # print("im here")

        current_time = self.get_clock().now().nanoseconds * 1e-9

        if self.state == "WAIT":
            x_des = self.start_pos
            
            if current_time - self.state_start_time > 5.0:   # wait 10 seconds
                self.state = "GO_TO_TARGET_HIGH"
                # self.state_start_time = current_time
                print("GO TO TARGET_HIGH")


        elif self.state == "GO_TO_TARGET_HIGH":

            target = self.targets[self.current_target]
            if target is None:
                return

            x_des = target + np.array([0.0, 0.0, 0.6])  # 20 cm above
            # print(np.linalg.norm(x_des - np.array([self.x_ee, self.y_ee, self.z_ee])))
            
            if np.linalg.norm(x_des - np.array([self.x_ee, self.y_ee, self.z_ee])) < 0.1:
                self.state = "GO_TO_TARGET_LOW"
                print("GO TO TARGET LOW")
                
        elif self.state == "GO_TO_TARGET_LOW":
            
            target = self.targets[self.current_target]

            x_des = target + np.array([0.0, 0.0, 0.2])
            
            if np.linalg.norm(x_des - np.array([self.x_ee, self.y_ee, self.z_ee])) < 0.05:
                self.state_start_time = current_time
                self.state = "HOVER_TARGET"

        elif self.state == "HOVER_TARGET":

            target = self.targets[self.current_target]
            
            x_des = target + np.array([0.0, 0.0, 0.2])
            # print(np.linalg.norm(x_des - np.array([self.x_ee, self.y_ee, self.z_ee])))

            if current_time - self.state_start_time > 10.0:
                self.current_target += 1

                if self.current_target >= 4:
                    self.state = "GO_HOME"
                    print("GO HOME")
                else:
                    self.state = "GO_TO_TARGET_HIGH"
                    print("GO TO TARGET HIGH")


        elif self.state == "GO_HOME":
            x_des = self.start_pos
            self.null_q1 = 0
            self.null_q2 = 0

        error = x_des - np.array([self.x_ee, self.y_ee, self.z_ee])
        k = 0.2
        p_des_vel = k * error

        # error_x = -1.0 - self.x_ee 
        # error_y = -1.0 - self.y_ee
        # error_z = 1.5 - self.z_ee
     
        # k = 0.2

        # p_des_vel = np.array([
        #     error_x * k,
        #     error_y * k,
        #     error_z * k
        # ])



        error_yaw = 0.0 - self.yaw
        error_q1 = self.null_q1 - self.q1
        error_q2 = self.null_q2 - self.q2
        k_n = 0.2
        v = np.array([
            0, 0, 0, 
            0*error_yaw * k_n,
            error_q1 * k_n,
            error_q2 * k_n
        ])

        l1 = 0.3
        l2 = 0.3
        
        a1 = -l1*math.sin(self.q1) - l2*math.sin(self.q1+self.q2)
        a2 = -l2*math.sin(self.q1+self.q2)
        b1 =  l1*math.cos(self.q1) + l2*math.cos(self.q1+self.q2)
        b2 =  l2*math.cos(self.q1+self.q2)
        ry = l1 * math.cos(self.q1) + l2 * math.cos(self.q1 + self.q2)

        # mobile Jacobian elements
        # J11 = 1; J12 = 0; J13 = 0
        # J21 = 0; J22 = 1; J23 = 0
        # J31 = 0; J32 = 0; J33 = 1

        self.psi = self.yaw

        J14 = - math.cos(self.psi) * ry
        J15 = -math.sin(self.psi)*a1
        J16 = -math.sin(self.psi)*a2

        J24 = - math.sin(self.psi) * ry
        J25 = math.cos(self.psi)*a1
        J26 = math.cos(self.psi)*a2

        # J34 = 0
        # J35 = b1
        # J36 = b2

        J = np.array([
            [1, 0, 0, J14, J15, J16],
            [0, 1, 0, J24, J25, J26],
            [0, 0, 1, 0,   b1,  b2]
        ])

        J_pinv = J.T @ np.linalg.inv(J @ J.T)

        I = np.eye(6)

        q_dot = J_pinv @ p_des_vel + (I - J_pinv @ J) @ v

        
        self.q1 += q_dot[4] * self.dt
        self.q2 += q_dot[5] * self.dt
        
        msg = Float64MultiArray()
        msg.data = [self.q1, self.q2]
        # print("q1: ", round(self.q1,4))
        # print("q2: ", round(self.q2,4), "\n")
        self.publisher_.publish(msg)

        self.x += q_dot[0] * self.dt
        self.y += q_dot[1] * self.dt
        self.z += q_dot[2] * self.dt
        self.yaw += q_dot[3] * self.dt
        
        msg = PoseStamped()


        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z
        
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = sy
        msg.pose.orientation.w = cy

        self.pos_publisher_.publish(msg)

        self.t += self.dt  # speed (bigger = faster)


def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
