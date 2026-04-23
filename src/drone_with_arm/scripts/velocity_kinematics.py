#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
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
        self.gimbal_mode_pub = self.create_publisher(String, '/gimbal_mode', 10)


        # Time parameters
        self.dt = 0.02  # 50 Hz (smooth)
        self.t = 0.0
        self.timer = self.create_timer(self.dt, self.timer_callback)


        # Links lengths
        self.link1 = 0.3
        self.link2 = 0.3
        

        # Joint angles
        # self.q1 = np.deg2rad(85)
        # self.q2 = np.deg2rad(-170)
        # self.q1 = np.deg2rad(-70)
        # self.q2 = np.deg2rad(100)
        self.q1 = np.deg2rad(-90) # initial joint angles
        self.q2 = np.deg2rad(0) # init 

        self.null_q1 = np.deg2rad(-60) # target/nullspace joint angles
        self.null_q2 = np.deg2rad(30) 

        self.null_q1_travel = np.deg2rad(-80)
        self.null_q2_travel = np.deg2rad(70)
                

        # Drone attitude 
        self.x = 0.0
        self.y = 0.0
        self.z = 1.0
        self.yaw = 0.0


        # End-effector position
        self.x_ee = 0.0
        self.y_ee = 0.0
        self.z_ee = 0.0


        #State machine
        self.mode = ""
        self.armed = False
        self.state = "WAIT"
        self.state_start_time = self.get_clock().now().seconds_nanoseconds()[0]


        # Home positions, targets and current target
        self.home_pos = np.array([0.0, 0.0, 3.5])
        self.targets = [None, None, None, None]
        self.current_target = 0


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

    
    def desired_position_velocity(self, x_des, k=0.2):
        error = x_des - np.array([self.x_ee, self.y_ee, self.z_ee])
        p_des_vel = k * error
        return p_des_vel

    def null_space_armJoint_and_yaw(
        self,
        des_yaw,
        des_joints,
        k_null_yaw,
        k_null_joints,
        current_target,
        repulsion_radius=1.0,
        k_repulsion=0.3
    ):
        
        current_pos = np.array([self.x, self.y, self.z])

        # --- Pure repulsion in position ---
        repulsion = np.zeros(3)
        

        for i, target in enumerate(self.targets):
            if i == current_target or i == current_target -1:
                continue  # skip active target

            target_pos = np.array(target)
            diff = current_pos - target_pos
            dist = np.linalg.norm(diff)

            if dist < repulsion_radius and dist > 1e-6:
                direction = diff / dist  # unit vector AWAY

                repulsion += k_repulsion * direction
                print("dist: ", dist)
                print("repulsion: ", repulsion)
                k_null_joints = 10
               

        # --- Yaw + joint attraction ---
        error_yaw = des_yaw - self.yaw
        error_q1 = des_joints[0] - self.q1
        error_q2 = des_joints[1] - self.q2

        v = np.array([
            repulsion[0],
            repulsion[1],
            abs(repulsion[2]),
            error_yaw * k_null_yaw,
            error_q1 * k_null_joints,
            error_q2 * k_null_joints
        ])

        return v


    def mobile_jacobian(self):
        #Links
        l1 = self.link1
        l2 = self.link2

        # For J_arm:
        a1 = -l1*math.sin(self.q1) - l2*math.sin(self.q1+self.q2)
        a2 = -l2*math.sin(self.q1+self.q2)
        b1 =  l1*math.cos(self.q1) + l2*math.cos(self.q1+self.q2)
        b2 =  l2*math.cos(self.q1+self.q2)

        #For J_base:
        ry = l1 * math.cos(self.q1) + l2 * math.cos(self.q1 + self.q2)

        # "read" current yaw angle of drone
        psi = self.yaw

        J14 = - math.cos(psi) * ry
        J15 = -math.sin(psi)*a1
        J16 = -math.sin(psi)*a2

        J24 = - math.sin(psi) * ry
        J25 = math.cos(psi)*a1
        J26 = math.cos(psi)*a2

        # J34 = 0
        # J35 = b1
        # J36 = b2

        J = np.array([
            [1, 0, 0, J14, J15, J16],
            [0, 1, 0, J24, J25, J26],
            [0, 0, 1, 0,   b1,  b2]
        ])

        return J
    

    def publish_q_dot(self, q_dot):
        self.q1 += q_dot[4] * self.dt
        self.q2 += q_dot[5] * self.dt
        
        msg = Float64MultiArray()
        msg.data = [self.q1, self.q2] # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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




    def timer_callback(self):
        # print("implement wait for OFFBOARD mode in topic /mavros/state/mode")
        # print("Fix link1 rotates around center!!!!!!!!")
        # print("mode", self.mode)
        # print("armed", self.armed)
        current_time = self.get_clock().now().nanoseconds * 1e-9

        #State machine controlling the drone
        if self.mode != "OFFBOARD" or not self.armed:
            msg = PoseStamped()
            msg.pose.position.x = 0.0
            msg.pose.position.y = 0.0
            msg.pose.position.z = 0.0
            self.pos_publisher_.publish(msg)
            # print("0,0,0")
            msg = Float64MultiArray()
            msg.data = [self.q1, self.q2] # !!!!!!!!!!!!!!!!!!!!!!!!!!!
            
            self.publisher_.publish(msg)
            
            mode_msg = String()
            mode_msg.data = "tuck"
            self.gimbal_mode_pub.publish(mode_msg)

            self.state_start_time = current_time
            return
        # print("im here")


        if self.state == "WAIT":
            x_des = self.home_pos
            q_des = (np.deg2rad(-90), 0)
            
            if current_time - self.state_start_time > 5.0:   # wait 5 seconds
                self.state = "UNTUCK_GIMBAL"
                self.state_start_time = current_time
                print("UNTUCK GIMBAL")

        elif self.state == "UNTUCK_GIMBAL":
            mode_msg = String()
            mode_msg.data = "stabilize"
            self.gimbal_mode_pub.publish(mode_msg)


            x_des = self.home_pos
            q_des = (np.deg2rad(-90),0)

            if current_time - self.state_start_time > 5.0:
                self.state_start_time = current_time
                self.state = "UNTUCK_ARM"
                print("UNTUCK ARM")

        elif self.state == "UNTUCK_ARM":
            x_des = self.home_pos
            q_des = (0, 0)

            if current_time - self.state_start_time > 5.0:
                self.state = "GO_TO_TARGET_HIGH"
                print("GO TO TARGET 1 HIGH")


        elif self.state == "GO_TO_TARGET_HIGH":

            target = self.targets[self.current_target]
            if target is None:
                return

            x_des = target + np.array([0.0, 0.0, 0.6])  #60 cm above
            # q_des = (0, 0)
            q_des = (self.null_q1_travel, self.null_q2_travel)

            # print(np.linalg.norm(x_des - np.array([self.x_ee, self.y_ee, self.z_ee])))
            
            if np.linalg.norm(x_des - np.array([self.x_ee, self.y_ee, self.z_ee])) < 0.1: #10 cm from desired pos
                self.state = "GO_TO_TARGET_LOW"
                print("GO TO TARGET LOW")
                
        elif self.state == "GO_TO_TARGET_LOW":
            
            target = self.targets[self.current_target]

            x_des = target + np.array([0.0, 0.0, 0.2]) #20cm above target
            # q_des = (np.deg2rad(-60), np.deg2rad(110))
            q_des = (self.null_q1, self.null_q2)
            
            
            if np.linalg.norm(x_des - np.array([self.x_ee, self.y_ee, self.z_ee])) < 0.05: # 5 cm from desired pos
                self.state_start_time = current_time
                self.state = "HOVER_TARGET"
                print("HOVER TARGET")

        elif self.state == "HOVER_TARGET":

            target = self.targets[self.current_target]
            
            x_des = target + np.array([0.0, 0.0, 0.2]) #20cm above target
            # q_des = (np.deg2rad(-60), np.deg2rad(110))
            q_des = (self.null_q1, self.null_q2)

            # print(np.linalg.norm(x_des - np.array([self.x_ee, self.y_ee, self.z_ee])))

            if current_time - self.state_start_time > 10.0:
                self.state = "ASCEND_OVER_TARGET"
                print("ASCEND OVER TARGET")

        elif self.state == "ASCEND_OVER_TARGET":
                target = self.targets[self.current_target]
                
                x_des = target + np.array([0.0, 0.0, 0.6])
                q_des = (self.null_q1, self.null_q2)

                if np.linalg.norm(x_des - np.array([self.x_ee, self.y_ee, self.z_ee])) < 0.05:
                    self.current_target += 1
                    if self.current_target >= 4:
                        self.state = "GO_HOME"
                        print("GO HOME")
                    else:
                        self.state = "GO_TO_TARGET_HIGH"
                        print("GO TO TARGET ", self.current_target + 1,  "HIGH")


        elif self.state == "GO_HOME":
            x_des = self.home_pos
            q_des = (0, 0)
            self.null_q1 = 0
            self.null_q2 = 0
            if np.linalg.norm(self.home_pos - np.array([self.x_ee, self.y_ee, self.z_ee])) < 0.1:
                print("IM HOME NERDS")
                self.state_start_time = current_time
                self.state = "TUCK_ARM"

        elif self.state == "TUCK_ARM":
            x_des = self.home_pos
            q_des = (np.deg2rad(-90), 0)

            if current_time - self.state_start_time > 5.0:
                self.state = "TUCK_GIMBAL"
                self.state_start_time = current_time
                print("TUCKING GIMBAL")

        elif self.state == "TUCK_GIMBAL":
            mode_msg = String()
            mode_msg.data = "tuck"
            self.gimbal_mode_pub.publish(mode_msg)

            x_des = self.home_pos
            q_des = (np.deg2rad(-90),0)

            if current_time - self.state_start_time > 5.0:
                self.state = "HOME"
                print("READY FOR LANDING")

        elif self.state == "HOME":
            x_des = self.home_pos
            q_des = (np.deg2rad(-90), 0)


        #Calculate desired movement velocity
        p_des_vel = self.desired_position_velocity(x_des, k=0.2)

        # Null space (joint 1, joint 2 and yaw) and for obstacle avoidance using repulsion
        v = self.null_space_armJoint_and_yaw(des_yaw=0.0, des_joints=q_des, k_null_yaw=0, k_null_joints=0.8, current_target=self.current_target, k_repulsion=15)


        # Jacobian
        J = self.mobile_jacobian()

        # Moore Penrose Pseudoinverse of the jacobian J
        J_pinv = J.T @ np.linalg.inv(J @ J.T)

        # Calculate q_dot using Jacobian J and null space (desired joints & yaw)
        I = np.eye(6)
        q_dot = J_pinv @ p_des_vel + (I - J_pinv @ J) @ v

        # Publish position commands and joint commands
        self.publish_q_dot(q_dot)

        # Increment  time
        self.t += self.dt  # speed (bigger = faster)


def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
