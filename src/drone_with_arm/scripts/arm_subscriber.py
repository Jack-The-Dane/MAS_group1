#!/usr/bin/env python3

import rclpy
from read_write_test import ControlMotors
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray



class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('control_motors')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # subsribe to armcontroller
        self.armController_subscription = self.create_subscription(
            Float64MultiArray,
            'arm_controller/commands',
            self.armController_callback,
            qos_profile
        )

        self.gimbalController_subscription = self.create_subscription(
            Float64MultiArray,
            '/gimbal_controller/commands',
            self.gimbalController_callback,
            10
        )

        # communication_port = "/dev/U2D2"
        communication_port = "/dev/ttyUSB1"
        self.motors = ControlMotors([1,2,3,4], communication_port)

        self.current_targets = [1200, 1900, 2048, 2048]



    def armController_callback(self, msg):
        #convert to degrees
        joint1 = -180/3.14 * msg.data[0]
        joint2 = 180/3.14 * msg.data[1]

        # plus 2047 to center in the middle
        # times 360/4096 to convert degress to positon bc 4096 [pulse/rev]
        joint1 = int(1900 + joint1 * 4096 / 360)  # motor 1
        joint2 = int(1170 + joint2 * 4096 / 360) # motor 2

        #limit values to what it accepts
        self.current_targets[1] = max(-400, min(4095, joint1)) #1200 # MOTOR index 2 (Joint 1)
        self.current_targets[0] = max(-1000, min(4095, joint2)) #1900 # MOTOR index 1 (joint 2)


        print("curr target: ", self.current_targets)

        self.motors.set_position(self.current_targets)


    def gimbalController_callback(self, msg):
        #Explination above
        roll_gimbal = -180/3.14 * msg.data[1]
        pitch_gimbal = 180/3.14 * msg.data[0]

        roll_gimbal = int(2048 + roll_gimbal * 4096 / 360)   # motor 3
        pitch_gimbal = int(2048 + pitch_gimbal * 4096 / 360)   # motor 4

        self.current_targets[2] = max(0, min(4095, roll_gimbal)) #2048
        self.current_targets[3] = max(0, min(4095, pitch_gimbal)) #2048

        self.motors.set_position(self.current_targets)

    


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()