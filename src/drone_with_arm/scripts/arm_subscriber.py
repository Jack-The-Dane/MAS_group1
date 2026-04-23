#!/usr/bin/env python3

import rclpy
from read_write_test import ControlMotors
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from mavros_msgs.msg import State

DEG_PER_TIC = 360/4096

offsets = {
    1: 1170,
    2: 1900,
    3: 2048,
    4: 2048
}

def deg_to_tick(deg: float, offset: int):
    return int(deg/DEG_PER_TIC + offset)

def almost_equal(a: list[int], b: list[int], delta = 200):
    for i in range(len(a)):
        if abs(a[i]-b[i]) > delta:
            return False
    return True

class TuckCommands:
    LIFT_ARM = [deg_to_tick(0, offsets[1]), deg_to_tick(90, offsets[2]), deg_to_tick(0, offsets[3]), deg_to_tick(0, offsets[4])]
    TUCK_GIMBAL = [deg_to_tick(0, offsets[1]), deg_to_tick(90, offsets[2]), deg_to_tick(90, offsets[3]), deg_to_tick(0, offsets[4])]
    UNTUCK_GIMBAL = [deg_to_tick(0, offsets[1]), deg_to_tick(90, offsets[2]), deg_to_tick(0, offsets[3]), deg_to_tick(0, offsets[4])]
    LOWER_ARM = [deg_to_tick(0, offsets[1]), deg_to_tick(0, offsets[2]), deg_to_tick(0, offsets[3]), deg_to_tick(0, offsets[4])]

class ArmStates:
    TUCK_ARM = "TUCK_ARM"
    TUCK_GIMBAL = "TUCK_GIMBAL"
    TUCKED = "TUCKED"
    UNTUCK_GIMBAL = "UNTUCK_GIMBAL"
    UNTUCK_ARM = "UNTUCK_ARM"
    UNTUCKED = "UNTUCKED"
    TUCKING = [TUCK_ARM, TUCK_GIMBAL, TUCKED]
    UNTUCKING = [UNTUCK_ARM, UNTUCK_GIMBAL, UNTUCKED]

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
        self.arm_state = ArmStates.TUCK_ARM
        self.drone_state_subscriber = self.create_subscription(State, "/mavros/state", self.state_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        #communication_port = "/dev/U2D2"
        communication_port = "/dev/ttyUSB0"
        self.motors = ControlMotors([1,2,3,4], communication_port)

        self.current_targets = [1200, 1900, 2048, 2048]
    
    def timer_callback(self):
        current_state = self.arm_state
        new_state = self.arm_state
        match self.arm_state:
            case ArmStates.UNTUCK_GIMBAL:
                self.motors.set_position(TuckCommands.UNTUCK_GIMBAL)
                current_pos = self.motors.get_position()
                if almost_equal(current_pos, TuckCommands.UNTUCK_GIMBAL):
                    new_state = ArmStates.UNTUCK_ARM
            
            case ArmStates.UNTUCK_ARM:
                self.motors.set_position(TuckCommands.LOWER_ARM)
                current_pos = self.motors.get_position()
                if almost_equal(current_pos, TuckCommands.LOWER_ARM):
                    new_state = ArmStates.UNTUCKED
            
            case ArmStates.TUCK_ARM:
                self.motors.set_position(TuckCommands.LIFT_ARM)
                current_pos = self.motors.get_position()
                print(current_pos)
                if almost_equal(current_pos, TuckCommands.LIFT_ARM):
                    new_state = ArmStates.TUCK_GIMBAL
            
            case ArmStates.TUCK_GIMBAL:
                self.motors.set_position(TuckCommands.TUCK_GIMBAL)
                current_pos = self.motors.get_position()
                if almost_equal(current_pos, TuckCommands.TUCK_GIMBAL):
                    new_state = ArmStates.TUCKED

            case _:
                return
        
        if current_state != new_state:
            self.arm_state = new_state
            print(f"Switching arm from state {current_state} to state {new_state}")
        return

    def state_callback(self, msg:State):
        if msg.mode != "OFFBOARD" or not msg.armed:
            if self.arm_state not in ArmStates.TUCKING:
                self.arm_state = ArmStates.TUCK_ARM
        else:
            if self.arm_state not in ArmStates.UNTUCKING:
                self.arm_state = ArmStates.UNTUCK_GIMBAL
    
    def armController_callback(self, msg):
        # Ignore all commands when not untucked
        if self.arm_state != ArmStates.UNTUCKED:
            return
        #convert to degrees
        joint1 = -180/3.14 * msg.data[0]
        joint2 = 180/3.14 * msg.data[1]

        # plus 2047 to center in the middle
        # times 360/4096 to convert degress to positon bc 4096 [pulse/rev]
        joint1 = int(1900 + joint1 * 4096 / 360)
        joint2 = int(1170 + joint2 * 4096 / 360)

        #limit values to what it accepts
        self.current_targets[1] = max(-400, min(4095, joint1)) #1200 # MOTOR index 2 (Joint 1)
        self.current_targets[0] = max(-1000, min(4095, joint2)) #1900 # MOTOR index 1 (joint 2)


        print("curr target: ", self.current_targets)

        self.motors.set_position(self.current_targets)


    def gimbalController_callback(self, msg):
        # Ignore all commands when not untucked
        if self.arm_state != ArmStates.UNTUCKED:
            return
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
