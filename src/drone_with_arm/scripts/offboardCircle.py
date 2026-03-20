#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy



class CircleTest(Node):

    def __init__(self):
        super().__init__("circle_test")

        self.pub = self.create_publisher(
            PoseStamped,
            "/mavros/setpoint_position/local",
            10
        )

        self.create_subscription(
            State,
            "/mavros/state",
            self.state_callback,
            10
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile
        )

        self.state = State()

        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.pos = None

        self.reached_start = False
        self.start_time = None
        self.reached_end = False

        # Circle 
        self.r = 2.0
        self.z = 2.0
        self.period = 40.0




    def state_callback(self, msg):
        self.state = msg

    def pose_callback(self, msg):
        self.pos = msg.pose.position

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish(self, x, y, z):
        msg = PoseStamped()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        self.pub.publish(msg)

    def timer_callback(self):

        # Wait for OFFBOARD + armed
        if not (self.state.mode == "OFFBOARD" and self.state.armed):
            self.publish(0.0, 0.0, self.z)
            return

        if self.pos is None:
            return

        if not self.reached_start: # Vent på den når start pos
            p = self.pos
            dx = p.x - 2.0
            dy = p.y - 0.0
            dz = p.z - 2.0
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)

            if dist < 0.05:
                self.reached_start = True
                self.get_logger().info("Reached start point")
            else:
                self.publish(2.0, 0.0, 2.0)
                return        

        if self.start_time is None:
            self.start_time = self.now()
            self.get_logger().info("Starting circle")

        t = self.now() - self.start_time

        if t >= self.period: # stop efter 1 omgang :)
            self.publish(2.0, 0.0, 2.0)
            if not self.reached_end:
                self.reached_end = True
                self.get_logger().info("mom im finish")
            return

        omega = 2 * math.pi / self.period
        theta = omega * t

        x = self.r * math.cos(theta)
        y = self.r * math.sin(theta)

        self.publish(x, y, self.z)


def main():
    rclpy.init()
    node = CircleTest()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


















# import math

# import rclpy
# from rclpy.node import Node

# from geometry_msgs.msg import PoseStamped
# from mavros_msgs.msg import State

# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


# class OffboardCircleTest(Node):
#     def __init__(self):
#         super().__init__('offboard_circle_test')

#         # Publishers
#         self.setpoint_pub = self.create_publisher(
#             PoseStamped,
#             '/mavros/setpoint_position/local',
#             10
#         )

#         # Subscribers
#         self.state_sub = self.create_subscription(
#             State,
#             '/mavros/state',
#             self.state_callback,
#             10
#         )

#         qos_profile = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=10
#         )

#         self.pose_sub = self.create_subscription(
#             PoseStamped,
#             '/mavros/local_position/pose',
#             self.pose_callback,
#             qos_profile
#         )

#         # Timer
#         self.dt = 0.02  # 50 Hz
#         self.timer = self.create_timer(self.dt, self.timer_callback)

#         # MAVROS state
#         self.current_state = State()
#         self.current_pose = None
#         self.have_pose = False

#         # Mission state machine
#         self.phase = 'WAIT_FOR_OFFBOARD'
#         self.phase_start_time = None

#         # Mission parameters
#         self.goto_target = (2.0, 0.0, 2.0)

#         # Circle centered at origin, radius 2 m, altitude 2 m
#         # Starts at (2,0,2)
#         self.circle_center = (0.0, 0.0)
#         self.circle_radius = 2.0
#         self.circle_altitude = 2.0
#         self.circle_period = 20.0   # seconds for one full circle
#         self.circle_laps = 1.0
#         self.circle_duration = self.circle_period * self.circle_laps

#         # Hold point before Offboard is engaged
#         self.preoffboard_hold = None

#         # Final hold setpoint
#         self.final_hold = None

#         self.get_logger().info('Offboard circle test node started.')

#     def state_callback(self, msg: State):
#         self.current_state = msg

#     def pose_callback(self, msg: PoseStamped):
#         self.current_pose = msg
#         self.have_pose = True

#         if self.preoffboard_hold is None:
#             self.preoffboard_hold = (
#                 msg.pose.position.x,
#                 msg.pose.position.y,
#                 msg.pose.position.z
#             )

#     def now_sec(self):
#         return self.get_clock().now().nanoseconds * 1e-9

#     def publish_setpoint(self, x, y, z, yaw=0.0):
#         msg = PoseStamped()
#         msg.header.stamp = self.get_clock().now().to_msg()

#         msg.pose.position.x = x
#         msg.pose.position.y = y
#         msg.pose.position.z = z

#         cy = math.cos(yaw * 0.5)
#         sy = math.sin(yaw * 0.5)
#         msg.pose.orientation.x = 0.0
#         msg.pose.orientation.y = 0.0
#         msg.pose.orientation.z = sy
#         msg.pose.orientation.w = cy

#         self.setpoint_pub.publish(msg)

#     def distance_to(self, x, y, z):
#         if not self.have_pose:
#             return float('inf')

#         px = self.current_pose.pose.position.x
#         py = self.current_pose.pose.position.y
#         pz = self.current_pose.pose.position.z

#         return math.sqrt((x - px)**2 + (y - py)**2 + (z - pz)**2)

#     def timer_callback(self):
#         if not self.have_pose:
#             return

#         # Always publish something
#         if self.phase == 'WAIT_FOR_OFFBOARD':
#             # Before OFFBOARD is engaged, keep publishing the current/initial hold point
#             x, y, z = self.preoffboard_hold
#             self.publish_setpoint(x, y, z, yaw=0.0)

#             if self.current_state.mode == 'OFFBOARD' and self.current_state.armed:
#                 self.phase = 'GO_TO_START'
#                 self.phase_start_time = self.now_sec()
#                 self.get_logger().info('OFFBOARD detected, going to (2, 0, 2).')

#             return

#         if self.phase == 'GO_TO_START':
#             x, y, z = self.goto_target
#             self.publish_setpoint(x, y, z, yaw=0.0)

#             if self.distance_to(x, y, z) < 0.20:
#                 self.phase = 'CIRCLE'
#                 self.phase_start_time = self.now_sec()
#                 self.get_logger().info('Reached start point, starting circle.')

#             return

#         if self.phase == 'CIRCLE':
#             t = self.now_sec() - self.phase_start_time

#             omega = 2.0 * math.pi / self.circle_period
#             theta = omega * t

#             cx, cy = self.circle_center
#             r = self.circle_radius
#             z = self.circle_altitude

#             x = cx + r * math.cos(theta)
#             y = cy + r * math.sin(theta)

#             self.publish_setpoint(x, y, z, yaw=0.0)

#             if t >= self.circle_duration:
#                 self.phase = 'HOLD'
#                 self.final_hold = (x, y, z)
#                 self.get_logger().info('Circle complete, holding final position.')

#             return

#         if self.phase == 'HOLD':
#             x, y, z = self.final_hold
#             self.publish_setpoint(x, y, z, yaw=0.0)
#             return


# def main(args=None):
#     rclpy.init(args=args)
#     node = OffboardCircleTest()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
