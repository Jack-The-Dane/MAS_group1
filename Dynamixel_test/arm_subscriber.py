import rclpy
from read_write_test import ControlMotor
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        port_aksel_pc = "/dev/ttyUSB0"
        self.motor1 = ControlMotor(1, port_aksel_pc)




    def listener_callback(self, msg):

        while True:
            try:
                target_position = int(input("Enter target position (0 ~ 4095, -1 to exit): "))
            except ValueError:
                print("Please enter an integer.")
                continue

            if target_position == -1:
                break
            elif target_position < 0 or target_position > 4095:
                print("Position must be between 0 and 4095.")
                continue

            self.motor1.set_position(target_position)

            while True:
                present_position = self.motor1.get_position()
                print(f"Current Position: {present_position}")
                if abs(target_position - present_position) <= 10:
                    break


        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()