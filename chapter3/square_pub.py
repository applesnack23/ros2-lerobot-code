import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SquarePublisher(Node):

    def __init__(self):
        super().__init__('square_publisher')

        self.publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.state = 'MOVING'
        self.elapsed = 0.0
        self.turned_angle = 0.0

        self.move_duration = 2.0
        self.angular_speed = math.pi / 2
        self.dt = 0.1

        self.get_logger().info('Square Publisher Node Started')

    def timer_callback(self):
        msg = Twist()

        if self.state == 'MOVING':
            msg.linear.x = 2.0
            msg.angular.z = 0.0

            self.elapsed += self.dt

            if self.elapsed >= self.move_duration:
                self.state = 'TURNING'
                self.elapsed = 0.0
                self.turned_angle = 0.0

        elif self.state == 'TURNING':
            msg.linear.x = 0.0
            msg.angular.z = self.angular_speed

            self.turned_angle += self.angular_speed * self.dt

            if self.turned_angle >= math.pi / 2 - 0.01:
                self.state = 'MOVING'
                self.elapsed = 0.0
                self.turned_angle = 0.0

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = SquarePublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
