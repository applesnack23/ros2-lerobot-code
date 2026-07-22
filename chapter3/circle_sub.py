import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim_msgs.msg import Pose


class CircleNode(Node):

    def __init__(self):
        super().__init__('circle_node')

        self.publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.linear_speed = 2.0
        self.angular_speed = 1.0

        self.total_angle = 0.0
        self.previous_theta = None
        self.quarter = 0
        self.done = False

        self.get_logger().info('Circle Node Started')

    def pose_callback(self, msg):
        if self.previous_theta is None:
            self.previous_theta = msg.theta
            return

        diff = msg.theta - self.previous_theta

        if diff > math.pi:
            diff -= 2 * math.pi
        elif diff < -math.pi:
            diff += 2 * math.pi

        self.total_angle += diff
        self.previous_theta = msg.theta

        current_quarter = int(
            self.total_angle / (math.pi / 2)
        )

        if current_quarter > self.quarter:
            self.quarter = current_quarter
            self.get_logger().info(
                f'Progress: {self.quarter}/4'
            )

        if self.total_angle >= 2 * math.pi:
            self.get_logger().info(
                'Circle completed. Shutting down.'
            )

            stop_msg = Twist()
            self.publisher.publish(stop_msg)
            self.done = True

    def timer_callback(self):
        if self.done:
            return

        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = CircleNode()

    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
