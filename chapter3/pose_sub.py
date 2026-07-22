import rclpy
from rclpy.node import Node
from turtlesim_msgs.msg import Pose


class PoseSubscriber(Node):

    def __init__(self):
        super().__init__('pose_subscriber')

        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        self.get_logger().info('Pose Subscriber Node Started')

    def pose_callback(self, msg):
        self.get_logger().info(
            f'x: {msg.x:.2f}, '
            f'y: {msg.y:.2f}, '
            f'theta: {msg.theta:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = PoseSubscriber()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
