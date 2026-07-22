import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim_msgs.msg import Pose

from first_interfaces.srv import SetBackground


class CircleServiceClient(Node):

    def __init__(self):
        super().__init__('circle_service_client')

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

        self.client = self.create_client(
            SetBackground,
            '/set_background'
        )

        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.total_angle = 0.0
        self.previous_theta = None
        self.quarter = 0

        self.motion_done = False
        self.response_received = False

        self.get_logger().info(
            'Circle Service Client Node Started'
        )

    def pose_callback(self, msg):
        if self.motion_done:
            return

        if self.previous_theta is None:
            self.previous_theta = msg.theta
            return

        diff = msg.theta - self.previous_theta

        if diff > math.pi:
            diff -= 2 * math.pi
        elif diff < -math.pi:
            diff += 2 * math.pi

        self.total_angle += abs(diff)
        self.previous_theta = msg.theta

        current_quarter = int(
            self.total_angle / (math.pi / 2)
        )

        if current_quarter > self.quarter:
            self.quarter = min(current_quarter, 4)
            self.get_logger().info(
                f'Progress: {self.quarter}/4'
            )

        if self.total_angle >= 2 * math.pi:
            self.motion_done = True

            self.get_logger().info(
                'Circle completed. Requesting background change.'
            )

            stop_msg = Twist()
            self.publisher.publish(stop_msg)

            self.send_background_request(
                100,
                200,
                255
            )

    def send_background_request(self, r, g, b):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(
                'Background server is not available.'
            )
            self.response_received = True
            return

        request = SetBackground.Request()
        request.r = r
        request.g = g
        request.b = b

        future = self.client.call_async(request)
        future.add_done_callback(
            self.background_response_callback
        )

    def background_response_callback(self, future):
        try:
            response = future.result()

            if response.success:
                self.get_logger().info(
                    f'배경색 변경 성공: {response.message}'
                )
            else:
                self.get_logger().warning(
                    f'배경색 변경 실패: {response.message}'
                )

        except Exception as error:
            self.get_logger().error(
                f'Service 요청 처리 중 오류 발생: {error}'
            )

        self.response_received = True

    def timer_callback(self):
        if self.motion_done:
            return

        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CircleServiceClient()

    while rclpy.ok() and not node.response_received:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
