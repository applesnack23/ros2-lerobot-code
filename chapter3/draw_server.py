import asyncio
import math
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim_msgs.msg import Pose

from first_interfaces.action import TurtleDraw
from first_interfaces.srv import SetBackground


class DrawServer(Node):

    def __init__(self):
        super().__init__('draw_server')

        self.callback_group = ReentrantCallbackGroup()

        self.action_server = ActionServer(
            self,
            TurtleDraw,
            '/turtle_draw',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        self.publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10,
            callback_group=self.callback_group
        )

        self.bg_client = self.create_client(
            SetBackground,
            '/set_background',
            callback_group=self.callback_group
        )

        self.current_pose = None
        self.dt = 0.1

        self.get_logger().info('Draw Server Started')

    def goal_callback(self, goal_request):
        radius = goal_request.length

        self.get_logger().info(
            f'Goal 수신: radius={radius:.2f}'
        )

        if radius <= 0.0:
            self.get_logger().warning(
                '반지름은 0보다 커야 합니다. Goal을 거절합니다.'
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('취소 요청을 수신했습니다.')
        return CancelResponse.ACCEPT

    def pose_callback(self, msg):
        self.current_pose = msg

    def send_background(self, r, g, b):
        if not self.bg_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(
                'Background Service 서버를 찾을 수 없습니다.'
            )
            return

        request = SetBackground.Request()
        request.r = r
        request.g = g
        request.b = b

        future = self.bg_client.call_async(request)
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
                f'배경색 변경 요청 중 오류 발생: {error}'
            )

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Goal 실행을 시작합니다.')

        radius = goal_handle.request.length
        circumference = 2.0 * math.pi * radius

        angular_speed = 1.0
        linear_speed = radius * angular_speed

        feedback_msg = TurtleDraw.Feedback()
        result = TurtleDraw.Result()

        while self.current_pose is None:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()

                result.success = False
                result.message = 'Pose 수신 대기 중 취소됨'
                result.elapsed_time = 0.0

                return result

            await asyncio.sleep(self.dt)

        previous_theta = self.current_pose.theta
        total_angle = 0.0
        start_time = time.time()

        while rclpy.ok():

            if goal_handle.is_cancel_requested:
                self.publish_stop()
                goal_handle.canceled()

                self.send_background(255, 0, 0)

                elapsed_time = time.time() - start_time

                result.success = False
                result.message = '원 그리기가 취소되었습니다.'
                result.elapsed_time = float(elapsed_time)

                self.get_logger().info(
                    'Goal이 취소되어 배경색을 빨간색으로 변경합니다.'
                )

                return result

            current_theta = self.current_pose.theta
            diff = current_theta - previous_theta

            if diff > math.pi:
                diff -= 2.0 * math.pi
            elif diff < -math.pi:
                diff += 2.0 * math.pi

            total_angle += abs(diff)
            previous_theta = current_theta

            traveled_distance = radius * total_angle

            remaining_distance = max(
                circumference - traveled_distance,
                0.0
            )

            feedback_msg.distance = float(remaining_distance)
            goal_handle.publish_feedback(feedback_msg)

            if total_angle >= 2.0 * math.pi:
                break

            move_msg = Twist()
            move_msg.linear.x = linear_speed
            move_msg.angular.z = angular_speed

            self.publisher.publish(move_msg)

            await asyncio.sleep(self.dt)

        self.publish_stop()

        elapsed_time = time.time() - start_time

        self.send_background(0, 255, 0)

        goal_handle.succeed()

        result.success = True
        result.message = '원 그리기를 완료했습니다.'
        result.elapsed_time = float(elapsed_time)

        self.get_logger().info(
            '완주하여 배경색을 초록색으로 변경합니다.'
        )

        return result

    def publish_stop(self):
        stop_msg = Twist()
        self.publisher.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)

    node = DrawServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
