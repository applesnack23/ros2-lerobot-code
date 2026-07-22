import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from first_interfaces.action import TurtleDraw


class DrawClient(Node):

    def __init__(self):
        super().__init__('draw_client')

        self.action_client = ActionClient(
            self,
            TurtleDraw,
            '/turtle_draw'
        )

        self.done = False

        self.get_logger().info(
            'Draw Client Node Started'
        )

    def send_goal(self, length):
        self.get_logger().info(
            'Action 서버 연결을 기다립니다.'
        )

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'Action 서버를 찾을 수 없습니다.'
            )
            self.done = True
            return

        goal_msg = TurtleDraw.Goal()
        goal_msg.length = float(length)

        self.get_logger().info(
            f'Goal 전송: length={length:.2f}'
        )

        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()

            if not goal_handle.accepted:
                self.get_logger().warning(
                    'Goal이 거절되었습니다.'
                )
                self.done = True
                return

            self.get_logger().info(
                'Goal이 수락되었습니다.'
            )

            result_future = goal_handle.get_result_async()

            result_future.add_done_callback(
                self.result_callback
            )

        except Exception as error:
            self.get_logger().error(
                f'Goal 응답 처리 중 오류 발생: {error}'
            )
            self.done = True

    def feedback_callback(self, feedback_msg):
        distance = feedback_msg.feedback.distance

        self.get_logger().info(
            f'남은 거리: {distance:.2f}'
        )

    def result_callback(self, future):
        try:
            wrapped_result = future.result()
            result = wrapped_result.result
            status = wrapped_result.status

            self.get_logger().info(
                f'완료 상태 코드: {status}'
            )

            self.get_logger().info(
                f'완료: success={result.success}, '
                f'message={result.message}, '
                f'elapsed_time={result.elapsed_time:.2f}초'
            )

        except Exception as error:
            self.get_logger().error(
                f'Result 처리 중 오류 발생: {error}'
            )

        self.done = True


def main(args=None):
    rclpy.init(args=args)

    node = DrawClient()
    node.send_goal(2.0)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        node.get_logger().info(
            '사용자가 클라이언트를 종료했습니다.'
        )

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
