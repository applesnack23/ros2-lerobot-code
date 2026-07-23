import csv
import os
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


CSV_FILE = os.path.expanduser(
    '~/project/ros2_ws/so_arm101_teaching.csv'
)

ARM_JOINT_NAMES = [
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
]

GRIPPER_JOINT_NAME = 'gripper'


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(
            fd,
            termios.TCSADRAIN,
            old_settings
        )

    return key


class PlaybackNode(Node):

    def __init__(self):
        super().__init__('playback_node')

        self.teaching_points = {}
        self.gripper_positions = {}

        self.joint_goal_pub = self.create_publisher(
            JointState,
            '/so_arm101/joint_goal',
            10
        )

        self.load_teaching_data()

        self.get_logger().info(
            'SO-ARM101 Playback Node started.'
        )

        self.print_menu()

    def load_teaching_data(self):
        if not os.path.exists(CSV_FILE):
            raise FileNotFoundError(
                f'Teaching CSV file not found: {CSV_FILE}'
            )

        with open(
            CSV_FILE,
            'r',
            newline='',
            encoding='utf-8-sig'
        ) as file:
            reader = csv.DictReader(file)

            for row in reader:
                data_type = row.get('type', '').strip()
                data_name = row.get('name', '').strip()

                if data_type == 'point':
                    positions = {}

                    for joint_name in ARM_JOINT_NAMES:
                        value = row.get(joint_name, '').strip()

                        if value == '':
                            raise ValueError(
                                f'Missing {joint_name} value '
                                f'in Teaching Point {data_name}'
                            )

                        positions[joint_name] = float(value)

                    self.teaching_points[data_name] = positions

                elif data_type == 'gripper':
                    value = row.get(
                        GRIPPER_JOINT_NAME,
                        ''
                    ).strip()

                    if value == '':
                        raise ValueError(
                            f'Missing gripper value: {data_name}'
                        )

                    self.gripper_positions[data_name] = (
                        float(value)
                    )

        self.get_logger().info(
            f'Teaching data loaded: {CSV_FILE}'
        )

        self.get_logger().info(
            f'Teaching Points: '
            f'{list(self.teaching_points.keys())}'
        )

        self.get_logger().info(
            f'Gripper Positions: '
            f'{list(self.gripper_positions.keys())}'
        )

    def print_menu(self):
        print('====================================')
        print(' SO-ARM101 ROS2 Playback Program')
        print('====================================')
        print('1 : Move to Teaching Point 1')
        print('2 : Move to Teaching Point 2')
        print('3 : Move to Teaching Point 3')
        print('+ : Move to Grip Position')
        print('- : Move to Ungrip Position')
        print('q : Quit')
        print('====================================')

    def publish_arm_goal(self, point_name):
        if point_name not in self.teaching_points:
            self.get_logger().warning(
                f'Teaching Point {point_name} is not saved.'
            )
            return

        positions = self.teaching_points[point_name]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ARM_JOINT_NAMES.copy()
        msg.position = [
            positions[joint_name]
            for joint_name in ARM_JOINT_NAMES
        ]

        self.joint_goal_pub.publish(msg)

        self.get_logger().info(
            f'Move to Teaching Point {point_name}'
        )

        for name, position in zip(
            msg.name,
            msg.position
        ):
            self.get_logger().info(
                f'  {name}: {position}'
            )

    def publish_gripper_goal(self, position_name):
        if position_name not in self.gripper_positions:
            self.get_logger().warning(
                f'Gripper position is not saved: '
                f'{position_name}'
            )
            return

        position = self.gripper_positions[position_name]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [GRIPPER_JOINT_NAME]
        msg.position = [position]

        self.joint_goal_pub.publish(msg)

        self.get_logger().info(
            f'Move Gripper: {position_name} -> {position}'
        )

    def run(self):
        while rclpy.ok():
            key = get_key()

            if key in ['1', '2', '3']:
                self.publish_arm_goal(key)

            elif key == '+':
                self.publish_gripper_goal('grip')

            elif key == '-':
                self.publish_gripper_goal('ungrip')

            elif key.lower() == 'q':
                print('\nExit requested.')
                break

            else:
                print(f'\nUnknown key: {repr(key)}')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = PlaybackNode()

    except (FileNotFoundError, ValueError) as error:
        print(f'Playback initialization failed: {error}')
        rclpy.shutdown()
        return

    try:
        node.run()

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()