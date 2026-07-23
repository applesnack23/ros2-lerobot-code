import csv
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


CSV_FILE = os.path.expanduser(
    '~/project/ros2_ws/so_arm101_teaching.csv'
)

# 각 동작 이후 대기 시간
MOVE_DELAY = 2.0
GRIP_DELAY = 1.0

# 드라이버 연결을 기다리는 최대 시간
DRIVER_WAIT_TIMEOUT = 5.0

ARM_JOINT_NAMES = [
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
]

GRIPPER_JOINT_NAME = 'gripper'


def load_teaching_data_csv(filename):
    teaching_data = {
        'points': {},
        'gripper': {
            'grip': None,
            'ungrip': None,
        },
    }

    with open(
        filename,
        'r',
        newline='',
        encoding='utf-8-sig'
    ) as file:
        reader = csv.DictReader(file)

        for row in reader:
            row_type = row.get('type', '').strip()
            name = row.get('name', '').strip()

            if row_type == 'point':
                positions = {}

                for joint_name in ARM_JOINT_NAMES:
                    value = row.get(joint_name, '').strip()

                    if value == '':
                        raise ValueError(
                            f'Missing {joint_name} value '
                            f'in Teaching Point {name}'
                        )

                    positions[joint_name] = float(value)

                teaching_data['points'][name] = positions

            elif row_type == 'gripper':
                value = row.get(
                    GRIPPER_JOINT_NAME,
                    ''
                ).strip()

                if name not in ['grip', 'ungrip']:
                    continue

                if value == '':
                    raise ValueError(
                        f'Missing gripper value: {name}'
                    )

                teaching_data['gripper'][name] = float(value)

    return teaching_data


class AutoPickPlaceNode(Node):

    def __init__(self):
        super().__init__('auto_pick_place_node')

        self.joint_goal_pub = self.create_publisher(
            JointState,
            '/so_arm101/joint_goal',
            10,
        )

        if not os.path.exists(CSV_FILE):
            raise FileNotFoundError(
                f'CSV file not found: {CSV_FILE}'
            )

        self.teaching_data = load_teaching_data_csv(
            CSV_FILE
        )

        self.validate_teaching_data()

        self.get_logger().info(
            'SO-ARM101 Auto Pick & Place Node started.'
        )
        self.get_logger().info(
            f'Loaded CSV: {CSV_FILE}'
        )

    def validate_teaching_data(self):
        required_points = ['1', '2', '3']

        for point_name in required_points:
            if point_name not in self.teaching_data['points']:
                raise ValueError(
                    f'Teaching Point {point_name} is not saved.'
                )

        for mode in ['grip', 'ungrip']:
            if self.teaching_data['gripper'][mode] is None:
                raise ValueError(
                    f'Gripper position is not saved: {mode}'
                )

    def wait_for_driver(self):
        self.get_logger().info(
            'Waiting for feetech_driver_node...'
        )

        start_time = time.time()

        while rclpy.ok():
            subscriber_count = (
                self.joint_goal_pub.get_subscription_count()
            )

            if subscriber_count > 0:
                self.get_logger().info(
                    'feetech_driver_node connected.'
                )
                return True

            elapsed_time = time.time() - start_time

            if elapsed_time >= DRIVER_WAIT_TIMEOUT:
                self.get_logger().error(
                    'feetech_driver_node was not found.'
                )
                return False

            time.sleep(0.1)

        return False

    def move_arm(self, point_name):
        point_data = self.teaching_data[
            'points'
        ][point_name]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ARM_JOINT_NAMES.copy()
        msg.position = [
            point_data[joint_name]
            for joint_name in ARM_JOINT_NAMES
        ]

        self.joint_goal_pub.publish(msg)

        self.get_logger().info(
            f'Move -> Point {point_name}'
        )

        for name, position in zip(
            msg.name,
            msg.position
        ):
            self.get_logger().info(
                f'  {name}: {position}'
            )

    def move_gripper(self, mode):
        position = self.teaching_data[
            'gripper'
        ][mode]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [GRIPPER_JOINT_NAME]
        msg.position = [position]

        self.joint_goal_pub.publish(msg)

        self.get_logger().info(
            f'Gripper -> {mode}: {position}'
        )

    def run_sequence(self):
        if not self.wait_for_driver():
            return

        self.get_logger().info(
            'Start Auto Pick & Place'
        )

        # 1. Point 1: 대기 위치
        self.move_arm('1')
        time.sleep(MOVE_DELAY)

        # 2. Ungrip: Gripper 열기
        self.move_gripper('ungrip')
        time.sleep(GRIP_DELAY)

        # 3. Point 2: Pick 위치
        self.move_arm('2')
        time.sleep(MOVE_DELAY)

        # 4. Grip: 물체 잡기
        self.move_gripper('grip')
        time.sleep(GRIP_DELAY)

        # 5. Point 1: 대기 위치로 복귀
        self.move_arm('1')
        time.sleep(MOVE_DELAY)

        # 6. Point 3: Place 위치
        self.move_arm('3')
        time.sleep(MOVE_DELAY)

        # 7. Ungrip: 물체 놓기
        self.move_gripper('ungrip')
        time.sleep(GRIP_DELAY)

        # 8. Point 1: 대기 위치로 복귀
        self.move_arm('1')
        time.sleep(MOVE_DELAY)

        self.get_logger().info(
            'Auto Pick & Place Finished'
        )


def main(args=None):
    rclpy.init(args=args)

    try:
        node = AutoPickPlaceNode()

    except (FileNotFoundError, ValueError) as error:
        print(
            f'Auto Pick & Place initialization failed: '
            f'{error}'
        )
        rclpy.shutdown()
        return

    try:
        node.run_sequence()

    except KeyboardInterrupt:
        node.get_logger().warning(
            'Auto Pick & Place interrupted.'
        )

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()