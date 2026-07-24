import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)


ARM_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]

GRIPPER_JOINT_NAMES = [
    "gripper",
]


# ========================================
# 사용자의 로봇 환경에 맞게 수정할 위치값
# ========================================

WAIT_POSITION = [
    0.0,    # shoulder_pan
    -0.4,   # shoulder_lift
    0.4,    # elbow_flex
    1.2,    # wrist_flex
    0.0,    # wrist_roll
]

PICK_POSITION = [
    -0.3,   # shoulder_pan
    0.2,    # shoulder_lift
    0.3,    # elbow_flex
    1.0,    # wrist_flex
    -0.4,   # wrist_roll
]

PLACE_POSITION = [
    0.2,    # shoulder_pan
    0.4,    # shoulder_lift
    0.0,    # elbow_flex
    1.2,    # wrist_flex
    0.2,    # wrist_roll
]

GRIPPER_OPEN = 0.5
GRIPPER_CLOSE = 0.27


class PickPlaceSequenceNode(Node):

    def __init__(self):
        super().__init__("pick_place_sequence_node")

        self.arm_pub = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            10,
        )

        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            "/gripper_controller/joint_trajectory",
            10,
        )

        self.get_logger().info(
            "Pick & Place Sequence Node Started"
        )

    def move_arm(self, positions, sec=3):
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = sec

        msg.points.append(point)

        self.arm_pub.publish(msg)

        self.get_logger().info(
            f"Move arm: {positions}, duration={sec}s"
        )

        time.sleep(sec + 0.5)

    def move_gripper(self, position, sec=1):
        msg = JointTrajectory()
        msg.joint_names = GRIPPER_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = sec

        msg.points.append(point)

        self.gripper_pub.publish(msg)

        self.get_logger().info(
            f"Move gripper: {position}, duration={sec}s"
        )

        time.sleep(sec + 0.5)

    def run_pick_place(self):
        self.get_logger().info("Pick & Place Start")

        # 1. 대기 위치
        self.move_arm(
            WAIT_POSITION,
            sec=3,
        )

        # 2. Gripper 열기
        self.move_gripper(
            GRIPPER_OPEN,
            sec=1,
        )

        # 3. Pick 위치
        self.move_arm(
            PICK_POSITION,
            sec=3,
        )

        # 4. Gripper 닫기
        self.move_gripper(
            GRIPPER_CLOSE,
            sec=1,
        )

        # 5. 대기 위치로 복귀
        self.move_arm(
            WAIT_POSITION,
            sec=3,
        )

        # 6. Place 위치
        self.move_arm(
            PLACE_POSITION,
            sec=3,
        )

        # 7. Gripper 열기
        self.move_gripper(
            GRIPPER_OPEN,
            sec=1,
        )

        # 8. 대기 위치로 복귀
        self.move_arm(
            WAIT_POSITION,
            sec=3,
        )

        self.get_logger().info("Pick & Place Done")


def main(args=None):
    rclpy.init(args=args)

    node = PickPlaceSequenceNode()

    # Publisher와 Controller의 연결 대기
    time.sleep(1.0)

    try:
        node.run_pick_place()

    except KeyboardInterrupt:
        node.get_logger().info(
            "Pick & Place Interrupted"
        )

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()