import csv
import os
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


CSV_FILE = "so_arm101_teaching.csv"

ARM_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]

GRIPPER_JOINT_NAME = "gripper"


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    return key


class TeachingNode(Node):
    def __init__(self):
        super().__init__("teaching_node")

        self.current_joint_positions = {}

        self.teaching_data = {
            "points": {
                "1": None,
                "2": None,
                "3": None,
            },
            "gripper": {
                "grip": None,
                "ungrip": None,
            },
        }

        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.get_logger().info("SO-ARM101 Teaching Node started.")
        self.print_menu()

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.current_joint_positions[name] = position

    def print_menu(self):
        print("====================================")
        print(" SO-ARM101 ROS2 Teaching Program")
        print("====================================")
        print("1 : Save Teaching Point 1")
        print("2 : Save Teaching Point 2")
        print("3 : Save Teaching Point 3")
        print("+ : Save Grip Position")
        print("- : Save Ungrip Position")
        print("q : Quit and Save")
        print("====================================")

    def check_joint_data_ready(self):
        if len(self.current_joint_positions) == 0:
            print("\nNo /joint_states data received yet.")
            print("Run feetech_driver_node first.")
            return False

        return True

    def read_arm_positions(self):
        if not self.check_joint_data_ready():
            return None

        positions = {}

        for joint_name in ARM_JOINT_NAMES:
            if joint_name not in self.current_joint_positions:
                print(f"\nJoint not found: {joint_name}")
                print("Available joints:")
                for name in self.current_joint_positions.keys():
                    print(f" - {name}")
                return None

            positions[joint_name] = self.current_joint_positions[joint_name]

        return positions

    def read_gripper_position(self):
        if not self.check_joint_data_ready():
            return None

        if GRIPPER_JOINT_NAME not in self.current_joint_positions:
            print(f"\nGripper joint not found: {GRIPPER_JOINT_NAME}")
            print("Available joints:")
            for name in self.current_joint_positions.keys():
                print(f" - {name}")
            return None

        return self.current_joint_positions[GRIPPER_JOINT_NAME]

    def save_teaching_data_csv(self):
        output_path = os.path.join(os.getcwd(), CSV_FILE)

        headers = [
            "type",
            "name",
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
            "gripper",
        ]

        with open(output_path, "w", newline="", encoding="utf-8-sig") as f:
            writer = csv.DictWriter(f, fieldnames=headers)
            writer.writeheader()

            for point_name, positions in self.teaching_data["points"].items():
                if positions is None:
                    continue

                writer.writerow({
                    "type": "point",
                    "name": point_name,
                    "shoulder_pan": positions["shoulder_pan"],
                    "shoulder_lift": positions["shoulder_lift"],
                    "elbow_flex": positions["elbow_flex"],
                    "wrist_flex": positions["wrist_flex"],
                    "wrist_roll": positions["wrist_roll"],
                    "gripper": "",
                })

            if self.teaching_data["gripper"]["grip"] is not None:
                writer.writerow({
                    "type": "gripper",
                    "name": "grip",
                    "shoulder_pan": "",
                    "shoulder_lift": "",
                    "elbow_flex": "",
                    "wrist_flex": "",
                    "wrist_roll": "",
                    "gripper": self.teaching_data["gripper"]["grip"],
                })

            if self.teaching_data["gripper"]["ungrip"] is not None:
                writer.writerow({
                    "type": "gripper",
                    "name": "ungrip",
                    "shoulder_pan": "",
                    "shoulder_lift": "",
                    "elbow_flex": "",
                    "wrist_flex": "",
                    "wrist_roll": "",
                    "gripper": self.teaching_data["gripper"]["ungrip"],
                })

        print(f"\nSaved: {output_path}")

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            key = get_key()

            if key in ["1", "2", "3"]:
                positions = self.read_arm_positions()

                if positions is None:
                    continue

                self.teaching_data["points"][key] = positions
                print(f"\nTeaching Point {key} saved")
                print(positions)

            elif key == "+":
                grip_pos = self.read_gripper_position()

                if grip_pos is None:
                    continue

                self.teaching_data["gripper"]["grip"] = grip_pos
                print("\nGrip position saved")
                print("gripper:", grip_pos)

            elif key == "-":
                ungrip_pos = self.read_gripper_position()

                if ungrip_pos is None:
                    continue

                self.teaching_data["gripper"]["ungrip"] = ungrip_pos
                print("\nUngrip position saved")
                print("gripper:", ungrip_pos)

            elif key.lower() == "q":
                print("\nExit requested.")
                save = input("Save teaching data? (y/n): ")

                if save.lower() == "y":
                    self.save_teaching_data_csv()
                else:
                    print("Not saved.")

                break


def main(args=None):
    rclpy.init(args=args)

    node = TeachingNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()