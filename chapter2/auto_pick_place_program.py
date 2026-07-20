import csv
import time

from lerobot.motors.feetech.feetech import FeetechMotorsBus
from lerobot.motors.motors_bus import Motor


CSV_FILE = "so_arm101_teaching.csv"
AUTO_DELAY = 1.0

motors = {
    "shoulder_pan": Motor(id=1, model="sts3215", norm_mode="position"),
    "shoulder_lift": Motor(id=2, model="sts3215", norm_mode="position"),
    "elbow_flex": Motor(id=3, model="sts3215", norm_mode="position"),
    "wrist_flex": Motor(id=4, model="sts3215", norm_mode="position"),
    "wrist_roll": Motor(id=5, model="sts3215", norm_mode="position"),
    "gripper": Motor(id=6, model="sts3215", norm_mode="position"),
}

arm_motor_names = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]

grip_motor_name = "gripper"


def load_teaching_data_csv(filename):
    teaching_data = {
        "points": {},
        "gripper": {
            "grip": None,
            "ungrip": None,
        }
    }

    with open(filename, "r", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)

        for row in reader:
            row_type = row["type"]
            name = row["name"]

            if row_type == "point":
                teaching_data["points"][name] = {
                    "shoulder_pan": int(row["shoulder_pan"]),
                    "shoulder_lift": int(row["shoulder_lift"]),
                    "elbow_flex": int(row["elbow_flex"]),
                    "wrist_flex": int(row["wrist_flex"]),
                    "wrist_roll": int(row["wrist_roll"]),
                }

            elif row_type == "gripper":
                teaching_data["gripper"][name] = int(row["gripper"])

    return teaching_data


def check_auto_data(teaching_data):
    for point in ["1", "2", "3"]:
        if point not in teaching_data["points"]:
            raise ValueError(f"Teaching Point {point} is not saved.")

    if teaching_data["gripper"]["grip"] is None:
        raise ValueError("Grip position is not saved.")

    if teaching_data["gripper"]["ungrip"] is None:
        raise ValueError("Ungrip position is not saved.")


def move_arm_to_point(bus, point_data):
    for motor_name in arm_motor_names:
        bus.write(
            "Goal_Position",
            motor_name,
            point_data[motor_name],
            normalize=False
        )

    print("Move arm:", point_data)


def move_gripper(bus, position):
    bus.write(
        "Goal_Position",
        grip_motor_name,
        position,
        normalize=False
    )

    print("Move gripper:", position)


def run_auto_sequence(bus, teaching_data):
    sequence = [
        ("point", "1"),
        ("gripper", "ungrip"),
        ("point", "2"),
        ("gripper", "grip"),
        ("point", "1"),
        ("point", "3"),
        ("gripper", "ungrip"),
        ("point", "1"),
    ]

    print("Auto sequence start")

    for action_type, name in sequence:
        if action_type == "point":
            move_arm_to_point(bus, teaching_data["points"][name])

        elif action_type == "gripper":
            move_gripper(bus, teaching_data["gripper"][name])

        time.sleep(AUTO_DELAY)

    print("Auto sequence finished")


teaching_data = load_teaching_data_csv(CSV_FILE)
check_auto_data(teaching_data)

bus = FeetechMotorsBus(
    port="/dev/ttyACM0",
    motors=motors
)

try:
    bus.connect()
    bus.enable_torque()

    run_auto_sequence(bus, teaching_data)

finally:
    try:
        bus.disable_torque()
    except Exception:
        pass

    try:
        bus.disconnect()
    except Exception:
        pass

    print("Motor bus disconnected.")
