import csv
import time
import termios
import tty
import sys
from datetime import datetime

from lerobot.motors.feetech.feetech import FeetechMotorsBus
from lerobot.motors.motors_bus import Motor


# =========================
# 1. Motor 설정
# =========================
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


# =========================
# 2. 키 입력 함수
# =========================
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    return key


# =========================
# 3. 값 변환 함수
# =========================
def to_int(value):
    if isinstance(value, list):
        return int(value[0])

    try:
        return int(value)
    except TypeError:
        return int(value.item())


# =========================
# 4. 위치 읽기 함수
# =========================
def read_motor_position(bus, motor_name):
    pos = bus.read(
        "Present_Position",
        motor_name,
        normalize=False
    )
    return to_int(pos)


def read_arm_positions(bus):
    positions = {}

    for name in arm_motor_names:
        positions[name] = read_motor_position(bus, name)

    return positions


def read_grip_position(bus):
    return read_motor_position(bus, grip_motor_name)


# =========================
# 5. CSV 저장 함수
# =========================
def save_teaching_data_csv(teaching_data):
    filename = f"so_arm101_teaching.csv"

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

    with open(filename, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=headers)
        writer.writeheader()

        for point_name, positions in teaching_data["points"].items():
            if positions is None:
                continue

            row = {
                "type": "point",
                "name": point_name,
                "shoulder_pan": positions["shoulder_pan"],
                "shoulder_lift": positions["shoulder_lift"],
                "elbow_flex": positions["elbow_flex"],
                "wrist_flex": positions["wrist_flex"],
                "wrist_roll": positions["wrist_roll"],
                "gripper": "",
            }

            writer.writerow(row)

        if teaching_data["gripper"]["grip"] is not None:
            writer.writerow({
                "type": "gripper",
                "name": "grip",
                "shoulder_pan": "",
                "shoulder_lift": "",
                "elbow_flex": "",
                "wrist_flex": "",
                "wrist_roll": "",
                "gripper": teaching_data["gripper"]["grip"],
            })

        if teaching_data["gripper"]["ungrip"] is not None:
            writer.writerow({
                "type": "gripper",
                "name": "ungrip",
                "shoulder_pan": "",
                "shoulder_lift": "",
                "elbow_flex": "",
                "wrist_flex": "",
                "wrist_roll": "",
                "gripper": teaching_data["gripper"]["ungrip"],
            })

    print(f"Saved: {filename}")


# =========================
# 6. 메인 프로그램
# =========================
teaching_data = {
    "points": {
        "1": None,
        "2": None,
        "3": None,
    },
    "gripper": {
        "grip": None,
        "ungrip": None,
    }
}


bus = FeetechMotorsBus(
    port="/dev/ttyACM0",
    motors=motors
)


try:
    bus.connect()

    # 손으로 티칭하기 위해 토크 OFF
    bus.disable_torque()

    print("====================================")
    print(" SO-ARM101 Teaching Program")
    print("====================================")
    print("1 : Teaching Point 1 저장")
    print("2 : Teaching Point 2 저장")
    print("3 : Teaching Point 3 저장")
    print("+ : Grip Position 저장")
    print("- : Ungrip Position 저장")
    print("q : 종료")
    print("====================================")

    while True:
        key = get_key()

        if key in ["1", "2", "3"]:
            positions = read_arm_positions(bus)
            teaching_data["points"][key] = positions

            print(f"\nTeaching Point {key} saved")
            print(positions)

        elif key == "+":
            grip_pos = read_grip_position(bus)
            teaching_data["gripper"]["grip"] = grip_pos

            print("\nGrip position saved")
            print("gripper:", grip_pos)

        elif key == "-":
            ungrip_pos = read_grip_position(bus)
            teaching_data["gripper"]["ungrip"] = ungrip_pos

            print("\nUngrip position saved")
            print("gripper:", ungrip_pos)

        elif key.lower() == "q":
            print("\nExit requested.")
            save = input("Save teaching data? (y/n): ")

            if save.lower() == "y":
                save_teaching_data_csv(teaching_data)
            else:
                print("Not saved.")

            break

        time.sleep(0.05)


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
