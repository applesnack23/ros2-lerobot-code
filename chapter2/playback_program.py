import csv
import time
import termios
import tty
import sys

from lerobot.motors.feetech.feetech import FeetechMotorsBus
from lerobot.motors.motors_bus import Motor


# =========================
# 1. 설정
# =========================
CSV_FILE = "so_arm101_teaching.csv"

MOVE_DELAY = 0.5

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
# 3. CSV 읽기
# =========================
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


# =========================
# 4. 모터 이동 함수
# =========================
def move_arm_to_point(bus, point_data):
    for motor_name in arm_motor_names:
        bus.write(
            "Goal_Position",
            motor_name,
            point_data[motor_name],
            normalize=False
        )

    print("Arm moved:")
    print(point_data)


def move_gripper(bus, position):
    bus.write(
        "Goal_Position",
        grip_motor_name,
        position,
        normalize=False
    )

    print("Gripper moved:", position)


# =========================
# 5. 메인 프로그램
# =========================
teaching_data = load_teaching_data_csv(CSV_FILE)

bus = FeetechMotorsBus(
    port="/dev/ttyACM0",
    motors=motors
)


try:
    bus.connect()

    # 실행 프로그램은 토크 ON
    bus.enable_torque()

    print("====================================")
    print(" SO-ARM101 Playback Program")
    print("====================================")
    print(f"Loaded CSV: {CSV_FILE}")
    print("------------------------------------")
    print("1 : Move to Teaching Point 1")
    print("2 : Move to Teaching Point 2")
    print("3 : Move to Teaching Point 3")
    print("+ : Grip")
    print("- : Ungrip")
    print("q : Quit")
    print("====================================")

    while True:
        key = get_key()

        if key in ["1", "2", "3"]:
            if key not in teaching_data["points"]:
                print(f"\nTeaching Point {key} is not saved.")
                continue

            move_arm_to_point(bus, teaching_data["points"][key])
            time.sleep(MOVE_DELAY)

        elif key == "+":
            grip_pos = teaching_data["gripper"]["grip"]

            if grip_pos is None:
                print("\nGrip position is not saved.")
                continue

            move_gripper(bus, grip_pos)
            time.sleep(MOVE_DELAY)

        elif key == "-":
            ungrip_pos = teaching_data["gripper"]["ungrip"]

            if ungrip_pos is None:
                print("\nUngrip position is not saved.")
                continue

            move_gripper(bus, ungrip_pos)
            time.sleep(MOVE_DELAY)

        elif key.lower() == "q":
            print("\nQuit requested.")
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
