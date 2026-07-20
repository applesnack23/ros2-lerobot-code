from lerobot.motors.feetech.feetech import FeetechMotorsBus
from lerobot.motors.motors_bus import Motor

import time
import os
import sys
import glob
import csv
import select

MOTOR_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

motors = {
    "shoulder_pan": Motor(id=1, model="sts3215", norm_mode="position"),
    "shoulder_lift": Motor(id=2, model="sts3215", norm_mode="position"),
    "elbow_flex": Motor(id=3, model="sts3215", norm_mode="position"),
    "wrist_flex": Motor(id=4, model="sts3215", norm_mode="position"),
    "wrist_roll": Motor(id=5, model="sts3215", norm_mode="position"),
    "gripper": Motor(id=6, model="sts3215", norm_mode="position"),
}


def select_port(title):
    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))

    if not ports:
        print("No serial ports found.")
        sys.exit(1)

    print(f"\nSelect {title} port")
    print("-" * 40)

    for i, port in enumerate(ports, start=1):
        print(f"{i}. {port}")

    print("-" * 40)

    while True:
        choice = input("Select port number: ").strip()

        if choice.isdigit():
            index = int(choice) - 1
            if 0 <= index < len(ports):
                return ports[index]

        print("Invalid selection. Try again.")


def load_min_max(filename):
    data = {}

    with open(filename, "r") as f:
        reader = csv.DictReader(f)

        for row in reader:
            motor = row["motor"]
            data[motor] = {
                "min": int(row["min"]),
                "max": int(row["max"]),
            }

    return data


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def map_position(leader_pos, leader_min, leader_max, follower_min, follower_max):
    if leader_max == leader_min:
        return follower_min

    ratio = (leader_pos - leader_min) / (leader_max - leader_min)
    ratio = clamp(ratio, 0.0, 1.0)

    follower_pos = follower_min + ratio * (follower_max - follower_min)
    return int(follower_pos)


def enter_pressed():
    return select.select([sys.stdin], [], [], 0)[0]


leader_range = load_min_max("leader_arm_min_max.csv")
follower_range = load_min_max("follower_arm_min_max.csv")

leader_port = select_port("LEADER ARM")
follower_port = select_port("FOLLOWER ARM")

if leader_port == follower_port:
    print("Leader and follower ports cannot be the same.")
    sys.exit(1)


leader_bus = FeetechMotorsBus(
    port=leader_port,
    motors=motors
)

follower_bus = FeetechMotorsBus(
    port=follower_port,
    motors=motors
)


try:
    leader_bus.connect()
    follower_bus.connect()

    # leader arm은 손으로 움직여야 하므로 torque OFF
    leader_bus.disable_torque()

    # follower arm은 명령을 받아 움직여야 하므로 torque ON
    follower_bus.enable_torque()

    print("\nLeader-Follower sync started.")
    print("Move the leader arm by hand.")
    print("Press Enter to stop.")
    time.sleep(1)

    while True:
        if enter_pressed():
            sys.stdin.readline()
            break

        os.system("clear")

        print("SO-ARM101 Leader → Follower Sync")
        print(f"Leader Port  : {leader_port}")
        print(f"Follower Port: {follower_port}")
        print("-" * 75)
        print(f"{'Motor':<8} {'Leader':>10} {'Target':>10} {'L-Min':>10} {'L-Max':>10} {'F-Min':>10} {'F-Max':>10}")
        print("-" * 75)

        for name in MOTOR_NAMES:
            try:
                leader_pos = leader_bus.read(
                    "Present_Position",
                    name,
                    normalize=False
                )

                if hasattr(leader_pos, "item"):
                    leader_pos = leader_pos.item()

                leader_pos = int(leader_pos)

                l_min = leader_range[name]["min"]
                l_max = leader_range[name]["max"]
                f_min = follower_range[name]["min"]
                f_max = follower_range[name]["max"]

                target_pos = map_position(
                    leader_pos,
                    l_min,
                    l_max,
                    f_min,
                    f_max
                )

                follower_bus.write(
                    "Goal_Position",
                    name,
                    target_pos,
                    normalize=False
                )

                print(
                    f"{name:<8} "
                    f"{leader_pos:>10} "
                    f"{target_pos:>10} "
                    f"{l_min:>10} "
                    f"{l_max:>10} "
                    f"{f_min:>10} "
                    f"{f_max:>10}"
                )

            except Exception as e:
                print(f"{name:<8} ERROR: {e}")

        print("-" * 75)
        print("Press Enter to stop.")

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nStopped by Ctrl + C.")

finally:
    try:
        follower_bus.disable_torque()
    except:
        pass

    try:
        leader_bus.disconnect()
    except:
        pass

    try:
        follower_bus.disconnect()
    except:
        pass

    print("Disconnected.")
