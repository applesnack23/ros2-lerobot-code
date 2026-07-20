from lerobot.motors.feetech.feetech import FeetechMotorsBus
from lerobot.motors.motors_bus import Motor
import time
import os
import sys
import select
import csv
import glob

motors = {
    "shoulder_pan": Motor(id=1, model="sts3215", norm_mode="position"),
    "shoulder_lift": Motor(id=2, model="sts3215", norm_mode="position"),
    "elbow_flex": Motor(id=3, model="sts3215", norm_mode="position"),
    "wrist_flex": Motor(id=4, model="sts3215", norm_mode="position"),
    "wrist_roll": Motor(id=5, model="sts3215", norm_mode="position"),
    "gripper": Motor(id=6, model="sts3215", norm_mode="position"),
}

min_pos = {name: None for name in motors}
max_pos = {name: None for name in motors}


def select_port():
    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))

    if not ports:
        print("No serial ports found.")
        print("Check USB connection.")
        sys.exit(1)

    print("Available serial ports:")
    print("-" * 30)

    for i, port in enumerate(ports, start=1):
        print(f"{i}. {port}")

    print("-" * 30)

    while True:
        choice = input("Select port number: ").strip()

        if choice.isdigit():
            index = int(choice) - 1

            if 0 <= index < len(ports):
                return ports[index]

        print("Invalid selection. Try again.")


def enter_pressed():
    return select.select([sys.stdin], [], [], 0)[0]


def select_arm_type():
    print("\nSelect arm type:")
    print("1. leader arm")
    print("2. follower arm")

    while True:
        choice = input("Select 1 or 2: ").strip()

        if choice == "1":
            return "leader_arm"

        if choice == "2":
            return "follower_arm"

        print("Invalid selection. Select 1 or 2.")


def save_min_max(arm_type):
    filename = f"{arm_type}_min_max.csv"

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["arm_type", "motor", "min", "max"])

        for name in motors:
            writer.writerow([
                arm_type,
                name,
                min_pos[name],
                max_pos[name]
            ])

    print(f"Saved: {filename}")


selected_port = select_port()

bus = FeetechMotorsBus(
    port=selected_port,
    motors=motors
)

try:
    bus.connect()

    print(f"\nConnected to {selected_port}")
    print("Motor position monitor started.")
    print("Press Enter to stop.")
    time.sleep(1)

    while True:
        if enter_pressed():
            sys.stdin.readline()
            break

        os.system("clear")

        print("SO-ARM101 / STS3215 Position Monitor")
        print(f"Port: {selected_port}")
        print("-" * 55)
        print(f"{'Motor':<8} {'Current':>10} {'Min':>10} {'Max':>10}")
        print("-" * 55)

        for name in motors:
            try:
                pos = bus.read(
                    "Present_Position",
                    name,
                    normalize=False
                )

                if hasattr(pos, "item"):
                    pos = pos.item()

                pos = int(pos)

                if min_pos[name] is None or pos < min_pos[name]:
                    min_pos[name] = pos

                if max_pos[name] is None or pos > max_pos[name]:
                    max_pos[name] = pos

                print(
                    f"{name:<8} "
                    f"{pos:>10} "
                    f"{min_pos[name]:>10} "
                    f"{max_pos[name]:>10}"
                )

            except Exception as e:
                print(f"{name:<8} READ ERROR: {e}")

        print("-" * 55)
        print("Press Enter to stop monitoring.")

        time.sleep(0.1)

    print("\nMonitoring stopped.")
    print('To save min/max values, type exactly "yes".')
    answer = input("Save min/max values? ").strip()

    if answer == "yes":
        arm_type = select_arm_type()
        save_min_max(arm_type)
    else:
        print("Not saved.")

except KeyboardInterrupt:
    print("\nStopped by Ctrl + C.")
    print('To save min/max values, type exactly "yes".')
    answer = input("Save min/max values? ").strip()

    if answer == "yes":
        arm_type = select_arm_type()
        save_min_max(arm_type)
    else:
        print("Not saved.")

finally:
    try:
        bus.disconnect()
    except:
        pass

    print("Bus disconnected.")
