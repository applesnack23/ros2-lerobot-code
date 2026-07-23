import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from lerobot.motors.feetech.feetech import FeetechMotorsBus
from lerobot.motors.motors_bus import Motor


class FeetechDriverNode(Node):
    def __init__(self):
        super().__init__("feetech_driver_node")

        self.port = "/dev/ttyACM0"
        self.read_period = 0.05  # 20Hz

        # 동작 명령 후 Torque를 유지할 시간
        self.torque_hold_time = 1.0
        self.last_goal_time = None
        self.torque_enabled = False

        self.motors = {
            "shoulder_pan": Motor(id=1, model="sts3215", norm_mode="position"),
            "shoulder_lift": Motor(id=2, model="sts3215", norm_mode="position"),
            "elbow_flex": Motor(id=3, model="sts3215", norm_mode="position"),
            "wrist_flex": Motor(id=4, model="sts3215", norm_mode="position"),
            "wrist_roll": Motor(id=5, model="sts3215", norm_mode="position"),
            "gripper": Motor(id=6, model="sts3215", norm_mode="position"),
        }

        self.joint_names = list(self.motors.keys())

        self.bus = FeetechMotorsBus(
            port=self.port,
            motors=self.motors
        )

        self.connect_motor_bus()

        self.joint_state_pub = self.create_publisher(
            JointState,
            "/joint_states",
            10
        )

        self.joint_goal_sub = self.create_subscription(
            JointState,
            "/so_arm101/joint_goal",
            self.joint_goal_callback,
            10
        )

        self.timer = self.create_timer(
            self.read_period,
            self.publish_joint_states
        )

        self.get_logger().info("SO-ARM101 Feetech Driver Node started.")
        self.get_logger().info("Default torque state: DISABLED")
        self.get_logger().info("Publish  : /joint_states")
        self.get_logger().info("Subscribe: /so_arm101/joint_goal")

    def connect_motor_bus(self):
        try:
            self.bus.connect()

            # 기본 상태는 Teaching을 위해 Torque OFF
            self.disable_torque()

            self.get_logger().info(f"Connected to Feetech bus: {self.port}")

        except Exception as e:
            self.get_logger().error(f"Failed to connect motor bus: {e}")
            raise e

    def enable_torque(self):
        if self.torque_enabled:
            return

        try:
            self.bus.enable_torque()
            self.torque_enabled = True
            self.get_logger().info("Torque enabled.")

        except Exception as e:
            self.get_logger().error(f"Failed to enable torque: {e}")

    def disable_torque(self):
        if not self.torque_enabled:
            try:
                self.bus.disable_torque()
            except Exception:
                pass
            return

        try:
            self.bus.disable_torque()
            self.torque_enabled = False
            self.get_logger().info("Torque disabled.")

        except Exception as e:
            self.get_logger().error(f"Failed to disable torque: {e}")

    def to_int(self, value):
        if isinstance(value, list):
            return int(value[0])

        try:
            return int(value)
        except TypeError:
            return int(value.item())

    def read_motor_position(self, motor_name):
        pos = self.bus.read(
            "Present_Position",
            motor_name,
            normalize=False
        )
        return self.to_int(pos)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []

        for motor_name in self.joint_names:
            try:
                position = self.read_motor_position(motor_name)

                msg.name.append(motor_name)
                msg.position.append(float(position))
                msg.velocity.append(0.0)
                msg.effort.append(0.0)

            except Exception as e:
                self.get_logger().warn(
                    f"Failed to read {motor_name}: {e}"
                )

        if len(msg.name) > 0:
            self.joint_state_pub.publish(msg)

        self.auto_disable_torque()

    def auto_disable_torque(self):
        if not self.torque_enabled:
            return

        if self.last_goal_time is None:
            return

        elapsed = time.time() - self.last_goal_time

        if elapsed >= self.torque_hold_time:
            self.disable_torque()

    def joint_goal_callback(self, msg):
        # 목표 위치 명령이 들어오면 Torque ON
        self.enable_torque()
        self.last_goal_time = time.time()

        for name, position in zip(msg.name, msg.position):
            if name not in self.joint_names:
                self.get_logger().warn(f"Unknown joint name: {name}")
                continue

            try:
                goal_position = int(position)

                self.bus.write(
                    "Goal_Position",
                    name,
                    goal_position,
                    normalize=False
                )

                self.get_logger().info(
                    f"Move {name} -> {goal_position}"
                )

            except Exception as e:
                self.get_logger().error(
                    f"Failed to move {name}: {e}"
                )

    def shutdown(self):
        self.get_logger().info("Shutting down Feetech driver node.")

        try:
            self.bus.disable_torque()
        except Exception:
            pass

        try:
            self.bus.disconnect()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)

    node = FeetechDriverNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()