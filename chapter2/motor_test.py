from lerobot.motors.feetech.feetech import FeetechMotorsBus
from lerobot.motors.motors_bus import Motor

# Motor 객체로 정의
motors = {
	"m1": Motor(
		id=4,
		model="sts3215",
		norm_mode="position"
	)
}

# 버스 생성
bus = FeetechMotorsBus(
	port="/dev/ttyACM0",
	motors=motors
)

# 연결
bus.connect()
bus.enable_torque()

# 현재 위치 읽기 (raw)
pos = bus.read("Present_Position", "m1", normalize=False)
print("pos:", pos)

# 조금 이동
bus.write("Goal_Position", "m1", pos - 100, normalize=False)
print("pos:", pos - 100)

# 종료
import time
time.sleep(1)

bus.disable_torque()
bus.disconnect()
