"""
com_sensors 패키지의 hmi_bridge_node

역할: 목적지 메뉴 + 이동중 눈 + 도착화면을 한 화면에서 보여주는
통합 TFT+조이스틱 보드(eyes_board_jul31a.ino, ARS36125D/MCUFRIEND_kbv)와
라즈베리파이 사이의 시리얼 브릿지.

시리얼 프로토콜 (eyes_board_jul31a.ino 와 반드시 일치해야 함, 115200 baud, 개행 종료):
  [보드 -> Pi]
    "SELECT:<destination_id>"   목적지 선택 확정 -> /start_mission 호출
    "READY"                     부팅 완료 (로그만 남김)
  [Pi -> 보드]
    "S:MOVING"                  이동 중 화면 전환 (state_code == 0x03: GUIDING/RETURNING/DOCKING)
    "S:ARRIVED"                 도착 화면 전환 (state_code == 0x04: ARRIVED/PERSON_WAIT)
    "S:SELECT"                  메뉴 화면 강제 복귀 (state_code == 0x0A: ERROR/EMERGENCY_STOP)
    "B:<0-100>"                 배터리 퍼센트 (service_manager의 /battery_percent 구독)
    "W:<0-4>"                   WiFi 막대 (라즈베리파이 /proc/net/wireless 직접 읽음)

state_code 값은 service_manager_node.ServiceManagerNode.STATE_CODE_MAP 기준.
READY 등 나머지 상태는 보드가 ARRIVED 진입 3초 후 자체적으로 메뉴 화면으로
복귀하므로 별도 명령을 보내지 않는다 (ERROR/EMERGENCY_STOP만 예외 - S:SELECT로
즉시 복귀시킴, 안 그러면 화면이 MOVING에 계속 멈춰있었음).

destination_id 유효성 검사는 service_manager_node(/start_mission)가 waypoints.yaml
기준으로 수행하므로 이 노드는 별도로 검증하지 않는다.

2026-08-04: 조이스틱이 이 보드가 아니라 메인센서보드(com_sensor_bridge_node가
/com_sensors/joystick_x, /com_sensors/joystick_button으로 발행)에 물려있어서,
예전에 eyes_board_jul31a.ino가 로컬 핀에서 직접 하던 디바운스/임계값 판정을
여기서 대신 하고 "MOVE:UP" / "MOVE:DOWN" / "CONFIRM"만 보드로 중계한다
(임계값·디바운스 시간은 원래 .ino 로직과 동일하게 맞춤: 이동 250ms, 확정 300ms).
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt8, Float32, Int32, Bool
import serial

from com_interfaces.srv import StartMission

MOVING_STATE_CODE = 0x03
ARRIVED_STATE_CODE = 0x04
# ERROR/EMERGENCY_STOP 둘 다 STATE_CODE_MAP에서 0x0A로 합쳐져 있음
# (service_manager_node.STATE_CODE_MAP 참고)
ERROR_STATE_CODE = 0x0A

JOY_X_LOW_THRESHOLD = 300
JOY_X_HIGH_THRESHOLD = 700
JOY_MOVE_DEBOUNCE_S = 0.25
JOY_CONFIRM_DEBOUNCE_S = 0.3

# /proc/net/wireless의 link quality는 드라이버별로 상한이 다르나
# 라즈베리파이 내장 wifi(brcmfmac 등)는 보통 0~70 범위를 사용한다.
WIFI_LINK_QUALITY_MAX = 70


class HmiBridgeNode(Node):
    def __init__(self):
        super().__init__('com_hmi_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value

        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f'HMI 보드 시리얼 연결 성공: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'HMI 보드 시리얼 연결 실패: {e}')
            raise

        self.last_state_code = None
        self.last_sent_battery = None
        self.last_sent_wifi = None

        self.latest_joy_x = 512  # 중앙값 기본값 (아직 값 안 받았을 때 이동 안 하도록)
        self.last_joy_button = False
        self.last_move_time = 0.0
        self.last_confirm_time = 0.0

        self.start_mission_client = self.create_client(StartMission, '/start_mission')
        self.state_code_sub = self.create_subscription(
            UInt8, '/state_code', self.on_state_code, 10
        )
        self.battery_percent_sub = self.create_subscription(
            Float32, '/battery_percent', self.on_battery_percent, 10
        )
        # com_sensor_bridge_node가 이 두 토픽을 BEST_EFFORT로 발행하므로 QoS를 맞춰야
        # 메시지가 실제로 전달된다 (기본 RELIABLE로 구독하면 QoS 불일치로 조용히 안 받음).
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.joystick_x_sub = self.create_subscription(
            Int32, '/com_sensors/joystick_x', self.on_joystick_x, sensor_qos
        )
        self.joystick_button_sub = self.create_subscription(
            Bool, '/com_sensors/joystick_button', self.on_joystick_button, sensor_qos
        )

        self.timer = self.create_timer(0.02, self.read_serial_callback)
        self.wifi_timer = self.create_timer(5.0, self.on_wifi_timer)
        self.joystick_move_timer = self.create_timer(0.05, self.on_joystick_move_timer)

    # ---------- 보드 -> Pi ----------
    def read_serial_callback(self):
        if self.ser.in_waiting == 0:
            return

        try:
            line = self.ser.readline().decode('utf-8').strip()
        except UnicodeDecodeError:
            self.get_logger().warn('HMI 보드 시리얼 데이터 디코딩 실패, 무시함')
            return

        if not line:
            return

        if line == 'READY':
            self.get_logger().info('HMI 보드 부팅 완료')
            return

        if line.startswith('SELECT:'):
            destination_id = line.split(':', 1)[1]
            self.call_start_mission(destination_id)

    def call_start_mission(self, destination_id: str):
        if not self.start_mission_client.service_is_ready():
            self.get_logger().warn('/start_mission 서비스 미준비 상태, 선택 무시')
            return

        request = StartMission.Request()
        request.destination_id = destination_id
        future = self.start_mission_client.call_async(request)
        future.add_done_callback(
            lambda f: self.on_start_mission_response(f, destination_id)
        )

    def on_start_mission_response(self, future, destination_id):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'/start_mission 호출 실패: {e}')
            return

        if response.success:
            self.get_logger().info(f'임무 시작: {destination_id}')
        else:
            self.get_logger().warn(f'임무 시작 거부: {response.message}')

    # ---------- 조이스틱 (메인센서보드에서 옴, com_sensor_bridge_node 경유) ----------
    def on_joystick_x(self, msg: Int32):
        self.latest_joy_x = msg.data

    def on_joystick_button(self, msg: Bool):
        pressed = msg.data
        if pressed and not self.last_joy_button:
            now = time.monotonic()
            if now - self.last_confirm_time > JOY_CONFIRM_DEBOUNCE_S:
                self.last_confirm_time = now
                self.ser.write(b'CONFIRM\n')
        self.last_joy_button = pressed

    def on_joystick_move_timer(self):
        now = time.monotonic()
        if now - self.last_move_time < JOY_MOVE_DEBOUNCE_S:
            return
        # 2026-08-04: 화면을 180도 반전(setRotation 1->3)하면서 조이스틱 방향도
        # 반대로 맞춤 - x가 작을 때 DOWN, 클 때 UP.
        x = self.latest_joy_x
        if x < JOY_X_LOW_THRESHOLD:
            self.ser.write(b'MOVE:DOWN\n')
            self.last_move_time = now
        elif x > JOY_X_HIGH_THRESHOLD:
            self.ser.write(b'MOVE:UP\n')
            self.last_move_time = now

    # ---------- Pi -> 보드 ----------
    def on_state_code(self, msg: UInt8):
        if msg.data == self.last_state_code:
            return
        self.last_state_code = msg.data

        if msg.data == MOVING_STATE_CODE:
            self.ser.write(b'S:MOVING\n')
            self.get_logger().info('state_code=0x03 -> 보드에 S:MOVING 전송')
        elif msg.data == ARRIVED_STATE_CODE:
            self.ser.write(b'S:ARRIVED\n')
            self.get_logger().info('state_code=0x04 -> 보드에 S:ARRIVED 전송')
        elif msg.data == ERROR_STATE_CODE:
            # 예전엔 이 분기가 없어서 ERROR 진입 시 보드 화면이 MOVING(눈)에 멈춰있었음
            self.ser.write(b'S:SELECT\n')
            self.get_logger().info('state_code=0x0A(ERROR) -> 보드에 S:SELECT 전송, 메뉴로 복귀')

    def on_battery_percent(self, msg: Float32):
        percent = max(0, min(100, round(msg.data)))
        if percent == self.last_sent_battery:
            return
        self.last_sent_battery = percent
        self.ser.write(f'B:{percent}\n'.encode('utf-8'))
        self.get_logger().info(f'battery_percent={msg.data:.1f} -> 보드에 B:{percent} 전송')

    def on_wifi_timer(self):
        bars = self.read_wifi_bars()
        if bars is None or bars == self.last_sent_wifi:
            return
        self.last_sent_wifi = bars
        self.ser.write(f'W:{bars}\n'.encode('utf-8'))
        self.get_logger().info(f'wifi link quality -> 보드에 W:{bars} 전송')

    def read_wifi_bars(self):
        """/proc/net/wireless 에서 첫 무선 인터페이스의 link quality를 읽어 0~4 막대로 변환한다."""
        try:
            with open('/proc/net/wireless', 'r') as f:
                lines = f.readlines()
        except OSError:
            return None

        if len(lines) < 3:
            return None

        fields = lines[2].split()
        if len(fields) < 3:
            return None

        try:
            quality = float(fields[2].rstrip('.'))
        except ValueError:
            return None

        bars = round(quality / WIFI_LINK_QUALITY_MAX * 4)
        return max(0, min(4, bars))


def main():
    rclpy.init()
    node = HmiBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
