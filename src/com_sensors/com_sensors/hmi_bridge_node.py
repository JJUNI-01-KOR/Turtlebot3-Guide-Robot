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
    "B:<0-100>"                 배터리 퍼센트 (service_manager의 /battery_percent 구독)
    "W:<0-4>"                   WiFi 막대 (라즈베리파이 /proc/net/wireless 직접 읽음)

state_code 값은 service_manager_node.ServiceManagerNode.STATE_CODE_MAP 기준.
그 외 상태(READY 등)는 보드가 ARRIVED 진입 3초 후 자체적으로 메뉴 화면으로
복귀하므로 별도 명령을 보내지 않는다.

destination_id 유효성 검사는 service_manager_node(/start_mission)가 waypoints.yaml
기준으로 수행하므로 이 노드는 별도로 검증하지 않는다.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Float32
import serial

from com_interfaces.srv import StartMission

MOVING_STATE_CODE = 0x03
ARRIVED_STATE_CODE = 0x04

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

        self.start_mission_client = self.create_client(StartMission, '/start_mission')
        self.state_code_sub = self.create_subscription(
            UInt8, '/state_code', self.on_state_code, 10
        )
        self.battery_percent_sub = self.create_subscription(
            Float32, '/battery_percent', self.on_battery_percent, 10
        )

        self.timer = self.create_timer(0.02, self.read_serial_callback)
        self.wifi_timer = self.create_timer(5.0, self.on_wifi_timer)

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
