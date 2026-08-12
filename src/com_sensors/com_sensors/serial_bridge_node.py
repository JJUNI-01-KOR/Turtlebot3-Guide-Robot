"""
sensor_bridge 노드 역할:
아두이노 메인 보드(DHT11 + 불꽃 + 후방 초음파)가 시리얼로 보내는
"T:25.0,H:60.0,F:0,R:15.3,O:1" 형태 문자열을 읽어서
ROS2 토픽으로 나눠서 발행한다.

IR/PIR 센서는 프로젝트 범위에서 제외하기로 확정됨(2026-08-04) - 후방 장애물 감지는
초음파(R/O)로 완전히 대체. arduino/flaem_pir_dht11_jul31a.ino 소스(폴더/파일명은 예전
이름 그대로 남아있음)는 이 결정 이전 버전(IR+버튼+조도+배터리전압)이라 실제 펌웨어와
다름 - IR/PIR 관련 코드를 되살릴 필요는 없고, .ino를 이 프로토콜 기준으로 정리하면 됨.

긴급정지버튼: 물리버튼은 구현 안 하고 GUI 버튼으로 대체 예정(com_gui, 아직 미구현).
배터리전압: SafetyMonitor는 이 노드가 아니라 OpenCR의 /battery_state를 구독함
(service_manager_node.py 참고, 센서보드 배터리분배회로는 제거됨).

2026-08-04: 조이스틱(HMI 메뉴 선택용)이 TFT보드가 아니라 이 메인센서보드에 물려있어
"J:<analogRead 0-1023>,B:<0/1>" 라인(약 50ms 주기)도 이 보드가 보낸다. 디바운스/
임계값 판정은 이 노드가 아니라 hmi_bridge_node.py가 함(raw 값만 중계).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Bool, Int32
import serial


class SensorBridgeNode(Node):
    def __init__(self):
        super().__init__('sensor_bridge_node')

        # 2026-08-11: udev 규칙(99-com-robot-serial.rules)으로 시리얼 번호 기반 고정
        # 심볼릭 링크를 만들어서 ttyACM 번호가 재부팅/재연결 때마다 바뀌는 문제를
        # 회피함 - 이제 기본값 자체가 항상 메인센서보드를 가리킴.
        self.declare_parameter('serial_port', '/dev/com_sensorboard')
        self.declare_parameter('baud_rate', 9600)

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # 2026-08-12: 최초 연결 시도가 USB 불안정(전원/허브 등)으로 실패하면 바로
        # 예외를 던져서 프로세스가 죽었고, PM2가 노드 초기화부터 통째로 재시작해야
        # 했다(재시작 카운트가 수십 회까지 누적되는 것 실측). read_serial_callback/
        # _write와 동일한 방어 철학을 최초 연결에도 적용 - 몇 초 기다리면 USB가
        # 다시 붙는 경우가 많으므로, 바로 죽지 않고 같은 프로세스 안에서 재시도한다.
        self.ser = self._open_serial_with_retry(port, baud, max_retries=5, retry_delay_sec=2.0)
        self.get_logger().info(f'시리얼 연결 성공: {port} @ {baud}')

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.temp_pub = self.create_publisher(Float32, '/com_sensors/temperature', sensor_qos)
        self.humidity_pub = self.create_publisher(Float32, '/com_sensors/humidity', sensor_qos)
        self.flame_pub = self.create_publisher(Bool, '/com_sensors/flame_detected', sensor_qos)
        self.rear_distance_pub = self.create_publisher(Float32, '/com_sensors/rear_distance', sensor_qos)
        self.rear_obstacle_pub = self.create_publisher(Bool, '/com_sensors/rear_obstacle', sensor_qos)
        self.joystick_x_pub = self.create_publisher(Int32, '/com_sensors/joystick_x', sensor_qos)
        self.joystick_button_pub = self.create_publisher(Bool, '/com_sensors/joystick_button', sensor_qos)

        self.timer = self.create_timer(0.05, self.read_serial_callback)

    def _open_serial_with_retry(self, port, baud, max_retries=5, retry_delay_sec=2.0):
        """USB 포트가 순간적으로 흔들려서 첫 시도에 열기 실패해도 바로 죽지 않고
        잠깐 기다렸다가 재시도한다 (2026-08-12: 'Inappropriate ioctl for device'
        등 USB 전원 불안정으로 인한 연결 실패 실측 대응).
        """
        import time
        last_exc = None
        for attempt in range(1, max_retries + 1):
            try:
                # 2026-08-12: timeout=1.0이면 read_serial_callback(20~50ms 주기) 안에서
                # readline()이 최대 1초까지 블로킹할 수 있어 다른 콜백이 밀릴 수 있다.
                # 짧게 잡아서 타이머 주기와 어울리게 함.
                return serial.Serial(port, baud, timeout=0.05)
            except serial.SerialException as e:
                last_exc = e
                self.get_logger().warn(
                    f'시리얼 연결 실패 (시도 {attempt}/{max_retries}): {e} '
                    f'- {retry_delay_sec}초 후 재시도'
                )
                time.sleep(retry_delay_sec)
        self.get_logger().error(f'{max_retries}회 재시도 모두 실패, 노드를 종료합니다: {last_exc}')
        raise last_exc

    def read_serial_callback(self):
        # 2026-08-11 실측: USB 연결이 간헐적으로 끊기면 pyserial이 OSError(Input/output
        # error)를 던지는데, 예외 처리가 없어서 이 콜백(20ms 주기) 안에서 그대로
        # rclpy executor까지 전파되어 노드 전체가 죽었다(hmi_bridge_node와 동일한
        # 버그, 실물 세션 중 이 노드도 같은 방식으로 크래시하는 것 실측함). 안 죽고
        # 다음 주기에 다시 시도하도록 방어(hmi_bridge_node.py의 처리와 동일한 패턴).
        # 2026-08-12: in_waiting(ioctl FIONREAD)이 이 보드(CDC-ACM)에서 간헐적으로
        # 'Inappropriate ioctl'/'Input/output error'를 던지는 것 실측 - 실제 read는
        # 문제없이 되는데(sudo cat으로 확인) in_waiting 체크만 불안정했다. in_waiting
        # 없이 readline()의 자체 timeout(위에서 0.05초로 설정)에 맡기도록 변경.
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                return
        except UnicodeDecodeError:
            self.get_logger().warn('시리얼 데이터 디코딩 실패, 무시함')
            return
        except OSError as e:
            self.get_logger().error(
                f'센서보드 시리얼 읽기 실패: {e} (USB 연결 확인 필요, 다음 주기에 재시도)',
                throttle_duration_sec=5.0
            )
            return

        if not line:
            return

        if line.startswith('J:'):
            self.parse_and_publish_joystick(line)
        else:
            self.parse_and_publish(line)

    def parse_and_publish_joystick(self, line: str):
        try:
            parts = dict(item.split(':') for item in line.split(','))
            x = int(parts['J'])
            button = bool(int(parts['B']))
        except (ValueError, KeyError):
            self.get_logger().warn(f'조이스틱 패킷 파싱 실패, 무시함: {line}')
            return

        self.joystick_x_pub.publish(Int32(data=x))
        self.joystick_button_pub.publish(Bool(data=button))

    def parse_and_publish(self, line: str):
        try:
            parts = dict(item.split(':') for item in line.split(','))
            temperature = float(parts['T'])
            humidity = float(parts['H'])
            flame = bool(int(parts['F']))
            rear_distance = float(parts['R'])
            rear_obstacle = bool(int(parts['O']))
        except (ValueError, KeyError):
            self.get_logger().warn(f'패킷 파싱 실패, 무시함: {line}')
            return

        self.temp_pub.publish(Float32(data=temperature))
        self.humidity_pub.publish(Float32(data=humidity))
        self.flame_pub.publish(Bool(data=flame))
        self.rear_distance_pub.publish(Float32(data=rear_distance))
        self.rear_obstacle_pub.publish(Bool(data=rear_obstacle))

        self.get_logger().debug(
            f'발행: T={temperature} H={humidity} F={flame} R={rear_distance} O={rear_obstacle}'
        )


def main():
    rclpy.init()
    node = SensorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()