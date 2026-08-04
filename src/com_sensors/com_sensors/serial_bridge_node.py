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
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Bool
import serial


class SensorBridgeNode(Node):
    def __init__(self):
        super().__init__('sensor_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value

        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f'시리얼 연결 성공: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'시리얼 연결 실패: {e}')
            raise

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.temp_pub = self.create_publisher(Float32, '/com_sensors/temperature', sensor_qos)
        self.humidity_pub = self.create_publisher(Float32, '/com_sensors/humidity', sensor_qos)
        self.flame_pub = self.create_publisher(Bool, '/com_sensors/flame_detected', sensor_qos)
        self.rear_distance_pub = self.create_publisher(Float32, '/com_sensors/rear_distance', sensor_qos)
        self.rear_obstacle_pub = self.create_publisher(Bool, '/com_sensors/rear_obstacle', sensor_qos)

        self.timer = self.create_timer(0.05, self.read_serial_callback)

    def read_serial_callback(self):
        if self.ser.in_waiting == 0:
            return

        try:
            line = self.ser.readline().decode('utf-8').strip()
        except UnicodeDecodeError:
            self.get_logger().warn('시리얼 데이터 디코딩 실패, 무시함')
            return

        if not line:
            return

        self.parse_and_publish(line)

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