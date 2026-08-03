"""
sensor_bridge 노드 역할:
아두이노 메인 보드(DHT11 + 불꽃 + IR + PIR + 조이스틱)가 시리얼로 보내는
"T:25.0,H:60.0,F:0,I:1,P:0" 형태 문자열을 읽어서
ROS2 토픽 5개로 나눠서 발행한다.

[역할 분리 이유]
이 노드는 "센서값 받아서 토픽으로 뿌리는 것"만 함.
나중에 이 값을 보고 뭔가 판단(예: 불꽃 감지되면 경고)하는 로직은
따로 판단 노드를 만들어서 분리하는 게 좋음.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Bool
import serial


class SensorBridgeNode(Node):
    def __init__(self):
        super().__init__('sensor_bridge_node')

        # ---- 파라미터로 포트/보드레이트 관리 (하드코딩 대신) ----
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

        # 센서 스트림이라 best_effort QoS 사용 (가끔 패킷 놓쳐도 괜찮음)
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ---- 토픽 발행자 5개 ----
        self.temp_pub = self.create_publisher(Float32, '/com_sensors/temperature', sensor_qos)
        self.humidity_pub = self.create_publisher(Float32, '/com_sensors/humidity', sensor_qos)
        self.flame_pub = self.create_publisher(Bool, '/com_sensors/flame_detected', sensor_qos)
        self.ir_pub = self.create_publisher(Bool, '/com_sensors/ir_detected', sensor_qos)
        self.pir_pub = self.create_publisher(Bool, '/com_sensors/pir_detected', sensor_qos)

        # 20Hz로 시리얼 버퍼 체크 (아두이노는 2초마다 보내지만, 자주 확인해서 놓치지 않게)
        self.timer = self.create_timer(0.05, self.read_serial_callback)

    def read_serial_callback(self):
        if self.ser.in_waiting == 0:
            return  # 새 데이터 없으면 그냥 넘어감

        try:
            line = self.ser.readline().decode('utf-8').strip()
        except UnicodeDecodeError:
            self.get_logger().warn('시리얼 데이터 디코딩 실패, 무시함')
            return

        if not line:
            return

        self.parse_and_publish(line)

    def parse_and_publish(self, line: str):
        # 예: "T:25.0,H:60.0,F:0,I:1,P:0"
        try:
            parts = dict(item.split(':') for item in line.split(','))
            temperature = float(parts['T'])
            humidity = float(parts['H'])
            flame = bool(int(parts['F']))
            ir_detected = bool(int(parts['I']))
            pir_detected = bool(int(parts['P']))
        except (ValueError, KeyError):
            self.get_logger().warn(f'패킷 파싱 실패, 무시함: {line}')
            return

        self.temp_pub.publish(Float32(data=temperature))
        self.humidity_pub.publish(Float32(data=humidity))
        self.flame_pub.publish(Bool(data=flame))
        self.ir_pub.publish(Bool(data=ir_detected))
        self.pir_pub.publish(Bool(data=pir_detected))

        self.get_logger().debug(
            f'발행: T={temperature} H={humidity} F={flame} I={ir_detected} P={pir_detected}'
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