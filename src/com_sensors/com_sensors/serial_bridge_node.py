"""
com_sensors 패키지의 serial_bridge_node

역할: 아두이노 메인 보드(DHT11 + 불꽃 + IR)가 시리얼로 보내는
"T:25.0,H:60.0,F:0,I:1" 문자열을 읽어서 ROS2 토픽으로 발행한다.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Bool
import serial


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('com_sensor_bridge_node')

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
        self.ir_pub = self.create_publisher(Bool, '/com_sensors/ir_detected', sensor_qos)

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
            ir_detected = bool(int(parts['I']))
        except (ValueError, KeyError):
            self.get_logger().warn(f'패킷 파싱 실패, 무시함: {line}')
            return

        self.temp_pub.publish(Float32(data=temperature))
        self.humidity_pub.publish(Float32(data=humidity))
        self.flame_pub.publish(Bool(data=flame))
        self.ir_pub.publish(Bool(data=ir_detected))


def main():
    rclpy.init()
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()