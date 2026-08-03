"""
sensor_bridge 노드 역할:
아두이노 메인 보드(DHT11 + 불꽃 + IR + 조도 + 배터리전압 + 긴급정지버튼)가
시리얼로 보내는 두 종류의 라인을 읽어서 ROS2 토픽으로 발행한다.
(arduino/flaem_pir_dht11_jul31a/flaem_pir_dht11_jul31a.ino 프로토콜과 반드시 일치해야 함)

1) 텔레메트리 라인 (약 2초 주기, DHT11 제약):
   "T:25.0,H:60.0,F:0,I:1,L:512,V:11.8"
2) 이벤트 라인 (버튼 입력 시 즉시):
   "EVT:B:1"  (짧게 누름)  또는  "EVT:B:2"  (길게 누름 -> 긴급정지)

com_service/service_manager_node(SafetyMonitor)가 실제로 구독하는 계약:
  - /rear_ir        (sensor_msgs/Range)  : handle_rear_ir, obstacle_stop_distance와 비교
  - /button_event   (std_msgs/String)    : handle_button_event ('short'/'long')
  - /battery_voltage(std_msgs/Float32)   : check_battery

나머지(/com_sensors/*)는 로깅/상태 조회용 부가 토픽.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Bool, String
from sensor_msgs.msg import Range
import serial


# IR은 디지털(감지/미감지) 센서라 실제 거리값이 없다.
# service_manager의 obstacle_stop_distance(기본 0.20m) 파라미터와 비교되므로,
# 감지 시 0.0m(정지 트리거), 미감지 시 그보다 확실히 먼 값을 보낸다.
IR_DETECTED_RANGE_M = 0.0
IR_CLEAR_RANGE_M = 1.0
IR_FIELD_OF_VIEW_RAD = 0.5

BUTTON_EVENT_MAP = {'1': 'short', '2': 'long'}


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

        # ---- service_manager(SafetyMonitor)가 실제로 구독하는 토픽 (SR-021~SR-026) ----
        self.rear_ir_pub = self.create_publisher(Range, '/rear_ir', sensor_qos)
        self.button_event_pub = self.create_publisher(String, '/button_event', 10)
        self.battery_voltage_pub = self.create_publisher(Float32, '/battery_voltage', sensor_qos)

        # ---- 로깅/상태 조회용 부가 토픽 ----
        self.temp_pub = self.create_publisher(Float32, '/com_sensors/temperature', sensor_qos)
        self.humidity_pub = self.create_publisher(Float32, '/com_sensors/humidity', sensor_qos)
        self.flame_pub = self.create_publisher(Bool, '/com_sensors/flame_detected', sensor_qos)
        self.ir_pub = self.create_publisher(Bool, '/com_sensors/ir_detected', sensor_qos)
        self.illuminance_pub = self.create_publisher(Float32, '/com_sensors/illuminance', sensor_qos)

        # 20Hz로 시리얼 버퍼 체크 (텔레메트리는 2초 주기지만, 버튼 이벤트는 즉시 와야 하므로 자주 확인)
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

        if line.startswith('EVT:'):
            self.parse_and_publish_event(line)
        else:
            self.parse_and_publish_telemetry(line)

    def parse_and_publish_event(self, line: str):
        # 형식: "EVT:B:1" / "EVT:B:2"
        try:
            _, key, code = line.split(':')
        except ValueError:
            self.get_logger().warn(f'이벤트 파싱 실패, 무시함: {line}')
            return

        if key != 'B':
            return

        event = BUTTON_EVENT_MAP.get(code)
        if event is None:
            self.get_logger().warn(f'알 수 없는 버튼 이벤트 코드, 무시함: {line}')
            return

        self.button_event_pub.publish(String(data=event))
        if event == 'long':
            self.get_logger().warn('긴급정지 버튼(길게 누름) 이벤트 수신 -> /button_event 발행')

    def parse_and_publish_telemetry(self, line: str):
        # 예: "T:25.0,H:60.0,F:0,I:1,L:512,V:11.8"
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

        ir_range = Range()
        ir_range.header.stamp = self.get_clock().now().to_msg()
        ir_range.header.frame_id = 'rear_ir_link'
        ir_range.radiation_type = Range.INFRARED
        ir_range.field_of_view = IR_FIELD_OF_VIEW_RAD
        ir_range.min_range = IR_DETECTED_RANGE_M
        ir_range.max_range = IR_CLEAR_RANGE_M
        ir_range.range = IR_DETECTED_RANGE_M if ir_detected else IR_CLEAR_RANGE_M
        self.rear_ir_pub.publish(ir_range)

        # L(조도), V(배터리전압)는 구형 프로토콜(필드 없음)과의 호환을 위해 선택 처리
        if 'L' in parts:
            try:
                self.illuminance_pub.publish(Float32(data=float(parts['L'])))
            except ValueError:
                self.get_logger().warn(f'조도값 파싱 실패, 무시함: {line}')

        if 'V' in parts:
            try:
                self.battery_voltage_pub.publish(Float32(data=float(parts['V'])))
            except ValueError:
                self.get_logger().warn(f'배터리 전압 파싱 실패, 무시함: {line}')

        self.get_logger().debug(
            f'발행: T={temperature} H={humidity} F={flame} I={ir_detected} '
            f"L={parts.get('L')} V={parts.get('V')}"
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
