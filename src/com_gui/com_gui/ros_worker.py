"""
ros_worker.py
-------------
ROS2(rclpy) 통신 전담 스레드.

왜 스레드를 나누나?
  PyQt는 자체 이벤트 루프가 있고, rclpy도 spin()이라는 자기 루프가 있어.
  둘을 같은 스레드에서 돌리면 서로 잡아먹어서 화면이 멈춰버려.
  그래서 rclpy는 QThread 안에서 돌리고, 화면에는 pyqtSignal로만 값을 넘긴다.

주의: 여기(ROS 스레드)에서 위젯을 직접 건드리면 절대 안 됨.
      반드시 signal.emit()으로 넘겨서 GUI 스레드가 그리게 해야 해.
"""

import os

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
)

from PyQt5.QtCore import QThread, pyqtSignal

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Image, LaserScan, Range
from nav_msgs.msg import OccupancyGrid

from com_interfaces.msg import RobotState
from com_interfaces.srv import StartMission, GetMeasurementLog

# 아래 두 메시지는 아직 com_interfaces에 없을 수 있어서 선택적으로 import.
# 없어도 GUI는 정상 동작하고, 해당 패널만 값이 안 들어올 뿐이야.
try:
    from com_interfaces.msg import EnvData
    HAS_ENV_DATA = True
except ImportError:
    HAS_ENV_DATA = False

try:
    from com_interfaces.msg import SystemError
    HAS_SYSTEM_ERROR = True
except ImportError:
    HAS_SYSTEM_ERROR = False


CAMERA_DISPLAY_HZ = 10          # 카메라를 초당 몇 장만 화면에 그릴지 (CPU 절약)
MAP_QOS_DEPTH = 1


class GuiRosNode(Node):
    """실제 구독/발행/서비스호출을 담당하는 rclpy 노드."""

    def __init__(self, sig):
        super().__init__('com_gui_node')
        self.sig = sig
        self._camera_skip = 0
        self._camera_every_n = max(1, int(30 / CAMERA_DISPLAY_HZ))

        # ---- QoS 설정 ----
        # 상태/명령류는 reliable(반드시 도착), 센서 스트림은 best_effort(최신 우선)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        # /map은 한 번만 발행되고 끝이라 transient_local이어야 나중에 켠 GUI도 받을 수 있음
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=MAP_QOS_DEPTH)

        # ---- 구독 ----
        self.create_subscription(
            RobotState, '/robot_state', self._on_robot_state, reliable_qos)
        self.create_subscription(
            OccupancyGrid, '/map', self._on_map, map_qos)
        self.create_subscription(
            Image, '/camera/image_raw', self._on_image, sensor_qos)
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, sensor_qos)
        self.create_subscription(
            Range, '/rear_ir', self._on_range, sensor_qos)
        # 불꽃 감지 (sensor_bridge가 불꽃감지센서 값을 Bool로 발행)
        self.create_subscription(
            Bool, '/flame_detected', self._on_flame, reliable_qos)
        self.create_subscription(
            Bool, '/person_detected', self._on_person, sensor_qos)

        # ---- 실제 serial_bridge_node가 발행하는 토픽들 (2026-08-04 확인) ----
        # 문서(ICD-001)의 /env_data, /rear_ir 대신 /com_sensors/* 로 발행 중.
        # 위의 문서 기준 구독도 남겨둠 (나중에 어느 쪽으로 통일해도 GUI가 동작하게).
        # serial_bridge QoS가 BEST_EFFORT라 여기도 sensor_qos로 맞춰야 수신됨.
        self.create_subscription(
            Float32, '/com_sensors/temperature', self._on_temperature, sensor_qos)
        self.create_subscription(
            Float32, '/com_sensors/humidity', self._on_humidity, sensor_qos)
        self.create_subscription(
            Bool, '/com_sensors/flame_detected', self._on_flame, sensor_qos)
        self.create_subscription(
            Float32, '/com_sensors/rear_distance', self._on_rear_distance_cm, sensor_qos)

        if HAS_ENV_DATA:
            self.create_subscription(
                EnvData, '/env_data', self._on_env_data, reliable_qos)
        if HAS_SYSTEM_ERROR:
            self.create_subscription(
                SystemError, '/system_error', self._on_system_error, 10)

        # ---- 서비스 클라이언트 (ICD-001 IF-017~019) ----
        self.cli_start = self.create_client(StartMission, '/start_mission')
        self.cli_cancel = self.create_client(Trigger, '/cancel_mission')
        self.cli_log = self.create_client(GetMeasurementLog, '/get_measurement_log')

        # ---- 퍼블리셔 (디버그 수동주행용, IF-001) ----
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

    # ------------------------------------------------------------------
    # 구독 콜백
    # ------------------------------------------------------------------
    def _on_robot_state(self, msg):
        self.sig.robot_state.emit(msg)

    def _on_map(self, msg):
        self.sig.map_received.emit(msg)

    def _on_scan(self, msg):
        # 전방 스캔 중 최소 거리만 뽑아서 넘김 (전체 배열을 GUI로 넘길 필요 없음)
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        valid = ranges[np.isfinite(ranges) & (ranges > msg.range_min)]
        min_dist = float(valid.min()) if valid.size > 0 else float('nan')
        self.sig.scan_summary.emit(min_dist, int(valid.size))

    def _on_range(self, msg):
        self.sig.rear_range.emit(float(msg.range))

    def _on_flame(self, msg):
        self.sig.flame_detected.emit(bool(msg.data))

    def _on_person(self, msg):
        self.sig.person_detected.emit(bool(msg.data))

    def _on_temperature(self, msg):
        self.sig.temperature.emit(float(msg.data))

    def _on_humidity(self, msg):
        self.sig.humidity.emit(float(msg.data))

    def _on_rear_distance_cm(self, msg):
        # serial_bridge의 R 값은 cm 단위 (아두이노 초음파 코드 기준)
        self.sig.rear_distance_cm.emit(float(msg.data))

    def _on_env_data(self, msg):
        self.sig.env_data.emit(
            float(msg.temperature), float(msg.humidity), float(msg.illuminance))

    def _on_system_error(self, msg):
        self.sig.system_error.emit(
            msg.node_name, msg.error_code, msg.message, int(msg.severity))

    def _on_image(self, msg):
        # 프레임 솎아내기: 30Hz 전부 그리면 CPU가 아까움
        self._camera_skip += 1
        if self._camera_skip % self._camera_every_n != 0:
            return
        try:
            arr = self._image_msg_to_rgb(msg)
            if arr is not None:
                # numpy 배열 자체를 넘기고, QImage 생성은 GUI 스레드에서 한다
                self.sig.camera_frame.emit(arr, msg.width, msg.height)
        except Exception as e:
            self.get_logger().warn(f'카메라 프레임 변환 실패: {e}')

    @staticmethod
    def _image_msg_to_rgb(msg):
        """sensor_msgs/Image -> RGB888 numpy 배열. cv_bridge 없이 직접 변환."""
        enc = msg.encoding
        buf = np.frombuffer(msg.data, dtype=np.uint8)

        if enc in ('rgb8', 'bgr8'):
            arr = buf.reshape((msg.height, msg.step))[:, :msg.width * 3]
            arr = arr.reshape((msg.height, msg.width, 3))
            if enc == 'bgr8':
                arr = arr[:, :, ::-1]           # BGR -> RGB
            return np.ascontiguousarray(arr)

        if enc == 'mono8':
            arr = buf.reshape((msg.height, msg.step))[:, :msg.width]
            return np.ascontiguousarray(np.dstack([arr] * 3))

        if enc in ('rgba8', 'bgra8'):
            arr = buf.reshape((msg.height, msg.step))[:, :msg.width * 4]
            arr = arr.reshape((msg.height, msg.width, 4))[:, :, :3]
            if enc == 'bgra8':
                arr = arr[:, :, ::-1]
            return np.ascontiguousarray(arr)

        return None  # 지원 안 하는 인코딩

    # ------------------------------------------------------------------
    # 서비스 호출 (전부 논블로킹: call_async + 콜백)
    # ------------------------------------------------------------------
    def call_start_mission(self, destination_id):
        if not self.cli_start.service_is_ready():
            self.sig.service_failed.emit('start_mission', '서비스 서버를 찾을 수 없습니다')
            return
        req = StartMission.Request()
        req.destination_id = destination_id
        fut = self.cli_start.call_async(req)
        fut.add_done_callback(
            lambda f: self._done(f, self.sig.start_result, 'start_mission'))

    def call_cancel_mission(self):
        if not self.cli_cancel.service_is_ready():
            self.sig.service_failed.emit('cancel_mission', '서비스 서버를 찾을 수 없습니다')
            return
        fut = self.cli_cancel.call_async(Trigger.Request())
        fut.add_done_callback(
            lambda f: self._done(f, self.sig.cancel_result, 'cancel_mission'))

    def _done(self, future, signal, name):
        try:
            res = future.result()
            signal.emit(bool(res.success), str(res.message))
        except Exception as e:
            self.sig.service_failed.emit(name, str(e))

    def call_get_log(self, waypoint_filter, date_filter):
        if not self.cli_log.service_is_ready():
            self.sig.service_failed.emit(
                'get_measurement_log', '서비스 서버를 찾을 수 없습니다')
            return
        req = GetMeasurementLog.Request()
        req.waypoint_id_filter = waypoint_filter
        req.date_filter = date_filter
        fut = self.cli_log.call_async(req)
        fut.add_done_callback(self._on_log_done)

    def _on_log_done(self, future):
        try:
            res = future.result()
            self.sig.log_result.emit(bool(res.success), list(res.records))
        except Exception as e:
            self.sig.service_failed.emit('get_measurement_log', str(e))

    # ------------------------------------------------------------------
    def publish_cmd_vel(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.pub_cmd_vel.publish(twist)


class RosWorkerThread(QThread):
    """rclpy를 돌리는 백그라운드 스레드. 모든 신호(pyqtSignal)가 여기 모여 있다."""

    robot_state = pyqtSignal(object)
    map_received = pyqtSignal(object)
    camera_frame = pyqtSignal(object, int, int)   # (numpy RGB, width, height)
    scan_summary = pyqtSignal(float, int)         # (최소거리 m, 유효 포인트 수)
    rear_range = pyqtSignal(float)                # 후방 거리 (m) — 문서 기준 /rear_ir
    flame_detected = pyqtSignal(bool)             # 불꽃 감지 여부
    person_detected = pyqtSignal(bool)            # 목적지 앞 사람 감지 (SR-012)
    temperature = pyqtSignal(float)               # /com_sensors/temperature (°C)
    humidity = pyqtSignal(float)                  # /com_sensors/humidity (%)
    rear_distance_cm = pyqtSignal(float)          # /com_sensors/rear_distance (cm)
    env_data = pyqtSignal(float, float, float)    # (온도, 습도, 조도)
    system_error = pyqtSignal(str, str, str, int)

    start_result = pyqtSignal(bool, str)
    cancel_result = pyqtSignal(bool, str)
    log_result = pyqtSignal(bool, list)
    service_failed = pyqtSignal(str, str)

    ros_ready = pyqtSignal()

    def __init__(self, ros_domain_id=30):
        super().__init__()
        self.ros_domain_id = ros_domain_id
        self.node = None
        self._running = False

    def run(self):
        os.environ['ROS_DOMAIN_ID'] = str(self.ros_domain_id)
        rclpy.init(args=None)
        self.node = GuiRosNode(self)
        self._running = True
        self.ros_ready.emit()

        while self._running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.node.destroy_node()
        # 2026-08-18: SIGINT로 정상 종료될 때 rclpy 시그널 핸들러가 이미 컨텍스트를
        # 닫아둔 상태라, 여기서 또 shutdown하면 RCLError가 터져 exit code 1로 끝난다.
        # PM2는 그걸 '크래시'로 세서 재시작 카운터를 소진시키고 결국 스택 전체를
        # 내려버린다(8/18 시험주행 후 8개 노드 전부 stopped 된 직접 원인).
        if rclpy.ok():
            rclpy.shutdown()

    def stop(self):
        self._running = False
        self.wait(2000)

    # ---- GUI 스레드에서 부르는 래퍼들 ----
    def start_mission(self, destination_id):
        if self.node:
            self.node.call_start_mission(destination_id)

    def cancel_mission(self):
        if self.node:
            self.node.call_cancel_mission()

    def get_log(self, waypoint_filter='', date_filter=''):
        if self.node:
            self.node.call_get_log(waypoint_filter, date_filter)

    def drive(self, linear_x, angular_z):
        if self.node:
            self.node.publish_cmd_vel(linear_x, angular_z)

    def drive_stop(self):
        if self.node:
            self.node.publish_cmd_vel(0.0, 0.0)
