#!/usr/bin/env python3
"""
DDS-001(상세설계명세서) 2.4절 aruco 노드 스펙 구현.

역할:
  - 카메라 영상에서 ArUco 마커(목적지 검증용 + 도킹용) 검출
  - 마커 중심-로봇 중심 좌우/거리 오차 계산 (정밀 도킹용)
  - 목적지 앞 사람 존재 여부 판단

입력 토픽: /camera/image_raw (sensor_msgs/Image)
출력 토픽:
  - /aruco_marker (com_interfaces/ArucoMarker) : marker_id, pose, lateral_error, distance_error
  - /person_detected (std_msgs/Bool)
  - /system_error (com_interfaces/SystemError) : 카메라 끊김 등 예외 상황

작성자: 권오민 (초안), 2026-07-27
수정: OpenCV ArUco API 버전 분기를 try/except 기반으로 변경 (hasattr 오판단 방지)
"""

import math
import time

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

# HOG(사람감지)가 내부적으로 여러 스레드를 쓰면 Nav2(controller_server 등)의
# 실시간성 스레드와 4코어 Pi에서 컨텍스트 스위칭 경합을 일으킨다 - 이 노드는
# 어차피 detection_rate 주기 타이머 하나로 순차 처리하므로 멀티스레드 이득이
# 크지 않고, 단일 스레드로 고정하는 쪽이 전체 시스템엔 더 유리하다(2026-08-11
# 실측: Nav2+비전 동시구동 시 load average 5~6대, /initialpose가 tf 타이밍
# 경합으로 계속 거부되는 문제 확인 후 person_detection_rate 분리와 함께 적용).
cv2.setNumThreads(1)

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped

from com_interfaces.msg import ArucoMarker, SystemError


class MarkerDetector:
    """ArUco 마커 검출 + 포즈(오차) 계산 담당 (OpenCV ArUco 래핑)."""

    def __init__(self, dictionary_name: str, marker_size_m: float,
                 camera_matrix: np.ndarray, dist_coeffs: np.ndarray):
        self.marker_size = marker_size_m
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

        # DICT_4X4_50 등 문자열 -> cv2 상수로 변환
        dict_id = getattr(cv2.aruco, dictionary_name)

        # OpenCV 버전별 ArUco API 분기
        # hasattr(cv2.aruco, 'ArucoDetector')만으로 판단하면 일부 환경(예: 라즈베리파이 OS
        # apt 빌드)에서 오작동할 수 있어서, 실제로 신버전 함수 호출을 시도해보고
        # 실패하면 구버전(OpenCV < 4.7)으로 자동 폴백하는 방식으로 변경
        try:
            self.dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
            self.detector_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.detector_params)
            self._use_legacy_api = False
        except AttributeError:
            self.dictionary = cv2.aruco.Dictionary_get(dict_id)
            self.detector_params = cv2.aruco.DetectorParameters_create()
            self._use_legacy_api = True

        # 마커의 3D 모서리 좌표 (마커 중심 기준, solvePnP용)
        half = self.marker_size / 2.0
        self.obj_points = np.array([
            [-half, half, 0],
            [half, half, 0],
            [half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float32)

    def detect_markers(self, frame):
        """frame(그레이스케일) -> (corners, ids) 반환. 미검출 시 ids=None."""
        if self._use_legacy_api:
            corners, ids, _rejected = cv2.aruco.detectMarkers(
                frame, self.dictionary, parameters=self.detector_params
            )
        else:
            corners, ids, _rejected = self.detector.detectMarkers(frame)
        return corners, ids

    def compute_pose_error(self, corners):
        """
        검출된 마커 1개의 코너 좌표 -> (pose, lateral_error, distance_error).

        lateral_error : 마커 중심-로봇(카메라) 중심의 좌우 오차 (m)
        distance_error: 마커까지의 거리 (m)
        (ICD-001 ArucoMarker.msg 정의 기준, 주로 도킹 정렬에 사용)
        """
        ok, rvec, tvec = cv2.solvePnP(
            self.obj_points, corners[0], self.camera_matrix, self.dist_coeffs
        )
        if not ok:
            return None, None, None

        # 카메라 좌표계: x=오른쪽, y=아래, z=전방(광축 방향)
        lateral_error = float(tvec[0][0])
        distance_error = float(tvec[2][0])

        pose = PoseStamped()
        pose.pose.position.x = float(tvec[0][0])
        pose.pose.position.y = float(tvec[1][0])
        pose.pose.position.z = float(tvec[2][0])

        rot_mat, _ = cv2.Rodrigues(rvec)
        qw, qx, qy, qz = self._rotmat_to_quaternion(rot_mat)
        pose.pose.orientation.w = qw
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz

        return pose, lateral_error, distance_error

    @staticmethod
    def _rotmat_to_quaternion(r):
        """3x3 회전행렬 -> 쿼터니언(w,x,y,z) 변환."""
        trace = r[0, 0] + r[1, 1] + r[2, 2]
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (r[2, 1] - r[1, 2]) * s
            y = (r[0, 2] - r[2, 0]) * s
            z = (r[1, 0] - r[0, 1]) * s
        else:
            i = np.argmax([r[0, 0], r[1, 1], r[2, 2]])
            if i == 0:
                s = 2.0 * math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
                w = (r[2, 1] - r[1, 2]) / s
                x = 0.25 * s
                y = (r[0, 1] + r[1, 0]) / s
                z = (r[0, 2] + r[2, 0]) / s
            elif i == 1:
                s = 2.0 * math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
                w = (r[0, 2] - r[2, 0]) / s
                x = (r[0, 1] + r[1, 0]) / s
                y = 0.25 * s
                z = (r[1, 2] + r[2, 1]) / s
            else:
                s = 2.0 * math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
                w = (r[1, 0] - r[0, 1]) / s
                x = (r[0, 2] + r[2, 0]) / s
                y = (r[1, 2] + r[2, 1]) / s
                z = 0.25 * s
        return w, x, y, z


class PersonPresenceChecker:
    """목적지 앞 사람 존재 여부 판단 (OpenCV HOG 보행자 검출기 사용)."""

    def __init__(self, confidence_threshold: float):
        self.confidence_threshold = confidence_threshold
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def check_person_presence(self, frame_bgr) -> bool:
        """사람이 하나라도 confidence_threshold 이상으로 검출되면 True.

        존재 여부만 필요하므로 절반 해상도로 축소하고 스케일 스텝을 넉넉히 잡아 검출한다.
        원본 640x480 + scale=1.05 조합은 라즈베리파이4 CPU에서 프레임당 수백ms가 걸려
        5Hz 타이머가 계속 밀리며 aruco_node CPU 사용률이 200%를 넘게 만들었다.
        """
        small = cv2.resize(frame_bgr, None, fx=0.5, fy=0.5)
        boxes, weights = self.hog.detectMultiScale(
            small, winStride=(8, 8), padding=(8, 8), scale=1.2
        )
        if len(weights) == 0:
            return False
        return bool(np.max(weights) >= self.confidence_threshold)


class FrameSampler:
    """카메라 콜백은 항상 최신 프레임만 저장, 타이머가 detection_rate 주기로 꺼내씀."""

    def __init__(self):
        self._latest_frame = None
        self._latest_stamp = None

    def update(self, frame, stamp):
        self._latest_frame = frame
        self._latest_stamp = stamp

    def get_latest(self):
        return self._latest_frame, self._latest_stamp


class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco')

        # ── 파라미터 (DDS-001 2.4절 기준 기본값) ──
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.10)               # m
        self.declare_parameter('detection_rate', 5.0)             # Hz
        # 사람감지(HOG)는 ArUco 마커 검출(도킹 정밀도 때문에 5Hz 필요)과 같은 주기로
        # 돌 필요가 없다 - 저속 실내 주행에서 사람 존재를 1초 이내에 알면 충분하고
        # (즉각 정지는 초음파 OBSTACLE_STOP이 더 빠른 별도 경로로 이미 담당),
        # detection_rate와 분리해서 훨씬 낮은 주기로 돌려 CPU를 아낀다.
        self.declare_parameter('person_detection_rate', 1.0)      # Hz
        self.declare_parameter('person_confidence_threshold', 0.5)
        self.declare_parameter('camera_frame_timeout', 1.0)       # sec (SR-025)
        self.declare_parameter(
            'camera_calibration_file',
            '/home/com/.ros/camera_info/imx219__base_soc_i2c0mux_i2c_1_imx219_10_640x480.yaml'
        )
        self.declare_parameter('markers_yaml_package', 'com_navigation')
        self.declare_parameter('markers_yaml_relpath', 'config/aruco_markers.yaml')

        dictionary_name = self.get_parameter('aruco_dictionary').value
        marker_size = self.get_parameter('marker_size').value
        self.detection_period = 1.0 / self.get_parameter('detection_rate').value
        self.person_detection_period = 1.0 / self.get_parameter('person_detection_rate').value
        person_threshold = self.get_parameter('person_confidence_threshold').value
        self.camera_timeout = self.get_parameter('camera_frame_timeout').value

        camera_matrix, dist_coeffs = self._load_camera_calibration()
        self.known_marker_ids = self._load_known_marker_ids()

        self.detector = MarkerDetector(dictionary_name, marker_size, camera_matrix, dist_coeffs)
        self.person_checker = PersonPresenceChecker(person_threshold)
        self.sampler = FrameSampler()
        self.bridge = CvBridge()

        self._last_frame_time = time.monotonic()
        self._last_person_check_time = 0.0
        self._error_reported = False
        # DOCKING 중에는 사람감지(HOG)를 건너뛰고 마커 검출에 CPU를 몰아준다 - 도킹
        # 정렬은 저속 근접 주행이라 사람감지의 우선순위가 낮고, 이 시점이 바로 CPU
        # 경합으로 마커 인식이 밀리면 안 되는 구간이라 트레이드오프가 명확함
        # (2026-08-06, Pi4 CPU 과부하로 도킹 정렬 실패한 실물 테스트 이후 추가).
        self._mission_state = ''

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.preprocess_image, sensor_qos
        )
        self.mission_state_sub = self.create_subscription(
            String, '/mission_state', self._on_mission_state, 10
        )
        self.marker_pub = self.create_publisher(ArucoMarker, '/aruco_marker', 10)
        self.person_pub = self.create_publisher(Bool, '/person_detected', 10)
        self.error_pub = self.create_publisher(SystemError, '/system_error', 10)

        self.timer = self.create_timer(self.detection_period, self._on_timer)

        self.get_logger().info(
            f'aruco 노드 시작: dict={dictionary_name}, marker_size={marker_size}m, '
            f'rate={self.get_parameter("detection_rate").value}Hz, '
            f'person_rate={self.get_parameter("person_detection_rate").value}Hz, '
            f'등록된 마커 {len(self.known_marker_ids)}개'
        )

    # ── 초기화 헬퍼 ──

    def _load_camera_calibration(self):
        """camera_calibration_file 파라미터로 캘리브레이션 로드. 없으면 기본 추정값 사용."""
        path = self.get_parameter('camera_calibration_file').value
        if path:
            try:
                with open(path, 'r') as f:
                    calib = yaml.safe_load(f)

                # ROS camera_calibration 표준 yaml 포맷:
                # camera_matrix / distortion_coefficients가 {rows, cols, data} dict로 저장됨
                cm = calib['camera_matrix']
                camera_matrix = np.array(
                    cm['data'], dtype=np.float64
                ).reshape(cm['rows'], cm['cols'])

                dc = calib.get('distortion_coefficients', calib.get('dist_coeffs'))
                dist_coeffs = np.array(
                    dc['data'], dtype=np.float64
                ).reshape(dc['rows'], dc['cols'])

                self.get_logger().info(f'카메라 캘리브레이션 로드 완료: {path}')
                return camera_matrix, dist_coeffs
            except Exception as e:
                self.get_logger().warn(
                    f'캘리브레이션 파일 로드 실패({e}), 기본 추정값 사용 - '
                    f'정확한 거리/오차 계산 전에 반드시 실제 캘리브레이션 필요'
                )

        # 미보정 상태의 대략적인 기본값 (640x480 기준 추정치, 실측 캘리브레이션으로 교체 필수)
        camera_matrix = np.array([
            [600.0, 0.0, 320.0],
            [0.0, 600.0, 240.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        return camera_matrix, dist_coeffs

    def _load_known_marker_ids(self):
        """aruco_markers.yaml에서 등록된 마커 ID 목록 로드 (미등록 마커 검출 시 경고용)."""
        try:
            pkg_name = self.get_parameter('markers_yaml_package').value
            relpath = self.get_parameter('markers_yaml_relpath').value
            pkg_path = get_package_share_directory(pkg_name)
            with open(f'{pkg_path}/{relpath}', 'r') as f:
                data = yaml.safe_load(f)
            return {m['id']: m.get('description', '') for m in data['markers']}
        except Exception as e:
            self.get_logger().warn(f'aruco_markers.yaml 로드 실패({e}), 마커 ID 검증 생략')
            return {}

    # ── 콜백 ──

    def _on_mission_state(self, msg: String):
        self._mission_state = msg.data

    def preprocess_image(self, msg: Image):
        """카메라 콜백: BGR 프레임으로 변환해 최신 프레임 버퍼에 저장만 함 (검출은 타이머가 수행)."""
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # libcamera 0.1은 orientation 파라미터 미지원 → 카메라 장착 방향 보정용 180도 회전
            frame_bgr = cv2.rotate(frame_bgr, cv2.ROTATE_180)
        except Exception as e:
            self.get_logger().error(f'이미지 변환 실패: {e}')
            return
        self._last_frame_time = time.monotonic()
        self.sampler.update(frame_bgr, msg.header.stamp)

    def _on_timer(self):
        """detection_rate(기본 5Hz) 주기로 최신 프레임에 대해 마커/사람 검출 수행."""
        # SR-025: 카메라 프레임 1초 이상 미수신 시 미검출 상태 발행 + /system_error
        if self._last_frame_time is None or \
           (time.monotonic() - self._last_frame_time) > self.camera_timeout:
            self._publish_no_detection()
            # 2026-08-18 코드 리뷰: /aruco_marker는 -1로 리셋하면서 /person_detected는
            # 손 안 대서 카메라가 죽기 직전 마지막 값이 그대로 고정돼있었다. 사람이
            # 있는 상태에서 카메라가 죽으면 PERSON_WAIT에 영원히 갇히는 것도 문제지만,
            # 반대로 없는 상태에서 죽으면 그 사이 사람이 다가와도 못 알아채고 그대로
            # 주행하는 게 더 위험하다 - 카메라가 안 보이는 동안은 "사람 있음"으로
            # 안전 쪽으로 고정한다(fail-safe). 단, DOCKING 중엔 기존과 동일하게
            # /person_detected를 아예 발행하지 않는다 - service_manager의
            # handle_person_detected는 상태와 무관하게 person_present=True면 즉시
            # /cmd_vel을 0으로 눌러버려서, 여기서 발행하면 도킹 접근이 매번 끊긴다
            # (아래 person 체크 스킵 로직과 동일한 이유, service_manager_node.py 주석 참고).
            if self._mission_state != 'DOCKING':
                self.publish_person(True)
            if not self._error_reported:
                self._publish_camera_timeout_error()
                self._error_reported = True
            return
        self._error_reported = False

        frame_bgr, stamp = self.sampler.get_latest()
        if frame_bgr is None:
            return

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids = self.detector.detect_markers(gray)
        self.publish_marker((corners, ids), stamp)

        now = time.monotonic()
        if (self._mission_state != 'DOCKING'
                and (now - self._last_person_check_time) >= self.person_detection_period):
            self._last_person_check_time = now
            person_present = self.person_checker.check_person_presence(frame_bgr)
            self.publish_person(person_present)

    # ── 발행 ──

    def publish_marker(self, result, stamp):
        """검출 결과(corners, ids) -> /aruco_marker 발행. 미검출 시 marker_id=-1로 발행."""
        corners, ids = result

        if ids is None or len(ids) == 0:
            self._publish_no_detection()
            return

        # 여러 마커가 동시에 시야에 들어오면(예: 미등록 마커 + 도킹 마커 29) OpenCV가
        # 반환하는 순서는 검출 순서일 뿐 우선순위가 아니다 - 예전엔 무조건 ids[0]만
        # 썼는데, 등록된(known) 마커가 있으면 그걸 우선 선택하도록 수정. 안 그러면
        # DockingController가 도킹 마커를 dock_marker_id로 필터링하는 로직 자체가
        # 무의미해짐(엉뚱한 마커만 계속 보고되어 도킹 마커를 매 프레임 놓칠 수 있음 -
        # 2026-08-06 실물 도킹 실패의 유력한 원인으로 판단해 수정).
        ids_flat = [int(i[0]) for i in ids]
        idx = 0
        for i, mid in enumerate(ids_flat):
            if mid in self.known_marker_ids:
                idx = i
                break

        marker_id = ids_flat[idx]
        pose, lateral_error, distance_error = \
            self.detector.compute_pose_error([corners[idx]])

        if pose is None:
            self._publish_no_detection()
            return

        if self.known_marker_ids and marker_id not in self.known_marker_ids:
            self.get_logger().warn(
                f'등록되지 않은 마커 ID {marker_id} 검출됨 - aruco_markers.yaml 확인 필요',
                throttle_duration_sec=5.0
            )

        msg = ArucoMarker()
        msg.marker_id = marker_id
        pose.header.stamp = stamp
        pose.header.frame_id = 'camera_frame'
        msg.pose = pose
        msg.lateral_error = lateral_error
        msg.distance_error = distance_error
        self.marker_pub.publish(msg)

    def _publish_no_detection(self):
        """미검출 상태 발행 (이전 값 유지하지 않음 - DDS-001 2.4절 예외처리 원칙)."""
        msg = ArucoMarker()
        msg.marker_id = -1
        self.marker_pub.publish(msg)

    def publish_person(self, present: bool):
        msg = Bool()
        msg.data = present
        self.person_pub.publish(msg)

    def _publish_camera_timeout_error(self):
        err = SystemError()
        err.node_name = 'aruco'
        err.error_code = 'E_CAMERA_TIMEOUT'
        err.message = f'카메라 프레임 {self.camera_timeout}초 이상 미수신'
        err.severity = 2  # ERROR
        err.stamp = self.get_clock().now().to_msg()
        self.error_pub.publish(err)
        self.get_logger().error(err.message)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # 2026-08-18: SIGINT로 정상 종료될 때 rclpy 시그널 핸들러가 이미 컨텍스트를
        # 닫아둔 상태라, 여기서 또 shutdown하면 RCLError가 터져 exit code 1로 끝난다.
        # PM2는 그걸 '크래시'로 세서 재시작 카운터를 소진시키고 결국 스택 전체를
        # 내려버린다(8/18 시험주행 후 8개 노드 전부 stopped 된 직접 원인).
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
