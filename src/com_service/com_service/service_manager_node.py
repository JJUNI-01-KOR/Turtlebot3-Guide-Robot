import math
import os
import yaml
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time as RclpyTime

from std_msgs.msg import UInt8, String, Float32, Bool
from std_srvs.srv import Trigger
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from com_interfaces.srv import StartMission
from com_interfaces.msg import ArucoMarker, TtsRequest
from nav2_msgs.action import NavigateToPose

from ament_index_python.packages import get_package_share_directory

from com_service.mission_state import MissionState


def _wrap_pi(angle: float) -> float:
    """각도를 -pi ~ +pi 로 접는다.
    안 하면 359도 -> 1도 넘어갈 때 오차가 -358도로 튀어 판정이 뒤집힌다."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _odom_yaw(q) -> float:
    """오도메트리 쿼터니언 -> z축(수직) 회전각. 2륜 로봇은 yaw만 필요."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _marker_yaw(q) -> float:
    """마커 자세 쿼터니언 -> "마커가 좌우로 얼마나 비틀렸나".

    카메라 좌표계는 x=오른쪽, y=아래, z=전방이므로 수직축은 y축이다.
    따라서 회전행렬 R에 대해 yaw = atan2(-R20, R00) 이고, 쿼터니언으로는
        R00 = 1 - 2(y^2 + z^2),  R20 = 2(xz - wy)
    로봇이 마커 정면에 있으면 0이 나온다.
    (부호가 실제 방향과 반대면 dock_k_yaw 를 음수로 주면 된다.)
    """
    r00 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    r20 = 2.0 * (q.x * q.z - q.w * q.y)
    return math.atan2(-r20, r00)


class WaypointSequencer:
    """
    목적지 웨이포인트 순차 제어 (SR-006).

    목적지 ID는 waypoints.yaml에 정의된 각 웨이포인트의 'id'를 그대로 사용한다
    (8개 강의실/회의실 구조 재설계 반영, 구 'dest_A/B/C' 합성 키 방식 폐기).
    """

    def __init__(self, node: Node):
        self.node = node
        self.waypoints = {}
        self.destination_ids = []
        self.docking_waypoint_id = node.get_parameter('docking_waypoint_id').value
        self.route_prefix_ids = []
        self.route_return_ids = []
        self.current_trajectory = []
        self.current_index = -1
        self.destination_id = None

    def load_waypoints(self, yaml_path: str = None):
        if yaml_path is None:
            share_dir = get_package_share_directory('com_navigation')
            yaml_path = os.path.join(share_dir, 'config', 'waypoints.yaml')

        if not os.path.exists(yaml_path):
            self.node.get_logger().error(f'waypoints.yaml not found: {yaml_path}')
            return False

        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        for wp in data.get('waypoints', []):
            self.waypoints[wp['id']] = wp

        self.destination_ids = [
            wp['id'] for wp in data.get('waypoints', []) if wp.get('is_destination')
        ]
        # 도킹 지점 -> 목적지 공통 경유 웨이포인트(선택 사항, 없으면 목적지로 직행)
        self.route_prefix_ids = data.get('trajectory_1', {}).get('waypoint_ids', [])
        self.route_return_ids = data.get('trajectory_2', {}).get('waypoint_ids', [])

        # 2026-08-19: 두 키가 없어도 코드는 조용히 "직행"으로 동작해버리므로,
        # 정의되지 않은 상태를 명시적으로 알린다. 실제로 way6~way16/docking2
        # 경유점 11개가 정의돼 있는데 trajectory 키가 없어 한 번도 안 쓰이고 있었다.
        unused = [
            wp_id for wp_id in self.waypoints
            if wp_id not in self.destination_ids
            and wp_id != self.docking_waypoint_id
            and wp_id not in self.route_prefix_ids
            and wp_id not in self.route_return_ids
        ]
        if not self.route_prefix_ids and not self.route_return_ids:
            self.node.get_logger().warn(
                'waypoints.yaml에 trajectory_1/trajectory_2가 없어 도킹<->목적지를 '
                f'단일 Nav2 goal로 직행합니다. 미사용 경유점 {len(unused)}개: {unused}'
            )
        elif unused:
            self.node.get_logger().info(f'어느 경로에도 안 쓰이는 웨이포인트: {unused}')

        if self.docking_waypoint_id not in self.waypoints:
            self.node.get_logger().warn(
                f"docking_waypoint_id='{self.docking_waypoint_id}' 이(가) "
                'waypoints.yaml에 없습니다 - DOCKING 전이가 발생하지 않습니다'
            )

        self.node.get_logger().info(
            f'waypoints loaded: {len(self.waypoints)}개, destinations={self.destination_ids}'
        )
        return True

    def is_valid_destination(self, destination_id: str) -> bool:
        return destination_id in self.destination_ids

    def build_trajectory_1(self, destination_id: str):
        self.destination_id = destination_id
        self.current_trajectory = self.route_prefix_ids + [destination_id]
        self.current_index = -1

    def build_trajectory_2(self, destination_id: str = None):
        self.destination_id = destination_id
        self.current_trajectory = self.route_return_ids + [self.docking_waypoint_id]
        self.current_index = -1

    def has_next(self) -> bool:
        return self.current_index + 1 < len(self.current_trajectory)

    def advance(self):
        self.current_index += 1
        if self.current_index >= len(self.current_trajectory):
            return None
        wp_id = self.current_trajectory[self.current_index]
        return self.waypoints.get(wp_id)

    def current_waypoint_id(self):
        if 0 <= self.current_index < len(self.current_trajectory):
            return self.current_trajectory[self.current_index]
        return None


class DockingController:
    """
    ArUco 기반 정밀 도킹 제어 (SR-008~SR-011, 핵심 신규 기술).

    입력: /aruco_marker (ArucoMarker.msg) - DOCKING 상태에서만 처리
    출력: /cmd_vel (저속, docking_speed_limit 이하)

    ═══ 2026-08-18 개정: 360도 탐색 -> 좌 90도 고정 회전 ═══

    [문제] 도킹 지점 도착 후 15도씩 회전하며 마커를 찾다가, 마커를 이미
           지나친 뒤에야 "여기 있다"고 인식해 엉뚱한 방향으로 갔다.

    [원인] 카메라 파이프라인 지연. 이미지가 촬영된 시각과 /aruco_marker가
           발행되는 시각 사이에 Pi4에서 수백 ms가 걸린다(캡처 -> image_raw ->
           aruco 노드 5Hz 타이머 -> solvePnP). 회전 중이면 그 지연 동안
           로봇이 계속 돌기 때문에, 받은 좌우오차는 "과거의 값"이고
           현재 로봇은 이미 마커를 지나쳐 있다. 회전이 빠를수록 더 어긋난다.

    [해결] 두 가지를 같이 적용한다.
      (1) 움직이면서는 절대 마커를 판정하지 않는다. 좌 90도로 돌고 -> 완전히
          멈추고 -> 그 다음에 본다 (stop-and-look).
      (2) 멈춘 뒤에도, 멈추기 전에 '촬영된' 프레임이 뒤늦게 도착할 수 있다.
          aruco 노드는 pose.header.stamp 에 카메라 프레임 시각을 넣어주므로,
          그 값이 정지 시각보다 이전이면 통째로 버린다. 이게 없으면 정지
          직후 회전 중 찍힌 낡은 값 하나에 속아서 똑같은 문제가 반복된다.

    단계(phase):
      TURN_LEFT -> SETTLE -> ALIGN -> (정렬 완료 시 DOCKED)
    """

    PHASE_TURN_LEFT = 'TURN_LEFT'
    PHASE_SETTLE = 'SETTLE'
    PHASE_ALIGN = 'ALIGN'

    def __init__(self, node: Node):
        self.node = node
        self.docking_speed_limit = node.get_parameter('docking_speed_limit').value
        self.docking_max_attempts = node.get_parameter('docking_max_attempts').value
        self.aruco_search_max = node.get_parameter('aruco_search_max').value
        self.dock_marker_id = node.get_parameter('dock_marker_id').value

        self.dock_turn_left_angle = node.get_parameter('dock_turn_left_angle').value
        self.dock_nudge_angle = node.get_parameter('dock_nudge_angle').value
        self.dock_turn_speed = node.get_parameter('dock_turn_speed').value
        self.dock_turn_tolerance = node.get_parameter('dock_turn_tolerance').value
        self.dock_settle_time = node.get_parameter('dock_settle_time').value
        self.dock_stale_margin = node.get_parameter('dock_stale_margin').value
        self.k_yaw = node.get_parameter('dock_k_yaw').value
        self.use_yaw_term = node.get_parameter('dock_use_yaw_term').value

        self.dock_target_distance = node.get_parameter('dock_target_distance').value
        self.lateral_tolerance = 0.02
        self.distance_tolerance = 0.03

        self.attempt_count = 0
        self.search_count = 0
        self.last_marker_stamp = None
        self.active = False

        self.k_lateral = 0.8
        self.k_distance = 0.3

        self.phase = self.PHASE_TURN_LEFT
        self.turn_accum = 0.0
        self.turn_prev_yaw = None
        self.turn_goal = self.dock_turn_left_angle
        self.stop_time = None

    # ── 시작/정지 ──
    def start(self):
        self.active = True
        self.attempt_count = 0
        self.search_count = 0
        self.last_marker_stamp = self.node.get_clock().now()
        self._begin_turn_left()
        self.node.get_logger().info(
            'DockingController 시작: 좌 %.0f도 회전 후 마커 확인'
            % (self.dock_turn_left_angle * 180.0 / math.pi)
        )

    def stop(self):
        self.active = False
        self.node.publish_cmd_vel(0.0, 0.0)

    def _begin_turn_left(self, angle=None):
        """좌회전 시작. angle 을 주면 그만큼만 돈다(미세 스캔용)."""
        self.phase = self.PHASE_TURN_LEFT
        self.turn_accum = 0.0
        self.turn_prev_yaw = self.node.odom_yaw
        self.turn_goal = self.dock_turn_left_angle if angle is None else angle
        self.stop_time = None

    def _enter_settle(self):
        self.phase = self.PHASE_SETTLE
        self.node.publish_cmd_vel(0.0, 0.0)
        # 정지 시각을 기록해두고, 이보다 먼저 촬영된 마커 프레임은 전부 버린다.
        self.stop_time = self.node.get_clock().now()
        self.node.get_logger().info(
            '좌회전 완료 (목표 %.1f도 / 실제 %.1f도) - 정지 후 마커 확인 대기'
            % (self.turn_goal * 180.0 / math.pi,
               self.turn_accum * 180.0 / math.pi)
        )

    # ── 주기 처리 (docking_timer, 20Hz) ──
    def update(self):
        if not self.active:
            return
        if self.phase == self.PHASE_TURN_LEFT:
            self._update_turn_left()
        elif self.phase == self.PHASE_SETTLE:
            # 정지 유지. 마커는 on_aruco_marker 콜백이 받아서 ALIGN으로 넘긴다.
            self.node.publish_cmd_vel(0.0, 0.0)

    def _update_turn_left(self):
        yaw = self.node.odom_yaw
        if yaw is None:
            self.node.get_logger().warn(
                '/odom 미수신 - 좌회전 대기', throttle_duration_sec=3.0
            )
            return

        if self.turn_prev_yaw is None:
            self.turn_prev_yaw = yaw
            return

        # 누적 방식: 매 주기 yaw 차분을 더한다. 시작yaw와 현재yaw를 직접 빼면
        # +pi/-pi 경계에서 부호가 뒤집혀 판정이 오락가락한다.
        self.turn_accum += abs(_wrap_pi(yaw - self.turn_prev_yaw))
        self.turn_prev_yaw = yaw

        remaining = self.turn_goal - self.turn_accum
        if remaining <= self.dock_turn_tolerance:
            self._enter_settle()
            return

        # 목표 근처 감속 - 관성 오버슈트를 줄인다
        speed = self.dock_turn_speed
        if remaining < 0.35:      # 약 20도
            speed = max(0.12, self.dock_turn_speed * remaining / 0.35)
        self.node.publish_cmd_vel(0.0, speed)   # +값 = 좌회전(CCW)

    # ── 마커 콜백 ──
    def on_aruco_marker(self, msg: ArucoMarker):
        if not self.active:
            return
        # aruco 노드는 미검출 시 marker_id=-1 을 계속 발행한다.
        # 이 조건 하나로 '미검출'과 '다른 마커'가 같이 걸러진다.
        if msg.marker_id != self.dock_marker_id:
            return

        # 회전 중에 들어온 값은 카메라 지연 때문에 이미 과거의 값이다. 무시.
        if self.phase == self.PHASE_TURN_LEFT:
            return

        # 정지 전에 '촬영된' 프레임이 뒤늦게 도착한 것이면 버린다.
        # pose.header.stamp 는 aruco 노드가 넣어준 카메라 프레임 시각이라
        # 발행 시각이 아니라 실제 촬영 시각 기준으로 판정할 수 있다.
        if self.stop_time is not None:
            frame_time = RclpyTime.from_msg(msg.pose.header.stamp)
            age = (frame_time - self.stop_time).nanoseconds / 1e9
            if age < self.dock_stale_margin:
                self.node.get_logger().debug(
                    '정지 이전에 촬영된 낡은 마커 프레임 무시 (%.2fs)' % age
                )
                return

        self.last_marker_stamp = self.node.get_clock().now()
        self.search_count = 0

        if self.phase == self.PHASE_SETTLE:
            self.phase = self.PHASE_ALIGN
            self.node.get_logger().info(
                '마커 확인 (거리 %.2fm, 좌우 %+.3fm) - 정렬/접근 시작'
                % (msg.distance_error, msg.lateral_error)
            )

        if self._is_aligned(msg):
            self._on_dock_success()
            return

        lin, ang = self.compute_docking_velocity(msg)
        self.node.publish_cmd_vel(lin, ang)

    def compute_docking_velocity(self, marker: ArucoMarker):
        lateral = marker.lateral_error
        distance = marker.distance_error

        # ArucoMarker.msg 에는 yaw_error 필드가 없지만, pose.orientation 에
        # solvePnP 결과가 그대로 들어있으므로 여기서 직접 뽑는다.
        # lateral 항만 쓰면 마커 '위치'로 커브를 그리며 갈 뿐이라, 도착했을 때
        # 마커를 비스듬히 보게 된다. yaw 항이 "마커 정면 직선 위로 올라타라"는
        # 성분을 더해줘서 전진하는 동안 두 오차가 같이 0으로 수렴한다.
        range_error = self._range_error(marker)
        steer = self.k_lateral * lateral
        if self.use_yaw_term:
            steer += self.k_yaw * _marker_yaw(marker.pose.pose.orientation)

        angular_z = -steer
        linear_x = self.k_distance * range_error

        linear_x = max(-self.docking_speed_limit, min(self.docking_speed_limit, linear_x))
        angular_z = max(-0.3, min(0.3, angular_z))
        return linear_x, angular_z

    def _range_error(self, marker: ArucoMarker) -> float:
        """정지 목표 지점까지 남은 거리(m). +면 더 전진, -면 너무 붙은 것.

        2026-08-19: ArucoMarker.distance_error는 이름과 달리 '오차'가 아니라
        카메라에서 마커까지의 생거리다(aruco_node.compute_pose_error가 tvec[2]를
        그대로 담음). 이 값을 그대로 제어 오차로 쓰면 목표 정지거리가 0이 되어
        마커 3cm 앞까지 밀고 들어간다 - 10cm 마커는 그 훨씬 전에 화각을 벗어나
        검출이 끊기므로 정렬 완료 판정에 도달할 수 없고, 로봇은 이미 도크에
        밀착한 상태가 된다. 목표 정지거리를 빼서 진짜 오차로 만든다.
        """
        return marker.distance_error - self.dock_target_distance

    def _is_aligned(self, marker: ArucoMarker) -> bool:
        return (abs(marker.lateral_error) <= self.lateral_tolerance
                and abs(self._range_error(marker)) <= self.distance_tolerance)

    def _on_dock_success(self):
        self.active = False
        self.node.publish_cmd_vel(0.0, 0.0)
        self.node.get_logger().info('정밀 도킹 완료 (정렬 오차 허용범위 이내)')
        self.node.state_machine.transition(MissionState.DOCKED)
        self.node.publish_tts('docked_arrive')

    def check_marker_timeout(self):
        """마커를 오래 못 보면 재시도.

        2026-08-18: 예전의 '15도씩 회전 재탐색'을 제거했다. 회전하면서 찾는
        방식이 바로 카메라 지연 때문에 마커를 지나치는 원인이었다. 대신
        제자리에서 다시 좌 90도 회전을 시도한다(오도메트리 오차 누적 대비).
        """
        if not self.active or self.last_marker_stamp is None:
            return
        if self.phase == self.PHASE_TURN_LEFT:
            return      # 회전 중엔 원래 마커가 안 보이는 게 정상

        elapsed = (self.node.get_clock().now() - self.last_marker_stamp).nanoseconds / 1e9
        if elapsed < 5.0:
            return

        self.search_count += 1
        self.last_marker_stamp = self.node.get_clock().now()

        if self.search_count <= self.aruco_search_max:
            # 2026-08-18: 예전엔 여기서 제자리 대기만 해서, 회전이 조금 모자라
            # 마커가 화각 밖에 있으면 영원히 못 찾았다. 이제 dock_nudge_angle
            # 만큼만 더 돌고 다시 멈춰서 본다("돌고-멈추고-보고" 반복).
            # 회전 중에는 절대 판정하지 않으므로 카메라 지연 문제는 생기지 않는다.
            self.node.get_logger().warn(
                'ArUco 마커 미검출 %.1fs -> 좌로 %.0f도 더 돌고 재확인 (%d/%d)'
                % (elapsed, self.dock_nudge_angle * 180.0 / math.pi,
                   self.search_count, self.aruco_search_max)
            )
            self._begin_turn_left(self.dock_nudge_angle)
            return

        self.search_count = 0
        self.attempt_count += 1
        if self.attempt_count > self.docking_max_attempts:
            self.node.get_logger().error('도킹 시도 3회 실패 -> 정지 상태 유지, 관리자 알림 (SR-011)')
            self.active = False
            self.node.publish_cmd_vel(0.0, 0.0)
            self.node.state_machine.transition(MissionState.ERROR)
            return

        # 미세 스캔을 다 소진했다는 건 방향 자체가 틀렸다는 뜻이므로, 여기서만
        # 처음부터(dock_turn_left_angle) 다시 돈다. 미세 스캔으로 이미 돌아간
        # 각도만큼은 빼서, 매 시도마다 180도로 누적되지 않게 한다.
        already = self.dock_nudge_angle * self.aruco_search_max
        retry_angle = max(0.0, self.dock_turn_left_angle - already)
        self.node.get_logger().warn(
            'ArUco 마커 재확인 %d회 실패 -> 좌로 %.0f도 더 회전 후 재시도 (%d/%d)'
            % (self.aruco_search_max, retry_angle * 180.0 / math.pi,
               self.attempt_count, self.docking_max_attempts)
        )
        self.node.publish_cmd_vel(0.0, 0.0)
        self._begin_turn_left(retry_angle)


class SafetyMonitor:
    """긴급정지/장애물/배터리 등 안전 이벤트 종합 판단 (SR-021~SR-026)."""

    def __init__(self, node: Node):
        self.node = node
        self.obstacle_stop_distance = node.get_parameter('obstacle_stop_distance').value
        self.obstacle_search_max = node.get_parameter('obstacle_search_max').value
        self.battery_return_threshold = node.get_parameter('battery_return_threshold').value

        self.battery_voltage_full = 12.6
        self.battery_voltage_empty = 9.0
        # OpenCR이 배터리 미장착/USB 급전 상태에서 voltage=0.0을 그대로 발행하는 경우가
        # 있다. 이걸 그냥 환산하면 0%가 되어 주행 중에 저전압 복귀가 오발동하므로,
        # 물리적으로 있을 수 없는 값은 아예 무시한다(3S LiPo는 6V 아래로 안 떨어진다).
        self.battery_voltage_min_valid = 6.0
        # 전압이 초 단위로 흔들린다(실측: 같은 충전 상태에서 퍼센트가 86~90을 오감).
        # 그대로 쓰면 (1) HMI/GUI 숫자가 계속 떨리고 (2) 주행 중 모터 부하로 순간
        # 전압 강하가 생기면 저전압 복귀(SR-022)가 오발동한다. 지수이동평균으로
        # 평활화한 값을 퍼센트 환산과 임계 판정에 함께 쓴다. (2026-08-19)
        self.battery_voltage_filtered = None
        self.battery_filter_alpha = 0.2
        self.last_battery_percent = 100.0
        self.battery_return_triggered = False

        self._obstacle_active = False
        self._obstacle_retry_count = 0

        # 후방 초음파 디바운스용.
        # 2026-08-19: 08-18에 도입된 "N회 연속 확인"은 초음파가 10Hz로 온다는
        # 전제였는데, 실측 결과 아두이노는 여전히 2초 주기(0.50Hz)로 보낸다
        # (arduino/flaem_pir_dht11_jul31a.ino의 TELEMETRY_INTERVAL=2000).
        # 그 상태에서 3회 연속을 요구하면 확정까지 6초가 걸려, 0.22m/s에서
        # 1.3m를 더 가고 나서야 정지한다. 발행 주기가 바뀌어도 항상 옳도록
        # 횟수가 아니라 "얼마나 오래 유지됐는가"로 판정한다.
        self.rear_confirm_sec = node.get_parameter('rear_obstacle_confirm_sec').value
        self.rear_clear_sec = node.get_parameter('rear_clear_confirm_sec').value
        # 발행이 느릴 때 기준이 되는 수신 간격(s). 이보다 느리면 필터를 얕게 쓴다.
        self.rear_sparse_interval = node.get_parameter('rear_sparse_interval').value
        self._rear_buf = deque(maxlen=5)
        self._rear_stamp = None
        self._rear_interval = None      # 수신 간격 EMA
        self._obstacle_since = None
        self._clear_since = None

        self.person_wait_retry_max = node.get_parameter('person_wait_retry_max').value
        self._person_active = False
        self._person_retry_count = 0

        # 2026-08-19: 장애물/사람 두 정지 사유가 겹칠 수 있으므로 "정지 직전의 주행
        # 상태"는 하나로 공유한다. 예전엔 두 핸들러가 각자 이전 상태를 저장/복원해서,
        # PERSON_WAIT 중 장애물이 감지된 뒤 사람만 먼저 비켜나면 장애물이 그대로인데도
        # _state_before_person(=GUIDING)으로 복귀하며 주행을 재개하는 버그가 있었다.
        self._state_before_stop = None
        # 래치와 별개로 "지금 이 순간" 센서 판정도 들고 있는다. 임무 시작 시점에 이미
        # 사람/장애물이 있는지 보려면 에지가 아니라 현재값이 필요하다(blocking_reason).
        self._last_person_present = False
        self._last_obstacle_present = False

    def _rear_is_sparse(self) -> bool:
        """초음파가 드물게 오는 상태인가 (중앙값 필터를 얕게 써야 하는가).

        2026-08-19: 08-18에 들어온 중앙값 필터(창 5)는 초음파가 10Hz라는 전제였다.
        실측은 0.50Hz(2초 주기)라, 창 5짜리 중앙값이 뒤집히는 데만 3개(6초)가
        걸리고 거기에 확정 대기까지 더해져 장애물 정지가 8초나 늦었다
        (0.22m/s에서 1.7m). 08-18 주석도 "2초에 한 번 올 때는 튄 값 확률이 낮아
        티가 안 났다"고 적고 있다 - 즉 느릴 때는 깊은 필터가 이득이 아니라 손해다.
        그래서 수신 간격을 재서, 느리면 창 3 + 즉시 확정으로 전환한다.
        빠른 펌웨어로 바꾸면 자동으로 창 5 + 시간 확정으로 되돌아간다.
        """
        return (self._rear_interval is not None
                and self._rear_interval > self.rear_sparse_interval)

    def reset_obstacle_retries(self):
        self._obstacle_retry_count = 0

    def reset_person_retries(self):
        self._person_retry_count = 0

    def reset_stop_conditions(self):
        """정지 사유 래치를 전부 해제한다 (임무 취소 등으로 흐름이 끊길 때).

        2026-08-19 실기체 실측: cancel_mission이 상태머신만 READY로 되돌리고
        _person_active를 True로 남겨두는 바람에, 사람이 앞에 그대로 서 있는데도
        다음 임무에서 `person_present and not self._person_active`가 False가 되어
        PERSON_WAIT가 한 번도 안 걸렸다. 로봇이 사람 앞에서 그냥 출발해버린다.
        """
        self._obstacle_active = False
        self._person_active = False
        self._state_before_stop = None
        # 2026-08-19: _rear_buf(중앙값 필터용 센서 이력)는 일부러 안 비운다.
        # 초음파가 0.5Hz라 버퍼를 비우면 다시 3개가 찰 때까지 6초 동안 장애물
        # 감지가 통째로 멎는다(취소 직후가 가장 위험한 구간인데 그때 눈이 감긴다).
        # 래치와 연속 판정만 초기화하면 충분하다.
        self._obstacle_since = None
        self._clear_since = None

    def blocking_reason(self):
        """지금 임무를 시작하면 안 되는 사유. 없으면 None."""
        if self._last_obstacle_present:
            return '후방에 장애물이 감지되어 있습니다'
        if self._last_person_present:
            return '전방에 사람이 감지되어 있습니다'
        return None

    # Nav2가 주행을 담당하는(=장애물/사람 감지가 의미 있는) 상태. 정지 상태 두 개도
    # 포함해야 "정지 중에 다른 사유가 추가로 발생"하는 경우를 놓치지 않는다.
    NAV_DRIVEN_STATES = (
        MissionState.GUIDING, MissionState.RETURNING,
        MissionState.PERSON_WAIT, MissionState.OBSTACLE_STOP,
    )

    # 실제로 주행 중인 상태. nav goal 결과를 처리해도 되는 상태이기도 하다 -
    # NAV_DRIVEN_STATES를 그대로 쓰면, 사람 감지로 취소한 goal의 CANCELED 결과가
    # PERSON_WAIT에서 _handle_nav_failure()로 흘러들어가 재시도 goal을 보내면서
    # 사람이 아직 앞에 있는데 주행이 재개된다(2026-08-19 실기체 실측).
    DRIVING_STATES = (MissionState.GUIDING, MissionState.RETURNING)

    # 도킹 중에는 후방 장애물 판정을 하지 않는다.
    # 도킹은 "충전 스테이션 벽에 일부러 붙는" 동작이라, 20cm 안으로 들어가는
    # 것이 정상이다. 여기서 정지를 걸면 도킹 자체가 원천적으로 불가능해진다.
    OBSTACLE_IGNORE_STATES = (
        MissionState.DOCKING, MissionState.DOCKED,
    )

    def handle_rear_ir(self, msg: Float32):
        """후방 초음파(/com_sensors/rear_distance, cm) 처리.

        아두이노는 echo 타임아웃 시 -1.0을 보낸다. 이건 "멀어서 안 잡힘"이므로
        장애물 없음으로 처리해야 한다(음수를 그냥 비교하면 항상 장애물로 오탐).

        2026-08-18: 초음파 발행 주기를 2초 -> 100ms 로 바꾼 뒤 주행이 망가졌다.
        원인은 필터가 없었던 것. 초음파는 원래 가끔 완전히 튄 값을 하나씩 섞어
        보내는데, 2초에 한 번 올 때는 그 확률이 낮아 티가 안 났다. 10Hz가 되자
        튄 값 하나가 곧바로 OBSTACLE_STOP -> Nav2 goal 취소 -> 복구 동작 ->
        해제 -> 재시도 를 초당 몇 번씩 반복시켰고, 그 사이 retry_count도 같이
        소진되면서 로봇이 엉뚱한 방향으로 가거나 벽에 붙었다.

        그래서 두 겹으로 막는다:
          (1) 중앙값 필터 - 튄 값 하나는 통째로 무시된다(평균은 끌려감)
          (2) 연속 확인 - N회 연속 같은 판정이 나와야 상태를 바꾼다
        """
        # 도킹 중에는 아예 개입하지 않는다
        if self.node.state_machine.state in self.OBSTACLE_IGNORE_STATES:
            self._rear_buf.clear()
            self._obstacle_since = None
            self._clear_since = None
            return

        # 수신 간격 측정 (필터 깊이를 발행률에 맞추기 위함)
        _now = self.node.get_clock().now()
        if self._rear_stamp is not None:
            _dt = (_now - self._rear_stamp).nanoseconds / 1e9
            self._rear_interval = _dt if self._rear_interval is None else \
                0.3 * _dt + 0.7 * self._rear_interval
        self._rear_stamp = _now

        distance_cm = msg.data
        # -1.0(미검출)은 "아주 멀다"로 환산해 버퍼에 넣는다. 버리면 허공에서
        # 버퍼가 안 차서 판정 자체가 멈춘다.
        if distance_cm < 0 or distance_cm > 400.0:
            self._rear_buf.append(float('inf'))
        else:
            self._rear_buf.append(distance_cm / 100.0)

        if len(self._rear_buf) < 3:
            return

        # 느리게 들어올 때는 최근 3개만 본다(위 _rear_is_sparse 주석 참고).
        window = list(self._rear_buf)[-3:] if self._rear_is_sparse() else list(self._rear_buf)
        finite = [v for v in window if v != float('inf')]
        # 유효값이 과반이 안 되면 "장애물 없음"으로 본다
        if len(finite) * 2 <= len(window):
            distance_m = float('inf')
        else:
            ordered = sorted(finite)
            distance_m = ordered[len(ordered) // 2]

        obstacle_now = distance_m <= self.obstacle_stop_distance
        self._last_obstacle_present = obstacle_now

        now = self.node.get_clock().now()
        if obstacle_now:
            if self._obstacle_since is None:
                self._obstacle_since = now
            self._clear_since = None
        else:
            if self._clear_since is None:
                self._clear_since = now
            self._obstacle_since = None

        # 희소 발행일 때는 중앙값 필터가 이미 튄 값을 걸러주므로 추가 확정
        # 대기를 두지 않는다(대기 1회가 곧 2초 = 44cm 추가 주행이라 손해가 크다).
        confirm_sec = 0.0 if self._rear_is_sparse() else self.rear_confirm_sec
        clear_sec = self.rear_clear_sec

        def _held(since, need_sec):
            if since is None:
                return False
            return (now - since).nanoseconds / 1e9 >= need_sec

        is_nav_driven = self.node.state_machine.state in self.NAV_DRIVEN_STATES

        if (not self._obstacle_active
                and _held(self._obstacle_since, confirm_sec)):
            # 확정된 시점에만 정지를 건다. 예전엔 매 콜백마다 무조건 정지를
            # 발행해서, 튄 값 하나에도 Nav2 명령을 10Hz로 덮어쓰고 있었다.
            self.node.publish_cmd_vel(0.0, 0.0)
            if is_nav_driven:
                self._on_obstacle_detected(distance_m)
            return

        if (self._obstacle_active
                and _held(self._clear_since, clear_sec)):
            self._on_obstacle_cleared()
            return

        # 장애물이 확정된 상태가 유지되는 동안에는 계속 정지를 유지한다
        if self._obstacle_active and obstacle_now:
            self.node.publish_cmd_vel(0.0, 0.0)

    def _remember_driving_state(self):
        """정지 사유가 처음 발생한 시점의 주행 상태만 기억한다 (2026-08-19).

        이미 다른 사유로 멈춰 있는 상태(OBSTACLE_STOP/PERSON_WAIT)에서 두 번째
        사유가 발생했을 때 그 정지 상태를 '복귀 대상'으로 덮어쓰면 안 된다.
        """
        if self.node.state_machine.state in self.DRIVING_STATES:
            self._state_before_stop = self.node.state_machine.state

    def _try_resume(self, reason: str):
        """남아 있는 정지 사유가 없을 때만 주행을 재개한다 (2026-08-19)."""
        if self._obstacle_active:
            self.node.get_logger().info(
                f'{reason} 해제됐지만 후방 장애물이 남아 있음 -> OBSTACLE_STOP 유지')
            self.node.state_machine.transition(MissionState.OBSTACLE_STOP)
            return
        if self._person_active:
            self.node.get_logger().info(
                f'{reason} 해제됐지만 전방에 사람이 남아 있음 -> PERSON_WAIT 유지')
            self.node.state_machine.transition(MissionState.PERSON_WAIT)
            return

        if self._state_before_stop is None:
            self.node.get_logger().warn(
                f'{reason} 해제됐지만 복귀할 주행 상태를 알 수 없음 -> 정지 상태 유지')
            return

        self.node.state_machine.transition(self._state_before_stop)
        self._state_before_stop = None
        self.node.resend_current_waypoint()

    def _on_obstacle_detected(self, distance: float):
        self._obstacle_active = True
        self._remember_driving_state()
        # Nav2가 계속 /cmd_vel을 발행해 정지 명령을 덮어쓰는 것을 막기 위해 goal 자체를 취소
        self.node.cancel_current_nav_goal()
        self.node.publish_cmd_vel(0.0, 0.0)
        self.node.get_logger().warn(
            f'후방 장애물 감지 ({distance:.2f}m <= {self.obstacle_stop_distance}m) '
            '-> Nav2 goal 취소 후 정지 (SR-021)'
        )
        self.node.state_machine.transition(MissionState.OBSTACLE_STOP)
        self.node.publish_tts('obstacle')

    def _on_obstacle_cleared(self):
        self._obstacle_active = False
        self._obstacle_retry_count += 1

        if self._obstacle_retry_count > self.obstacle_search_max:
            self.node.get_logger().error(
                f'장애물 재시도 {self.obstacle_search_max}회 초과 -> 정지 상태 유지, 관리자 알림'
            )
            self.node.publish_cmd_vel(0.0, 0.0)
            self.node.state_machine.transition(MissionState.ERROR)
            return

        self.node.get_logger().info(
            f'후방 장애물 해제 ({self._obstacle_retry_count}/{self.obstacle_search_max})'
        )
        self._try_resume('후방 장애물')

    def handle_person_detected(self, msg: Bool):
        person_present = msg.data
        self._last_person_present = person_present

        # 장애물과 동일하게 정지 명령은 상태와 무관하게 항상 적용(안전 최우선).
        # DOCKING 중에는 aruco_node가 애초에 /person_detected를 발행하지 않으므로
        # (사람감지 스킵 최적화, 2026-08-06) 이 콜백 자체가 호출되지 않는다.
        if person_present:
            self.node.publish_cmd_vel(0.0, 0.0)

        is_nav_driven = self.node.state_machine.state in self.NAV_DRIVEN_STATES

        if person_present and not self._person_active and is_nav_driven:
            self._on_person_detected()
        elif not person_present and self._person_active:
            self._on_person_cleared()

    def _on_person_detected(self):
        self._person_active = True
        self._remember_driving_state()
        # 장애물과 동일한 이유로 goal 자체를 취소(Nav2가 계속 /cmd_vel을 발행해
        # 정지 명령을 덮어쓰는 것을 막기 위함).
        self.node.cancel_current_nav_goal()
        self.node.publish_cmd_vel(0.0, 0.0)
        self.node.get_logger().warn('전방에 사람 감지 -> Nav2 goal 취소 후 정지 대기')
        self.node.state_machine.transition(MissionState.PERSON_WAIT)
        self.node.publish_tts('person_wait')

    def _on_person_cleared(self):
        self._person_active = False
        self._person_retry_count += 1

        if self._person_retry_count > self.person_wait_retry_max:
            self.node.get_logger().error(
                f'사람 감지-해제 반복 {self.person_wait_retry_max}회 초과 '
                '-> 정지 상태 유지, 관리자 알림'
            )
            self.node.publish_cmd_vel(0.0, 0.0)
            self.node.state_machine.transition(MissionState.ERROR)
            return

        self.node.get_logger().info(
            f'전방 사람 감지 해제 '
            f'({self._person_retry_count}/{self.person_wait_retry_max})'
        )
        self._try_resume('전방 사람')

    def handle_button_event(self, msg: String):
        if msg.data == 'long':
            self.node.get_logger().warn('긴급정지 버튼(길게 누름) 입력 -> EMERGENCY_STOP')
            self.node.publish_cmd_vel(0.0, 0.0)
            self.node.state_machine.transition(MissionState.EMERGENCY_STOP)
        elif msg.data == 'short':
            self.node.get_logger().info('물리 버튼(짧게 누름) 입력 감지')

    def check_battery(self, voltage: float):
        # 2026-08-19: OpenCR이 배터리 미장착/USB 급전 상태에서 0.0(또는 NaN)을
        # 발행하면 0%로 환산되어 주행 중 저전압 복귀가 오발동한다 - 무효값은 버린다.
        if not math.isfinite(voltage) or voltage < self.battery_voltage_min_valid:
            self.node.get_logger().warn(
                f'배터리 전압 {voltage}V는 유효 범위를 벗어남 '
                f'(< {self.battery_voltage_min_valid}V) - 무시함',
                throttle_duration_sec=10.0)
            return

        if self.battery_voltage_filtered is None:
            self.battery_voltage_filtered = voltage
        else:
            a = self.battery_filter_alpha
            self.battery_voltage_filtered = (
                a * voltage + (1.0 - a) * self.battery_voltage_filtered)
        voltage = self.battery_voltage_filtered

        span = self.battery_voltage_full - self.battery_voltage_empty
        percent = (voltage - self.battery_voltage_empty) / span * 100.0
        percent = max(0.0, min(100.0, percent))
        self.last_battery_percent = percent

        active_states = (
            MissionState.GUIDING, MissionState.ARRIVED, MissionState.PERSON_WAIT,
        )
        is_active = self.node.state_machine.state in active_states
        if percent <= self.battery_return_threshold and is_active:
            if not self.battery_return_triggered:
                self.node.get_logger().warn(
                    f'배터리 {percent:.1f}% <= {self.battery_return_threshold}% '
                    '-> 임무 중단, 복귀 우선 (SR-022)'
                )
                self.battery_return_triggered = True
                self.node.state_machine.transition(MissionState.RETURNING)
                self.node.start_return()
        elif percent > self.battery_return_threshold:
            self.battery_return_triggered = False


class MissionStateMachine:
    """SDD-001 3장 상태 머신 본체."""

    def __init__(self, node: Node):
        self.node = node
        self.state = MissionState.SYSTEM_INIT

    def transition(self, new_state: MissionState):
        self.node.get_logger().info(
            f'[state] {self.state.name} -> {new_state.name}'
        )
        self.node.cancel_hold_timer()
        self.state = new_state
        self.node.publish_state_code(new_state)
        self.node.publish_mission_state(new_state)

        # ERROR/EMERGENCY_STOP/PAUSED는 어느 경로로 들어왔든 문구가 같아서 여기서
        # 일괄 처리. READY/GUIDING/ARRIVED 등 나머지는 "왜 이 상태가 됐는지"에 따라
        # 문구가 달라져야 해서(예: READY도 최초 기동 완료 vs 취소 vs 도킹완료가
        # 서로 다른 문구) 각 호출부에서 개별적으로 publish_tts를 호출한다.
        tts_key = self.node.STATE_TTS_MAP.get(new_state)
        if tts_key:
            self.node.publish_tts(tts_key)

        if new_state == MissionState.DOCKING:
            self.node.docking_controller.start()
        else:
            self.node.docking_controller.stop()

        if new_state == MissionState.ARRIVED:
            self.node.start_hold_timer(self.node.arrival_wait, self.node.on_arrival_timeout)
        elif new_state == MissionState.DOCKED:
            self.node.start_hold_timer(self.node.docked_wait, self.node.on_docked_timeout)


class ServiceManagerNode(Node):
    """DDS-001 2.1 service_manager 노드 골격."""

    STATE_CODE_MAP = {
        MissionState.SYSTEM_INIT: 0x01,
        MissionState.LOCALIZING: 0x01,
        MissionState.READY: 0x02,
        MissionState.GUIDING: 0x03,
        MissionState.OBSTACLE_STOP: 0x05,
        MissionState.ARRIVED: 0x04,
        MissionState.PERSON_WAIT: 0x04,
        MissionState.RETURNING: 0x03,
        MissionState.DOCKING: 0x03,
        MissionState.DOCKED: 0x09,
        MissionState.PAUSED: 0x06,
        MissionState.EMERGENCY_STOP: 0x0A,
        MissionState.ERROR: 0x0A,
        MissionState.SHUTDOWN: 0xFF,
    }

    # 원인과 무관하게 문구가 고정인 상태만 여기 등록 (MissionStateMachine.transition
    # 참고). 나머지 text_key(system_start/ready/guiding_start/arrived/docked_arrive/
    # mission_complete/mission_cancelled)는 발생 지점에서 직접 publish_tts 호출.
    STATE_TTS_MAP = {
        MissionState.ERROR: 'error',
        MissionState.EMERGENCY_STOP: 'emergency_stop',
        MissionState.PAUSED: 'paused',
    }

    def __init__(self):
        super().__init__('service_manager')

        self.declare_parameter('battery_return_threshold', 20)
        self.declare_parameter('obstacle_stop_distance', 0.20)
        # 2026-08-18: 초음파 발행이 0.5Hz -> 10Hz 로 바뀌면서, 튄 값 하나에도
        # 즉시 OBSTACLE_STOP 이 걸려 Nav2 goal 이 취소되는 일이 초당 몇 번씩
        # 발생했다(주행이 엉뚱한 방향으로 가고 벽에 붙는 증상의 원인).
        # 연속 N회 확인을 통과해야만 상태를 바꾼다.
        # 2026-08-19: 횟수 -> 시간 기반. 초음파 실측 발행률이 0.50Hz라 "3회 연속"이
        # 6초를 의미했다(SafetyMonitor.__init__ 주석 참고). 아두이노를 빠른 주기로
        # 바꾸더라도 이 값은 그대로 두면 된다.
        self.declare_parameter('rear_obstacle_confirm_sec', 0.6)
        self.declare_parameter('rear_clear_confirm_sec', 1.5)
        # 초음파 수신 간격이 이 값보다 길면 "희소"로 보고 중앙값 창을 3으로 줄이고
        # 확정 대기를 생략한다 (SafetyMonitor._rear_is_sparse 주석 참고).
        self.declare_parameter('rear_sparse_interval', 0.5)
        self.declare_parameter('obstacle_search_max', 2)
        # 장애물(obstacle_search_max=2)보다 넉넉하게 잡음 - 사람은 장애물과 달리
        # 스스로 비켜줄 것으로 기대되는 일반적인 상황이라 더 여러 번 대기-재개를
        # 허용해도 안전하다고 판단.
        self.declare_parameter('person_wait_retry_max', 10)
        self.declare_parameter('aruco_search_max', 3)
        self.declare_parameter('docking_max_attempts', 3)
        self.declare_parameter('docking_speed_limit', 0.05)
        # URS-001/SDD-001 둘 다 "5초 대기"로 명시된 값 - eyes_board_jul31a.ino의
        # ARRIVED 로컬 타이머와 반드시 일치시킬 것(안 맞으면 보드가 먼저 SELECT 메뉴로
        # 복귀한 뒤 backend가 뒤늦게 RETURNING(S:MOVING)을 보내 화면이 튀는 버그가
        # 생김). 2026-08-06: .ino가 3000ms로 미동기화 상태였던 걸 발견해 5000ms로
        # 맞춤(문서 기준이 맞다고 확인됨) - 이 파라미터를 다시 낮추지 말 것.
        self.declare_parameter('arrival_wait', 5.0)
        self.declare_parameter('docked_wait', 2.0)
        self.declare_parameter('nav_timeout', 60.0)
        self.declare_parameter('xy_goal_tolerance', 0.03)
        self.declare_parameter('nav_retry_max', 3)
        self.declare_parameter('dock_marker_id', 29)
        # 2026-08-19: 도킹 마커 앞에서 최종적으로 멈출 거리(m).
        # DockingController._range_error 주석 참고. 0으로 두면 도크에 충돌한다.
        self.declare_parameter('dock_target_distance', 0.25)
        # 2026-08-18: 360도 탐색 -> 좌 90도 고정 회전으로 변경하며 추가된 값들
        # 2026-08-18 실측: 정확히 90도(1.5708)로 돌면 마커가 화각 경계에 걸려
        # 인식이 안 됐다. 5도 더 돌리면 화각 안으로 확실히 들어온다.
        # (오도메트리 각도 오차 + 카메라 장착 각도 오차가 합쳐진 결과로 보임)
        # 2026-08-19: 0으로 변경. 도킹 지점의 yaw를 마커 정면을 바라보도록 재측정해
        # 놨기 때문에(waypoints.yaml docking yaw=1.4982), 도착 후 추가로 90도를 더
        # 돌면 마커가 오히려 화각을 벗어난다. 0이면 도착 즉시 SETTLE->ALIGN 으로
        # 넘어가 바로 마커 정렬을 시작한다.
        # (도킹 지점 yaw를 다시 마커와 어긋나게 잡으면 이 값을 되살릴 것)
        self.declare_parameter('dock_turn_left_angle', 0.0)
        # 회전 후에도 못 찾으면 이 각도씩 "돌고-멈추고-보고"를 반복한다.
        self.declare_parameter('dock_nudge_angle', 0.1222)       # rad (7도)
        self.declare_parameter('dock_turn_speed', 0.35)          # rad/s
        self.declare_parameter('dock_turn_tolerance', 0.035)     # rad (약 2도)
        self.declare_parameter('dock_settle_time', 1.0)          # s (예비)
        # 정지 시각 + 이 여유 이후에 '촬영된' 프레임만 신뢰한다. 카메라 파이프라인
        # 지연(Pi4에서 수백 ms)만큼 잡아줘야 회전 중 찍힌 낡은 값에 안 속는다.
        self.declare_parameter('dock_stale_margin', 0.3)         # s
        self.declare_parameter('dock_k_yaw', 0.5)                # 반대로 돌면 -0.5
        self.declare_parameter('dock_use_yaw_term', True)
        self.declare_parameter('docking_waypoint_id', 'docking')
        self.declare_parameter('startup_ready_delay', 3.0)
        # 2026-08-19: `--ros-args -r`의 토픽 리맵 규칙은 액션 이름에 적용되지 않는다
        # (실측 확인). 실기체를 안 움직이고 상태머신만 시험하려면 이 파라미터로
        # 가짜 액션 서버를 가리켜야 한다.
        self.declare_parameter('nav_action_name', '/navigate_to_pose')

        self.nav_timeout = self.get_parameter('nav_timeout').value
        self.nav_retry_max = self.get_parameter('nav_retry_max').value
        self.nav_retry_count = 0
        self.startup_ready_delay = self.get_parameter('startup_ready_delay').value
        self.arrival_wait = self.get_parameter('arrival_wait').value
        self.docked_wait = self.get_parameter('docked_wait').value
        self._hold_timer = None

        self.state_machine = MissionStateMachine(self)
        self.docking_controller = DockingController(self)
        self.waypoint_sequencer = WaypointSequencer(self)
        self.safety_monitor = SafetyMonitor(self)

        self.waypoint_sequencer.load_waypoints()

        self.state_code_pub = self.create_publisher(UInt8, '/state_code', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.battery_percent_pub = self.create_publisher(Float32, '/battery_percent', 10)
        self.mission_state_pub = self.create_publisher(String, '/mission_state', 10)
        self.current_waypoint_pub = self.create_publisher(String, '/current_waypoint_id', 10)
        self.tts_request_pub = self.create_publisher(TtsRequest, '/tts_request', 10)

        self.cancel_mission_srv = self.create_service(
            Trigger, '/cancel_mission', self.on_cancel_mission
        )
        self.start_mission_srv = self.create_service(
            StartMission, '/start_mission', self.on_start_mission
        )

        self.aruco_marker_sub = self.create_subscription(
            ArucoMarker, '/aruco_marker', self.on_aruco_marker, 10
        )
        # 2026-08-18: 도킹 좌회전 90도를 정확히 돌기 위해 오도메트리가 필요하다.
        # 시간 기반(초 x 각속도)으로 돌면 배터리 전압/바닥 마찰에 따라 실제 각도가
        # 10도 이상 어긋나서, 회전 후 마커가 화각 밖으로 나가는 일이 생긴다.
        self.odom_yaw = None
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.on_odom, 10
        )
        # serial_bridge_node(com_sensors)가 BEST_EFFORT로 발행하므로 QoS를 맞춰야
        # 실제로 메시지가 전달됨(기본 RELIABLE로 구독하면 QoS 불일치로 조용히 안 옴 -
        # 오늘 세션에서 여러 번 겪은 것과 같은 패턴).
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.rear_ir_sub = self.create_subscription(
            Float32, '/com_sensors/rear_distance', self.on_rear_ir, sensor_qos
        )
        self.button_event_sub = self.create_subscription(
            String, '/button_event', self.on_button_event, 10
        )
        self.person_detected_sub = self.create_subscription(
            Bool, '/person_detected', self.on_person_detected, 10
        )
        # 센서보드의 배터리분배회로는 후방 초음파 교체 과정에서 제거됨(2026-08-04, 의도적
        # 간소화). 대신 OpenCR(turtlebot3_node)이 이미 발행하는 /battery_state.voltage를
        # 그대로 사용 - 로봇 배터리팩(3S LiPo, 12.6V full / 9.0V empty) 실측값이라
        # battery_voltage_full/empty 캘리브레이션과도 그대로 맞는다.
        self.battery_state_sub = self.create_subscription(
            BatteryState, '/battery_state', self.on_battery_state, 10
        )

        self.nav_action_name = self.get_parameter('nav_action_name').value
        self.nav_action_client = ActionClient(self, NavigateToPose, self.nav_action_name)
        self.get_logger().info(f'Nav2 액션 클라이언트: {self.nav_action_name}')
        self._nav_goal_handle = None
        self._nav_goal_generation = 0
        self._nav_timeout_timer = None
        # 2026-08-19: goal을 보낸 뒤 accept 응답(on_goal_response)이 도착하기 전
        # 구간에서 취소가 결정된 경우를 기록한다. 이 플래그가 없으면 그 창에 들어온
        # 취소가 통째로 유실된다(아래 cancel_current_nav_goal/on_goal_response 참고).
        self._nav_cancel_requested = False

        self.state_timer = self.create_timer(0.2, self.on_state_timer)
        self.docking_timer = self.create_timer(0.05, self.on_docking_timer)
        self.battery_timer = self.create_timer(1.0, self.on_battery_timer)
        # 늦게 켜진 status 노드도 최신 상태를 받을 수 있도록 1초마다 재발행
        self.mission_state_timer = self.create_timer(1.0, self.on_mission_state_timer)

        self.get_logger().info('service_manager node started (skeleton)')
        self.publish_tts('system_start')

        # SYSTEM_INIT -> LOCALIZING -> READY (SR-031: 30초 이내 active).
        # 실제 AMCL 수렴 판정은 미구현이라, 정해진 대기시간 후 READY로 전이한다.
        self.state_machine.transition(MissionState.LOCALIZING)
        self._ready_timer = self.create_timer(self.startup_ready_delay, self._on_ready_timer)

    def _on_ready_timer(self):
        self.destroy_timer(self._ready_timer)
        self._ready_timer = None
        if self.state_machine.state == MissionState.LOCALIZING:
            self.state_machine.transition(MissionState.READY)
            self.publish_tts('ready')

    def start_hold_timer(self, duration, callback):
        self.cancel_hold_timer()
        self._hold_timer = self.create_timer(duration, callback)

    def cancel_hold_timer(self):
        if self._hold_timer is not None:
            # cancel()만 하면 콜백만 멈추고 타이머 객체는 노드에 계속 등록된 채로
            # 남는다(누수). destroy_timer()로 완전히 제거해야 한다. 웨이포인트/
            # 재시도/도착·도킹 대기마다 새 타이머가 생성되므로, 장시간 운영하면
            # 죽은 타이머가 계속 쌓여 executor wait-set이 커진다.
            self.destroy_timer(self._hold_timer)
            self._hold_timer = None

    def on_arrival_timeout(self):
        self.cancel_hold_timer()
        if self.state_machine.state == MissionState.ARRIVED:
            self.get_logger().info(f'ARRIVED {self.arrival_wait}s 경과 -> 복귀 시작')
            self.state_machine.transition(MissionState.RETURNING)
            self.start_return()

    def on_docked_timeout(self):
        self.cancel_hold_timer()
        if self.state_machine.state == MissionState.DOCKED:
            self.get_logger().info(f'DOCKED {self.docked_wait}s 경과 -> READY 복귀')
            # docked_wait(2초)만으로는 실제 완충 여부를 보장 못 하므로, 다음
            # check_battery 호출이 저배터리를 새로 감지해 RETURNING을 다시
            # 걸 수 있도록 플래그를 리셋한다. 안 하면 도킹 직후 배터리가 여전히
            # 임계값 이하인 채로 새 임무가 시작돼도 재무장이 안 됐다.
            self.safety_monitor.battery_return_triggered = False
            self.state_machine.transition(MissionState.READY)
            self.publish_tts('mission_complete')

    def start_return(self):
        self.get_logger().info('복귀 경로 설정 -> 도킹 지점으로 이동')
        self.safety_monitor.reset_obstacle_retries()
        self.safety_monitor.reset_person_retries()
        self.nav_retry_count = 0
        self.waypoint_sequencer.build_trajectory_2()
        self.send_next_waypoint()

    def on_start_mission(self, request, response):
        dest = request.destination_id
        if not self.waypoint_sequencer.is_valid_destination(dest):
            response.success = False
            response.message = (
                f'잘못된 목적지 ID: {dest} '
                f'(사용 가능: {self.waypoint_sequencer.destination_ids})'
            )
            self.get_logger().warn(response.message)
            return response

        if self.state_machine.state != MissionState.READY:
            response.success = False
            response.message = f'현재 상태({self.state_machine.state.name})에서는 임무를 시작할 수 없습니다'
            self.get_logger().warn(response.message)
            return response

        # 배터리 복귀 임계값 이하에서는 새 임무를 시작하지 않는다. battery_return_triggered
        # 플래그만 믿으면, 도킹 후 짧은 docked_wait 안에 완충이 안 된 상태로 임무가
        # 시작될 때 재무장 타이밍에 따라 저배터리 상태로 계속 주행할 수 있었다.
        if self.safety_monitor.last_battery_percent <= self.safety_monitor.battery_return_threshold:
            response.success = False
            response.message = (
                f'배터리 부족({self.safety_monitor.last_battery_percent:.0f}% <= '
                f'{self.safety_monitor.battery_return_threshold}%) - 충전 후 이용 가능'
            )
            self.get_logger().warn(response.message)
            return response

        # 2026-08-19: 사람/장애물이 이미 앞에 있는 상태로는 출발하지 않는다. 에지
        # 트리거만 믿으면 "이미 True로 래치된 상태"에서 임무를 시작했을 때 정지가
        # 한 번도 안 걸리는 구멍이 생긴다(실기체에서 사람 앞으로 그냥 출발함).
        blocked = self.safety_monitor.blocking_reason()
        if blocked:
            response.success = False
            response.message = f'{blocked} - 해제 후 다시 시도하세요'
            self.get_logger().warn(response.message)
            return response

        self.destination_id = dest
        self.waypoint_sequencer.build_trajectory_1(dest)
        self.safety_monitor.reset_obstacle_retries()
        self.safety_monitor.reset_person_retries()
        self.state_machine.transition(MissionState.GUIDING)
        self.publish_tts('guiding_start')
        self.get_logger().info(f'start_mission requested: destination={dest}')

        self.send_next_waypoint()

        response.success = True
        response.message = '안내를 시작합니다'
        return response

    def on_cancel_mission(self, request, response):
        # v3: cancel = 관리자 확인 겸용. SDD-001의 'PAUSED/ERROR -> 관리자 확인 -> READY'
        # 절차를 GUI 취소 버튼으로 수행할 수 있게 READY로 직접 복귀한다.
        self.cancel_current_nav_goal()
        self.docking_controller.stop()
        self.publish_cmd_vel(0.0, 0.0)
        # 2026-08-19: 임무 흐름이 끊겼으므로 정지 사유 래치도 같이 푼다. 안 그러면
        # 다음 임무에서 같은 사유가 에지로 안 잡혀 안전정지가 통째로 무력화된다.
        self.safety_monitor.reset_stop_conditions()
        self.state_machine.transition(MissionState.READY)
        self.publish_tts('mission_cancelled')
        self.get_logger().info('cancel_mission requested')
        response.success = True
        response.message = '임무를 취소합니다'
        return response

    def on_odom(self, msg: Odometry):
        self.odom_yaw = _odom_yaw(msg.pose.pose.orientation)

    def on_aruco_marker(self, msg: ArucoMarker):
        if self.state_machine.state == MissionState.DOCKING:
            self.docking_controller.on_aruco_marker(msg)

    def on_docking_timer(self):
        """도킹 회전 제어 전용 20Hz 루프.

        state_timer(0.2초=5Hz)로 회전을 돌리면 한 주기에 4도씩 건너뛰어서
        허용오차 2도를 그냥 지나쳐 버린다. 회전 제어만 따로 빠르게 돌린다.
        """
        if self.state_machine.state == MissionState.DOCKING:
            self.docking_controller.update()

    def on_rear_ir(self, msg: Float32):
        self.safety_monitor.handle_rear_ir(msg)

    def on_person_detected(self, msg: Bool):
        self.safety_monitor.handle_person_detected(msg)

    def on_button_event(self, msg: String):
        self.safety_monitor.handle_button_event(msg)

    def on_battery_state(self, msg: BatteryState):
        self.safety_monitor.check_battery(msg.voltage)

    def on_state_timer(self):
        if self.state_machine.state == MissionState.DOCKING:
            self.docking_controller.check_marker_timeout()

    def on_battery_timer(self):
        self.battery_percent_pub.publish(
            Float32(data=self.safety_monitor.last_battery_percent)
        )

    def send_next_waypoint(self):
        if not self.waypoint_sequencer.has_next():
            self.get_logger().info('trajectory 완료: 더 이상 웨이포인트 없음')
            self.on_trajectory_complete()
            return

        wp = self.waypoint_sequencer.advance()
        wp_id = self.waypoint_sequencer.current_waypoint_id()

        if wp is None:
            self.get_logger().error(f'웨이포인트 정의를 찾을 수 없음: {wp_id}')
            self.state_machine.transition(MissionState.ERROR)
            return

        self.get_logger().info(f'다음 웨이포인트로 이동: {wp_id} ({wp["x"]}, {wp["y"]})')
        self.publish_current_waypoint(wp_id)
        self.nav_retry_count = 0
        self._send_nav_goal(wp)

    def _send_nav_goal(self, wp):
        if not self.nav_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('/navigate_to_pose 액션 서버를 찾을 수 없음 (연동 대기 중)')
            self.state_machine.transition(MissionState.ERROR)
            return

        # 재시도마다 세대를 증가시켜, 이미 대체된(취소되었거나 재시도로 밀려난) 이전
        # goal의 accept/result 콜백이 뒤늦게 도착해도 무시할 수 있게 한다. 2026-08-06
        # 실물 테스트에서 nav_timeout 재시도 3회가 취소 처리 전에 연달아 전송되면서,
        # 이미 ERROR로 전이된 뒤에 예전 goal의 성공 응답이 뒤늦게 도착해 ERROR->DOCKING으로
        # 조용히 넘어가버린 버그가 실측됨 - 세대 검사 + 아래 상태 검사로 방지.
        self._nav_goal_generation += 1
        generation = self._nav_goal_generation
        # 이번엔 의도적으로 새 goal을 보내는 것이므로 이전 취소 의사는 지운다.
        self._nav_cancel_requested = False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(wp['x'])
        goal_msg.pose.pose.position.y = float(wp['y'])
        goal_msg.pose.pose.position.z = 0.0
        # 2026-08-19: waypoints.yaml의 yaw를 실제로 반영한다. 예전엔
        # orientation.w=1.0을 하드코딩해서 yaml에 뭘 적든 항상 map +X 방향으로
        # 도착했다 - 도킹 지점처럼 도착 방향이 중요한 곳의 yaw가 조용히 무시됐다.
        yaw = float(wp.get('yaw', 0.0))
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        send_future = self.nav_action_client.send_goal_async(
            goal_msg, feedback_callback=self.on_nav_feedback
        )
        send_future.add_done_callback(
            lambda future, generation=generation: self.on_goal_response(future, generation)
        )

        self._start_nav_timeout_timer()

    def on_goal_response(self, future, generation):
        # 2026-08-19 실기체 실측한 치명적 경합:
        # send_goal_async() 직후 accept 응답이 오기 전(실측 최대 4.1초!)에 사람/
        # 장애물이 감지되면, cancel_current_nav_goal()을 부를 시점엔 _nav_goal_handle이
        # 아직 None이라 취소도 세대 증가도 일어나지 않는다. 그 결과 상태머신은
        # PERSON_WAIT인데 Nav2는 goal을 그대로 들고 목적지까지 5.6m를 완주했다
        # (안전정지 무력화). 핸들을 받는 이 시점에 취소 의사가 남아 있으면 즉시
        # 취소해야 한다.
        stale = (generation != self._nav_goal_generation)
        if stale or self._nav_cancel_requested:
            try:
                gh = future.result()
            except Exception:
                gh = None
            if gh is not None and gh.accepted:
                reason = '이미 새 목표로 대체됨' if stale else '수락 전에 취소 결정됨'
                self.get_logger().warn(f'nav goal 핸들 수신 즉시 취소 ({reason})')
                gh.cancel_goal_async()
            return

        try:
            goal_handle = future.result()
        except Exception as e:
            # 액션 서버가 도중에 죽거나(bt_navigator 재시작 등) DDS 전송 오류가 나면
            # future가 예외로 끝난다. 안 잡으면 executor 콜백 안에서 그대로 터져
            # rclpy.spin()까지 죽일 수 있다.
            self.get_logger().error(f'Nav2 goal 응답 처리 중 예외: {e}')
            self.state_machine.transition(MissionState.ERROR)
            return
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal이 거부됨')
            self.state_machine.transition(MissionState.ERROR)
            return

        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, generation=generation: self.on_nav_result_callback(future, generation)
        )

    def on_nav_feedback(self, feedback_msg):
        pass

    def on_nav_result_callback(self, future, generation):
        if generation != self._nav_goal_generation:
            self.get_logger().debug('만료된 nav goal 결과 무시 (이미 새 목표로 대체됨)')
            return
        # 2026-08-19: NAV_DRIVEN_STATES에는 정지 상태(PERSON_WAIT/OBSTACLE_STOP)도
        # 들어있으므로 여기서는 실제 주행 상태만 통과시킨다.
        if self.state_machine.state not in self.safety_monitor.DRIVING_STATES:
            self.get_logger().warn(
                f'nav goal 결과가 도착했지만 현재 상태({self.state_machine.state.name})는 '
                '더 이상 주행을 기대하지 않음 - 무시'
            )
            self._cancel_nav_timeout_timer()
            return

        self._cancel_nav_timeout_timer()
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f'Nav2 goal 결과 처리 중 예외: {e}')
            self._handle_nav_failure()
            return
        status = result.status

        if status == 4:
            self.get_logger().info(
                f'웨이포인트 도착: {self.waypoint_sequencer.current_waypoint_id()}'
            )
            self.on_waypoint_arrived()
        else:
            self.get_logger().warn(f'Nav2 goal 실패 (status={status})')
            self._handle_nav_failure()

    def on_waypoint_arrived(self):
        wp_id = self.waypoint_sequencer.current_waypoint_id()

        if wp_id == self.waypoint_sequencer.docking_waypoint_id:
            self.get_logger().info('충전 대기점 도착 -> DOCKING (정밀 도킹 제어 시작)')
            self.state_machine.transition(MissionState.DOCKING)
            return

        if wp_id and wp_id == self.waypoint_sequencer.destination_id:
            self.get_logger().info('목적지 도착 -> ARRIVED')
            self.state_machine.transition(MissionState.ARRIVED)
            self.publish_tts('arrived')
            return

        self.send_next_waypoint()

    def resend_current_waypoint(self):
        wp = self.waypoint_sequencer.waypoints.get(
            self.waypoint_sequencer.current_waypoint_id()
        )
        if wp is None:
            self.get_logger().error('재개할 웨이포인트를 찾을 수 없음')
            self.state_machine.transition(MissionState.ERROR)
            return
        self._send_nav_goal(wp)

    def on_trajectory_complete(self):
        pass

    def _handle_nav_failure(self):
        self.nav_retry_count += 1
        if self.nav_retry_count <= self.nav_retry_max:
            self.get_logger().warn(
                f'웨이포인트 재시도 {self.nav_retry_count}/{self.nav_retry_max}'
            )
            wp = self.waypoint_sequencer.waypoints.get(
                self.waypoint_sequencer.current_waypoint_id()
            )
            self._send_nav_goal(wp)
        elif self.state_machine.state == MissionState.RETURNING:
            self.get_logger().error('복귀 경로 이동 3회 실패 -> ERROR (관리자 개입 필요)')
            self.state_machine.transition(MissionState.ERROR)
        else:
            self.get_logger().error('웨이포인트 이동 3회 실패 -> 임무 실패, 복귀 시도')
            self.state_machine.transition(MissionState.RETURNING)
            self.start_return()

    def _start_nav_timeout_timer(self):
        self._cancel_nav_timeout_timer()
        self._nav_timeout_timer = self.create_timer(
            self.nav_timeout, self._on_nav_timeout
        )

    def _cancel_nav_timeout_timer(self):
        if self._nav_timeout_timer is not None:
            self.destroy_timer(self._nav_timeout_timer)  # cancel()만으론 누수 (위 cancel_hold_timer 참고)
            self._nav_timeout_timer = None

    def _on_nav_timeout(self):
        self.get_logger().warn(f'nav_timeout({self.nav_timeout}s) 초과')
        self.cancel_current_nav_goal()
        self._handle_nav_failure()

    def cancel_current_nav_goal(self):
        self._cancel_nav_timeout_timer()
        # 2026-08-19: 아직 accept 응답을 못 받아 핸들이 없더라도 취소 의사는 반드시
        # 남긴다. on_goal_response가 이 플래그를 보고 핸들을 받자마자 취소한다.
        self._nav_cancel_requested = True
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None
            # 세대를 올려 이 goal의 accept/result 콜백이 뒤늦게 도착해도 무시되게
            # 한다. 안 그러면 장애물/사람 감지로 여기서 취소한 직후 도착하는
            # CANCELED 결과가 "주행 실패"로 오인되어 _handle_nav_failure()가 같은
            # 목표를 재전송해버린다 - PERSON_WAIT 중에도 로봇이 다시 움직이는
            # 버그였다 (2026-08-18 코드 리뷰에서 발견, 재시도 경쟁상태 방지용
            # 세대 검사가 이 취소 경로에는 빠져있었음).
            self._nav_goal_generation += 1

    def publish_state_code(self, state: MissionState):
        code = self.STATE_CODE_MAP.get(state)
        if code is None:
            self.get_logger().warn(f'state_code 매핑 없음: {state.name}')
            return
        msg = UInt8()
        msg.data = code
        self.state_code_pub.publish(msg)

    def on_mission_state_timer(self):
        # 상태 전이가 없어도 주기적으로 현재 상태/경유점을 다시 발행
        self.publish_mission_state(self.state_machine.state)
        self.publish_current_waypoint(self.waypoint_sequencer.current_waypoint_id())

    def publish_mission_state(self, state: MissionState):
        # GUI 대시보드가 사람이 읽을 수 있는 상태 이름을 받도록 문자열로 발행
        msg = String()
        msg.data = state.name
        self.mission_state_pub.publish(msg)

    def publish_current_waypoint(self, wp_id):
        # GUI 대시보드의 '현재 경유점' 표시용. wp_id가 None이면 빈 문자열 발행
        msg = String()
        msg.data = wp_id or ''
        self.current_waypoint_pub.publish(msg)

    def publish_tts(self, text_key: str, sound_effect: str = ''):
        msg = TtsRequest()
        msg.text_key = text_key
        msg.sound_effect = sound_effect
        self.tts_request_pub.publish(msg)

    def publish_cmd_vel(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ServiceManagerNode()
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