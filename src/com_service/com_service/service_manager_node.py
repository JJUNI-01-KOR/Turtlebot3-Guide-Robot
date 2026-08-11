import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import UInt8, String, Float32, Bool
from std_srvs.srv import Trigger
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist
from com_interfaces.srv import StartMission
from com_interfaces.msg import ArucoMarker, TtsRequest
from nav2_msgs.action import NavigateToPose

from ament_index_python.packages import get_package_share_directory

from com_service.mission_state import MissionState


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
    """

    def __init__(self, node: Node):
        self.node = node
        self.docking_speed_limit = node.get_parameter('docking_speed_limit').value
        self.docking_max_attempts = node.get_parameter('docking_max_attempts').value
        self.aruco_search_max = node.get_parameter('aruco_search_max').value
        self.dock_marker_id = node.get_parameter('dock_marker_id').value

        self.lateral_tolerance = 0.02
        self.distance_tolerance = 0.03

        self.attempt_count = 0
        self.search_count = 0
        self.last_marker_stamp = None
        self.active = False

        self.k_lateral = 0.8
        self.k_distance = 0.3

    def start(self):
        self.active = True
        self.attempt_count = 0
        self.search_count = 0
        self.last_marker_stamp = self.node.get_clock().now()
        self.node.get_logger().info('DockingController 시작: ArUco 마커 탐색 대기')

    def stop(self):
        self.active = False
        self.node.publish_cmd_vel(0.0, 0.0)

    def on_aruco_marker(self, msg: ArucoMarker):
        if not self.active:
            return
        if msg.marker_id != self.dock_marker_id:
            return

        self.last_marker_stamp = self.node.get_clock().now()
        self.search_count = 0

        if self._is_aligned(msg):
            self._on_dock_success()
            return

        lin, ang = self.compute_docking_velocity(msg)
        self.node.publish_cmd_vel(lin, ang)

    def compute_docking_velocity(self, marker: ArucoMarker):
        lateral = marker.lateral_error
        distance = marker.distance_error

        angular_z = -self.k_lateral * lateral
        linear_x = self.k_distance * distance

        linear_x = max(-self.docking_speed_limit, min(self.docking_speed_limit, linear_x))
        angular_z = max(-0.3, min(0.3, angular_z))

        return linear_x, angular_z

    def _is_aligned(self, marker: ArucoMarker) -> bool:
        return (abs(marker.lateral_error) <= self.lateral_tolerance
                and abs(marker.distance_error) <= self.distance_tolerance)

    def _on_dock_success(self):
        self.active = False
        self.node.publish_cmd_vel(0.0, 0.0)
        self.node.get_logger().info('정밀 도킹 완료 (정렬 오차 허용범위 이내)')
        self.node.state_machine.transition(MissionState.DOCKED)
        self.node.publish_tts('docked_arrive')

    def check_marker_timeout(self):
        if not self.active or self.last_marker_stamp is None:
            return

        elapsed = (self.node.get_clock().now() - self.last_marker_stamp).nanoseconds / 1e9
        if elapsed < 5.0:
            return

        self.search_count += 1
        if self.search_count > self.aruco_search_max:
            self.node.get_logger().error('ArUco 마커 재탐색 3회 실패 -> 정지 후 관리자 알림')
            self.active = False
            self.node.publish_cmd_vel(0.0, 0.0)
            self.node.state_machine.transition(MissionState.ERROR)
            return

        self.attempt_count += 1
        if self.attempt_count > self.docking_max_attempts:
            self.node.get_logger().error('도킹 시도 3회 실패 -> 정지 상태 유지, 관리자 알림 (SR-011)')
            self.active = False
            self.node.publish_cmd_vel(0.0, 0.0)
            self.node.state_machine.transition(MissionState.ERROR)
            return

        self.node.get_logger().warn(
            f'ArUco 마커 미검출 {elapsed:.1f}s -> 15도 회전 재탐색 '
            f'({self.search_count}/{self.aruco_search_max})'
        )
        self.node.publish_cmd_vel(0.0, 0.26)
        self.last_marker_stamp = self.node.get_clock().now()


class SafetyMonitor:
    """긴급정지/장애물/배터리 등 안전 이벤트 종합 판단 (SR-021~SR-026)."""

    def __init__(self, node: Node):
        self.node = node
        self.obstacle_stop_distance = node.get_parameter('obstacle_stop_distance').value
        self.obstacle_search_max = node.get_parameter('obstacle_search_max').value
        self.battery_return_threshold = node.get_parameter('battery_return_threshold').value

        self.battery_voltage_full = 12.6
        self.battery_voltage_empty = 9.0
        self.last_battery_percent = 100.0
        self.battery_return_triggered = False

        self._obstacle_active = False
        self._obstacle_retry_count = 0
        self._state_before_obstacle = None

        self.person_wait_retry_max = node.get_parameter('person_wait_retry_max').value
        self._person_active = False
        self._person_retry_count = 0
        self._state_before_person = None

    def reset_obstacle_retries(self):
        self._obstacle_retry_count = 0

    def reset_person_retries(self):
        self._person_retry_count = 0

    NAV_DRIVEN_STATES = (
        MissionState.GUIDING, MissionState.RETURNING, MissionState.PERSON_WAIT,
    )

    def handle_rear_ir(self, msg: Float32):
        # 실제 후방 센서는 HC-SR04 초음파(/com_sensors/rear_distance, cm 단위) -
        # 예전엔 /rear_ir(Range, m)를 구독하도록 돼 있었는데 그 토픽을 발행하는
        # 코드가 프로젝트 어디에도 없어서 OBSTACLE_STOP이 한 번도 실제로 동작한
        # 적이 없었음(2026-08-06 문서 검토 중 발견, 실물 주행 여러 번 했는데도
        # 몰랐던 버그). 아두이노가 echo 타임아웃(약 5m 이상 미검출) 시 -1.0을
        # 보내는데 이건 "장애물 없음"으로 처리해야 함(그냥 비교하면 음수라 항상
        # 장애물로 오탐).
        distance_cm = msg.data
        if distance_cm < 0:
            obstacle_present = False
            distance_m = float('inf')
        else:
            distance_m = distance_cm / 100.0
            obstacle_present = distance_m <= self.obstacle_stop_distance

        # 즉시 정지는 상태와 무관하게 항상 적용 (안전 최우선)
        if obstacle_present:
            self.node.publish_cmd_vel(0.0, 0.0)

        is_nav_driven = self.node.state_machine.state in self.NAV_DRIVEN_STATES

        if obstacle_present and not self._obstacle_active and is_nav_driven:
            self._on_obstacle_detected(distance_m)
        elif not obstacle_present and self._obstacle_active:
            self._on_obstacle_cleared()

    def _on_obstacle_detected(self, distance: float):
        self._obstacle_active = True
        self._state_before_obstacle = self.node.state_machine.state
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
            f'후방 장애물 해제 -> 이동 재개 ({self._obstacle_retry_count}/{self.obstacle_search_max})'
        )
        self.node.state_machine.transition(self._state_before_obstacle)
        self.node.resend_current_waypoint()

    def handle_person_detected(self, msg: Bool):
        person_present = msg.data

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
        self._state_before_person = self.node.state_machine.state
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
            f'전방 사람 감지 해제 -> 이동 재개 '
            f'({self._person_retry_count}/{self.person_wait_retry_max})'
        )
        self.node.state_machine.transition(self._state_before_person)
        self.node.resend_current_waypoint()

    def handle_button_event(self, msg: String):
        if msg.data == 'long':
            self.node.get_logger().warn('긴급정지 버튼(길게 누름) 입력 -> EMERGENCY_STOP')
            self.node.publish_cmd_vel(0.0, 0.0)
            self.node.state_machine.transition(MissionState.EMERGENCY_STOP)
        elif msg.data == 'short':
            self.node.get_logger().info('물리 버튼(짧게 누름) 입력 감지')

    def check_battery(self, voltage: float):
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
        self.declare_parameter('docking_waypoint_id', 'docking')
        self.declare_parameter('startup_ready_delay', 3.0)

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

        self.nav_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._nav_goal_handle = None
        self._nav_goal_generation = 0
        self._nav_timeout_timer = None

        self.state_timer = self.create_timer(0.2, self.on_state_timer)
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
        self._ready_timer.cancel()
        self._ready_timer = None
        if self.state_machine.state == MissionState.LOCALIZING:
            self.state_machine.transition(MissionState.READY)
            self.publish_tts('ready')

    def start_hold_timer(self, duration, callback):
        self.cancel_hold_timer()
        self._hold_timer = self.create_timer(duration, callback)

    def cancel_hold_timer(self):
        if self._hold_timer is not None:
            self._hold_timer.cancel()
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
        self.state_machine.transition(MissionState.READY)
        self.publish_tts('mission_cancelled')
        self.get_logger().info('cancel_mission requested')
        response.success = True
        response.message = '임무를 취소합니다'
        return response

    def on_aruco_marker(self, msg: ArucoMarker):
        if self.state_machine.state == MissionState.DOCKING:
            self.docking_controller.on_aruco_marker(msg)

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

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(wp['x'])
        goal_msg.pose.pose.position.y = float(wp['y'])
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        send_future = self.nav_action_client.send_goal_async(
            goal_msg, feedback_callback=self.on_nav_feedback
        )
        send_future.add_done_callback(
            lambda future, generation=generation: self.on_goal_response(future, generation)
        )

        self._start_nav_timeout_timer()

    def on_goal_response(self, future, generation):
        if generation != self._nav_goal_generation:
            self.get_logger().debug('만료된 nav goal 응답 무시 (이미 새 목표로 대체됨)')
            return

        goal_handle = future.result()
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
        if self.state_machine.state not in self.safety_monitor.NAV_DRIVEN_STATES:
            self.get_logger().warn(
                f'nav goal 결과가 도착했지만 현재 상태({self.state_machine.state.name})는 '
                '더 이상 주행을 기대하지 않음 - 무시'
            )
            return

        self._cancel_nav_timeout_timer()
        result = future.result()
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
            self.get_logger().info('목적지 도착 -> ARRIVED (ArUco 검증은 8/3~ 예정)')
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
            self._nav_timeout_timer.cancel()
            self._nav_timeout_timer = None

    def _on_nav_timeout(self):
        self.get_logger().warn(f'nav_timeout({self.nav_timeout}s) 초과')
        self.cancel_current_nav_goal()
        self._handle_nav_failure()

    def cancel_current_nav_goal(self):
        self._cancel_nav_timeout_timer()
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None

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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
