#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
com_service/auto_dock_node.py
 
충전 스테이션 자동 도킹 노드 (7단계 상태 머신).  [2026-08-18]
 
  1. NAV_TO_DOCK  : Nav2로 도킹 지점까지 이동
  2. TURN_LEFT_90 : 왼쪽으로 90도 회전
  3. FIND_MARKER  : 제자리에서 ArUco 마커(ID 29) 확인
  4. APPROACH     : 마커 쪽으로 천천히 이동 (정렬 + 접근 동시)
  5. TURN_180     : 최소거리 도달 후 180도 회전 (카메라가 마커를 등지게)
  6. REVERSE      : 후방 초음파로 벽까지 target_gap 남을 때까지 후진
  7. ANNOUNCE     : /tts_request 로 'docked_arrive' 음성 재생
 
════════ 이 프로젝트의 실제 인터페이스에 맞춘 부분 ════════
 
[1] ArucoMarker.msg 에는 yaw_error 필드가 없다.
    대신 pose.orientation 에 solvePnP 결과(마커 자세)가 그대로 들어있으므로
    여기서 쿼터니언 -> 카메라 y축(수직축) 회전각을 직접 뽑아 쓴다.
    com_interfaces 를 고치지 않아도 되고, 재빌드 연쇄도 없다.
 
[2] aruco 노드는 미검출일 때 marker_id = -1 을 계속 발행한다 (5Hz).
    "토픽이 안 오면 미검출"이 아니라 "marker_id 가 -1이면 미검출"이다.
    ID 필터(!= target_marker_id)가 -1도 같이 걸러주므로 타임스탬프 방식이
    그대로 성립하지만, detection_rate 가 5Hz라 marker_timeout 은 최소 1.0초는
    줘야 한다 (0.5초면 프레임 2~3개라 한 번만 놓쳐도 '미검출'로 오판).
 
[3] 후방 초음파는 /com_sensors/rear_distance (std_msgs/Float32, 단위 cm).
    QoS 가 BEST_EFFORT 이므로 구독도 BEST_EFFORT 여야 한다.
    RELIABLE 로 구독하면 QoS 불일치로 콜백이 "조용히" 한 번도 안 불린다.
 
[4] 음성은 std_msgs/String 이 아니라 com_interfaces/TtsRequest 이고,
    토픽은 /tts_request, 사전 생성된 wav 파일명이 곧 text_key 다.
    audio/docked_arrive.wav 가 이미 있으므로 text_key='docked_arrive'.
 
[5] aruco 노드는 /mission_state 가 'DOCKING' 일 때만 HOG 사람감지를 건너뛰고
    마커 검출에 CPU를 몰아준다. 이 노드를 단독으로 쓸 때 그 신호가 없으면
    Pi4에서 CPU 경합으로 마커 인식이 밀린다. publish_mission_state 옵션으로
    도킹 중에 'DOCKING' 을 발행한다. (service_manager 와 동시 사용 금지)
 
════════ /cmd_vel 충돌 주의 ════════
  service_manager_node 의 DockingController 도 DOCKING 상태에서 /cmd_vel 을
  발행한다. 이 노드와 동시에 돌면 서로 명령을 덮어써서 로봇이 떨린다.
  반드시 둘 중 하나만 쓸 것.
"""
 
import math
from collections import deque
from enum import Enum
 
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
 
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32, Bool
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
 
from com_interfaces.msg import ArucoMarker, TtsRequest
 
 
CONTROL_HZ = 20.0
DEG = math.pi / 180.0
NO_DETECTION_ID = -1        # aruco 노드가 미검출 시 보내는 값
 
 
class State(Enum):
    IDLE = 'IDLE'
    NAV_TO_DOCK = 'NAV_TO_DOCK'
    TURN_LEFT_90 = 'TURN_LEFT_90'
    FIND_MARKER = 'FIND_MARKER'
    APPROACH = 'APPROACH'
    TURN_180 = 'TURN_180'
    REVERSE = 'REVERSE'
    ANNOUNCE = 'ANNOUNCE'
    DONE = 'DONE'
    FAILED = 'FAILED'
 
 
def odom_yaw_from_quaternion(q) -> float:
    """오도메트리용: z축(수직) 회전각. 지면 위 2륜 로봇이라 yaw만 필요."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)
 
 
def marker_yaw_from_quaternion(q) -> float:
    """마커용: 카메라 좌표계(x=오른쪽, y=아래, z=전방)에서 수직축은 y축이다.
    따라서 "마커가 좌우로 얼마나 비틀려 있나"는 y축 회전각이고,
    회전행렬 R에 대해  yaw = atan2(-R20, R00)  으로 구한다.
 
    쿼터니언에서 바로 계산:
        R00 = 1 - 2(y^2 + z^2)
        R20 = 2(xz - wy)
 
    로봇이 마커 정면에 있으면 0, 비스듬히 보면 부호 있는 각도가 나온다.
    (부호가 실제 방향과 반대면 k_yaw 를 음수로 주면 된다.)
    """
    r00 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    r20 = 2.0 * (q.x * q.z - q.w * q.y)
    return math.atan2(-r20, r00)
 
 
def wrap_pi(angle: float) -> float:
    """각도를 -pi ~ +pi 로 접는다.
    안 하면 359도 -> 1도 넘어갈 때 오차가 -358도로 튀어 로봇이 반대로 홱 돈다."""
    return math.atan2(math.sin(angle), math.cos(angle))
 
 
def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))
 
 
def median(values) -> float:
    s = sorted(values)
    n = len(s)
    if n == 0:
        return float('nan')
    mid = n // 2
    return s[mid] if n % 2 else 0.5 * (s[mid - 1] + s[mid])
 
 
class AutoDockNode(Node):
 
    def __init__(self):
        super().__init__('com_auto_dock_node')
 
        # ═══════ 1단계: Nav2 ═══════
        self.declare_parameter('target_marker_id', 29)
        self.declare_parameter('dock_approach_x', 0.0)
        self.declare_parameter('dock_approach_y', 0.0)
        self.declare_parameter('dock_approach_yaw', 0.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('skip_navigation', True)
 
        # ═══════ 2단계: 좌회전 ═══════
        self.declare_parameter('turn_left_angle', 90.0 * DEG)
        self.declare_parameter('turn_speed', 0.4)
        self.declare_parameter('turn_tolerance', 2.0 * DEG)
 
        # ═══════ 3단계: 마커 확인 ═══════
        # aruco 노드 detection_rate 가 5Hz 이므로 최소 1.0초는 줘야 한다
        self.declare_parameter('marker_timeout', 1.0)
        self.declare_parameter('find_settle_time', 1.5)
        self.declare_parameter('enable_fallback_sweep', True)
        self.declare_parameter('sweep_angle', 25.0 * DEG)
        self.declare_parameter('sweep_speed', 0.25)
 
        # ═══════ 4단계: 접근 (정렬 통합) ═══════
        self.declare_parameter('k_lateral', 0.8)
        self.declare_parameter('k_yaw', 0.5)
        self.declare_parameter('use_yaw_term', True)
        self.declare_parameter('approach_speed', 0.06)
        self.declare_parameter('max_angular', 0.4)
        self.declare_parameter('near_max_angular', 0.2)
        self.declare_parameter('near_distance', 0.4)
        self.declare_parameter('min_dock_distance', 0.25)
 
        # ═══════ 6단계: 초음파 후진 ═══════
        self.declare_parameter('ultrasonic_topic', '/com_sensors/rear_distance')
        self.declare_parameter('rear_obstacle_topic', '/com_sensors/rear_obstacle')
        self.declare_parameter('ultrasonic_offset', 0.0)     # m, 센서면-뒷범퍼 보정
        self.declare_parameter('ultrasonic_timeout', 1.5)    # s
        self.declare_parameter('ultrasonic_min_valid', 0.02)
        self.declare_parameter('ultrasonic_max_valid', 2.0)
        self.declare_parameter('target_gap', 0.03)           # m
        self.declare_parameter('gap_tolerance', 0.005)
        self.declare_parameter('reverse_speed', 0.04)
        self.declare_parameter('reverse_min_speed', 0.015)
        self.declare_parameter('k_reverse', 0.35)
        self.declare_parameter('max_reverse_distance', 0.6)
 
        # ═══════ 7단계: 음성 ═══════
        self.declare_parameter('tts_topic', '/tts_request')
        self.declare_parameter('tts_text_key', 'docked_arrive')
        self.declare_parameter('tts_sound_effect', '')
 
        # ═══════ 기타 ═══════
        self.declare_parameter('publish_mission_state', True)
        self.declare_parameter('timeout_turn_left', 20.0)
        self.declare_parameter('timeout_find', 30.0)
        self.declare_parameter('timeout_approach', 90.0)
        self.declare_parameter('timeout_turn_180', 25.0)
        self.declare_parameter('timeout_reverse', 60.0)
 
        self._load_params()
 
        # ═══════ 통신 ═══════
        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=10)
        # com_sensors 는 BEST_EFFORT 로 발행한다. 여기서 RELIABLE 로 구독하면
        # QoS 불일치로 콜백이 아예 안 불린다 (에러도 안 남 - 제일 찾기 어려운 버그).
        best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST, depth=10)
 
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', reliable)
        self.state_pub = self.create_publisher(String, '/docking_state', reliable)
        self.tts_pub = self.create_publisher(TtsRequest, self.tts_topic, reliable)
        self.mission_pub = self.create_publisher(String, '/mission_state', reliable)
 
        self.create_subscription(ArucoMarker, '/aruco_marker', self.on_marker, reliable)
        self.create_subscription(Odometry, '/odom', self.on_odom, reliable)
        self.create_subscription(Float32, self.ultrasonic_topic,
                                 self.on_rear_distance, best_effort)
        self.create_subscription(Bool, self.rear_obstacle_topic,
                                 self.on_rear_obstacle, best_effort)
        # service_manager의 DockingController와 이 노드는 둘 다 DOCKING 중
        # /cmd_vel을 발행한다(파일 상단 주석: "동시 사용 금지"). srv_start에서
        # service_manager가 READY가 아니면 거부할 수 있도록 상태를 구독해둔다.
        self._external_mission_state = None
        self.create_subscription(String, '/mission_state',
                                 self._on_external_mission_state, reliable)

        self.create_service(Trigger, '/start_docking', self.srv_start)
        self.create_service(Trigger, '/cancel_docking', self.srv_cancel)
 
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
 
        # ═══════ 내부 상태 ═══════
        self.state = State.IDLE
        self.state_enter_time = self.get_clock().now()
 
        self.marker = None
        self.marker_yaw = 0.0
        self.marker_stamp = None
        self.last_seen_distance = None
 
        self.odom_x = self.odom_y = self.odom_yaw = None
 
        self.us_buffer = deque(maxlen=5)
        self.us_stamp = None
        self.rear_obstacle = False
 
        self.nav_goal_handle = None
        self.nav_done = False
        self.nav_success = False
        self._nav_generation = 0

        self.turn_accum = 0.0
        self.turn_prev_yaw = None
        self.turn_target = 0.0
        self.turn_dir = 1.0
 
        self.sweep_phase = 0
        self.sweep_accum = 0.0
        self.sweep_prev_yaw = None
 
        self.reverse_start_x = None
        self.reverse_start_y = None
 
        self.tts_sent = False
 
        self.create_timer(1.0 / CONTROL_HZ, self.control_loop)
 
        self.get_logger().info(
            f'자동 도킹 노드 준비. 마커 ID={self.target_marker_id}, '
            f'초음파={self.ultrasonic_topic}, 목표간격={self.target_gap * 100:.1f}cm, '
            f'음성키={self.tts_text_key}')
 
    # ═══════════════════════════════════════
    def _load_params(self):
        g = lambda n: self.get_parameter(n).value  # noqa: E731
 
        self.target_marker_id = g('target_marker_id')
        self.dock_approach_x = g('dock_approach_x')
        self.dock_approach_y = g('dock_approach_y')
        self.dock_approach_yaw = g('dock_approach_yaw')
        self.map_frame = g('map_frame')
        self.skip_navigation = g('skip_navigation')
 
        self.turn_left_angle = g('turn_left_angle')
        self.turn_speed = g('turn_speed')
        self.turn_tolerance = g('turn_tolerance')
 
        self.marker_timeout = g('marker_timeout')
        self.find_settle_time = g('find_settle_time')
        self.enable_fallback_sweep = g('enable_fallback_sweep')
        self.sweep_angle = g('sweep_angle')
        self.sweep_speed = g('sweep_speed')
 
        self.k_lateral = g('k_lateral')
        self.k_yaw = g('k_yaw')
        self.use_yaw_term = g('use_yaw_term')
        self.approach_speed = g('approach_speed')
        self.max_angular = g('max_angular')
        self.near_max_angular = g('near_max_angular')
        self.near_distance = g('near_distance')
        self.min_dock_distance = g('min_dock_distance')
 
        self.ultrasonic_topic = g('ultrasonic_topic')
        self.rear_obstacle_topic = g('rear_obstacle_topic')
        self.ultrasonic_offset = g('ultrasonic_offset')
        self.ultrasonic_timeout = g('ultrasonic_timeout')
        self.ultrasonic_min_valid = g('ultrasonic_min_valid')
        self.ultrasonic_max_valid = g('ultrasonic_max_valid')
        self.target_gap = g('target_gap')
        self.gap_tolerance = g('gap_tolerance')
        self.reverse_speed = g('reverse_speed')
        self.reverse_min_speed = g('reverse_min_speed')
        self.k_reverse = g('k_reverse')
        self.max_reverse_distance = g('max_reverse_distance')
 
        self.tts_topic = g('tts_topic')
        self.tts_text_key = g('tts_text_key')
        self.tts_sound_effect = g('tts_sound_effect')
 
        self.publish_mission_state = g('publish_mission_state')
        self.timeout = {
            State.TURN_LEFT_90: g('timeout_turn_left'),
            State.FIND_MARKER: g('timeout_find'),
            State.APPROACH: g('timeout_approach'),
            State.TURN_180: g('timeout_turn_180'),
            State.REVERSE: g('timeout_reverse'),
        }
 
    # ═══════ 콜백 ═══════
    def on_marker(self, msg: ArucoMarker):
        # aruco 노드는 미검출 시 marker_id=-1 을 계속 보낸다. 이 조건 하나로
        # '미검출'과 '다른 마커' 둘 다 걸러진다.
        if msg.marker_id != self.target_marker_id:
            return
        self.marker = msg
        # ArucoMarker.msg 에 yaw_error 가 없으므로 pose 에서 직접 뽑는다
        self.marker_yaw = marker_yaw_from_quaternion(msg.pose.pose.orientation)
        self.marker_stamp = self.get_clock().now()
        self.last_seen_distance = msg.distance_error
 
    def on_odom(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_yaw = odom_yaw_from_quaternion(msg.pose.pose.orientation)
 
    def on_rear_distance(self, msg: Float32):
        """아두이노가 cm 로 보내므로 m 로 변환. 미검출은 -1.0 로 온다."""
        value = msg.data / 100.0
        if not math.isfinite(value):
            return
        # 아두이노 readDistanceCm() 은 에코 미수신 시 -1.0 을 반환한다.
        # 그걸 그대로 믿으면 "벽이 뒤에 있다"고 착각하거나 영원히 후진한다.
        if not (self.ultrasonic_min_valid <= value <= self.ultrasonic_max_valid):
            return
        self.us_buffer.append(value)
        self.us_stamp = self.get_clock().now()
 
    def on_rear_obstacle(self, msg: Bool):
        self.rear_obstacle = msg.data

    def _on_external_mission_state(self, msg: String):
        self._external_mission_state = msg.data

    # ═══════ 서비스 ═══════
    def srv_start(self, request, response):
        if self.state not in (State.IDLE, State.DONE, State.FAILED):
            response.success = False
            response.message = f'이미 진행 중 ({self.state.value})'
            return response
        # service_manager가 이미 임무/도킹 중이면 /cmd_vel이 충돌한다(파일 상단
        # 주석 참고). 신호를 아직 못 받았거나(None) READY일 때만 허용한다.
        if self._external_mission_state not in (None, 'READY'):
            response.success = False
            response.message = (
                f'service_manager가 이미 동작 중({self._external_mission_state}) - '
                '동시 사용 불가'
            )
            self.get_logger().warn(response.message)
            return response
        self._enter_state(State.TURN_LEFT_90 if self.skip_navigation
                          else State.NAV_TO_DOCK)
        response.success = True
        response.message = '도킹 시퀀스 시작'
        return response
 
    def srv_cancel(self, request, response):
        self._cancel_navigation()
        self._enter_state(State.IDLE)
        response.success = True
        response.message = '도킹 취소'
        return response
 
    # ═══════ 공용 ═══════
    def _enter_state(self, new_state: State):
        """상태 전환 = 정지 + 해당 상태 변수 초기화. 항상 이 함수로만 바꾼다."""
        self.stop_robot()
        self.state = new_state
        self.state_enter_time = self.get_clock().now()
        self.get_logger().info(f'[상태] -> {new_state.value}')
 
        if new_state == State.TURN_LEFT_90:
            self._begin_turn(self.turn_left_angle, +1.0)
        elif new_state == State.TURN_180:
            self._begin_turn(math.pi, +1.0)
        elif new_state == State.FIND_MARKER:
            self.sweep_phase = 0
            self.sweep_accum = 0.0
            self.sweep_prev_yaw = self.odom_yaw
        elif new_state == State.REVERSE:
            self.reverse_start_x = self.odom_x
            self.reverse_start_y = self.odom_y
        elif new_state == State.ANNOUNCE:
            self.tts_sent = False
 
    def _begin_turn(self, target_rad: float, direction: float):
        self.turn_accum = 0.0
        self.turn_prev_yaw = self.odom_yaw
        self.turn_target = target_rad
        self.turn_dir = direction
 
    def _elapsed(self) -> float:
        return (self.get_clock().now() - self.state_enter_time).nanoseconds / 1e9
 
    def marker_fresh(self) -> bool:
        if self.marker is None or self.marker_stamp is None:
            return False
        age = (self.get_clock().now() - self.marker_stamp).nanoseconds / 1e9
        return age <= self.marker_timeout
 
    def ultrasonic_gap(self):
        """뒷범퍼 ~ 벽 거리(m). 유효하지 않으면 None."""
        if self.us_stamp is None or len(self.us_buffer) < 3:
            return None
        age = (self.get_clock().now() - self.us_stamp).nanoseconds / 1e9
        if age > self.ultrasonic_timeout:
            return None
        # 중앙값 필터: 초음파는 가끔 완전히 튀는 값을 하나씩 섞어 보낸다.
        # 평균은 그 값에 끌려가지만 중앙값은 통째로 무시한다.
        return median(self.us_buffer) - self.ultrasonic_offset
 
    def stop_robot(self):
        self.cmd_pub.publish(Twist())
 
    def drive(self, linear_x: float, angular_z: float):
        t = Twist()
        t.linear.x = float(linear_x)
        t.angular.z = float(angular_z)
        self.cmd_pub.publish(t)
 
    def _steering(self, max_ang: float) -> float:
        """도킹 조향 법칙:  angular_z = -(k_lateral*lateral + k_yaw*marker_yaw)
 
        lateral 항만 있으면(=기존 DockingController) 마커 '위치'로 커브를 그리며
        갈 뿐이라 도착했을 때 마커를 비스듬히 보게 된다. yaw 항이 "마커 정면
        직선 위로 올라타라"는 성분을 더해줘서, 전진하는 동안 두 오차가 같이
        0으로 수렴한다. 정렬 단계를 따로 두지 않아도 되는 이유가 이것이고,
        6단계에서 스테이션에 모서리부터 박지 않으려면 반드시 필요하다."""
        steer = self.k_lateral * self.marker.lateral_error
        if self.use_yaw_term:
            steer += self.k_yaw * self.marker_yaw
        return clamp(-steer, max_ang)
 
    def _fail(self, reason: str):
        self.get_logger().error(f'도킹 실패: {reason}')
        self._cancel_navigation()
        self._enter_state(State.FAILED)
 
    # ═══════ 메인 루프 ═══════
    def control_loop(self):
        self.state_pub.publish(String(data=self.state.value))
 
        # aruco 노드는 /mission_state == 'DOCKING' 일 때 HOG 사람감지를 건너뛴다.
        # Pi4 에서 이게 없으면 CPU 경합으로 마커 인식이 밀린다(실측 이력 있음).
        if self.publish_mission_state and self.state not in (
                State.IDLE, State.DONE, State.FAILED):
            self.mission_pub.publish(String(data='DOCKING'))
 
        if self.state in (State.IDLE, State.DONE, State.FAILED):
            return
 
        if self.odom_yaw is None:
            self.get_logger().warn('/odom 대기 중...', throttle_duration_sec=3.0)
            return
 
        limit = self.timeout.get(self.state)
        if limit is not None and self._elapsed() > limit:
            self._fail(f'{self.state.value} 타임아웃 ({limit}s)')
            return
 
        {
            State.NAV_TO_DOCK: self._st_nav,
            State.TURN_LEFT_90: self._st_turn,
            State.FIND_MARKER: self._st_find,
            State.APPROACH: self._st_approach,
            State.TURN_180: self._st_turn,
            State.REVERSE: self._st_reverse,
            State.ANNOUNCE: self._st_announce,
        }[self.state]()
 
    # ── 1단계 ──
    def _st_nav(self):
        if self.nav_goal_handle is None and not self.nav_done:
            if not self.nav_client.wait_for_server(timeout_sec=0.1):
                self.get_logger().warn('Nav2 액션 서버 대기...',
                                       throttle_duration_sec=3.0)
                return
            self._send_nav_goal()
            return
        if self.nav_done:
            ok = self.nav_success
            self._reset_nav()
            if ok:
                self.get_logger().info('도킹 지점 도착 -> 좌회전')
                self._enter_state(State.TURN_LEFT_90)
            else:
                self._fail('Nav2 주행 실패')
 
    def _send_nav_goal(self):
        goal = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = self.map_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(self.dock_approach_x)
        p.pose.position.y = float(self.dock_approach_y)
        p.pose.orientation.z = math.sin(self.dock_approach_yaw / 2.0)
        p.pose.orientation.w = math.cos(self.dock_approach_yaw / 2.0)
        goal.pose = p
        self.get_logger().info(
            f'Nav2 목표: ({self.dock_approach_x:.2f}, {self.dock_approach_y:.2f})')
        # 세대를 올려, 취소 후 재시작된 이전 goal의 콜백이 뒤늦게 도착해도
        # (아직 도착 안 함=pending 이었을 때 취소하면 서버에는 요청이 여전히
        # 살아있다) 지금 시도의 nav_done/nav_success를 덮어쓰지 못하게 한다.
        self._nav_generation += 1
        generation = self._nav_generation
        self.nav_client.send_goal_async(goal).add_done_callback(
            lambda future, generation=generation: self._on_nav_resp(future, generation))
        self.nav_goal_handle = 'pending'

    def _on_nav_resp(self, future, generation):
        if generation != self._nav_generation:
            self.get_logger().debug('만료된 nav goal 응답 무시 (취소 후 재시작됨)')
            return
        try:
            h = future.result()
        except Exception as e:
            self.get_logger().error(f'Nav2 goal 응답 처리 중 예외: {e}')
            self.nav_done, self.nav_success = True, False
            return
        if not h.accepted:
            self.nav_done, self.nav_success = True, False
            return
        self.nav_goal_handle = h
        h.get_result_async().add_done_callback(
            lambda future, generation=generation: self._on_nav_result(future, generation))

    def _on_nav_result(self, future, generation):
        if generation != self._nav_generation:
            self.get_logger().debug('만료된 nav goal 결과 무시 (취소 후 재시작됨)')
            return
        try:
            self.nav_done = True
            self.nav_success = (future.result().status == 4)   # 4 = SUCCEEDED
        except Exception as e:
            self.get_logger().error(f'Nav2 goal 결과 처리 중 예외: {e}')
            self.nav_success = False

    def _cancel_navigation(self):
        if self.nav_goal_handle not in (None, 'pending'):
            self.nav_goal_handle.cancel_goal_async()
        # 아직 accept/result가 안 온 이전 시도의 콜백을 전부 무효화한다.
        self._nav_generation += 1
        self._reset_nav()
 
    def _reset_nav(self):
        self.nav_goal_handle = None
        self.nav_done = False
        self.nav_success = False
 
    # ── 2·5단계: 제자리 회전 ──
    def _st_turn(self):
        if self.turn_prev_yaw is None:
            self.turn_prev_yaw = self.odom_yaw
 
        # 누적 방식: 매 주기 yaw 차분을 더한다. 시작yaw와 현재yaw를 직접 빼면
        # 180도 부근에서 +pi/-pi 가 뒤집혀 판정이 오락가락한다.
        self.turn_accum += abs(wrap_pi(self.odom_yaw - self.turn_prev_yaw))
        self.turn_prev_yaw = self.odom_yaw
 
        remaining = self.turn_target - self.turn_accum
        if remaining <= self.turn_tolerance:
            self.get_logger().info(
                f'회전 완료 (목표 {self.turn_target / DEG:.0f}도, '
                f'실제 {self.turn_accum / DEG:.1f}도)')
            self._enter_state(State.FIND_MARKER
                              if self.state == State.TURN_LEFT_90
                              else State.REVERSE)
            return
 
        speed = self.turn_speed
        if remaining < 20 * DEG:      # 목표 근처 감속 (관성 오버슈트 방지)
            speed = max(0.12, self.turn_speed * remaining / (20 * DEG))
        self.drive(0.0, self.turn_dir * speed)
 
    # ── 3단계 ──
    def _st_find(self):
        if self.marker_fresh():
            self.get_logger().info(
                f'마커 확인! 거리 {self.marker.distance_error:.2f} m, '
                f'좌우 {self.marker.lateral_error:+.3f} m, '
                f'각도 {self.marker_yaw / DEG:+.1f}도')
            self._enter_state(State.APPROACH)
            return
 
        # 회전 직후엔 모션 블러 + 5Hz 검출 주기 때문에 인식이 늦다. 우선 멈춰서 대기.
        if self._elapsed() < self.find_settle_time:
            self.stop_robot()
            return
 
        if not self.enable_fallback_sweep:
            self.stop_robot()
            return
 
        # 90도 회전엔 오도메트리 오차가 몇 도씩 섞인다. 마커가 화각 경계에
        # 걸치면 못 보므로 좌우로 살짝 흔들어 본다.
        if self.sweep_prev_yaw is None:
            self.sweep_prev_yaw = self.odom_yaw
        self.sweep_accum += abs(wrap_pi(self.odom_yaw - self.sweep_prev_yaw))
        self.sweep_prev_yaw = self.odom_yaw
 
        legs = [(-1.0, self.sweep_angle),
                (+1.0, 2 * self.sweep_angle),
                (-1.0, self.sweep_angle)]
        if self.sweep_phase >= len(legs):
            self._fail('좌회전 후 마커 미발견 (도킹 지점 좌표/회전각 확인)')
            return
 
        direction, target = legs[self.sweep_phase]
        if self.sweep_accum >= target:
            self.sweep_phase += 1
            self.sweep_accum = 0.0
            self.stop_robot()
        else:
            self.drive(0.0, direction * self.sweep_speed)
 
    # ── 4단계: 접근 (정렬 통합) ──
    def _st_approach(self):
        if not self.marker_fresh():
            near = (self.last_seen_distance is not None
                    and self.last_seen_distance <= self.min_dock_distance + 0.10)
            if near:
                self.get_logger().info(
                    f'근접 중 인식 끊김 — 도착 간주 '
                    f'(마지막 {self.last_seen_distance:.3f} m)')
                self._enter_state(State.TURN_180)
            else:
                self.get_logger().warn('접근 중 마커 놓침 — 재확인')
                self._enter_state(State.FIND_MARKER)
            return
 
        distance = self.marker.distance_error
        if distance <= self.min_dock_distance:
            self.get_logger().info(
                f'최소거리 도달 ({distance:.3f} m, 좌우 '
                f'{self.marker.lateral_error:+.3f} m, '
                f'각도 {self.marker_yaw / DEG:+.1f}도) -> 180도 회전')
            self._enter_state(State.TURN_180)
            return
 
        # 가까울수록 각속도 제한. 코앞에서 홱 돌면 마커가 화각 밖으로 튄다.
        max_ang = (self.near_max_angular if distance < self.near_distance
                   else self.max_angular)
        self.drive(self.approach_speed, self._steering(max_ang))
 
    # ── 6단계: 초음파 폐루프 후진 ──
    def _st_reverse(self):
        traveled = math.hypot(self.odom_x - self.reverse_start_x,
                              self.odom_y - self.reverse_start_y)
        # 오도메트리 하드 리밋: 초음파가 거짓말을 해도 이 이상은 절대 안 간다.
        if traveled >= self.max_reverse_distance:
            self.stop_robot()
            self._fail(f'후진 상한 {self.max_reverse_distance:.2f} m 초과 '
                       f'(초음파 이상 의심)')
            return
 
        gap = self.ultrasonic_gap()
        if gap is None:
            # 초음파가 끊기면 절대 계속 후진하면 안 된다. 그대로 벽에 박는다.
            self.stop_robot()
            self.get_logger().warn(
                '초음파 값 없음 — 정지 대기 (센서보드 연결/발행주기 확인)',
                throttle_duration_sec=2.0)
            return
 
        error = gap - self.target_gap
        if error <= self.gap_tolerance:
            self.stop_robot()
            self.get_logger().info(
                f'목표 간격 도달: {gap * 100:.1f} cm (후진 {traveled * 100:.1f} cm)')
            self._enter_state(State.ANNOUNCE)
            return
 
        # 남은 거리에 비례 감속. 등속으로 밀면 정지 지연 때문에 목표를 지나친다.
        speed = max(self.reverse_min_speed,
                    abs(clamp(self.k_reverse * error, self.reverse_speed)))
        self.drive(-speed, 0.0)
 
    # ── 7단계: 음성 ──
    def _st_announce(self):
        self.stop_robot()
        if not self.tts_sent:
            msg = TtsRequest()
            msg.text_key = self.tts_text_key
            msg.sound_effect = self.tts_sound_effect
            self.tts_pub.publish(msg)
            self.tts_sent = True
            self.get_logger().info(f'음성 요청 발행: text_key={self.tts_text_key}')
            return
        # voice_node 는 큐에 넣고 순차 재생하므로 여기서 기다릴 필요는 없다.
        if self._elapsed() > 1.0:
            self.get_logger().info('=== 도킹 완료 ===')
            self._enter_state(State.DONE)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = AutoDockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        # 2026-08-18: SIGINT로 정상 종료될 때 rclpy 시그널 핸들러가 이미 컨텍스트를
        # 닫아둔 상태라, 여기서 또 shutdown하면 RCLError가 터져 exit code 1로 끝난다.
        # PM2는 그걸 '크래시'로 세서 재시작 카운터를 소진시키고 결국 스택 전체를
        # 내려버린다(8/18 시험주행 후 8개 노드 전부 stopped 된 직접 원인).
        if rclpy.ok():
            rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()