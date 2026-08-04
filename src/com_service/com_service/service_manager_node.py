import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import UInt8, String, Float32
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range, BatteryState
from geometry_msgs.msg import Twist
from com_interfaces.srv import StartMission
from com_interfaces.msg import ArucoMarker
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

    def reset_obstacle_retries(self):
        self._obstacle_retry_count = 0

    NAV_DRIVEN_STATES = (
        MissionState.GUIDING, MissionState.RETURNING, MissionState.PERSON_WAIT,
    )

    def handle_rear_ir(self, msg: Range):
        obstacle_present = msg.range <= self.obstacle_stop_distance

        # 즉시 정지는 상태와 무관하게 항상 적용 (안전 최우선)
        if obstacle_present:
            self.node.publish_cmd_vel(0.0, 0.0)

        is_nav_driven = self.node.state_machine.state in self.NAV_DRIVEN_STATES

        if obstacle_present and not self._obstacle_active and is_nav_driven:
            self._on_obstacle_detected(msg.range)
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
        self.state = new_state
        self.node.publish_state_code(new_state)
        self.node.publish_mission_state(new_state)

        if new_state == MissionState.DOCKING:
            self.node.docking_controller.start()
        else:
            self.node.docking_controller.stop()


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

    def __init__(self):
        super().__init__('service_manager')

        self.declare_parameter('battery_return_threshold', 20)
        self.declare_parameter('obstacle_stop_distance', 0.20)
        self.declare_parameter('obstacle_search_max', 2)
        self.declare_parameter('aruco_search_max', 3)
        self.declare_parameter('docking_max_attempts', 3)
        self.declare_parameter('docking_speed_limit', 0.05)
        self.declare_parameter('arrival_wait', 5.0)
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

        self.cancel_mission_srv = self.create_service(
            Trigger, '/cancel_mission', self.on_cancel_mission
        )
        self.start_mission_srv = self.create_service(
            StartMission, '/start_mission', self.on_start_mission
        )

        self.aruco_marker_sub = self.create_subscription(
            ArucoMarker, '/aruco_marker', self.on_aruco_marker, 10
        )
        self.rear_ir_sub = self.create_subscription(
            Range, '/rear_ir', self.on_rear_ir, 10
        )
        self.button_event_sub = self.create_subscription(
            String, '/button_event', self.on_button_event, 10
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
        self._nav_timeout_timer = None

        self.state_timer = self.create_timer(0.2, self.on_state_timer)
        self.battery_timer = self.create_timer(1.0, self.on_battery_timer)
        # 늦게 켜진 status 노드도 최신 상태를 받을 수 있도록 1초마다 재발행
        self.mission_state_timer = self.create_timer(1.0, self.on_mission_state_timer)

        self.get_logger().info('service_manager node started (skeleton)')

        # SYSTEM_INIT -> LOCALIZING -> READY (SR-031: 30초 이내 active).
        # 실제 AMCL 수렴 판정은 미구현이라, 정해진 대기시간 후 READY로 전이한다.
        self.state_machine.transition(MissionState.LOCALIZING)
        self._ready_timer = self.create_timer(self.startup_ready_delay, self._on_ready_timer)

    def _on_ready_timer(self):
        self._ready_timer.cancel()
        self._ready_timer = None
        if self.state_machine.state == MissionState.LOCALIZING:
            self.state_machine.transition(MissionState.READY)

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
        self.state_machine.transition(MissionState.GUIDING)
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
        self.get_logger().info('cancel_mission requested')
        response.success = True
        response.message = '임무를 취소합니다'
        return response

    def on_aruco_marker(self, msg: ArucoMarker):
        if self.state_machine.state == MissionState.DOCKING:
            self.docking_controller.on_aruco_marker(msg)

    def on_rear_ir(self, msg: Range):
        self.safety_monitor.handle_rear_ir(msg)

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
        send_future.add_done_callback(self.on_goal_response)

        self._start_nav_timeout_timer()

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal이 거부됨')
            self.state_machine.transition(MissionState.ERROR)
            return

        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_nav_result_callback)

    def on_nav_feedback(self, feedback_msg):
        pass

    def on_nav_result_callback(self, future):
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
        else:
            self.get_logger().error('웨이포인트 이동 3회 실패 -> 임무 실패, 복귀 시도')
            self.state_machine.transition(MissionState.RETURNING)

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
