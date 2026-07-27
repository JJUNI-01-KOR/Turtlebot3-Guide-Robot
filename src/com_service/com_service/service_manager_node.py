import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from std_msgs.msg import UInt8
from std_srvs.srv import Trigger
from com_interfaces.srv import StartMission
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

from ament_index_python.packages import get_package_share_directory

from com_service.mission_state import MissionState


class WaypointSequencer:
    """Trajectory1/2 웨이포인트 순차 제어 (SR-006)"""

    def __init__(self, node: Node):
        self.node = node
        self.waypoints = {}
        self.trajectory_1_ids = []
        self.trajectory_2_ids = []
        self.current_trajectory = []
        self.current_index = -1

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

        self.trajectory_1_ids = data.get('trajectory_1', {}).get('waypoint_ids', [])
        self.trajectory_2_ids = data.get('trajectory_2', {}).get('waypoint_ids', [])

        self.node.get_logger().info(
            f'waypoints loaded: {len(self.waypoints)}개, '
            f'trajectory1={self.trajectory_1_ids}, trajectory2={self.trajectory_2_ids}'
        )
        return True

    def build_trajectory_1(self, destination_id: str):
        dest_key = f'dest_{destination_id}'
        self.current_trajectory = self.trajectory_1_ids + [dest_key]
        self.current_index = -1

    def build_trajectory_2(self, destination_id: str):
        dest_key = f'dest_{destination_id}'
        self.current_trajectory = [dest_key] + self.trajectory_2_ids
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
    """ArUco 기반 정밀 도킹 제어 (SR-008~SR-011, 핵심 신규 기술)
    TODO(7/30~7/31): /aruco_marker 구독, lateral_error/distance_error 기반
                      저속(<=0.05m/s) /cmd_vel 계산
    """
    def __init__(self, node: Node):
        self.node = node
        self.docking_speed_limit = 0.05
        self.docking_max_attempts = 3
        self.attempt_count = 0

    def compute_docking_velocity(self, marker_pose):
        raise NotImplementedError


class SafetyMonitor:
    """긴급정지/장애물/배터리 등 안전 이벤트 종합 판단 (SR-021~SR-026)
    TODO(7/30, 8/3): /rear_ir, /button_event, 배터리 체크 로직
    """
    def __init__(self, node: Node):
        self.node = node
        self.obstacle_stop_distance = 0.20
        self.battery_return_threshold = 20

    def check_battery(self):
        raise NotImplementedError

    def handle_obstacle(self, msg):
        raise NotImplementedError


class MissionStateMachine:
    """SDD-001 3장 상태 머신 본체"""
    def __init__(self, node: Node):
        self.node = node
        self.state = MissionState.SYSTEM_INIT

    def transition(self, new_state: MissionState):
        self.node.get_logger().info(
            f'[state] {self.state.name} -> {new_state.name}'
        )
        self.state = new_state
        self.node.publish_state_code(new_state)


class ServiceManagerNode(Node):
    """DDS-001 2.1 service_manager 노드 골격"""

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

        self.nav_timeout = self.get_parameter('nav_timeout').value
        self.nav_retry_max = self.get_parameter('nav_retry_max').value
        self.nav_retry_count = 0

        self.state_machine = MissionStateMachine(self)
        self.docking_controller = DockingController(self)
        self.waypoint_sequencer = WaypointSequencer(self)
        self.safety_monitor = SafetyMonitor(self)

        self.waypoint_sequencer.load_waypoints()

        self.state_code_pub = self.create_publisher(UInt8, '/state_code', 10)

        self.cancel_mission_srv = self.create_service(
            Trigger, '/cancel_mission', self.on_cancel_mission
        )
        self.start_mission_srv = self.create_service(
            StartMission, '/start_mission', self.on_start_mission
        )

        self.nav_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._nav_goal_handle = None
        self._nav_timeout_timer = None

        self.state_timer = self.create_timer(0.2, self.on_state_timer)
        self.battery_timer = self.create_timer(1.0, self.on_battery_timer)

        self.get_logger().info('service_manager node started (skeleton)')

    def on_start_mission(self, request, response):
        dest = request.destination_id
        if dest not in ('A', 'B', 'C'):
            response.success = False
            response.message = f'잘못된 목적지 ID: {dest}'
            self.get_logger().warn(response.message)
            return response

        if self.state_machine.state != MissionState.READY:
            response.success = False
            response.message = f'현재 상태({self.state_machine.state.name})에서는 임무를 시작할 수 없습니다'
            self.get_logger().warn(response.message)
            return response

        self.destination_id = dest
        self.waypoint_sequencer.build_trajectory_1(dest)
        self.state_machine.transition(MissionState.GUIDING)
        self.get_logger().info(f'start_mission requested: destination={dest}')

        self.send_next_waypoint()

        response.success = True
        response.message = '안내를 시작합니다'
        return response

    def on_cancel_mission(self, request, response):
        self.cancel_current_nav_goal()
        self.state_machine.transition(MissionState.PAUSED)
        self.get_logger().info('cancel_mission requested')
        response.success = True
        response.message = '임무를 취소합니다'
        return response

    def on_state_timer(self):
        pass

    def on_battery_timer(self):
        pass

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
        if wp_id and wp_id.startswith('dest_'):
            self.get_logger().info('목적지 도착 -> ARRIVED (ArUco 검증은 7/30~ 예정)')
            self.state_machine.transition(MissionState.ARRIVED)
            return

        if wp_id == 'dock_point':
            self.get_logger().info('충전 대기점 도착 -> DOCKING (정밀 도킹은 7/30~ 예정)')
            self.state_machine.transition(MissionState.DOCKING)
            return

        self.send_next_waypoint()

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
