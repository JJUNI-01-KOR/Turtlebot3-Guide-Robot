import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger
from com_interfaces.srv import StartMission

from com_service.mission_state import MissionState


class WaypointSequencer:
    """Trajectory1/2 웨이포인트 순차 제어 (SR-006)
    TODO(7/28): trajectory1_waypoints, trajectory2_waypoints 파라미터 로드
    TODO(7/28): Nav2 /navigate_to_pose 액션 클라이언트 연동
    """
    def __init__(self, node: Node):
        self.node = node

    def load_waypoints(self):
        raise NotImplementedError


class DockingController:
    """ArUco 기반 정밀 도킹 제어 (SR-008~SR-011, 핵심 신규 기술)
    TODO(7/30~7/31): /aruco_marker 구독, lateral_error/distance_error 기반
                      저속(<=0.05m/s) /cmd_vel 계산
    """
    def __init__(self, node: Node):
        self.node = node
        self.docking_speed_limit = 0.05  # m/s, DDS-001 2.1 파라미터
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
        self.obstacle_stop_distance = 0.20  # m
        self.battery_return_threshold = 20  # %

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
        # TODO(7/29~): publish_state_code(code) 로 /state_code 발행 (status 노드 연동)


class ServiceManagerNode(Node):
    """DDS-001 2.1 service_manager 노드 골격
    입력 토픽: /amcl_pose, /aruco_marker, /person_detected, /rear_ir, /button_event, /system_error
    출력 토픽: /state_code, /tts_request
    서비스(제공): /start_mission, /cancel_mission
    액션(클라이언트): /navigate_to_pose
    """
    def __init__(self):
        super().__init__('service_manager')

        # 파라미터 선언 (DDS-001 2.1 파라미터 표)
        self.declare_parameter('battery_return_threshold', 20)
        self.declare_parameter('obstacle_stop_distance', 0.20)
        self.declare_parameter('obstacle_search_max', 2)
        self.declare_parameter('aruco_search_max', 3)
        self.declare_parameter('docking_max_attempts', 3)
        self.declare_parameter('docking_speed_limit', 0.05)
        self.declare_parameter('arrival_wait', 5.0)
        self.declare_parameter('nav_timeout', 60.0)

        # 서브 컴포넌트
        self.state_machine = MissionStateMachine(self)
        self.docking_controller = DockingController(self)
        self.waypoint_sequencer = WaypointSequencer(self)
        self.safety_monitor = SafetyMonitor(self)

        # 출력 토픽
        self.state_code_pub = self.create_publisher(UInt8, '/state_code', 10)

        # 서비스
        self.cancel_mission_srv = self.create_service(
            Trigger, '/cancel_mission', self.on_cancel_mission
        )

        self.start_mission_srv = self.create_service(
            StartMission, '/start_mission', self.on_start_mission
        )

        # 상태전이 판단 타이머 5Hz (DDS-001 2.1 타이머 주기)
        self.state_timer = self.create_timer(0.2, self.on_state_timer)
        # 배터리 체크 1Hz
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

        # TODO(8/3): SafetyMonitor 배터리 90% 이상 체크 연동
        self.state_machine.transition(MissionState.GUIDING)
        self.get_logger().info(f'start_mission requested: destination={dest}')
        response.success = True
        response.message = '안내를 시작합니다'
        return response

    def on_cancel_mission(self, request, response):
        # TODO(8/6): PAUSED 전이 시 SafetyMonitor.stop_and_hold() 등 안전 정지 연동
        self.state_machine.transition(MissionState.PAUSED)
        self.get_logger().info('cancel_mission requested')
        response.success = True
        response.message = '임무를 취소합니다'
        return response

    def on_state_timer(self):
        # TODO: 상태전이 판단 로직 (장애물, 도착, 배터리 등 종합)
        pass

    def on_battery_timer(self):
        # TODO(8/3): SafetyMonitor.check_battery() 연동
        pass


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
