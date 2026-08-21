"""
status_node.py
---------------
DDS-001 2.2절 status 노드.

하는 일:
  1. 여러 곳에서 따로따로 오는 정보(위치, 배터리, 상태, 경유점)를
     RobotState.msg 하나로 조립해서 2Hz로 /robot_state 발행 (GUI가 구독)
  2. /env_data(온습도·조도)를 메모리에 모아두고 /get_measurement_log로
     조회 서비스 제공

왜 이 노드가 따로 필요하냐면:
  service_manager는 "임무를 어떻게 진행할지"에만 집중해야 하는 노드라서
  GUI 표시용 데이터 조립까지 거기서 하면 책임이 섞여버려 (SAD-001 3.2
  계층 구조: service_manager=응용 계층, status=응용 계층이지만 역할 분리).
  그래서 "여러 소스를 모아서 GUI에 보여주기 좋은 형태로 바꾸는" 역할만
  이 노드가 전담한다.
"""

import math
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D

from com_interfaces.msg import RobotState
from com_interfaces.srv import GetMeasurementLog

# /env_data 는 sensor_bridge 쪽에서 아직 발행 안 할 수도 있어서 선택적으로 처리
try:
    from com_interfaces.msg import EnvData
    HAS_ENV_DATA = True
except ImportError:
    HAS_ENV_DATA = False


ROBOT_STATE_PUBLISH_HZ = 2.0     # ICD-001 IF-012 기준
MAX_LOG_RECORDS = 5000           # 메모리에 쌓아둘 최대 로그 개수 (너무 커지지 않게)


def quaternion_to_yaw(qz: float, qw: float) -> float:
    """
    쿼터니언 -> yaw(라디안) 변환.
    로봇은 지면 위에서만 움직이니까(2D) roll/pitch는 0이라고 가정하고
    z, w 성분만으로 계산해도 충분하다. tf_transformations 의존성을 줄이려고
    공식을 직접 씀: yaw = atan2(2*(w*z), 1 - 2*z^2)  (roll=pitch=0 가정)
    """
    return math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)


class StatusNode(Node):

    def __init__(self):
        super().__init__('status')

        # ---- 최신값 캐시 (뭔가 하나라도 아직 안 왔으면 기본값 유지) ----
        self._pose = Pose2D()
        self._battery_percent = 0.0
        self._mission_state = 'SYSTEM_INIT'
        self._current_waypoint_id = ''

        # ---- 측정 로그 저장소 (메모리) ----
        self._log_records = deque(maxlen=MAX_LOG_RECORDS)

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        # ---- 구독 ----
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, reliable_qos)
        self.create_subscription(
            Float32, '/battery_percent', self._on_battery, reliable_qos)
        self.create_subscription(
            String, '/mission_state', self._on_mission_state, reliable_qos)
        self.create_subscription(
            String, '/current_waypoint_id', self._on_waypoint_id, reliable_qos)

        if HAS_ENV_DATA:
            self.create_subscription(
                EnvData, '/env_data', self._on_env_data, reliable_qos)
        else:
            self.get_logger().warn(
                'com_interfaces에 EnvData.msg가 없어서 /env_data 구독을 건너뜁니다. '
                '측정 로그 조회 기능이 항상 빈 결과를 반환합니다.')

        # ---- 발행 ----
        self.robot_state_pub = self.create_publisher(RobotState, '/robot_state', 10)
        self.create_timer(1.0 / ROBOT_STATE_PUBLISH_HZ, self._publish_robot_state)

        # ---- 서비스 ----
        self.create_service(
            GetMeasurementLog, '/get_measurement_log', self._on_get_log)

        self.get_logger().info('status node started')

    # ------------------------------------------------------------------
    # 구독 콜백 - 최신값만 캐시에 저장 (실제 발행은 타이머가 주기적으로)
    # ------------------------------------------------------------------
    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        self._pose.x = msg.pose.pose.position.x
        self._pose.y = msg.pose.pose.position.y
        self._pose.theta = quaternion_to_yaw(q.z, q.w)

    def _on_battery(self, msg: Float32):
        self._battery_percent = float(msg.data)

    def _on_mission_state(self, msg: String):
        self._mission_state = msg.data

    def _on_waypoint_id(self, msg: String):
        self._current_waypoint_id = msg.data

    def _on_env_data(self, msg):
        # 그대로 저장해뒀다가 로그 조회 요청 오면 필터링해서 돌려준다
        self._log_records.append(msg)

    # ------------------------------------------------------------------
    def _publish_robot_state(self):
        msg = RobotState()
        msg.pose = self._pose
        msg.battery_percent = self._battery_percent
        msg.mission_state = self._mission_state
        msg.current_waypoint_id = self._current_waypoint_id
        self.robot_state_pub.publish(msg)

    # ------------------------------------------------------------------
    def _on_get_log(self, request, response):
        wp_filter = (request.waypoint_id_filter or '').strip()
        date_filter = (request.date_filter or '').strip()   # 'YYYY-MM-DD'

        results = []
        for rec in self._log_records:
            if wp_filter and rec.waypoint_id != wp_filter:
                continue
            if date_filter:
                # 2026-08-18 코드 리뷰: 여기는 UTC로 날짜를 계산하는데 GUI(main_window.py
                # _on_log_result)는 같은 타임스탬프를 로컬시간(KST)으로 그냥 표시한다.
                # KST는 UTC+9라서 자정 근처(로컬 00~09시) 로그는 필터링 기준 날짜가
                # 화면에 보이는 날짜와 하루 어긋난다. GUI 쪽을 UTC로 바꾸는 것보다
                # 로컬시간으로 맞추는 게 사용자가 실제로 입력하는 날짜와 일치한다.
                rec_date = datetime.fromtimestamp(rec.stamp.sec).strftime('%Y-%m-%d')
                if rec_date != date_filter:
                    continue
            results.append(rec)

        response.success = True
        response.records = results
        self.get_logger().info(
            f'get_measurement_log 조회: wp={wp_filter or "전체"}, '
            f'date={date_filter or "전체"} -> {len(results)}건')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = StatusNode()
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
