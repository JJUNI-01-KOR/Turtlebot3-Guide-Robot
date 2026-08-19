#!/usr/bin/env python3
"""service_manager_node 통합 테스트 드라이버 (로봇 무접촉).

로봇의 Nav2(/navigate_to_pose)는 진짜로 바퀴를 굴리므로 절대 안 쓴다.
대신 여기서 가짜 NavigateToPose 액션 서버를 /test/navigate_to_pose에 띄우고,
service_manager를 전 토픽 리맵으로 격리 실행해서 붙인다.
로봇의 구버전 service_manager(PM2)는 그대로 두고 건드리지 않는다.
"""
import sys
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, String

from com_interfaces.srv import StartMission

fails = []


def check(label, cond, detail=''):
    print(('  PASS   ' if cond else '>>FAIL<< ') + f'{label} {detail}', flush=True)
    if not cond:
        fails.append(label)


class Driver(Node):
    def __init__(self):
        super().__init__('itest_driver')
        cbg = ReentrantCallbackGroup()

        # ---- 가짜 Nav2 액션 서버 ----
        self.goals_received = []
        self.cancel_requests = 0
        self._active_goal = None
        self.nav_srv = ActionServer(
            self, NavigateToPose, '/test/navigate_to_pose',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=cbg)

        # ---- 센서 주입 퍼블리셔 ----
        be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_rear = self.create_publisher(Float32, '/test/rear_distance', be)
        self.pub_person = self.create_publisher(Bool, '/test/person_detected', 10)
        self.pub_batt = self.create_publisher(BatteryState, '/test/battery_state', 10)

        # ---- 관측 ----
        self.state = None
        self.states_seen = []
        self.cmds = []
        self.create_subscription(String, '/test/mission_state',
                                 self._on_state, 10, callback_group=cbg)
        self.create_subscription(Twist, '/test/cmd_vel',
                                 self._on_cmd, 10, callback_group=cbg)
        self.create_subscription(Float32, '/test/battery_percent',
                                 self._on_batt_pct, 10, callback_group=cbg)
        self.battery_percent = None

        self.cli_start = self.create_client(StartMission, '/test/start_mission',
                                            callback_group=cbg)

    # --- 가짜 액션 서버 콜백 ---
    def _on_goal(self, goal_request):
        self.goals_received.append(time.time())
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        self.cancel_requests += 1
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        self._active_goal = goal_handle
        # 취소 요청이 올 때까지 붙잡고 있는다 (실제 주행처럼 오래 걸리는 goal 흉내)
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.05)
        goal_handle.abort()
        return NavigateToPose.Result()

    # --- 관측 콜백 ---
    def _on_state(self, msg):
        if msg.data != self.state:
            self.states_seen.append(msg.data)
            print(f'      [state] {self.state} -> {msg.data}', flush=True)
        self.state = msg.data

    def _on_cmd(self, msg):
        self.cmds.append((round(msg.linear.x, 4), round(msg.angular.z, 4)))

    def _on_batt_pct(self, msg):
        self.battery_percent = msg.data

    # --- 헬퍼 ---
    def wait_state(self, want, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            if self.state == want:
                return True
            time.sleep(0.05)
        return False

    def stays(self, want, seconds=2.5):
        """seconds 동안 want 상태를 계속 유지하는지 확인."""
        end = time.time() + seconds
        while time.time() < end:
            if self.state != want:
                return False
            time.sleep(0.05)
        return True

    def rear(self, cm):
        for _ in range(4):
            self.pub_rear.publish(Float32(data=float(cm)))
            time.sleep(0.05)

    def person(self, present):
        for _ in range(3):
            self.pub_person.publish(Bool(data=bool(present)))
            time.sleep(0.05)

    def battery(self, volts):
        m = BatteryState()
        m.voltage = float(volts)
        for _ in range(3):
            self.pub_batt.publish(m)
            time.sleep(0.05)


def scenario(d):
    time.sleep(1.0)
    print('\n[0] 기동 / READY 도달', flush=True)
    check('READY 도달', d.wait_state('READY', 20.0), f'state={d.state}')
    check('/test/start_mission 서비스 발견', d.cli_start.wait_for_service(10.0))

    print('\n[A] 임무 시작 -> GUIDING', flush=True)
    req = StartMission.Request()
    req.destination_id = 'class5'
    fut = d.cli_start.call_async(req)
    end = time.time() + 10
    while not fut.done() and time.time() < end:
        time.sleep(0.05)
    check('start_mission 수락', fut.done() and fut.result().success,
          fut.result().message if fut.done() else '(타임아웃)')
    check('GUIDING 전이', d.wait_state('GUIDING', 5.0), f'state={d.state}')
    end = time.time() + 5
    while len(d.goals_received) < 1 and time.time() < end:
        time.sleep(0.05)
    check('가짜 Nav2가 goal 1개 수신', len(d.goals_received) == 1,
          f'goals={len(d.goals_received)}')

    print('\n[B] 사람 감지 -> PERSON_WAIT (fix #1: 취소결과가 재시도로 새면 안 됨)', flush=True)
    n_goals = len(d.goals_received)
    d.person(True)
    check('PERSON_WAIT 전이', d.wait_state('PERSON_WAIT', 5.0), f'state={d.state}')
    check('nav goal 취소 요청됨', d.cancel_requests >= 1, f'cancels={d.cancel_requests}')
    check('정지 명령 발행', (0.0, 0.0) in d.cmds[-6:], str(d.cmds[-3:]))
    # 취소 결과(CANCELED)가 도착한 뒤에도 재시도 goal이 나가면 안 된다
    check('PERSON_WAIT 유지 (자동 재출발 없음)', d.stays('PERSON_WAIT', 3.0),
          f'state={d.state}')
    check('새 goal 안 나감 (fix #1)', len(d.goals_received) == n_goals,
          f'{n_goals} -> {len(d.goals_received)}')

    print('\n[C] 사람 대기 중 장애물 추가 -> OBSTACLE_STOP', flush=True)
    d.rear(10.0)                      # 10cm
    check('OBSTACLE_STOP 전이', d.wait_state('OBSTACLE_STOP', 5.0), f'state={d.state}')

    print('\n[D] 사람만 먼저 해제 (fix #3: 장애물 남았으면 재개 금지)', flush=True)
    n_goals = len(d.goals_received)
    d.person(False)
    check('OBSTACLE_STOP 유지', d.stays('OBSTACLE_STOP', 3.0), f'state={d.state}')
    check('주행 재개 안 함 (새 goal 없음)', len(d.goals_received) == n_goals,
          f'{n_goals} -> {len(d.goals_received)}')

    print('\n[E] 장애물도 해제 -> GUIDING 복귀 + 재전송', flush=True)
    d.rear(-1.0)                      # 초음파 미검출 센티널
    check('GUIDING 복귀', d.wait_state('GUIDING', 5.0), f'state={d.state}')
    end = time.time() + 5
    while len(d.goals_received) < n_goals + 1 and time.time() < end:
        time.sleep(0.05)
    check('goal 재전송됨', len(d.goals_received) == n_goals + 1,
          f'{n_goals} -> {len(d.goals_received)}')

    print('\n[F] 배터리 0.0V 오독 (fix #4)', flush=True)
    d.battery(0.0)
    check('GUIDING 유지 (RETURNING 오발동 없음)', d.stays('GUIDING', 3.0),
          f'state={d.state}')
    check('battery_percent 0%로 안 떨어짐',
          d.battery_percent is None or d.battery_percent > 50.0,
          f'percent={d.battery_percent}')

    print('\n[G] 실제 저전압 9.5V -> RETURNING', flush=True)
    d.battery(9.5)
    check('RETURNING 전이', d.wait_state('RETURNING', 6.0), f'state={d.state}')
    check('battery_percent 약 13.9%',
          d.battery_percent is not None and abs(d.battery_percent - 13.89) < 1.0,
          f'percent={d.battery_percent}')

    print('\n' + ('=== 실패 없음 — 전부 통과 ===' if not fails
                  else f'=== 실패 {len(fails)}건: {fails} ==='), flush=True)


def main():
    rclpy.init()
    d = Driver()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(d)
    t = threading.Thread(target=ex.spin, daemon=True)
    t.start()
    try:
        scenario(d)
    finally:
        ex.shutdown()
        d.destroy_node()
        rclpy.shutdown()
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
