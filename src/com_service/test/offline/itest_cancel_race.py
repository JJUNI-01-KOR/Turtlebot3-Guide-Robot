#!/usr/bin/env python3
"""goal 수락 전 취소 유실 경합 재현 테스트.

2026-08-19 실기체에서 실측된 사고를 그대로 재현한다:
  start_mission -> send_goal_async 직후, accept 응답이 오기 전에 사람 감지가
  들어오면 cancel_current_nav_goal()이 아무 일도 못 해서 Nav2가 goal을 들고
  끝까지 주행해버렸다.

가짜 액션 서버의 goal_callback에 인위적 지연을 넣어 그 창을 넓힌다.
"""
import sys
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, String

from com_interfaces.srv import StartMission

ACCEPT_DELAY = 1.5      # accept 응답을 이만큼 늦춘다 (경합 창 확대)
fails = []


def check(label, cond, detail=''):
    print(('  PASS   ' if cond else '>>FAIL<< ') + f'{label} {detail}', flush=True)
    if not cond:
        fails.append(label)


class D(Node):
    def __init__(self):
        super().__init__('race_driver')
        g = ReentrantCallbackGroup()
        self.goals = 0
        self.cancels = 0
        self.terminal = []        # 각 goal이 어떻게 끝났는지
        ActionServer(self, NavigateToPose, '/test/navigate_to_pose',
                     execute_callback=self._ex, goal_callback=self._g,
                     cancel_callback=self._c, callback_group=g)
        self.pub_person = self.create_publisher(Bool, '/test/person_detected', 10)
        self.state = None
        self.create_subscription(String, '/test/mission_state',
                                 self._st, 10, callback_group=g)
        self.cli = self.create_client(StartMission, '/test/start_mission',
                                      callback_group=g)

    def _g(self, req):
        self.goals += 1
        print(f'      [fake] goal #{self.goals} 수신 -> {ACCEPT_DELAY}s 후 ACCEPT', flush=True)
        time.sleep(ACCEPT_DELAY)        # 수락을 일부러 늦춤
        return GoalResponse.ACCEPT

    def _c(self, gh):
        self.cancels += 1
        print(f'      [fake] 취소 요청 #{self.cancels} 수신', flush=True)
        return CancelResponse.ACCEPT

    def _ex(self, gh):
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < 20.0:
            if gh.is_cancel_requested:
                gh.canceled()
                self.terminal.append('CANCELED')
                print('      [fake] goal CANCELED', flush=True)
                return NavigateToPose.Result()
            time.sleep(0.05)
        gh.succeed()
        self.terminal.append('SUCCEEDED')
        print('      [fake] goal SUCCEEDED (= 로봇이 끝까지 주행했다는 뜻)', flush=True)
        return NavigateToPose.Result()

    def _st(self, m):
        if m.data != self.state:
            print(f'      [state] {self.state} -> {m.data}', flush=True)
            self.state = m.data

    def wait_state(self, want, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            if self.state == want:
                return True
            time.sleep(0.05)
        return False


def main():
    rclpy.init()
    d = D()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(d)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(1.5)

    check('READY 도달', d.wait_state('READY', 20.0), f'state={d.state}')
    d.cli.wait_for_service(10.0)

    print('\n[재현] start_mission 직후 accept 대기 중에 사람 감지 주입', flush=True)
    req = StartMission.Request()
    req.destination_id = 'class5'
    d.cli.call_async(req)

    # accept가 오기 전(ACCEPT_DELAY=1.5s 창) 안에 사람 감지를 밀어넣는다
    time.sleep(0.3)
    for _ in range(3):
        d.pub_person.publish(Bool(data=True))
        time.sleep(0.05)
    print('      >>> person_detected=True 주입 (goal accept 전)', flush=True)

    check('PERSON_WAIT 전이', d.wait_state('PERSON_WAIT', 6.0), f'state={d.state}')

    # accept가 돌아온 뒤 취소가 실제로 나가는지
    end = time.time() + 8
    while d.cancels < 1 and time.time() < end:
        time.sleep(0.1)
    check('수락 전 취소가 유실되지 않고 실제로 전달됨', d.cancels >= 1,
          f'cancels={d.cancels}')

    end = time.time() + 10
    while not d.terminal and time.time() < end:
        time.sleep(0.1)
    check('goal이 CANCELED로 끝남 (SUCCEEDED면 로봇이 완주했다는 뜻)',
          d.terminal and d.terminal[0] == 'CANCELED', f'terminal={d.terminal}')

    check('사람이 남아 있는 동안 PERSON_WAIT 유지', d.wait_state('PERSON_WAIT', 1.0)
          and d.state == 'PERSON_WAIT', f'state={d.state}')
    check('재출발용 새 goal 없음', d.goals == 1, f'goals={d.goals}')

    print('\n' + ('=== 실패 없음 — 경합 차단 확인 ===' if not fails
                  else f'=== 실패 {len(fails)}건: {fails} ==='), flush=True)
    ex.shutdown()
    rclpy.shutdown()
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
