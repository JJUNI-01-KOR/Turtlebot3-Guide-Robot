"""service_manager_node의 SafetyMonitor / DockingController 로직만 떼어내
가짜 노드로 검증한다 (rclpy 실행 없이)."""
import sys, types

sys.path.insert(0, '/home/com/com_ws/src/com_service')
sys.path.insert(0, '/home/com/com_ws/src/com_service/com_service')

# rclpy/메시지 의존성을 최소한으로 스텁 처리
for name in ['rclpy', 'rclpy.node', 'rclpy.action', 'rclpy.qos', 'rclpy.duration', 'rclpy.time', 'std_msgs',
             'std_msgs.msg', 'std_srvs', 'std_srvs.srv', 'sensor_msgs',
             'sensor_msgs.msg', 'geometry_msgs', 'geometry_msgs.msg',
             'com_interfaces', 'com_interfaces.srv', 'com_interfaces.msg',
             'nav2_msgs', 'nav2_msgs.action', 'nav_msgs', 'nav_msgs.msg', 'ament_index_python',
             'ament_index_python.packages', 'yaml']:
    m = types.ModuleType(name)
    m.__getattr__ = lambda attr: type(attr, (), {'__init__': lambda s, *a, **k: None})
    sys.modules.setdefault(name, m)

import service_manager_node as smn  # noqa
from com_service.mission_state import MissionState  # noqa

PARAMS = {
    'obstacle_stop_distance': 0.20, 'obstacle_search_max': 2,
    'battery_return_threshold': 20, 'person_wait_retry_max': 10,
    'docking_speed_limit': 0.05, 'docking_max_attempts': 3,
    'aruco_search_max': 3, 'dock_marker_id': 29, 'dock_target_distance': 0.25,
    'dock_turn_left_angle': 0.0, 'dock_nudge_angle': 0.1222,
    'dock_turn_speed': 0.35, 'dock_turn_tolerance': 0.035, 'dock_settle_time': 1.0,
    'dock_stale_margin': 0.3, 'dock_k_yaw': 0.5, 'dock_use_yaw_term': True,
    'rear_obstacle_confirm_sec': 0.6, 'rear_clear_confirm_sec': 1.5, 'rear_sparse_interval': 0.5,
}


class FakeLogger:
    def __getattr__(self, _):
        return lambda *a, **k: None


class FakeSM:
    def __init__(self):
        self.state = MissionState.GUIDING
        self.history = []

    def transition(self, s):
        self.state = s
        self.history.append(s.name)


class FakeTime:
    def __init__(self, sec):
        self.nanoseconds = int(sec * 1e9)

    def __sub__(self, o):
        return FakeTime((self.nanoseconds - o.nanoseconds) / 1e9)

    def __add__(self, d):
        return FakeTime((self.nanoseconds + d.nanoseconds) / 1e9)


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return FakeTime(self.t)


class FakeNode:
    def __init__(self):
        self.state_machine = FakeSM()
        self.resends = 0
        self.cmds = []
        self.clock = FakeClock()

    def get_clock(self):
        return self.clock

    def get_parameter(self, n):
        return types.SimpleNamespace(value=PARAMS[n])

    def get_logger(self):
        return FakeLogger()

    def publish_cmd_vel(self, l, a):
        self.cmds.append((l, a))

    def cancel_current_nav_goal(self):
        pass

    def resend_current_waypoint(self):
        self.resends += 1

    def publish_tts(self, *a):
        pass

    def start_return(self):
        self.returns = getattr(self, 'returns', 0) + 1


F = lambda v: types.SimpleNamespace(data=v)   # noqa: E731
fails = []



def rear(sm, cm, n=8):
    """중앙값 필터(>=3) + 시간 기반 확정(0.6s/1.5s)을 통과하도록 주입.

    FakeNode의 시계를 진행시켜야 시간 판정이 성립한다(2026-08-19: 횟수 기반에서
    시간 기반으로 바뀜 - 초음파 실측 발행률이 0.5Hz라 횟수로는 6초가 걸렸다)."""
    for _ in range(n):
        sm.handle_rear_ir(F(cm))
        sm.node.clock.t += 0.5

def check(label, cond, detail=''):
    print(('  PASS  ' if cond else '> FAIL <') + f' {label} {detail}')
    if not cond:
        fails.append(label)


# ── 3. 장애물/사람 교차 복구 ─────────────────────────────────────────
print('\n[3] PERSON_WAIT 중 장애물 발생 -> 사람만 먼저 해제')
node = FakeNode()
sm = smn.SafetyMonitor(node)
sm.handle_person_detected(F(True))                 # GUIDING -> PERSON_WAIT
check('사람 감지 -> PERSON_WAIT',
      node.state_machine.state == MissionState.PERSON_WAIT)
rear(sm, 10.0)                                     # 10cm -> OBSTACLE_STOP
check('장애물 추가 -> OBSTACLE_STOP',
      node.state_machine.state == MissionState.OBSTACLE_STOP)
sm.handle_person_detected(F(False))                # 사람만 비켜남
check('사람 해제해도 주행 재개 안 함 (resend=0)', node.resends == 0,
      f'resends={node.resends}')
check('OBSTACLE_STOP 유지',
      node.state_machine.state == MissionState.OBSTACLE_STOP,
      node.state_machine.state.name)
rear(sm, -1.0)                                     # 장애물도 해제
check('둘 다 해제 -> GUIDING 복귀',
      node.state_machine.state == MissionState.GUIDING)
check('이때 주행 재개 (resend=1)', node.resends == 1, f'resends={node.resends}')

print('\n[3b] 반대 순서 (OBSTACLE_STOP 중 사람 -> 장애물만 먼저 해제)')
node = FakeNode()
sm = smn.SafetyMonitor(node)
rear(sm, 10.0)
sm.handle_person_detected(F(True))
rear(sm, -1.0)
check('장애물만 해제 -> 재개 안 함', node.resends == 0, f'resends={node.resends}')
check('PERSON_WAIT 유지',
      node.state_machine.state == MissionState.PERSON_WAIT,
      node.state_machine.state.name)
sm.handle_person_detected(F(False))
check('둘 다 해제 -> RETURNING 아닌 GUIDING 복귀',
      node.state_machine.state == MissionState.GUIDING)
check('복귀 후 주행 재개', node.resends == 1)

# ── 1. PERSON_WAIT에서 nav 결과 무시 ────────────────────────────────
print('\n[1] nav goal 결과 처리 상태 집합')
check('DRIVING_STATES에 PERSON_WAIT 없음',
      MissionState.PERSON_WAIT not in smn.SafetyMonitor.DRIVING_STATES)
check('DRIVING_STATES에 OBSTACLE_STOP 없음',
      MissionState.OBSTACLE_STOP not in smn.SafetyMonitor.DRIVING_STATES)
check('DRIVING_STATES = (GUIDING, RETURNING)',
      smn.SafetyMonitor.DRIVING_STATES ==
      (MissionState.GUIDING, MissionState.RETURNING))

# ── 4. 배터리 무효값 ────────────────────────────────────────────────
print('\n[4] 배터리 전압 유효성')
node = FakeNode()
sm = smn.SafetyMonitor(node)
sm.check_battery(0.0)
check('0.0V 무시 (percent 유지)', sm.last_battery_percent == 100.0,
      f'percent={sm.last_battery_percent}')
check('0.0V로 RETURNING 전이 안 함',
      node.state_machine.state == MissionState.GUIDING)
sm.check_battery(float('nan'))
check('NaN 무시', sm.last_battery_percent == 100.0)
sm.check_battery(12.6)
check('12.6V -> 100%', abs(sm.last_battery_percent - 100.0) < 0.01)
# 평활화(EMA) 도입 후: 단발 강하는 무시, 지속 저전압은 트리거되어야 한다.
sm.check_battery(9.6)
check('9.6V 단발로는 RETURNING 오발동 없음',
      node.state_machine.state != MissionState.RETURNING,
      f'percent={sm.last_battery_percent:.1f}')
for _ in range(30):
    sm.check_battery(9.6)
check('9.6V 지속 -> 16.7%로 수렴', abs(sm.last_battery_percent - 16.67) < 0.5,
      f'percent={sm.last_battery_percent:.2f}')
check('지속 저전압 -> RETURNING 전이',
      node.state_machine.state == MissionState.RETURNING)

# ── 2. 도킹 목표 거리 ───────────────────────────────────────────────
print('\n[2] 도킹 거리 제어 (목표 0.25m)')
node = FakeNode()
dc = smn.DockingController(node)
def mk(lat, dist):
    q = types.SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0)   # yaw = 0
    pose = types.SimpleNamespace(pose=types.SimpleNamespace(orientation=q))
    return types.SimpleNamespace(marker_id=29, lateral_error=lat,
                                 distance_error=dist, pose=pose)
lin, _ = dc.compute_docking_velocity(mk(0.0, 1.00))
check('1.00m -> 전진', lin > 0, f'v={lin:.3f}')
lin, _ = dc.compute_docking_velocity(mk(0.0, 0.25))
check('0.25m(목표) -> 정지', abs(lin) < 1e-9, f'v={lin:.3f}')
lin, _ = dc.compute_docking_velocity(mk(0.0, 0.15))
check('0.15m(과접근) -> 후진', lin < 0, f'v={lin:.3f}')
check('0.03m에서 정렬완료 아님 (예전 버그)', not dc._is_aligned(mk(0.0, 0.03)))
check('0.26m + lateral 0.01 -> 정렬완료', dc._is_aligned(mk(0.01, 0.26)))
check('0.26m + lateral 0.05 -> 미정렬', not dc._is_aligned(mk(0.05, 0.26)))

import math as _m

# ── 8. 08-18 도킹 단계 구조가 유지되는지 (15도 회전 재탐색은 폐기됨) ──
print('\n[8] 도킹 단계 구조 (2026-08-18 재설계 유지 확인)')
check('PHASE_TURN_LEFT 존재', hasattr(smn.DockingController, 'PHASE_TURN_LEFT'))
check('PHASE_SETTLE 존재', hasattr(smn.DockingController, 'PHASE_SETTLE'))
check('PHASE_ALIGN 존재', hasattr(smn.DockingController, 'PHASE_ALIGN'))
check('15도 회전 재탐색 방식은 제거됨',
      not hasattr(smn.DockingController, 'search_rotate_sec'))
import inspect as _insp
_src = _insp.getsource(smn.DockingController)
check('stop-and-look(정지 후 판정) 주석 유지', 'stop-and-look' in _src)
check('_range_error가 목표거리를 뺌', 'dock_target_distance' in _src)

# ── 7. yaw 반영 ─────────────────────────────────────────────────────
print('\n[7] waypoint yaw -> quaternion')
for yaw in (0.0, _m.pi / 2, -_m.pi / 2, _m.pi):
    qz, qw = _m.sin(yaw / 2.0), _m.cos(yaw / 2.0)
    back = 2.0 * _m.atan2(qz, qw)
    check(f'yaw={yaw:+.3f} 왕복 변환', abs(_m.atan2(_m.sin(back - yaw),
                                                  _m.cos(back - yaw))) < 1e-9)
check('yaw 키가 없으면 0.0 기본값', {'x': 1, 'y': 2}.get('yaw', 0.0) == 0.0)

# ── 신규: 취소 후 정지 래치 잔존 (실기체 실측 버그) ────────────────
print('\n[X] cancel 후 래치 잔존 -> 다음 임무에서 안전정지 무력화')
node = FakeNode()
sm = smn.SafetyMonitor(node)
sm.handle_person_detected(F(True))
check('사람 감지 -> PERSON_WAIT', node.state_machine.state == MissionState.PERSON_WAIT)
check('_person_active 래치됨', sm._person_active)
check('blocking_reason 이 사유를 보고함', sm.blocking_reason() is not None,
      str(sm.blocking_reason()))

# cancel_mission이 하는 일
sm.reset_stop_conditions()
node.state_machine.state = MissionState.READY
check('reset 후 _person_active 해제', not sm._person_active)
check('reset 후 _obstacle_active 해제', not sm._obstacle_active)
check('reset 후 _state_before_stop 해제', sm._state_before_stop is None)
check('사람이 아직 있으면 blocking_reason 유지 (출발 거부용)',
      sm.blocking_reason() is not None, str(sm.blocking_reason()))

# 래치가 풀렸으니 같은 사람이 다시 에지로 잡혀야 한다
node.state_machine.state = MissionState.GUIDING
sm.handle_person_detected(F(True))
check('재시작 후 사람 재감지 -> PERSON_WAIT 다시 걸림',
      node.state_machine.state == MissionState.PERSON_WAIT,
      node.state_machine.state.name)

sm.handle_person_detected(F(False))
check('사람 사라지면 blocking_reason 없음', sm.blocking_reason() is None)
node2 = FakeNode(); sm2 = smn.SafetyMonitor(node2)
rear(sm2, 10.0)
check('장애물도 blocking_reason 보고', sm2.blocking_reason() is not None,
      str(sm2.blocking_reason()))
rear(sm2, -1.0)
check('장애물 해제 시 blocking_reason 없음', sm2.blocking_reason() is None)


# ── 신규: 0.5Hz 실측 발행률에서 장애물 확정 지연 ──────────────────
print('\n[R] 초음파 0.5Hz(실측)에서 장애물 확정까지 걸리는 시간')
node = FakeNode()
sm = smn.SafetyMonitor(node)
# 정상 주행 중 멀쩡한 값 몇 개 (버퍼 채우기)
for _ in range(4):
    sm.handle_rear_ir(F(150.0)); node.clock.t += 2.0
t_start = node.clock.t
n_msgs = 0
while node.state_machine.state != MissionState.OBSTACLE_STOP and n_msgs < 20:
    sm.handle_rear_ir(F(10.0)); node.clock.t += 2.0; n_msgs += 1
delay = node.clock.t - t_start
check('OBSTACLE_STOP 도달', node.state_machine.state == MissionState.OBSTACLE_STOP)
check('확정 지연 <= 4초 (구 방식은 6초 이상)', delay <= 4.0, f'{delay:.1f}s, {n_msgs}개 메시지')
travelled = 0.22 * delay
check('0.22m/s 기준 주행거리 <= 0.9m', travelled <= 0.9, f'{travelled:.2f}m')


print('\n[R2] 아두이노 수정 후(200ms=5Hz) 예상 지연 - 적응 로직이 깊은 필터로 돌아가는지')
node = FakeNode()
sm = smn.SafetyMonitor(node)
DT = 0.2
for _ in range(6):
    sm.handle_rear_ir(F(150.0)); node.clock.t += DT
check('5Hz는 희소로 분류되지 않음', not sm._rear_is_sparse(),
      f'interval={sm._rear_interval:.2f}s')
t0 = node.clock.t; n = 0
while node.state_machine.state != MissionState.OBSTACLE_STOP and n < 40:
    sm.handle_rear_ir(F(10.0)); node.clock.t += DT; n += 1
d = node.clock.t - t0
check('확정 지연 <= 1.5초', d <= 1.5, f'{d:.2f}s, {n}개')
check('0.22m/s 기준 주행거리 <= 0.35m', 0.22 * d <= 0.35, f'{0.22*d:.2f}m')

# 5Hz에서 튄 값 하나는 무시되어야 한다 (중앙값 필터 유지 확인)
node2 = FakeNode(); sm2 = smn.SafetyMonitor(node2)
for _ in range(6):
    sm2.handle_rear_ir(F(150.0)); node2.clock.t += DT
sm2.handle_rear_ir(F(5.0)); node2.clock.t += DT      # 튄 값 1개
sm2.handle_rear_ir(F(150.0)); node2.clock.t += DT
check('5Hz에서 튄 값 1개는 무시됨',
      node2.state_machine.state != MissionState.OBSTACLE_STOP,
      node2.state_machine.state.name)


print('\n[D] 도킹 회전각 0 -> 도착 즉시 마커 정렬로 진입하는지')
node = FakeNode(); node.odom_yaw = 0.0
node.state_machine.state = MissionState.DOCKING
dc = smn.DockingController(node)
check('turn_goal 이 0', dc.turn_goal == 0.0, str(dc.turn_goal))
dc.start()
check('시작 단계는 TURN_LEFT', dc.phase == smn.DockingController.PHASE_TURN_LEFT, dc.phase)
node.cmds.clear()
dc.update()                    # 1회차: turn_prev_yaw 초기화
node.odom_yaw = 0.001          # 아주 미세한 오도메트리 변화
dc.update()                    # 2회차: remaining <= tolerance -> SETTLE
check('두 번째 주기에 TURN_LEFT 탈출',
      dc.phase != smn.DockingController.PHASE_TURN_LEFT, dc.phase)
check('회전 명령이 나가지 않음',
      all(abs(a) < 1e-9 for _, a in node.cmds), str(node.cmds))

print('\n' + ('실패 없음 — 전부 통과' if not fails else f'실패: {fails}'))
sys.exit(1 if fails else 0)
