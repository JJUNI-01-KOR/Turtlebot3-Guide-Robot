"""실제 waypoints.yaml로 WaypointSequencer 로드/경로생성 검증."""
import sys, types
sys.path.insert(0, '/home/com/com_ws/src/com_service')
sys.path.insert(0, '/home/com/com_ws/src/com_service/com_service')
for n in ['rclpy', 'rclpy.node', 'rclpy.action', 'rclpy.qos', 'rclpy.duration', 'rclpy.time',
          'std_msgs', 'std_msgs.msg', 'std_srvs', 'std_srvs.srv', 'sensor_msgs',
          'sensor_msgs.msg', 'geometry_msgs', 'geometry_msgs.msg', 'com_interfaces',
          'com_interfaces.srv', 'com_interfaces.msg', 'nav2_msgs', 'nav2_msgs.action', 'nav_msgs', 'nav_msgs.msg',
          'ament_index_python', 'ament_index_python.packages']:
    m = types.ModuleType(n)
    m.__getattr__ = lambda a: type(a, (), {'__init__': lambda s, *x, **k: None})
    sys.modules.setdefault(n, m)
import service_manager_node as smn

logs = []
class L:
    def __getattr__(self, lvl):
        return lambda msg, **k: logs.append((lvl, msg))
class N:
    def get_parameter(self, n): return types.SimpleNamespace(value='docking')
    def get_logger(self): return L()

fails = []
def check(label, cond, detail=''):
    print(('  PASS  ' if cond else '> FAIL <') + f' {label} {detail}')
    if not cond: fails.append(label)

ws = smn.WaypointSequencer(N())
ok = ws.load_waypoints('/home/com/com_ws/src/com_navigation/config/waypoints.yaml')
check('로드 성공', ok)
check('웨이포인트 20개', len(ws.waypoints) == 20, str(len(ws.waypoints)))
DEST_IDS = ['class1', 'class2', 'class3', 'class4', 'class5',
            'info', 'office', 'meeting']
check('목적지 8개 class1~meeting', ws.destination_ids == DEST_IDS,
      str(ws.destination_ids))
check('docking이 waypoints에 존재 (warn 없음)',
      not any('docking_waypoint_id' in m for _, m in logs))

warns = [m for lvl, m in logs if lvl == 'warn']
check('미사용 경유점 warn 발생', any('trajectory_1/trajectory_2가 없어' in m for m in warns),
      str(warns))
unused_warn = [m for m in warns if '미사용 경유점' in m]
check('미사용 11개 보고 (20 - 목적지8 - docking1)', unused_warn and '11개' in unused_warn[0],
      unused_warn[0] if unused_warn else '(없음)')

ws.build_trajectory_1('class5')
check('trajectory_1 = [class5]', ws.current_trajectory == ['class5'], str(ws.current_trajectory))
ws.build_trajectory_2()
check('trajectory_2 = [docking]', ws.current_trajectory == ['docking'], str(ws.current_trajectory))

check('class5 yaw 키 존재', 'yaw' in ws.waypoints['class5'])
# 2026-08-19: 도킹 지점을 RViz 2D Pose Estimate로 재측정하며 yaw가 생겼다.
# _send_nav_goal이 yaw를 실제로 반영하므로 0이 아닌 값이 유지되는지 확인한다.
import math as _mm
_dy = ws.waypoints['docking']['yaw']
check('docking yaw 가 재측정값(1.4982 rad, 85.8도)', abs(_dy - 1.4981870525) < 1e-6,
      f'{_dy:.6f} rad = {_mm.degrees(_dy):.2f}도')
import yaml as _y
_n = _y.safe_load(open('/home/com/com_ws/src/com_navigation/config/nav2_params.yaml'))['amcl']['ros__parameters']['initial_pose']
_a = _y.safe_load(open('/home/com/com_ws/src/com_service/config/auto_dock_params.yaml'))['com_auto_dock_node']['ros__parameters']
_d = ws.waypoints['docking']
check('도킹 좌표가 세 파일에서 일치',
      abs(_n['x'] - _d['x']) < 1e-9 and abs(_n['y'] - _d['y']) < 1e-9
      and abs(_n['yaw'] - _d['yaw']) < 1e-9
      and abs(_a['dock_approach_x'] - _d['x']) < 1e-9
      and abs(_a['dock_approach_yaw'] - _d['yaw']) < 1e-9,
      f"wp=({_d['x']:.4f},{_d['y']:.4f},{_d['yaw']:.4f})")
check('잘못된 목적지 거부', not ws.is_valid_destination('way6'))
check('meeting 유효', ws.is_valid_destination('meeting'))


print()
expect = {'class1': (4.560420989990234, 0.5400186777114868),
          'class2': (9.178903579711914, 0.6446712613105774),
          'class3': (17.902101516723633, 0.5400208234786987),
          'class4': (20.9245548248291, -0.8556486964225769),
          'class5': (22.593984603881836, 0.41806095838546753)}
for rid, (ex, ey) in expect.items():
    w = ws.waypoints[rid]
    check(f'{rid} 좌표 반영', abs(w['x']-ex) < 1e-9 and abs(w['y']-ey) < 1e-9,
          f"({w['x']:.4f}, {w['y']:.4f})")
check('info/office/meeting 은 기존 좌표 유지',
      abs(ws.waypoints['info']['x'] - 12.564574241638184) < 1e-9
      and abs(ws.waypoints['office']['x'] - 2.9564778804779053) < 1e-9
      and abs(ws.waypoints['meeting']['x'] - 12.709179878234863) < 1e-9)
print('\n' + ('실패 없음 — 전부 통과' if not fails else f'실패: {fails}'))
sys.exit(1 if fails else 0)
