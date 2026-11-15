"""serial_bridge_node 의 라인 버퍼링 검증 (실제 시리얼/rclpy 없이)."""
import sys, types
import os
# 저장소의 src/ 폴더 (이 파일 기준 상대 경로라 어느 PC에서든 실행된다)
SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.insert(0, os.path.join(SRC, 'com_sensors'))
sys.path.insert(0, os.path.join(SRC, 'com_sensors', 'com_sensors'))
for n in ['rclpy','rclpy.node','rclpy.qos','std_msgs','std_msgs.msg','serial']:
    m = types.ModuleType(n)
    m.__getattr__ = lambda a: type(a, (), {'__init__': lambda s,*x,**k: None})
    sys.modules.setdefault(n, m)
import serial_bridge_node as sb

fails = []
def check(label, cond, detail=''):
    print(('  PASS   ' if cond else '>>FAIL<< ') + f'{label} {detail}')
    if not cond: fails.append(label)

class FakeSerial:
    """readline()이 개행 전에 timeout 나서 라인을 쪼개는 상황을 재현."""
    def __init__(self, chunks): self.chunks = list(chunks)
    def readline(self):
        return self.chunks.pop(0) if self.chunks else b''

class FakeLog:
    def __init__(self): self.msgs = []
    def warn(self, m, **k): self.msgs.append(('warn', m))
    def error(self, m, **k): self.msgs.append(('error', m))
    def info(self, m, **k): self.msgs.append(('info', m))
    def debug(self, m, **k): pass

node = sb.SensorBridgeNode.__new__(sb.SensorBridgeNode)   # __init__ 우회
node._rx_buf = b''
node._line_synced = True   # 기본 테스트는 동기 완료 상태로
node._log = FakeLog()
node.get_logger = lambda: node._log
got = []
node.parse_and_publish_joystick = lambda l: got.append(('J', l))
node.parse_and_publish_rear     = lambda l: got.append(('R', l))
node.parse_and_publish          = lambda l: got.append(('T', l))

print('[1] 정상 라인 (개행 포함)')
node.ser = FakeSerial([b'J:513,B:0\n', b'R:15.3,O:1\n'])
node.read_serial_callback()
check('조이스틱 수신', ('J', 'J:513,B:0') in got, str(got))
check('후방거리 수신', ('R', 'R:15.3,O:1') in got, str(got))

print('\n[2] 라인이 읽기 경계에서 쪼개진 경우 (실측 버그 재현)')
got.clear(); node._rx_buf = b''; node._line_synced = True; node._log.msgs.clear()
node.ser = FakeSerial([b'J:', b'513,B:0\n'])       # timeout으로 "J:" 만 먼저 옴
node.read_serial_callback()
check('쪼개진 조이스틱 라인이 복원됨', ('J', 'J:513,B:0') in got, str(got))
check('파싱 실패 경고 없음', not any(m[0]=='warn' for m in node._log.msgs),
      str(node._log.msgs))

print('\n[3] 3조각으로 쪼개진 경우')
got.clear(); node._rx_buf = b''; node._line_synced = True
node.ser = FakeSerial([b'R:1', b'5.', b'3,O:1\n'])
node.read_serial_callback()
check('3조각 복원', ('R', 'R:15.3,O:1') in got, str(got))

print('\n[4] 한 번에 여러 줄이 붙어 들어온 경우')
got.clear(); node._rx_buf = b''; node._line_synced = True
node.ser = FakeSerial([b'J:1,B:0\nR:2.0,O:0\nT:25.0,H:60.0,F:0,R:15.3,O:1\n'])
node.read_serial_callback()
check('3줄 모두 처리', len(got) == 3, str(got))
check('종류별 분류 정확',
      got[0][0]=='J' and got[1][0]=='R' and got[2][0]=='T', str(got))

print('\n[5] 보드 상태 메시지는 경고가 아니라 info')
got.clear(); node._rx_buf = b''; node._line_synced = True; node._log.msgs.clear()
node.ser = FakeSerial([b'[\xec\x84\xbc\xec\x84\x9c\xeb\xb3\xb4\xeb\x93\x9c] \xec\x8b\x9c\xec\x9e\x91\xeb\x90\xa8\n'])
node.read_serial_callback()
check('텔레메트리로 오파싱 안 함', len(got) == 0, str(got))
check('info 로 기록', any(m[0]=='info' for m in node._log.msgs), str(node._log.msgs))

print('\n[6] 조각이 과도하게 길면 폐기 (메모리 보호)')
node._rx_buf = b''; node._line_synced = True; node._log.msgs.clear()
node.ser = FakeSerial([b'X'*5000, b''])
node.read_serial_callback()
check('버퍼 폐기됨', node._rx_buf == b'', f'len={len(node._rx_buf)}')
check('경고 기록', any('과다' in str(m[1]) for m in node._log.msgs), str(node._log.msgs))


print('\n[7] 포트 개방 직후 라인 중간부터 시작하는 조각 (실측 버그)')
got.clear(); node._rx_buf = b''; node._line_synced = False; node._log.msgs.clear()
node.ser = FakeSerial([b'513,B:0\n', b'J:600,B:1\n'])   # 첫 줄이 뒤토막
node.read_serial_callback()
check('뒤토막은 버려짐', ('T', '513,B:0') not in got and ('J', '513,B:0') not in got, str(got))
check('파싱 실패 경고 없음', not any(m[0]=='warn' for m in node._log.msgs), str(node._log.msgs))
check('다음 정상 줄은 처리됨', ('J', 'J:600,B:1') in got, str(got))

print('\n' + ('실패 없음 — 전부 통과' if not fails else f'실패: {fails}'))
sys.exit(1 if fails else 0)
