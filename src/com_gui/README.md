# COM 관제 대시보드 (com_gui v2.0)

TurtleBot3 기반 실내 자율주행 방문자 안내 로봇의 PyQt5 관제 GUI.
다크 테마 대시보드로 모니터링 + 임무 제어 + 디버그용 수동 주행을 제공한다.

## 파일 구성

| 파일 | 역할 |
|---|---|
| `theme.py` | 색상 팔레트 + 전체 스타일시트(QSS). 색을 바꾸려면 여기만 고치면 됨 |
| `widgets.py` | 직접 그리는 커스텀 위젯 (배터리 도넛, 스파크라인, 맵뷰, 카메라뷰, 센서바) |
| `ros_worker.py` | ROS2 통신 전담 QThread. 모든 구독/발행/서비스호출 |
| `main_window.py` | 대시보드 레이아웃 및 화면 갱신 로직 |
| `gui_node.py` | 실행 진입점 (`ros2 run com_gui gui_node`) |

## 구독하는 토픽

| 토픽 | 타입 | 쓰이는 곳 |
|---|---|---|
| `/robot_state` | com_interfaces/RobotState | 헤더, 로봇 상태, 배터리, 맵 위 로봇 위치 |
| `/map` | nav_msgs/OccupancyGrid | SLAM 맵 뷰 |
| `/camera/image_raw` | sensor_msgs/Image | 카메라 실시간 영상 |
| `/scan` | sensor_msgs/LaserScan | LiDAR 스캔 상태/최소거리 |
| `/rear_ir` | sensor_msgs/Range | 후방 거리 게이지 |
| `/flame_detected` | std_msgs/Bool | 불꽃 감지 (경고 배너 + 알람) |
| `/person_detected` | std_msgs/Bool | 사람 감지 (SR-012) |
| `/env_data` | com_interfaces/EnvData | 온도/습도 게이지 |
| `/system_error` | com_interfaces/SystemError | 알람·이벤트 |

`/env_data`, `/system_error` 메시지가 `com_interfaces`에 아직 없어도
GUI는 정상 실행되며, 해당 패널만 값이 안 들어온다 (선택적 import 처리).

## 설치 & 실행 (개발 PC)

```bash
sudo apt install python3-pyqt5 python3-numpy

cp -r com_gui ~/com_ws/src/
cd ~/com_ws
colcon build --packages-select com_gui
source install/setup.bash

export ROS_DOMAIN_ID=30
ros2 run com_gui gui_node
```

## 디버그 수동 주행 안전장치

`/cmd_vel`은 아래 두 조건이 **모두** 참일 때만 발행된다:

1. `mission_state == 'READY'`
2. "수동모드 활성화" 체크박스 ON

임무가 시작되면 체크박스가 자동으로 해제된다. Nav2와 `/cmd_vel`이
충돌하면 로봇이 예측 불가능하게 움직이므로 반드시 지켜야 한다.
속도는 `MANUAL_LINEAR_SPEED=0.10 m/s`로 제한 (SR-032: 최대 0.22 m/s).

HMI-001 6장에 따라 **긴급정지 버튼은 GUI에 두지 않는다.** 긴급정지는
하드웨어 버튼(길게 누름)으로만 동작한다 (SR-021).

## 확인 필요 사항

- `DESTINATIONS`의 코드값(`LECTURE1` 등)은 `service_manager`가 실제 쓰는
  ID와 일치해야 함. `com_navigation/config/waypoints.yaml`과 대조할 것.
- 카메라가 물리적으로 180도 뒤집혀 있으므로, `camera_rotator.py` 적용 전에는
  영상이 거꾸로 보일 수 있음.


## 불꽃감지센서 연동

GUI는 `/flame_detected` (std_msgs/Bool) 토픽을 구독한다.
`sensor_bridge` 노드에서 불꽃감지센서 값을 아래처럼 발행하면 된다.

```python
from std_msgs.msg import Bool

self.flame_pub = self.create_publisher(Bool, '/flame_detected', 10)

msg = Bool()
msg.data = flame_is_detected   # True = 불꽃 감지
self.flame_pub.publish(msg)
```

동작:
- `True`  → 센서 카드 상단에 빨간 경고 띠가 나타나 1초 간격으로 깜빡이고,
            알람 목록과 타임라인에 기록되며, 상태바에 화재 경보 표시
- `False` → "불꽃 미감지" 초록 배지
- 3초 이상 미수신 → "미수신" 회색 배지 (센서 고장/연결 끊김 구분용)

알람은 감지가 **시작되는 순간에만** 한 번 기록된다. 계속 True로 들어와도
목록이 도배되지 않는다.

> 대부분의 불꽃감지센서(KY-026 등) 디지털 출력은 **감지 시 LOW**다.
> 아두이노 코드에서 `digitalRead(FLAME_PIN) == LOW` 가 감지임을 확인할 것.

## PM2 프로세스 관리 (임무 이력 타임라인 하단)

우측 컬럼 맨 아래(임무 이력 타임라인 카드 바로 밑)에 pm2 프로세스를
개별로, 또는 한 번에 시작/종료/재시작하는 카드가 있다. **임무 제어
카드의 목적지 콤보박스와 똑같은 방식**이다 — 콤보에서 대상을 고르고
버튼을 누른다.

- 콤보에는 라즈베리파이의 pm2 프로세스 8개(`main_window.PM2_PROCESSES`:
  `bringup`, `hmi_bridge`, `navigation`, `sensor_bridge`,
  `service_manager`, `status`, `vision`, `voice`)가 들어있고, 맨 아래
  **"전체 프로세스"** 를 고르면 8개를 한 번에 제어한다. 목록이 바뀌면
  `PM2_PROCESSES`만 고치면 된다.
- **시작(초록) / 종료(빨강) / 재시작(주황)**: 버튼마다 색을 다르게 줘서
  실수로 잘못 누르는 걸 줄였다. 클릭하면 "OO 프로세스를 OO할까요?"
  확인창이 한 번 뜨고, 콤보에서 고른 대상으로 `pm2 <action> <대상>`을
  `QProcess`로 비동기 실행한다.
- 명령이 실행되는 동안 세 버튼은 모두 비활성화되고, 끝나면 결과가
  카드 하단 캡션·상태바·임무 이력 타임라인에 반영된다. 실패하면
  빨간 알람도 함께 남는다.

GUI는 개발 PC에서 뜨고 pm2는 로봇(라즈베리파이4)에서 돌기 때문에,
버튼은 로컬 pm2가 아니라 **SSH로 라즈베리파이에 접속해서** 그쪽 pm2에
명령을 보낸다 (`main_window.PM2_SSH_USER` / `PM2_SSH_HOST`. 접속 정보는
환경변수 `COM_ROBOT_SSH_USER` / `COM_ROBOT_SSH_HOST`로 바꾼다).

- 카드에는 항상 "대상: ssh → user@host" 캡션이 떠 있어서, 실제로 어디로
  명령이 나가는지 화면에서 바로 확인할 수 있다.
- `PM2_TIMEOUT_MS`(기본 15초) 안에 응답이 없으면 강제로 중단하고 버튼을
  되살린다 — SSH가 멈춰서 GUI 전체가 안 눌리는 상황을 막기 위한 안전장치.

### 필수 사전 조건: SSH 키 기반 무비밀번호 로그인

GUI(`QProcess`)는 터미널이 아니라서 SSH 비밀번호 프롬프트에 응답할 수
없다. **미리 키 인증을 설정해두지 않으면 버튼이 항상 실패한다**
(`-o BatchMode=yes`를 넣어뒀기 때문에 무한정 멈추지는 않고 바로
"Permission denied" 로 실패하긴 한다).

개발 PC에서:
```bash
ssh-keygen -t ed25519           # 이미 키가 있으면 생략
ssh-copy-id <계정>@<로봇 주소>  # 라즈베리파이 실제 계정/주소로
ssh <계정>@<로봇 주소> 'pm2 list'   # 비밀번호 안 물어보고 바로 목록이 나오면 성공
```

### 자주 겪는 문제

- **"Process or Namespace ... not found" / "No process found"**
  라즈베리파이 터미널에서 `pm2 list`는 되는데 GUI만 이 에러를 내면,
  대부분 GUI가 (SSH 설정 전이라) **로컬 PC의 pm2**를 부르고 있었거나,
  라즈베리파이 쪽 SSH 로그인 계정이 pm2를 등록한 계정과 다른 경우다.
  카드에 뜨는 "대상: ..." 캡션의 계정이 라즈베리파이에서 실제로
  `pm2 list`가 되는 계정과 같은지 확인할 것.
- **그래도 안 되면**: `main_window.PM2_HOME_OVERRIDE`에 라즈베리파이
  쪽 실제 `$PM2_HOME`(보통 `/home/<계정>/.pm2`)을 채워 넣으면, SSH
  로그인 셸의 기본값과 무관하게 항상 그 경로를 강제로 쓴다.
