# TurtleBot3 기반 방문자 안내 로봇 (COM팀)

> **최종 업데이트: 2026-08-06** — 실제 자율주행 성공(GUIDING→ARRIVED→RETURNING까지 실측 완료),
> 도킹은 정렬 단계 재검증 필요. 아래 내용은 "구현 예정"이 아니라 **실제 동작 확인된 현재 상태** 기준입니다.

TurtleBot3 Burger, ROS 2 Humble, Nav2, ArUco 마커를 이용한 실내 방문자 안내 로봇입니다. 로봇은 방문자를 지정 목적지(강의실1~5, 회의실, 인포, 상담실 = room1~room8)까지 자율주행으로 안내하고, 이동 중 온도·습도·불꽃을 측정하며, 임무 완료 후 **ArUco 마커 기반 정밀 도킹**으로 충전 스테이션에 자동 복귀합니다.

- 저장소: `unitydt-ros2-class2-2026/com_main_project_01` (통합 브랜치: `feature/mission`)
- 팀: COM (김찬중, 권오민)
- 프로젝트 기간: 4주 (7/20 ~ 8/14)
- 핵심 신규 기술: **ArUco 기반 정밀 도킹 상태머신**
- 로봇 실물: `192.168.210.15` (SSH, 계정 `com`), `ROS_DOMAIN_ID=30`

---

## 목차

1. [프로젝트 소개](#1-프로젝트-소개)
2. [서비스 상태머신](#2-서비스-상태머신)
3. [하드웨어 구성](#3-하드웨어-구성)
4. [소프트웨어 구성](#4-소프트웨어-구성)
5. [패키지 구조](#5-패키지-구조)
6. [설치 방법](#6-설치-방법)
7. [빌드 방법](#7-빌드-방법)
8. [실행 방법 (실물 기동 순서)](#8-실행-방법-실물-기동-순서)
9. [토픽 · 서비스 · 액션 목록](#9-토픽--서비스--액션-목록)
10. [지도와 웨이포인트](#10-지도와-웨이포인트)
11. [운영상 주의사항 (SSH·백그라운드 실행)](#11-운영상-주의사항-ssh백그라운드-실행)
12. [알려진 문제 / 미구현 항목](#12-알려진-문제--미구현-항목)
13. [팀원 역할](#13-팀원-역할)

---

## 1. 프로젝트 소개

처음 방문한 실내 공간(강의실·복도)에서 방문자가 목적지를 찾지 못해 발생하는 문제를 해결하는 로봇입니다. 목적지까지 직접 안내하면서 이동 경로상의 환경 상태(온도·습도·불꽃)를 함께 점검·기록합니다.

**구현 및 실측 완료**
- Nav2 기반 자율주행으로 목적지(room1~room8) 안내 — **2026-08-06 실제 주행 성공** (`/start_mission` → 실제 도착 확인, 좌표 오차 3.6cm)
- ArUco 마커 인식 + 사람 감지(HOG) — 구현 완료, Pi4에서 Nav2와 동시 구동 시 CPU 여유가 빠듯함(§12 참고)
- **ARRIVED → RETURNING → DOCKING → DOCKED → READY 자동 사이클** — 2026-08-06 구현·부분 검증(복귀 주행까지 실측 성공, 도킹 정렬은 재검증 필요)
- **PERSON_WAIT**(사람 감지 시 정지·대기) — 2026-08-06 구현(OBSTACLE_STOP과 동일 패턴), 실물 검증 예정
- 후방 초음파 장애물 감지(OBSTACLE_STOP) — 2026-08-06 배선 오류 발견·수정(그 전까지는 실제로 동작한 적이 없었음), 실물 검증 예정
- HMI: TFT 화면 보드 + 조이스틱 메인센서보드 2보드 구성, 한글 UI — 실물 검증 완료
- 음성 안내(`voice_node`) — 노드는 동작하나 `/tts_request`를 발행하는 코드가 없어 실제로는 무음 (§12)
- PyQt 관제 GUI(`com_gui`) — **미구현** (빈 스켈레톤). 목적지 선택은 로봇에 탑재된 TFT+조이스틱으로 함

**미구현**
- 물리 긴급정지 버튼 — 없음. GUI 버튼으로 대체 예정이나 GUI 자체가 미구현이라 긴급정지 기능이 현재 시스템에 없음
- `/env_data` 발행자 없음 — 측정 로그 조회(`/get_measurement_log`)는 항상 빈 결과 반환
- PM2 프로세스 관리 — 도입 안 함. 모든 노드는 SSH로 수동 실행(§8)

---

## 2. 서비스 상태머신

```
SYSTEM_INIT → LOCALIZING → READY → GUIDING → ARRIVED → RETURNING → DOCKING → DOCKED → READY
                                       │                    │
                                       └─(사람 감지)→ PERSON_WAIT
                              (장애물 감지 시 어느 상태에서든)→ OBSTACLE_STOP
                              (긴급정지 버튼, 현재 미구현)→ EMERGENCY_STOP
                              (재시도 초과 등)→ ERROR
```

- `ARRIVED`는 `arrival_wait`(기본 5.0초, TFT 보드 로컬 타이머와 반드시 동일해야 함) 후 자동으로 `RETURNING`으로 전이하고 도킹 지점으로 복귀 주행을 시작합니다.
- `DOCKED`는 `docked_wait`(기본 2.0초) 후 자동으로 `READY`로 복귀합니다.
- 배터리 20% 이하(`battery_return_threshold`) 도달 시 어느 주행 상태에서든 즉시 `RETURNING`으로 전이합니다.
- 실제 상태 전이 로직은 `com_service/com_service/service_manager_node.py`의 `MissionStateMachine`/`SafetyMonitor`를 참고하세요.

---

## 3. 하드웨어 구성

| 구성 요소 | 역할 | 포트/비고 |
|---|---|---|
| TurtleBot3 Burger | 2륜 차동구동 로봇 베이스 (최대 0.22 m/s) | - |
| OpenCR 보드 | 모터 제어 + IMU, `turtlebot3_node` 구동 | `/dev/ttyACM1` (STM32, ROBOTIS) |
| 2D LiDAR (COIN D4) | 전방 장애물 감지 및 AMCL 위치추정 | `/dev/ttyUSB0` (CP210x) |
| Raspberry Pi Camera (imx219) | ArUco 마커 인식·사람 감지 | CSI, `format:=RGB888` 필수(§12) |
| 메인센서보드 (Arduino Uno) | DHT11(온습도)+불꽃+후방 초음파+조이스틱 | `/dev/ttyACM0`, 9600 baud |
| TFT보드 (Arduino Uno) | 3.5인치 TFT 화면(ARS36125D/ILI9486)만 전담, 한글 UI | `/dev/ttyACM2`, 115200 baud |
| Raspberry Pi 4 | 온보드 컴퓨터 | 모든 ROS 2 노드 수동 기동(PM2 미사용) |
| 충전 스테이션 | ArUco 마커(ID 29) 부착, 정밀 도킹 대상 | `waypoints.yaml`의 `docking` 웨이포인트 |
| 관제 PC/노트북 | 개발/미러용 워크스페이스, `docs/`, docx 열람 등 | 로봇 핵심 기능과 무관 |

> ⚠️ 포트 번호(`ttyACM0/1/2`, `ttyUSB0`)는 재부팅·재연결 시 바뀔 수 있습니다. 매번 `lsusb` + `udevadm info -q property -n /dev/ttyACMx`로 재확인하세요. Uno가 2개(메인센서보드/TFT보드)라 장치번호만으로는 구분 안 되고, 실제로 보내는 데이터(9600 baud 텔레메트리 vs 115200 baud `READY`)로 식별해야 합니다.

---

## 4. 소프트웨어 구성

| 구분 | 내용 |
|---|---|
| OS | Ubuntu 22.04 LTS (Raspberry Pi 4, 관제 PC 동일) |
| 미들웨어 | ROS 2 Humble Hawksbill (DDS: Fast-DDS) |
| 내비게이션 | Nav2 (bt_navigator, planner_server, controller_server, amcl, map_server, costmap 2종). `com_navigation/config/nav2_params.yaml`로 Pi4 CPU에 맞게 파라미터 튜닝됨(§12) |
| SLAM | slam_toolbox (사전 지도 작성 단계에서만 사용) |
| 비전 | OpenCV + ArUco 모듈 (`DICT_4X4_50`) |
| GUI | PyQt5 — **미구현** |
| 프로세스 관리 | 없음(PM2 도입 안 함) — 전부 수동 `ros2 run`/`ros2 launch` |
| 음성 | 로컬 wav 재생(`aplay`) — `/tts_request` 발행자가 없어 현재 무음 |
| 하드웨어 브릿지 | pyserial 기반 시리얼 통신 (com_sensors 패키지) |

---

## 5. 패키지 구조

```
com_ws/
└── src/
    ├── com_bringup/      # navigation.launch.py (Nav2 기동, nav2_params.yaml 사용)
    ├── com_navigation/   # config/waypoints.yaml, aruco_markers.yaml, nav2_params.yaml, maps/map.yaml
    ├── com_description/  # 빈 스켈레톤 (미구현)
    ├── com_interfaces/   # 커스텀 msg/srv (StartMission, RobotState, ArucoMarker, EnvData, TtsRequest, GetMeasurementLog, SystemError)
    ├── com_sensors/      # 실물 검증 완료
    │                     #   ├─ serial_bridge_node.py : 메인센서보드(DHT11+불꽃+초음파+조이스틱), 엔트리포인트 com_sensor_bridge_node
    │                     #   └─ hmi_bridge_node.py     : TFT보드 브릿지(화면 전환, 배터리/WiFi 표시, 조이스틱 릴레이)
    ├── com_vision/       # camera + aruco 인식·정밀도킹 오차계산·사람감지, 실물 검증 완료
    │                     #   └─ aruco_node.py : /mission_state 구독해 DOCKING 중엔 사람감지 스킵(CPU 최적화)
    ├── com_gui/          # 빈 스켈레톤 (미구현) — 목적지 선택은 TFT+조이스틱으로 대체
    ├── com_launch/       # 빈 스켈레톤 (미구현, PM2 관련 계획 폐기)
    ├── com_service/      # 임무 상태머신 + 음성
    │                     #   ├─ mission_state.py       : 상태 14종 정의
    │                     #   ├─ service_manager_node.py: 상태머신, WaypointSequencer, DockingController, SafetyMonitor
    │                     #   └─ voice_node.py          : /tts_request 구독 → aplay 재생 (발행자 없어 현재 미사용)
    ├── com_status/       # status_node.py : /robot_state, /mission_state, /current_waypoint_id 발행, 측정 로그 서비스
    └── coin_d4_driver/   # LiDAR(COIN D4) 서드파티 드라이버 (별도 git 서브트리)
```

---

## 6. 설치 방법

**① ROS 2 Humble 설치** (라즈베리파이 4)
```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-ros-base python3-argcomplete python3-colcon-common-extensions python3-rosdep
sudo rosdep init && rosdep update

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
echo "export LDS_MODEL=LDS-03" >> ~/.bashrc
source ~/.bashrc
```

**② TurtleBot3 · LiDAR(COIN D4) 패키지 설치**
```bash
sudo apt install -y ros-humble-turtlebot3 ros-humble-turtlebot3-msgs \
  ros-humble-dynamixel-sdk ros-humble-hls-lfcd-lds-driver \
  ros-humble-turtlebot3-teleop ros-humble-turtlebot3-navigation2
sudo usermod -aG dialout $USER   # OpenCR/아두이노 USB 권한 (재접속 필요)
```

**③ 카메라 · Nav2 · SLAM · OpenCV**
```bash
sudo apt install -y ros-humble-camera-ros ros-humble-camera-calibration \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-cv-bridge
pip3 install opencv-contrib-python pyserial --break-system-packages
```

**④ 워크스페이스 클론**
```bash
mkdir -p ~/com_ws/src
cd ~/com_ws
git clone git@github.com:unitydt-ros2-class2-2026/com_main_project_01.git .
```

> GUI/PM2 설치 단계는 삭제했습니다 — 둘 다 이 프로젝트에서 최종적으로 사용하지 않습니다(§12 참고).

---

## 7. 빌드 방법

```bash
cd ~/com_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

> ⚠️ **`com_navigation`을 먼저 빌드하지 않으면 `service_manager_node`가 `PackageNotFoundError`로 즉시 죽습니다** (`waypoints.yaml`을 `com_navigation`의 install share 경로에서 읽기 때문). `colcon build`를 패키지 전체 대상으로 한 번에 돌리면 의존성 순서를 colcon이 알아서 맞추므로 문제 없지만, 개별 패키지만 부분 빌드할 때는 순서를 챙기세요.

패키지 개별/선택 빌드:
```bash
colcon build --symlink-install --packages-select com_interfaces com_navigation com_sensors com_service com_status com_vision
```

`setup.py`의 `entry_points`(노드 실행 커맨드)를 바꿨는데 반영이 안 될 때 — `--symlink-install`은 지워진 entry_points의 wrapper 스크립트를 자동으로 안 지웁니다:
```bash
rm -rf build/<패키지명> install/<패키지명>
colcon build --symlink-install --packages-select <패키지명>
ros2 pkg executables <패키지명>   # 실제 등록된 실행파일 확인
```

---

## 8. 실행 방법 (실물 기동 순서)

로봇(`192.168.210.15`)에 SSH로 접속한 뒤, **아래 순서대로** 별도 터미널(또는 백그라운드, §11 참고)에서 실행합니다.

### 8-1. 베이스 + 라이다 + OpenCR

```bash
source /opt/ros/humble/setup.bash
source ~/com_ws/install/setup.bash
export LDS_MODEL=LDS-03
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```
`LDS_MODEL=LDS-03`이면 `turtlebot3_bringup`이 `coin_d4_driver`를 자동으로 포함해서 베이스+라이다가 한 번에 뜹니다. **배터리는 12V 근처(완충에 가까운 상태)로 충전한 후 기동하세요** — 11V대 저전압에서는 OpenCR이 약 90초 후 시리얼 통신 실패로 크래시하는 게 실측 확인됐습니다.

### 8-2. Nav2

```bash
source /opt/ros/humble/setup.bash
source ~/com_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch com_bringup navigation.launch.py
```
기본 파라미터 파일은 `com_navigation/config/nav2_params.yaml`(Pi4 CPU에 맞게 튜닝됨)입니다. 원래 turtlebot3_navigation2 기본값으로 띄우고 싶으면 `params_file:=` 인자로 오버라이드하세요.

**초기 위치 설정 (AMCL)** — 로봇이 지도 상 어디 있는지 알려줘야 합니다:
```bash
ros2 topic pub --once --qos-reliability best_effort /initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: <로봇의 실제 x>, y: <실제 y>, z: 0.0}, \
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, \
  covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}"
```
> ⚠️ AMCL의 `/initialpose` 구독은 **BEST_EFFORT** QoS입니다. `--qos-reliability best_effort`를 빼먹으면 "publisher: beginning loop"까지는 뜨는데 AMCL이 영원히 못 받습니다(경고 로그만 반복). 한 번에 안 먹히면 `-r 2`로 잠깐(2~4초) 반복 발행하세요.

### 8-3. 비전 (카메라 + ArUco)

```bash
source /opt/ros/humble/setup.bash
source ~/com_ws/install/setup.bash
ros2 launch com_vision aruco.launch.py
```
카메라는 `format:=RGB888`로 고정돼 있어야 `cv_bridge`가 프레임을 읽습니다(기본 NV12/NV21은 인식 못 함). Nav2와 동시에 돌리면 Pi4 CPU가 빠듯할 수 있습니다 — §12 참고.

### 8-4. 센서 + HMI

```bash
source /opt/ros/humble/setup.bash
source ~/com_ws/install/setup.bash
ros2 run com_sensors com_sensor_bridge_node                                    # 메인센서보드, 기본 /dev/ttyACM0
ros2 run com_sensors hmi_bridge_node --ros-args -p serial_port:=/dev/ttyACM2   # TFT보드, 반드시 포트 오버라이드
```
`hmi_bridge_node`의 `serial_port` 기본값은 `/dev/ttyACM0`(메인센서보드와 겹침)이므로 **반드시 `/dev/ttyACM2`로 오버라이드**해야 합니다.

### 8-5. 임무 상태머신 + 상태 종합 + 음성

```bash
source /opt/ros/humble/setup.bash
source ~/com_ws/install/setup.bash
ros2 run com_service service_manager_node
ros2 run com_status status_node
ros2 run com_service com_voice_node
```

### 8-6. 임무 시작

로봇에 탑재된 TFT+조이스틱으로 목적지를 선택하거나, 서비스를 직접 호출:
```bash
ros2 service call /start_mission com_interfaces/srv/StartMission "{destination_id: room1}"
```
유효한 `destination_id`는 `room1`~`room8`입니다. `state == READY`일 때만 수락됩니다.

### 상태 확인용 명령 모음

```bash
ros2 node list                                    # 전체 노드 기동 확인
ros2 topic echo /mission_state --once              # 현재 상태머신 상태
ros2 topic echo /robot_state --once                 # 위치/배터리/상태 종합
ros2 topic echo /battery_state --once               # OpenCR 배터리 원본값(전압/퍼센트)
ros2 run tf2_ros tf2_echo map base_footprint         # 로봇 현재 위치(맵 기준)
ros2 service call /cancel_mission std_srvs/srv/Trigger   # 임무 취소 → READY 복귀
```

---

## 9. 토픽 · 서비스 · 액션 목록

### 실제 동작 확인된 것

| 구분 | 이름 | 타입 | 비고 |
|---|---|---|---|
| Topic | `/com_sensors/temperature`, `/humidity`, `/flame_detected` | Float32/Bool | BEST_EFFORT, 메인센서보드 |
| Topic | `/com_sensors/rear_distance`(Float32, cm), `/rear_obstacle`(Bool) | - | 후방 초음파, BEST_EFFORT |
| Topic | `/com_sensors/joystick_x`(Int32), `/joystick_button`(Bool) | - | BEST_EFFORT |
| Topic | `/aruco_marker` | com_interfaces/ArucoMarker | marker_id, pose, lateral_error, distance_error |
| Topic | `/person_detected` | std_msgs/Bool | DOCKING 상태에서는 발행 안 함(최적화) |
| Topic | `/state_code` | std_msgs/UInt8 | 상태 전이 시에만 발행(주기적 아님) |
| Topic | `/mission_state`, `/current_waypoint_id` | std_msgs/String | 1Hz 재발행 |
| Topic | `/battery_percent` | std_msgs/Float32 | 1Hz |
| Topic | `/robot_state` | com_interfaces/RobotState | status 노드, 2Hz |
| Topic | `/battery_state` | sensor_msgs/BatteryState | OpenCR(`turtlebot3_node`) 발행, service_manager가 직접 구독 |
| Service | `/start_mission` | com_interfaces/StartMission | destination_id: room1~room8 |
| Service | `/cancel_mission` | std_srvs/Trigger | READY로 직접 복귀 |
| Service | `/get_measurement_log` | com_interfaces/GetMeasurementLog | `/env_data` 발행자가 없어 **항상 빈 결과** |
| Action | `/navigate_to_pose` | nav2_msgs/action/NavigateToPose | service_manager → Nav2 |

### 발행자가 없어 사용 불가능한 것 (구독 코드는 있음)

| 이름 | 이유 |
|---|---|
| `/button_event` | 물리 긴급정지 버튼이 실제로 없음. GUI 버튼 도입 시 재사용 예정 |
| `/env_data` | 온습도 데이터를 waypoint_id와 묶어 재발행하는 다리 노드가 없음 |
| `/tts_request` | `service_manager`가 상태 전이 시 이걸 발행하는 코드가 없음(voice_node는 떠 있지만 무음) |

전체 커스텀 msg/srv 정의는 `com_interfaces/msg/`, `com_interfaces/srv/`를 참고하세요.

---

## 10. 지도와 웨이포인트

- 지도: `com_navigation/maps/map.yaml` (+ `map.pgm`), SLAM으로 사전 작성.
- 웨이포인트: `com_navigation/config/waypoints.yaml` — `id`, `label`, `x`, `y`, `yaw`, (목적지면 `is_destination: true`).
  - 목적지(8곳): `room1`(강의실1) ~ `room5`(강의실5), `room6`(회의실), `room7`(인포), `room8`(상담실)
  - 특수 지점: `docking`(충전 스테이션 정렬점), `docking2`
  - 나머지(`way6`~`way16`)는 경유점(현재 `WaypointSequencer`는 `trajectory_1`/`trajectory_2` 키가 정의 안 되어 있어 **직행** — 경유점을 거치지 않고 목적지로 바로 이동함)
- ArUco 마커: `com_navigation/config/aruco_markers.yaml` — 현재 도킹 마커(ID 29)만 등록. 목적지 도착 판정은 마커가 아니라 Nav2 좌표 기반.
- 도킹 마커 사양: `DICT_4X4_50`, 10cm x 10cm.

---

## 11. 운영상 주의사항 (SSH·백그라운드 실행)

- **SSH 원격 세션에서 `pkill -f <이름>`을 쓰지 마세요.** 원격 명령 문자열 자체에 그 이름이 들어가 있어서 pkill이 부모 bash(자기 자신)를 잡아 SSH 세션이 통째로 끊기는 사고가 실제로 있었습니다. `ps aux | grep '[n]ode이름'`으로 PID를 찾아 `kill <pid>`로 지정 종료하세요.
- **백그라운드 기동은 노드 하나당 SSH 명령 하나로 분리하세요.** `setsid nohup ros2 run ... > log 2>&1 < /dev/null &` 패턴으로 완전히 분리해야 SSH 세션 종료 후에도 살아있습니다. 여러 노드를 `disown`과 함께 한 명령에 몰아넣으면 원격 bash가 `wait()`에 멈춰서 첫 노드 뒤로 아무것도 안 뜨는 문제가 실측됐습니다 — 절대 한 명령에 여러 노드를 묶지 마세요.
- 이런 방식으로 백그라운드 기동한 명령은 로컬 도구가 120초 타임아웃으로 "백그라운드로 이동"시킬 수 있는데, 이건 SSH 연결 자체의 특성일 뿐이고 원격 프로세스는 이미 정상적으로 떠 있습니다. 새 SSH 연결로 `ps aux`나 로그 파일을 확인해서 실제 기동 여부를 검증하세요.
- 노드를 여러 번 껐다 켜면 `ros2 node list`에 노드가 하나도 안 뜨거나 CLI 명령(`topic echo`, `topic hz`)이 응답 없이 멈추는 경우가 있었습니다 — `ros2 daemon stop && ros2 daemon start`로 대체로 해결됩니다(DDS 디스커버리 캐시 오염으로 추정).

---

## 12. 알려진 문제 / 미구현 항목

| 문제 | 영향 | 상태 |
|---|---|---|
| Nav2 + 비전(카메라+ArUco+사람감지)을 Pi4 한 대에서 동시 구동 시 CPU 부하(load average 5~7대, 4코어 기준)로 `/tf`·`/scan`이 간헐적으로 끊김 | 주행 중 정지·타임아웃 가능성 | `nav2_params.yaml` 튜닝으로 완화(load 6~7→1.5~3), 사람감지까지 같이 켜면 여전히 부하 있음 - 완전 해결 아님 |
| 물리 긴급정지 버튼 없음 | 긴급정지 기능이 현재 시스템에 아예 없음 | GUI 버튼으로 대체 예정, `com_gui` 미구현이라 보류 |
| `/env_data` 발행자 없음 | 측정 로그 조회가 항상 빈 결과 | 미착수 (온습도+waypoint_id를 묶는 다리 노드 필요) |
| `/tts_request` 발행자 없음 | 음성 안내가 실제로는 무음 | 미착수 |
| `WaypointSequencer`의 `trajectory_1`/`trajectory_2` 경유점 시퀀스 미정의 | 출발지→목적지, 목적지→도킹이 경유점 없이 직행(Nav2 자체 경로계획에 의존) | 설계상 문제는 아니나 문서(SR-006)와는 다름 |
| ArUco 도킹 정렬 신뢰성 | 다중 마커 동시 검출 시 잘못된 마커를 보고하던 버그는 수정(2026-08-06)했으나 실물 재검증 필요 | 다음 세션에서 재시도 필요 |
| PM2 미도입 | 노드 비정상 종료 시 자동 재시작 없음 | 수동 운영으로 대체, 재도입 여부 미정 |
| libcamera 0.1(vc4 파이프라인)에서 `orientation` 파라미터 미지원 | 카메라 프레임이 상하 반전되어 들어옴 | `aruco_node.py`에서 `cv2.rotate(frame, cv2.ROTATE_180)`로 코드에서 보정 처리 완료 |
| `coin_d4_driver` 전체 빌드 시 간헐적 Aborted | 통합 빌드 시간 지연 가능 | 미해결, 재확인 필요 |

---

## 13. 팀원 역할

| 김찬중 | 권오민 |
|---|---|
| 서류작성 (문서 7종 + 발표 PPT) | 인터페이스: HMI(TFT+조이스틱), 목적지 메뉴 UI |
| 환경 세팅, bringup + camera(OpenCV 캘리브레이션) | Nav2·SLAM 준비, 하드웨어 조립, 실공간 SLAM 매핑 |
| voice(음성 안내, `/tts_request` 배선은 미완) | aruco(마커 인식·정밀 도킹 오차 계산·사람 감지) |
| **service_manager**(상태머신, ArUco 정밀 도킹 로직) — 핵심 신규 기술 | sensor_bridge, status(배터리 %, 측정 로그, 상태 발행) |

**공동 작업**
- `com_interfaces`(RobotState/ArucoMarker/EnvData/StartMission 등 메시지 정의)
- 통합 테스트(8/3, 8/4, 8/5, 8/6 진행) 및 시험 결과 기록, 발표 리허설(8/13~8/14 예정)

> 상세 일일 진행 상황은 `docs/worklog/` 폴더의 작업일지를 참고하세요.
