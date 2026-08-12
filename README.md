# TurtleBot3 기반 방문자 안내 로봇 (COM팀)

> **최종 업데이트: 2026-08-12.** 아래 내용은 "구현 예정"이 아니라 **실제 동작 확인된 현재 상태** 기준입니다.
> 8/11: 음성 안내(`/tts_request`) 실제 연결, 사람감지 CPU 최적화, `com_launch` 통합 launch 파일 +
> **PM2 자동기동/자동재시작(`ecosystem.config.js`)**, USB 시리얼 포트 udev 고정,
> `hmi_bridge_node`/`com_sensor_bridge_node` 크래시 버그 수정, **`/initialpose` 레이스 문제
> 해결**(`set_initial_pose` 파라미터 방식). 다만 같은 날 실물 테스트로 **AMCL이 `map→odom` tf를
> 지속 갱신하지 못해 Nav2가 실제 이동 없이 "목표 도달"로 오판하는 별도 버그를 새로 발견**(§12) —
> 8/6 기록된 "실주행 성공"은 실제 이동 시간·최종 위치 오차로 검증된 것이라 유효하지만, **이 버그가
> 해결되기 전까지는 매번 실주행 여부를 직접 확인 없이 `/start_mission` 결과를 신뢰하면 안 됩니다.**
> 8/12: USB 오디오 동글 재생 끊김(볼륨 100%→80%로 완화), 시리얼 최초연결 재시도 + `in_waiting`
> ioctl 에러 제거, PM2 실제 가동 확인, `/com_gui_node` 실행 중인 것 발견(구현 범위 미확인).
> 다음 세션 재개 절차는 바로 아래 참고.

## 🔋 다음 세션 재개 절차 (2026-08-11 세션, 배터리 충전 후 이어서)

1. **배터리 충전 확인** (12V 근처 권장, §8-1 참고):
   ```bash
   ssh com@192.168.210.15
   source /opt/ros/humble/setup.bash && source ~/com_ws/install/setup.bash
   ros2 topic echo /battery_state --once | grep -E "voltage|percentage"
   ```
2. **로봇을 충전 스테이션 앞 정렬점에 둔 상태인지 확인** — `nav2_params.yaml`의 `set_initial_pose` 좌표(도킹 지점 x=1.079, y=0.901)가 이 위치를 전제로 합니다. 다른 곳에 있다면 §8-2 "AMCL 상태 확인" 절차를 보기 전에 좌표부터 실제 위치로 갱신하세요.
3. **기존 프로세스가 남아있으면 정리** (이 세션에서 노드를 개별적으로 많이 재시작해서 지저분한 상태일 수 있음):
   ```bash
   ps aux | grep -E "com_ws/install|turtlebot3_ros|component_container|camera_ros" | grep -v grep
   # 나온 PID들을 개별적으로 kill (pkill -f 절대 쓰지 말 것! §11 참고)
   ```
   확실하지 않으면 그냥 `sudo reboot`로 재부팅하는 게 제일 깔끔합니다(오늘 세션에서도 이 방법으로 여러 문제가 정리됨).
4. **전체 스택 기동**:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/com_ws/install/setup.bash
   export TURTLEBOT3_MODEL=burger
   export LDS_MODEL=LDS-03
   setsid nohup ros2 launch com_launch robot_all.launch.py > ~/com_launch.log 2>&1 < /dev/null &
   disown
   ```
   20~30초 기다린 뒤 확인:
   ```bash
   grep -iE "died|Traceback" ~/com_launch.log | grep -v "No such device\|audio open error"
   ```
   아무것도 안 뜨면 정상. 뭔가 죽었으면(특히 `camera_node`가 "Device or resource busy"로 죽는 경우가 있었음 - 이전 세션에서 정리 안 된 프로세스가 카메라를 붙잡고 있을 때) 해당 노드만 개별적으로 재시작(§8-3~8-5 패턴 참고).
5. **AMCL이 실제로 잘 도는지 확인 (§8-2 "AMCL 상태 확인" 필수 — 이거 안 하면 8/11에 발견된 "거짓 도착" 버그를 또 만날 수 있음)**:
   ```bash
   ros2 run tf2_ros tf2_echo map odom     # 계속 갱신되는지
   ros2 topic hz /amcl_pose                # 계속 갱신되는지 (Ctrl+C로 종료)
   ```
   둘 다 멈추지 않고 계속 갱신되면 안전하게 `/start_mission` 테스트 진행. 멈춰있거나 몇십 초씩 안 갱신되면 §12의 "AMCL이 map→odom tf를 지속 갱신 못 함" 항목부터 다시 조사할 것 — **이 상태에서 주행 테스트하면 실제로 안 움직였는데 도착으로 오판할 수 있음.**
6. **음성 안내가 필요하면 스피커 확인**: USB 오디오 동글(Jieli UACDemoV1.0)이 하드웨어적으로 불안정한 상태로 남아있습니다(§12). **2026-08-12: 볼륨 100%(유니티 게인)에서 재생 중 끊김이 재현 확인돼 기본값을 80%로 낮춤 - 이걸로 재생 자체는 안정화됐지만 하드웨어 불안정성 자체가 없어진 건 아니니 여전히 `aplay`로 먼저 확인 권장.**

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
- ArUco 마커 인식 + 사람 감지(HOG) — 구현 완료. 2026-08-11에 사람감지 주기를 마커 검출(5Hz)과 분리(기본 1Hz)해 CPU 사용량 약 23% 절감(67.5%→52%, `aruco_node` 단독 기준). 다만 Pi4에서 Nav2와 동시 구동 시 여전히 여유가 빠듯함(§12 참고)
- **ARRIVED → RETURNING → DOCKING → DOCKED → READY 자동 사이클** — 2026-08-06 구현·부분 검증(복귀 주행까지 실측 성공, 도킹 정렬은 재검증 필요)
- **PERSON_WAIT**(사람 감지 시 정지·대기) — 2026-08-06 구현(OBSTACLE_STOP과 동일 패턴), 실물 검증 예정
- 후방 초음파 장애물 감지(OBSTACLE_STOP) — 2026-08-06 배선 오류 발견·수정(그 전까지는 실제로 동작한 적이 없었음), 실물 검증 예정
- HMI: TFT 화면 보드 + 조이스틱 메인센서보드 2보드 구성, 한글 UI — 실물 검증 완료
- 음성 안내(`voice_node`) — 2026-08-11에 `service_manager_node`가 주요 상태 전이(안내 시작/도착/복귀/취소/장애물/사람대기 등) 시점에 `/tts_request`를 실제로 발행하도록 연결 완료, 더 이상 무음 아님. 재생 문구는 gTTS로 생성(`com_service/scripts/generate_audio.py`, 재실행하면 재생성 가능). 연달아 겹쳐 재생 요청되면 이전 재생이 끝날 때까지 큐잉 후 순차 재생(이전엔 ALSA 충돌로 뒤 안내가 씹히는 버그가 있었음, 수정 완료)
- PyQt 관제 GUI(`com_gui`) — **문서상 미구현(빈 스켈레톤)으로 남아있었으나, 2026-08-12에 `ros2 node list`에서 `/com_gui_node`가 실제로 떠 있는 것을 확인함.** 누가/언제 작업했는지, 실제 구현 범위가 어디까지인지는 이 세션에서 확인 못 함 - 다음에 GUI 얘기 나오면 먼저 실물 확인 필요. 목적지 선택은 여전히 로봇 탑재 TFT+조이스틱으로 함

**미구현**
- 물리 긴급정지 버튼 — 없음. GUI 버튼으로 대체 예정이나 GUI 실제 구현 범위 미확인이라 긴급정지 기능이 현재 확실히 동작한다고 보기 어려움
- `/env_data` 발행자 없음 — 측정 로그 조회(`/get_measurement_log`)는 항상 빈 결과 반환

**PM2 프로세스 관리 (2026-08-11 도입, 2026-08-12 실측 확인)**
- `com_launch/ecosystem.config.js` + `pm2-com.service`(systemd)로 전원 인가 시 8개 노드(bringup/navigation/vision/sensor_bridge/hmi_bridge/service_manager/status/voice) 자동 기동, 노드별 최대 3회 자동 재시작(SR-026).
- 2026-08-12 실측: 8개 전부 `online` 상태로 장시간 안정 가동 확인(`pm2 list`). **주의**: `pm2 save`로 저장된 마지막 상태가 "전부 stopped"였던 적이 있어, 그 상태로 저장된 채 재부팅하면 자동 기동이 아예 안 될 수 있음 - 부팅 후 `pm2 list`로 전부 `online`인지 항상 확인할 것. 문제 있으면 `pm2 start ~/com_ws/src/com_launch/ecosystem.config.js && pm2 save`로 복구.
- `com_launch/robot_all.launch.py`(단일 `ros2 launch` 프로세스로 통합)는 수동 디버깅용 대안으로 계속 존재 - **PM2와 동시에 띄우면 노드 중복 실행으로 충돌(리소스 경합·오디오 장치 경합 등)하니 절대 같이 쓰지 말 것**(2026-08-12 실측: 동시 구동 시 CPU load average 7대까지 치솟고 음성 노드가 반복 크래시하는 것 확인).

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
| 메인센서보드 (Arduino Uno) | DHT11(온습도)+불꽃+후방 초음파+조이스틱 | `/dev/com_sensorboard`(고정 심볼릭 링크), 9600 baud |
| TFT보드 (Arduino Uno) | 3.5인치 TFT 화면(ARS36125D/ILI9486)만 전담, 한글 UI | `/dev/com_tftboard`(고정 심볼릭 링크), 115200 baud |
| Raspberry Pi 4 | 온보드 컴퓨터 | 모든 ROS 2 노드 수동 기동(PM2 미사용) |
| 충전 스테이션 | ArUco 마커(ID 29) 부착, 정밀 도킹 대상 | `waypoints.yaml`의 `docking` 웨이포인트 |
| 관제 PC/노트북 | 개발/미러용 워크스페이스, `docs/`, docx 열람 등 | 로봇 핵심 기능과 무관 |

> ✅ **(2026-08-11) 포트 번호 고정 완료.** `ttyACM0/1/2/3` 같은 커널 할당 번호는 여전히 재부팅·재연결 시 바뀌지만, `com_launch/udev/99-com-robot-serial.rules`(시리얼 번호 기반)를 `/etc/udev/rules.d/`에 설치해두면 `/dev/com_sensorboard`(메인센서보드), `/dev/com_tftboard`(TFT보드) 심볼릭 링크가 항상 올바른 장치를 가리킵니다(OpenCR `/dev/opencr`, 라이다 `/dev/tb3_lidar`는 TurtleBot3 표준 설치가 이미 처리). 새 라즈베리파이로 옮기면 규칙을 다시 설치해야 합니다: `sudo cp com_launch/udev/99-com-robot-serial.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger`. 이 규칙이 없는 환경이라면 여전히 매번 `lsusb` + `udevadm info -q property -n /dev/ttyACMx`로 재확인해야 합니다(Uno가 2개라 장치번호만으로는 구분 안 되고, 실제로 보내는 데이터(9600 baud 텔레메트리 vs 115200 baud `READY`)로 식별).

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
| 음성 | 로컬 wav 재생(`aplay`), 문구는 gTTS로 사전 생성 — `service_manager_node`가 `/tts_request` 발행 (2026-08-11 연결) |
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
    │                     #   └─ aruco_node.py : /mission_state 구독해 DOCKING 중엔 사람감지 스킵(CPU 최적화),
    │                     #                       사람감지는 detection_rate(마커, 5Hz)와 분리된
    │                     #                       person_detection_rate(기본 1Hz)로 실행(2026-08-11)
    ├── com_gui/          # 문서상 빈 스켈레톤이었으나 2026-08-12에 /com_gui_node 실행 확인(범위 미확인) — 목적지 선택은 여전히 TFT+조이스틱
    ├── com_launch/       # launch/robot_all.launch.py : §8의 8단계를 하나의 ros2 launch로 통합(2026-08-11)
    ├── com_service/      # 임무 상태머신 + 음성
    │                     #   ├─ mission_state.py       : 상태 14종 정의
    │                     #   ├─ service_manager_node.py: 상태머신, WaypointSequencer, DockingController, SafetyMonitor,
    │                     #   │                            주요 상태 전이 시 /tts_request 발행
    │                     #   ├─ voice_node.py          : /tts_request 구독 → aplay 순차 재생(큐잉)
    │                     #   └─ scripts/generate_audio.py : gTTS로 audio/*.wav 재생성(개발용, 로봇에서 실행 불필요)
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

**⑤ (2026-08-11 추가) 시리얼 포트 고정용 udev 규칙 설치**
```bash
sudo cp com_ws/src/com_launch/udev/99-com-robot-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```
안 하면 재부팅·재연결마다 `ttyACM` 번호가 바뀌어서 `hmi_bridge_node`가 엉뚱한(또는 존재하지 않는) 포트를 붙잡고 계속 I/O 에러를 내는 문제가 생깁니다(§3, §12 참고). 아두이노 보드를 교체하면 규칙 파일의 시리얼 번호도 다시 확인해서 갱신해야 합니다.

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

> **통합 launch(2026-08-11 추가)**: 아래 8-1~8-5를 8개 SSH 터미널로 나눠 실행하는 대신, `com_launch` 패키지의 `robot_all.launch.py` 하나로 한 번에 띄울 수 있습니다.
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/com_ws/install/setup.bash
> export TURTLEBOT3_MODEL=burger
> export LDS_MODEL=LDS-03
> ros2 launch com_launch robot_all.launch.py
> ```
> `hmi_serial_port`(기본 `/dev/com_tftboard`), `sensor_serial_port`(기본 `/dev/com_sensorboard`, 둘 다 udev 고정 심볼릭 링크) 등 인자로 포트를 오버라이드할 수 있습니다. 전체 인자 목록은 `ros2 launch com_launch robot_all.launch.py --show-args`. SSH 세션 종료 후에도 살려두려면 §11의 `setsid nohup ... &` 패턴을 그대로 쓰세요. 아래 8-1~8-5는 개별 노드를 따로 띄우거나 디버깅할 때 참고하세요.

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

**초기 위치 설정 (AMCL)** — 로봇이 지도 상 어디 있는지 알려줘야 합니다.

> ✅ **(2026-08-11) 권장 방법: `nav2_params.yaml`에 미리 박아두기.** `/initialpose` 토픽으로 런타임에 주는 방식은 AMCL이 메시지 stamp가 아니라 자기 `now()`로 tf를 조회해서 `/odom`이 조금만 뒤처져도(항상 그럼) `Failed to transform initial pose in time`으로 거부하는 고질적인 문제가 있습니다(§12 참고). 대신 `com_navigation/config/nav2_params.yaml`의 `amcl:` 섹션에 아래처럼 박아두면 Nav2 기동 시점에 바로 적용되어 이 문제를 피합니다:
> ```yaml
> amcl:
>   ros__parameters:
>     set_initial_pose: true
>     initial_pose:
>       x: <로봇의 실제 시작 x>
>       y: <로봇의 실제 시작 y>
>       z: 0.0
>       yaw: <로봇의 실제 시작 방향, 라디안>
> ```
> **로봇을 옮겨서 다시 기동할 때마다 이 좌표를 실제 위치로 갱신해야 합니다** — 안 그러면 AMCL이 엉뚱한 위치를 "확신"한 채로 시작합니다. 정상 운영 시 로봇은 항상 충전 스테이션 앞(`waypoints.yaml`의 `docking` 웨이포인트, 현재 x=1.079, y=0.901)에서 시작하는 게 기본 전제이므로, 특별한 사정이 없으면 이 좌표를 기본값으로 유지하세요.
>
> 파일을 고친 뒤에는 반드시 다시 빌드하고(`colcon build --symlink-install --packages-select com_navigation`) Nav2를 재기동해야 반영됩니다.

기존 토픽 방식(레이스 컨디션 있음, 잘 안 먹힐 수 있음)은 그래도 참고용으로 남겨둡니다:
```bash
ros2 topic pub --once --qos-reliability best_effort /initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: <로봇의 실제 x>, y: <실제 y>, z: 0.0}, \
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, \
  covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}"
```
> ⚠️ AMCL의 `/initialpose` 구독은 **BEST_EFFORT** QoS입니다. `--qos-reliability best_effort`를 빼먹으면 "publisher: beginning loop"까지는 뜨는데 AMCL이 영원히 못 받습니다(경고 로그만 반복). 한 번에 안 먹혀도(대부분 안 먹힙니다) 위 파라미터 방식을 쓰세요.

> ⚠️ **(2026-08-11) 초기위치가 잡혀도 안심 금물 — Nav2가 실제 이동 없이 "목표 도달"로 오판하는 별도 버그가 있습니다.** `/start_mission` 실행 전 반드시 아래 "AMCL 상태 확인" 절차로 `map→odom` tf가 실시간으로 갱신되고 있는지 먼저 확인하세요. §12 참고.

**AMCL 상태 확인 (주행 시도 전 필수, 2026-08-11 추가)**
```bash
# map->odom tf가 몇 초 이내로 계속 갱신되는지 확인 (오래 멈춰있으면 위험 신호)
ros2 run tf2_ros tf2_echo map odom
# amcl_pose가 계속 갱신되는지도 확인 (한 번 찍고 멈추면 안 됨)
ros2 topic hz /amcl_pose
```
둘 다 몇 초 이내 간격으로 계속 갱신되는 걸 확인한 뒤에만 `/start_mission`을 호출하세요. 정지해 있거나 몇십 초씩 안 갱신되면 실제로는 안 움직였는데 목표 도달로 잘못 처리될 수 있습니다(§12).

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
ros2 run com_sensors com_sensor_bridge_node    # 메인센서보드, 기본 /dev/com_sensorboard(udev 고정 링크)
ros2 run com_sensors hmi_bridge_node           # TFT보드, 기본 /dev/com_tftboard(udev 고정 링크)
```
`99-com-robot-serial.rules`가 설치돼 있으면(§3 참고) 포트 오버라이드 없이 그대로 실행하면 됩니다. 규칙이 없는 새 환경이라면 `--ros-args -p serial_port:=/dev/ttyACMx`로 실제 포트를 확인해서 오버라이드하세요.

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
| Topic | `/tts_request` | com_interfaces/TtsRequest | `service_manager`가 주요 상태 전이 시 발행 → `voice_node`가 구독해 재생 (2026-08-11 연결) |
| Service | `/start_mission` | com_interfaces/StartMission | destination_id: room1~room8 |
| Service | `/cancel_mission` | std_srvs/Trigger | READY로 직접 복귀 |
| Service | `/get_measurement_log` | com_interfaces/GetMeasurementLog | `/env_data` 발행자가 없어 **항상 빈 결과** |
| Action | `/navigate_to_pose` | nav2_msgs/action/NavigateToPose | service_manager → Nav2 |

### 발행자가 없어 사용 불가능한 것 (구독 코드는 있음)

| 이름 | 이유 |
|---|---|
| `/button_event` | 물리 긴급정지 버튼이 실제로 없음. GUI 버튼 도입 시 재사용 예정 |
| `/env_data` | 온습도 데이터를 waypoint_id와 묶어 재발행하는 다리 노드가 없음 |

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
- **(2026-08-12) `ssh host "command"`처럼 비대화형으로 원격 명령을 실행하면 `~/.bashrc`가 안 읽혀서 `ROS_DOMAIN_ID=30`이 적용 안 됩니다.** 이 상태에서 `ros2 topic pub`/`ros2 node list`를 실행하면 기본 도메인(0)으로 붙어서, 실제로는 정상 동작 중인 PM2 관리 노드들이 전혀 안 보이거나(`ros2 node list` 결과 텅 빔) `Waiting for at least 1 matching subscription(s)...`로 계속 멈춥니다 - 노드가 죽은 게 아니라 도메인이 안 맞아서 서로 안 보이는 것이니, 원격 명령엔 항상 `export ROS_DOMAIN_ID=30`을 명시적으로 넣으세요.
- **(2026-08-12) `robot_all.launch.py`를 PM2가 이미 전체 스택을 띄운 상태에서 추가로 실행하면 절대 안 됩니다.** 노드 이름이 겹쳐서 리소스(특히 USB 오디오 장치)를 두 프로세스가 동시에 붙잡아 재생이 반복 실패/크래시하고, CPU load average도 7대까지 치솟습니다. 뭔가 띄우기 전에 `pm2 list`로 이미 떠 있는지 먼저 확인하는 습관이 필요합니다.

---

## 12. 알려진 문제 / 미구현 항목

| 문제 | 영향 | 상태 |
|---|---|---|
| ~~`/initialpose`가 계속 거부됨~~ (해결) | ~~AMCL이 map→odom을 못 잡아 실제 주행 테스트가 막힘~~ | **2026-08-11 해결.** 원인: AMCL이 `/initialpose` 메시지 처리 시 메시지의 `header.stamp`가 아니라 자기 `now()` 기준으로 tf lookup을 해서, `/odom`이 조금이라도 뒤처져 있으면(항상 그럼) `Failed to transform initial pose in time`으로 거부 - 토픽 기반으로는 구조적으로 이 레이스를 못 피함. **해결책: `nav2_params.yaml`의 amcl 섹션에 `set_initial_pose: true` + `initial_pose.x/y/z/yaw`를 박아서 AMCL 시작 시점에 파라미터로 위치를 줌** - 토픽을 아예 안 타서 레이스 자체가 안 생김. 실측: 재시작 즉시 `Setting pose` 성공, `/amcl_pose` 정상 발행 확인. §8-2 참고. **주의: 현재 좌표는 8/11 테스트 시점 로봇의 실제 위치(1강의실 앞) 기준 임시값 - 운영 시 도킹 지점 좌표로 되돌리거나 매번 실제 시작 위치에 맞게 갱신 필요.** |
| **(신규, 미해결) AMCL이 `map→odom` tf를 지속 갱신 못 함 → Nav2가 실제 이동 없이 "목표 도달"로 오판** | `/start_mission`이 실제로는 로봇이 전혀 안 움직였는데 몇백ms 만에 GUIDING→ARRIVED로 넘어감(2026-08-11 조이스틱 실주행 테스트로 실측, 6m 목적지가 0.65초 만에 "도착" 처리됨) - **주행 로직 전체가 실제로는 검증 안 된 것과 같음** | 로그에 `[tf_help] Transform data too old when converting from map to odom`(수십 초 차이) 직후 `controller_server: Reached the goal!`가 뜸 - AMCL이 초기 위치는 잡지만 이후 `map→odom` tf를 실시간으로 못 갱신하고(스캔 처리 적체로 추정, `Message Filter dropping message... queue is full` 경고 반복 관찰) 오래된 tf 때문에 컨트롤러가 목표까지 거리 계산에 실패 → 거짓 성공으로 처리되는 것으로 보임. 사람감지 A/B 테스트로 CPU/사람감지와 무관함은 확정됐으나 근본 원인(AMCL 처리 적체 자체의 원인)은 미확정. **다음 세션 최우선 과제** - `amcl_pose`/`tf`가 스캔 주기(라이다 10Hz)만큼 계속 갱신되는지 실측 확인부터 시작할 것 |
| Nav2 + 비전(카메라+ArUco+사람감지)을 Pi4 한 대에서 동시 구동 시 CPU 부하(load average 5대, 4코어 기준)로 `/tf`·`/scan`이 간헐적으로 끊김 | 주행 중 정지·타임아웃 가능성 | `nav2_params.yaml` 튜닝 + 2026-08-11 사람감지 주기 분리·단일스레드화(`aruco_node` CPU 67.5%→52%)로 완화(완전 해결 아님). 단, 위 `/initialpose` 문제와는 무관함이 확정됨(A/B 테스트 참고) |
| USB 시리얼 장치(메인센서보드/TFT보드/OpenCR/오디오)가 간헐적으로 재연결됨 | 예전엔 이때 `hmi_bridge_node`/`com_sensor_bridge_node`가 예외처리 없이 그대로 죽었음(수동 재시작 전까지 HMI 화면·조이스틱·센서값 전부 먹통) | 2026-08-11: 두 노드 모두 시리얼 read/write를 `OSError` try/except로 감싸서 죽지 않고 재시도하도록 수정. **2026-08-12 추가 수정: 최초 연결 자체가 실패하면(USB 전원 불안정) 그대로 죽어서 PM2가 노드를 통째로 재시작해야 했던 문제 - 5회/2초 간격 재시도로 방어. 이 CDC-ACM 보드에서 `in_waiting`(ioctl) 체크가 간헐적으로 `Inappropriate ioctl for device`를 던지던 것도 발견, `in_waiting` 없이 `readline()` 자체 짧은 timeout에 맡기도록 변경 - 실측 결과 두 노드 모두 장시간(8분+) 에러 0건.** 포트 번호 자체가 바뀌는 문제는 udev 규칙으로 별도 해결(§3, §6-⑤) |
| USB 오디오 동글(Jieli UACDemoV1.0, 스피커) 재생 중 끊김/잡음 | `aplay: write error: No such device` 이후 ALSA가 카드를 아예 못 찾아(`Cannot get card index for 1`) 안내가 깨지거나 통째로 안 들림 | **2026-08-12: 볼륨 100%(유니티 게인)가 원인 중 하나로 확인 - 80%(-6.36dB)로 낮추니 반복 재생에도 끊김 없음(실측, `/tts_request` 실제 파이프라인으로 검증). `voice_node.py`의 `volume` 파라미터 기본값을 80으로 변경.** 단, USB 동글 자체가 여전히 불안정한 하드웨어라는 근본 문제는 남아있음(간헐적 재연결 등) - 완전 해결은 아니고 완화 |
| 물리 긴급정지 버튼 없음 | 긴급정지 기능이 현재 시스템에 아예 없음 | GUI 버튼으로 대체 예정. `com_gui` 문서상 미구현이었으나 2026-08-12에 `/com_gui_node`가 실제로 떠 있는 것을 발견 - 실제 긴급정지 버튼 구현 여부는 미확인, 다음에 확인 필요 |
| `/env_data` 발행자 없음 | 측정 로그 조회가 항상 빈 결과 | 미착수 (온습도+waypoint_id를 묶는 다리 노드 필요) |
| `WaypointSequencer`의 `trajectory_1`/`trajectory_2` 경유점 시퀀스 미정의 | 출발지→목적지, 목적지→도킹이 경유점 없이 직행(Nav2 자체 경로계획에 의존) | 설계상 문제는 아니나 문서(SR-006)와는 다름 |
| ArUco 도킹 정렬 신뢰성 | 다중 마커 동시 검출 시 잘못된 마커를 보고하던 버그는 수정(2026-08-06)했으나 실물 재검증 필요 | 다음 세션에서 재시도 필요 |
| ~~PM2 미도입~~ (해결) | ~~노드 비정상 종료 시 자동 재시작 없음~~ | **2026-08-11 도입, 2026-08-12 실측 확인.** `com_launch/ecosystem.config.js` + `pm2-com.service`로 전원 인가 시 8개 노드 자동 기동 + 노드별 최대 3회 자동 재시작(SR-026). 단 `pm2 save` 저장 상태가 "전부 stopped"면 재부팅해도 자동 기동 안 될 수 있으니 부팅 후 `pm2 list` 확인 습관 필요. `robot_all.launch.py`는 수동 디버깅용 대안으로 남아있으나 **PM2와 동시 실행 금지**(노드 중복으로 리소스·오디오 장치 경합, 2026-08-12 실측: load average 7대까지 치솟고 음성 노드 반복 크래시) |
| libcamera 0.1(vc4 파이프라인)에서 `orientation` 파라미터 미지원 | 카메라 프레임이 상하 반전되어 들어옴 | `aruco_node.py`에서 `cv2.rotate(frame, cv2.ROTATE_180)`로 코드에서 보정 처리 완료 |
| `coin_d4_driver` 전체 빌드 시 간헐적 Aborted | 통합 빌드 시간 지연 가능 | 미해결, 재확인 필요 |

---

## 13. 팀원 역할

| 김찬중 | 권오민 |
|---|---|
| 서류작성 (문서 7종 + 발표 PPT) | 인터페이스: HMI(TFT+조이스틱), 목적지 메뉴 UI |
| 환경 세팅, bringup + camera(OpenCV 캘리브레이션) | Nav2·SLAM 준비, 하드웨어 조립, 실공간 SLAM 매핑 |
| voice(음성 안내, `/tts_request` 배선 2026-08-11 완료) | aruco(마커 인식·정밀 도킹 오차 계산·사람 감지) |
| **service_manager**(상태머신, ArUco 정밀 도킹 로직) — 핵심 신규 기술 | sensor_bridge, status(배터리 %, 측정 로그, 상태 발행) |

**공동 작업**
- `com_interfaces`(RobotState/ArucoMarker/EnvData/StartMission 등 메시지 정의)
- 통합 테스트(8/3, 8/4, 8/5, 8/6 진행) 및 시험 결과 기록, 발표 리허설(8/13~8/14 예정)

> 상세 일일 진행 상황은 `docs/worklog/` 폴더의 작업일지를 참고하세요.
