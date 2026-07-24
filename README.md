# TurtleBot3 기반 방문자 안내 로봇 (COM팀)

> **최종 업데이트: 2026-07-24 (2일차)** — 이 README는 간트차트에 맞춰 매일 갱신됩니다.

TurtleBot3 Burger, ROS 2 Humble, Nav2, ArUco 마커를 이용한 실내 방문자 안내 로봇입니다. 로봇은 방문자를 지정 목적지(부스 A/B/C)까지 자율주행으로 안내하고, 이동 중 온도·습도·조도를 측정하며, 임무 완료 후 **ArUco 마커 기반 정밀 도킹**으로 충전 스테이션에 자동 복귀합니다.

- 저장소: `unitydt-ros2-class2-2026/com_main_project_01`
- 팀: COM (김찬중, 권오민)
- 프로젝트 기간: 4주 (7/20 ~ 8/14)
- 핵심 신규 기술: **ArUco 기반 정밀 도킹 상태머신**

---

## 목차

1. [프로젝트 소개](#1-프로젝트-소개)
2. [서비스 시나리오](#2-서비스-시나리오)
3. [하드웨어 구성](#3-하드웨어-구성)
4. [소프트웨어 구성](#4-소프트웨어-구성)
5. [패키지 구조](#5-패키지-구조)
6. [설치 방법](#6-설치-방법)
7. [빌드 방법](#7-빌드-방법)
8. [실행 방법](#8-실행-방법)
9. [자동 실행 방법](#9-자동-실행-방법)
10. [토픽 · 서비스 · 액션 목록](#10-토픽--서비스--액션-목록)
11. [지도와 경유점 설명](#11-지도와-경유점-설명)
12. [시험 결과](#12-시험-결과)
13. [알려진 문제](#13-알려진-문제)
14. [팀원 역할](#14-팀원-역할)

---

## 1. 프로젝트 소개

처음 방문한 실내 공간(강의실·복도)에서 방문자가 목적지를 찾지 못해 발생하는 문제를 해결하는 로봇입니다. 목적지까지 직접 안내하면서 이동 경로상의 환경 상태(온도·습도·조도)를 함께 점검·기록합니다.

**주요 기능**
- Nav2 기반 자율주행으로 목적지(부스 A/B/C) 안내 — *구현 예정 (7/28~)*
- 경유점마다 온습도·조도 측정 및 이상 여부 기록 — *구현 예정 (7/30~7/31)*
- ArUco 마커로 목적지 검증 + 목적지 앞 사람 감지 시 대기 — *구현 예정 (7/29)*
- **ArUco 마커 기반 정밀 도킹**으로 충전 스테이션 자동 복귀 — *구현 예정 (7/30~7/31)*
- 상태 전환마다 음성 안내 + 로봇 눈(LCD) 표정 + 웰컴 라이트 LED 출력 — *구현 예정 (8/4~8/7)*
- PyQt 관제 GUI — *구현 예정 (8/6~8/7)*

**현재까지 구현 완료된 부분**
- 카메라 드라이버 연동 및 캘리브레이션 (§8 참고)
- `com_service` 패키지 상태머신 골격 및 `/cancel_mission` 서비스

---

## 2. 서비스 시나리오

1. 전원 인가 → PM2가 노드 자동 실행(30초 이내 active, *PM2 구성은 8/7 예정*) → 초기 위치(충전 스테이션 앞 정렬점) 설정
2. 사용자가 GUI 또는 로봇 물리 버튼에서 목적지(A/B/C) 선택 → 임무 시작
3. Trajectory 1(출발점→WP1~WP4→목적지)을 따라 이동하며 경유점마다 온습도·조도 측정
4. 목적지 도착 시 ArUco 마커로 목적지 검증, 3cm 이내 정지, 음성·표정으로 도착 안내
5. 5초 대기 후 Trajectory 2(목적지→WP5~WP6→충전대기점→정렬점→도킹점)로 복귀
6. 충전 스테이션 ArUco 마커로 좌우·거리 오차를 계산하며 저속 정밀 도킹
7. 도킹 완료 시 음성 안내 + 웃는 눈 표정으로 임무 완료

> 정상 동작 12단계, 예외 상황 12종 전체 목록은 `docs/URS-001` 3장을 참고하세요. 위 시나리오는 **7/28~8/7 사이 순차 구현 예정**이며 현재는 각 노드 골격/카메라 파이프라인만 준비된 상태입니다.

---

## 3. 하드웨어 구성

| 구성 요소 | 역할 | 상태 |
|---|---|---|
| TurtleBot3 Burger | 2륜 차동구동 로봇 베이스 (최대 0.22 m/s) | 연동 완료 |
| OpenCR 보드 | 모터 제어 + IMU, Raspberry Pi와 USB Serial 연결 | 연동 완료 |
| 2D LiDAR (COIN D4) | 전방 장애물 감지 및 위치추정 | 드라이버 설치·udev 링크 완료 |
| Raspberry Pi Camera (imx219) | ArUco 마커 인식·사람 감지 | **연동 및 캘리브레이션 완료** |
| 센서 브릿지 보드 (Arduino/GPIO) | IR(후방)·버튼·DHT11·조도 | 구현 예정 (7/30~7/31) |
| 보조 아두이노 | LCD 로봇 눈 + 웰컴 라이트 LED | 구현 예정 (8/3~8/4) |
| Raspberry Pi 4 | 온보드 컴퓨터, PM2로 노드 관리 | ROS 2 설치 완료, PM2 구성 예정(8/7) |
| 충전 스테이션 | ArUco 마커 부착, 정밀 도킹 대상 | 도킹 로직 구현 예정 (7/30~7/31) |
| 관제 PC/노트북 | PyQt GUI 실행 (선택적 연결) | 구현 예정 (8/6~8/7) |

---

## 4. 소프트웨어 구성

| 구분 | 내용 | 상태 |
|---|---|---|
| OS | Ubuntu 22.04 LTS | 완료 |
| 미들웨어 | ROS 2 Humble Hawksbill (DDS: Fast-DDS) | 완료 |
| 내비게이션 | Nav2 (bt_navigator, planner_server, controller_server, amcl, map_server) | 패키지 설치 완료, 연동 예정 (7/28) |
| SLAM | slam_toolbox | 지도 작성 예정 |
| 비전 | OpenCV + ArUco 모듈 | **카메라 파이프라인·캘리브레이션 완료**, 마커 인식 로직은 예정 (7/28~7/29) |
| GUI | PyQt5 | 구현 예정 (8/6~8/7) |
| 프로세스 관리 | PM2 | 구성 예정 (8/7) |
| 음성 | 로컬 음성 재생(사전 녹음 wav) + 로컬 TTS 보완 | 구현 예정 (8/4) |
| 하드웨어 브릿지 | Serial/UART | 구현 예정 (7/30~7/31) |

---

## 5. 패키지 구조

TurtleBot3 원본 패키지를 그대로 제출하지 않고, 팀 이름을 포함한 패키지로 재구성했습니다.

```
com_ws/
└── src/
    ├── com_bringup/      # OpenCR·LiDAR 드라이버 launch (구현 예정)
    ├── com_navigation/   # Nav2 스택, nav2_params.yaml, waypoints.yaml (구현 예정, 7/28~)
    ├── com_description/  # 로봇 URDF/mesh (구현 예정)
    ├── com_interfaces/   # 커스텀 msg/srv - StartMission 등 (구현 예정, 7/27)
    ├── com_sensors/      # IR·버튼·DHT11·조도 (구현 예정, 7/30~7/31)
    ├── com_vision/       # camera + aruco 인식·정밀도킹 오차계산
    │                     #   ├─ config/ : 카메라 캘리브레이션 yaml (완료)
    │                     #   └─ ArUco 인식 로직 (구현 예정, 7/28~7/29)
    ├── com_gui/          # PyQt 관제 GUI (구현 예정, 8/6~8/7)
    ├── com_service/      # 임무 상태머신(service_manager) + status + voice
    │                     #   ├─ mission_state.py : 상태 14종 정의 (완료)
    │                     #   ├─ service_manager_node.py : 골격 + /cancel_mission (완료)
    │                     #   └─ /start_mission, 도킹 제어 로직 (구현 예정, 7/27~7/31)
    └── com_launch/       # PM2 ecosystem, 통합 launch (구현 예정, 8/7)
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
source ~/.bashrc
```

**② TurtleBot3 · LiDAR(COIN D4) 패키지 설치**
```bash
sudo apt install -y ros-humble-turtlebot3 ros-humble-turtlebot3-msgs \
  ros-humble-dynamixel-sdk ros-humble-hls-lfcd-lds-driver \
  ros-humble-turtlebot3-teleop
sudo usermod -aG dialout $USER   # OpenCR USB 권한 (재접속 필요)
```

**③ 카메라 (완료 사용 중)**
```bash
sudo apt install -y ros-humble-camera-ros ros-humble-camera-calibration
```

**④ Nav2 · SLAM · OpenCV (설치 완료, 연동은 예정)**
```bash
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-cv-bridge
pip3 install opencv-contrib-python --break-system-packages
```

**⑤ GUI · PM2 (구현 예정 시점에 설치)**
```bash
sudo apt install -y python3-pyqt5 nodejs npm
sudo npm install -g pm2
```

**⑥ 워크스페이스 클론**
```bash
mkdir -p ~/com_ws/src
cd ~/com_ws
git clone git@github.com:unitydt-ros2-class2-2026/com_main_project_01.git .
```

---

## 7. 빌드 방법

```bash
cd ~/com_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

패키지 개별 빌드가 필요할 때:
```bash
colcon build --symlink-install --packages-select com_service
```

---

## 8. 실행 방법

### 현재 실행 가능한 부분

| 터미널 | 명령 | 설명 |
|---|---|---|
| 1 | `ros2 run camera_ros camera_node --ros-args -p camera:=0 -p format:=RGB888 -p width:=640 -p height:=480` | 카메라 구동 (imx219, 640x480, 캘리브레이션 적용됨) |
| 2 | `ros2 run com_service service_manager_node` | 상태머신 골격 노드 실행 |

검증:
```bash
ros2 topic hz /camera/image_raw          # 30Hz 확인
ros2 node list                           # /camera, /service_manager 확인
ros2 service call /cancel_mission std_srvs/srv/Trigger
```

### 구현 예정 부분

| 노드/기능 | 실행 명령(예정) | 구현 예정일 |
|---|---|---|
| bringup (LiDAR/OpenCR) | `ros2 launch com_bringup bringup.launch.py` | 7/23 완료분 재정리 → com_bringup 패키지화 예정 |
| navigation (Nav2) | `ros2 launch com_navigation navigation.launch.py map:=maps/hallway_map.yaml` | 7/28 |
| aruco 인식 | `ros2 run com_vision aruco_node` | 7/28~7/29 |
| sensor_bridge | `ros2 run com_sensors sensor_bridge_node` | 7/30~7/31 |
| voice | `ros2 run com_service voice_node` | 8/4 |
| gui | `ros2 run com_gui gui_main` | 8/6~8/7 |

---

## 9. 자동 실행 방법

**현재**: PM2 미구성 — 각 노드를 §8 방식으로 개별 실행 중입니다.

**구현 예정 (8/7)**: `com_launch` 패키지에 PM2 ecosystem 설정 후 아래 방식으로 통합 기동 예정
```bash
pm2 start com_launch/ecosystem.config.js
pm2 status
pm2 logs com_service
pm2 stop all
```
GUI(`com_gui`)는 PM2 관리 대상이 아니며 관제 PC에서 수동 실행 예정입니다.

---

## 10. 토픽 · 서비스 · 액션 목록

### 현재 구현된 것
| 구분 | 이름 | 설명 |
|---|---|---|
| Topic | `/camera/image_raw` | 카메라 원본 영상 (30Hz) |
| Topic | `/camera/camera_info` | 캘리브레이션 결과 반영된 카메라 정보 |
| Topic | `/state_code` | 상태머신 코드 발행 (수신측 status 노드는 미구현) |
| Service | `/cancel_mission` | std_srvs/Trigger, 골격만 구현(항상 success=True 응답) |

### 구현 예정 (전체 목록은 `docs/ICD-001` 참고)
| 구분 | 이름 | 구현 예정일 |
|---|---|---|
| Service | `/start_mission` | 7/27 (com_interfaces 확정 후) |
| Topic | `/aruco_marker`, `/person_detected` | 7/28~7/29 |
| Topic | `/rear_ir`, `/button_event`, `/env_data` | 7/30~7/31 |
| Topic | `/robot_state` | 8/4~8/5 |
| Topic | `/tts_request` | 8/4 |
| Action | `/navigate_to_pose` | 7/28 |
| Service | `/get_measurement_log` | 8/5 |

---

## 11. 지도와 경유점 설명

- SLAM 지도 작성 및 `waypoints.yaml`, `aruco_markers.yaml` 작성은 **구현 예정 (7/24 SLAM 매핑 착수 → 7/27 waypoints/markers 확정)** 입니다.
- 경유점 구성 계획(URS-001 기준): 경유점 10개 이상 — 출발점(충전 스테이션), WP1~WP6, 목적지 A/B/C, 충전 대기점, 정렬점, 도킹점
- Trajectory 1(안내): 출발점 → WP1 → WP2 → WP3 → WP4 → 목적지
- Trajectory 2(복귀·충전): 목적지 → WP5 → WP6 → 충전 대기점 → 정렬점 → 도킹점

확정되는 대로 `com_navigation/config/waypoints.yaml`, `com_vision/config/aruco_markers.yaml`로 등록 예정입니다.

---

## 12. 시험 결과

> 실제 측정은 4주차(8/10~8/14) 테스트 기간에 진행 예정이며, 완료 후 아래 표에 결과를 기록합니다. 현재는 카메라 캘리브레이션 결과만 확정되어 있습니다.

| 항목 | 관련 요구사항 | 목표값 | 측정값 | 결과 |
|---|---|---|---|---|
| 카메라 캘리브레이션 (imx219, 640x480) | - | reprojection error 최소화 | fx=1334.97, fy=1337.16, D=[0.139,0.951,0.016,0.016,0.0] | 완료 |
| 목적지 도착 위치 오차 | SR-002 | 3cm 이내 | - | 예정 (8/11) |
| 전방 장애물 감속/정지 거리 | SR-003 | 30cm 감속 / 20cm 정지 | - | 예정 (8/11) |
| 후방 장애물 정지 거리 | SR-004 | 20cm | - | 예정 (8/11) |
| 긴급정지 반응 시간 | SR-021 | 500ms 이내 | - | 예정 (8/11) |
| 노드 기동 시간 | SR-031 | 30초 이내 | - | 예정 (8/10~8/11) |
| ArUco 정밀 도킹 성공률 | SR-008~011 | 3회 이내 성공 | - | 예정 (8/11) |
| 목표 이동 실패 타임아웃 | SR-019 | 60초 | - | 예정 (8/11) |

---

## 13. 알려진 문제

| 문제 | 영향 | 상태 |
|---|---|---|
| libcamera 0.1(vc4 파이프라인)에서 `orientation` 파라미터 미지원 — 카메라 화면 상하 반전 | 육안 확인 시 불편, 인식 정확도엔 무관 | com_vision aruco 노드 구현 시 `cv2.rotate(frame, cv2.ROTATE_180)`로 코드 처리 예정 (7/28~) |
| Wi-Fi 대역폭 대비 카메라 원본 스트리밍(640x480 RGB888 @30Hz) 시 경미한 버벅임 | 원격 모니터링 시 다소 지연 | 실사용 지장 없는 수준으로 판단, 보류 |
| `coin_d4_driver` 전체 빌드 시 간헐적 Aborted | 통합 빌드 시간 지연 가능 | 미해결, 재확인 필요 |
| 저조도/역광 환경에서 ArUco 마커 인식률 저하 가능성 | 목적지 검증·도킹 실패율 증가 가능 | 구현 후 확인 필요 (7/28~) |
| DHT11 센서 응답 지연(최대 2초) | 경유점 통과가 빠르면 측정 누락 가능 | 구현 후 확인 필요 (7/30~7/31) |

---

## 14. 팀원 역할

| 김찬중 | 권오민 |
|---|---|
| 서류작성 (문서 7종 + 발표 PPT) | 인터페이스: GUI(PyQt), PM2 |
| 환경 세팅 (완료) | Nav2·SLAM 준비, 하드웨어 조립 |
| bringup + camera(OpenCV 캘리브레이션) — **완료** | 실공간 SLAM 매핑, 지도 저장 |
| voice(음성 안내) — 예정 8/4 | aruco(마커 인식·정밀 도킹 오차 계산) — 예정 7/28~7/29 |
| **service_manager** (상태머신 골격 완료, ArUco 정밀 도킹 로직 예정 7/30~7/31) — 핵심 신규 기술 | sensor_bridge, status(배터리 %, 측정 로그, GUI 상태 발행) — 예정 7/30~8/5 |

**공동 작업**
- `com_interfaces`(RobotState/ArucoMarker/EnvData/StartMission 등 메시지 정의) — **7/27 오전 함께 확정 예정**
- 통합 테스트(8/3, 8/10~8/11) 및 시험 결과 기록, 발표 리허설(8/13~8/14)

> 상세 일일 진행 상황은 `docs/worklog/` 폴더의 작업일지를 참고하세요. 이 README는 매일 간트차트 진행에 맞춰 갱신됩니다.
