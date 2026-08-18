# TurtleBot3 기반 방문자 안내 로봇 (COM팀)

> **최종 업데이트: 2026-08-18.** 이 문서는 "구현 예정"이 아니라 **실제 동작 확인된 현재 상태**를 기준으로 씁니다.
>
> ### 🎉 8/18 저녁 실주행 테스트: 자율주행 전 구간 성공
>
> | 항목 | 결과 |
> |---|---|
> | room1까지 실주행 (5.66m) | **77초** ✅ |
> | ARRIVED 대기 | 5.0초 (파라미터대로) ✅ |
> | 도킹 지점 복귀 | **43초** ✅ |
> | 최종 위치 정확도 | 목표 대비 **3mm** ✅ |
> | ~~"거짓 도착" 버그~~ | **재현 안 됨 — 원인 규명·해결** ✅ |
>
> **원인은 AMCL tf 문제가 아니라 `set_initial_pose` 좌표가 벽에 파묻혀 있던 것이었습니다.** 기존 값 (1.079, 0.901)은 지도상 **벽에서 0.101m** 지점인데 `robot_radius`가 0.1m라, AMCL이 믿는 출발점이 그대로 lethal 셀이 되어 **Nav2 planner가 목적지와 무관하게 경로 계산 자체를 거부**하고 있었습니다(`GridBased: failed to create plan` → 전부 `status=6 ABORTED`). `waypoints.yaml`의 docking 좌표(벽에서 1.007m)로 맞추자 즉시 정상화됐습니다. 자세한 내용은 §12 참고.
>
> - **같은 날 함께 해결**: 카메라가 1분 후 조용히 멈추던 문제(→ `FrameDurationLimits` 10fps 제한), PM2가 정상 종료를 크래시로 오집계해 스택 전체를 내려버리던 문제(→ 8개 노드에 `rclpy.shutdown()` 가드).
> - **⏳ 유일하게 남은 미검증 항목**: **ArUco 마커(ID 29) 실검출**. 카메라 안정화 후 360° 스캔했으나 미검출 상태에서 배터리가 소진돼 중단. 충전 후 로봇을 마커 정면에 두고 `/aruco_marker`부터 확인할 것(§12).
>
> ---
>
> - **8/18(후반부): 도킹 방식 재설계 + `auto_dock_node` 신규 추가.** 기존 "15도씩 회전 재탐색" 방식이 카메라 파이프라인 지연(수백 ms) 때문에 회전 중 마커를 지나치는 원인으로 밝혀져, `service_manager`의 `DockingController`를 "좌 95도 고정 회전 → 완전 정지 → 그때 판정(stop-and-look)" 방식으로 전면 재설계(오도메트리 기반 정밀 회전 + 낡은 카메라 프레임 거부 로직 포함). 실주행에서 회전 정확도 확인됨(`목표 95.0도 / 실제 93.2도`). 별도로 더 정교한 7단계 상태머신(`NAV_TO_DOCK→TURN_LEFT_90→FIND_MARKER→APPROACH→TURN_180→REVERSE→ANNOUNCE`)을 가진 `auto_dock_node`도 신규 작성됨 — **두 도킹 로직은 상호 배타적으로 하나만 사용해야 함**(§12 "`/cmd_vel` 발행 주체 충돌" 참고).
> - **8/18**: `com_gui`(PyQt5 관제 대시보드, **관제 PC 전용** — 파이엔 설치 안 함) 구현 완료. 긴급정지는 GUI에도 두지 않기로 확정(HMI-001 6장) — 프로젝트 전체에서 최종적으로 미구현. TFT 부팅 웰컴 화면 추가. `DockingController`가 존재하지 않는 필드(`marker.yaw_error`)를 읽어 도킹 중 크래시하는 버그 발견·수정. 정체불명 PM2 프로세스(`com_robot`) 제거, 워크스페이스 클린업.
> - **8/11~8/12**: 음성 안내(`/tts_request`) 연결, **PM2 자동기동/자동재시작** 도입 및 실측 확인, USB 시리얼 포트 udev 고정, `/initialpose` 레이스 문제 해결, USB 오디오 볼륨 조정으로 재생 끊김 완화.
> - **주의**: 위 수정 전부 **git 미커밋 상태**입니다(파이에만 존재). 실주행 완주 확인 후 일괄 커밋 예정.

## 🔋 다음 세션 재개 절차

1. **PM2가 정상 기동 중인지부터 확인** (평소엔 전원 인가 시 자동으로 떠 있어야 정상):
   ```bash
   ssh com@192.168.210.15
   pm2 list   # 8개(bringup/navigation/vision/sensor_bridge/hmi_bridge/service_manager/status/voice) 전부 online인지 확인
   ```
   `ecosystem.config.js`에 없는 이름(예: 과거의 `com_robot`처럼)이 섞여 있으면 `pm2 delete <이름>`으로 정리하세요. 8개 중 일부가 안 떠 있으면 `pm2 start ~/com_ws/src/com_launch/ecosystem.config.js && pm2 save`로 복구.
2. **배터리 충전 확인** (12V 근처 권장, §8-1 참고):
   ```bash
   source /opt/ros/humble/setup.bash && source ~/com_ws/install/setup.bash
   export ROS_DOMAIN_ID=30
   ros2 topic echo /battery_state --once | grep -E "voltage|percentage"
   ```
3. **로봇을 충전 스테이션 앞 정렬점에 둔 상태인지 확인** — `nav2_params.yaml`의 `set_initial_pose` 좌표(도킹 지점, 현재 x=0.735, y=-0.007 — **2026-08-18 실주행으로 검증 완료**)가 이 위치를 전제로 합니다. **로봇을 다른 곳에 두고 시작하면 이 좌표를 반드시 실제 위치로 갱신하세요** — 8/18에 이 값이 어긋나 있어서(벽에서 10cm 지점) Nav2가 경로 계산 자체를 거부했던 전례가 있습니다(§12).
4. **PM2로 안 뜨는 경우에만 수동 기동** (평소엔 1번에서 이미 떠 있어야 하므로 이 단계는 예외 상황용):
   ```bash
   ps aux | grep -E "com_ws/install|turtlebot3_ros|component_container|camera_ros" | grep -v grep
   # 나온 PID들을 개별적으로 kill (pkill -f 절대 쓰지 말 것! §11 참고)
   ```
   확실하지 않으면 그냥 `sudo reboot`로 재부팅하는 게 제일 깔끔합니다(PM2가 자동으로 다시 뜸, §1 "PM2 프로세스 관리" 참고).
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/com_ws/install/setup.bash
   export TURTLEBOT3_MODEL=burger
   export LDS_MODEL=LDS-03
   setsid nohup ros2 launch com_launch robot_all.launch.py > ~/com_launch.log 2>&1 < /dev/null &
   disown
   ```
   **PM2가 이미 떠 있는 상태에서 이 명령을 추가로 실행하지 마세요** — 노드 중복으로 리소스/오디오 장치 경합이 실측됐습니다(§11). 20~30초 기다린 뒤 확인:
   ```bash
   grep -iE "died|Traceback" ~/com_launch.log | grep -v "No such device\|audio open error"
   ```
   아무것도 안 뜨면 정상. 뭔가 죽었으면(특히 `camera_node`가 "Device or resource busy"로 죽는 경우가 있었음 - 이전 세션에서 정리 안 된 프로세스가 카메라를 붙잡고 있을 때) 해당 노드만 개별적으로 재시작(§8-3~8-5 패턴 참고).
5. **AMCL이 실제로 잘 도는지 확인 (§8-2 "AMCL 상태 확인" 필수 — 이거 안 하면 "거짓 도착" 버그를 또 만날 수 있음)**:
   ```bash
   ros2 run tf2_ros tf2_echo map odom     # 계속 갱신되는지
   ros2 topic hz /amcl_pose                # 계속 갱신되는지 (Ctrl+C로 종료)
   ```
   둘 다 멈추지 않고 계속 갱신되면 안전하게 `/start_mission` 테스트 진행. (참고: 예전 "거짓 도착" 버그는 2026-08-18에 `set_initial_pose` 좌표 문제로 원인이 밝혀져 해결됐습니다 — §12. 그래도 주행 전 이 확인은 습관으로 유지하는 게 좋습니다.)
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
- Nav2 기반 자율주행으로 목적지(room1~room8) 안내 — **2026-08-18 전 구간 실주행 성공** (room1까지 77초·5.66m 실주행 → ARRIVED 5초 대기 → 도킹 지점 복귀 43초, 최종 위치 오차 **3mm**). 2026-08-06에도 실주행 성공한 바 있음(좌표 오차 3.6cm)
- ArUco 마커 인식 + 사람 감지(HOG) — 구현 완료. 2026-08-11에 사람감지 주기를 마커 검출(5Hz)과 분리(기본 1Hz)해 CPU 사용량 약 23% 절감(67.5%→52%, `aruco_node` 단독 기준). 다만 Pi4에서 Nav2와 동시 구동 시 여전히 여유가 빠듯함(§12 참고)
- **ARRIVED → RETURNING → DOCKING → DOCKED → READY 자동 사이클** — 2026-08-18 실주행에서 `ARRIVED`(5초 대기) → `RETURNING` → 도킹 지점 도착 → `DOCKING` 진입까지 전 구간 확인. **마지막 ArUco 정렬 단계만 미검증**(마커 실검출 확인 전, §12)
- **PERSON_WAIT**(사람 감지 시 정지·대기) — 2026-08-06 구현(OBSTACLE_STOP과 동일 패턴), 실물 검증 예정
- 후방 초음파 장애물 감지(OBSTACLE_STOP) — 2026-08-06 배선 오류 발견·수정(그 전까지는 실제로 동작한 적이 없었음), 실물 검증 예정
- HMI: TFT 화면 보드 + 조이스틱 메인센서보드 2보드 구성, 한글 UI — 실물 검증 완료
- 음성 안내(`voice_node`) — 2026-08-11에 `service_manager_node`가 주요 상태 전이(안내 시작/도착/복귀/취소/장애물/사람대기 등) 시점에 `/tts_request`를 실제로 발행하도록 연결 완료, 더 이상 무음 아님. 재생 문구는 gTTS로 생성(`com_service/scripts/generate_audio.py`, 재실행하면 재생성 가능). 연달아 겹쳐 재생 요청되면 이전 재생이 끝날 때까지 큐잉 후 순차 재생(이전엔 ALSA 충돌로 뒤 안내가 씹히는 버그가 있었음, 수정 완료)
- PyQt5 관제 GUI(`com_gui`) — **2026-08-18 실제 구현 완료.** 맵뷰(`/map`), 카메라 실시간 영상, 임무 제어(`/start_mission`·`/cancel_mission`), 로봇 상태(`/robot_state`), 센서 모니터, 배터리 추이, 알람·이벤트 타임라인, 측정 로그 조회, 디버그용 수동 주행(`/cmd_vel`, READY 상태+체크박스 이중 잠금), 파이 PM2 프로세스 원격 제어(SSH) 패널 제공. **로봇(파이)이 아니라 관제용 메인 PC에서 실행하는 용도**이고 소스만 워크스페이스 미러링 관례상 파이에도 같이 있음 — 파이엔 `PyQt5` 자체가 설치돼 있지 않으며 이건 정상입니다. 목적지 선택은 여전히 로봇 탑재 TFT+조이스틱으로 함(GUI는 관제/모니터링용, 목적지 선택 대체 아님)

**미구현**
- 물리 긴급정지 버튼 — 없음. **GUI에도 두지 않기로 확정**(`com_gui` 코드 주석 근거: HMI-001 6장에 따라 안전기능은 GUI 밖에서 처리, 수동주행은 READY 상태+체크박스 이중 잠금으로만 허용) — 긴급정지는 이 프로젝트에서 최종적으로 미구현 상태로 남음
- `/env_data` 발행자 없음 — 측정 로그 조회(`/get_measurement_log`)는 항상 빈 결과 반환

**PM2 프로세스 관리 (2026-08-11 도입, 2026-08-12 실측 확인)**
- `com_launch/ecosystem.config.js` + `pm2-com.service`(systemd)로 전원 인가 시 8개 노드(bringup/navigation/vision/sensor_bridge/hmi_bridge/service_manager/status/voice) 자동 기동, 노드별 최대 3회 자동 재시작(SR-026).
- 2026-08-12 실측: 8개 전부 `online` 상태로 장시간 안정 가동 확인(`pm2 list`). **주의**: `pm2 save`로 저장된 마지막 상태가 "전부 stopped"였던 적이 있어, 그 상태로 저장된 채 재부팅하면 자동 기동이 아예 안 될 수 있음 - 부팅 후 `pm2 list`로 전부 `online`인지 항상 확인할 것. 문제 있으면 `pm2 start ~/com_ws/src/com_launch/ecosystem.config.js && pm2 save`로 복구.
- `com_launch/robot_all.launch.py`(단일 `ros2 launch` 프로세스로 통합)는 수동 디버깅용 대안으로 계속 존재 - **PM2와 동시에 띄우지 말 것** (노드 중복 실행으로 리소스·오디오 장치 경합, §11 참고).

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
| Raspberry Pi 4 | 온보드 컴퓨터 | 전원 인가 시 PM2가 8개 노드 자동 기동(§1 "PM2 프로세스 관리" 참고) |
| 충전 스테이션 | ArUco 마커(ID 29) 부착, 정밀 도킹 대상 | `waypoints.yaml`의 `docking` 웨이포인트 |
| 관제 PC/노트북 | 개발/미러용 워크스페이스, `docs/`, docx 열람, **`com_gui` PyQt5 관제 대시보드 실행** | `com_gui`(§5) 참고 |

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
| GUI | PyQt5 — **구현 완료(2026-08-18), 관제 PC 전용**(§5 `com_gui` 참고) |
| 프로세스 관리 | PM2(`ecosystem.config.js`) — 전원 인가 시 8개 노드 자동 기동 + 노드별 최대 3회 자동 재시작. `robot_all.launch.py`(수동 `ros2 launch` 통합)는 디버깅용 대안으로 남아있으나 PM2와 동시 실행 금지(§11) |
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
    ├── com_gui/          # PyQt5 관제 대시보드(2026-08-18 구현 완료), 관제 PC 전용 — 파이엔 PyQt5 없음(정상)
    │                     #   ├─ gui_node.py   : 실행 진입점 (ros2 run com_gui gui_node), 노드명 com_gui_node
    │                     #   ├─ ros_worker.py : ROS2 통신 전담 QThread(구독/발행/서비스호출)
    │                     #   ├─ main_window.py, widgets.py, theme.py : 레이아웃/커스텀 위젯/다크테마 QSS
    │                     #   └─ 목적지 선택은 여전히 로봇 탑재 TFT+조이스틱으로 함(GUI는 관제/모니터링용)
    ├── com_launch/       # launch/robot_all.launch.py : §8의 8단계 + auto_dock_node(2026-08-18 추가)를 하나의 ros2 launch로 통합
    ├── com_service/      # 임무 상태머신 + 음성 + 자동 도킹
    │                     #   ├─ mission_state.py       : 상태 14종 정의
    │                     #   ├─ service_manager_node.py: 상태머신, WaypointSequencer, DockingController(2026-08-18 좌90도
    │                     #   │                            stop-and-look 방식으로 재설계), SafetyMonitor, /tts_request 발행
    │                     #   ├─ auto_dock_node.py      : (2026-08-18 신규) 7단계 자동 도킹 상태머신(NAV_TO_DOCK→
    │                     #   │                            TURN_LEFT_90→FIND_MARKER→APPROACH→TURN_180→REVERSE→ANNOUNCE),
    │                     #   │                            /start_docking·/cancel_docking 서비스. DockingController와
    │                     #   │                            상호 배타적으로 하나만 사용할 것(§12 참고)
    │                     #   ├─ config/auto_dock_params.yaml : auto_dock_node 파라미터(마커ID/제어게인/후진오프셋 등)
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

**⑥ PM2 설치 (파이, 자동기동/자동재시작용)**
```bash
sudo apt install -y npm && sudo npm install -g pm2
cd ~/com_ws/src/com_launch
pm2 start ecosystem.config.js
pm2 save
pm2 startup   # 안내되는 sudo 명령을 그대로 복사해서 실행하면 재부팅 시 자동 기동
```

**⑦ `com_gui` 설치 (관제 PC, 파이 아님)**
```bash
sudo apt install -y python3-pyqt5 python3-numpy
# com_ws가 관제 PC에도 미러링돼 있다는 전제 하에:
cd ~/com_ws
colcon build --symlink-install --packages-select com_gui com_interfaces
source install/setup.bash
export ROS_DOMAIN_ID=30
ros2 run com_gui gui_node
```
파이와 같은 `ROS_DOMAIN_ID=30` 네트워크에 있으면 SSH 없이도 DDS로 로봇의 모든 토픽/서비스를 그대로 볼 수 있습니다.

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
> **로봇을 옮겨서 다시 기동할 때마다 이 좌표를 실제 위치로 갱신해야 합니다** — 안 그러면 AMCL이 엉뚱한 위치를 "확신"한 채로 시작합니다. 정상 운영 시 로봇은 항상 충전 스테이션 앞(`waypoints.yaml`의 `docking` 웨이포인트, 현재 x=0.735, y=-0.007 — 2026-08-14 RViz Publish Point로 재측정, **아직 실물 재검증 전**)에서 시작하는 게 기본 전제이므로, 특별한 사정이 없으면 이 좌표를 기본값으로 유지하세요.
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
- **(2026-08-18) `pm2 list`에 `ecosystem.config.js`에 정의되지 않은 이름이 섞여 있을 수 있습니다** — 디버깅 중 `pm2 start <스크립트> --name <임의이름>`처럼 임시로 띄우고 안 지운 프로세스가 남아있던 사례가 실제로 있었음(`com_robot`, `errored` 상태로 방치). `pm2 list` 이름을 정식 8개(bringup/navigation/vision/sensor_bridge/hmi_bridge/service_manager/status/voice)와 대조해서 낯선 게 있으면 `pm2 delete <이름> && pm2 save`로 정리하세요.

---

## 12. 알려진 문제 / 미구현 항목

| 문제 | 영향 | 상태 |
|---|---|---|
| ~~`/initialpose`가 계속 거부됨~~ (해결) | ~~AMCL이 map→odom을 못 잡아 실제 주행 테스트가 막힘~~ | **2026-08-11 해결.** 원인: AMCL이 `/initialpose` 메시지 처리 시 메시지의 `header.stamp`가 아니라 자기 `now()` 기준으로 tf lookup을 해서, `/odom`이 조금이라도 뒤처져 있으면(항상 그럼) `Failed to transform initial pose in time`으로 거부 - 토픽 기반으로는 구조적으로 이 레이스를 못 피함. **해결책: `nav2_params.yaml`의 amcl 섹션에 `set_initial_pose: true` + `initial_pose.x/y/z/yaw`를 박아서 AMCL 시작 시점에 파라미터로 위치를 줌** - 토픽을 아예 안 타서 레이스 자체가 안 생김. 실측: 재시작 즉시 `Setting pose` 성공, `/amcl_pose` 정상 발행 확인. §8-2 참고. **주의: 현재 좌표는 8/11 테스트 시점 로봇의 실제 위치(1강의실 앞) 기준 임시값 - 운영 시 도킹 지점 좌표로 되돌리거나 매번 실제 시작 위치에 맞게 갱신 필요.** |
| ~~AMCL이 `map→odom` tf를 지속 갱신 못 함 → Nav2가 실제 이동 없이 "목표 도달"로 오판~~ (해결) | ~~`/start_mission`이 실제로는 로봇이 전혀 안 움직였는데 몇백ms 만에 GUIDING→ARRIVED로 넘어감(2026-08-11 실측, 6m 목적지가 0.65초 만에 "도착" 처리)~~ | **2026-08-18 해결. 진짜 원인은 AMCL tf 적체가 아니라 `set_initial_pose` 좌표가 벽에 파묻혀 있던 것이었음.** 지도(map.pgm)를 직접 파싱해 좌표별 최근접 장애물 거리를 계산한 결과: 기존 `set_initial_pose`(1.079, 0.901)는 **벽에서 0.101m**, `waypoints.yaml`의 docking(0.735, -0.007)은 1.007m, room1은 0.328m. `robot_radius=0.1m`이므로 **AMCL이 믿는 출발점이 그대로 inscribed(치명) 셀**이 되고, Nav2 planner는 출발점이 lethal이면 경로 계산 자체를 거부한다 → 목적지와 무관하게 전부 `status=6 ABORTED`(`GridBased: failed to create plan`). `set_initial_pose`를 docking 좌표로 수정 후 **실주행 전 구간 성공**(room1 77초·복귀 43초·최종 오차 3mm, 거짓 도착 재현 없음). **교훈: planner가 경로를 못 만들면 목적지보다 출발점의 장애물 여유부터 확인할 것.** |
| ~~카메라가 기동 1분 후 프레임 발행을 조용히 멈춤~~ (해결) | `camera_node` 프로세스는 살아있고 CPU도 쓰는데 `/camera/image_raw`만 0Hz가 되어 ArUco 도킹이 원천 불가. PM2는 launch 래퍼만 보고 `vision`을 `online`으로 표시해 **조용한 실패**가 됨 | **2026-08-18 해결.** 단계 실험으로 규명 — camera_node 단독은 100초+ 27Hz 안정(하드웨어·드라이버 정상), camera+aruco+Nav2 동시엔 8~28Hz 요동 + 수십초 완전 정지. **원인: 카메라가 ~28fps로 발행하는데 `aruco_node`는 `detection_rate=5Hz`로만 샘플링해 나머지를 전부 버리면서 CPU/DDS 대역만 낭비** → Pi4 4코어에서 Nav2와 겹치면 카메라 파이프라인이 굶음. `aruco.launch.py`의 camera_node에 `FrameDurationLimits: [100000, 100000]`(10fps 고정) 추가로 해결. 실측: 카메라 9.2~10.1Hz 안정, aruco 5.0Hz 유지, load 7.13→5.05, PM2 재기동 후 프레임 미수신 에러 0건 |
| ~~정상 종료가 exit code 1로 끝나 PM2가 크래시로 오집계~~ (해결) | SIGINT로 정상 종료할 때마다 PM2 재시작 카운터가 소진되어, 한 번 흔들리면 **8개 노드 전체가 `stopped`로 내려감**(8/18 오후 실측) | **2026-08-18 해결.** rclpy 시그널 핸들러가 이미 컨텍스트를 닫은 뒤 `finally: rclpy.shutdown()`이 또 호출되며 `RCLError: rcl_shutdown already called`가 발생하던 것. 전 노드(8개)에 `if rclpy.ok():` 가드 추가 |
| **(미검증) ArUco 마커(ID 29) 실검출** | 도킹 마지막 단계가 아직 한 번도 성공 검증되지 않음 | 카메라 안정화 후 제자리 360° 스캔했으나 미검출(180샘플 전부 `marker_id: -1`). 프레임 자체는 정상(밝기 176, 포화 0%), 완화 파라미터로도 미검출 → 로봇이 마커를 등졌거나 화각·거리 문제로 추정되나 **배터리 소진으로 검증 중단**. **다음 세션 최우선**: 충전 후 로봇을 마커 정면에 두고 `/aruco_marker`가 29를 잡는지부터 확인할 것. 참고로 로그에 한 번 찍힌 `마커 ID 10 검출`은 물리 마커가 29뿐임을 확인했으므로 `DICT_4X4_50`의 오검출(false positive)임 |
| Nav2 + 비전(카메라+ArUco+사람감지)을 Pi4 한 대에서 동시 구동 시 CPU 부하(load average 5대, 4코어 기준)로 `/tf`·`/scan`이 간헐적으로 끊김 | 주행 중 정지·타임아웃 가능성 | `nav2_params.yaml` 튜닝 + 2026-08-11 사람감지 주기 분리·단일스레드화(`aruco_node` CPU 67.5%→52%)로 완화(완전 해결 아님). 단, 위 `/initialpose` 문제와는 무관함이 확정됨(A/B 테스트 참고) |
| USB 시리얼 장치(메인센서보드/TFT보드/OpenCR/오디오)가 간헐적으로 재연결됨 | 예전엔 이때 `hmi_bridge_node`/`com_sensor_bridge_node`가 예외처리 없이 그대로 죽었음(수동 재시작 전까지 HMI 화면·조이스틱·센서값 전부 먹통) | 2026-08-11: 두 노드 모두 시리얼 read/write를 `OSError` try/except로 감싸서 죽지 않고 재시도하도록 수정. **2026-08-12 추가 수정: 최초 연결 자체가 실패하면(USB 전원 불안정) 그대로 죽어서 PM2가 노드를 통째로 재시작해야 했던 문제 - 5회/2초 간격 재시도로 방어. 이 CDC-ACM 보드에서 `in_waiting`(ioctl) 체크가 간헐적으로 `Inappropriate ioctl for device`를 던지던 것도 발견, `in_waiting` 없이 `readline()` 자체 짧은 timeout에 맡기도록 변경 - 실측 결과 두 노드 모두 장시간(8분+) 에러 0건.** 포트 번호 자체가 바뀌는 문제는 udev 규칙으로 별도 해결(§3, §6-⑤) |
| USB 오디오 동글(Jieli UACDemoV1.0, 스피커) 재생 중 끊김/잡음 | `aplay: write error: No such device` 이후 ALSA가 카드를 아예 못 찾아(`Cannot get card index for 1`) 안내가 깨지거나 통째로 안 들림 | **2026-08-12: 볼륨 100%(유니티 게인)가 원인 중 하나로 확인 - 80%(-6.36dB)로 낮추니 반복 재생에도 끊김 없음(실측, `/tts_request` 실제 파이프라인으로 검증). `voice_node.py`의 `volume` 파라미터 기본값을 80으로 변경.** 단, USB 동글 자체가 여전히 불안정한 하드웨어라는 근본 문제는 남아있음(간헐적 재연결 등) - 완전 해결은 아니고 완화 |
| 물리 긴급정지 버튼 없음 | 긴급정지 기능이 현재 시스템에 아예 없음 | **2026-08-18 확정: GUI에도 두지 않기로 결정됨**(`com_gui` 코드 주석 근거: HMI-001 6장에 따라 안전기능은 GUI 밖에서 처리). `com_gui`는 실제로 구현 완료됐지만(§5) 긴급정지 버튼은 없음 — 이 프로젝트에서 긴급정지는 최종적으로 미구현 상태로 남음 |
| `/env_data` 발행자 없음 | 측정 로그 조회가 항상 빈 결과 | 미착수 (온습도+waypoint_id를 묶는 다리 노드 필요) |
| `WaypointSequencer`의 `trajectory_1`/`trajectory_2` 경유점 시퀀스 미정의 | 출발지→목적지, 목적지→도킹이 경유점 없이 직행(Nav2 자체 경로계획에 의존) | 설계상 문제는 아니나 문서(SR-006)와는 다름 |
| ArUco 도킹 정렬 신뢰성 | 다중 마커 동시 검출 시 잘못된 마커를 보고하던 버그는 수정(2026-08-06)했으나 실물 재검증 필요 | **2026-08-14(추정, 미커밋): 재탐색/재시도 카운터 버그, `marker.yaw_error` 크래시 버그 발견·수정(둘 다 2026-08-18 반영 완료).** **2026-08-18 후반부, 근본 원인 재진단: "15도씩 회전하며 재탐색"하던 기존 방식 자체가 문제였음** — 카메라 파이프라인 지연(Pi4에서 캡처→발행까지 수백 ms)때문에 회전 중 받은 마커 좌표는 이미 "과거의 값"이라, 로봇이 이미 마커를 지나친 뒤에야 인식하는 구조적 결함이 있었음. `DockingController`를 "좌 95도 고정 회전(오도메트리 기반 정밀 제어) → 완전 정지 → 정지 이후 촬영된 프레임만 신뢰해서 판정(stop-and-look)" 방식으로 전면 재설계. 별도로 더 정교한 7단계 상태머신을 가진 `auto_dock_node`도 신규 작성(§5 참고) — 아직 둘 다 **실물 도킹 재검증 전**, git 미커밋 |
| **(신규, 2026-08-18) `/cmd_vel` 발행 주체 3개 충돌 위험** | Nav2(controller_server), `service_manager`의 `DockingController`, `auto_dock_node`가 전부 `/cmd_vel`을 발행할 수 있음 - 나중에 도착한 메시지가 이기는 구조라 두 개 이상이 동시에 다른 값을 쏘면 로봇이 떨리거나 엉뚱하게 움직일 위험 | `auto_dock_node`는 기동 직후 IDLE 상태라 아무것도 안 쏴서 띄워두는 것 자체는 안전하나, **`/start_docking` 호출 시점에 Nav2 주행이나 `service_manager`의 DOCKING 단계가 동시에 돌지 않도록 반드시 배타적으로 관리할 것**(운영 시 DockingController와 auto_dock_node 둘 중 하나만 사용). 정리 방향(미착수): DockingController를 없애고 service_manager가 `/start_docking`을 호출한 뒤 `/docking_state`의 DONE/FAILED를 구독해 임무를 이어가는 구조로 통합 |
| ~~PM2 미도입~~ (해결) | ~~노드 비정상 종료 시 자동 재시작 없음~~ | **2026-08-11 도입, 2026-08-12 실측 확인.** `com_launch/ecosystem.config.js` + `pm2-com.service`로 전원 인가 시 8개 노드 자동 기동 + 노드별 최대 3회 자동 재시작(SR-026). `pm2 save` 저장 상태가 "전부 stopped"면 재부팅해도 자동 기동 안 될 수 있으니 부팅 후 `pm2 list` 확인 습관 필요. 운영 주의사항은 §11 참고 |
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
