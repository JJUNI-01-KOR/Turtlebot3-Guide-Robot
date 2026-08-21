// com_launch/ecosystem.config.js
// README §8(실행 순서) 8단계를 PM2로 자동 기동 + 노드별 최대 3회 재시작(SR-026).
//
// 목표:
//   1) 전원 인가 시 자동 실행 (pm2 startup + pm2 save)
//   2) WiFi(GUI 연결)와 무관하게 로봇 내부에서 계속 동작 (SR-023) — 각 노드가
//      로봇 로컬에서 독립 프로세스로 뜨기 때문에 GUI가 안 붙어있어도 그대로 동작함.
//      GUI 쪽은 이미 com_gui의 연결끊김 배지(ros_worker의 연결 감시 로직)가 처리.
//   3) 노드가 죽으면 PM2가 최대 3회까지 자동 재시작
//
// robot_all.launch.py(ros2 launch 통합본)와의 차이:
//   robot_all.launch.py 하나로 묶으면 프로세스가 1개라서, 그 안의 노드 하나가 죽어도
//   launch 프로세스 자체는 안 죽는 경우가 많아 PM2가 "이상 없음"으로 오판할 수 있다.
//   그래서 PM2는 8개를 "각각 별도 프로세스"로 관리한다 (SR-026이 요구하는 노드 단위
//   재시작을 정확히 구현하려면 이 방식이 맞음).
//
// 사용법 (로봇, com@COM 에서):
//   cd ~/com_ws/src/com_launch
//   pm2 start ecosystem.config.js
//   pm2 status
//   pm2 logs service_manager        # 특정 노드 로그만 보기
//   pm2 restart hmi_bridge          # 특정 노드만 재시작
//   pm2 stop all && pm2 delete all  # 전체 중지/삭제
//
// 부팅 시 자동 시작 (최초 1회만 설정하면 됨):
//   pm2 startup                     # 안내되는 sudo 명령을 그대로 복사해서 실행
//   pm2 save                        # 지금 떠있는 프로세스 목록을 부팅용으로 저장
//   (이후 전원을 완전히 뽑았다 켜도 pm2가 systemd로 자동 기동됨)

const path = require('path');

const RUN = path.join(__dirname, 'scripts', 'run_node.sh');

// SR-026 "최대 3회 자동 재시작"
const COMMON = {
  interpreter: 'bash',
  autorestart: true,
  max_restarts: 3,
  min_uptime: '10s',     // 10초 이상 살아있어야 "정상 기동"으로 카운트 (배터리 저전압
                          // 크래시처럼 90초 뒤 죽는 경우는 재시작 카운트에 안 들어감)
  restart_delay: 2000,
  // out_file/error_file을 지정하지 않으면 PM2가 앱 이름별로 자동 분리해서
  // ~/.pm2/logs/<name>-out.log, <name>-error.log 로 각각 저장한다.
  // (이전 버전은 8개 앱이 전부 같은 파일을 같이 써서 로그가 뒤섞이는 버그가 있었음)
};

module.exports = {
  apps: [
    {
      ...COMMON,
      name: 'bringup',
      script: RUN,
      args: 'launch turtlebot3_bringup robot.launch.py',
    },
    {
      ...COMMON,
      name: 'navigation',
      script: RUN,
      args: 'launch com_bringup navigation.launch.py',
      // navigation.launch.py 내부 기본값(map/params_file)을 그대로 사용.
      // 지도가 바뀌면 아래처럼 args만 바꿔주면 됨:
      // args: 'launch com_bringup navigation.launch.py map:=/home/com/com_ws/src/com_navigation/maps/new_map.yaml',
    },
    {
      ...COMMON,
      name: 'vision',
      script: RUN,
      args: 'launch com_vision aruco.launch.py',   // 카메라+ArUco 통합 launch
    },
    {
      ...COMMON,
      name: 'sensor_bridge',
      script: RUN,
      // 99-com-robot-serial.rules(팀 기존 udev 규칙)의 고정 심볼릭 링크 사용.
      // ttyACM 번호에 의존하지 않아 재부팅/재연결 순서가 바뀌어도 안전하다.
      args: 'run com_sensors com_sensor_bridge_node --ros-args -p serial_port:=/dev/com_sensorboard',
    },
    {
      ...COMMON,
      name: 'hmi_bridge',
      script: RUN,
      args: 'run com_sensors hmi_bridge_node --ros-args -p serial_port:=/dev/com_tftboard',
    },
    {
      ...COMMON,
      name: 'service_manager',
      script: RUN,
      args: 'run com_service service_manager_node',
    },
    {
      ...COMMON,
      name: 'status',
      script: RUN,
      args: 'run com_status status_node',
    },
    {
      ...COMMON,
      name: 'voice',
      script: RUN,
      args: 'run com_service com_voice_node',
    },
  ],
};
