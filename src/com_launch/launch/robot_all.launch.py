"""com_launch: README §8(실행 방법) 8단계를 ros2 launch 하나로 통합.
 
기존에는 SSH 터미널 8개에 아래 명령을 하나씩 따로 실행해야 했다:
  1) turtlebot3_bringup robot.launch.py   (베이스+라이다+OpenCR)
  2) com_bringup navigation.launch.py     (Nav2)
  3) com_vision aruco.launch.py            (카메라+ArUco)
  4) com_sensors com_sensor_bridge_node    (메인센서보드)
  5) com_sensors hmi_bridge_node           (TFT보드, serial_port 오버라이드 필수)
  6) com_service service_manager_node      (임무 상태머신)
  7) com_status status_node                (상태 종합)
  8) com_service com_voice_node            (음성 안내)
  9) com_service auto_dock_node            (자동 충전 도킹)  ← 2026-08-18 추가
 
이 launch 파일은 그 9개를 전부 하나의 `ros2 launch` 프로세스 밑에서 관리한다.
SSH에서 실행할 때는 README §11 주의사항대로 노드 여러 개를 한 셸 명령에
욱여넣지 말라고 되어 있는데, 이 launch 파일 자체는 "SSH 명령 1개"이고
launch가 내부적으로 자식 프로세스를 관리하는 방식이라 그 문제와 다르다
(bash가 여러 `&`/`disown`을 기다리며 멈추는 상황이 아님). 다만 완전히
분리해서 백그라운드로 띄우고 싶으면 다음과 같이 실행:
 
  setsid nohup ros2 launch com_launch robot_all.launch.py \
      > ~/com_launch.log 2>&1 < /dev/null &
 
주의:
- turtlebot3_bringup은 LDS_MODEL=LDS-03, TURTLEBOT3_MODEL=burger 환경변수를
  실행 "전에" export 해둬야 한다 (launch 파일 안에서 SetEnvironmentVariable로도
  넣어주지만, launch 프로세스 시작 전에 export 해두는 쪽이 더 안전하다).
- 배터리 11V대(저전압)에서는 OpenCR이 약 90초 후 시리얼 통신 실패로 크래시하는
  현상이 실측됐다 (README §8-1) — 완충 후 기동할 것.
- 시리얼 포트는 udev 고정 링크(/dev/com_sensorboard, /dev/com_tftboard)를 쓰므로
  평소엔 오버라이드가 필요 없다. 새 라즈베리파이라 udev 규칙이 아직 없다면
  ttyACM 번호를 직접 넘길 것:
    ros2 launch com_launch robot_all.launch.py hmi_serial_port:=/dev/ttyACM2
- 임무 시작은 TFT+조이스틱 물리 UI 또는:
    ros2 service call /start_mission com_interfaces/srv/StartMission \
      "{destination_id: class1}"
 
[/cmd_vel 발행자 충돌 주의 - 2026-08-18]
  이 launch에는 /cmd_vel을 발행할 수 있는 주체가 셋이다:
    (a) Nav2 (controller_server -> velocity_smoother)
    (b) service_manager_node 내부 DockingController
    (c) auto_dock_node
  ROS2는 마지막에 도착한 메시지가 이기므로, 둘 이상이 동시에 20Hz로
  다른 값을 쏘면 로봇이 떨거나 엉뚱하게 움직인다.
  auto_dock_node는 기동 직후 IDLE 상태라 아무것도 발행하지 않으므로
  띄워두는 것 자체는 안전하다. 다만 /start_docking 호출 시점에
  Nav2 주행이나 service_manager의 도킹 단계가 동시에 돌지 않도록
  임무 상태머신 쪽에서 반드시 배타적으로 관리할 것.
  (정리 방향: DockingController를 삭제하고 service_manager가
   /start_docking 서비스를 호출한 뒤 /docking_state의 DONE/FAILED를
   구독해 임무를 이어가는 구조로 통합)
"""
import os
 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
 
 
def generate_launch_description():
    # --- 실행 인자 (launch 파일 하나로 여러 상황에 대응하기 위함) ---
    turtlebot3_model = LaunchConfiguration('turtlebot3_model')
    lds_model = LaunchConfiguration('lds_model')
    map_yaml = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('params_file')
    sensor_serial_port = LaunchConfiguration('sensor_serial_port')
    hmi_serial_port = LaunchConfiguration('hmi_serial_port')
    auto_dock_params_file = LaunchConfiguration('auto_dock_params')
 
    com_navigation_share = get_package_share_directory('com_navigation')
    default_map = os.path.join(com_navigation_share, 'maps', 'map.yaml')
    default_params = os.path.join(com_navigation_share, 'config', 'nav2_params.yaml')
 
    com_service_share = get_package_share_directory('com_service')
    default_dock_params = os.path.join(
        com_service_share, 'config', 'auto_dock_params.yaml')
 
    turtlebot3_bringup_launch = os.path.join(
        get_package_share_directory('turtlebot3_bringup'), 'launch', 'robot.launch.py')
    com_bringup_nav_launch = os.path.join(
        get_package_share_directory('com_bringup'), 'launch', 'navigation.launch.py')
    com_vision_aruco_launch = os.path.join(
        get_package_share_directory('com_vision'), 'launch', 'aruco.launch.py')
 
    return LaunchDescription([
        DeclareLaunchArgument('turtlebot3_model', default_value='burger'),
        DeclareLaunchArgument('lds_model', default_value='LDS-03',
                               description='LDS-03이면 turtlebot3_bringup이 '
                                            'coin_d4_driver를 자동 포함'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),
        # 2026-08-19: udev 규칙(com_launch/udev/99-com-robot-serial.rules)이
        # 시리얼 번호 기준으로 만들어주는 고정 심볼릭 링크를 쓴다. ttyACM 번호는
        # 재부팅/재연결마다 바뀌므로 여기에 ttyACM0/2를 박아두면 두 보드가 뒤바뀐다
        # (노드 기본값과 ecosystem.config.js는 이미 고정 경로를 쓰고 있었다).
        DeclareLaunchArgument('sensor_serial_port', default_value='/dev/com_sensorboard',
                               description='com_sensor_bridge_node (메인센서보드)'),
        DeclareLaunchArgument('hmi_serial_port', default_value='/dev/com_tftboard',
                               description='hmi_bridge_node (TFT보드)'),
        DeclareLaunchArgument('auto_dock_params', default_value=default_dock_params,
                               description='auto_dock_node 파라미터 파일 '
                                            '(마커 ID, 제어 게인, 후진 오프셋 등)'),
 
        SetEnvironmentVariable('TURTLEBOT3_MODEL', turtlebot3_model),
        SetEnvironmentVariable('LDS_MODEL', lds_model),
 
        # 8-1. 베이스 + 라이다 + OpenCR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot3_bringup_launch),
        ),
 
        # 8-2. Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(com_bringup_nav_launch),
            launch_arguments={
                'map': map_yaml,
                'params_file': nav2_params_file,
                'use_sim_time': 'false',
            }.items(),
        ),
 
        # 8-3. 비전 (카메라 + ArUco)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(com_vision_aruco_launch),
        ),
 
        # 8-4. 센서 + HMI
        Node(
            package='com_sensors',
            executable='com_sensor_bridge_node',
            name='com_sensor_bridge_node',
            output='screen',
            parameters=[{'serial_port': sensor_serial_port}],
        ),
        Node(
            package='com_sensors',
            executable='hmi_bridge_node',
            name='hmi_bridge_node',
            output='screen',
            parameters=[{'serial_port': hmi_serial_port}],
        ),
 
        # 8-5. 임무 상태머신 + 상태 종합 + 음성
        Node(
            package='com_service',
            executable='service_manager_node',
            name='service_manager',
            output='screen',
        ),
        Node(
            package='com_status',
            executable='status_node',
            name='status',
            output='screen',
        ),
        Node(
            package='com_service',
            executable='com_voice_node',
            name='voice',
            output='screen',
        ),
 
        # 8-6. 자동 충전 도킹 (2026-08-18 추가)
        #   좌표이동 -> 마커탐색 -> 정면정렬 -> 최소거리접근 -> 180도회전 -> 후진
        #
        #   name은 반드시 'com_auto_dock_node' 여야 한다. auto_dock_params.yaml의
        #   최상위 키가 'com_auto_dock_node:' 이기 때문에, 여기서 이름을 다르게
        #   주면 파라미터가 "에러 없이" 하나도 적용되지 않고 전부 기본값으로
        #   돌아간다 (target_marker_id가 29가 아닌 1이 되어 마커를 통째로 무시).
        #   ROS2 파라미터 파일에서 가장 흔하고 가장 찾기 어려운 함정.
        #
        #   기동 직후에는 IDLE 상태라 /cmd_vel을 발행하지 않으므로 항상 띄워둬도
        #   안전하다. 실제 동작은 아래 서비스를 호출해야 시작된다:
        #     ros2 service call /start_docking  std_srvs/srv/Trigger
        #     ros2 service call /cancel_docking std_srvs/srv/Trigger
        #   진행 상황은 /docking_state 토픽(String)으로 확인:
        #     ros2 topic echo /docking_state
        Node(
            package='com_service',
            executable='auto_dock_node',
            name='com_auto_dock_node',
            output='screen',
            parameters=[auto_dock_params_file],
        ),
    ])
