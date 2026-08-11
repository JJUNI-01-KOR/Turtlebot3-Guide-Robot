"""com_launch: README §8(실행 방법) 8단계를 ros2 launch 하나로 통합.

기존에는 SSH 터미널 8개에 아래 명령을 하나씩 따로 실행해야 했다:
  1) turtlebot3_bringup robot.launch.py   (베이스+라이다+OpenCR)
  2) com_bringup navigation.launch.py     (Nav2)
  3) com_vision aruco.launch.py            (카메라+ArUco)
  4) com_sensors com_sensor_bridge_node    (메인센서보드)
  5) com_sensors hmi_bridge_node           (TFT보드, serial_port 오버라이드 필수)
  6) com_service service_manager_node      (임무 상태머신)
  7) com_status status_node                (상태 종합)
  8) com_service com_voice_node            (음성, service_manager가 /tts_request 발행)

이 launch 파일은 그 8개를 전부 하나의 `ros2 launch` 프로세스 밑에서 관리한다.
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
- 2026-08-11: udev 규칙(99-com-robot-serial.rules, 시리얼 번호 기반)으로
  /dev/com_sensorboard, /dev/com_tftboard 고정 심볼릭 링크를 만들어서 기본값으로
  씀 - ttyACM 번호가 재부팅/재연결 때마다 바뀌는 문제(실물 세션에서 여러 번 겪음,
  hmi_bridge_node가 죽은 포트를 계속 붙잡고 있어서 크래시까지 난 적 있음)를 회피.
  이 udev 규칙이 없는 새 라즈베리파이라면 다시 ttyACM0/ttyACM2 등으로 오버라이드
  필요(README §3 참고).
- 임무 시작은 TFT+조이스틱 물리 UI 또는:
    ros2 service call /start_mission com_interfaces/srv/StartMission \
      "{destination_id: room1}"
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

    com_navigation_share = get_package_share_directory('com_navigation')
    default_map = os.path.join(com_navigation_share, 'maps', 'map.yaml')
    default_params = os.path.join(com_navigation_share, 'config', 'nav2_params.yaml')

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
        DeclareLaunchArgument('sensor_serial_port', default_value='/dev/com_sensorboard',
                               description='com_sensor_bridge_node (메인센서보드) - '
                                            '99-com-robot-serial.rules 고정 심볼릭 링크'),
        DeclareLaunchArgument('hmi_serial_port', default_value='/dev/com_tftboard',
                               description='hmi_bridge_node (TFT보드) - '
                                            '99-com-robot-serial.rules 고정 심볼릭 링크'),

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
    ])
