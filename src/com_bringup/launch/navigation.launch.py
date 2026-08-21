import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
ROS_DISTRO = os.environ.get('ROS_DISTRO', 'humble')


def generate_launch_description():
    default_map = os.path.join(
        get_package_share_directory('com_navigation'), 'maps', 'map.yaml'
    )
    # com_navigation/config/nav2_params.yaml: turtlebot3_navigation2 기본값에서
    # 라즈베리파이4 실물 CPU 부하(bt_loop_duration, DWB 샘플 수 등)를 낮추고
    # use_sim_time 잔재 버그(global_costmap/recoveries_server)를 정정한 프로젝트
    # 전용 파라미터 파일 (2026-08-06). 필요 시 params_file 인자로 원래 기본값
    # (turtlebot3_navigation2 패키지 제공)으로 되돌릴 수 있음.
    default_params = os.path.join(
        get_package_share_directory('com_navigation'), 'config', 'nav2_params.yaml'
    )

    map_arg = LaunchConfiguration('map')
    params_arg = LaunchConfiguration('params_file')
    use_sim_time_arg = LaunchConfiguration('use_sim_time')

    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='com_navigation 패키지에 저장된 지도(map.yaml) 경로'
        ),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description=(
                'Nav2 파라미터 파일 '
                f'(TURTLEBOT3_MODEL={TURTLEBOT3_MODEL} 기준 turtlebot3_navigation2 기본값)'
            )
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Gazebo 시뮬레이션 시간 사용 여부'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_launch_dir, '/bringup_launch.py']
            ),
            launch_arguments={
                'map': map_arg,
                'use_sim_time': use_sim_time_arg,
                'params_file': params_arg,
            }.items(),
        ),
    ])
