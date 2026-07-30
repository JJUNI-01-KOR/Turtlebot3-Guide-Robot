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
    default_params = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'param', ROS_DISTRO, f'{TURTLEBOT3_MODEL}.yaml'
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
