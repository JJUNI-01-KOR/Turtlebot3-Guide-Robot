from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='com_vision',
            executable='aruco_node',
            name='aruco',
            output='screen',
            parameters=[{
                'aruco_dictionary': 'DICT_4X4_50',
                'marker_size': 0.10,
                'detection_rate': 5.0,
                'person_confidence_threshold': 0.5,
                'camera_frame_timeout': 1.0,
                # 실제 캘리브레이션 완료 후 파일 경로로 채워넣기
                'camera_calibration_file': '',
                'markers_yaml_package': 'com_navigation',
                'markers_yaml_relpath': 'config/aruco_markers.yaml',
            }],
        )
    ])
