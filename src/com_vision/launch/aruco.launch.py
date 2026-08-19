from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 2026-08-18 코드 리뷰: 이 경로가 하드코딩이라 SD카드 재설치/카메라 교체/
    # 다른 계정에서 실행하면 aruco_node가 캘리브레이션을 못 읽고 조용히
    # 깨진다. robot_all.launch.py의 sensor_serial_port 등과 동일하게
    # 오버라이드 가능한 launch argument로 뺀다(기본값은 그대로 유지).
    default_calibration_file = (
        '/home/com/.ros/camera_info/'
        'imx219__base_soc_i2c0mux_i2c_1_imx219_10_640x480.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_calibration_file', default_value=default_calibration_file,
            description='ArUco용 카메라 캘리브레이션 yaml 경로',
        ),
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[{
                # camera_ros 기본 픽셀포맷(NV12/NV21)은 cv_bridge가 인식 못 해서
                # aruco_node의 imgmsg_to_cv2(desired_encoding='bgr8')가 매 프레임 실패함.
                # RGB888로 고정해야 aruco_node가 실제로 프레임을 받는다.
                'format': 'RGB888',
                'width': 640,
                'height': 480,
                # 2026-08-19: 프레임 간격 하한/상한(us). aruco_node는 detection_rate
                # =5Hz로만 소비하는데 기본 설정에서는 카메라가 약 17Hz로 뿜어서,
                # 0.92MB짜리 원본 프레임을 초당 17번 직렬화/전송/역직렬화하느라
                # CPU를 그냥 버렸다(실측 camera_node 17.7%, load 4.46).
                # 약 10Hz로 제한하니 camera_node 9.6%, load 3.70으로 떨어졌다.
                # 검출 주기(5Hz)보다는 여유가 있어야 하므로 더 낮추지는 말 것.
                'FrameDurationLimits': [100000, 100000],
            }],
        ),
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
                'camera_calibration_file': LaunchConfiguration('camera_calibration_file'),
                'markers_yaml_package': 'com_navigation',
                'markers_yaml_relpath': 'config/aruco_markers.yaml',
            }],
        ),
    ])
