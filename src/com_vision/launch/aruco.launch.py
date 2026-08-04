from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
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
                'camera_calibration_file':
                    '/home/com/.ros/camera_info/'
                    'imx219__base_soc_i2c0mux_i2c_1_imx219_10_640x480.yaml',
                'markers_yaml_package': 'com_navigation',
                'markers_yaml_relpath': 'config/aruco_markers.yaml',
            }],
        ),
    ])
