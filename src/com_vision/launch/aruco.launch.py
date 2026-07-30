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
                'camera_calibration_file':
                    '/home/com/.ros/camera_info/'
                    'imx219__base_soc_i2c0mux_i2c_1_imx219_10_640x480.yaml',
                'markers_yaml_package': 'com_navigation',
                'markers_yaml_relpath': 'config/aruco_markers.yaml',
            }],
        ),
        Node(
            package='image_rotate',
            executable='image_rotate',
            name='image_rotate',
            output='screen',
            remappings=[('image', '/camera/image_raw')],
            parameters=[{
                'use_camera_info': False,
                'target_x': 0.0,
                'target_y': 1.0,
                'target_z': 0.0,
                'source_x': 0.0,
                'source_y': -1.0,
                'source_z': 0.0,
            }],
        ),
    ])
