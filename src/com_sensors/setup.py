from setuptools import find_packages, setup

package_name = 'com_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='com',
    maintainer_email='com@todo.todo',
    description='아두이노 보드 시리얼 브릿지: 메인센서보드(serial_bridge), TFT보드(hmi_bridge)',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'com_sensor_bridge_node = com_sensors.serial_bridge_node:main',
            'hmi_bridge_node = com_sensors.hmi_bridge_node:main',
        ],
    },
)
