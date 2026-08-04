from setuptools import find_packages, setup

package_name = 'com_status'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='KOM',
    maintainer_email='kom@example.com',
    description='status 노드 - RobotState 조립 및 측정 로그 조회 서비스',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'status_node = com_status.status_node:main',
        ],
    },
)
