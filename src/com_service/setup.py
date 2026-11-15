from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'com_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # 2026-08-19: 안내 음성. 예전엔 설치가 안 돼서 voice_node가 소스 트리
        # 경로를 직접 읽고 있었고, 워크스페이스를 옮기면 조용히 무음이 됐다.
        (os.path.join('share', package_name, 'audio'), glob('audio/*.wav')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='com',
    maintainer_email='com@todo.todo',
    description='임무 상태머신과 ArUco 정밀 도킹(service_manager), 음성 안내(voice), 자동 도킹(auto_dock) 노드',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'service_manager_node = com_service.service_manager_node:main',
            'com_voice_node = com_service.voice_node:main',
            'auto_dock_node = com_service.auto_dock_node:main',
        ],
    },
)
