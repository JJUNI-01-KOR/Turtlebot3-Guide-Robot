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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='com',
    maintainer_email='com@todo.todo',
    description='TODO: Package description',
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
