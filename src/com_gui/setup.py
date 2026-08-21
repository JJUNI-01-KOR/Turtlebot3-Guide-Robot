from setuptools import find_packages, setup

package_name = 'com_gui'

setup(
    name=package_name,
    version='2.0.0',
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
    description='COM 방문자 안내 로봇 관제 대시보드 (PyQt5 다크 테마)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_node = com_gui.gui_node:main',
        ],
    },
)
