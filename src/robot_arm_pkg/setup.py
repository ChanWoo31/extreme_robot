from setuptools import find_packages, setup
import os
from pathlib import Path

def read_requirements():
    here = Path(__file__).resolve().parent
    candidates = [
        here / 'requirements.txt',                                  # 빌드 폴더에 있을 경우
        (here.parent.parent / 'src' / 'robot_arm_pkg' / 'requirements.txt'),  # 소스 폴더
    ]
    for p in candidates:
        if p.exists():
            return [l.strip() for l in p.read_text().splitlines()
                    if l.strip() and not l.startswith('#')]
    return []

package_name = 'robot_arm_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=read_requirements(),
    zip_safe=True,
    maintainer='han',
    maintainer_email='cwcw0301@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_server = robot_arm_pkg.arm_server:main',
            'unified_arm_controller = robot_arm_pkg.unified_arm_controller:main',
            'manual_publisher = robot_arm_pkg.manual_publisher:main',
            'controller_node_pub = robot_arm_pkg.controller_node_pub:main',
            'dynamixel_bus_node = robot_arm_pkg.dynamixel_bus_node:main',
            'dynamixel_manual_mode = robot_arm_pkg.dynamixel_manual_mode:main',
            'realsense = robot_arm_pkg.realsense:main',
            'real_test = robot_arm_pkg.real_test:main',
            'robotarm = robot_arm_pkg.robotarm:main',
            'robotarm_groupsync = robot_arm_pkg.robotarm_groupsync:main',
            'robot_arm_control_t = robot_arm_pkg.robot_arm_control_t:main',
            'robot_arm_sj = robot_arm_pkg.robot_arm_sj:main',
            'serial_test = robot_arm_pkg.serial_test:main',
            'test_opencv = robot_arm_pkg.test_opencv:main',
            'vision = robot_arm_pkg.vision:main',
            'motor_test = robot_arm_pkg.motor_test:main',
        ],
    },
)
