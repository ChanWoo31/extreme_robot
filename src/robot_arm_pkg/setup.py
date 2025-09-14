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
            'robot_arm_control_test2 = robot_arm_pkg.robot_arm_control_test2:main',
            'motor_test = robot_arm_pkg.motor_test:main',
            'serial_test = robot_arm_pkg.serial_test:main',
            'dynamixel_bus_node = robot_arm_pkg.dynamixel_bus_node:main',
            'imu_node = robot_arm_pkg.imu_node:main',
            'robotarm_groupsync = robot_arm_pkg.robotarm_groupsync:main',
            'arm_server = robot_arm_pkg.arm_server:main',
        ],
    },
)
