from setuptools import find_packages, setup

package_name = 'final_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/final.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='han',
    maintainer_email='cwcw0301@gmail.com',
    description='Final package for extreme robot project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unified_arm_controller = final_pkg.unified_arm_controller:main',
            'arduino_vs = final_pkg.arduino_vs:main',
            'auto_drive_code = final_pkg.auto_drive_code:main',
            'IMU_pub = final_pkg.IMU_pub:main',
            'Lidar_final0927 = final_pkg.Lidar_final0927:main',
            'main_code = final_pkg.main_code:main',
            'vision = final_pkg.vision:main',
        ],
    },
)
