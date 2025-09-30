#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # 1. controller_node_pub - 아두이노 시리얼 메시지를 ROS2 토픽으로 발행
        Node(
            package='final_pkg',
            executable='arduino_vs',
            name='arduino_vs',
            output='screen',
            parameters=[],
            remappings=[]
        ),

        Node(
            package='final_pkg',
            executable='IMU_pub',
            name='IMU_pub',
            output='screen',
            parameters=[],
            remappings=[]
        ),

        Node(
            package='final_pkg',
            executable='vision',
            name='vision',
            output='screen',
            parameters=[],
            remappings=[]
        ),

        Node(
            package='final_pkg',
            executable='Lidar_final0927',
            name='Lidar_final0927',
            output='screen',
            parameters=[],
            remappings=[]
        ),

        Node(
            package='final_pkg',
            executable='main_code',
            name='main_code',
            output='screen',
            parameters=[],
            remappings=[]
        ),

        Node(
            package='final_pkg',
            executable='unified_arm_controller',
            name='unified_arm_controller',
            output='screen',
            parameters=[],
            remappings=[]
        ),
    ])