#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray
import serial
import math
import re

# ================================
# 시리얼 설정
# ================================
COM_PORT = "/dev/ttyIMU"
BAUDRATE = 115200
ser = serial.Serial(port=COM_PORT, baudrate=BAUDRATE, timeout=0.01)  # timeout 짧게

# ================================
# 쿼터니안 → Euler 변환 (degree)
# ================================
def quaternion_to_euler(x, y, z, w):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w*y - z*x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi/2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w*z + x*y)
    cosy_cosp = 1 - 2 * (y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

# ================================
# 시리얼 라인 파싱
# ================================
def parse_quaternion_line(line):
    line = line.replace('*','')
    numbers = re.findall(r'[-+]?\d*\.\d+|\d+', line)
    if len(numbers) < 4:
        return None
    try:
        qx, qy, qz, qw = map(float, numbers[:4])
        return qx, qy, qz, qw
    except:
        return None

# ================================
# ROS2 퍼블리셔 노드
# ================================
class EbimuPublisher(Node):
    def __init__(self):
        super().__init__('ebimu_publisher')
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(Float32MultiArray, 'ebimu_rpy', qos_profile)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz

    def timer_callback(self):
        try:
            # 버퍼에 데이터가 있을 때만 처리
            while ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                parsed = parse_quaternion_line(line)
                if parsed is None:
                    continue

                qx, qy, qz, qw = parsed

                # 쿼터니안 출력
                print(f"Quaternion: qx={qx}, qy={qy}, qz={qz}, qw={qw}")

                # Euler 계산
                roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)

                # Euler 출력
                print(f"Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°\n")

                # 퍼블리시
                msg = Float32MultiArray()
                msg.data = [roll, pitch, yaw]
                self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"IMU parse error: {e}")

# ================================
# 메인
# ================================
def main(args=None):
    rclpy.init(args=args)
    node = EbimuPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
