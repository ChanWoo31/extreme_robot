#!/usr/bin/env python3
import math
import serial
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header



def parse_csv_line(line: str):
    # 기대 포맷: roll,pitch,yaw,ax,ay,az
    try:
        parts = [p.strip() for p in line.strip().split(',')]
        if len(parts) < 6:
            return None
        vals = list(map(float, parts[:6]))
        r, p, y, ax, ay, az = vals
        return r, p, y, ax, ay, az
    except Exception:
        return None

class ImuSerialNode(Node):
    def __init__(self):
        super().__init__('imu_serial_node')

        # parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'imu_link')

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # pubs
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 50)
        self.pub_rpy = self.create_publisher(Vector3, '/imu/rpy', 50)

        # serial open
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.2)
            self.get_logger().info(f'Opened {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial: {e}')
            raise

        self.last_msg = None
        self.run = True
        self.th = threading.Thread(target=self.reader_thread, daemon=True)
        self.th.start()

        # publish timer
        self.timer = self.create_timer(0.01, self.publish_latest)  # 100 Hz

    def reader_thread(self):
        while self.run:
            try:
                raw = self.ser.readline().decode(errors='ignore')
                parsed = parse_csv_line(raw)
                if not parsed:
                    continue
                r, p, y, ax, ay, az = parsed
                # pack IMU message
                hdr = Header()
                hdr.stamp = self.get_clock().now().to_msg()
                hdr.frame_id = self.frame_id

                imu_msg = Imu()
                imu_msg.header = hdr

                

                # angular_velocity unknown -> leave 0, mark unknown
                imu_msg.angular_velocity_covariance[0] = -1.0

                # linear acceleration (m/s^2) from CSV
                imu_msg.linear_acceleration.x = ax
                imu_msg.linear_acceleration.y = ay
                imu_msg.linear_acceleration.z = az
                imu_msg.linear_acceleration_covariance[0] = -1.0

                # rpy in degrees (for convenience)
                rpy_msg = Vector3(x=r, y=p, z=y)

                # store latest
                self.last_msg = (imu_msg, rpy_msg)
            except Exception as e:
                self.get_logger().warn(f'reader error: {e}')

    def publish_latest(self):
        if self.last_msg is None:
            return
        imu_msg, rpy_msg = self.last_msg
        # update timestamp for publish tick
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_imu.publish(imu_msg)
        self.pub_rpy.publish(rpy_msg)

    def destroy_node(self):
        self.run = False
        try:
            if hasattr(self, 'ser'):
                self.ser.close()
        except:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = ImuSerialNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
