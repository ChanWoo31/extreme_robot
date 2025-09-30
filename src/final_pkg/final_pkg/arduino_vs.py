#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import serial
import time

class ArduinoNode(Node):
    def __init__(self, serial_port='/dev/ttyARDUINO', baudrate=115200):
        super().__init__('arduino_node')

        # 시리얼 초기화
        self.ser = serial.Serial(serial_port, baudrate, timeout=1)
        time.sleep(2)

        # 최신 명령 & 모드
        self.latest_cmd = None
        # self.mode = 5  # 기본 모드를 원격주행(5번)으로 시작
        self.mode = 1
        # QoS 설정
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ROS 구독: 자율주행 명령
        self.sub_cmd = self.create_subscription(String, '/direction_cmd', self.cmd_callback, qos)

        # ROS 퍼블리셔: 아두이노 데이터 & 모드
        self.pub_remote = self.create_publisher(String, '/arm/manual', 10)
        self.pub_mode = self.create_publisher(Int32, '/current_mode', 10)

        # 타이머: 20Hz 업데이트
        self.create_timer(0.05, self.update)

        self.get_logger().info(f"ArduinoNode started. Serial: {serial_port}, Baudrate: {baudrate}")

    def cmd_callback(self, msg: String):
        # 자율주행 명령은 모드가 1~4일 때만 사용
        if self.mode != 5:
            self.latest_cmd = msg.data
            print(f"Received command: {self.latest_cmd}")

    def update(self):
        # 1. 아두이노로 명령 전송 (자율주행 모드일 때만)
        if self.latest_cmd and self.mode != 5:
            self.ser.reset_input_buffer()
            send_msg = f"[Con]{self.latest_cmd}\n"
            self.ser.write(send_msg.encode('utf-8'))
            self.get_logger().info(f"Sent to Arduino (Auto mode): {send_msg.strip()}")
            self.latest_cmd = None

        # 2. 아두이노에서 데이터 수신
        while self.ser.in_waiting > 0:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line:
                # [RX] 접두사 제거
                if line.startswith("[RX] "):
                    line = line[5:]
                    print(line)
                
                print(line)
                # Mode 변경 신호라면 업데이트 + 퍼블리시
                if line.startswith("Mode"):
                    print("SSSSSSSSS")
                    try:
                        new_mode = int(line[-1])  # "Mode5" → 5
                        self.mode = new_mode
                        self.get_logger().info(f"Mode changed to: {self.mode}")
                        # 모드 퍼블리시
                        mode_msg = Int32()
                        mode_msg.data = self.mode
                        self.pub_mode.publish(mode_msg)
                    except ValueError:
                        self.get_logger().warn(f"Invalid mode string: {line}")
                    continue

                # 원격주행 모드(5번)일 때만 퍼블리시
                if self.mode == 5:
                    msg = String()
                    msg.data = line
                    self.pub_remote.publish(msg)
                    self.get_logger().info(f"Published (Remote mode): {line}")
                    print("5555")
            
        mode_msg = Int32()
        mode_msg.data = self.mode
        self.pub_mode.publish(mode_msg)
        print(self.mode)

def main(args=None):
    rclpy.init(args=args)
    # 일반적인 아두이노 포트들을 순서대로 시도
    possible_ports = ['/dev/ttyACM0', '/dev/ttyARDUINO', '/dev/ttyUSB0', '/dev/ttyUSB1']

    for port in possible_ports:
        try:
            node = ArduinoNode(serial_port=port, baudrate=115200)
            print(f"Successfully connected to {port}")
            break
        except:
            print(f"Failed to connect to {port}, trying next port...")
            continue
    else:
        print("No available serial ports found!")
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
