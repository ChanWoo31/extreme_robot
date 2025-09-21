import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialReceiver(Node):
    def __init__(self):
        super().__init__('serial_receiver')
        self.publisher_ = self.create_publisher(String, 'joystick_cmd', 10)

        # 아두이노 포트와 보드레이트 맞춰야 함
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line:
                # [RX] 접두사 제거
                if line.startswith("[RX] "):
                    line = line[5:]  # '[RX] ' 제거

                msg = String()
                msg.data = line
                self.publisher_.publish(msg)
                self.get_logger().info(f"Published: {line}")


def main(args=None):
    rclpy.init(args=args)
    node = SerialReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()