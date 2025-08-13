import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading

class SerialBridge(Node):
    def __init__(self):
        super().__init__('arduino_serial_bridge')

        # Publish: 아두이노 -> ROS2
        self.pub_arduino = self.create_publisher(String, 'arduino_data', 10)

        # Subscribe: ROS2 -> 아두이노 (go 등)
        self.sub_cmd = self.create_subscription(
            String, 'go_cmd', self.on_cmd, 10
        )

        # 시리얼 설정 (개행은 \n 기준)
        self.ser = serial.Serial('/dev/ttyACM0', 57600, timeout=0.05)
        self.lock = threading.Lock()

        # 주기적으로 읽기
        self.timer = self.create_timer(0.02, self.read_serial)

        self.get_logger().info('SerialBridge started')

    def on_cmd(self, msg: String):
        """ROS2에서 들어온 명령을 시리얼로 전송"""
        data = msg.data.strip()
        # 관례적으로 개행 포함해 전송
        line = (data + '\n').encode('utf-8')
        try:
            with self.lock:
                self.ser.write(line)
                self.ser.flush()
            self.get_logger().info(f'Sent -> Arduino: {data}')
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')

    def read_serial(self):
        """시리얼로부터 수신 -> arduino_data 토픽 publish"""
        try:
            if self.ser.in_waiting > 0:
                with self.lock:
                    raw = self.ser.readline()
                if not raw:
                    return
                line = raw.decode('utf-8', errors='ignore').strip()
                if line:
                    msg = String()
                    msg.data = line
                    self.pub_arduino.publish(msg)
                    # 필요시 로그 줄이려면 주석
                    self.get_logger().info(f'From Arduino <- {line}')
        except Exception as e:
            self.get_logger().error(f'Serial read error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
