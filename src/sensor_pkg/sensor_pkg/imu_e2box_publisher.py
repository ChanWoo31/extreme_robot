# sensor_pkg/imu_e2box_publisher.py
import re
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import serial

NUM_FIELDS = 10  # ax,ay,az,gx,gy,gz,qw,qx,qy,qz

def safe_float(tok: str):
    """
    숫자 앞뒤에 붙은 비숫자 기호(*,#,단위 등) 제거 후 float 변환.
    실패 시 None 반환.
    """
    if tok is None:
        return None
    s = tok.strip()
    # 앞쪽 비숫자 제거(부호/소수/지수만 허용)
    s = re.sub(r'^[^0-9+\-\.eE]+', '', s)
    # 숫자 패턴만 취함(뒤 체크섬, 단위 제거)
    m = re.match(r'^[+\-]?\d+(?:\.\d+)?(?:[eE][+\-]?\d+)?', s)
    if not m:
        return None
    try:
        return float(m.group(0))
    except Exception:
        return None

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # ROS 파라미터
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('topic', 'imu/data_raw')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('debug_raw', False)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = int(self.get_parameter('baud').get_parameter_value().integer_value or 115200)
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        rate_hz = float(self.get_parameter('rate_hz').get_parameter_value().double_value or 100.0)
        self.debug_raw = bool(self.get_parameter('debug_raw').get_parameter_value().bool_value)

        # 시리얼 오픈
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Opened {port} @ {baud}')
        except Exception as e:
            self.get_logger().fatal(f'Failed to open serial: {e}')
            raise

        self.pub = self.create_publisher(Imu, topic, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)

    def parse_line(self, line: str):
        """
        line 예시: 'ax,ay,az,gx,gy,gz,qw,qx,qy,qz'
        비숫자 접두/접미 제거 대응.
        """
        parts = [p for p in line.replace('\r', '').split(',') if p != '']
        if len(parts) < NUM_FIELDS:
            return None

        vals = [safe_float(p) for p in parts[:NUM_FIELDS]]
        if any(v is None for v in vals):
            return None

        ax, ay, az, gx, gy, gz, qw, qx, qy, qz = vals
        return ax, ay, az, gx, gy, gz, qw, qx, qy, qz

    def tick(self):
        try:
            raw = self.ser.readline()  # bytes
            if not raw:
                return
            line = raw.decode('utf-8', errors='ignore').strip()
            if not line:
                return
            if self.debug_raw:
                self.get_logger().info(repr(line))

            parsed = self.parse_line(line)
            if not parsed:
                # 주기적 경고 과다 방지: 필요시 debug_raw로만 확인
                return

            ax, ay, az, gx, gy, gz, qw, qx, qy, qz = parsed

            msg = Imu()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            # 가속도/각속도/자세. 단위는 센서가 보낸 그대로 사용.
            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az

            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz

            msg.orientation.w = qw
            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz

            # 공분산은 미지 → -1로 처리하거나 0 유지
            msg.orientation_covariance[0] = -1.0

            self.pub.publish(msg)

        except Exception as e:
            # 치명적 오류만 로그
            self.get_logger().error(f'tick error: {e}')

def main():
    rclpy.init()
    node = ImuPublisher()
    try:
        rclpy.spin(node)
    finally:
        if hasattr(node, 'ser') and node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
