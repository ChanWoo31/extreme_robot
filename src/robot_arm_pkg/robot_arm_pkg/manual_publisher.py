#!/usr/bin/env python3
import sys, termios, tty, select, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def getch(timeout=0.1):
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return None

class KeyTeleop(Node):
    def __init__(self):
        super().__init__('key_teleop')
        self.pub_manual = self.create_publisher(String, '/arm/manual', 10)

        self.get_logger().info("start key teleop")

    def send(self, pub, data):
        pub.publish(String(data=data))

    def tick(self):
        ch = getch(0.05)
        if not ch:
            return True

        # 개별 조인트 제어 (틱값 직접 제어)
        if   ch == 'q': self.send(self.pub_manual, 'j1_plus')    # J1 +
        elif ch == 'a': self.send(self.pub_manual, 'j1_minus')   # J1 -
        elif ch == 'w': self.send(self.pub_manual, 'j2_plus')    # J2 +
        elif ch == 's': self.send(self.pub_manual, 'j2_minus')   # J2 -
        elif ch == 'e': self.send(self.pub_manual, 'j3_plus')    # J3 +
        elif ch == 'd': self.send(self.pub_manual, 'j3_minus')   # J3 -
        elif ch == 'r': self.send(self.pub_manual, 'j4_plus')    # J4 +
        elif ch == 'f': self.send(self.pub_manual, 'j4_minus')   # J4 -
        elif ch == 'b': self.send(self.pub_manual, 'branch_clear')
        elif ch == '`': self.send(self.pub_manual, 'home')

        # 앞 리프트
        elif ch == 'z': self.send(self.pub_manual, 'flift_up')
        elif ch == 'x': self.send(self.pub_manual, 'flift_center')
        elif ch == 'c': self.send(self.pub_manual, 'flift_down')

        # 뒤 리프트
        elif ch == '1': self.send(self.pub_manual, 'blift_up')
        elif ch == '2': self.send(self.pub_manual, 'blift_center')
        elif ch == '3': self.send(self.pub_manual, 'blift_down')
        elif ch == '4': self.send(self.pub_manual, 'blift_down45')

        # 그리퍼
        elif ch == 'h': self.send(self.pub_manual, 'open')
        elif ch == 'g': self.send(self.pub_manual, 'close')
        elif ch == 'j': self.send(self.pub_manual, 'close_fully')

        return True

def main():
    # 터미널 원상복구 보장
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        rclpy.init()
        node = KeyTeleop()
        while rclpy.ok() and node.tick():
            rclpy.spin_once(node, timeout_sec=0.0)
        node.destroy_node()
        rclpy.shutdown()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

if __name__ == '__main__':
    main()