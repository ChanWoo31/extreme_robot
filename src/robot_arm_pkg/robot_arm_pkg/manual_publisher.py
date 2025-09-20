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
        self.pub_arm   = self.create_publisher(String, '/arm/cmd/manual', 10)
        self.pub_lift  = self.create_publisher(String, '/lift/cmd/manual', 10)
        self.pub_back  = self.create_publisher(String, '/back_lift/cmd/manual', 10)
        self.pub_grip  = self.create_publisher(String, '/gripper/cmd/manual', 10)

        self.get_logger().info("start key teleop")

    def send(self, pub, data):
        pub.publish(String(data=data))

    def tick(self):
        ch = getch(0.05)
        if not ch:
            return True

        # 팔 이동
        if   ch == 'q': self.send(self.pub_arm, 'front')
        elif ch == 'e': self.send(self.pub_arm, 'back')
        elif ch == 'w': self.send(self.pub_arm, 'up')
        elif ch == 's': self.send(self.pub_arm, 'down')
        elif ch == 'a': self.send(self.pub_arm, 'left')
        elif ch == 'd': self.send(self.pub_arm, 'right')
        elif ch == 'b': self.send(self.pub_arm, 'branch_clear')
        elif ch == '`': self.send(self.pub_arm, 'home')

        # 앞 리프트
        elif ch == 'z': self.send(self.pub_lift, 'up')
        elif ch == 'x': self.send(self.pub_lift, 'center')
        elif ch == 'c': self.send(self.pub_lift, 'down')

        # 뒤 리프트
        elif ch == '1': self.send(self.pub_back, 'up')
        elif ch == '2': self.send(self.pub_back, 'center')
        elif ch == '3': self.send(self.pub_back, 'down')
        elif ch == '4': self.send(self.pub_back, 'down45')

        # 그리퍼
        elif ch == 'h': self.send(self.pub_grip, 'open')
        elif ch == 'g': self.send(self.pub_grip, 'close')
        elif ch == 'j': self.send(self.pub_grip, 'close_fully')

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