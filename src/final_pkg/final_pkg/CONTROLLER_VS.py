import serial
import time
from pynput import keyboard
from pynput.keyboard import Key
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 아두이노 연결
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)  # 연결 안정화 대기

current_cmd = "Stop"

# ROS2 초기화
rclpy.init()
node = Node("keyboard_controller")
pub = node.create_publisher(String, '/direction_cmd', 10)

def send_command(cmd, force=False):
    global current_cmd
    # force=True면 같은 명령도 무조건 전송
    if force or cmd != current_cmd:
        # 아두이노 전송
        arduino.write((cmd + '\n').encode())
        print("Send:", cmd)
        current_cmd = cmd

        # ROS2 퍼블리시
        msg = String()
        msg.data = cmd
        pub.publish(msg)

def on_press(key):
    try:
        # 이동 키 (연속 입력 방지)
        if key == Key.up:         
            send_command("Go")
        elif key == Key.down:     
            send_command("Back")
        elif key == Key.left:     
            send_command("Left")
        elif key == Key.right:    
            send_command("Right")

        # 일반 문자 키 (연속 입력 허용)
        elif key.char == '0': send_command("LED_ON", force=True)
        elif key.char == '-': send_command("LED_OFF", force=True)
        elif key.char == '8': send_command("ROVER", force=True)
        elif key.char == '9': send_command("NORMAL", force=True)
        elif key.char == 'esc':   # 종료
            print("프로그램 종료")
            return False

        # Arm
        elif key.char == 'a': send_command("j1_plus", force=True)
        elif key.char == 'q': send_command("j1_minus", force=True)               
        elif key.char == 's': send_command("j2_plus", force=True)
        elif key.char == 'w': send_command("j2_minus", force=True)
        elif key.char == 'e': send_command("j3_plus", force=True)
        elif key.char == 'd': send_command("j3_minus", force=True)
        elif key.char == 'r': send_command("j4_plus", force=True)
        elif key.char == 'f': send_command("j4_minus", force=True)
        elif key.char == 'z': send_command("base_home", force=True)
        elif key.char == '`': send_command("home", force=True)

        # Lift
        elif key.char == 't': send_command("flift_up", force=True)
        elif key.char == 'y': send_command("flift_center", force=True)
        elif key.char == 'u': send_command("flift_down", force=True)

        # Back Lift
        elif key.char == 'v': send_command("blift_up", force=True)
        elif key.char == 'b': send_command("blift_center", force=True)
        elif key.char == 'n': send_command("blift_down", force=True)
        elif key.char == 'm': send_command("blift_down45", force=True)

        # Gripper
        elif key.char == 'g': send_command("close", force=True)
        elif key.char == 'h': send_command("open", force=True)
        elif key.char == 'j': send_command("close_fully", force=True)
        
        elif key.char == '1': send_command("Mode1", force=True)
        elif key.char == '2': send_command("Mode2", force=True)
        elif key.char == '3': 
            
            # 허용된 색깔 목록
            valid_colors = {"blue", "green", "yellow"}
    
            while True:
                color = input("Enter color (blue / green / yellow): ").strip().lower()
                if color in valid_colors:
                    send_command(color, force=True)
                    send_command("Mode3", force=True)
                    break
                else:
                    print("잘못 입력했습니다. blue, green, yellow 중에서 입력하세요.")

        elif key.char == '4': send_command("Mode4", force=True)
        elif key.char == '5': send_command("Tele_mode", force=True)

    except AttributeError:
        pass

def on_release(key):
    if key in [Key.up, Key.down, Key.left, Key.right]:
        send_command("Stop")

try:
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()
finally:
    node.destroy_node()
    rclpy.shutdown()
