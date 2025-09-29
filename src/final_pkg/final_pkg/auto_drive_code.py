#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import time
import math
import json
from std_srvs.srv import Trigger

from arm_interfaces.srv import ArmDoTask
from arm_interfaces.msg import DetectedObject


class DirectionController(Node):
    def __init__(self, pub_direction_topic='/direction_cmd'):
        super().__init__('direction_controller')

        # super().__init__('arm_client')
        # self.cli = self.create_client(ArmDoTask, 'arm_do_task')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service not available, waiting...')
        # self.req = ArmDoTask.Request()
        
        # 퍼블리셔
        self.pub_direction = self.create_publisher(String, pub_direction_topic, 10)
        self.subscription = self.create_subscription(Float32MultiArray, 'ebimu_rpy', self.rpy_callback, 10)
        self.sub_color_direction = self.create_subscription(
            String,
            '/detected_objects',
            self.camera_data_callback,
            10
        )
        
        self.arm_service_shoot = self.create_client(ArmDoTask, '/ArmDoTask')
        while not self.arm_service_shoot.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Arm service not available, waiting...')
        self.get_logger().info('Arm service connected!')

        # 클래스 변수
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0
        self.IMU_Flag = True
        self.get_logger().info("DirectionController Node Started")
        self.from_center_x = None
        self.Straight_flag = True
        self.Straight_end_flag = True
        self.Starget_roll = 0
        self.Stair_count = 0
        self.Stair_end_flag = False

    def arm_service(self, task):
        request = ArmDoTask.Request()
        request.task = task
        
        future = self.arm_service_shoot.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        response = future.result()
        if response.success:
            self.get_logger().info("앙 되노")
        else:   
            self.get_logger().info("앙 안되노")
    # ------------------- 명령 발행 -------------------
    def send_command(self, command: str):
        msg = String()
        msg.data = command
        self.pub_direction.publish(msg)
        self.get_logger().info(f"Published Command: {command}")

    # ------------------- RPY 콜백 -------------------
    def rpy_callback(self, msg):
        self.roll, self.pitch, self.yaw = msg.data
        # 0~360 범위로 변환
        self.roll = self.roll % 360
        self.pitch = self.pitch % 360
        self.yaw = self.yaw 
    
    def camera_data_callback(self, msg: String):
        self.camera_data = json.loads(msg.data)
        self.from_center_x = float(self.camera_data["from_center_x"])
        self.from_center_y = float(self.camera_data["from_center_y"])

    # ------------------- 유틸 함수 -------------------
    def angle_diff(self, target, current):
        # ±180 범위로 차이 계산 (항상 최단 경로)
        return (target - current + 180) % 360 - 180

    # ------------------- 모드 함수 -------------------
    def obstacle_mode(self):
        self.send_command('Stop')
        time.sleep(1)
        self.send_command('ROVER')
        time.sleep(0.5)
        self.send_command('LED_ON')
        time.sleep(1)
        self.send_command('blift_center')
        time.sleep(4)
        self.send_command('LED_OFF')
        time.sleep(1)
        self.send_command('flift_up')
        time.sleep(3)
        self.send_command('Go')
        time.sleep(3)
        self.send_command('Stop')
        time.sleep(0.5)
        self.send_command('flift_down')
        time.sleep(3)
        self.send_command('blift_down')
        time.sleep(4)
        self.send_command('Go')
        time.sleep(2)
        self.send_command('Stop')
        time.sleep(0.5)
        self.send_command('flift_center')
        self.send_command('blift_center')
        time.sleep(3)
        self.send_command('Go')
        time.sleep(2)
        self.send_command('Stop')
        time.sleep(1)

    def Go_and_Right(self):
        self.send_command('Go')
        time.sleep(4)
        self.send_command('Stop')
        time.sleep(1)
        self.turn_right_mode()
        
    def Go(self):
        self.send_command('Go')
        time.sleep(8)
        
    def ROBOT_ARM_HOME(self):
        self.send_command('go_home')
    
    def ROBOT_ARM_HOME_MODE3(self):
        self.send_command('mission_3_basic_state')
        
    def Go_Straight(self):    
        if self.IMU_Flag == True and self.roll == 0:
            for _ in range(3):
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.05)
                self.IMU_Flag = False
                
        rclpy.spin_once(self, timeout_sec=0.01)
        if self.Straight_flag == True:
            self.Starget_roll = self.roll
            self.Straight_flag = False
        
        
        current_roll = self.roll
        diff = self.angle_diff(self.Starget_roll, current_roll)
        print(f"Target Roll: {self.Starget_roll:.2f}, Current Roll: {current_roll:.2f}, Diff: {diff:.2f}")

        # 목표 각도 근처 도달
        if -5 <= diff <= 5:
            self.send_command('Go_auto')
            print("X in range, go straight.")
                
        elif diff < -40:
            self.send_command('Left_auto')
            print("X < -10, turn left fast.")
                
        elif diff > 40:
            self.send_command('Right_auto')
            print("X > 10, turn right fast.")
                
        elif diff < -5:
            self.send_command('Left_obj')
            print("-10 <= X < -5, turn left slow.")
            time.sleep(0.5)
                
        elif diff > 5:
            self.send_command('Right_obj')
            print("5 < X <= 10, turn right slow.")
            time.sleep(0.5)

    def Stair_end_detect(self):
        print(self.yaw,"yaw value")
        if self.yaw < -10:
            self.Straight_end_flag = False
        
        if self.yaw >= 0 and self.Straight_end_flag == False:
            self.Straight_end_flag = True
            self.Stair_count += 1
            
        if self.Stair_count > 5:
           self.Stair_end_flag == True     
        
        return self.Stair_end_flag
        
    def Go_to_Stairs(self):
        start_time = time.time()  # 시작 시간 기록
        print(self.from_center_x)
        X = None
        while True:
                # 경과 시간 체크
            elapsed = time.time() - start_time
            if elapsed > 30:  # 10초 넘으면 강제 종료
                print("Timeout! Exit loop.")
                break
            
            if self.from_center_x is None:
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.05)
                continue
            rclpy.spin_once(self, timeout_sec=0.01)
            X = self.from_center_x
            
            if -5 <= X <= 5:
                self.send_command('Go_auto')
                time.sleep(1)
                break
                print("X in range, go straight.")
            elif X < -40:
                self.send_command('Left_auto')
                print("X < -10, turn left fast.")
            elif X > 40:
                self.send_command('Right_auto')
                print("X > 10, turn right fast.")
            elif X < -5:
                self.send_command('Left_obj')
                print("-10 <= X < -5, turn left slow.")
                time.sleep(0.5)
            elif X > 5:
                self.send_command('Right_obj')
                print("5 < X <= 10, turn right slow.")
                time.sleep(0.5)
 
                
        self.send_command('LED_ON')  
        self.send_command('Stop')
        time.sleep(2)
        self.send_command('LED_OFF')
        
    def Go_to_Box(self):
        start_time = time.time()  # 시작 시간 기록
        print(self.from_center_x)
        X = None
        while True:
                # 경과 시간 체크
            elapsed = time.time() - start_time
            if elapsed > 10:  # 10초 넘으면 강제 종료
                print("Timeout! Exit loop.")
                break
            
            if self.from_center_x is None:
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.05)
                continue
            rclpy.spin_once(self, timeout_sec=0.01)
            X = self.from_center_x
            
            if -5 <= X <= 5:
                self.send_command('Go_auto')
                time.sleep(1)
                print("X in range, go straight.")
                break
                
            elif X < -40:
                self.send_command('Left_auto')
                print("X < -10, turn left fast.")
            elif X > 40:
                self.send_command('Right_auto')
                print("X > 10, turn right fast.")
            elif X < -5:
                self.send_command('Left_obj')
                print("-10 <= X < -5, turn left slow.")
                time.sleep(0.5)
            elif X > 5:
                self.send_command('Right_obj')
                print("5 < X <= 10, turn right slow.")
                time.sleep(0.5)
 
                
        self.send_command('LED_ON')  
        self.send_command('Stop')
        time.sleep(2)
        self.send_command('LED_OFF')
    
            
    def Go_to_door(self):
        start_time = time.time()  # 시작 시간 기록
        print(self.from_center_x)
        X = None
        while True:
                # 경과 시간 체크
            elapsed = time.time() - start_time
            if elapsed > 20:  # 10초 넘으면 강제 종료
                print("Timeout! Exit loop.")
                break
            
            if self.from_center_x is None:
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.05)
                continue
            rclpy.spin_once(self, timeout_sec=0.01)
            X = self.from_center_x
            
            if -5 <= X <= 5:
                self.send_command('Go_auto')
                time.sleep(1)
                print("X in range, go straight.")
                break
                
            elif X < -40:
                self.send_command('Left_auto')
                print("X < -10, turn left fast.")
            elif X > 40:
                self.send_command('Right_auto')
                print("X > 10, turn right fast.")
            elif X < -5:
                self.send_command('Left_obj')
                print("-10 <= X < -5, turn left slow.")
                time.sleep(0.5)
            elif X > 5:
                self.send_command('Right_obj')
                print("5 < X <= 10, turn right slow.")
                time.sleep(0.5)
 
        
        self.send_command('LED_ON')  
        self.send_command('Go')
        time.sleep(1)
        self.send_command('Stop')
        time.sleep(1)
        self.send_command('LED_OFF')
        
    def Press_door(self):
        self.send_command('Stop')
        time.sleep(1)
        self.arm_service('open_door')
        self.send_command('Go')
        time.sleep(5)
        self.send_command('Stop')
        time.sleep(1)
    
    def box_mode(self):
        self.send_command('Stop')
        time.sleep(1)
        self.arm_service('pick')
        self.send_command('Stop')
        time.sleep(1)

    def branch_mode(self):
        self.send_command('Go')
        time.sleep(10)
        self.send_command('Stop')
        time.sleep(1)

    # def brick_mode(self):
    #     self.send_command('Go')
    #     time.sleep(10)
    #     self.send_command('Stop')
    #     time.sleep(1)

    def Press_LED_mode(self):
        self.send_command('Stop')
        time.sleep(1)
        self.arm_service('press_button')
        
    def fire_search_mode(self):
        self.send_command('LED_ON')
        self.send_command('Go')
        time.sleep(3)
        self.send_command('Stop')
        time.sleep(1)
        self.send_command('see_around')
        time.sleep(12)
        self.send_command('LED_OFF')

    # ------------------- 개선된 turn_right_mode -------------------
    def turn_right_mode(self):
        self.send_command('LED_ON')

        if self.IMU_Flag == True and self.roll == 0:
            for _ in range(3):
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.05)
                self.IMU_Flag = False
            
        start_roll = self.roll
        if start_roll > 270:
            target_roll = start_roll - 360 + 90
        else:
            target_roll = start_roll
        
        start_time = time.time()
        while True:
            current_roll = self.roll
            diff = self.angle_diff(target_roll, current_roll)
            print(f"Target Roll: {target_roll:.2f}, Current Roll: {current_roll:.2f}, Diff: {diff:.2f}")

            # 목표 각도 근처 도달
            if abs(diff) <= 5:
                print("Rotation complete!")
                break

            self.send_command('Right_auto')

            # ROS 콜백 갱신
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)

            # 타임아웃 (15초)
            if time.time() - start_time > 10:
                print("Timeout: Exiting turn loop")
                break

        self.send_command('Stop')
        time.sleep(1)
        self.send_command('LED_OFF')

    # ------------------- heading 기반 제어 -------------------
    def direction_control(self, yaw):
        if yaw is None:
            return

        heading_deg = yaw
        if -5 <= heading_deg <= 5:
            self.send_command('Go')
        elif heading_deg < -5:
            self.send_command('Right_auto')
        elif heading_deg > 5:
            self.send_command('Left_auto')
        else:
            self.send_command('Go_auto')


# ------------------- 실행 -------------------
def main(args=None):
    rclpy.init(args=args)
    node = DirectionController()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
