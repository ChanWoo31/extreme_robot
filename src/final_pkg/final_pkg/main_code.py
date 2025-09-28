#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, Bool, String
import json

from final_pkg.auto_drive_code import DirectionController

class ModeController(Node):
    def __init__(self):
        super().__init__('mode_controller')

        # 현재 모드 저장
        self.current_mode = 2

        # 최신 yaw와 장애물 상태 저장
        self.current_yaw = 0.0
        self.obstacle_detected = False
        self.camera_detected = ' '
        
        self.direction_ctrl = DirectionController()
        
        self.auto_drive_flag_mode1 = True
        self.auto_drive_flag_mode2 = True
        self.auto_drive_flag_mode3 = True
        self.auto_drive_flag_mode4 = True
        
        self.fire_search_flag = True
        
        self.Go_to_stairs_flag = True
        
        # /current_mode 구독
        self.sub_mode = self.create_subscription(
            Int32,
            '/current_mode',
            self.mode_callback,
            10
        )

        # /yaw_value 구독
        self.sub_yaw = self.create_subscription(
            Float32,
            '/yaw_value',
            self.yaw_callback,
            10
        )

        # obstacle_detected 구독
        self.sub_obstacle = self.create_subscription(
            Bool,
            'obstacle_detected',
            self.obstacle_callback,
            10
        )
        self.sub_obstacle_name = self.create_subscription(
            String,
            '/detected_object_names',
            self.camera_name_callback,
            10
        )
        self.sub_obstacle_name = self.create_subscription(
            String,
            '/detected_objects',
            self.camera_data_callback,
            10
        )
        self.sub_color_direction = self.create_subscription(
            String,
            '/direction',
            self.camera_color_direction_callback,
            10
        )
        
        # 0.1초마다 현재 모드에 따른 동작 수행
        self.timer = self.create_timer(0.1, self.execute_mode)
        self.get_logger().info("ModeController Node Started")

    # ------------------ 콜백 ------------------
    def mode_callback(self, msg):
        self.current_mode = msg.data
        self.get_logger().info(f"Mode updated: {self.current_mode}")

    def yaw_callback(self, msg: Float32):
        self.current_yaw = msg.data
        # self.get_logger().info(f"Yaw updated: {self.current_yaw:.2f}")

    def obstacle_callback(self, msg: Bool):
        self.obstacle_detected = msg.data
        # self.get_logger().info(f"Obstacle detected: {self.obstacle_detected}")
        
    def camera_name_callback(self, msg: String):
        self.camera_detected = msg.data
        # self.get_logger().info(f"Obstacle detected: {self.obstacle_detected}")
        
    def camera_data_callback(self, msg: String):
        self.camera_data = json.loads(msg.data)
        self.from_center_x = float(self.camera_data["from_center_x"])
        self.from_center_y = float(self.camera_data["from_center_y"])
        # self.get_logger().info(f"Obstacle detected: {self.obstacle_detected}")
        
    def camera_color_direction_callback(self, msg: String):
        self.color_direction = msg.data
        # self.get_logger().info(f"Obstacle detected: {self.obstacle_detected}")

    # ------------------ 모드 실행 ------------------
    def execute_mode(self):
        if self.current_mode == 1:
            self.mode_1_action()
        elif self.current_mode == 2:
            self.mode_2_action()
        elif self.current_mode == 3:
            self.mode_3_action()
        elif self.current_mode == 4:
            self.mode_4_action()
        else:
            self.get_logger().info("Unknown mode")

    # ------------------ 모드별 동작 ------------------
    def mode_1_action(self):
        self.get_logger().info(f"Mode 1: yaw={self.current_yaw:.2f}, obstacle={self.obstacle_detected}")
        
        self.direction_ctrl.direction_control(self.current_yaw)
        
        if self.obstacle_detected == True:
            self.direction_ctrl.obstacle_mode()
            self.obstacle_detected = False
            
        
    def mode_2_action(self):
        self.get_logger().info(f"Mode 2: yaw={self.current_yaw:.2f}, obstacle={self.obstacle_detected}")
        # self.direction_ctrl.turn_right_mode()
        if self.Go_to_stairs_flag == True:
            self.direction_ctrl.Go_and_Right()
            self.Go_to_stairs_flag = False
        
        
        if self.direction_ctrl.Stair_end_detect() == True:
            self.auto_drive_flag_mode2 = True
            print("Stair END!")
          
        
        if self.auto_drive_flag_mode2 == False:
            self.direction_ctrl.direction_control(self.current_yaw)
            
        else:
            self.direction_ctrl.Go_Straight()
        
        if self.obstacle_detected:
           self.direction_ctrl.obstacle_mode() 
            
            

        # if self.obstacle_detected:
        #    if self.camera_detected == 'stairs':
        #       self.direction_ctrl.obstacle_mode()
        #    elif self.camera_detected == 'Tree':
        #       self.direction_ctrl.branch_mode()
        # else:
        #     self.direction_ctrl.obstacle_mode()


    def mode_3_action(self):
        self.get_logger().info(f"Mode 3: yaw={self.current_yaw:.2f}, obstacle={self.obstacle_detected}")
        
        # if self.fire_search_flag == True:
        #     self.direction_ctrl.fire_search_mode()
        #     self.fire_search_flag = False
            
        #     if self.auto_drive_flag_mode3 == True:
        #         self.direction_ctrl.direction_control(self.current_yaw)
        
        # if self.color_direction == 'None':
        #     self.direction_ctrl.Press_LED_mode()
            
        # if self.camera_detected == 'door':
        #       self.direction_ctrl.Go_to_door(self.pixel_x,self.pixel_y)
        

    def mode_4_action(self):
        self.get_logger().info(f"Mode 4: yaw={self.current_yaw:.2f}, obstacle={self.obstacle_detected}")
        
        # if self.camera_detected == 'ebox':
        self.direction_ctrl.Go_to_Box()


def main(args=None):
    rclpy.init(args=args)
    node = ModeController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
