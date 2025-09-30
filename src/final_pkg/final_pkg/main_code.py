#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, Bool, String
from final_pkg.auto_drive_code import DirectionController

class ModeController(Node):
    def __init__(self):
        super().__init__('mode_controller')

        # 현재 모드 저장
        self.current_mode = 4

        # 최신 yaw와 장애물 상태 저장
        self.current_yaw = 0.0
        self.obstacle_detected = False
        self.camera_detected = ' '
        self.color_direction = None
        self.selected_color = None
        self.from_center_x = 0.0
        self.from_center_y = 0.0

        self.direction_ctrl = DirectionController()
        
        self.init_1_flag = True
        self.init_2_flag = True
        self.init_3_flag = True
        self.init_4_flag = True
        
        self.auto_drive_flag_mode1 = True
        self.auto_drive_flag_mode2 = True
        self.auto_drive_flag_mode3 = True
        self.auto_drive_flag_mode4 = True

        self.fire_search_flag = True
        self.Go_to_stairs_flag = True

        self.Press_button_flag = False
        self.box_mode_executed = False
        self.sub_mode = self.create_subscription(Int32, '/current_mode', self.mode_callback, 10)

        # ------------------ Subscriber ------------------
        self.sub_yaw = self.create_subscription(Float32, '/yaw_value', self.yaw_callback, 10)
        self.sub_obstacle = self.create_subscription(Bool, '/obstacle_detected', self.obstacle_callback, 10)
        self.sub_obstacle_name = self.create_subscription(String, '/detected_object_names', self.camera_name_callback, 10)
        self.sub_from_center_x = self.create_subscription(Float32, '/from_center_x', self.from_center_x_callback, 10)
        self.sub_from_center_y = self.create_subscription(Float32, '/from_center_y', self.from_center_y_callback, 10)
        self.sub_color_direction = self.create_subscription(String, '/direction', self.camera_color_direction_callback, 10)

        # 추가: 색상 선택 구독
        self.sub_selected_color = self.create_subscription(String, '/selected_color', self.selected_color_callback, 10)

        # ------------------ Publisher ------------------
        self.pub_target_color = self.create_publisher(String, '/target_color', 10)
        
        # ------------------ Timer ------------------
        self.timer = self.create_timer(0.1, self.execute_mode)
        self.get_logger().info("ModeController Node Started")

    # ------------------ 콜백 ------------------
    def mode_callback(self, msg):
        self.current_mode = msg.data
        self.get_logger().info(f"Mode updated: {self.current_mode}")

    def yaw_callback(self, msg: Float32):
        self.current_yaw = msg.data

    def obstacle_callback(self, msg: Bool):
        self.obstacle_detected = msg.data
        
    def camera_name_callback(self, msg: String):
        self.camera_detected = msg.data
        
    def from_center_x_callback(self, msg: Float32):
        self.from_center_x = msg.data

    def from_center_y_callback(self, msg: Float32):
        self.from_center_y = msg.data
        
    def camera_color_direction_callback(self, msg: String):
        self.color_direction = msg.data

    def selected_color_callback(self, msg: String):
        self.selected_color = msg.data
        self.get_logger().info(f"Selected color received: {self.selected_color}")

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
        if self.init_1_flag == True:
            self.direction_ctrl.ROBOT_ARM_HOME()
            self.init_1_flag = False
        
        self.direction_ctrl.direction_control(self.current_yaw)
        # self.direction_ctrl.GO_SHOOT()
        if self.obstacle_detected:
            self.direction_ctrl.obstacle_mode()
            self.obstacle_detected = False
            
    def mode_2_action(self):
        self.get_logger().info(f"Mode 2: yaw={self.current_yaw:.2f}, obstacle={self.obstacle_detected}")
        
        if self.init_2_flag == True:
            self.direction_ctrl.ROBOT_ARM_HOME()
            self.init_2_flag = False
        
        if self.Go_to_stairs_flag:
            self.direction_ctrl.Go_and_Right()
            self.Go_to_stairs_flag = False
        
        if self.direction_ctrl.Stair_end_detect():
            self.auto_drive_flag_mode2 = True
            print("Stair END!")
          
        if self.auto_drive_flag_mode2 == False:
            self.direction_ctrl.direction_control(self.current_yaw)
        else:
            self.direction_ctrl.Go_Straight()
        
        if self.obstacle_detected:
            if self.camera_detected == 'branch':
                self.direction_ctrl.Go()
            else: 
                self.direction_ctrl.obstacle_mode() 
            
    def mode_3_action(self):
        self.get_logger().info(f"Mode 3: yaw={self.current_yaw:.2f}, obstacle={self.obstacle_detected}")
        
        if self.init_3_flag == True:
            self.direction_ctrl.ROBOT_ARM_HOME_MODE3()
            self.init_3_flag = False
        
        if self.fire_search_flag:
            self.direction_ctrl.fire_search_mode()
            self.fire_search_flag = False
            
        if self.auto_drive_flag_mode3:
            self.direction_ctrl.direction_control(self.current_yaw)

        # self.selected_color = 'blue'
        if self.selected_color:
            msg = String()
            msg.data = self.selected_color
            self.pub_target_color.publish(msg)
            self.get_logger().info(f"Published target color: {self.selected_color}")
            # 한 번 보낸 후 초기화 (원하면 유지 가능)
            self.selected_color = None
        
        if self.color_direction and self.Press_button_flag == False:
            self.direction_ctrl.Press_LED_mode()
            self.Press_button_flag = True
            
        if self.camera_detected == 'door':
            self.auto_drive_flag_mode3 = False
            self.direction_ctrl.Go_to_door()
            self.direction_ctrl.Press_door()

        else:
            self.auto_drive_flag_mode3 = True
        
    def mode_4_action(self):
        self.get_logger().info(f"Mode 4: camera={self.camera_detected}, box_executed={self.box_mode_executed}")
        if self.init_4_flag == True:
            # self.direction_ctrl.ROBOT_ARM_HOME_MODE4()
            self.init_4_flag = False

        # box_mode가 이미 실행되었으면 더 이상 실행하지 않음
        if self.box_mode_executed:
            if self.auto_drive_flag_mode4:
                self.direction_ctrl.direction_control(self.current_yaw)
            return  # box_mode 이미 완료, 더 이상 체크 안 함

        # box_mode가 아직 실행 안 됐고, ebox 감지되면 실행
        if self.camera_detected == 'ebox' and not self.box_mode_executed:
            self.get_logger().warn("Starting box_mode sequence...")
            self.auto_drive_flag_mode4 = False
            self.direction_ctrl.Go_to_Box()
            self.direction_ctrl.box_mode()
            self.box_mode_executed = True  # 플래그 설정
            self.get_logger().warn("Box_mode completed! Will never execute again.")
            self.auto_drive_flag_mode4 = True
            return

        if self.auto_drive_flag_mode4:
            self.direction_ctrl.direction_control(self.current_yaw)

        
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