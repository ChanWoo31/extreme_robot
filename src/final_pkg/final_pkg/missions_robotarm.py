# missions_robotarm.py
import numpy as np
import time
from abc import ABC, abstractmethod

class MissionBase(ABC):
    """로봇팔 미션의 베이스 클래스"""

    def __init__(self, arm_controller):
        self.arm = arm_controller  # ArmController 인스턴스 참조

    @abstractmethod
    def execute(self, x, y, z):
        """미션 실행 메인 로직

        Args:
            x, y, z: 카메라 좌표계에서의 타겟 위치

        Returns:
            bool: 미션 성공 여부
        """
        pass

    def log_info(self, msg):
        """로그 출력 헬퍼"""
        self.arm.get_logger().info(f"[{self.__class__.__name__}] {msg}")

    def log_error(self, msg):
        """에러 로그 출력 헬퍼"""
        self.arm.get_logger().error(f"[{self.__class__.__name__}] {msg}")

class Mission1ButtonPress(MissionBase):
    """미션 1: 버튼 누르기"""

    def execute(self, x, y, z):
        try:
            self.log_info(f"버튼 누르기 미션 시작 - 픽셀 좌표: ({x}, {y}, {z})")

            # 버튼 누르기 시퀀스 실행 (LED 버튼 시퀀스 사용)
            success = self.arm.led_button_press_sequence()

            if success:
                self.log_info("버튼 누르기 미션 성공")
            else:
                self.log_error("버튼 누르기 미션 실패")

            return success

        except Exception as e:
            self.log_error(f"버튼 누르기 미션 중 오류 발생: {e}")
            return False

class Mission2HandleTurn(MissionBase):
    """미션 2: 핸들 돌리기"""

    def execute(self, x, y, z):
        try:
            self.log_info(f"핸들 돌리기 미션 시작 - 픽셀 좌표: ({x}, {y}, {z})")

            # 핸들 돌리기 시퀀스 실행 (현재 미구현)
            self.log_info("핸들 돌리기 시퀀스는 아직 구현되지 않았습니다.")
            success = False

            if success:
                self.log_info("핸들 돌리기 미션 성공")
            else:
                self.log_error("핸들 돌리기 미션 실패")

            return success

        except Exception as e:
            self.log_error(f"핸들 돌리기 미션 중 오류 발생: {e}")
            return False

    def _handle_turn_sequence(self, base_coord):
        """핸들 돌리기 시퀀스 (임시 구현)"""
        # 1. 핸들에 접근
        if not self.arm.approach(base_coord, dz=0.05):
            return False

        # 2. 그리퍼로 잡기
        self.arm.gripper(True)
        time.sleep(0.5)

        # 3. 회전 동작 (간단한 원호 궤적)
        center = base_coord
        for angle in np.linspace(0, np.pi/3, 10):  # 60도 회전
            target = center + np.array([0.05 * np.cos(angle), 0.05 * np.sin(angle), 0])
            if not self.arm.move_ik(*target):
                return False
            time.sleep(0.1)

        # 4. 그리퍼 해제 및 후퇴
        self.arm.gripper(False)
        return self.arm.retreat(base_coord, dz=0.05)

class Mission3BoxGrip(MissionBase):
    """미션 3: 박스 그리퍼로 잡기"""

    def execute(self, x, y, z):
        try:
            self.log_info(f"박스 그립 미션 시작 - 픽셀 좌표: ({x}, {y}, {z})")

            # 박스 그립 시퀀스 실행 (pick_seq 사용)
            success = self.arm.pick_seq()

            if success:
                self.log_info("박스 그립 미션 성공")
            else:
                self.log_error("박스 그립 미션 실패")

            return success

        except Exception as e:
            self.log_error(f"박스 그립 미션 중 오류 발생: {e}")
            return False

class Mission4PlaceBox(MissionBase):
    """미션 4: 박스 놓기"""

    def execute(self, x, y, z):
        try:
            self.log_info(f"박스 놓기 미션 시작 - 픽셀 좌표: ({x}, {y}, {z})")

            # 박스 놓기 시퀀스 실행 (place_seq 사용)
            success = self.arm.place_seq([x, y, z])

            if success:
                self.log_info("박스 놓기 미션 성공")
            else:
                self.log_error("박스 놓기 미션 실패")

            return success

        except Exception as e:
            self.log_error(f"박스 놓기 미션 중 오류 발생: {e}")
            return False

# 미션 팩토리 함수
def get_mission_handler(mission_type, arm_controller):
    """미션 타입에 따라 적절한 미션 핸들러 반환

    Args:
        mission_type (str): 미션 타입 문자열
        arm_controller: ArmController 인스턴스

    Returns:
        MissionBase: 해당 미션 핸들러 인스턴스 또는 None
    """
    mission_map = {
        'mission_1_box_grip': Mission3BoxGrip,
        'box_grip': Mission3BoxGrip,
        'pick': Mission3BoxGrip,
        'box': Mission3BoxGrip,
        'ebox': Mission3BoxGrip,  # ebox는 박스 그립 미션으로 처리

        'mission_2_button_press': Mission1ButtonPress,
        'button_press': Mission1ButtonPress,
        'button': Mission1ButtonPress,

        'mission_3_handle_turn': Mission2HandleTurn,
        'handle_turn': Mission2HandleTurn,
        'handle': Mission2HandleTurn,

        'mission_4_place_box': Mission4PlaceBox,
        'place_box': Mission4PlaceBox,
        'place': Mission4PlaceBox,
    }

    mission_class = mission_map.get(mission_type.lower())
    if mission_class:
        return mission_class(arm_controller)
    else:
        return None

# 사용 가능한 미션 목록 반환
def get_available_missions():
    """사용 가능한 모든 미션 타입 목록 반환"""
    return [
        'mission_1_box_grip',
        'mission_2_button_press',
        'mission_3_handle_turn',
        'mission_4_place_box'
    ]