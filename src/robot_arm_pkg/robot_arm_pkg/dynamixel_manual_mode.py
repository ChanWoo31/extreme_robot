#groupsync version -> 매뉴얼 (기구학 제거)
import numpy as np
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite

RAD2TICKS = 4096 / (2*np.pi)
AX_TICKS_PER_DEG = 1023.0 / 300.0
 
class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        # self.sub = self.create_subscription(
        #     Point, 'target_point', self.on_point, 10
        # )
        
        self.DEVICENAME = '/dev/ttyROBOTARM'
        self.BAUDRATE = 1000000
        self.ax_base_id = 1   # 베이스 J1
        self.ax_grip_id = 5   # 그리퍼
        self.ids_x = [2, 3, 4]  # J2, J3, J4

        # Control table(protocol 1.0)
        self.AX_ADDR_TORQUE_ENABLE = 24
        self.AX_ADDR_GOAL_POSITION = 30
        self.AX_ADDR_MOVING_SPEED = 32
        self.AX_ADDR_PRESENT_POS = 36

        # Control table(protocol 2.0)
        self.ADDR_OPERATING_MODE = 11
        self.ADDR_CURRENT_LIMIT = 38
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_PROFILE_ACCEL = 108
        self.ADDR_PROFILE_VELOCITY = 112
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132
        
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0

        # ---- 추가 파라미터 ----
        # self.ax_lift_id  = self.declare_parameter('ax_lift_id', 6).get_parameter_value().integer_value
        self.x_lift_id = self.declare_parameter('x_lift_id', 6).get_parameter_value().integer_value
        self.x_lift_up_tick = self.declare_parameter('x_lift_up_tick', 5461).get_parameter_value().integer_value
        self.x_lift_center_tick = self.declare_parameter('x_lift_center_tick',2048).get_parameter_value().integer_value
        self.x_lift_down_tick = self.declare_parameter('x_lift_down_tick',-1365).get_parameter_value().integer_value

        self.xl_back_id  = self.declare_parameter('xl_back_id', 7).get_parameter_value().integer_value
        self.back_up_tick    = self.declare_parameter('back_up_tick',    -5120).get_parameter_value().integer_value
        self.back_center_tick= self.declare_parameter('back_center_tick',0).get_parameter_value().integer_value
        self.back_down_tick  = self.declare_parameter('back_down_tick',  5120).get_parameter_value().integer_value
        self.back_down_45_tick  = self.declare_parameter('back_down_45_tick',  2560).get_parameter_value().integer_value

        self.grip_open_tick  = self.declare_parameter('grip_open_tick', 480).get_parameter_value().integer_value
        self.grip_close_tick = self.declare_parameter('grip_close_tick', 330).get_parameter_value().integer_value
        self.grip_close_fully_tick = self.declare_parameter('grip_close_fully_tick', 0).get_parameter_value().integer_value

        self.base_step_deg = self.declare_parameter('base_step_deg', 5.0).get_parameter_value().double_value

        # ---- 통합 구독 ----
        self.sub_manual = self.create_subscription(String, '/arm/manual', self.on_manual_command, 50)

        # 포트
        self.port = PortHandler(self.DEVICENAME)
        
        if not self.port.openPort():
            self.get_logger().error('Failed to open port')
            return
        if not self.port.setBaudRate(self.BAUDRATE):
            self.get_logger().error('Failed to set baudrate')
            return
        
        # 프로토콜 핸들러
        self.packet_ax = PacketHandler(1.0)
        self.packet_x = PacketHandler(2.0)

        # AX-12A 초기 설정
        for ax_id in [i for i in [self.ax_base_id, self.ax_grip_id] if i is not None]:
            self.packet_ax.write1ByteTxRx(self.port, ax_id, self.AX_ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            self.packet_ax.write2ByteTxRx(self.port, ax_id, self.AX_ADDR_MOVING_SPEED, 100)
            self.packet_ax.write1ByteTxRx(self.port, ax_id, self.AX_ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        
        # XM lift arm, XL backlift init: Position mode(3)
        for x_id in [i for i in [self.x_lift_id, self.xl_back_id] if i is not None]:
            self.packet_x.write1ByteTxRx(self.port, x_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            self.packet_x.write1ByteTxRx(self.port, x_id, self.ADDR_OPERATING_MODE, 4)
            self.packet_x.write4ByteTxRx(self.port, x_id, self.ADDR_PROFILE_ACCEL, 20)
            self.packet_x.write4ByteTxRx(self.port, x_id, self.ADDR_PROFILE_VELOCITY, 100)
            self.packet_x.write1ByteTxRx(self.port, x_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)

        # 팔 X-series 초기 설정
        for dxl_id in self.ids_x:
            
            # Extended 포지션 모드 4번
            self.packet_x.write1ByteTxRx(self.port, dxl_id, self.ADDR_OPERATING_MODE, 4)
            # current limit
            self.packet_x.write2ByteTxRx(self.port, dxl_id, self.ADDR_CURRENT_LIMIT, 500)
            # 가속/속도
            self.packet_x.write4ByteTxRx(self.port, dxl_id, self.ADDR_PROFILE_ACCEL, 10)
            self.packet_x.write4ByteTxRx(self.port, dxl_id, self.ADDR_PROFILE_VELOCITY, 100)
            # 필요시 Homing Offset (엔코더 0점을 기구학적 0으로 보전)
            # self.packet.write4ByteTxRx(self.port, self.ids, self.ADDR_HOMING_OFFSET, offset_ticks)

            # 토크 온
            self.packet_x.write1ByteTxRx(self.port, dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)

        self.sync_write = GroupSyncWrite(
            self.port, self.packet_x,
            self.ADDR_GOAL_POSITION, 4
        )
        self.sync_speed = GroupSyncWrite(self.port, self.packet_x, self.ADDR_PROFILE_VELOCITY, 4)

        self.get_logger().info('ArmController initialized')

        # 기구/맵핑 (틱값 직접 제어용)
        self.gear = [1.0, 5.0, 5.0, 5.0]
        self.dir = [1, 1, 1, 1]

        # 조인트 리미트 (라디안)
        self.limits=[(-np.deg2rad(150), np.deg2rad(150)),
                     (-np.deg2rad(170), np.deg2rad(5)),
                     (-np.deg2rad(5), np.deg2rad(90)),
                     (-np.deg2rad(5), np.deg2rad(100))]

        # 홈 위치로 이동한 후 zero 설정
        self.init_home_positions()
        
    def int32_to_le(self, v:int):
        v &= 0xFFFFFFFF
        return bytes([v & 0xFF, (v>>8)&0xFF, (v>>16)&0xFF, (v>>24)&0xFF])

    def joint_rad_to_motor_ticks(self, i, q_joint):
        q_motor = (q_joint * self.dir[i]) * self.gear[i]
        ticks = int(round(q_motor * RAD2TICKS)) + self.zero[i]
        return max(-2_147_483_648, min(2_147_483_647, ticks))

    def send_x_positions(self, q2, q3, q4):
        ticks = [self.joint_rad_to_motor_ticks(1, q2), self.joint_rad_to_motor_ticks(2, q3), self.joint_rad_to_motor_ticks(3, q4)]
        self.sync_write.clearParam()
        for dxl_id, t in zip(self.ids_x, ticks):
            self.sync_write.addParam(dxl_id, self.int32_to_le(t))
        self.sync_write.txPacket()

    def send_ax_base_deg(self, deg):
        # 각도 리미트 확인
        deg_rad = np.deg2rad(deg)
        lo, hi = self.limits[0]
        if deg_rad < lo or deg_rad > hi:
            self.get_logger().warn(f"Base angle {deg:.1f}° blocked by limit (limit: {np.rad2deg(lo):.1f}° ~ {np.rad2deg(hi):.1f}°)")
            return False

        ax_ticks = int(round((deg / 300.0) * 1023.0)) + 512
        ax_ticks = max(0, min(1023, ax_ticks))
        self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, ax_ticks)
        return True

    def gripper(self, close: bool):
        if self.ax_grip_id is None: return
        tgt = 250 if close else 350 # 예시값 조정해야함. 왼쪽 숫자가 닫힘, 오른쪽이 열렸을 때.
        self.packet_ax.write2ByteTxRx(self.port, self.ax_grip_id, self.AX_ADDR_GOAL_POSITION, tgt)

    def stop_hold(self):
        # 현재 위치에서 정지
        pos_ax,_,_ = self.packet_ax.read2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_PRESENT_POS)
        self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, pos_ax)

    def go_home(self):
        # 홈 위치로 이동
        self.go_home_ticks()

    def ticks_to_joint_rad(self, joint_idx, ticks):
        if joint_idx == 0:  # AX-12A J1
            deg = (ticks - 512) * (300.0/1023.0)
            return np.deg2rad(deg) * self.dir[0]

        # joint_idx를 배열 인덱스로 변환 (J1→0, J2→1, J3→2, J4→3)
        if joint_idx == 1:  # J1은 베이스라서 위에서 처리됨
            return 0.0
        elif joint_idx in [2, 3, 4]:  # J2, J3, J4
            zero_idx = joint_idx - 1  # J2→1, J3→2, J4→3
            gear_idx = joint_idx - 1  # J2→1, J3→2, J4→3
            zero = self.zero[zero_idx]
            motor = (ticks - zero) / RAD2TICKS
            return (motor / self.gear[gear_idx]) * self.dir[gear_idx]
        else:
            return 0.0

    def init_home_positions(self):
        # 홈 틱을 무조건 512로 지정
        self.base_home_tick = 512
        self.base_goal = 512
        self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id,
                                    self.AX_ADDR_GOAL_POSITION,
                                    self.base_home_tick)

        self.xl_home_ticks = []

        # Joint 2, 3번 모터를 먼저 2048로 이동
        for j, dxl_id in enumerate(self.ids_x, start=1):  # j=1..3 → J2..J4
            if j == 1 or j == 2:  # Joint 2, 3번 (ids_x[0], ids_x[1])
                # 포지션 모드에서 2048로 이동
                self.packet_x.write4ByteTxRx(self.port, dxl_id, self.ADDR_GOAL_POSITION, 2048)
                self.get_logger().info(f"Moving joint {j+1} (ID {dxl_id}) to position 2048")

                # 이동 완료까지 대기
                self.write_goal_and_wait(dxl_id, 2048, tol=10, timeout=5.0)

                # 2048을 홈 틱으로 저장
                self.xl_home_ticks.append(2048)
            else:  # Joint 4번은 현재 위치 유지
                pos, res, err = self.packet_x.read4ByteTxRx(self.port, dxl_id, self.ADDR_PRESENT_POSITION)
                if res==0 and err==0:
                    self.xl_home_ticks.append(int(pos))       # 홈 틱 저장
                else:
                    self.xl_home_ticks.append(0)

        # 홈 위치 이동 후 zero offset 설정
        self.zero = [0]
        for dxl_id in self.ids_x:
            pos, result, error = self.packet_x.read4ByteTxRx(self.port, dxl_id, self.ADDR_PRESENT_POSITION)
            if result == 0 and error == 0:
                self.zero.append(pos)
                self.get_logger().info(f"ID {dxl_id} zero offset set to {pos}")
            else:
                self.zero.append(0)
                self.get_logger().error(f"Failed to read position for ID {dxl_id}")

        self.get_logger().info("Home positions initialized")


    def go_home_ticks(self):
        # 베이스 AX 복귀
        self.base_goal = int(self.base_home_tick)  # 512로 리셋
        self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, self.base_goal)
        self.get_logger().info(f"Home: base_goal reset to {self.base_goal}")

        # J2~J4 XL 복귀(GroupSyncWrite)
        self.sync_write.clearParam()
        for dxl_id, t in zip(self.ids_x, self.xl_home_ticks):
            self.sync_write.addParam(dxl_id, self.int32_to_le(int(t)))
        self.sync_write.txPacket()

        self.get_logger().info("Returned to home position")

    def base_rotate_deg(self, delta_deg: float, wait_s: float = 0.0):
        dtick = int(round(delta_deg * AX_TICKS_PER_DEG))
        new_goal = self.base_goal + dtick

        # 베이스 리미트 체크 비활성화
        # new_rad = self.ticks_to_joint_rad(0, new_goal)
        # lo, hi = self.limits[0]  # J1은 인덱스 0
        # if new_rad < lo or new_rad > hi:
        #     self.get_logger().warn(f"Base rotation blocked by limit: {np.rad2deg(new_rad):.1f}° (limit: {np.rad2deg(lo):.1f}° ~ {np.rad2deg(hi):.1f}°)")
        #     return

        self.base_goal = max(0, min(1023, new_goal))
        self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, int(self.base_goal))
        if wait_s > 0:
            time.sleep(wait_s)

    def nudge_joint2_deg(self, delta_deg: float):
        dxl_id = self.ids_x[0]  # J2
        # 현재 틱 읽기
        curr, res, err = self.packet_x.read4ByteTxRx(self.port, dxl_id, self.ADDR_PRESENT_POSITION)
        if res != 0 or err != 0:
            self.get_logger().warn('read fail'); return

        # 조인트 각도(deg) → 모터 틱 델타로 변환 (기어/방향 고려)
        delta_rad  = np.deg2rad(delta_deg)
        delta_tick = int(round(delta_rad * self.dir[1] * self.gear[1] * RAD2TICKS))

        new_tick = curr + delta_tick
        # 필요시 소프트 리미트 걸고 싶으면, 조인트 리미트로 클램프:
        q2_new = self.ticks_to_joint_rad(1, new_tick)
        lo, hi = self.limits[1]
        q2_new = max(lo, min(hi, q2_new))
        new_tick = self.joint_rad_to_motor_ticks(1, q2_new)

        # 목표 틱 쓰기
        self.packet_x.write4ByteTxRx(self.port, dxl_id, self.ADDR_GOAL_POSITION, int(new_tick))

    def nudge_joint_deg(self, joint_num: int, delta_deg: float):
        """조인트 직접 제어 (1=베이스, 2-4=XL 모터)"""
        if joint_num == 1:  # 베이스 AX-12A
            self.base_rotate_deg(delta_deg)
        elif joint_num in [2, 3, 4]:  # XL 모터들
            dxl_id = self.ids_x[joint_num - 2]  # J2=ids_x[0], J3=ids_x[1], J4=ids_x[2]

            # 현재 틱 읽기
            curr, res, err = self.packet_x.read4ByteTxRx(self.port, dxl_id, self.ADDR_PRESENT_POSITION)
            if res != 0 or err != 0:
                self.get_logger().warn(f'Joint {joint_num} read fail'); return

            # unsigned를 signed 32bit로 변환
            if curr > 2_147_483_647:
                curr = curr - 4_294_967_296

            # 조인트 각도(deg) → 모터 틱 델타로 변환
            delta_rad = np.deg2rad(delta_deg)
            # joint_num은 1,2,3,4이지만 배열 인덱스는 0,1,2,3이므로 -1
            gear_idx = joint_num - 1  # J1→0, J2→1, J3→2, J4→3
            delta_tick = int(round(delta_rad * self.dir[gear_idx] * self.gear[gear_idx] * RAD2TICKS))

            new_tick = curr + delta_tick
            # Extended Position 모드 범위로 클램핑
            new_tick = max(-2_147_483_648, min(2_147_483_647, new_tick))

            # 조인트 리미트 체크
            q_new = self.ticks_to_joint_rad(joint_num, new_tick)
            limit_idx = joint_num - 1  # 배열 인덱스 맞춤
            lo, hi = self.limits[limit_idx]

            # 디버그 정보
            self.get_logger().info(f"Joint {joint_num}: curr={curr}, delta_tick={delta_tick}, new_tick={new_tick}, q_new={np.rad2deg(q_new):.1f}°")

            if q_new < lo or q_new > hi:
                self.get_logger().warn(f"Joint {joint_num} limit blocked: {np.rad2deg(q_new):.1f}° (limit: {np.rad2deg(lo):.1f}° ~ {np.rad2deg(hi):.1f}°)")
                return

            # 목표 틱 쓰기
            self.packet_x.write4ByteTxRx(self.port, dxl_id, self.ADDR_GOAL_POSITION, int(new_tick))

    def write_goal_and_wait(self, dxl_id:int, tick:int, tol:int=8, timeout:float=2.0):
        self.packet_x.write4ByteTxRx(self.port, dxl_id, self.ADDR_GOAL_POSITION, int(tick))
        t0 = time.time()
        while time.time()-t0 < timeout:
            pos, res, err = self.packet_x.read4ByteTxRx(self.port, dxl_id, self.ADDR_PRESENT_POSITION)
            if res==0 and err==0 and abs(int(pos)-tick) <= tol:
                return True
            time.sleep(0.01)
        return False

    def on_manual_command(self, msg: String):
        c = msg.data.strip().lower()

        # 초기자세
        if   c == 'home':
            self.go_home_ticks()
            return
        
        # 나뭇가지 치우기용
        if c == 'branch_clear':
            # 특정 틱값으로 이동하는 시퀀스
            self.base_rotate_deg(30, wait_s=2.5)
            time.sleep(2.0)
            self.nudge_joint2_deg(10)
            time.sleep(4.0)
            self.go_home_ticks()
            return

        # 왼/오 → AX 베이스 틱 증분 (단순 회전, IK 동기화 제거)
        if c in ('left','l','right','r'):
            delta_deg = self.base_step_deg if c in ('left','l') else -self.base_step_deg
            dtick = int(round(delta_deg * AX_TICKS_PER_DEG))
            old_goal = self.base_goal
            new_goal = self.base_goal + dtick

            # 각도 리미트 확인
            new_rad = self.ticks_to_joint_rad(0, new_goal)
            lo, hi = self.limits[0]
            if new_rad < lo or new_rad > hi:
                self.get_logger().warn(f"Base {c} blocked by limit: {np.rad2deg(new_rad):.1f}° (limit: {np.rad2deg(lo):.1f}° ~ {np.rad2deg(hi):.1f}°)")
                return

            self.base_goal = max(0, min(1023, new_goal))
            self.get_logger().info(f"Base {c}: {old_goal} -> {self.base_goal} (step={dtick})")
            self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, int(self.base_goal))
            return


        # Back lift commands
        elif c == 'blift_up':
            tick = self.back_up_tick
            self.packet_x.write4ByteTxRx(self.port, self.xl_back_id, self.ADDR_GOAL_POSITION, int(tick))
            return
        elif c == 'blift_down':
            tick = self.back_down_tick
            self.packet_x.write4ByteTxRx(self.port, self.xl_back_id, self.ADDR_GOAL_POSITION, int(tick))
            return
        elif c == 'downblift_down4545':
            tick = self.back_down_45_tick
            self.packet_x.write4ByteTxRx(self.port, self.xl_back_id, self.ADDR_GOAL_POSITION, int(tick))
            return
        elif c in ('blift_center'):
            tick = self.back_center_tick
            self.packet_x.write4ByteTxRx(self.port, self.xl_back_id, self.ADDR_GOAL_POSITION, int(tick))
            return

        # Gripper commands
        elif c == 'open':
            tgt = self.grip_open_tick
            self.packet_ax.write2ByteTxRx(self.port, self.ax_grip_id, self.AX_ADDR_GOAL_POSITION, int(tgt))
            return
        elif c == 'close':
            tgt = self.grip_close_tick
            self.packet_ax.write2ByteTxRx(self.port, self.ax_grip_id, self.AX_ADDR_GOAL_POSITION, int(tgt))
            return
        elif c == 'close_fully':
            tgt = self.grip_close_fully_tick
            self.packet_ax.write2ByteTxRx(self.port, self.ax_grip_id, self.AX_ADDR_GOAL_POSITION, int(tgt))
            return

        # Lift commands
        elif c == 'flift_down':
            tgt = self.x_lift_down_tick
            self.packet_x.write4ByteTxRx(self.port, self.x_lift_id, self.ADDR_GOAL_POSITION, int(tgt))
            return
        elif c == 'flift_center':
            tgt = self.x_lift_center_tick
            self.packet_x.write4ByteTxRx(self.port, self.x_lift_id, self.ADDR_GOAL_POSITION, int(tgt))
            return
        elif c == 'flift_up':
            tgt = self.x_lift_up_tick
            self.packet_x.write4ByteTxRx(self.port, self.x_lift_id, self.ADDR_GOAL_POSITION, int(tgt))
            return

        # 개별 조인트 틱값 직접 제어
        elif c == 'j1_plus':
            self.nudge_joint_deg(1, self.base_step_deg)
            return
        elif c == 'j1_minus':
            self.nudge_joint_deg(1, -self.base_step_deg)
            return
        elif c == 'j2_plus':
            self.nudge_joint_deg(2, 5.0)
            return
        elif c == 'j2_minus':
            self.nudge_joint_deg(2, -5.0)
            return
        elif c == 'j3_plus':
            self.nudge_joint_deg(3, 5.0)
            return
        elif c == 'j3_minus':
            self.nudge_joint_deg(3, -5.0)
            return
        elif c == 'j4_plus':
            self.nudge_joint_deg(4, 5.0)
            return
        elif c == 'j4_minus':
            self.nudge_joint_deg(4, -5.0)
            return

        # Arduino commands (pass through, no action needed here)
        elif c in ('go', 'stop'):
            return

        else:
            # 유효하지 않은 명령어는 조용히 무시
            return



        

def main():
    rclpy.init()
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
