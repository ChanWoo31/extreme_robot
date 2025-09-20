#groupsync version -> 매뉴얼
import numpy as np
import time
from ikpy.link import OriginLink, DHLink
from ikpy.chain import Chain

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite

d1 = 0
a2, a3, a4 = 0.18525, 0.18525, 0.256

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

        # --- arm manual ---
        self.home_x = self.declare_parameter('home_x', 0.07075).get_parameter_value().double_value
        self.home_y = self.declare_parameter('home_y', 0.0).get_parameter_value().double_value
        self.home_z = self.declare_parameter('home_z', 0.18525).get_parameter_value().double_value
        self.cart_step = self.declare_parameter('cart_step_m', 0.1).get_parameter_value().double_value  # 10 cm
        self.home = np.array([self.home_x, self.home_y, self.home_z], dtype=float)
        # self.manual_offset = np.zeros(3, dtype=float)

        self.base_step_deg = self.declare_parameter('base_step_deg', 5.0).get_parameter_value().double_value

        # ---- 추가 구독 ----
        self.sub_arm_manual  = self.create_subscription(String, '/arm/cmd/manual',       self.on_arm_manual, 50)
        self.sub_lift        = self.create_subscription(String, '/lift/cmd/manual',      self.on_lift, 20)
        self.sub_back_lift   = self.create_subscription(String, '/back_lift/cmd/manual', self.on_back_lift, 20)
        self.sub_gripper     = self.create_subscription(String, '/gripper/cmd/manual',   self.on_gripper, 20)

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
            # # 초기 위치 정밀 세팅을 위한 포지션 모드 3번
            # self.packet_x.write1ByteTxRx(self.port, dxl_id, self.ADDR_OPERATING_MODE, 3)
            # # 틱값 2800으로 이동
            # self.packet_x.write4ByteTxRx(self.port, dxl_id, self.ADDR_GOAL_POSITION, 2800)
            # time.sleep(0.5)  # 잠시 대기
            # # 위치 도달 대기
            # self.write_goal_and_wait(dxl_id, 2800, tol=16, timeout=3.0)
            # # 토크 끄고
            # self.packet_x.write1ByteTxRx(self.port, dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            # time.sleep(0.1)
            # # 역 ㄷ자에 맞게 홈 오프셋 설정 (엔코더 0점 보정)
            # self.packet_x.write4ByteTxRx(self.port, dxl_id, 20, 2800)
            # time.sleep(0.1)
            # # 다시 토크 온
            # self.packet_x.write1ByteTxRx(self.port, dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            # time.sleep(0.1)
            # # 이제부터는 Extended 포지션 모드 4번
            # Extended 포지션 모드 4번
            self.packet_x.write1ByteTxRx(self.port, dxl_id, self.ADDR_OPERATING_MODE, 4)
            # current limit
            self.packet_x.write2ByteTxRx(self.port, dxl_id, self.ADDR_CURRENT_LIMIT, 300)
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

        # 기구/맵핑
        self.gear = [1.0, 5.0, 5.0, 5.0]
        self.dir = [1, -1, 1, 1]
        self.zero = [0]
        for dxl_id in self.ids_x:
            pos, result, error = self.packet_x.read4ByteTxRx(self.port, dxl_id, self.ADDR_PRESENT_POSITION)
            if result == 0 and error == 0:
                self.zero.append(pos)
                self.get_logger().info(f"ID {dxl_id} initial pos (zero offset)")
            else:
                self.zero.append(0)
                self.get_logger().error(f"Failed") 

        # 역 ㄷ자 시작
        self.chain = Chain(name='arm4', links=[
            OriginLink(),
            DHLink(d=d1, a=0, alpha=np.deg2rad(90), theta=0),
            DHLink(d=0, a=a2, alpha=0, theta=np.deg2rad(0)),
            DHLink(d=0, a=a3, alpha=0, theta=np.deg2rad(90)),
            DHLink(d=0, a=a4, alpha=0, theta=np.deg2rad(90)),
        ]
        )
        self.z_offset = 0.0
        # 리미트!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        self.limits=[(-np.deg2rad(150), np.deg2rad(150)),
                     (-np.deg2rad(0), np.deg2rad(170)),
                     (-np.deg2rad(90), np.deg2rad(5)),
                     (-np.deg2rad(100), np.deg2rad(5))]
        
        #잘 되는지 테스트 필요
        # self.last_q = np.zeros(len(self.chain.links))
        self.violation_latched = False
        self.block_until = 0.0
        self.block_sec = 1.0
        
        self.init_last_q_from_present()
        # self.go_home()

        


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
        ax_ticks = int(round((deg / 300.0) * 1023.0)) + 512
        ax_ticks = max(0, min(1023, ax_ticks))
        self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, ax_ticks)

    def gripper(self, close: bool):
        if self.ax_grip_id is None: return
        tgt = 250 if close else 350 # 예시값 조정해야함. 왼쪽 숫자가 닫힘, 오른쪽이 열렸을 때.
        self.packet_ax.write2ByteTxRx(self.port, self.ax_grip_id, self.AX_ADDR_GOAL_POSITION, tgt)

    def stop_hold(self):
        pos_ax,_,_ = self.packet_ax.read2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_PRESENT_POS)
        self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, pos_ax)
        self.send_x_positions(*[ (self.last_q[i+1]) for i in range(3)])

    def go_home(self):
        q = [0.0, 0.0, 0.0, 0.0]
        self.send_ax_base_deg(np.rad2deg(q[0]) * self.dir[0])
        self.send_x_positions(q[1], q[2] ,q[3])
        tmp = np.zeros(len(self.chain.links)); tmp[1:5] = q; self.last_q = tmp

    def ticks_to_joint_rad(self, joint_idx, ticks):
        if joint_idx == 0:  # AX-12A J1
            deg = (ticks - 512) * (300.0/1023.0)
            return np.deg2rad(deg) * self.dir[0]
        zero = self.zero[joint_idx]                  # zero[1]→ID2, zero[2]→ID3, zero[3]→ID4
        motor = (ticks - zero) / RAD2TICKS
        return (motor / self.gear[joint_idx]) * self.dir[joint_idx]

    def init_last_q_from_present(self):
        q = [0.0]*4
        ax_pos,_,_ = self.packet_ax.read2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_PRESENT_POS)

        # 홈 틱을 무조건 512로 지정
        self.base_home_tick = 512
        self.base_goal = 512
        self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id,
                                    self.AX_ADDR_GOAL_POSITION,
                                    self.base_home_tick)

        self.xl_home_ticks = []
        q[0] = self.ticks_to_joint_rad(0, ax_pos)
        for j, dxl_id in enumerate(self.ids_x, start=1):  # j=1..3 → J2..J4
            pos, res, err = self.packet_x.read4ByteTxRx(self.port, dxl_id, self.ADDR_PRESENT_POSITION)
            if res==0 and err==0:
                self.xl_home_ticks.append(int(pos))       # 홈 틱 저장
                q[j] = self.ticks_to_joint_rad(j, int(pos))
            else:
                self.xl_home_ticks.append(0)

        self.last_q = np.zeros(len(self.chain.links))
        self.last_q[1:5] = q
        self.get_logger().info(f"Init last_q(deg)={np.rad2deg(self.last_q[1:5])}")

    def move_ik(self, x, y, z):
        ik = self.chain.inverse_kinematics(
            target_position=[-x, y, z],
            orientation_mode=None,
            initial_position=self.last_q,
        )
        q = list(ik[1:5])

        for i in range(4):
            lo, hi = self.limits[i]
            if q[i] < lo: q[i] = lo
            if q[i] > hi: q[i] = hi

        self.send_ax_base_deg(np.rad2deg(q[0]) * self.dir[0])
        self.send_x_positions(q[1], q[2] ,q[3])
        tmp = self.last_q.copy(); tmp[1:5] = q; self.last_q = tmp

        fk = self.chain.forward_kinematics(np.r_[0.0, q])  # 0+J1..J4
        reach = fk[:3, 3]
        err = np.linalg.norm(reach - np.array([x, y, z]))
        self.get_logger().info(f"FK reach=({reach[0]:.5f},{reach[1]:.5f},{reach[2]:.5f}), err={err*1000:.3f} mm")

    def go_home_ticks(self):
        # 베이스 AX 복귀
        self.base_goal = int(self.base_home_tick)
        self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, self.base_goal)

        # J2~J4 XL 복귀(GroupSyncWrite)
        self.sync_write.clearParam()
        for dxl_id, t in zip(self.ids_x, self.xl_home_ticks):
            self.sync_write.addParam(dxl_id, self.int32_to_le(int(t)))
        self.sync_write.txPacket()

        # IK 내부 상태 동기
        q0 = self.ticks_to_joint_rad(0, self.base_home_tick)
        q1 = self.ticks_to_joint_rad(1, self.xl_home_ticks[0])
        q2 = self.ticks_to_joint_rad(2, self.xl_home_ticks[1])
        q3 = self.ticks_to_joint_rad(3, self.xl_home_ticks[2])
        self.last_q[1:5] = [q0, q1, q2, q3]

    def goto_xyz(self, x, y, z):
        """임의 좌표로 점프 + 이후 조그는 이 지점에서 계속 누적"""
        # 1) IK로 이동
        self.move_ik(x, y, z)
        # 2) 누적 기준을 이 점에 맞추기
        self.manual_offset[:] = np.array([x, y, z]) - self.home

    def base_rotate_deg(self, delta_deg: float, wait_s: float = 0.0):
        dtick = int(round(delta_deg * AX_TICKS_PER_DEG))
        self.base_goal = max(0, min(1023, self.base_goal + dtick))
        self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, int(self.base_goal))
        # IK 초기값 동기 (J1 라디안 재계산)
        self.last_q[1] = self.ticks_to_joint_rad(0, self.base_goal)
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

        # 내부 상태 업데이트(다음 IK/조깅 일관성)
        self.last_q[2] = q2_new

    def write_goal_and_wait(self, dxl_id:int, tick:int, tol:int=8, timeout:float=2.0):
        self.packet_x.write4ByteTxRx(self.port, dxl_id, self.ADDR_GOAL_POSITION, int(tick))
        t0 = time.time()
        while time.time()-t0 < timeout:
            pos, res, err = self.packet_x.read4ByteTxRx(self.port, dxl_id, self.ADDR_PRESENT_POSITION)
            if res==0 and err==0 and abs(int(pos)-tick) <= tol:
                return True
            time.sleep(0.01)
        return False

    def on_arm_manual(self, msg: String):
        c = msg.data.strip().lower()
        s = self.cart_step

        # 초기자세
        if   c == 'home':
            self.go_home_ticks()
            return
        
        # 나뭇가지 치우기용
        if c == 'branch_clear':
            self.goto_xyz(0.55, 0.0, -0.2)
            time.sleep(8.0)
            self.base_rotate_deg(30, wait_s=2.5)
            # self.base_rotate_deg(-30, wait_s=2.5)
            # self.goto_xyz(0.55, 0.0, 0.0)
            time.sleep(2.0)
            self.nudge_joint2_deg(10)
            time.sleep(4.0)
            self.go_home_ticks()
            return

        # 왼/오 → AX 베이스 틱 증분
        if c in ('left','l','right','r'):
            dtick = int(round(self.base_step_deg * AX_TICKS_PER_DEG))
            if c in ('left','l'):
                self.base_goal = min(1023, self.base_goal + dtick)
            else:
                self.base_goal = max(0,    self.base_goal - dtick)
            self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, int(self.base_goal))
            # IK 초기값 동기
            self.last_q[1] = self.ticks_to_joint_rad(0, self.base_goal)
            return

        # 앞/뒤/위/아래 → 홈 기준 누적 후 IK
        if   c in ('front','f'): self.manual_offset[0] += s
        elif c in ('back','b'):  self.manual_offset[0] -= s
        elif c in ('up','u'):    self.manual_offset[2] += s
        elif c in ('down','d'):  self.manual_offset[2] -= s
        elif c.startswith('step '):
            try:
                self.cart_step = float(c.split()[1]); self.get_logger().info(f'step={self.cart_step} m')
            except Exception:
                self.get_logger().warn('usage: step <meters>')
            return
        else:
            self.get_logger().warn(f'unknown cmd: {c}'); return

        tgt = self.home + self.manual_offset
        self.move_ik(tgt[0], tgt[1], tgt[2])


    def on_back_lift(self, msg: String):
        c = msg.data.strip().lower()
        if   c == 'up':       tick = self.back_up_tick
        elif c == 'down':     tick = self.back_down_tick
        elif c == 'down45':     tick = self.back_down_45_tick
        elif c in ('center','home'):
            tick = self.back_center_tick
        else:
            self.get_logger().warn('use: up|down|center'); return
        self.packet_x.write4ByteTxRx(self.port, self.xl_back_id, self.ADDR_GOAL_POSITION, int(tick))

    def on_gripper(self, msg: String):
        c = msg.data.strip().lower()
        if   c == 'open':  tgt = self.grip_open_tick
        elif c == 'close': tgt = self.grip_close_tick
        elif c == 'close_fully': tgt = self.grip_close_fully_tick
        else:
            self.get_logger().warn('use: open|close'); return
        self.packet_ax.write2ByteTxRx(self.port, self.ax_grip_id, self.AX_ADDR_GOAL_POSITION, int(tgt))

    def on_lift(self, msg: String):
        c = msg.data.strip().lower()
        if   c == 'down':   tgt = self.x_lift_down_tick
        elif c == 'center': tgt = self.x_lift_center_tick
        elif c == 'up':     tgt = self.x_lift_up_tick
        else: return
        self.packet_x.write4ByteTxRx(self.port, self.x_lift_id, self.ADDR_GOAL_POSITION, int(tgt))

        

def main():
    rclpy.init()
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
