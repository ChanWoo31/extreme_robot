# arm server
import numpy as np
import time
import threading
from ikpy.link import OriginLink, DHLink
from ikpy.chain import Chain

import rclpy
from rclpy.node import Node

from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite

from arm_interfaces.srv import ArmDoTask
from arm_interfaces.msg import DetectedObject
from std_msgs.msg import String

from .missions_robotarm import get_mission_handler, get_available_missions

# d1 = 0.27
d1 = 0.0
a2, a3, a4 = 0.18525, 0.18525, 0.256

RAD2TICKS = 4096 / (2*np.pi)
AX_TICKS_PER_DEG = 1023.0 / 300.0
 
class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.srv = self.create_service(ArmDoTask, 'arm_do_task', self.handle_arm_do_task)

        # 자동 감지 시스템
        self.detected_sub = self.create_subscription(
            DetectedObject, 'detected_objects', self.handle_detected_object, 10
        )

        # 색상 추적용 서브스크라이버
        self.direction_subscriber = self.create_subscription(
            String, '/direction', self.direction_callback, 10
        )

        # 색상 추적 상태
        self.current_direction = None
        self.color_tracking_active = False

        # ebox 위치 추적 상태
        self.latest_ebox_detection = None

        # 미션 실행 상태 플래그
        self.mission_in_progress = False
        self.ebox_aligned = False  # ebox 정렬 완료 플래그
        self.last_mission_time = 0  # 마지막 미션 완료 시간
        self.mission_cooldown = 10.0  # 10초 쿨다운

        # 자동 미션 제어
        self.enable_auto_mission = False  # 자동 미션 기본 꺼짐
        self.active_task = None           # 현재 실행 중 태스크명
        self.state_lock = threading.Lock()

        # 미션 시스템 초기화
        self.available_missions = get_available_missions()
        self.get_logger().info(f"Available missions: {self.available_missions}")
        
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
        
        # X-series 초기 설정
        for dxl_id in self.ids_x:
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
        self.dir = [1, 1, 1, 1]
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
            DHLink(d=d1, a=0, alpha=np.deg2rad(-90), theta=0),
            DHLink(d=0, a=a2, alpha=0, theta=np.deg2rad(0)),
            DHLink(d=0, a=a3, alpha=0, theta=np.deg2rad(-90)),
            DHLink(d=0, a=a4, alpha=np.deg2rad(-90), theta=np.deg2rad(-90)),
        ]
        )
        self.z_offset = 0.0
        # 위에보다 더 보수적인 리미트.
        self.limits=[(-np.deg2rad(150), np.deg2rad(150)),
                     (-np.deg2rad(180), np.deg2rad(5)),
                     (-np.deg2rad(5), np.deg2rad(90)),
                     (-np.deg2rad(5), np.deg2rad(100))]
        
        #잘 되는지 테스트 필요
        # self.last_q = np.zeros(len(self.chain.links))
        self.violation_latched = False
        self.block_until = 0.0
        self.block_sec = 1.0
        
        self.init_last_q_from_present()
        self.dt = 0.02 # 50Hz
        # self.go_home()

        # 카메라-엔드이펙터 변환 매트릭스 (수정 가능)
        self.camera_offset = np.array([-0.084, 0.0, 0.04155])  # 카메라가 엔드이펙터에서 뒤쪽으로 8.4cm
        self.camera_rotation = np.eye(3)  # 회전 없음 (cam_to_common에서 이미 변환됨)

    # 유틸

    def int32_to_le(self, v:int):
        """32비트 정수를 리틀 엔디안 바이트 배열로 변환"""
        v &= 0xFFFFFFFF
        return bytes([v & 0xFF, (v>>8)&0xFF, (v>>16)&0xFF, (v>>24)&0xFF])

    def joint_rad_to_motor_ticks(self, i, q_joint):
        """관절 라디안 값을 모터 틱 값으로 변환 (기어비, 방향, 영점 오프셋 적용)"""
        q_motor = (q_joint * self.dir[i]) * self.gear[i]
        ticks = int(round(q_motor * RAD2TICKS)) + self.zero[i]
        return max(-2_147_483_648, min(2_147_483_647, ticks))

    def send_x_positions(self, q2, q3, q4):
        """X시리즈 모터(J2,J3,J4)에 동기 명령 전송"""
        ticks = [self.joint_rad_to_motor_ticks(1, q2), self.joint_rad_to_motor_ticks(2, q3), self.joint_rad_to_motor_ticks(3, q4)]
        self.sync_write.clearParam()
        for dxl_id, t in zip(self.ids_x, ticks):
            self.sync_write.addParam(dxl_id, self.int32_to_le(t))
        self.sync_write.txPacket()

    def send_ax_base_deg(self, deg):
        """AX-12A 베이스 모터(J1)에 각도 명령 전송"""
        ax_ticks = int(round((deg / 300.0) * 1023.0)) + 512
        ax_ticks = max(0, min(1023, ax_ticks))
        self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, ax_ticks)

    def gripper(self, close: bool):
        """그리퍼 열기/닫기 제어"""
        if self.ax_grip_id is None: return
        tgt = 330 if close else 480 # 예시값 조정해야함. 왼쪽 숫자가 닫힘, 오른쪽이 열렸을 때.
        self.packet_ax.write2ByteTxRx(self.port, self.ax_grip_id, self.AX_ADDR_GOAL_POSITION, tgt)

    def stop_hold(self):
        """현재 위치에서 정지 및 유지"""
        pos_ax,_,_ = self.packet_ax.read2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_PRESENT_POS)
        self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, pos_ax)
        self.send_x_positions(*[ (self.last_q[i+1]) for i in range(3)])

    def go_home(self):
        """홈 포지션(0,0,0,0)으로 이동"""
        q = [0.0, 0.0, 0.0, 0.0]
        self.send_ax_base_deg(np.rad2deg(q[0]) * self.dir[0])
        self.send_x_positions(q[1], q[2] ,q[3])
        tmp = np.zeros(len(self.chain.links)); tmp[1:5] = q; self.last_q = tmp

    def ticks_to_joint_rad(self, joint_idx, ticks):
        """모터 틱 값을 관절 라디안 값으로 역변환"""
        if joint_idx == 0:  # AX-12A J1
            deg = (ticks - 512) * (300.0/1023.0)
            return np.deg2rad(deg) * self.dir[0]
        zero = self.zero[joint_idx]                  # zero[1]→ID2, zero[2]→ID3, zero[3]→ID4
        motor = (ticks - zero) / RAD2TICKS
        return (motor / self.gear[joint_idx]) * self.dir[joint_idx]

    def write_goal_and_wait(self, dxl_id:int, tick:int, tol:int=8, timeout:float=2.0):
        """목표 위치 전송 후 도달까지 대기"""
        self.packet_x.write4ByteTxRx(self.port, dxl_id, self.ADDR_GOAL_POSITION, int(tick))
        t0 = time.time()
        while time.time()-t0 < timeout:
            pos, res, err = self.packet_x.read4ByteTxRx(self.port, dxl_id, self.ADDR_PRESENT_POSITION)
            if res==0 and err==0 and abs(int(pos)-tick) <= tol:
                return True
            time.sleep(0.01)
        return False

    def init_last_q_from_present(self):
        """현재 모터 위치로부터 마지막 관절각 초기화 및 홈 포지션 설정"""
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
                q[j] = self.ticks_to_joint_rad(j, 2048)
            else:  # Joint 4번은 현재 위치 유지
                pos, res, err = self.packet_x.read4ByteTxRx(self.port, dxl_id, self.ADDR_PRESENT_POSITION)
                if res==0 and err==0:
                    self.xl_home_ticks.append(int(pos))       # 홈 틱 저장
                    q[j] = self.ticks_to_joint_rad(j, int(pos))
                else:
                    self.xl_home_ticks.append(0)

        self.last_q = np.zeros(len(self.chain.links))
        self.last_q[1:5] = q
        self.get_logger().info(f"Init last_q(deg)={np.rad2deg(self.last_q[1:5])}")

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

    def go_autonomous_driving_position(self):
        """자율주행용 포지션: J2를 -10도 조정해서 카메라가 아래를 바라보도록"""
        # 베이스는 홈 위치 유지
        self.base_goal = int(self.base_home_tick)
        self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, self.base_goal)

        # J2: DH 기준 0도에서 -10도, J3: DH 기준 0도 유지, J4: 홈 위치 유지
        j2_angle = np.deg2rad(-20)  # DH 파라미터 기준 -10도
        j2_ticks = self.zero[1] + int(j2_angle * self.gear[1] * self.dir[1] * RAD2TICKS)

        autonomous_ticks = [
            j2_ticks,  # J2: -10도 (DH 기준, 아래 바라보도록)
            2048,      # J3: 90도 (물리적 위치)
            self.xl_home_ticks[2]  # J4: 홈 위치 유지
        ]

        # 동기 전송
        self.sync_write.clearParam()
        for dxl_id, t in zip(self.ids_x, autonomous_ticks):
            self.sync_write.addParam(dxl_id, self.int32_to_le(int(t)))
        self.sync_write.txPacket()

        self.get_logger().info("Moved to autonomous driving position: J2=-10° (DH basis, looking down)")

    def go_ik_ready_position(self):
        """역기구학 작업용 포지션: DH 파라미터 기준 자세로 복귀"""
        # 홈 틱 기준으로 복귀 (J2,J3=90도)
        self.go_home_ticks()
        self.get_logger().info("Returned to IK-ready position (DH parameter reference pose)")

    def move_from_home_ticks(self, offset_j1=0, offset_j2=0, offset_j3=0, offset_j4=0):
        """go_home_ticks 위치에서 오프셋만큼 이동"""
        # J1 (베이스) - base_home_tick에서 오프셋
        target_base = self.base_home_tick + offset_j1
        self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, int(target_base))

        # J2, J3, J4 - xl_home_ticks에서 오프셋
        target_positions = [
            self.xl_home_ticks[0] + offset_j2,  # J2
            self.xl_home_ticks[1] + offset_j3,  # J3
            self.xl_home_ticks[2] + offset_j4   # J4
        ]

        # 동기 전송
        self.sync_write.clearParam()
        for dxl_id, target_pos in zip(self.ids_x, target_positions):
            self.sync_write.addParam(dxl_id, self.int32_to_le(int(target_pos)))

        self.sync_write.txPacket()

        time.sleep(self.dt)
        return True

    def move_xl_only(self, offset_j2=0, offset_j3=0, offset_j4=0):
        """베이스는 유지하고 J2, J3, J4만 이동"""
        # J2, J3, J4 - xl_home_ticks에서 오프셋
        target_positions = [
            self.xl_home_ticks[0] + offset_j2,  # J2
            self.xl_home_ticks[1] + offset_j3,  # J3
            self.xl_home_ticks[2] + offset_j4   # J4
        ]

        # 동기 전송 (베이스 제외)
        self.sync_write.clearParam()
        for dxl_id, target_pos in zip(self.ids_x, target_positions):
            self.sync_write.addParam(dxl_id, self.int32_to_le(int(target_pos)))

        self.sync_write.txPacket()

        time.sleep(self.dt)
        return True

    def solve_ik_q(self, x, y, z):
        # 여러 초기값으로 IK 시도하여 리미트 내 해 찾기
        initial_positions = [
            self.last_q,  # 현재 자세
            np.array([0, 0, 0, 0, 0]),  # 홈 포지션
            np.array([0, 0, np.deg2rad(90), 0, 0]),  # J3=90도 시작점

            # J1=0 고정, J2=-130°까지 제한된 초기값들
            np.array([0, np.deg2rad(-90), np.deg2rad(45), np.deg2rad(80), 0]),   # J2=-90°
            np.array([0, np.deg2rad(-100), np.deg2rad(50), np.deg2rad(85), 0]),  # J2=-100°
            np.array([0, np.deg2rad(-110), np.deg2rad(55), np.deg2rad(90), 0]),  # J2=-110°
            np.array([0, np.deg2rad(-120), np.deg2rad(60), np.deg2rad(85), 0]),  # J2=-120°
            np.array([0, np.deg2rad(-130), np.deg2rad(65), np.deg2rad(80), 0]),  # J2=-130°
            np.array([0, np.deg2rad(-80), np.deg2rad(40), np.deg2rad(75), 0]),   # J2=-80°
            np.array([0, np.deg2rad(-70), np.deg2rad(35), np.deg2rad(70), 0]),   # J2=-70°
            np.array([0, np.deg2rad(-60), np.deg2rad(30), np.deg2rad(65), 0]),   # J2=-60°
        ]

        for init_q in initial_positions:
            try:
                ik = self.chain.inverse_kinematics(
                    target_position=[-x, y, z],
                    orientation_mode=None,
                    initial_position=init_q,
                )
                q = list(ik[1:5])

                # 리미트 체크
                valid = True
                for i, (lo, hi) in enumerate(self.limits):
                    if q[i] < lo or q[i] > hi:
                        valid = False
                        break

                if valid:
                    return q
            except:
                continue

        self.get_logger().warn(f"IK failed for target ({x:.3f}, {y:.3f}, {z:.3f}) - no valid solution within limits")
        self.get_logger().warn(f"Transformed target: {[-x, y, z]}")
        return None

    def debug_ik_without_limits(self, x, y, z):
        """디버깅: 다양한 초기값으로 IK 해 확인"""
        initial_positions = [
            self.last_q,
            np.array([0, 0, 0, 0, 0]),
            # J1=0 고정, J2=-130°까지 제한
            np.array([0, np.deg2rad(-90), np.deg2rad(45), np.deg2rad(80), 0]),
            np.array([0, np.deg2rad(-100), np.deg2rad(50), np.deg2rad(85), 0]),
            np.array([0, np.deg2rad(-110), np.deg2rad(55), np.deg2rad(90), 0]),
            np.array([0, np.deg2rad(-120), np.deg2rad(60), np.deg2rad(85), 0]),
            np.array([0, np.deg2rad(-130), np.deg2rad(65), np.deg2rad(80), 0]),
        ]

        for i, init_q in enumerate(initial_positions):
            try:
                ik = self.chain.inverse_kinematics(
                    target_position=[-x, y, z],
                    orientation_mode=None,
                    initial_position=init_q,
                )
                q = list(ik[1:5])

                self.get_logger().info(f"Initial {i}: {[np.rad2deg(angle) for angle in init_q[1:5]]}")
                self.get_logger().info(f"IK solution {i}: {[np.rad2deg(angle) for angle in q]}")

                # 리미트 체크
                all_ok = True
                for j, (lo, hi) in enumerate(self.limits):
                    status = "OK" if lo <= q[j] <= hi else "VIOLATION"
                    if status == "VIOLATION":
                        all_ok = False
                    self.get_logger().info(f"  J{j+1}: {np.rad2deg(q[j]):.1f}° [{np.rad2deg(lo):.1f}°, {np.rad2deg(hi):.1f}°] {status}")

                if all_ok:
                    self.get_logger().info(f"*** FOUND VALID SOLUTION with initial {i} ***")
                    return q

                self.get_logger().info("---")

            except Exception as e:
                self.get_logger().warn(f"Initial {i} failed: {e}")

        return None
    
    def send_q(self, q):
        self.send_ax_base_deg(np.rad2deg(q[0]) * self.dir[0])
        self.send_x_positions(q[1], q[2] ,q[3])
        tmp = self.last_q.copy(); tmp[1:5] = q; self.last_q = tmp

    def move_ik(self, x, y, z):
        self.get_logger().info(f"move_ik called with target: ({x}, {y}, {z})")
        q = self.solve_ik_q(x, y, z)
        if q is None:
            self.get_logger().error(f"IK failed for target ({x}, {y}, {z})")
            return False
        self.get_logger().info(f"IK success: q = {[np.rad2deg(angle) for angle in q]}")
        self.send_q(q)
        time.sleep(self.dt)
        return True
    
    def move_lin(self, p_from, p_to, T=1.5, steps=20):
        for s in np.linspace(0, 1, steps):
            p = (1-s)*p_from + s*p_to
            if not self.move_ik(*p): return False
            time.sleep(T/steps)
        return True
    

    
    # 동작 시퀀스

    def pick_seq(self, target_pos=None):
        """박스 pick 시퀀스"""
        self.get_logger().info("박스 pick 시퀀스 시작")
        self.gripper(False)  # 그리퍼 열기
        # 카메라 다운룩 포즈 등 안전자세 진입
        self.move_from_home_ticks(offset_j2=-3500, offset_j3=952, offset_j4=0)
        self._sleep(3.0)

        self.get_logger().info("Aligning with ebox position...")
        aligned = self.align_with_ebox_position(max_attempts=20, timeout=25.0)
        if not aligned:
            self.get_logger().warn("Alignment failed. Abort pick.")
            self.go_home_ticks()
            self.get_logger().info("박스 pick 시퀀스 완료")
            return False

        # 정렬된 경우에만 접근·그립
        self.move_xl_only(offset_j2=-7748, offset_j3=2048, offset_j4=2187)
        self._sleep(5.0)
        self.gripper(True)  # 그리퍼 닫기
        self._sleep(1.5)
        self.move_xl_only(offset_j2=-4120, offset_j3=2048, offset_j4=2187)
        self._sleep(3.0)

        self.go_home_ticks()
        self.get_logger().info("박스 pick 시퀀스 완료")
        return True

    def direction_callback(self, msg):
        """RealSense 색상 추적에서 오는 방향 정보 처리"""
        if self.color_tracking_active:
            self.current_direction = msg.data
            self.get_logger().info(f"Received direction: {self.current_direction}")

    def publish_color_target(self, color):
        """색상 추적 목표 색상 설정"""
        msg = String()
        msg.data = color
        self.target_color_publisher.publish(msg)
        self.get_logger().info(f"Published target color: {color}")

    def stop_color_tracking(self):
        """색상 추적 중지"""
        msg = String()
        msg.data = ""  # 빈 문자열로 추적 중지
        self.target_color_publisher.publish(msg)
        self.color_tracking_active = False
        self.get_logger().info("Color tracking stopped")

    def led_button_press_sequence(self):
        """
        LED 버튼 누르기 전체 시퀀스 (외부 색상 신호 대기)
        1. 위로 올리기 (충돌 방지)
        2. 외부 색상 토픽 대기 (test_opencv에서 /target_color 받을 때까지)
        3. 베이스 회전으로 중앙점 맞추기 (left/right 신호 기반)
        4. center 신호 받으면 버튼 누르기
        5. 홈으로 복귀
        """

        # 1단계: 위로 올리기 (안전 위치)
        self.get_logger().info("Step 1: Moving to safe position")
        if not self.move_to_safe_position():
            self.get_logger().error("Failed to move to safe position")
            return False

        # 2단계: 외부 색상 신호 대기
        self.get_logger().info("Step 2: Waiting for external color signal...")
        self.color_tracking_active = True
        self.current_direction = None

        # 외부에서 색상 토픽이 발행될 때까지 대기
        # (누군가 test_opencv의 /target_color 토픽으로 색상을 보내줄 때까지)

        # 3단계: 베이스 회전으로 중앙점 맞추기
        self.get_logger().info("Step 3: Aligning with color target")
        if not self.align_with_color_target(max_attempts=20, timeout=30.0):  # 타임아웃 증가
            self.get_logger().error("Failed to align with color target")
            self.color_tracking_active = False
            return False

        # 4단계: 색상 추적 중지
        self.get_logger().info("Step 4: Stopping color tracking")
        self.stop_color_tracking()

        # 5단계: 버튼 누르기
        self.get_logger().info("Step 5: Pressing LED button")
        if not self.press_led_button_ticks():
            self.get_logger().error("Failed to press LED button")
            return False

        # 6단계: 홈으로 복귀
        self.get_logger().info("Step 6: Returning home")
        self.go_home_ticks()
        self.get_logger().info("LED button press sequence completed successfully")
        return True

    def align_with_color_target(self, max_attempts=20, timeout=10.0):
        """색상 추적 기반 베이스 회전 정렬"""
        attempts = 0
        start_time = time.time()

        while attempts < max_attempts and (time.time() - start_time) < timeout:
            if self.current_direction is None:
                # 방향 신호 대기
                time.sleep(0.1)
                attempts += 1
                continue

            if self.current_direction == "center":
                self.get_logger().info("Target centered! Alignment complete")
                return True

            elif self.current_direction == "left":
                # 목표가 왼쪽에 있으면 베이스를 반시계방향으로 회전
                self.rotate_base_by_ticks(-15)
                self.get_logger().info("Rotating base counter-clockwise")

            elif self.current_direction == "right":
                # 목표가 오른쪽에 있으면 베이스를 시계방향으로 회전
                self.rotate_base_by_ticks(15)
                self.get_logger().info("Rotating base clockwise")

            time.sleep(0.3)  # 회전 후 안정화 대기
            attempts += 1

        self.get_logger().warn(f"Failed to align after {attempts} attempts or {timeout}s timeout")
        return False

    def align_with_ebox_position(self, max_attempts=25, timeout=30.0, threshold=15.0):
        """ebox 감지 정보 기반 베이스 회전 정렬 (정교한 중앙 정렬)"""
        attempts = 0
        start = time.time()
        last_dx = None
        stagnant = 0

        while attempts < max_attempts and (time.time() - start) < timeout:
            det = self.latest_ebox_detection
            if det is None:
                self.get_logger().info("Waiting for ebox detection...")
                time.sleep(0.1)
                attempts += 1
                continue

            dx = det.distance_from_center_x
            self.get_logger().info(f"Ebox distance from center_x: {dx:.1f}, threshold: {threshold}")

            if abs(dx) <= threshold:
                self.get_logger().info("Ebox centered! Alignment complete")
                self.ebox_aligned = True
                return True

            # 개선 여부 체크
            if last_dx is not None and abs(dx) >= abs(last_dx) - 1.0:
                stagnant += 1
            else:
                stagnant = 0
            last_dx = dx
            if stagnant >= 4:
                self.get_logger().warn("Alignment stalled (no improvement 4 steps). Abort.")
                return False

            # 정교한 스텝 크기 조정
            if abs(dx) > 60:
                step = 12    # 멀리 있을 때: 큰 스텝
            elif abs(dx) > 30:
                step = 6     # 중간 거리: 중간 스텝
            elif abs(dx) > 15:
                step = 3     # 가까운 거리: 작은 스텝
            else:
                step = 1     # 매우 가까운 거리: 미세 조정

            if dx < -threshold:
                self.rotate_base_by_ticks(+step)   # 좌측 → 시계방향
                self.get_logger().info(f"Rotating base clockwise +{step} ticks (ebox on left)")
            else:
                self.rotate_base_by_ticks(-step)   # 우측 → 반시계
                self.get_logger().info(f"Rotating base counter-clockwise -{step} ticks (ebox on right)")

            # 프레임 갱신 대기
            time.sleep(0.3)
            attempts += 1

        self.get_logger().warn(f"Failed to align with ebox after {attempts} attempts or {timeout}s timeout")
        return False

    def move_to_safe_position(self):
        """1단계: 안전한 높은 위치로 이동 (틱값 하드코딩)"""
        # TODO: 실제 측정 후 안전한 틱값으로 교체
        safe_ticks = [
            204,   # J1: 베이스 중앙
            -4057,  # J2: 약간 앞으로 (안전 높이)
            3953,  # J3: 위쪽
            -268   # J4: 중립
        ]

        try:
            self.move_from_home_ticks(offset_j1=safe_ticks[0], offset_j2=safe_ticks[1], offset_j3=safe_ticks[2], offset_j4=safe_ticks[3])
            time.sleep(5.0)  # 이동 완료 대기
            self.get_logger().info("Moved to safe position")
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to move to safe position: {e}")
            return False

    def align_camera_center(self, left_dist, right_dist, threshold, max_attempts=10):
        """2단계: 베이스 회전으로 좌우 거리 차이 줄이기"""
        attempts = 0

        while attempts < max_attempts:
            distance_diff = abs(left_dist - right_dist)
            self.get_logger().info(f"Distance diff: {distance_diff:.3f}, threshold: {threshold}")

            if distance_diff <= threshold:
                self.get_logger().info("Camera centered successfully")
                return True

            # 베이스 회전 방향 결정
            if left_dist > right_dist:
                # 왼쪽이 더 멀면 시계방향 회전 (베이스 틱 증가)
                self.rotate_base_by_ticks(10)
                self.get_logger().info("Rotating base clockwise")
            else:
                # 오른쪽이 더 멀면 반시계방향 회전 (베이스 틱 감소)
                self.rotate_base_by_ticks(-10)
                self.get_logger().info("Rotating base counterclockwise")

            time.sleep(0.5)  # 회전 후 안정화
            attempts += 1

            # TODO: 여기서 새로운 left_dist, right_dist 값을 받아야 함
            # 현재는 고정값이므로 실제 구현 시 카메라에서 업데이트된 값 받기

        self.get_logger().warn(f"Failed to center camera after {max_attempts} attempts")
        return False

    def rotate_base_by_ticks(self, tick_offset):
        """베이스를 지정된 틱만큼 회전"""
        try:
            # 현재 베이스 위치 읽기
            current_pos, result, error = self.packet_ax.read2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_PRESENT_POS)

            if result == 0 and error == 0:
                new_pos = max(0, min(1023, current_pos + tick_offset))
                self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, new_pos)
                self.get_logger().info(f"Base moved from {current_pos} to {new_pos}")
            else:
                self.get_logger().error("Failed to read base position")

        except Exception as e:
            self.get_logger().error(f"Base rotation failed: {e}")

    def press_led_button_ticks(self):
        """3단계: 미리 정의된 틱값으로 버튼 누르기"""
        # TODO: 실제 측정 후 버튼 누르는 틱값으로 교체
        press_near_1_ticks = [
            -4057,  # J2: 버튼 위치로
            3953,  # J3: 버튼 높이로
            1232   # J4: 누르기 자세
        ]

        press_near_2_ticks = [
            -4545,  # J2: 버튼 위치로
            2953,  # J3: 버튼 높이로
            3482   # J4: 누르기 자세
        ]

        push_ticks = [
            -4545,  # J2: 살짝 더 앞으로 (누르기)
            2425,  # J3: 높이 유지
            3482   # J4: 자세 유지
        ]

        try:
            # 1. 버튼 앞 위치로 이동
            self.move_from_home_ticks(offset_j2=press_near_1_ticks[0], offset_j3=press_near_1_ticks[1], offset_j4=press_near_1_ticks[2])
            time.sleep(2.0)

            # 2 버튼 앞 위치로 이동 2
            self.move_from_home_ticks(offset_j2=press_near_2_ticks[0], offset_j3=press_near_2_ticks[1], offset_j4=press_near_2_ticks[2])
            time.sleep(2.0)

            # 3 버튼 누르기
            self.move_from_home_ticks(offset_j2=push_ticks[0], offset_j3=push_ticks[1], offset_j4=push_ticks[2])
            time.sleep(1.0)

            # 4 원위치
            self.move_from_home_ticks(offset_j2=press_near_1_ticks[0], offset_j3=press_near_1_ticks[1], offset_j4=press_near_1_ticks[2])
            time.sleep(1.0)

            self.get_logger().info("LED button pressed successfully")
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to press LED button: {e}")
            return False
    
    def open_door(self):
        self.move_ik(0.34, 0.0, 0.38)
        time.sleep(5.0)
        self.move_ik(0.34, 0.0, 0.3)
        return True



    # def pick_seq(self, p):
    #     # IK 작업 전에 DH 기준 자세로 복귀
    #     self.go_home_ticks()
    #     time.sleep(1.0)  # 이동 완료 대기

    #     self.gripper(False)
    #     # if not self.approach(p, dz=0.08): return False
    #     # down = p + np.array([0,0,-0.025])
    #     # if not self.move_lin(p, down, T=0.5): return False
    #     if not self.move_ik(0.34, 0.0, -0.1):
    #         self.get_logger().error("Failed to move to pick position")
    #         return False

    #     time.sleep(1.0)
    #     self.gripper(True)
    #     # lift = down + np.array([0,0,0.1])
    #     # return self.move_lin(down, lift, T=0.7)
    #     return True

    def place_seq(self, p):
        """박스 place 시퀀스 - 기본 위치로 이동 후 그리퍼 열기"""
        self.get_logger().info("박스 place 시퀀스 시작")
        # 홈으로 이동 후 그리퍼 열기
        self.go_home_ticks()
        time.sleep(1.0)
        self.gripper(False)
        self.get_logger().info("박스 place 시퀀스 완료")
        return True
    
    # 서비스 핸들러
    def handle_arm_do_task(self, request, response):
        x, y, z, task = request.x, request.y, request.z, request.task

        # # 카메라 좌표를 베이스 좌표로 변환
        # base_coord = self.camera_to_base(camera_coord)
        # self.get_logger().info(f"Camera coord: {camera_coord}, Base coord: {base_coord}")

        ok = False
        if task == 'pick':
            with self.state_lock:
                if self.active_task is not None:
                    response.success = False
                    response.message = f'busy: {self.active_task}'
                    return response
                self.active_task = 'pick'
            # 비동기 실행 → 스핀은 계속 돌아감 → 감지 콜백 수신 가능
            threading.Thread(target=self._run_pick_once, args=(x,y,z), daemon=True).start()
            ok = True
            response.success = ok
            response.message = 'started'
            return response
        elif task == 'enable_auto':
            self.enable_auto_mission = True
            ok = True
        elif task == 'disable_auto':
            self.enable_auto_mission = False
            ok = True
        elif task == 'press_button':
            self.get_logger().warn("press_button_seq function not implemented")
            ok = False
        elif task == 'open_door': ok = self.open_door()
        elif task == 'place': ok = self.place_seq([x, y, z])
        elif task == 'move': self.move_ik(x, y, z); ok = True
        elif task == 'go_home': self.go_home_ticks(); ok = True
        elif task == 'go_autonomous_driving': self.go_autonomous_driving_position(); ok = True
        elif task == 'go_ik_ready': self.go_ik_ready_position(); ok = True
        elif task == 'led_button_press':
            # 외부 색상 신호를 기다리는 시퀀스 실행
            ok = self.led_button_press_sequence()
        elif task == 'debug_ik':
            # IK 디버깅용
            result = self.debug_ik_without_limits(x, y, z)
            ok = result is not None
        elif task == 'move_from_home':
            # go_home_ticks 기준 오프셋 이동 (x=offset_j1, y=offset_j2, z=offset_j3)
            ok = self.move_from_home_ticks(int(x), int(y), int(z), 0)
        elif task == 'reset_ebox_aligned':
            # ebox 정렬 플래그 리셋
            self.ebox_aligned = False
            self.get_logger().info("Ebox alignment flag reset")
            ok = True
        else:
            # 미션 시스템으로 시도
            mission_handler = get_mission_handler(task, self)
            if mission_handler:
                ok = mission_handler.execute(x, y, z)
            else:
                response.success = False; response.message = f'Unknown task {task}'; return response
        response.success = bool(ok)
        response.message = 'done' if ok else 'failed'
        return response

    def _run_pick_once(self, x, y, z):
        """비동기 픽 실행 함수"""
        try:
            self.mission_in_progress = True
            # 자동 미션이 끼어들지 않도록 강제 OFF
            auto_prev = self.enable_auto_mission
            self.enable_auto_mission = False

            self.pick_seq([x, y, z])  # 내부에서 정렬 포함. 1회 수행

            # 쿨다운 타임스탬프 갱신(자동미션과 동일 정책 재사용)
            self.last_mission_time = time.time()
        except Exception as e:
            self.get_logger().error(f"pick_once failed: {e}")
        finally:
            self.mission_in_progress = False
            self.ebox_aligned = False
            self.enable_auto_mission = auto_prev
            with self.state_lock:
                self.active_task = None
            self.get_logger().info("pick one-shot finished")

    def _sleep(self, dt):
        """짧은 슬립으로 쪼개서 콜백 응답성 개선"""
        t0 = time.time()
        while time.time() - t0 < dt:
            time.sleep(0.02)

    def handle_detected_object(self, msg):
        """감지된 물체에 대해 자동으로 동작 수행"""
        object_name = msg.object_name.lower()

        # 최신 ebox 위치는 항상 갱신
        if object_name == "ebox":
            self.latest_ebox_detection = msg

        # 자동 미션이 꺼져 있으면 즉시 반환
        if not self.enable_auto_mission:
            return

        self.get_logger().info(f"Object detected: {object_name} at pixel ({msg.pixel_x}, {msg.pixel_y}), distance from center: ({msg.distance_from_center_x}, {msg.distance_from_center_y})")

        # 미션이 이미 실행 중이면 무시
        if self.mission_in_progress:
            return

        # 쿨다운 체크 (마지막 미션 완료 후 일정 시간 대기)
        current_time = time.time()
        if current_time - self.last_mission_time < self.mission_cooldown:
            return

        # 미션 핸들러 가져오기
        mission_handler = get_mission_handler(object_name, self)

        if not mission_handler:
            self.get_logger().warn(f"Unknown object/mission: {object_name}")
            self.get_logger().info(f"Available missions: {self.available_missions}")
            return

        # 미션 실행
        self.mission_in_progress = True
        self.get_logger().info(f"Starting mission for {object_name}")

        try:
            # DetectedObject 메시지 구조 변경으로 인한 수정
            # pixel_x, pixel_y를 x, y로 전달하고 z는 0으로 설정
            success = mission_handler.execute(msg.pixel_x, msg.pixel_y, 0.0)
            result = "SUCCESS" if success else "FAILED"
            self.get_logger().info(f"Mission {object_name}: {result}")
        except Exception as e:
            self.get_logger().error(f"Mission {object_name} failed with exception: {e}")
        finally:
            # 미션 완료 후 플래그 해제 및 쿨다운 시작
            self.mission_in_progress = False
            self.ebox_aligned = False  # ebox 정렬 플래그 초기화
            self.last_mission_time = time.time()  # 마지막 미션 완료 시간 기록
            self.get_logger().info(f"Mission for {object_name} completed, cooldown for {self.mission_cooldown}s")

    def execute_mission_by_name(self, mission_name, x, y, z):
        """미션 이름으로 직접 미션 실행 (테스트/디버깅용)"""
        mission_handler = get_mission_handler(mission_name, self)
        if not mission_handler:
            self.get_logger().error(f"Mission not found: {mission_name}")
            return False

        return mission_handler.execute(x, y, z)

    # def 

def main():
    rclpy.init()
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


