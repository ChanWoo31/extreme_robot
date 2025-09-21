# arm server
import numpy as np
import time
from ikpy.link import OriginLink, DHLink
from ikpy.chain import Chain

import rclpy
from rclpy.node import Node

from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite

from arm_interfaces.srv import ArmDoTask
from arm_interfaces.msg import DetectedObject

from .missions_robotarm import get_mission_handler, get_available_missions

d1 = 0
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
            DHLink(d=0, a=a4, alpha=0, theta=np.deg2rad(-90)),
        ]
        )
        self.z_offset = 0.0
        # 리미트!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # self.limits=[(-np.deg2rad(150), np.deg2rad(150)),
        #              (-np.deg2rad(5), np.deg2rad(180)),
        #              (-np.deg2rad(100), np.deg2rad(10)),
        #              (-np.deg2rad(100), np.deg2rad(10))]
        
        # 위에보다 더 보수적인 리미트.
        self.limits=[(-np.deg2rad(150), np.deg2rad(150)),
                     (-np.deg2rad(170), np.deg2rad(0)),
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
        self.camera_offset = np.array([-0.084, 0.0, 0.04155])  # 카메라가 엔드이펙터에서 +Z 방향으로 5cm
        self.camera_rotation = np.eye(3)  # 회전 없음 (필요시 수정)

    # 유틸

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
        tgt = 330 if close else 480 # 예시값 조정해야함. 왼쪽 숫자가 닫힘, 오른쪽이 열렸을 때.
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

    def write_goal_and_wait(self, dxl_id:int, tick:int, tol:int=8, timeout:float=2.0):
        self.packet_x.write4ByteTxRx(self.port, dxl_id, self.ADDR_GOAL_POSITION, int(tick))
        t0 = time.time()
        while time.time()-t0 < timeout:
            pos, res, err = self.packet_x.read4ByteTxRx(self.port, dxl_id, self.ADDR_PRESENT_POSITION)
            if res==0 and err==0 and abs(int(pos)-tick) <= tol:
                return True
            time.sleep(0.01)
        return False

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

    # IK / motion

    # def move_ik(self, x, y, z):
    #     ik = self.chain.inverse_kinematics(
    #         target_position=[-x, y, z],
    #         orientation_mode=None,
    #         initial_position=self.last_q,
    #     )
    #     q = list(ik[1:5])

    #     for i in range(4):
    #         lo, hi = self.limits[i]
    #         if q[i] < lo: q[i] = lo
    #         if q[i] > hi: q[i] = hi

    #     self.send_ax_base_deg(np.rad2deg(q[0]) * self.dir[0])
    #     self.send_x_positions(q[1], q[2] ,q[3])
    #     tmp = self.last_q.copy(); tmp[1:5] = q; self.last_q = tmp

    #     fk = self.chain.forward_kinematics(np.r_[0.0, q])  # 0+J1..J4
    #     reach = fk[:3, 3]
    #     err = np.linalg.norm(reach - np.array([x, y, z]))
    #     self.get_logger().info(f"FK reach=({reach[0]:.5f},{reach[1]:.5f},{reach[2]:.5f}), err={err*1000:.3f} mm")

    # def on_point(self, msg: Point):
    #     now = time.time()
    #     if now < self.block_until:
    #         self.get_logger().warn('throttle'); return
        
    #     x,y,z = msg.x, msg.y, msg.z
    #     # if max(abs(x), abs(y), abs(z)) > 5.0:
    #     #     x,y,z = x*0.001, y*0.001, z*0.001
    #     self.get_logger().info(f"Target (m): {x:.5f}, {y:.5f}, {z:.5f}")

    #     try:
    #         self.move_ik(x, y, z + self.z_offset)
    #     except Exception as e:
    #         self.get_logger().error(f"IK error: {e}")
    #         self.stop_hold()
    #     finally:
    #         self.block_until = time.time() + self.block_sec
    def solve_ik_q(self, x, y, z):
        # 여러 초기값으로 IK 시도하여 리미트 내 해 찾기
        initial_positions = [
            self.last_q,  # 현재 자세
            np.array([0, 0, 0, 0, 0]),  # 홈 포지션
            np.array([0, 0, np.deg2rad(90), 0, 0]),  # 다른 시작점
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
        return None
    
    def send_q(self, q):
        self.send_ax_base_deg(np.rad2deg(q[0]) * self.dir[0])
        self.send_x_positions(q[1], q[2] ,q[3])
        tmp = self.last_q.copy(); tmp[1:5] = q; self.last_q = tmp

    def move_ik(self, x, y, z):
        q = self.solve_ik_q(x, y, z)
        if q is None: return False
        self.send_q(q)
        time.sleep(self.dt)
        return True
    
    def move_lin(self, p_from, p_to, T=1.5, steps=20):
        for s in np.linspace(0, 1, steps):
            p = (1-s)*p_from + s*p_to
            if not self.move_ik(*p): return False
            time.sleep(T/steps)
        return True
    
    def fk_tip(self):
        T = self.chain.forward_kinematics(self.last_q)
        return T[:3, 3]

    def camera_to_base(self, camera_xyz):
        """카메라 좌표를 로봇 베이스 좌표계로 변환"""
        camera_xyz = np.array(camera_xyz, dtype=float)

        # 현재 엔드이펙터 위치와 자세 구하기
        T_base_to_ee = self.chain.forward_kinematics(self.last_q)
        ee_pos = T_base_to_ee[:3, 3]
        ee_rot = T_base_to_ee[:3, :3]

        # 카메라 좌표를 엔드이펙터 좌표계로 변환
        camera_in_ee = self.camera_rotation @ camera_xyz + self.camera_offset

        # 엔드이펙터 좌표계를 베이스 좌표계로 변환
        base_xyz = ee_rot @ camera_in_ee + ee_pos

        return base_xyz
    
    # 프리미티브(접근/이탈)

    def approach(self, p, dz=0.06):
        now = self.fk_tip()
        above = p + np.array([0,0,dz])
        return self.move_lin(now, above, T=1.0) and self.move_lin(above, p, T=0.8)
    
    def retreat(self, p, dz=0.06):
        up = p + np.array([0,0,dz])
        return self.move_lin(p, up, T=0.8)
    
    # 동작 시퀀스
    
    def press_button_seq(self, p):
        if not self.approach(p, dz=0.05): return False
        push = p + np.array([0.0, -0.018])
        if not self.move_lin(p, push, T=0.4): return False
        if not self.move_lin(push, p, T=0.4): return False
        return self.retreat(p, dz=0.05)
    
    # def open_handle_seq(self, p):

    def pick_seq(self, p):
        self.gripper(False)
        if not self.approach(p, dz=0.08): return False
        down = p + np.array([0,0,-0.025])
        if not self.move_lin(p, down, T=0.5): return False
        self.gripper(True)
        lift = down + np.array([0,0,0.1])
        return self.move_lin(down, lift, T=0.7)

    def place_seq(self, p):
        if not self.approach(p, dz=0.08): return False
        down = p + np.array([0,0,-0.025])
        if not self.move_lin(p, down, T=0.5): return False
        self.gripper(False)
        up = down + np.array([0,0,0.10])
        return self.move_lin(down, up, T=0.7)
    
    # 서비스 핸들러
    def handle_arm_do_task(self, request, response):
        x, y, z, task = request.x, request.y, request.z, request.task
        camera_coord = np.array([x, y, z], dtype=float)

        # 카메라 좌표를 베이스 좌표로 변환
        base_coord = self.camera_to_base(camera_coord)
        self.get_logger().info(f"Camera coord: {camera_coord}, Base coord: {base_coord}")

        ok = False
        if task == 'press_button': ok = self.press_button_seq(base_coord)
        elif task == 'open_handle': ok = self.turn_handle_seq(base_coord)
        elif task == 'pick': ok = self.pick_seq(base_coord)
        elif task == 'place': ok = self.place_seq(base_coord)
        elif task == 'move_to_coord': ok = self.move_ik(*base_coord)
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

    def handle_detected_object(self, msg):
        """감지된 물체에 대해 자동으로 동작 수행"""
        object_name = msg.object_name.lower()

        self.get_logger().info(f"Object detected: {object_name} at ({msg.x}, {msg.y}, {msg.z})")

        # 미션 핸들러 가져오기
        mission_handler = get_mission_handler(object_name, self)

        if not mission_handler:
            self.get_logger().warn(f"Unknown object/mission: {object_name}")
            self.get_logger().info(f"Available missions: {self.available_missions}")
            return

        # 미션 실행
        try:
            success = mission_handler.execute(msg.x, msg.y, msg.z)
            result = "SUCCESS" if success else "FAILED"
            self.get_logger().info(f"Mission {object_name}: {result}")
        except Exception as e:
            self.get_logger().error(f"Mission {object_name} failed with exception: {e}")

    def execute_mission_by_name(self, mission_name, x, y, z):
        """미션 이름으로 직접 미션 실행 (테스트/디버깅용)"""
        mission_handler = get_mission_handler(mission_name, self)
        if not mission_handler:
            self.get_logger().error(f"Mission not found: {mission_name}")
            return False

        return mission_handler.execute(x, y, z)


def main():
    rclpy.init()
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


