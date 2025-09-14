#groupsync version
import numpy as np
import time
from ikpy.link import OriginLink, DHLink
from ikpy.chain import Chain

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite

from arm_interfaces.srv import MoveArm

d1 = 0
a2, a3, a4 = 0.18525, 0.18525, 0.256

RAD2TICKS = 4096 / (2*np.pi)
AX_TICKS_PER_DEG = 1023.0 / 300.0
 
class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.srv = self.create_service(MoveArm, 'move_arm', self.handle_move_arm)
        
        self.DEVICENAME = '/dev/ttyUSB0'
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
            self.packet_x.write4ByteTxRx(self.port, dxl_id, self.ADDR_PROFILE_VELOCITY, 50)
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
                     (-np.deg2rad(5), np.deg2rad(180)),
                     (-np.deg2rad(100), np.deg2rad(10)),
                     (-np.deg2rad(100), np.deg2rad(10))]
        
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
        tgt = 820 if close else 520 # 예시값 조정해야함.
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
        q[0] = self.ticks_to_joint_rad(0, ax_pos)
        for j, dxl_id in enumerate(self.ids_x, start=1):  # j=1..3 → J2..J4
            pos, res, err = self.packet_x.read4ByteTxRx(self.port, dxl_id, self.ADDR_PRESENT_POSITION)
            if res==0 and err==0:
                q[j] = self.ticks_to_joint_rad(j, int(pos))
        self.last_q = np.zeros(len(self.chain.links))
        self.last_q[1:5] = q
        self.get_logger().info(f"Init last_q(deg)={np.rad2deg(self.last_q[1:5])}")

    def handle_move_arm(self, req, res):
        # 요청 좌표
        x, y, z = req.x, req.y, req.z
        self.get_logger().info(f"[srv] target: x={x:.4f}, y={y:.4f}, z={z:.4f}")

        # IK 계산
        ik = self.chain.inverse_kinematics(
            target_position=[-x, y, z],
            orientation_mode=None,
            initial_position=self.last_q,
        )
        q = list(ik[1:5])

        # 리미트 검사: 위반 시 동작하지 않음
        for i, (lo, hi) in enumerate(self.limits):
            if q[i] < lo or q[i] > hi:
                res.success = False
                res.message = f"limit hit at J{i+1}"
                return res

        # 모터 명령
        self.send_ax_base_deg(np.rad2deg(q[0]) * self.dir[0])
        self.send_x_positions(q[1], q[2], q[3])

        # 상태 갱신
        tmp = self.last_q.copy(); tmp[1:5] = q; self.last_q = tmp

        res.success = True
        res.message = "ok"
        return res

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

    def on_point(self, msg: Point):
        now = time.time()
        if now < self.block_until:
            self.get_logger().warn('throttle'); return
        
        x,y,z = msg.x, msg.y, msg.z
        # if max(abs(x), abs(y), abs(z)) > 5.0:
        #     x,y,z = x*0.001, y*0.001, z*0.001
        self.get_logger().info(f"Target (m): {x:.5f}, {y:.5f}, {z:.5f}")

        try:
            self.move_ik(x, y, z + self.z_offset)
        except Exception as e:
            self.get_logger().error(f"IK error: {e}")
            self.stop_hold()
        finally:
            self.block_until = time.time() + self.block_sec


        

def main():
    rclpy.init()
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


