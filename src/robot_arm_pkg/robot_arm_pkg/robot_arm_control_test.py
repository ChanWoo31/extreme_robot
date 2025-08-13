#groupsync version
import numpy as np
import time
from ikpy.link import OriginLink, DHLink
from ikpy.chain import Chain

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite

d1 = 0
a2, a3, a4 = 0.28525, 0.19025, 0.256

TICKS_PER_REV = 4096
RAD2TICKS = TICKS_PER_REV / (2*np.pi)
AX_TICKS_PER_DEG = 1023.0 / 300.0

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.sub = self.create_subscription(
            Point, 'target_point', self.on_point, 10
        )
        
        self.DEVICENAME = '/dev/ttyUSB0'
        self.BAUDRATE = 1000000
        self.id_ax = 1
        self.ids_x = [2, 3, 4]

        # Control table(protocol 1.0)
        self.AX_ADDR_TORQUE_ENABLE = 24
        self.AX_ADDR_GOAL_POSITION = 30
        self.AX_ADDR_MOVING_SPEED = 32
        self.AX_ADDR_PRESENT_POS = 36

        # Control table(protocol 2.0)
        self.ADDR_OPERATING_MODE = 11
        self.ADDR_HOMING_OFFSET = 20
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
        self.packet_ax.write1ByteTxRx(self.port, self.id_ax, self.AX_ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        self.packet_ax.write2ByteTxRx(self.port, self.id_ax, self.AX_ADDR_MOVING_SPEED, 100)
        self.packet_ax.write1ByteTxRx(self.port,self.id_ax, self.AX_ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        
        for dxl_id in self.ids_x:
            # Extended
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

        self.gear = [1.0, 7.0, 4.0, 4.0]
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

        self.chain = Chain(name='arm4', links=[
            OriginLink(),
            DHLink(d=d1, a=0, alpha=np.deg2rad(-90), theta=0),
            DHLink(d=0, a=a2, alpha=0, theta=np.deg2rad(0)),
            DHLink(d=0, a=a3, alpha=0, theta=np.deg2rad(-90)),
            DHLink(d=0, a=a4, alpha=0, theta=np.deg2rad(-90)),
        ])
        self.z_offset = 0.0
        # 리미트!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        self.limits=[(-np.deg2rad(150), np.deg2rad(150)),
                     (-np.deg2rad(180), np.deg2rad(0)),
                     (-np.deg2rad(10), np.deg2rad(100)),
                     (-np.deg2rad(10), np.deg2rad(100))]
        
        #잘 되는지 테스트 필요
        self.last_q = np.zeros(len(self.chain.links))
        self.violation_latched = False
        self.block_until = 0.0
        self.block_sec = 1.0
        
        self.go_home()

    def int32_to_le(self, v:int):
        v &= 0xFFFFFFFF
        return bytes([v & 0xFF, (v>>8)&0xFF, (v>>16)&0xFF, (v>>24)&0xFF])

    def joint_rad_to_motor_ticks(self, i, q_joint):
        q_motor = (q_joint * self.dir[i]) * self.gear[i]
        ticks = int(round(q_motor * RAD2TICKS)) + self.zero[i]
        return max(-2_147_483_648, min(2_147_483_647, ticks))

    def on_point(self, msg: Point):
        now = time.time()
        if self.violation_latched and now < self.block_until:
            self.get_logger().warn("Limit latch: 명령 무시")
            return

        x, y, z = msg.x, msg.y, msg.z
        if max(abs(x), abs(y), abs(z)) > 2.0:
            x, y, z = x*0.001, y*0.001, z*0.001
        self.get_logger().info(f'Received target: x={x:.2f}, y={y:.2f}, z={z:.2f}')
        
        ik = self.chain.inverse_kinematics(
            target_position=[x, y, z + self.z_offset],
            orientation_mode=None,
            target_orientation=[0, 0, 1],
            initial_position= self.last_q,
        )
        self.last_q = ik
        q = list(ik[1:1+4]) # rad
        # 소프트 제한
        clamped = [False]*4
        for i in range(4):
            lo, hi = self.limits[i]
            if q[i] < lo:
                self.get_logger().warn(
                    f"Joint {i+1} 하한 제한 적용: {np.rad2deg(q[i]):.2f} -> {np.rad2deg(lo):.2f}"
                )
                q[i] = lo
                clamped[i] = True
            elif q[i] > hi:
                self.get_logger().warn(
                    f"Joint {i+1} 상한 제한 적용: {np.rad2deg(q[i]):.2f} -> {np.rad2deg(hi):.2f}"
                )
                q[i] = hi
                clamped[i] = True

        if any(clamped):
            self.get_logger().error(f"리밋 위반 -> 즉시 정지 + 라치 ON")
            self.stop_motion()
            self.violation_latched = True
            self.block_until = now + self.block_sec
            tmp = self.last_q.copy()
            tmp[1:1+4] = q
            self.last_q = tmp
            return
        
        tmp = self.last_q.copy()
        tmp[1:1+4] = q
        self.last_q = tmp
        
        # J1: AX-12A (fkeldks -> 도 -> 0~1023)
        j1_deg = np.rad2deg(q[0]) * self.dir[0]
        ax_ticks = int(round((j1_deg / 300.0) * 1023.0)) + 512
        ax_ticks = max(0, min(1023, ax_ticks))
        self.packet_ax.write2ByteTxRx(self.port, self.id_ax, self.AX_ADDR_GOAL_POSITION, ax_ticks)

        # J2-J4 (라디안 -> 멀티턴 틱)
        ticks = [self.joint_rad_to_motor_ticks(i, q[i]) for i in range(1, 4)]

        self.sync_write.clearParam()
        for dxl_id, t in zip(self.ids_x, ticks):
            self.sync_write.addParam(dxl_id, self.int32_to_le(t))
        self.sync_write.txPacket()

        if self.violation_latched and now >= self.block_until:
            self.violation_latched = False

        

    def stop_motion(self):
        ax_pos, _, _ = self.packet_ax.read2ByteTxRx(self.port, self.id_ax, self.AX_ADDR_PRESENT_POS)
        self.packet_ax.write2ByteTxRx(self.port, self.id_ax, self.AX_ADDR_GOAL_POSITION, ax_pos)

        # X-series (present position 4B)
        self.sync_write.clearParam()
        for dxl_id in self.ids_x:
            pos, res, err = self.packet_x.read4ByteTxRx(self.port, dxl_id, self.ADDR_PRESENT_POSITION)
            if res == 0 and err == 0:
                self.sync_write.addParam(dxl_id, self.int32_to_le(int(pos)))
        self.sync_write.txPacket()
        
    def go_home(self):
        home_q = [0.0, 0.0, 0.0, 0.0]

        j1_deg = np.rad2deg(home_q[0]) * self.dir[0]
        ax_ticks = int(round((j1_deg / 300.0) * 1023.0)) + 512
        ax_ticks = max(0, min(1023, ax_ticks))
        self.packet_ax.write2ByteTxRx(self.port, self.id_ax, self.AX_ADDR_GOAL_POSITION, ax_ticks)

        # J2-J4
        ticks = [self.joint_rad_to_motor_ticks(i, home_q[i]) for i in range(1, 4)]
        self.sync_write.clearParam()
        for dxl_id, t in zip(self.ids_x, ticks):
            self.sync_write.addParam(dxl_id, self.int32_to_le(t))
        self.sync_write.txPacket()

        self.get_logger().info("Moved to home position")

        

def main():
    rclpy.init()
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


