# sequential control version
import numpy as np
import time
from ikpy.link import OriginLink, DHLink
from ikpy.chain import Chain

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from dynamixel_sdk import PortHandler, PacketHandler

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
        self.id_gripper = 5

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
        self.packet_ax.write2ByteTxRx(self.port, self.id_ax, self.AX_ADDR_MOVING_SPEED, 20)
        self.packet_ax.write1ByteTxRx(self.port,self.id_ax, self.AX_ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        
        for dxl_id in self.ids_x:
            # Extended
            self.packet_x.write1ByteTxRx(self.port, dxl_id, self.ADDR_OPERATING_MODE, 4)
            # current limit
            self.packet_x.write2ByteTxRx(self.port, dxl_id, self.ADDR_CURRENT_LIMIT, 150)
            # 가속/속도
            self.packet_x.write4ByteTxRx(self.port, dxl_id, self.ADDR_PROFILE_ACCEL, 10)
            self.packet_x.write4ByteTxRx(self.port, dxl_id, self.ADDR_PROFILE_VELOCITY, 20)
            # 필요시 Homing Offset (엔코더 0점을 기구학적 0으로 보전)
            # self.packet.write4ByteTxRx(self.port, self.ids, self.ADDR_HOMING_OFFSET, offset_ticks)

            # 토크 온
            self.packet_x.write1ByteTxRx(self.port, dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)

       


        self.get_logger().info('ArmController initialized')

        self.gear = [1.0, 7.0, 4.0, 4.0]
        self.dir = [1, -1, 1, 1]
        self.joint_to_id = {1:2, 2:3, 3:4}
        self.zero_ticks = {}
        for j in (1,2,3):
            dxl_id = self.joint_to_id[j]
            pos, res, err = self.packet_x.read4ByteTxRx(self.port, dxl_id, self.ADDR_PRESENT_POSITION)
            if res == 0 and err == 0:
                self.zero_ticks[dxl_id] = int(pos)
            else:
                self.zero_ticks[dxl_id] = 0
                self.get_logger().error(f"Zero capture failed for ID{dxl_id}")

        self.chain = Chain(name='arm4', links=[
            OriginLink(),
            DHLink(d=d1, a=0, alpha=np.deg2rad(-90), theta=0),
            DHLink(d=0, a=a2, alpha=0, theta=np.deg2rad(0)),
            DHLink(d=0, a=a3, alpha=0, theta=np.deg2rad(-90)),
            DHLink(d=0, a=a4, alpha=0, theta=np.deg2rad(-90)),
        ],
        active_links_mask=[False, True, True, True, True],
        )

        self.init_last_q_from_present()

        self.active_mask = [False, True, True, True, True]
        self.z_offset = 0.0
        # 리미트!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        self.limits=[(-np.deg2rad(150), np.deg2rad(150)),
                     (-np.deg2rad(180), np.deg2rad(0)),
                     (-np.deg2rad(10), np.deg2rad(100)),
                     (-np.deg2rad(10), np.deg2rad(100))]
        
        
        self.violation_latched = False
        self.block_until = 0.0
        self.block_sec = 1.0
        
        self.go_home()

    
    def x_write_goal(self, dxl_id:int, ticks:int):
        self.packet_x.write4ByteTxRx(self.port, dxl_id, self.ADDR_GOAL_POSITION, ticks)

    def wait_reach(self, dxl_id:int, target:int, tol:int=20, timeout:float=2.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            pos, res, err = self.packet_x.read4ByteTxRx(self.port, dxl_id, self.ADDR_PRESENT_POSITION)
            if res == 0 and err == 0 and abs(int(pos) - int(target)) <= tol:
                return True
            time.sleep(0.02)
        return False
    
    def init_last_q_from_present(self):
        q = [0.0]*4
        # J1 (AX)
        ax_pos, _, _ = self.packet_ax.read2ByteTxRx(self.port, self.id_ax, self.AX_ADDR_PRESENT_POS)
        q[0] = self.ticks_to_joint_rad(0, ax_pos)

        # J2~J4
        for j in (1,2,3):  # J2..J4
            dxl_id = self.joint_to_id[j]
            pos, res, err = self.packet_x.read4ByteTxRx(self.port, dxl_id, self.ADDR_PRESENT_POSITION)
            if res == 0 and err == 0:
                q[j] = self.ticks_to_joint_rad(j, int(pos))
        # IKPy full q벡터
        self.last_q = np.zeros(len(self.chain.links))
        self.last_q[1:1+4] = np.array(q)
        self.get_logger().info(f"Init last_q(deg) = {np.rad2deg(self.last_q[1:1+4])}")

    def joint_rad_to_motor_ticks(self, joint_idx, q_joint):
        # joint_idx: 0..3 (J1..J4)
        if joint_idx == 0:
            j1_deg = np.rad2deg(q_joint) * self.dir[0]
            return int(round((j1_deg/300.0)*1023.0)) + 512

        # X-series
        motor_rad = (q_joint * self.dir[joint_idx]) * self.gear[joint_idx]
        base = int(round(motor_rad * RAD2TICKS))

        dxl_id = self.joint_to_id[joint_idx]        # 1..3 → ID
        zero = self.zero_ticks.get(dxl_id, 0)
        return base + zero
    
    def ticks_to_joint_rad(self, joint_idx, ticks):
        if joint_idx == 0:
            deg = (ticks - 512) * (300.0/1023.0)
            return np.deg2rad(deg) * self.dir[0]

        dxl_id = self.joint_to_id[joint_idx]
        zero = self.zero_ticks.get(dxl_id, 0)
        motor = (ticks - zero) / RAD2TICKS
        joint = (motor / self.gear[joint_idx]) * self.dir[joint_idx]
        return joint

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
            target_position=[-x, y, z + self.z_offset],
            orientation_mode=None,
            initial_position= self.last_q,
            max_iter=1000,
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

        for j in (1,2,3):
            dxl_id = self.joint_to_id[j]
            t = self.joint_rad_to_motor_ticks(j, q[j])
            self.x_write_goal(dxl_id, t)

        if self.violation_latched and now >= self.block_until:
            self.violation_latched = False

        

    def stop_motion(self):
        ax_pos, _, _ = self.packet_ax.read2ByteTxRx(self.port, self.id_ax, self.AX_ADDR_PRESENT_POS)
        self.packet_ax.write2ByteTxRx(self.port, self.id_ax, self.AX_ADDR_GOAL_POSITION, ax_pos)

        # X-series (present position 4B)
        for dxl_id in self.ids_x:
            pos, res, err = self.packet_x.read4ByteTxRx(self.port, dxl_id, self.ADDR_PRESENT_POSITION)
            if res == 0 and err == 0:
                self.x_write_goal(dxl_id, int(pos))
        
    def go_home(self):
        home_q = [0.0, 0.0, 0.0, 0.0]

        j1_deg = np.rad2deg(home_q[0]) * self.dir[0]
        ax_ticks = int(round((j1_deg / 300.0) * 1023.0)) + 512
        ax_ticks = max(0, min(1023, ax_ticks))
        self.packet_ax.write2ByteTxRx(self.port, self.id_ax, self.AX_ADDR_GOAL_POSITION, ax_ticks)

        for j in (1,2,3):
            dxl_id = self.joint_to_id[j]
            t = self.joint_rad_to_motor_ticks(j, home_q[j])
            self.x_write_goal(dxl_id, t)

        self.get_logger().info("Moved to home position")

        

def main():
    rclpy.init()
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


