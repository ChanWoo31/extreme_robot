#!/usr/bin/env python3

import numpy as np
import time
from ikpy.link import OriginLink, DHLink
from ikpy.chain import Chain

import rclpy
from rclpy.node import Node

from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite

# DH 파라미터
d1 = 0
a2, a3, a4 = 0.18525, 0.18525, 0.256

RAD2TICKS = 4096 / (2*np.pi)
AX_TICKS_PER_DEG = 1023.0 / 300.0

class ArmTester(Node):
    def __init__(self):
        super().__init__('arm_tester')

        self.DEVICENAME = '/dev/ttyUSB1'
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

            # 토크 온
            self.packet_x.write1ByteTxRx(self.port, dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)

        self.sync_write = GroupSyncWrite(
            self.port, self.packet_x,
            self.ADDR_GOAL_POSITION, 4
        )

        self.get_logger().info('ArmTester initialized')

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

        # 역기구학 체인
        self.chain = Chain(name='arm4', links=[
            OriginLink(),
            DHLink(d=d1, a=0, alpha=np.deg2rad(90), theta=0),
            DHLink(d=0, a=a2, alpha=0, theta=np.deg2rad(0)),
            DHLink(d=0, a=a3, alpha=0, theta=np.deg2rad(90)),
            DHLink(d=0, a=a4, alpha=0, theta=np.deg2rad(90)),
        ])

        # 리미트
        self.limits=[(-np.deg2rad(150), np.deg2rad(150)),
                     (-np.deg2rad(0), np.deg2rad(170)),
                     (-np.deg2rad(90), np.deg2rad(5)),
                     (-np.deg2rad(100), np.deg2rad(5))]

        self.init_last_q_from_present()
        self.dt = 0.02 # 50Hz

    # 유틸 함수들
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

    def ticks_to_joint_rad(self, joint_idx, ticks):
        if joint_idx == 0:  # AX-12A J1
            deg = (ticks - 512) * (300.0/1023.0)
            return np.deg2rad(deg) * self.dir[0]
        zero = self.zero[joint_idx]
        motor = (ticks - zero) / RAD2TICKS
        return (motor / self.gear[joint_idx]) * self.dir[joint_idx]

    def init_last_q_from_present(self):
        q = [0.0]*4
        ax_pos,_,_ = self.packet_ax.read2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_PRESENT_POS)

        # 홈 틱을 현재 위치로 저장
        self.base_home_tick = ax_pos
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
        self.get_logger().info(f"Home ticks: Base={self.base_home_tick}, XL={self.xl_home_ticks}")

    # dynamixel_manual_mode.py와 동일한 IK 함수
    def move_ik_original(self, x, y, z):
        """Original IK from dynamixel_manual_mode.py"""
        ik = self.chain.inverse_kinematics(
            target_position=[-x, y, z],
            orientation_mode=None,
            initial_position=self.last_q,
        )
        q = list(ik[1:5])

        # 리미트 클램핑 (manual_mode 방식)
        for i in range(4):
            lo, hi = self.limits[i]
            if q[i] < lo: q[i] = lo
            if q[i] > hi: q[i] = hi

        self.send_ax_base_deg(np.rad2deg(q[0]) * self.dir[0])
        self.send_x_positions(q[1], q[2] ,q[3])
        tmp = self.last_q.copy(); tmp[1:5] = q; self.last_q = tmp

        # FK 검증
        fk = self.chain.forward_kinematics(np.r_[0.0, q])  # 0+J1..J4
        reach = fk[:3, 3]
        err = np.linalg.norm(reach - np.array([-x, y, z]))  # target_position과 동일한 형태로 비교
        self.get_logger().info(f"FK reach=({reach[0]:.5f},{reach[1]:.5f},{reach[2]:.5f}), err={err*1000:.3f} mm")
        return True

    # 수정된 IK 함수 (여러 초기값 시도)
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
        if q is None:
            self.get_logger().error(f"Cannot reach target ({x:.3f}, {y:.3f}, {z:.3f})")
            return False
        self.send_q(q)
        time.sleep(self.dt)
        self.get_logger().info(f"Moved to ({x:.3f}, {y:.3f}, {z:.3f}) with joints: {np.rad2deg(q)}")
        return True

    def go_home_ticks(self):
        """저장된 홈 틱 위치로 복귀"""
        # 베이스 AX 복귀
        self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, int(self.base_home_tick))

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

        self.get_logger().info("Returned to home position")

    def test_coordinate(self, x, y, z):
        """베이스 좌표로 이동 테스트"""
        self.get_logger().info(f'Testing coordinate: ({x}, {y}, {z})')
        success = self.move_ik(x, y, z)
        result_msg = "SUCCESS" if success else "FAILED"
        self.get_logger().info(f'Test Result: {result_msg}')
        return success

    def test_coordinate_original(self, x, y, z):
        """Original manual_mode 방식으로 테스트"""
        self.get_logger().info(f'Testing coordinate (ORIGINAL): ({x}, {y}, {z})')
        success = self.move_ik_original(x, y, z)
        self.get_logger().info(f'Original Test Result: SUCCESS')
        return success

    def test_joint_direct(self, joint_id, angle_deg):
        """특정 조인트를 직접 각도로 움직여서 테스트 (DISABLED FOR SAFETY)"""
        self.get_logger().error("Joint direct movement DISABLED after motor damage!")
        self.get_logger().error("Use 'home' to return to safe position only.")
        return

def main(args=None):
    rclpy.init(args=args)

    try:
        tester = ArmTester()
    except Exception as e:
        print(f"Failed to initialize ArmTester: {e}")
        rclpy.shutdown()
        return

    print("\n=== ARM IK DIRECT TEST ===")
    print("Commands:")
    print("  coord x y z       - Move to coordinate (modified IK - strict limits)")
    print("  orig x y z        - Move to coordinate (original IK - clamping)")
    print("  home              - Go to home position")
    print("  quit              - Exit")
    print("Example: coord 0.3 0.1 0.2 or orig 0.3 0.1 0.2")
    print("WARNING: Joint direct control disabled for safety")
    print("========================================\n")

    try:
        while True:
            user_input = input("Enter command: ").strip()

            if not user_input:
                continue

            if user_input.lower() in ['quit', 'exit', 'q']:
                break

            parts = user_input.split()
            if len(parts) < 1:
                continue

            command = parts[0].lower()

            if command == 'home':
                tester.go_home_ticks()

            elif command in ['coord', 'orig']:
                if len(parts) != 4:
                    print("Error: Need 3 coordinates (x y z)")
                    continue

                try:
                    x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                except ValueError:
                    print("Error: Coordinates must be numbers")
                    continue

                if command == 'coord':
                    success = tester.test_coordinate(x, y, z)
                else:  # orig
                    success = tester.test_coordinate_original(x, y, z)

            elif command == 'joint':
                if len(parts) != 3:
                    print("Error: Need joint number and angle (joint J angle)")
                    continue

                try:
                    joint_id = int(parts[1])
                    angle = float(parts[2])
                except ValueError:
                    print("Error: Joint ID must be integer, angle must be number")
                    continue

                if joint_id not in [1, 2, 3, 4]:
                    print("Error: Joint ID must be 1, 2, 3, or 4")
                    continue

                tester.test_joint_direct(joint_id, angle)

            else:
                print("Unknown command. Use: coord, orig, joint, home, quit")

    except KeyboardInterrupt:
        print("\nExiting...")

    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
