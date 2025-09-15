import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite

import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, DHLink



# ---- 공통/상수 ----
PROTOCOL_1 = 1.0   # AX-12A
PROTOCOL_2 = 2.0   # XL430

# AX-12A (Prot 1.0)
AX_ADDR_TORQUE_ENABLE = 24
AX_ADDR_GOAL_POSITION = 30   # 2B
AX_ADDR_MOVING_SPEED  = 32   # 2B

# XL430 (Prot 2.0)
XL_ADDR_OP_MODE       = 11
XL_ADDR_TORQUE_ENABLE = 64
XL_ADDR_PROFILE_ACC   = 108
XL_ADDR_PROFILE_VEL   = 112
XL_ADDR_GOAL_POSITION = 116
XL_OP_POSITION        = 3
XL_TICKS_PER_REV      = 4096

# RAD2TICKS = 4096 / (2*np.pi)


def ax_deg2tick(deg: float)->int:
    return max(0, min(1023, int(round(deg * 1023.0 / 300.0))))

def xl_deg2tick(deg: float)->int:
    return max(0, min(4095, int(round(deg * XL_TICKS_PER_REV / 360.0))))

class DynamixelManualMode(Node):
    def __init__(self):
        super().__init__('dynamixel_manual_mode')

        # ---- 파라미터 ----
        self.port_name   = self.declare_parameter('port', '/dev/ttyUSB0').get_parameter_value().string_value
        self.baud        = self.declare_parameter('baud', 1000000).get_parameter_value().integer_value
        # IDs
        self.ax_lift_id  = self.declare_parameter('ax_lift_id', 6).get_parameter_value().integer_value
        self.ax_grip_id  = self.declare_parameter('ax_grip_id', 5).get_parameter_value().integer_value
        self.xl_ids      = list(self.declare_parameter('xl_ids', [1,2,3,4]).get_parameter_value().integer_array_value)
        # 동작 파라미터
        self.lift_step_deg   = self.declare_parameter('lift_step_deg', 90.0).get_parameter_value().double_value
        self.arm_step_deg    = self.declare_parameter('arm_step_deg', 3.0).get_parameter_value().double_value
        self.grip_open_tick  = max(0, min(1023, self.declare_parameter('grip_open_tick', 850).get_parameter_value().integer_value))
        self.grip_close_tick = max(0, min(1023, self.declare_parameter('grip_close_tick', 200).get_parameter_value().integer_value))
        self.xl_profile_vel  = self.declare_parameter('xl_profile_vel', 80).get_parameter_value().integer_value
        self.xl_profile_acc  = self.declare_parameter('xl_profile_acc', 30).get_parameter_value().integer_value
        self.xl_neutral      = list(self.declare_parameter('xl_neutral_ticks', [2048,2048,2048,2048]).get_parameter_value().integer_array_value)
        self.ax_lift_center  = self.declare_parameter('ax_lift_center_deg', 150.0).get_parameter_value().double_value

        # ---- 포트/패킷 핸들러 ----
        self.port   = PortHandler(self.port_name)
        self.pk1    = PacketHandler(PROTOCOL_1)  # AX
        self.pk2    = PacketHandler(PROTOCOL_2)  # XL
        if not self.port.openPort() or not self.port.setBaudRate(self.baud):
            raise RuntimeError('Failed to open/set baud on port')

        # ---- AX(리프트/그리퍼) 초기화 ----
        self._ax_write1(self.ax_lift_id, AX_ADDR_TORQUE_ENABLE, 0)
        self._ax_write1(self.ax_grip_id, AX_ADDR_TORQUE_ENABLE, 0)
        self._ax_write2(self.ax_lift_id, AX_ADDR_MOVING_SPEED, 200)  # 보수적 속도
        self._ax_write2(self.ax_grip_id, AX_ADDR_MOVING_SPEED, 150)
        self._ax_write1(self.ax_lift_id, AX_ADDR_TORQUE_ENABLE, 1)
        self._ax_write1(self.ax_grip_id, AX_ADDR_TORQUE_ENABLE, 1)

        self.ax_lift_goal = ax_deg2tick(self.ax_lift_center)
        self._ax_write2(self.ax_lift_id, AX_ADDR_GOAL_POSITION, self.ax_lift_goal)

        # ---- XL(4DoF 팔) 초기화 ----
        for mid in self.xl_ids:
            self._xl_write1(mid, XL_ADDR_TORQUE_ENABLE, 0)
            self._xl_write1(mid, XL_ADDR_OP_MODE, XL_OP_POSITION)
            self._xl_write4(mid, XL_ADDR_PROFILE_VEL, self.xl_profile_vel)
            self._xl_write4(mid, XL_ADDR_PROFILE_ACC, self.xl_profile_acc)
            self._xl_write1(mid, XL_ADDR_TORQUE_ENABLE, 1)

        self.xl_goals = list(self.xl_neutral)
        self.xl_sync  = GroupSyncWrite(self.port, self.pk2, XL_ADDR_GOAL_POSITION, 4)
        self._xl_sync_write_all()

        # ---- 구독(매뉴얼 토픽) ----
        self.create_subscription(String, '/lift/cmd/manual',    self._on_lift,    20)
        self.create_subscription(String, '/arm/cmd/manual',     self._on_arm,     50)
        self.create_subscription(String, '/gripper/cmd/manual', self._on_gripper, 20)

        # 타이머: 50Hz로 변경사항 쓰기(안정적 I/O)
        self.timer = self.create_timer(0.02, self._flush)

        # 변경 플래그
        self._dirty_ax_lift   = True
        self._dirty_ax_grip   = False
        self._dirty_xl_arm    = True

    # ---------- 콜백 ----------
    def _on_lift(self, msg: String):
        step = ax_deg2tick(self.lift_step_deg)
        c = msg.data.strip().lower()
        if c == 'down':
            self.ax_lift_goal = 0
            self._dirty_ax_lift = True
        elif c == 'up':
            self.ax_lift_goal = 1023
            self._dirty_ax_lift = True
        elif c == 'center':
            self.ax_lift_goal = 512
            self._dirty_ax_lift = True

    def _on_gripper(self, msg: String):
        c = msg.data.strip().lower()
        if c == 'open':
            self.ax_grip_goal = self.grip_open_tick
            self._dirty_ax_grip = True
        elif c == 'close':
            self.ax_grip_goal = self.grip_close_tick
            self._dirty_ax_grip = True

    def _on_arm(self, msg: String):
        s = xl_deg2tick(self.arm_step_deg)
        c = msg.data.strip().lower()
        # J1 yaw: left/right
        if c == 'left':
            self.xl_goals[0] = max(0, min(4095, self.xl_goals[0] + s))
        elif c == 'right':
            self.xl_goals[0] = max(0, min(4095, self.xl_goals[0] - s))
        # J2/J3 pitch 조합
        elif c == 'up':
            self.xl_goals[1] = max(0, min(4095, self.xl_goals[1] - s))
            self.xl_goals[2] = max(0, min(4095, self.xl_goals[2] + s))
        elif c == 'down':
            self.xl_goals[1] = max(0, min(4095, self.xl_goals[1] + s))
            self.xl_goals[2] = max(0, min(4095, self.xl_goals[2] - s))
        elif c == 'front':
            self.xl_goals[1] = max(0, min(4095, self.xl_goals[1] - s))
            self.xl_goals[2] = max(0, min(4095, self.xl_goals[2] - s))
        elif c == 'back':
            self.xl_goals[1] = max(0, min(4095, self.xl_goals[1] + s))
            self.xl_goals[2] = max(0, min(4095, self.xl_goals[2] + s))
        else:
            return
        self._dirty_xl_arm = True

    # ---------- 주기적 하드웨어 쓰기 ----------
    def _flush(self):
        # AX lift
        if getattr(self, 'ax_lift_goal', None) is not None and self._dirty_ax_lift:
            self._ax_write2(self.ax_lift_id, AX_ADDR_GOAL_POSITION, int(self.ax_lift_goal))
            self._dirty_ax_lift = False
        # AX gripper
        if getattr(self, 'ax_grip_goal', None) is not None and self._dirty_ax_grip:
            self._ax_write2(self.ax_grip_id, AX_ADDR_GOAL_POSITION, int(self.ax_grip_goal))
            self._dirty_ax_grip = False
        # XL arm (GroupSyncWrite)
        if self._dirty_xl_arm:
            self._xl_sync_write_all()
            self._dirty_xl_arm = False

    # ---------- 로우레벨 I/O ----------
    def _ax_write1(self, mid, addr, val):
        r,e = self.pk1.write1ByteTxRx(self.port, mid, addr, int(val))
        if r != 0 or e != 0: self.get_logger().warn(f'AX id{mid} w1 a{addr}: {r},{e}')

    def _ax_write2(self, mid, addr, val):
        r,e = self.pk1.write2ByteTxRx(self.port, mid, addr, int(val))
        if r != 0 or e != 0: self.get_logger().warn(f'AX id{mid} w2 a{addr}: {r},{e}')

    def _xl_write1(self, mid, addr, val):
        r,e = self.pk2.write1ByteTxRx(self.port, mid, addr, int(val))
        if r != 0 or e != 0: self.get_logger().warn(f'XL id{mid} w1 a{addr}: {r},{e}')

    def _xl_write4(self, mid, addr, val):
        r,e = self.pk2.write4ByteTxRx(self.port, mid, addr, int(val))
        if r != 0 or e != 0: self.get_logger().warn(f'XL id{mid} w4 a{addr}: {r},{e}')

    def _xl_sync_write_all(self):
        self.xl_sync.clearParam()
        for mid, tgt in zip(self.xl_ids, self.xl_goals):
            b = [tgt & 0xFF, (tgt>>8)&0xFF, (tgt>>16)&0xFF, (tgt>>24)&0xFF]
            if not self.xl_sync.addParam(mid, bytes(b)):
                self.get_logger().error(f'GroupSyncWrite addParam failed id {mid}')
        if self.xl_sync.txPacket() != 0:
            self.get_logger().warn('GroupSyncWrite txPacket failed')

def main():
    rclpy.init()
    node = DynamixelManualMode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()



