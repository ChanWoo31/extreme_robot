import numpy as np
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS, GroupSyncWrite
import time




DIRECTION = [ 1]

JOINT_IDS = [1]
JOINT_IDS = np.array(JOINT_IDS).astype(int)

DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116  # 4 byte
ADDR_MOVING_SPEED = 112  # 4 byte
ADDR_PRESENT_POSITION = 132  # 4 byte
ADDR_MX_TORQUE_LIMIT = 38
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

ADDR_OPERATING_MODE = 11
MODE_EXT_POSITION = 4

Moving_Speed = 32
z_offset = 43

def deg2dxl(deg: float) -> int:
    ang = ((deg + 180) % 360) - 180
    ratio = (ang + 180) / 360
    raw = int(ratio * 4096)
    return max(0, min(4096, raw))

def dxl2deg(raw: int) -> float:
    return raw / 4096 * 360 - 180

def deg2dxl_ext(deg: float) -> int:
    """
    확장 Position 모드용 변환.
    deg: 회전 각도 (예: 360→1회전, 1080→3회전)
    """
    # 1회전 = 4096 count
    raw = int(deg / 360.0 * 4096)
    return raw



class DXLController:
    def __init__(self, device=DEVICENAME, baud=BAUDRATE):
        self.port = PortHandler(device)
        if not self.port.openPort():
            raise IOError(f"Failed to open port {device}")
        if not self.port.setBaudRate(baud):
            raise IOError(f"Failed to set baudrate {baud}")
        self.packet = PacketHandler(PROTOCOL_VERSION)
        self.sync_write = GroupSyncWrite(self.port, self.packet, ADDR_GOAL_POSITION, 4)
        self.sync_speed = GroupSyncWrite(self.port, self.packet,
                                         ADDR_MOVING_SPEED, 4)

    def enable_torque(self, ids):
        for j in ids:
            self.packet.write1ByteTxRx(self.port, j, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def enable_torque_single(self, id):
        self.packet.write1ByteTxRx(self.port, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def disable_torque(self, ids):
        for j in ids:
            self.packet.write1ByteTxRx(self.port, j, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    def set_positions(self, deg_list):
        self.sync_write.clearParam()
        for idx, j in enumerate(JOINT_IDS):
            angle = deg_list[idx] * DIRECTION[idx]
            raw = deg2dxl(angle)
            # 4 바이트 little endian
            param = [
                raw & 0xFF,
                (raw >> 8) & 0xFF,
                (raw >> 16) & 0xFF,
                (raw >> 24) & 0xFF,
            ]   
            self.sync_write.addParam(j, bytes(param))
        self.sync_write.txPacket()
        if self.sync_write.txPacket() != COMM_SUCCESS:
            raise RuntimeError("SyncWrite error")

    def get_position(self, j):
        val, comm, err = self.packet.read2ByteTxRx(self.port, j, ADDR_PRESENT_POSITION)
        if comm != COMM_SUCCESS or err != 0:
            raise RuntimeError(f"Dxl error ID{j}")
        return dxl2deg(val)

    def close(self):
        self.port.closePort()

    def motor_speed(self, dxl_id, speed_val):
        self.packet.write2ByteTxRx(self.port, dxl_id, Moving_Speed, speed_val)

    def set_speed(self, speed):
        self.sync_speed.clearParam()
        for j in JOINT_IDS:
            # 4바이트 little endian으로 분할
            param = [
                speed        & 0xFF,
                (speed >> 8) & 0xFF,
                (speed >> 16)& 0xFF,
                (speed >> 24)& 0xFF,
            ]
            if not self.sync_speed.addParam(j, bytes(param)):
                raise RuntimeError(f"Speed addParam 실패 ID={j}")
        if self.sync_speed.txPacket() != COMM_SUCCESS:
            raise RuntimeError("Speed SyncWrite error")
        self.sync_speed.clearParam()

    def move_joint(self, joint_id: int, position_deg: float, speed: int = None):
        raw_pos = deg2dxl(position_deg)
        self.packet.write4ByteTxRx(self.port, joint_id, ADDR_GOAL_POSITION, raw_pos)

    def set_mode(self, mode: int):
        """Operating Mode(1byte)를 한 번에 바꿉니다."""
        for j in JOINT_IDS:
            self.packet.write1ByteTxRx(self.port, j,
                                       ADDR_OPERATING_MODE,
                                       mode)    
    
    def move_joint_ext(self, joint_id: int, position_deg: float, speed: int=None):
        raw_pos = deg2dxl_ext(position_deg)
        # Goal Position(4바이트)으로 직접 쓰기
        self.packet.write4ByteTxRx(self.port, joint_id,
                                   ADDR_GOAL_POSITION,
                                   raw_pos)
    




if __name__ == "__main__":

    dxl = DXLController()
    dxl.set_mode(MODE_EXT_POSITION)        # Extended 모드
    dxl.enable_torque(JOINT_IDS)           # 토크 켜기

    # 1회전(360°), 2회전, 3회전 각각 테스트
    for rev in [1, 2, 3]:
        dxl.move_joint_ext(JOINT_IDS[0], 360 * rev)
        time.sleep(3)

    dxl.disable_torque(JOINT_IDS)
    dxl.close()