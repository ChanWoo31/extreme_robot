#!/usr/bin/env python3
import numpy as np
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from arm_interfaces.srv import ArmDoTask
from arm_interfaces.msg import DetectedObject

from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite
from ikpy.link import OriginLink, DHLink
from ikpy.chain import Chain

from .missions_robotarm import get_mission_handler, get_available_missions

# ===== 공통 상수 =====
RAD2TICKS = 4096 / (2*np.pi)
AX_TICKS_PER_DEG = 1023.0 / 300.0
d1 = 0.0
a2, a3, a4 = 0.18525, 0.18525, 0.256

class UnifiedArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        # ---------- 취소 이벤트 ----------
        self.cancel_event = threading.Event()

        # QoS (센서용)
        qos_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)

        # ---------- 서비스 (auto에서 사용) ----------
        self.srv = self.create_service(ArmDoTask, 'arm_do_task', self.handle_arm_do_task)

        # ---------- 구독 (auto 경로) ----------
        self.detected_sub = self.create_subscription(DetectedObject, 'detected_objects',
                                                     self.handle_detected_object, qos_sensor)
        self.direction_subscriber = self.create_subscription(String, '/direction',
                                                             self.direction_callback, qos_sensor)

        # ---------- 매뉴얼 명령 ----------
        self.sub_manual = self.create_subscription(String, '/arm/manual',
                                                   self.on_manual_command, 50)

        # ---------- 상태 ----------
        self.current_direction = None
        self.latest_ebox_detection = None
        self.mission_in_progress = False
        self.active_task = None
        self.state_lock = threading.Lock()
        self.enable_auto_mission = False
        self.last_mission_time = 0.0
        self.mission_cooldown = 10.0
        self.color_tracking_active = False
        self.ebox_aligned = False

        # 미션 시스템 초기화
        self.available_missions = get_available_missions()
        self.get_logger().info(f"Available missions: {self.available_missions}")

        # ---------- 하드웨어 ----------
        self.DEVICENAME = '/dev/ttyROBOTARM'
        self.BAUDRATE = 1000000
        self.ax_base_id = 1
        self.ax_grip_id = 5
        self.ids_x = [2, 3, 4]

        # manual 전용 추가 축/파라미터
        self.x_lift_id = self.declare_parameter('x_lift_id', 6).get_parameter_value().integer_value
        self.x_lift_up_tick = self.declare_parameter('x_lift_up_tick', 5461).get_parameter_value().integer_value
        self.x_lift_center_tick = self.declare_parameter('x_lift_center_tick', 2048).get_parameter_value().integer_value
        self.x_lift_down_tick = self.declare_parameter('x_lift_down_tick', -1365).get_parameter_value().integer_value
        self.x_lift_up_tick_little = self.declare_parameter('x_lift_up_tick_little', 3755).get_parameter_value().integer_value
        self.x_lift_down_tick_little = self.declare_parameter('x_lift_down_tick_little', 0).get_parameter_value().integer_value

        self.xl_back_id  = self.declare_parameter('xl_back_id', 7).get_parameter_value().integer_value
        self.back_up_tick     = self.declare_parameter('back_up_tick',    -2560).get_parameter_value().integer_value
        self.back_center_tick = self.declare_parameter('back_center_tick', 0).get_parameter_value().integer_value
        self.back_down_tick   = self.declare_parameter('back_down_tick',  5120).get_parameter_value().integer_value
        self.back_down_45_tick= self.declare_parameter('back_down_45_tick', 2560).get_parameter_value().integer_value

        self.grip_open_tick   = self.declare_parameter('grip_open_tick', 480).get_parameter_value().integer_value
        self.grip_close_tick  = self.declare_parameter('grip_close_tick', 220).get_parameter_value().integer_value
        self.grip_close_fully_tick = self.declare_parameter('grip_close_fully_tick', 10).get_parameter_value().integer_value
        self.base_step_deg = self.declare_parameter('base_step_deg', 5.0).get_parameter_value().double_value

        # --- Dynamixel reg ---
        self.AX_ADDR_TORQUE_ENABLE = 24
        self.AX_ADDR_GOAL_POSITION = 30
        self.AX_ADDR_MOVING_SPEED  = 32
        self.AX_ADDR_PRESENT_POS   = 36
        self.ADDR_OPERATING_MODE   = 11
        self.ADDR_CURRENT_LIMIT    = 38
        self.ADDR_TORQUE_ENABLE    = 64
        self.ADDR_PROFILE_ACCEL    = 108
        self.ADDR_PROFILE_VELOCITY = 112
        self.ADDR_GOAL_POSITION    = 116
        self.ADDR_PRESENT_POSITION = 132
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0

        # 포트/패킷
        self.port = PortHandler(self.DEVICENAME)
        if not self.port.openPort():
            self.get_logger().error('Failed to open port'); return
        if not self.port.setBaudRate(self.BAUDRATE):
            self.get_logger().error('Failed to set baudrate'); return
        self.packet_ax = PacketHandler(1.0)
        self.packet_x  = PacketHandler(2.0)

        # 초기화 (AX / X / Lift / Back)
        for ax_id in [i for i in [self.ax_base_id, self.ax_grip_id] if i is not None]:
            self.packet_ax.write1ByteTxRx(self.port, ax_id, self.AX_ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            self.packet_ax.write2ByteTxRx(self.port, ax_id, self.AX_ADDR_MOVING_SPEED, 100)
            self.packet_ax.write1ByteTxRx(self.port, ax_id, self.AX_ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)

        for x_id in [*self.ids_x, self.x_lift_id, self.xl_back_id]:
            if x_id is None: continue
            self.packet_x.write1ByteTxRx(self.port, x_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            self.packet_x.write1ByteTxRx(self.port, x_id, self.ADDR_OPERATING_MODE, 4)  # extended pos
            self.packet_x.write4ByteTxRx(self.port, x_id, self.ADDR_PROFILE_ACCEL, 20 if x_id in [self.x_lift_id, self.xl_back_id] else 10)
            self.packet_x.write4ByteTxRx(self.port, x_id, self.ADDR_PROFILE_VELOCITY, 100)
            if x_id in self.ids_x:
                self.packet_x.write2ByteTxRx(self.port, x_id, self.ADDR_CURRENT_LIMIT, 300)
            self.packet_x.write1ByteTxRx(self.port, x_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)

        self.sync_write = GroupSyncWrite(self.port, self.packet_x, self.ADDR_GOAL_POSITION, 4)
        self.sync_speed = GroupSyncWrite(self.port, self.packet_x, self.ADDR_PROFILE_VELOCITY, 4)

        # 기구학/맵핑
        self.gear = [1.0, 5.0, 5.0, 5.0]
        self.dir  = [1, 1, 1, 1]
        self.zero = [0]
        for dxl_id in self.ids_x:
            pos, res, err = self.packet_x.read4ByteTxRx(self.port, dxl_id, self.ADDR_PRESENT_POSITION)
            self.zero.append(pos if res==0 and err==0 else 0)

        self.chain = Chain(name='arm4', links=[
            OriginLink(),
            DHLink(d=d1, a=0, alpha=np.deg2rad(-90), theta=0),
            DHLink(d=0, a=a2, alpha=0, theta=np.deg2rad(0)),
            DHLink(d=0, a=a3, alpha=0, theta=np.deg2rad(-90)),
            DHLink(d=0, a=a4, alpha=np.deg2rad(-90), theta=np.deg2rad(-90)),
        ])

        self.limits = [(-np.deg2rad(150), np.deg2rad(150)),
                       (-np.deg2rad(180), np.deg2rad(5)),
                       (-np.deg2rad(5),   np.deg2rad(90)),
                       (-np.deg2rad(5),   np.deg2rad(100))]

        self.init_last_q_from_present()
        self.dt = 0.02

        # 카메라-엔드이펙터 변환 매트릭스
        self.camera_offset = np.array([-0.084, 0.0, 0.04155])
        self.camera_rotation = np.eye(3)

        self.get_logger().info('UnifiedArmController ready')

    # ---------- 공용 유틸 (하드웨어/수학) ----------
    def int32_to_le(self, v:int):
        v &= 0xFFFFFFFF
        return bytes([v & 0xFF, (v>>8)&0xFF, (v>>16)&0xFF, (v>>24)&0xFF])

    def joint_rad_to_motor_ticks(self, i, q_joint):
        q_motor = (q_joint * self.dir[i]) * self.gear[i]
        ticks = int(round(q_motor * RAD2TICKS)) + self.zero[i]
        return max(-2_147_483_648, min(2_147_483_647, ticks))

    def ticks_to_joint_rad(self, joint_idx, ticks):
        if joint_idx == 0:
            deg = (ticks - 512) * (300.0/1023.0)
            return np.deg2rad(deg) * self.dir[0]
        zero = self.zero[joint_idx]
        motor = (ticks - zero) / RAD2TICKS
        return (motor / self.gear[joint_idx]) * self.dir[joint_idx]

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
        tgt = 220 if close else 460
        self.packet_ax.write2ByteTxRx(self.port, self.ax_grip_id, self.AX_ADDR_GOAL_POSITION, tgt)

    def gripper_fully(self, close: bool):
        """그리퍼 완전 열기/닫기 제어"""
        if self.ax_grip_id is None: return
        tgt = 10 if close else 460
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

    # ---------- 매뉴얼 경로 ----------
    def on_manual_command(self, msg: String):
        c = msg.data.strip().lower()

        # 홈
        if c == 'home':
            self.go_home_ticks(); return

        # 베이스 단순 회전
        if c in ('left','l','right','r'):
            delta_deg = self.base_step_deg if c in ('left','l') else -self.base_step_deg
            dtick = int(round(delta_deg * AX_TICKS_PER_DEG))
            new_goal = max(0, min(1023, getattr(self, 'base_goal', 512) + dtick))
            self.base_goal = new_goal
            self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, int(self.base_goal))
            return

        # 리프트/백리프트
        if c == 'flift_up':    self.packet_x.write4ByteTxRx(self.port, self.x_lift_id, self.ADDR_GOAL_POSITION, int(self.x_lift_up_tick)); return
        if c == 'flift_center':self.packet_x.write4ByteTxRx(self.port, self.x_lift_id, self.ADDR_GOAL_POSITION, int(self.x_lift_center_tick)); return
        if c == 'flift_down':  self.packet_x.write4ByteTxRx(self.port, self.x_lift_id, self.ADDR_GOAL_POSITION, int(self.x_lift_down_tick)); return
        if c == 'flift_up_little':    self.packet_x.write4ByteTxRx(self.port, self.x_lift_id, self.ADDR_GOAL_POSITION, int(self.x_lift_up_tick_little)); return
        if c == 'flift_down_little':  self.packet_x.write4ByteTxRx(self.port, self.x_lift_id, self.ADDR_GOAL_POSITION, int(self.x_lift_down_tick_little)); return

        if c == 'blift_up':    self.packet_x.write4ByteTxRx(self.port, self.xl_back_id, self.ADDR_GOAL_POSITION, int(self.back_up_tick)); return
        if c == 'blift_center':self.packet_x.write4ByteTxRx(self.port, self.xl_back_id, self.ADDR_GOAL_POSITION, int(self.back_center_tick)); return
        if c == 'blift_down':  self.packet_x.write4ByteTxRx(self.port, self.xl_back_id, self.ADDR_GOAL_POSITION, int(self.back_down_tick)); return
        if c == 'blift_down_45':  self.packet_x.write4ByteTxRx(self.port, self.xl_back_id, self.ADDR_GOAL_POSITION, int(self.back_down_45_tick)); return

        # 그리퍼
        if c == 'open':        self.packet_ax.write2ByteTxRx(self.port, self.ax_grip_id, self.AX_ADDR_GOAL_POSITION, int(self.grip_open_tick)); return
        if c == 'close':       self.packet_ax.write2ByteTxRx(self.port, self.ax_grip_id, self.AX_ADDR_GOAL_POSITION, int(self.grip_close_tick)); return
        if c == 'close_fully': self.packet_ax.write2ByteTxRx(self.port, self.ax_grip_id, self.AX_ADDR_GOAL_POSITION, int(self.grip_close_fully_tick)); return

        # 조인트 미세조정
        if c in ('j2_plus','j2_minus','j3_plus','j3_minus','j4_plus','j4_minus'):
            joint = {'j2':2,'j3':3,'j4':4}[c[:2]]
            sign  = +1 if c.endswith('plus') else -1
            self._nudge_joint_deg(joint, 5.0*sign)
            return

        if c == 'base_home':
            self.base_goal = 512
            self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, 512)
            return

    def _nudge_joint_deg(self, joint_num:int, delta_deg:float):
        if joint_num == 1:
            dtick = int(round(delta_deg * AX_TICKS_PER_DEG))
            self.base_goal = max(0, min(1023, getattr(self,'base_goal',512) + dtick))
            self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, int(self.base_goal))
            return
        dxl_id = self.ids_x[joint_num-2]
        curr, res, err = self.packet_x.read4ByteTxRx(self.port, dxl_id, self.ADDR_PRESENT_POSITION)
        if res!=0 or err!=0: return
        if curr > 2_147_483_647: curr -= 4_294_967_296
        delta_tick = int(round(np.deg2rad(delta_deg) * self.dir[joint_num-1] * self.gear[joint_num-1] * RAD2TICKS))
        new_tick = max(-2_147_483_648, min(2_147_483_647, curr + delta_tick))
        self.packet_x.write4ByteTxRx(self.port, dxl_id, self.ADDR_GOAL_POSITION, int(new_tick))

    # ---------- 자동 경로 ----------
    def handle_arm_do_task(self, request, response):
        x, y, z, task = request.x, request.y, request.z, request.task
        ok = False

        if task == 'pick':
            with self.state_lock:
                if self.active_task is not None:
                    response.success = False
                    response.message = f'busy: {self.active_task}'
                    return response
                self.active_task = 'pick'
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
        elif task == 'mission_3_basic_state':
            ok = self.mission_3_basic_state()
        # 이게 미션4에서 왼쪽 보고 가는 동작
        elif task == 'mission_4_basic_state':
            ok = self.mission_4_basic_state()
        elif task == 'press_button':
            self.get_logger().warn("press_button_seq function not implemented")
            ok = False
        elif task == 'open_door': ok = self.open_door()
        elif task == 'place': ok = self.place_seq([x, y, z])
        elif task == 'move': self.move_ik(x, y, z); ok = True
        elif task == 'go_home': self.go_home_ticks(); ok = True
        elif task == 'go_autonomous_driving': self.go_autonomous_driving_position(); ok = True
        elif task == 'go_ik_ready': self.go_ik_ready_position(); ok = True
        elif task == 'see_around': self.ax_see_around(); ok = True
        elif task == 'see_around_90': self.ax_see_around_90(); ok = True
        elif task == 'led_button_press':
            with self.state_lock:
                if self.active_task is not None:
                    response.success = False
                    response.message = f'busy: {self.active_task}'
                    return response
                self.active_task = 'led_button_press'
            threading.Thread(target=self._run_led_button_press_once, daemon=True).start()
            response.success = True
            response.message = 'started'
            return response
        elif task == 'reset_ebox_aligned':
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

    def handle_detected_object(self, msg: DetectedObject):
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

        # 쿨다운 체크
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
            success = mission_handler.execute(msg.pixel_x, msg.pixel_y, 0.0)
            result = "SUCCESS" if success else "FAILED"
            self.get_logger().info(f"Mission {object_name}: {result}")
        except Exception as e:
            self.get_logger().error(f"Mission {object_name} failed with exception: {e}")
        finally:
            self.mission_in_progress = False
            self.ebox_aligned = False
            self.last_mission_time = time.time()
            self.get_logger().info(f"Mission for {object_name} completed, cooldown for {self.mission_cooldown}s")

    def direction_callback(self, msg: String):
        self.current_direction = msg.data
        self.get_logger().info(f"Direction updated: {self.current_direction}")

    # ---------- 기존 arm_server 함수들 ----------
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
        for j, dxl_id in enumerate(self.ids_x, start=1):
            if j == 1 or j == 2:  # Joint 2, 3번
                self.packet_x.write4ByteTxRx(self.port, dxl_id, self.ADDR_GOAL_POSITION, 2048)
                self.get_logger().info(f"Moving joint {j+1} (ID {dxl_id}) to position 2048")
                self.write_goal_and_wait(dxl_id, 2048, tol=10, timeout=5.0)
                self.xl_home_ticks.append(2048)
                q[j] = self.ticks_to_joint_rad(j, 2048)
            else:  # Joint 4번은 현재 위치 유지
                pos, res, err = self.packet_x.read4ByteTxRx(self.port, dxl_id, self.ADDR_PRESENT_POSITION)
                if res==0 and err==0:
                    self.xl_home_ticks.append(int(pos))
                    q[j] = self.ticks_to_joint_rad(j, int(pos))
                else:
                    self.xl_home_ticks.append(0)

        self.last_q = np.zeros(len(self.chain.links))
        self.last_q[1:5] = q
        self.get_logger().info(f"Init last_q(deg)={np.rad2deg(self.last_q[1:5])}")

    def ax_see_around(self):
        self.move_xl_only(offset_j2=0, offset_j3=0, offset_j4=1000)
        """AX 베이스를 0틱에서 1023틱까지 천천히 회전시키고 홈으로 복귀"""
        self.get_logger().info("Starting AX see around - rotating from 0 to 1023 ticks")

        # 0틱부터 1023틱까지 천천히 회전
        step_size = 20  # 한 번에 10틱씩 이동
        delay = 0.2     # 각 스텝 간 0.2초 대기

        for tick in range(0, 1024, step_size):
            # 마지막 스텝에서는 정확히 1023으로
            if tick + step_size > 1023:
                tick = 1023

            self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, tick)
            self.get_logger().info(f"AX base moving to tick: {tick}")
            time.sleep(delay)

            if tick == 1023:
                break

        self.get_logger().info("AX see around completed, returning home")
        time.sleep(1.0)  # 마지막 위치에서 잠시 대기

        # 홈으로 복귀
        self.go_home_ticks()

    def ax_see_around_90(self):
        self.move_xl_only(offset_j2=0, offset_j3=0, offset_j4=1000)
        """AX 베이스를 0틱에서 1023틱까지 천천히 회전시키고 홈으로 복귀"""
        self.get_logger().info("Starting AX see around - rotating from 0 to 1023 ticks")

        # 0틱부터 1023틱까지 천천히 회전
        step_size = 20  # 한 번에 10틱씩 이동
        delay = 0.2     # 각 스텝 간 0.2초 대기

        for tick in range(204, 820, step_size):
            # 마지막 스텝에서는 정확히 1023으로
            if tick + step_size > 820:
                tick = 820

            self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, tick)
            self.get_logger().info(f"AX base moving to tick: {tick}")
            time.sleep(delay)

            if tick == 820:
                break

        self.get_logger().info("AX see around completed, returning home")
        time.sleep(1.0)  # 마지막 위치에서 잠시 대기

        # 홈으로 복귀
        self.go_home_ticks()

    def go_home_ticks(self):
        # 베이스 AX 복귀
        self.base_goal = int(self.base_home_tick)
        self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, self.base_goal)
        self.get_logger().info(f"Home: base_goal reset to {self.base_goal}")

        # J2~J4 XL 복귀
        self.sync_write.clearParam()
        for dxl_id, t in zip(self.ids_x, self.xl_home_ticks):
            self.sync_write.addParam(dxl_id, self.int32_to_le(int(t)))
        self.sync_write.txPacket()

        self.get_logger().info("Returned to home position")

    def go_autonomous_driving_position(self):
        """자율주행용 포지션"""
        self.base_goal = int(self.base_home_tick)
        self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, self.base_goal)

        j2_angle = np.deg2rad(-20)
        j2_ticks = self.zero[1] + int(j2_angle * self.gear[1] * self.dir[1] * RAD2TICKS)

        autonomous_ticks = [
            j2_ticks,
            2048,
            self.xl_home_ticks[2]
        ]

        self.sync_write.clearParam()
        for dxl_id, t in zip(self.ids_x, autonomous_ticks):
            self.sync_write.addParam(dxl_id, self.int32_to_le(int(t)))
        self.sync_write.txPacket()

        self.get_logger().info("Moved to autonomous driving position")

    def go_ik_ready_position(self):
        """역기구학 작업용 포지션"""
        self.go_home_ticks()
        self.get_logger().info("Returned to IK-ready position")

    def move_from_home_ticks(self, offset_j1=0, offset_j2=0, offset_j3=0, offset_j4=0):
        """go_home_ticks 위치에서 오프셋만큼 이동"""
        target_base = self.base_home_tick + offset_j1
        self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, int(target_base))

        target_positions = [
            self.xl_home_ticks[0] + offset_j2,
            self.xl_home_ticks[1] + offset_j3,
            self.xl_home_ticks[2] + offset_j4
        ]

        self.sync_write.clearParam()
        for dxl_id, target_pos in zip(self.ids_x, target_positions):
            self.sync_write.addParam(dxl_id, self.int32_to_le(int(target_pos)))

        self.sync_write.txPacket()
        time.sleep(self.dt)
        return True

    def move_xl_only(self, offset_j2=0, offset_j3=0, offset_j4=0):
        """베이스는 유지하고 J2, J3, J4만 이동"""
        target_positions = [
            self.xl_home_ticks[0] + offset_j2,
            self.xl_home_ticks[1] + offset_j3,
            self.xl_home_ticks[2] + offset_j4
        ]

        self.sync_write.clearParam()
        for dxl_id, target_pos in zip(self.ids_x, target_positions):
            self.sync_write.addParam(dxl_id, self.int32_to_le(int(target_pos)))

        self.sync_write.txPacket()
        time.sleep(self.dt)
        return True

    def solve_ik_q(self, x, y, z):
        # 여러 초기값으로 IK 시도
        initial_positions = [
            self.last_q,
            np.array([0, 0, 0, 0, 0]),
            np.array([0, 0, np.deg2rad(90), 0, 0]),
            np.array([0, np.deg2rad(-90), np.deg2rad(45), np.deg2rad(80), 0]),
            np.array([0, np.deg2rad(-100), np.deg2rad(50), np.deg2rad(85), 0]),
            np.array([0, np.deg2rad(-110), np.deg2rad(55), np.deg2rad(90), 0]),
            np.array([0, np.deg2rad(-120), np.deg2rad(60), np.deg2rad(85), 0]),
            np.array([0, np.deg2rad(-130), np.deg2rad(65), np.deg2rad(80), 0]),
            np.array([0, np.deg2rad(-80), np.deg2rad(40), np.deg2rad(75), 0]),
            np.array([0, np.deg2rad(-70), np.deg2rad(35), np.deg2rad(70), 0]),
            np.array([0, np.deg2rad(-60), np.deg2rad(30), np.deg2rad(65), 0]),
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

        self.get_logger().warn(f"IK failed for target ({x:.3f}, {y:.3f}, {z:.3f})")
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

    def pick_seq(self, target_pos=None):
        """박스 pick 시퀀스"""
        self.get_logger().info("박스 pick 시퀀스 시작")
        self.gripper(False)
        self.move_from_home_ticks(offset_j2=-3500, offset_j3=952, offset_j4=0)
        self._sleep(3.0)

        self.get_logger().info("Aligning with ebox position...")
        aligned = self.align_with_ebox_position(max_attempts=20, timeout=25.0)
        if not aligned:
            self.get_logger().warn("Alignment failed. Abort pick.")
            self.go_home_ticks()
            self.get_logger().info("박스 pick 시퀀스 완료")
            return False

        self.move_xl_only(offset_j2=-7748, offset_j3=2048, offset_j4=2187)
        self._sleep(5.0)
        self.gripper(True)
        self._sleep(1.5)
        self.move_xl_only(offset_j2=-4120, offset_j3=2048, offset_j4=2187)
        self._sleep(3.0)

        self.go_home_ticks()
        self.get_logger().info("박스 pick 시퀀스 완료")
        return True

    def led_button_press_sequence(self):
        """LED 버튼 누르기 시퀀스"""
        self.get_logger().info("LED button press sequence started")

        self.get_logger().info("Step 1: Color tracking already active, starting alignment...")
        self.color_tracking_active = True

        self.get_logger().info("Step 2: Aligning with color target")
        if not self.align_with_color_target(max_attempts=40, timeout=30.0):
            self.get_logger().error("Failed to align with color target")
            self.color_tracking_active = False
            return False

        self.get_logger().info("Step 3: Color tracking completed")

        self.get_logger().info("Step 4: Pressing LED button")
        if not self.press_led_button_ticks():
            self.get_logger().error("Failed to press LED button")
            return False

        self.get_logger().info("Step 5: Returning home")
        self.go_home_ticks()
        self.move_xl_only(offset_j2=0, offset_j3=0, offset_j4=3000)
        self.get_logger().info("LED button press sequence completed successfully")
        return True

    def align_with_color_target(self, max_attempts=40, timeout=30.0):
        """색상 추적 기반 베이스 회전 정렬 (적응형 스텝 크기)"""
        attempts = 0
        start_time = time.time()
        consecutive_same_direction = 0
        last_direction = None

        self.get_logger().info("Starting color-based alignment...")

        while attempts < max_attempts and (time.time() - start_time) < timeout:
            if self.current_direction is None:
                self.get_logger().info("Waiting for color direction...")
                time.sleep(0.1)
                attempts += 1
                continue

            direction = self.current_direction
            self.get_logger().info(f"Color direction: {direction}")

            if direction == "center":
                self.get_logger().info("Color target centered! Alignment complete")
                return True

            # 연속으로 같은 방향이 나오는 횟수 카운트
            if direction == last_direction:
                consecutive_same_direction += 1
            else:
                consecutive_same_direction = 1
            last_direction = direction

            # 적응형 스텝 크기
            if consecutive_same_direction >= 8:
                step = 15
            elif consecutive_same_direction >= 5:
                step = 12
            elif consecutive_same_direction >= 3:
                step = 8
            else:
                step = 6

            if direction == "left":
                self.rotate_base_by_ticks(+step)
                self.get_logger().info(f"Rotating base clockwise +{step} ticks (color on left, consecutive: {consecutive_same_direction})")
            elif direction == "right":
                self.rotate_base_by_ticks(-step)
                self.get_logger().info(f"Rotating base counter-clockwise -{step} ticks (color on right, consecutive: {consecutive_same_direction})")

            time.sleep(0.4)
            attempts += 1

        self.get_logger().warn(f"Failed to align with color after {attempts} attempts or {timeout}s timeout")
        return False

    def align_with_ebox_position(self, max_attempts=25, timeout=30.0, threshold=15.0):
        """ebox 감지 정보 기반 베이스 회전 정렬"""
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
            if stagnant >= 15:
                self.get_logger().warn("Alignment stalled. Abort.")
                return False

            # 정교한 스텝 크기 조정 (더 큰 스텝으로 개선)
            if abs(dx) > 60:
                step = 15    # 12 -> 15
            elif abs(dx) > 30:
                step = 10    # 6 -> 10
            elif abs(dx) > 15:
                step = 6     # 3 -> 6
            else:
                step = 3     # 1 -> 3

            if dx < -threshold:
                self.rotate_base_by_ticks(+step)
                self.get_logger().info(f"Rotating base clockwise +{step} ticks (ebox on left)")
            else:
                self.rotate_base_by_ticks(-step)
                self.get_logger().info(f"Rotating base counter-clockwise -{step} ticks (ebox on right)")

            time.sleep(0.3)
            attempts += 1

        self.get_logger().warn(f"Failed to align with ebox after {attempts} attempts or {timeout}s timeout")
        return False

    def move_to_safe_position(self):
        """안전한 높은 위치로 이동"""
        safe_ticks = [-308, -4057, 3953, -268]

        try:
            self.move_from_home_ticks(offset_j1=safe_ticks[0], offset_j2=safe_ticks[1], offset_j3=safe_ticks[2], offset_j4=safe_ticks[3])
            time.sleep(5.0)
            self.get_logger().info("Moved to safe position")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to move to safe position: {e}")
            return False

    def rotate_base_by_ticks(self, tick_offset):
        """베이스를 지정된 틱만큼 회전 (누적 목표값 사용)"""
        try:
            # 현재값이 아니라 '마지막 목표값' 기준으로 누적
            new_goal = max(0, min(1023, int(self.base_goal) + int(tick_offset)))
            self.packet_ax.write2ByteTxRx(self.port, self.ax_base_id, self.AX_ADDR_GOAL_POSITION, new_goal)
            self.base_goal = new_goal
            self.get_logger().info(f"Base goal {self.base_goal} (offset {tick_offset:+d})")
        except Exception as e:
            self.get_logger().error(f"Base rotation failed: {e}")

    def press_led_button_ticks(self):
        """미리 정의된 틱값으로 버튼 누르기"""
        self.gripper_fully(True)

        press_near_1_ticks = [-4057, 3953, 1232]
        press_near_1_2_ticks = [-4057, 3953, 2400]
        press_near_2_ticks = [-4545, 2953, 3482]
        push_ticks = [-4545, 2425, 3482]

        try:
            self.move_xl_only(offset_j2=press_near_1_ticks[0], offset_j3=press_near_1_ticks[1], offset_j4=press_near_1_ticks[2])
            time.sleep(4.0)

            self.move_xl_only(offset_j2=press_near_1_2_ticks[0], offset_j3=press_near_1_2_ticks[1], offset_j4=press_near_1_2_ticks[2])
            time.sleep(4.0)

            self.move_xl_only(offset_j2=press_near_2_ticks[0], offset_j3=press_near_2_ticks[1], offset_j4=press_near_2_ticks[2])
            time.sleep(4.0)

            self.move_xl_only(offset_j2=push_ticks[0], offset_j3=push_ticks[1], offset_j4=push_ticks[2])
            time.sleep(2.0)

            self.move_xl_only(offset_j2=press_near_1_ticks[0], offset_j3=press_near_1_ticks[1], offset_j4=press_near_1_ticks[2])
            time.sleep(1.0)

            self.get_logger().info("LED button pressed successfully")
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to press LED button: {e}")
            return False

    def open_door(self):
        self.gripper_fully(True)

        press_near_1_ticks = [-4774, 3236, 1791]
        push_ticks = [-4888, 1823, 2413]

        try:
            self.move_xl_only(offset_j2=press_near_1_ticks[0], offset_j3=press_near_1_ticks[1], offset_j4=press_near_1_ticks[2])
            time.sleep(4.0)

            self.move_xl_only(offset_j2=push_ticks[0], offset_j3=push_ticks[1], offset_j4=push_ticks[2])
            time.sleep(2.0)


            self.get_logger().info("LED button pressed successfully")
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to press LED button: {e}")
            return False

    def place_seq(self, p):
        """박스 place 시퀀스"""
        """
        J1: 512 -> 820
        J2: 2048 -> 20
        J3: 2048 -> 354
        """
        self.get_logger().info("박스 place 시퀀스 시작")
        self.go_home_ticks()
        time.sleep(1.0)
        self.move_from_home_ticks(offset_j1=308)
        time.sleep(3.0)
        self.move_xl_only(offset_j2=-2028, offset_j3=-1694)
        time.sleep(2.0)
        self.gripper(False)
        time.sleep(2.0)
        self.move_xl_only(offset_j2=0, offset_j3=0)
        time.sleep(2.0)
        self.go_home_ticks()
        time.sleep(1.0)
        self.get_logger().info("박스 place 시퀀스 완료")
        return True

    def mission_3_basic_state(self):
        """미션3 기본 상태"""
        self.get_logger().info("Moving to Mission 3 basic state")

        if not self.move_to_safe_position():
            self.get_logger().error("Failed to move to safe position")
            return False

        self.get_logger().info("Mission 3 basic state ready")
        return True
    
    def mission_4_basic_state(self):
        """미션3 기본 상태"""
        self.get_logger().info("Moving to Mission 3 basic state")

        if not self.move_to_4_position():
            self.get_logger().error("Failed to move to safe position")
            return False

        self.get_logger().info("Mission 3 basic state ready")
        return True

    def move_to_4_position(self):
        """안전한 높은 위치로 이동"""
        safe_ticks = [308, -4057, 3953, -268]

        try:
            self.move_from_home_ticks(offset_j1=safe_ticks[0], offset_j2=safe_ticks[1], offset_j3=safe_ticks[2], offset_j4=safe_ticks[3])
            time.sleep(5.0)
            self.get_logger().info("Moved to safe position")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to move to safe position: {e}")
            return False

    def _run_pick_once(self, x, y, z):
        """비동기 픽 실행 함수"""
        try:
            self.mission_in_progress = True
            auto_prev = self.enable_auto_mission
            self.enable_auto_mission = False

            self.pick_seq([x, y, z])
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

    def _run_led_button_press_once(self):
        """비동기 LED 버튼 프레스 실행 함수"""
        try:
            self.mission_in_progress = True

            success = self.led_button_press_sequence()
            if success:
                self.get_logger().info("LED button press completed successfully")
            else:
                self.get_logger().error("LED button press failed")

        except Exception as e:
            self.get_logger().error(f"led_button_press failed: {e}")
            import traceback
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
        finally:
            self.get_logger().info("Cleaning up led_button_press task...")
            self.mission_in_progress = False
            with self.state_lock:
                self.active_task = None
            self.get_logger().info("led_button_press task cleanup complete")

    def _sleep(self, dt):
        """짧은 슬립으로 쪼개서 콜백 응답성 개선"""
        t0 = time.time()
        while time.time() - t0 < dt:
            time.sleep(0.02)

def main():
    rclpy.init()
    node = UnifiedArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()