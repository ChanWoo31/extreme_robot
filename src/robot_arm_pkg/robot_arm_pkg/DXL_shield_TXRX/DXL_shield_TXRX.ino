#include <Dynamixel2Arduino.h>
#include <SoftwareSerial.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
#define DXL_SERIAL Serial
const int DXL_DIR_PIN = 2;  // Shield DIR 핀
#else
#define DXL_SERIAL Serial1
const int DXL_DIR_PIN = 2;
#endif

// SoftwareSerial 설정 (핀 7 = RX, 8 = TX)
SoftwareSerial softSerial(7, 8);

const uint8_t DXL_ID_LIFT = 6;
const uint8_t DXL_ID_robot_arm_2 = 2;
const uint8_t DXL_ID_robot_arm_3 = 3;
const uint8_t DXL_ID_robot_arm_4 = 4;
const float DXL_PROTOCOL_VERSION_AX = 1.0;  // AX-18 Protocol 1.0
const float DXL_PROTOCOL_VERSION_XL = 2.0;
Dynamixel2Arduino dxl_AX(DXL_SERIAL, DXL_DIR_PIN);
Dynamixel2Arduino dxl_XL(DXL_SERIAL, DXL_DIR_PIN);

inline void use_proto_ax() { dxl_AX.setPortProtocolVersion(DXL_PROTOCOL_VERSION_AX); }
inline void use_proto_xl() { dxl_XL.setPortProtocolVersion(DXL_PROTOCOL_VERSION_XL); }

using namespace ControlTableItem;

// 목표 위치 raw 값 정의
const int POS_0 = 512;           // 0도
const int POS_UP = 512 - 250;    // 30도
const int POS_DOWN = 512 + 250;  // -30도

const int DXL_SPIN = 1024;

int32_t base_raw_2 = 0, base_raw_3 = 0, base_raw_4 = 0;

int32_t motor_goal_spin_2 = 0;
int32_t motor_goal_spin_3 = 0;
int32_t motor_goal_spin_4 = 0;

const int32_t LIM2_MIN_TICK = 0;
const int32_t LIM2_MAX_TICK = 1024*7;

const int32_t LIM3_MIN_TICK = 0;
const int32_t LIM3_MAX_TICK = 1024*4;

const int32_t LIM4_MIN_TICK = 0;
const int32_t LIM4_MAX_TICK = 1024*4;

void init_xl_offsets() {
  // 현재 위치를 기준점으로 저장
  uint8_t arm_ids[] = {DXL_ID_robot_arm_2, DXL_ID_robot_arm_3, DXL_ID_robot_arm_4};
  for (uint8_t i = 0; i < sizeof(arm_ids)/sizeof(arm_ids[0]); i++) {
    uint8_t id = arm_ids[i];
    int32_t now = dxl_XL.getPresentPosition(id, UNIT_RAW);
    if (id == DXL_ID_robot_arm_2) base_raw_2 = now;
    else if (id == DXL_ID_robot_arm_3) base_raw_3 = now;
    else if (id == DXL_ID_robot_arm_4) base_raw_4 = now;
  }
  // 누적 각도는 0으로 초기화
  motor_goal_spin_2 = 0;
  motor_goal_spin_3 = 0;
  motor_goal_spin_4 = 0;
}

static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi){
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  softSerial.begin(9600);
  Serial.println("Ready to receive commands via SoftwareSerial");

  // 모터 초기화
  dxl_AX.begin(1000000);
  dxl_AX.setPortProtocolVersion(DXL_PROTOCOL_VERSION_AX);

  dxl_AX.torqueOff(DXL_ID_LIFT);
  dxl_AX.setOperatingMode(DXL_ID_LIFT, OP_POSITION);
  dxl_AX.torqueOn(DXL_ID_LIFT);

  dxl_AX.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_LIFT, 50);  // 속도 제한

  dxl_XL.begin(1000000);
  dxl_XL.setPortProtocolVersion(DXL_PROTOCOL_VERSION_XL);

  uint8_t arm_ids[] = {DXL_ID_robot_arm_2, DXL_ID_robot_arm_3, DXL_ID_robot_arm_4};
  for (uint8_t i = 0; i < sizeof(arm_ids); i++) {
    uint8_t id = arm_ids[i];
    dxl_XL.torqueOff(id);
    dxl_XL.setOperatingMode(id, OP_EXTENDED_POSITION);
    dxl_XL.writeControlTableItem(PROFILE_ACCELERATION, id, 50);
    dxl_XL.writeControlTableItem(PROFILE_VELOCITY, id, 100);
    dxl_XL.torqueOn(id);
  }

  init_xl_offsets();

  // delay(200);
  // test_once_arm4down();
}

// 현재 위치가 목표 위치 근처에 올 때까지 대기
void waitUntilPosition(int target_raw) {
  int present = dxl_AX.getPresentPosition(DXL_ID_LIFT);
  while (abs(target_raw - present) > 5) {  // ±5 raw 허용
    present = dxl_AX.getPresentPosition(DXL_ID_LIFT);
    delay(10);
  }
  delay(50);
}

void loop() {
  // 소프트 시리얼로 들어온 문자열 확인
  if (softSerial.available()) {
    String cmd = softSerial.readStringUntil('\n');  // 개행까지 읽기
    cmd.trim();                                     // 공백 제거

    if (cmd == "UP") {
      lift_up();
    } else if (cmd == "ORIGIN") {
      lift_origin();
    } else if (cmd == "DOWN") {
      lift_down();
    } else if (cmd == "ARM2UP") {
      dynamixel_motor_2_plus(); 
    } else if (cmd == "ARM2DOWN") {
      dynamixel_motor_2_minus(); 
    } else if (cmd == "ARM3UP") {
      dynamixel_motor_3_plus(); 
    } else if (cmd == "ARM3DOWN") {
      dynamixel_motor_3_minus(); 
    } else if (cmd == "ARM4UP") {
      dynamixel_motor_4_plus(); 
    } else if (cmd == "ARM4DOWN") {
      dynamixel_motor_4_minus(); 
    }else {
      Serial.print("Unknown command: ");
      Serial.println(cmd);
    }
  }
}
// 리프트 올리기
void lift_up() {
  use_proto_ax();
  dxl_AX.setGoalPosition(DXL_ID_LIFT, POS_UP);  // 30도
  delay(1000);
}

// 리프트 내리기
void lift_down() {
  use_proto_ax();
  dxl_AX.setGoalPosition(DXL_ID_LIFT, POS_DOWN);  // -30도
  delay(1000);
}

void lift_origin() {
  use_proto_ax();
  dxl_AX.setGoalPosition(DXL_ID_LIFT, POS_0);  // -30도
  delay(2000);
}

void dynamixel_motor_2_plus() {
  use_proto_xl();
  motor_goal_spin_2 = clamp_i32(motor_goal_spin_2 + DXL_SPIN, LIM2_MIN_TICK, LIM2_MAX_TICK);
  int32_t goal_raw = base_raw_2 + motor_goal_spin_2;
  dxl_XL.setGoalPosition(DXL_ID_robot_arm_2, goal_raw, UNIT_RAW);
}

void dynamixel_motor_2_minus() {
  use_proto_xl();
  motor_goal_spin_2 = clamp_i32(motor_goal_spin_2 - DXL_SPIN, LIM2_MIN_TICK, LIM2_MAX_TICK);
  int32_t goal_raw = base_raw_2 + motor_goal_spin_2;
  dxl_XL.setGoalPosition(DXL_ID_robot_arm_2, goal_raw, UNIT_RAW);
}

void dynamixel_motor_3_plus() {
  use_proto_xl();
  motor_goal_spin_3 = clamp_i32(motor_goal_spin_3 + DXL_SPIN, LIM3_MIN_TICK, LIM3_MAX_TICK);
  int32_t goal_raw = base_raw_3 + motor_goal_spin_3;
  dxl_XL.setGoalPosition(DXL_ID_robot_arm_3, goal_raw, UNIT_RAW);
}

void dynamixel_motor_3_minus() {
  use_proto_xl();
  motor_goal_spin_3 = clamp_i32(motor_goal_spin_3 - DXL_SPIN, LIM3_MIN_TICK, LIM3_MAX_TICK);
  int32_t goal_raw = base_raw_3 + motor_goal_spin_3;
  dxl_XL.setGoalPosition(DXL_ID_robot_arm_3, goal_raw, UNIT_RAW);
}

void dynamixel_motor_4_plus() {
  use_proto_xl();
  motor_goal_spin_4 = clamp_i32(motor_goal_spin_4 + DXL_SPIN, LIM4_MIN_TICK, LIM4_MAX_TICK);
  int32_t goal_raw = base_raw_4 + motor_goal_spin_4;
  dxl_XL.setGoalPosition(DXL_ID_robot_arm_4, goal_raw, UNIT_RAW);
}

void dynamixel_motor_4_minus() {
  use_proto_xl();
  motor_goal_spin_4 = clamp_i32(motor_goal_spin_4 - DXL_SPIN, LIM4_MIN_TICK, LIM4_MAX_TICK);
  int32_t goal_raw = base_raw_4 + motor_goal_spin_4;
  dxl_XL.setGoalPosition(DXL_ID_robot_arm_4, goal_raw, UNIT_RAW);
}

