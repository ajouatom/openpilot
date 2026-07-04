#pragma once

#include "safety_declarations.h"
#include "safety_volkswagen_common.h"

// MEB platform CAN message IDs
#define MSG_ESC_51           0x0FC   // RX from ESC, for wheel speeds and brake pressure
#define MSG_QFK_01           0x13D   // RX from EPS, for HCA status and curvature feedback
#define MSG_MOTOR_51         0x10B   // RX from ECU, for ACC status and throttle input
#define MSG_HCA_03           0x303   // TX by OP, Heading Control Assist curvature command (MEB)
#define MSG_ACC_18           0x14D   // TX by OP (long), ACC acceleration command (carrot DBC: ACC_18, 32B)
#define MSG_ACC_19           0x300   // TX by OP (long), ACC HUD (carrot DBC: MEB_ACC_01, 48B)
#define MSG_TA_01            0x26B   // TX by OP (long), Travel Assist status (8B)
#define MSG_KLR_01           0x25D   // TX by OP, capacitive steering wheel touch (EA hands-on pacify, 8B)
#define MSG_EA_02            0x1F0   // TX by OP, Emergency Assist HUD relay (steering-wheel icon, 8B)

// Note: MSG_LH_EPS_03 (0x09F), MSG_GRA_ACC_01 (0x12B), MSG_MOTOR_14 (0x3BE),
//       MSG_LDW_02 (0x397) are shared with MQB (defined in safety_volkswagen_common.h)

#define FLAG_VOLKSWAGEN_LONG_CONTROL 1  // matches VolkswagenSafetyFlags.LONG_CONTROL

static bool volkswagen_meb_brake_pedal_switch = false;
static bool volkswagen_meb_longitudinal = false;


static safety_config volkswagen_meb_init(uint16_t param) {
  // MEB platform: gateway harness (J533)
  // Bus 0: powertrain CAN (ESC, ECU, EPS)
  // Bus 2: extended CAN (camera, radar)
  static const CanMsg VOLKSWAGEN_MEB_STOCK_TX_MSGS[] = {
    {MSG_HCA_03, 0, 24},      // Curvature-based steering control to EPS (24 bytes)
    {MSG_GRA_ACC_01, 0, 8},   // ACC buttons (cancel/resume) to gateway
    {MSG_GRA_ACC_01, 2, 8},   // ACC buttons to camera bus
    {MSG_LDW_02, 0, 8},       // Lane departure warning HUD
    {MSG_LH_EPS_03, 2, 8},    // EPS update for Emergency Assist pacification
    {MSG_KLR_01, 0, 8},       // Capacitive steering wheel touch (EA hands-on)
    {MSG_KLR_01, 2, 8},
    {MSG_EA_02, 0, 8},        // Emergency Assist HUD relay (steering-wheel icon)
  };

  // openpilot longitudinal: openpilot replaces stock ACC accel command (ACC_18) and HUD (ACC_19/MEB_ACC_01),
  // and sends Travel Assist status (TA_01). Gateway harness keeps stock radar/AEB active (no radar disable).
  static const CanMsg VOLKSWAGEN_MEB_LONG_TX_MSGS[] = {
    {MSG_HCA_03, 0, 24},
    {MSG_GRA_ACC_01, 0, 8},
    {MSG_GRA_ACC_01, 2, 8},
    {MSG_LDW_02, 0, 8},
    {MSG_LH_EPS_03, 2, 8},
    {MSG_ACC_18, 0, 32},      // ACC acceleration command
    {MSG_ACC_19, 0, 48},      // ACC HUD (MEB_ACC_01)
    {MSG_TA_01, 0, 8},        // Travel Assist
    {MSG_KLR_01, 0, 8},       // Capacitive steering wheel touch (EA hands-on)
    {MSG_KLR_01, 2, 8},
    {MSG_EA_02, 0, 8},        // Emergency Assist HUD relay (steering-wheel icon)
  };

  // 크기는 vw_meb.dbc 실제 길이와 일치해야 함 (ESC_51=48, QFK_01/Motor_51=32 CAN-FD).
  // CRC lut는 LH_EPS_03/GRA_ACC_01만 커버 → 나머지는 checksum/counter 무시(bring-up).
  static RxCheck volkswagen_meb_rx_checks[] = {
    {.msg = {{MSG_ESC_51, 0, 48, .ignore_checksum = true, .ignore_counter = true, .frequency = 100U}, { 0 }, { 0 }}},
    {.msg = {{MSG_LH_EPS_03, 0, 8, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},
    {.msg = {{MSG_QFK_01, 0, 32, .ignore_checksum = true, .ignore_counter = true, .frequency = 100U}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_51, 0, 32, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_14, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 10U}, { 0 }, { 0 }}},
    {.msg = {{MSG_GRA_ACC_01, 0, 8, .max_counter = 15U, .frequency = 33U}, { 0 }, { 0 }}},
  };

  volkswagen_meb_longitudinal = GET_FLAG(param, FLAG_VOLKSWAGEN_LONG_CONTROL);

  volkswagen_set_button_prev = false;
  volkswagen_resume_button_prev = false;
  volkswagen_meb_brake_pedal_switch = false;

  gen_crc_lookup_table_8(0x2F, volkswagen_crc8_lut_8h2f);
  if (volkswagen_meb_longitudinal) {
    return BUILD_SAFETY_CFG(volkswagen_meb_rx_checks, VOLKSWAGEN_MEB_LONG_TX_MSGS);
  }
  return BUILD_SAFETY_CFG(volkswagen_meb_rx_checks, VOLKSWAGEN_MEB_STOCK_TX_MSGS);
}

static void volkswagen_meb_rx_hook(const CANPacket_t *to_push) {
  if (GET_BUS(to_push) == 0U) {
    int addr = GET_ADDR(to_push);

    // Update in-motion state from ESC_51 wheel speeds
    // Signals: ESC_51.VL_Radgeschw, VR_Radgeschw, HL_Radgeschw, HR_Radgeschw
    if (addr == MSG_ESC_51) {
      // 휠속도: HL@64(byte8), HR@80(byte10), VL@96(byte12), VR@112(byte14) 각 16비트
      int speed = 0;
      for (uint8_t i = 8U; i < 16U; i += 2U) {
        int wheel_speed = GET_BYTE(to_push, i) | (GET_BYTE(to_push, i + 1U) << 8);
        speed += wheel_speed;
      }
      vehicle_moving = speed > 0;
    }

    // Update driver input torque samples
    // Signal: LH_EPS_03.EPS_Lenkmoment (absolute torque)
    // Signal: LH_EPS_03.EPS_VZ_Lenkmoment (direction)
    if (addr == MSG_LH_EPS_03) {
      // EPS_Lenkmoment @40:10 (byte5 + byte6 하위 2비트), 부호 EPS_VZ_Lenkmoment @55
      int torque_driver_new = GET_BYTE(to_push, 5) | ((GET_BYTE(to_push, 6) & 0x3U) << 8);
      int sign = (GET_BYTE(to_push, 6) & 0x80U) >> 7;
      if (sign == 1) {
        torque_driver_new *= -1;
      }
      update_sample(&torque_driver, torque_driver_new);
    }

    // ACC status from Motor_51 (MEB equivalent of TSK_06)
    // Signal: Motor_51.TSK_Status
    if (addr == MSG_MOTOR_51) {
      // TSK_Status @88:3 → byte 11 하위 3비트
      int acc_status = (GET_BYTE(to_push, 11) & 0x7U);
      bool cruise_engaged = (acc_status == 3) || (acc_status == 4) || (acc_status == 5);
      acc_main_on = cruise_engaged || (acc_status == 2);

      // Accel_Pedal_Pressure @12:9 (byte1 bit4 .. byte2 bit4)
      int accel_p = ((GET_BYTE(to_push, 1) >> 4) | (GET_BYTE(to_push, 2) << 4)) & 0x1FFU;
      gas_pressed = accel_p > 0;

      if (!acc_main_on) {
        controls_allowed = false;
      }

      // PCM cruise check: stock ACC only (if2/upstream VW identical).
      // With openpilot longitudinal, entry/exit is button-based; following TSK edge-wise makes
      // controls_allowed flap when this car's TSK state jitters at low speed -> controlsMismatch.
      if (!volkswagen_meb_longitudinal) {
        pcm_cruise_check(cruise_engaged);
      } else {
        // carrot HKG(hyundai_common_cruise_state_check)와 동일한 레벨 방식 재주장:
        // TSK가 인게이지(3/4/5)인 동안 매 프레임 controls_allowed 유지.
        // carrot은 alternativeExperience=0이라 가속페달 rising edge마다 generic_rx_checks가
        // controls_allowed를 클리어하는데(가스 오버라이드 중 rlog로 확인), 이 재주장이 다음
        // Motor_51(20ms) 안에 복구해 controlsMismatch 누적을 막는다. TSK 지터로 인한
        // 클리어는 하지 않음(해제는 cancel/브레이크/메인off/openpilot 자체 해제 경로 유지).
        if (cruise_engaged) {
          controls_allowed = true;
        }
      }
    }

    // ACC buttons - enter/exit controls
    // Signal: GRA_ACC_01.GRA_Tip_Setzen, GRA_ACC_01.GRA_Tip_Wiederaufnahme
    if (addr == MSG_GRA_ACC_01) {
      bool set_button = GET_BIT(to_push, 16U);
      bool resume_button = GET_BIT(to_push, 19U);
      // openpilot long: enter controls on SET/RESUME falling edge (if2 original logic).
      // TSK mirroring alone can deadlock: panda blocks active ACC_18 until controls_allowed,
      // so TSK never engages -> controlsMismatch ("Controls Mismatch") 2s after engaging.
      if (volkswagen_meb_longitudinal) {
        if ((volkswagen_set_button_prev && !set_button) || (volkswagen_resume_button_prev && !resume_button)) {
          controls_allowed = acc_main_on;
        }
      }
      // Signal: GRA_ACC_01.GRA_Abbrechen (cancel)
      if (GET_BIT(to_push, 13U)) {
        controls_allowed = false;
      }
      volkswagen_set_button_prev = set_button;
      volkswagen_resume_button_prev = resume_button;
    }

    // Brake pedal switch from Motor_14
    // Signal: Motor_14.MO_Fahrer_bremst
    if (addr == MSG_MOTOR_14) {
      volkswagen_meb_brake_pedal_switch = (GET_BYTE(to_push, 3) & 0x10U) >> 4;
    }

    // 운전자 페달(MO_Fahrer_bremst)만 brake로 인정. 순정 ACC가 앞차 따라 제동할 때
    // ESC_51.Brake_Pressure가 올라가는데, 이를 brake로 보면 controls_allowed가 꺼지고
    // 재engage edge가 없어 복구 안 됨 -> Controls Mismatch로 크루즈 해제됨.
    brake_pressed = volkswagen_meb_brake_pedal_switch;

    generic_rx_checks((addr == MSG_HCA_03));
  }
}

static bool volkswagen_meb_tx_hook(const CANPacket_t *to_send) {
  // HCA_03 layout (vw_meb.dbc): RequestStatus@12:4, Power@16:8, Curvature@24:15, Curvature_VZ@39:1
  // Curvature CAN scale 6.7e-6 rad/m; 0.195 rad/m / 6.7e-6 = 29105 (from infiniteCable2 proven safety)
  const int VOLKSWAGEN_MEB_MAX_CURVATURE = 29105;

  int addr = GET_ADDR(to_send);
  bool tx = true;

  // Safety check for HCA_03 curvature command (MEB steering)
  if (addr == MSG_HCA_03) {
    // Curvature: bytes 3-4, 15 bits (bit 24..38)
    int desired_curvature = (GET_BYTE(to_send, 3) | ((GET_BYTE(to_send, 4) & 0x7FU) << 8));
    bool sign = GET_BIT(to_send, 39U);  // Curvature_VZ
    if (!sign) {
      desired_curvature *= -1;
    }
    // steer_req: RequestStatus (byte1 high nibble, bit 12..15) == 4 (active)
    bool steer_req = (((GET_BYTE(to_send, 1) >> 4) & 0x0FU) == 4U);

    if (steer_req && ((desired_curvature > VOLKSWAGEN_MEB_MAX_CURVATURE) || (desired_curvature < -VOLKSWAGEN_MEB_MAX_CURVATURE))) {
      tx = false;
    }
    // 상시조향(always-on-lateral): carrot은 횡제어를 controls_allowed 없이 허용(aol_allowed=true).
    // 따라서 ACC engage 여부와 무관하게 곡률 크기 제한만 통과하면 HCA_03 송신 허용.
    // (종방향만 controls_allowed 필요. 운전자 토크 오버라이드는 그대로 동작.)
  }

  // Safety check for ACC_18 acceleration command (MEB longitudinal)
  if (addr == MSG_ACC_18) {
    // ACC_Sollbeschleunigung_02 @24:11, scale 0.005, offset -7.22 -> compare in m/s^2 * 1000
    int desired_accel = ((((GET_BYTE(to_send, 4) & 0x7U) << 8) | GET_BYTE(to_send, 3)) * 5) - 7220;
    const LongitudinalLimits VOLKSWAGEN_MEB_LONG_LIMITS = {
      .max_accel = 2000,        // 2.0 m/s^2
      .min_accel = -3500,       // -3.5 m/s^2
      .inactive_accel = 3010,   // VW sends one increment above max range when inactive (3.01)
    };
    if (longitudinal_accel_checks(desired_accel, VOLKSWAGEN_MEB_LONG_LIMITS)) {
      tx = false;
    }
  }

  // FORCE CANCEL: only allow cancel button when controls are off
  if ((addr == MSG_GRA_ACC_01) && !controls_allowed) {
    if ((GET_BYTE(to_send, 2) & 0x9U) != 0U) {
      tx = false;
    }
  }

  return tx;
}

static int volkswagen_meb_fwd_hook(CANPacket_t *to_send) {
  const int bus_num = GET_BUS(to_send);
  const int addr = GET_ADDR(to_send);

  int bus_fwd = -1;

  switch (bus_num) {
    case 0:
      // MEB는 LH_EPS_03 대체본을 보내지 않음(MQB와 달리) → 차단 금지, 전부 카메라로 포워딩
      bus_fwd = 2;
      break;
    case 2:
      if ((addr == MSG_HCA_03) || (addr == MSG_LDW_02) || (addr == MSG_EA_02)) {
        // openpilot이 MEB 조향(HCA_03)/LKA HUD(LDW_02)/EA HUD(EA_02)를 대체 → 카메라 스톡 차단
        bus_fwd = -1;
      } else if (volkswagen_meb_longitudinal && ((addr == MSG_ACC_18) || (addr == MSG_ACC_19) || (addr == MSG_TA_01))) {
        // 종방향 제어 시 openpilot이 ACC 가감속(ACC_18)/HUD(ACC_19=MEB_ACC_01)/TA_01을 대체 → 순정 차단
        bus_fwd = -1;
      } else {
        // 나머지 확장 CAN 트래픽은 차량(bus 0)으로 포워딩
        bus_fwd = 0;
      }
      break;
    default:
      bus_fwd = -1;
      break;
  }

  return bus_fwd;
}

const safety_hooks volkswagen_meb_hooks = {
  .init = volkswagen_meb_init,
  .rx = volkswagen_meb_rx_hook,
  .tx = volkswagen_meb_tx_hook,
  .fwd = volkswagen_meb_fwd_hook,
  .get_counter = volkswagen_mqb_meb_get_counter,
  .get_checksum = volkswagen_mqb_meb_get_checksum,
  .compute_checksum = volkswagen_mqb_meb_compute_crc,
};
