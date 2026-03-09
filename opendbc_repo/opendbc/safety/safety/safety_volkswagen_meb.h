#pragma once

#include "safety_declarations.h"
#include "safety_volkswagen_common.h"

static bool volkswagen_meb_brake_pressed = false;


// MEB-specific message IDs
#define MSG_ESC_51           0x0FC   // RX from ESC, for wheel speeds
#define MSG_QFK_01           0x13D   // RX from QFK, for lateral control status
#define MSG_VMM_02           0x139   // RX from VMM, for ESP hold status
#define MSG_MOTOR_51         0x10B   // RX from ECU, for ACC status
#define MSG_MOTOR_54         0x14C   // RX from ECU, for gas pedal input
#define MSG_HCA_03           0x303   // TX by OP, lateral curvature control
#define MSG_EA_01            0x1A4   // TX by OP, Emergency Assist control
#define MSG_EA_02            0x1F0   // TX by OP, Emergency Assist HUD


static safety_config volkswagen_meb_init(uint16_t param) {
  // MEB uses angle control, not torque control, so different TX messages
  static const CanMsg VOLKSWAGEN_MEB_TX_MSGS[] = {{MSG_HCA_03, 0, 24}, {MSG_LDW_02, 0, 8},
                                                   {MSG_EA_01, 0, 8}, {MSG_EA_02, 0, 8}};

  static RxCheck volkswagen_meb_rx_checks[] = {
    {.msg = {{MSG_LH_EPS_03, 0, 8, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},
    {.msg = {{MSG_ESC_51, 0, 48, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{MSG_QFK_01, 0, 32, .ignore_checksum = true, .ignore_counter = true, .frequency = 100U}, { 0 }, { 0 }}},
    {.msg = {{MSG_VMM_02, 0, 32, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_51, 0, 32, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_54, 0, 32, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_14, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 10U}, { 0 }, { 0 }}},
    {.msg = {{MSG_GRA_ACC_01, 0, 8, .max_counter = 15U, .frequency = 33U}, { 0 }, { 0 }}},
  };

  UNUSED(param);

  volkswagen_set_button_prev = false;
  volkswagen_resume_button_prev = false;
  volkswagen_meb_brake_pressed = false;

  gen_crc_lookup_table_8(0x2F, volkswagen_crc8_lut_8h2f);
  return BUILD_SAFETY_CFG(volkswagen_meb_rx_checks, VOLKSWAGEN_MEB_TX_MSGS);
}

static void volkswagen_meb_rx_hook(const CANPacket_t *to_push) {
  if (GET_BUS(to_push) == 0U) {
    int addr = GET_ADDR(to_push);

    // Update in-motion state by sampling wheel speeds
    // Signal: ESC_51.VL_Radgeschw, VR_Radgeschw, HL_Radgeschw, HR_Radgeschw
    if (addr == MSG_ESC_51) {
      // Check all wheel speeds for any movement
      // HL_Radgeschw: bit 64 = bytes 8-9
      // HR_Radgeschw: bit 80 = bytes 10-11
      // VL_Radgeschw: bit 96 = bytes 12-13
      // VR_Radgeschw: bit 112 = bytes 14-15
      uint32_t fl = GET_BYTES(to_push, 8, 2);
      uint32_t fr = GET_BYTES(to_push, 10, 2);
      uint32_t rl = GET_BYTES(to_push, 12, 2);
      uint32_t rr = GET_BYTES(to_push, 14, 2);
      vehicle_moving = (fl + fr + rl + rr) > 0U;
    }

    // Update driver input torque samples
    // Signal: LH_EPS_03.EPS_Lenkmoment (absolute torque)
    // Signal: LH_EPS_03.EPS_VZ_Lenkmoment (direction)
    if (addr == MSG_LH_EPS_03) {
      int torque_driver_new = GET_BYTE(to_push, 5) | ((GET_BYTE(to_push, 6) & 0x3U) << 8);
      int sign = (GET_BYTE(to_push, 6) & 0x80U) >> 7;
      if (sign == 1) {
        torque_driver_new *= -1;
      }
      update_sample(&torque_driver, torque_driver_new);
    }

    if (addr == MSG_MOTOR_51) {
      // When using stock ACC, enter controls on rising edge of stock ACC engage, exit on disengage
      // Always exit controls on main switch off
      // Signal: Motor_51.TSK_Status
      int acc_status = (GET_BYTE(to_push, 3) & 0x7U);
      bool cruise_engaged = (acc_status == 3) || (acc_status == 4) || (acc_status == 5);
      acc_main_on = cruise_engaged || (acc_status == 2);

      // MEB does not support openpilot longitudinal control yet (dashcamOnly = True)
      pcm_cruise_check(cruise_engaged);

      if (!acc_main_on) {
        controls_allowed = false;
      }
    }

    if (addr == MSG_GRA_ACC_01) {
      // Always exit controls on rising edge of Cancel
      // Signal: GRA_ACC_01.GRA_Abbrechen
      if (GET_BIT(to_push, 13U)) {
        controls_allowed = false;
      }
    }

    // Signal: Motor_54.Accelerator_Pressure
    if (addr == MSG_MOTOR_54) {
      gas_pressed = ((GET_BYTES(to_push, 0, 4) >> 12) & 0xFFU) != 0U;
    }

    // Signal: Motor_14.MO_Fahrer_bremst
    if (addr == MSG_MOTOR_14) {
      volkswagen_meb_brake_pressed = GET_BIT(to_push, 28U);
    }

    brake_pressed = volkswagen_meb_brake_pressed;

    generic_rx_checks((addr == MSG_HCA_03));
  }
}

static bool volkswagen_meb_tx_hook(const CANPacket_t *to_send) {
  // MEB uses curvature control instead of torque control
  const AngleSteeringLimits VOLKSWAGEN_MEB_STEERING_LIMITS = {
    .angle_deg_to_can = 100,           // 1 unit = 0.01 rad/m
    .angle_rate_up_lookup = {
      {0., 5., 15.},
      {5., .8, .25}
    },
    .angle_rate_down_lookup = {
      {0., 5., 15.},
      {5., 3.5, .8}
    },
    .max_angle = 100,                  // 1.0 rad/m, MEB allows up to ~1 rad/m curvature
    .max_angle_error = 50,             // 0.5 rad/m error tolerance
    .angle_error_min_speed = 10.0,     // m/s
    .enforce_angle_error = false,      // Don't enforce for now
    .angle_is_curvature = true,        // This is curvature, not steering angle
    .inactive_angle_is_zero = true,    // Must send 0 when inactive
  };

  int addr = GET_ADDR(to_send);
  bool tx = true;

  // Safety check for HCA_03 curvature control
  // Signal: HCA_03.Curvature (absolute curvature in rad/m * 100)
  // Signal: HCA_03.Curvature_VZ (direction: 0 = left, 1 = right)
  if (addr == MSG_HCA_03) {
    int desired_curvature = GET_BYTE(to_send, 0) | ((GET_BYTE(to_send, 1) & 0x7FU) << 8);
    bool sign = GET_BIT(to_send, 15U);
    if (sign) {
      desired_curvature *= -1;
    }

    bool steer_req = GET_BIT(to_send, 16U);  // RequestStatus field: 4 = active, 2 = inactive

    if (steer_angle_cmd_checks(desired_curvature, steer_req, VOLKSWAGEN_MEB_STEERING_LIMITS)) {
      tx = false;
    }
  }

  return tx;
}

static int volkswagen_meb_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;

  switch (bus_num) {
    case 0:
      if (addr == MSG_LH_EPS_03) {
        // openpilot needs to replace apparent driver steering input torque to pacify VW Emergency Assist
        bus_fwd = -1;
      } else {
        // Forward all remaining traffic from Extended CAN onward
        bus_fwd = 2;
      }
      break;
    case 2:
      if ((addr == MSG_HCA_03) || (addr == MSG_LDW_02) || (addr == MSG_EA_01) || (addr == MSG_EA_02)) {
        // openpilot takes over lateral control and Emergency Assist from the camera
        bus_fwd = -1;
      } else {
        // Forward all remaining traffic from Extended CAN devices to J533 gateway
        bus_fwd = 0;
      }
      break;
    default:
      // No other buses should be in use; fallback to do-not-forward
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
