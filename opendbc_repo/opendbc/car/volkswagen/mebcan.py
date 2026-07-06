from opendbc.car.volkswagen.values import VolkswagenFlags
from opendbc.car.volkswagen.mebutils import map_speed_to_acc_tempolimit
from opendbc.car.common.conversions import Conversions as CV

# ACC longitudinal constants (ported from infiniteCable2/opendbc VW MEB)
ACCEL_INACTIVE = 3.01
ACCEL_OVERRIDE = 0.00

ACC_CTRL_ERROR    = 6
ACC_CTRL_OVERRIDE = 4
ACC_CTRL_ACTIVE   = 3
ACC_CTRL_ENABLED  = 2
ACC_CTRL_DISABLED = 0

ACC_HMS_RAMP_RELEASE = 5
ACC_HMS_RELEASE      = 4
ACC_HMS_HOLD         = 1
ACC_HMS_NO_REQUEST   = 0

ACC_HUD_ERROR    = 6
ACC_HUD_OVERRIDE = 4
ACC_HUD_ACTIVE   = 3
ACC_HUD_ENABLED  = 2
ACC_HUD_DISABLED = 0

# from speed_limit_manager (sunnypilot); only the constant is needed here
PSD_TYPE_SPEED_LIMIT = 1
PSD_TYPE_CURV_SPEED = 2


def create_steering_control(packer, bus, apply_curvature, lkas_enabled, power):
  values = {
    "Curvature": abs(apply_curvature),
    "Curvature_VZ": 1 if apply_curvature > 0 and lkas_enabled else 0,
    "Power": power if lkas_enabled else 0,
    "RequestStatus": 4 if lkas_enabled else 2,
    "HighSendRate": lkas_enabled,
  }
  return packer.make_can_msg("HCA_03", bus, values)


def create_eps_update(packer, bus, eps_stock_values, ea_simulated_torque):
  values = {s: eps_stock_values[s] for s in [
    "COUNTER",
    "EPS_Lenkungstyp",
    "EPS_Berechneter_LW",
    "EPS_VZ_BLW",
    "EPS_HCA_Status",
  ]}
  values.update({
    "EPS_Lenkmoment": abs(ea_simulated_torque),
    "EPS_VZ_Lenkmoment": 1 if ea_simulated_torque < 0 else 0,
  })
  return packer.make_can_msg("LH_EPS_03", bus, values)


def create_lka_hud_control(packer, bus, ldw_stock_values, lat_active, steering_pressed, hud_alert, hud_control, sound_alert):
  display_mode = 1 if lat_active else 0

  values = {}
  if len(ldw_stock_values):
    values = {s: ldw_stock_values[s] for s in [
      "LDW_SW_Warnung_links",
      "LDW_SW_Warnung_rechts",
      "LDW_Seite_DLCTLC",
      "LDW_DLC",
      "LDW_TLC",
    ]}

  values.update({
    "LDW_Gong": sound_alert,
    "LDW_Status_LED_gelb": 1 if lat_active and steering_pressed else 0,
    "LDW_Status_LED_gruen": 1 if lat_active and not steering_pressed else 0,
    "LDW_Lernmodus_links": 3 + display_mode if hud_control.leftLaneDepart else 1 + hud_control.leftLaneVisible + display_mode,
    "LDW_Lernmodus_rechts": 3 + display_mode if hud_control.rightLaneDepart else 1 + hud_control.rightLaneVisible + display_mode,
    "LDW_Texte": hud_alert,
  })
  return packer.make_can_msg("LDW_02", bus, values)


def create_blinker_control(packer, bus, ea_hud_stock_values, ea_control_stock_values, left_blinker, right_blinker, hide_error):
  # Relay the Emergency Assist HUD (EA_02): preserves EA hands-off/blinker state and, when hide_error is set,
  # suppresses the EA error text (EA errors when its radar ethernet / HCA path is disturbed). Not the steering
  # wheel icon (that is TA_01). Matches infiniteCable2's create_blinker_control.
  values = {s: ea_hud_stock_values[s] for s in [
    "EA_Texte",
    "ACF_Lampe_Hands_Off",
    "EA_Infotainment_Anf",
    "EA_Tueren_Anf",
    "EA_Innenraumlicht_Anf",
    "zFAS_Warnblinken",
    "STP_Primaeranz",
    "EA_Bremslichtblinken",
    "EA_Blinken",
    "EA_Unknown",
  ]}

  if ea_hud_stock_values["EA_Blinken"] == 0:
    values.update({
      "EA_Blinken": 1 if left_blinker else (2 if right_blinker else ea_hud_stock_values["EA_Blinken"]),
    })

  # hide EA error (e.g. camera EA went to error because openpilot replaced its HCA steering)
  if hide_error and ea_control_stock_values.get("EA_Funktionsstatus", 0) in (0, 1, 7, 8):
    values.update({
      "EA_Texte": 0,
      "EA_Unknown": 1,
    })

  return packer.make_can_msg("EA_02", bus, values)


def create_capacitive_wheel_touch(packer, bus, lat_active, klr_stock_values):
  # Pacify VW Emergency Assist hands-on detection on capacitive steering wheel cars.
  # When lateral is active, report "hands on" touch so the car doesn't escalate (brake jolt / EA stop).
  values = {s: klr_stock_values[s] for s in [
    "COUNTER",
    "KLR_Touchintensitaet_1",
    "KLR_Touchintensitaet_2",
    "KLR_Touchintensitaet_3",
    "KLR_Touchauswertung",
  ]}
  if lat_active:
    values.update({
      "COUNTER": (klr_stock_values["COUNTER"] + 1) % 16,
      "KLR_Touchintensitaet_1": 80,
      "KLR_Touchintensitaet_2": 200,
      "KLR_Touchintensitaet_3": 10,
      "KLR_Touchauswertung": 10,
    })
  return packer.make_can_msg("KLR_01", bus, values)


def create_acc_buttons_control(packer, bus, gra_stock_values, cancel=False, resume=False, up=False, down=False):
  values = {s: gra_stock_values[s] for s in [
    "GRA_Hauptschalter",
    "GRA_Typ_Hauptschalter",
    "GRA_Codierung",
    "GRA_Tip_Stufe_2",
    "GRA_ButtonTypeInfo",
  ]}
  values.update({
    "COUNTER": (gra_stock_values["COUNTER"] + 1) % 16,
    "GRA_Abbrechen": cancel,
    "GRA_Tip_Wiederaufnahme": resume or up,
    "GRA_Tip_Setzen": down,
  })
  return packer.make_can_msg("GRA_ACC_01", bus, values)


# ***** Longitudinal (ACC) control - ported from infiniteCable2/opendbc VW MEB ***** #

def acc_control_value(main_switch_on, acc_faulted, long_active, override):
  if acc_faulted:
    acc_control = ACC_CTRL_ERROR
  elif long_active:
    if override:
      acc_control = ACC_CTRL_OVERRIDE
    else:
      acc_control = ACC_CTRL_ACTIVE
  elif main_switch_on:
    acc_control = ACC_CTRL_ENABLED
  else:
    acc_control = ACC_CTRL_DISABLED
  return acc_control


def acc_hold_type(main_switch_on, acc_faulted, long_active, starting, stopping, esp_hold, override, override_begin, long_disabling):
  if acc_faulted:
    acc_hold_type = ACC_HMS_NO_REQUEST
  elif not long_active:
    if long_disabling:
      acc_hold_type = ACC_HMS_RAMP_RELEASE  # ramp release right after disabling (prevents car error with EPB at low speed)
    else:
      acc_hold_type = ACC_HMS_NO_REQUEST
  elif override:
    if override_begin:
      acc_hold_type = ACC_HMS_RAMP_RELEASE  # ramp release at begin of override (prevents car error with EPB at low speed)
    else:
      acc_hold_type = ACC_HMS_NO_REQUEST
  elif starting:
    acc_hold_type = ACC_HMS_RELEASE
  elif stopping or esp_hold:
    acc_hold_type = ACC_HMS_HOLD
  else:
    acc_hold_type = ACC_HMS_NO_REQUEST
  return acc_hold_type


def create_acc_accel_control(packer, bus, CP, acc_type, acc_enabled, upper_jerk, lower_jerk, upper_control_limit, lower_control_limit,
                             accel, acc_control, acc_hold_type, stopping, starting, esp_hold, speed, override, travel_assist_available):
  commands = []

  # ACC_Anhalteweg: when stopping, MEB values <> 0 can execute a hard brake if target too close; MQBevo value 0 => hard brake
  terminal_rollout = 0.5 if CP.flags & VolkswagenFlags.MQB_EVO else 0

  full_stop          = stopping and esp_hold
  full_stop_no_start = esp_hold and not starting
  actually_stopping  = stopping and not esp_hold

  if acc_enabled:
    if override:  # the car expects a non-inactive accel while overriding
      acceleration = ACCEL_OVERRIDE
    elif full_stop:
      acceleration = ACCEL_INACTIVE  # newer gen >2024 errors on non-neutral value at full stop
    else:
      acceleration = accel
  else:
    acceleration = ACCEL_INACTIVE

  values = {
    "ACC_Typ":                    acc_type,
    "ACC_Status_ACC":             acc_control,
    "ACC_StartStopp_Info":        acc_enabled,
    "ACC_Sollbeschleunigung_02":  acceleration,
    "ACC_zul_Regelabw_unten":     lower_control_limit if acc_control in (ACC_CTRL_ACTIVE, ACC_CTRL_OVERRIDE) and not full_stop_no_start else 0,
    "ACC_zul_Regelabw_oben":      upper_control_limit if acc_control in (ACC_CTRL_ACTIVE, ACC_CTRL_OVERRIDE) and not full_stop_no_start else 0,
    "ACC_neg_Sollbeschl_Grad_02": lower_jerk if acc_control in (ACC_CTRL_ACTIVE, ACC_CTRL_OVERRIDE) and not full_stop_no_start else 0,
    "ACC_pos_Sollbeschl_Grad_02": upper_jerk if acc_control in (ACC_CTRL_ACTIVE, ACC_CTRL_OVERRIDE) and not full_stop_no_start else 0,
    "ACC_Anfahren":               starting,
    "ACC_Anhalten":               1 if actually_stopping else 0,
    "ACC_Anhalteweg":             terminal_rollout if actually_stopping else 20.46,
    "ACC_Anforderung_HMS":        acc_hold_type,
    "ACC_AKTIV_regelt":           1 if acc_control == ACC_CTRL_ACTIVE else 0,
    "Speed":                      speed,
    "SET_ME_0XFE":                0xFE,
    "SET_ME_0X1":                 0x1,
    "SET_ME_0X9":                 0x9,
  }

  if CP.flags & VolkswagenFlags.MEB_GEN2:
    values.update({
      "SET_ME_0x2FE": 0x2FE,
    })

  commands.append(packer.make_can_msg("ACC_18", bus, values))

  if travel_assist_available:
    values_ta = {
      "Travel_Assist_Status":    4 if acc_enabled else 2,
      "Travel_Assist_Request":   0,
      "Travel_Assist_Available": 1,
    }
    commands.append(packer.make_can_msg("TA_01", bus, values_ta))

  return commands


def acc_hud_status_value(main_switch_on, acc_faulted, long_active, override):
  if acc_faulted:
    acc_hud_control = ACC_HUD_ERROR
  elif long_active:
    if override:
      acc_hud_control = ACC_HUD_OVERRIDE
    else:
      acc_hud_control = ACC_HUD_ACTIVE
  elif main_switch_on:
    acc_hud_control = ACC_HUD_ENABLED
  else:
    acc_hud_control = ACC_HUD_DISABLED
  return acc_hud_control


def acc_hud_event(acc_hud_control, esp_hold, speed_limit_predicative, speed_limit_predicative_type, speed_limit):
  acc_event = 0
  if esp_hold and acc_hud_control == ACC_HUD_ACTIVE:
    acc_event = 3  # acc ready message at standstill
  elif acc_hud_control in (ACC_HUD_ACTIVE, ACC_HUD_OVERRIDE) and speed_limit_predicative:
    if speed_limit_predicative_type == PSD_TYPE_CURV_SPEED:
      acc_event = 6  # acc limited by curve (predicative)
    else:
      acc_event = 4  # acc limited by speed limit by nav (predicative)
  elif acc_hud_control in (ACC_HUD_ACTIVE, ACC_HUD_OVERRIDE) and speed_limit:
    acc_event = 5  # acc limited by speed limit by camera (recently detected)
  return acc_event


def get_desired_gap(distance_bars, desired_gap, current_gap_signal):
  gap = 0
  if distance_bars == current_gap_signal:
    gap = desired_gap
  return gap


def create_acc_hud_control(packer, bus, acc_control, set_speed, lead_visible, distance_bars, show_distance_bars, esp_hold, distance, desired_gap, fcw_alert, acc_event, speed_limit, event_speed_kph=0):
  values = {
    "ACC_Status_ACC":                acc_control,
    "ACC_Tempolimit":                map_speed_to_acc_tempolimit(speed_limit) if acc_control in (ACC_HUD_ACTIVE, ACC_HUD_OVERRIDE) else 0,
    "ACC_Wunschgeschw_02":           set_speed if set_speed < 250 else 327.36,
    "ACC_Gesetzte_Zeitluecke":       distance_bars,
    "ACC_Display_Prio":              0 if fcw_alert and acc_control in (ACC_HUD_ACTIVE, ACC_HUD_OVERRIDE) else 1,
    "ACC_Optischer_Fahrerhinweis":   1 if fcw_alert and acc_control in (ACC_HUD_ACTIVE, ACC_HUD_OVERRIDE) else 0,
    "ACC_Akustischer_Fahrerhinweis": 3 if fcw_alert and acc_control in (ACC_HUD_ACTIVE, ACC_HUD_OVERRIDE) else 0,
    "ACC_Texte_Zusatzanz_02":        11 if fcw_alert and acc_control in (ACC_HUD_ACTIVE, ACC_HUD_OVERRIDE) else 0,
    "ACC_Abstandsindex_02":          569,
    "ACC_EGO_Fahrzeug":              2 if fcw_alert and acc_control in (ACC_HUD_ACTIVE, ACC_HUD_OVERRIDE) else (1 if acc_control == ACC_HUD_ACTIVE else 0),
    "Lead_Type_Detected":            1 if lead_visible else 0,
    "Lead_Type":                     3 if lead_visible else 0,
    "Lead_Distance":                 distance if lead_visible else 0,
    "ACC_Enabled":                   1 if acc_control in (ACC_HUD_ACTIVE, ACC_HUD_OVERRIDE) else 0,
    "ACC_Standby_Override":          1 if acc_control != ACC_HUD_ACTIVE else 0,
    "Street_Color":                  1 if acc_control in (ACC_HUD_ACTIVE, ACC_HUD_OVERRIDE) else 0,
    "Lead_Brightness":               3 if acc_control == ACC_HUD_ACTIVE else 0,
    "ACC_Events":                    acc_event,
    # 커브(6)/교차로(9) 이벤트일 때 계기판에 표시되는 목표속도 (카메라 표지판은 ACC_Tempolimit 사용).
    # tjddyd0130/opendbc a6869ce 검증: if2 ACC_19의 140|10 (구 SET_ME_0X3FF = 미사용시 0)
    "ACC_Event_Wunschgeschw":        event_speed_kph if event_speed_kph > 0 else speed_limit * CV.MS_TO_KPH,
    "Zeitluecke_1":                  get_desired_gap(distance_bars, desired_gap, 1),
    "Zeitluecke_2":                  get_desired_gap(distance_bars, desired_gap, 2),
    "Zeitluecke_3":                  get_desired_gap(distance_bars, desired_gap, 3),
    "Zeitluecke_4":                  get_desired_gap(distance_bars, desired_gap, 4),
    "Zeitluecke_5":                  get_desired_gap(distance_bars, desired_gap, 5),
    "Zeitluecke_Farbe":              1 if acc_control in (ACC_HUD_ENABLED, ACC_HUD_ACTIVE, ACC_HUD_OVERRIDE) else 0,
    "ACC_Anzeige_Zeitluecke":        show_distance_bars if acc_control != ACC_HUD_DISABLED else 0,
    "SET_ME_0X1":                    0x1,
    "SET_ME_0X6A":                   0x6A,
    "SET_ME_0XFFFF":                 0xFFFF,
    "SET_ME_0X7FFF":                 0x7FFF,
  }
  # carrot DBC names this message MEB_ACC_01 (== infiniteCable2 ACC_19, BO_ 768)
  return packer.make_can_msg("MEB_ACC_01", bus, values)
