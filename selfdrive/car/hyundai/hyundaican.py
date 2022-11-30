import copy

import crcmod
from selfdrive.car.hyundai.values import CAR, CHECKSUM, CAMERA_SCC_CAR
from selfdrive.car.hyundai.values import FEATURES

hyundai_checksum = crcmod.mkCrcFun(0x11D, initCrc=0xFD, rev=False, xorOut=0xdf)

def create_lkas11(packer, frame, car_fingerprint, send_lfa_mfa, apply_steer, steer_req,
                  torque_fault, lkas11, sys_warning, sys_state, enabled,
                  left_lane, right_lane,
                  left_lane_depart, right_lane_depart):
  values = lkas11
  values["CF_Lkas_LdwsSysState"] = sys_state
  values["CF_Lkas_SysWarning"] = 0 # ajouatom: 계기판에 안나오게함..   #3 if sys_warning else 0
  values["CF_Lkas_LdwsLHWarning"] = left_lane_depart
  values["CF_Lkas_LdwsRHWarning"] = right_lane_depart
  values["CR_Lkas_StrToqReq"] = apply_steer
  values["CF_Lkas_ActToi"] = steer_req
  values["CF_Lkas_ToiFlt"] = torque_fault  # seems to allow actuation on CR_Lkas_StrToqReq
  values["CF_Lkas_MsgCount"] = frame % 0x10

  if send_lfa_mfa:
  #if car_fingerprint in (CAR.SONATA, CAR.PALISADE, CAR.KIA_NIRO_EV, CAR.KIA_NIRO_HEV_2021, CAR.SANTA_FE,
  #                       CAR.IONIQ_EV_2020, CAR.IONIQ_PHEV, CAR.KIA_SELTOS, CAR.ELANTRA_2021, CAR.GENESIS_G70_2020,
  #                       CAR.ELANTRA_HEV_2021, CAR.SONATA_HYBRID, CAR.KONA_EV, CAR.KONA_HEV, CAR.KONA_EV_2022,
  #                       CAR.SANTA_FE_2022, CAR.KIA_K5_2021, CAR.IONIQ_HEV_2022, CAR.SANTA_FE_HEV_2022,
  #                       CAR.SANTA_FE_PHEV_2022, CAR.KIA_STINGER_2022, CAR.NEXO):
    values["CF_Lkas_LdwsActivemode"] = int(left_lane) + (int(right_lane) << 1)
    values["CF_Lkas_LdwsOpt_USM"] = 2

    # FcwOpt_USM 5 = Orange blinking car + lanes
    # FcwOpt_USM 4 = Orange car + lanes
    # FcwOpt_USM 3 = Green blinking car + lanes
    # FcwOpt_USM 2 = Green car + lanes
    # FcwOpt_USM 1 = White car + lanes
    # FcwOpt_USM 0 = No car + lanes
    values["CF_Lkas_FcwOpt_USM"] = 2 if enabled else 1

    # SysWarning 4 = keep hands on wheel
    # SysWarning 5 = keep hands on wheel (red)
    # SysWarning 6 = keep hands on wheel (red) + beep
    # Note: the warning is hidden while the blinkers are on
    values["CF_Lkas_SysWarning"] = 0 #4 if sys_warning else 0
  # Likely cars lacking the ability to show individual lane lines in the dash
  elif car_fingerprint in (CAR.KIA_OPTIMA_G4, CAR.KIA_OPTIMA_G4_FL):
    # SysWarning 4 = keep hands on wheel + beep
    values["CF_Lkas_SysWarning"] = 4 if sys_warning else 0

    # SysState 0 = no icons
    # SysState 1-2 = white car + lanes
    # SysState 3 = green car + lanes, green steering wheel
    # SysState 4 = green car + lanes
    values["CF_Lkas_LdwsSysState"] = 3 if enabled else 1
    values["CF_Lkas_LdwsOpt_USM"] = 2  # non-2 changes above SysState definition

    # these have no effect
    values["CF_Lkas_LdwsActivemode"] = 0
    values["CF_Lkas_FcwOpt_USM"] = 0

  elif car_fingerprint == CAR.HYUNDAI_GENESIS:
    # This field is actually LdwsActivemode
    # Genesis and Optima fault when forwarding while engaged
    values["CF_Lkas_LdwsActivemode"] = 2
    #values["CF_Lkas_SysWarning"] = lkas11["CF_Lkas_SysWarning"] 

  dat = packer.make_can_msg("LKAS11", 0, values)[2]

  if car_fingerprint in CHECKSUM["crc8"]:
    # CRC Checksum as seen on 2019 Hyundai Santa Fe
    dat = dat[:6] + dat[7:8]
    checksum = hyundai_checksum(dat)
  elif car_fingerprint in CHECKSUM["6B"]:
    # Checksum of first 6 Bytes, as seen on 2018 Kia Sorento
    checksum = sum(dat[:6]) % 256
  else:
    # Checksum of first 6 Bytes and last Byte as seen on 2018 Kia Stinger
    checksum = (sum(dat[:6]) + dat[7]) % 256

  values["CF_Lkas_Chksum"] = checksum

  return packer.make_can_msg("LKAS11", 0, values)


def create_clu11(packer, frame, clu11, button, car_fingerprint):
  values = clu11
  values["CF_Clu_CruiseSwState"] = button
  values["CF_Clu_AliveCnt1"] = frame % 0x10
  # send buttons to camera on camera-scc based cars
  bus = 2 if car_fingerprint in CAMERA_SCC_CAR else 0
  return packer.make_can_msg("CLU11", bus, values)

def create_clu11_button(packer, frame, clu11, button, car_fingerprint):
  values = clu11
  values["CF_Clu_CruiseSwState"] = button
  #values["CF_Clu_AliveCnt1"] = frame % 0x10
  values["CF_Clu_AliveCnt1"] = (values["CF_Clu_AliveCnt1"] + 1) % 0x10
  # send buttons to camera on camera-scc based cars
  bus = 2 if car_fingerprint in CAMERA_SCC_CAR else 0
  return packer.make_can_msg("CLU11", bus, values)


def create_lfahda_mfc(packer, CC, blinking_signal):
  values = {
    "LFA_Icon_State": 3 if CC.latOverride else 2 if CC.latActive else 1 if CC.latEnabled else 0,
    "HDA_Active": 1 if CC.activeHda > 0 else 0,
    "HDA_Icon_State": 0 if CC.activeHda > 1 and blinking_signal else 2 if CC.activeHda > 0 else 0,
    "HDA_VSetReq": 1 if CC.activeHda > 0 else 0, #enabled,
    "HDA_USM" : 2,
    "HDA_Icon_Wheel" : 1 if CC.latActive else 0,
    "HDA_Chime" : 1 if CC.latEnabled else 0,
}
  # VAL_ 1157 LFA_Icon_State 0 "no_wheel" 1 "white_wheel" 2 "green_wheel" 3 "green_wheel_blink";
  # VAL_ 1157 LFA_SysWarning 0 "no_message" 1 "switching_to_hda" 2 "switching_to_scc" 3 "lfa_error" 4 "check_hda" 5 "keep_hands_on_wheel_orange" 6 "keep_hands_on_wheel_red";
  # VAL_ 1157 HDA_Icon_State 0 "no_hda" 1 "white_hda" 2 "green_hda";
  # VAL_ 1157 HDA_SysWarning 0 "no_message" 1 "driving_convenience_systems_cancelled" 2 "highway_drive_assist_system_cancelled";
  return packer.make_can_msg("LFAHDA_MFC", 0, values)

def create_acc_commands_mix_scc(CP, packer, enabled, accel, upper_jerk, idx, hud_control, set_speed, stopping, CC, CS, softHoldMode):
  lead_visible = hud_control.leadVisible
  cruiseGap = hud_control.cruiseGap
  softHold = hud_control.softHold
  softHoldInfo = softHold  #계기판에 표시안하게 하려면 False로 하면됨~
  long_override = CC.cruiseControl.override
  brakePressed = CS.out.brakePressed
  longEnabled = CC.longEnabled
  longActive = CC.longActive
  radarAlarm = hud_control.radarAlarm
  stopReq = 1 if stopping and longEnabled else 0
  accel = accel if longEnabled and not long_override else 0.0
  d = hud_control.objDist
  objGap = 0 if d == 0 else 2 if d < 25 else 3 if d < 40 else 4 if d < 70 else 5 
  objGap2 = 0 if objGap == 0 else 2 if hud_control.objRelSpd < -0.1 else 1

  driverOverride =  CS.out.driverOverride  #1:gas, 2:braking, 0: normal
  if enabled:
    scc12_accMode = 2 if long_override else 0 if brakePressed else 1 if longActive else 0 #Brake, Accel, LongActiveUser < 0
    scc14_accMode = 4 if long_override or not longEnabled else 4 if brakePressed else 1 if longActive else 0
    if CS.out.brakeHoldActive: # autoHold가 작동한경우..
      scc12_accMode = 0
      scc14_accMode = 4
    elif softHold and brakePressed and longEnabled and softHoldMode == 2: #longActive:
      scc12_accMode = 1
      scc14_accMode = 1
      stopReq = 1
    comfortBandUpper = 0.9 + accel * 0.2
    comfortBandLower = 0.8 + accel * 0.2
    jerkUpperLimit = upper_jerk
    jerkLowerLimit = upper_jerk #5.0
  else:
    scc12_accMode = 0
    scc14_accMode = 0
    comfortBandUpper = 0.0
    comfortBandLower = 0.0
    jerkUpperLimit = upper_jerk
    jerkLowerLimit = upper_jerk #5.0
    stopReq = 0

  makeNewCommands = True if CP.sccBus == 0 else False
  commands = []
  if makeNewCommands:
    scc11_values = {
    "MainMode_ACC": 1 if enabled else 0 ,
    "TauGapSet": cruiseGap,
    "VSetDis": set_speed if longEnabled else 0,
    "AliveCounterACC": idx % 0x10,
    "SCCInfoDisplay" : 3 if longActive and radarAlarm else 4 if longActive and softHoldInfo else 0 if enabled else 0,   #2: 크루즈 선택, 3: 전방상황주의, 4: 출발준비 <= 주의 2를 선택하면... 선행차아이콘이 안나옴.
    "ObjValid": 1 if lead_visible else 0, # close lead makes controls tighter
    "ACC_ObjStatus": 1 if lead_visible else 0, # close lead makes controls tighter
    #"ACC_ObjLatPos": 0,
    "ACC_ObjRelSpd": hud_control.objRelSpd,
    "ACC_ObjDist": hud_control.objDist, # close lead makes controls tighter
    "DriverAlertDisplay": 0,
    }
    commands.append(packer.make_can_msg("SCC11", 0, scc11_values))
  else:
    values = CS.scc11
    values["MainMode_ACC"] = 1 if enabled else 0 
    values["TauGapSet"] = cruiseGap
    values["VSetDis"] = set_speed if longEnabled else 0
    values["AliveCounterACC"] = idx % 0x10
    #values["SCCInfoDisplay"] = 4 if longEnabled and softHoldInfo else 3 if longEnabled and radarAlarm else 2 if enabled else 0   #3: 전방상황주의, 4: 출발준비
    values["SCCInfoDisplay"] = 3 if longActive and radarAlarm else 4 if longActive and softHoldInfo else 0 if enabled else 0   #2: 크루즈 선택, 3: 전방상황주의, 4: 출발준비
    values["ObjValid"] = 1 if lead_visible else 0
    values["ACC_ObjStatus"] = 1 if lead_visible else 0
    #values["ACC_ObjLatPos"] = 0
    values["ACC_ObjRelSpd"] = hud_control.objRelSpd
    values["ACC_ObjDist"] = hud_control.objDist
    values["DriverAlertDisplay"] = 0
    commands.append(packer.make_can_msg("SCC11", 0, values))

  # SCC12.ACCMode: Init: 0, Brake: 0, Accel:2, Cruise: 1   KONA_EV에서 측정함.
  if makeNewCommands:
    scc12_values = {
      "ACCMode": scc12_accMode, #0 if brakePressed else 2 if enabled and long_override else 1 if longEnabled else 0,
      "StopReq": stopReq,
      "aReqRaw": accel,
      "aReqValue": accel, # stock ramps up and down respecting jerk limit until it reaches aReqRaw
      "CR_VSM_Alive": idx % 0xF,
    }
    scc12_dat = packer.make_can_msg("SCC12", 0, scc12_values)[2]
    scc12_values["CR_VSM_ChkSum"] = 0x10 - sum(sum(divmod(i, 16)) for i in scc12_dat) % 0x10

    commands.append(packer.make_can_msg("SCC12", 0, scc12_values))
  else:
    values = CS.scc12
    values["ACCMode"] = scc12_accMode #0 if brakePressed else 2 if enabled and long_override else 1 if longEnabled else 0
    values["StopReq"] = stopReq
    values["aReqRaw"] = accel
    values["aReqValue"] = accel

    values["CR_VSM_ChkSum"] = 0
    values["CR_VSM_Alive"] = idx % 0xF
    scc12_dat = packer.make_can_msg("SCC12", 0, values)[2]
    values["CR_VSM_ChkSum"] = 0x10 - sum(sum(divmod(i, 16)) for i in scc12_dat) % 0x10

    commands.append(packer.make_can_msg("SCC12", 0, values))

  # SCC14.ACCMode: Init: 0, Brake: 4, Accel:2, Cruise: 1   KONA_EV에서 측정함.
  if makeNewCommands:
    scc14_values = {
      "ComfortBandUpper": comfortBandUpper, # stock usually is 0 but sometimes uses higher values
      "ComfortBandLower": comfortBandLower, # stock usually is 0 but sometimes uses higher values
      "JerkUpperLimit": jerkUpperLimit, # stock usually is 1.0 but sometimes uses higher values
      "JerkLowerLimit": jerkLowerLimit, # stock usually is 0.5 but sometimes uses higher values
      "ACCMode": scc14_accMode, #0 if not enabled else 4 if brakePressed else 2 if enabled and long_override else 1 if longEnabled else 4, # stock will always be 4 instead of 0 after first disengage
      "ObjGap": objGap,
      "ObjGap2" : objGap2,
    }
    commands.append(packer.make_can_msg("SCC14", 0, scc14_values))
  else:
    values = CS.scc14
    values["ComfortBandUpper"] = comfortBandUpper
    values["ComfortBandLower"] = comfortBandLower
    values["JerkUpperLimit"] = jerkUpperLimit
    values["JerkLowerLimit"] = jerkLowerLimit
    values["ACCMode"] = scc14_accMode #0 if not enabled else 4 if brakePressed else 2 if enabled and long_override else 1 if longEnabled else 4 # stock will always be 4 instead of 0 after first disengage
    values["ObjGap"] = objGap
    values["ObjGap2"] = objGap2
    commands.append(packer.make_can_msg("SCC14", 0, values))

  if makeNewCommands:
    fca11_values = {
      "CR_FCA_Alive": idx % 0xF,
      "PAINT1_Status": 1,
      "FCA_DrvSetStatus": 1,
      "FCA_Status": 1, # AEB disabled
    }
    fca11_dat = packer.make_can_msg("FCA11", 0, fca11_values)[2]
    fca11_values["CR_FCA_ChkSum"] = hyundai_checksum(fca11_dat[:7])
    commands.append(packer.make_can_msg("FCA11", 0, fca11_values))

  return commands

def create_acc_commands(packer, enabled, accel, upper_jerk, idx, car_fingerprint, lead_visible, set_speed, stopping, long_override):
  commands = []

  scc11_values = {
    "MainMode_ACC": 1,
    "TauGapSet": 4,
    "VSetDis": set_speed if enabled else 0,
    "AliveCounterACC": idx % 0x10,
    "ObjValid": 1, # close lead makes controls tighter
    "ACC_ObjStatus": 1, # close lead makes controls tighter
    "ACC_ObjLatPos": 0,
    "ACC_ObjRelSpd": 0,
    "ACC_ObjDist": 1, # close lead makes controls tighter
    }
  commands.append(packer.make_can_msg("SCC11", 0, scc11_values))

  scc12_values = {
    "ACCMode": 2 if enabled and long_override else 1 if enabled else 0,
    "StopReq": 1 if stopping else 0,
    "aReqRaw": accel,
    "aReqValue": accel,  # stock ramps up and down respecting jerk limit until it reaches aReqRaw
    "CR_VSM_Alive": idx % 0xF,
  }
  scc12_dat = packer.make_can_msg("SCC12", 0, scc12_values)[2]
  scc12_values["CR_VSM_ChkSum"] = 0x10 - sum(sum(divmod(i, 16)) for i in scc12_dat) % 0x10

  commands.append(packer.make_can_msg("SCC12", 0, scc12_values))

  scc14_values = {
    "ComfortBandUpper": 0.0, # stock usually is 0 but sometimes uses higher values
    "ComfortBandLower": 0.0, # stock usually is 0 but sometimes uses higher values
    "JerkUpperLimit": upper_jerk, # stock usually is 1.0 but sometimes uses higher values
    "JerkLowerLimit": 5.0, # stock usually is 0.5 but sometimes uses higher values
    "ACCMode": 2 if enabled and long_override else 1 if enabled else 4, # stock will always be 4 instead of 0 after first disengage
    "ObjGap": 2 if lead_visible else 0, # 5: >30, m, 4: 25-30 m, 3: 20-25 m, 2: < 20 m, 0: no lead
    "ObjGap2" : 2 if lead_visible else 0,
  }
  commands.append(packer.make_can_msg("SCC14", 0, scc14_values))

  # note that some vehicles most likely have an alternate checksum/counter definition
  # https://github.com/commaai/opendbc/commit/9ddcdb22c4929baf310295e832668e6e7fcfa602
  fca11_values = {
    "CR_FCA_Alive": idx % 0xF,
    "PAINT1_Status": 1,
    "FCA_DrvSetStatus": 1,
    "FCA_Status": 1, # AEB disabled
  }
  fca11_dat = packer.make_can_msg("FCA11", 0, fca11_values)[2]
  fca11_values["CR_FCA_ChkSum"] = hyundai_checksum(fca11_dat[:7])
  commands.append(packer.make_can_msg("FCA11", 0, fca11_values))

  return commands

def create_acc_opt(CP, CS, packer):
  commands = []
  scc13_values = {
    "SCCDrvModeRValue": 3 if CP.carFingerprint in (CAR.KONA_EV, CAR.SANTA_FE_HEV_2022) else 2,  #KONA_EV는 3이네?, 
    "SCC_Equip": 1,
    "Lead_Veh_Dep_Alert_USM": 2,
  }
  commands.append(packer.make_can_msg("SCC13", 0, scc13_values))

  fca12_values = {
    "FCA_DrvSetState": 2,
    "FCA_USM": 1, # AEB disabled
  }
  commands.append(packer.make_can_msg("FCA12", 0, fca12_values))

  return commands

def create_acc_opt_copy(CP, CS, packer):
  return packer.make_can_msg("SCC13", 0, CS.scc13)

def create_frt_radar_opt(packer):
  frt_radar11_values = {
    "CF_FCA_Equip_Front_Radar": 1,
  }
  return packer.make_can_msg("FRT_RADAR11", 0, frt_radar11_values)

def create_clu11_mdps(packer, frame, clu11, button, car_fingerprint, speed):
  values = clu11
  values["CF_Clu_CruiseSwState"] = button
  values["CF_Clu_AliveCnt1"] = frame % 0x10
  values["CF_Clu_Vanz"] = speed
  return packer.make_can_msg("CLU11", 1, values)


def create_mdps12(packer, frame, mdps12):
  values = mdps12
  values["CF_Mdps_ToiActive"] = 0
  values["CF_Mdps_ToiUnavail"] = 1
  values["CF_Mdps_MsgCount2"] = frame % 0x100
  values["CF_Mdps_Chksum2"] = 0

  dat = packer.make_can_msg("MDPS12", 2, values)[2]
  checksum = sum(dat) % 256
  values["CF_Mdps_Chksum2"] = checksum

  return packer.make_can_msg("MDPS12", 2, values)
