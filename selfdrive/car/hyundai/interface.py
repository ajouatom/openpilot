from cereal import car
from panda import Panda
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.hyundai.hyundaicanfd import CanBus
from openpilot.selfdrive.car.hyundai.values import HyundaiFlags, CAR, DBC, CANFD_CAR, CAMERA_SCC_CAR, CANFD_RADAR_SCC_CAR, \
                                         CANFD_UNSUPPORTED_LONGITUDINAL_CAR, EV_CAR, HYBRID_CAR, LEGACY_SAFETY_MODE_CAR, \
                                         UNSUPPORTED_LONGITUDINAL_CAR, Buttons, HyundaiExtFlags
from openpilot.selfdrive.car.hyundai.radar_interface import RADAR_START_ADDR
from openpilot.selfdrive.car import create_button_events, get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.car.disable_ecu import disable_ecu
from openpilot.common.params import Params  #ajouatom
from selfdrive.car.isotp_parallel_query import IsoTpParallelQuery #ajouatom

Ecu = car.CarParams.Ecu
ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName
ENABLE_BUTTONS = (Buttons.RES_ACCEL, Buttons.SET_DECEL, Buttons.CANCEL)
BUTTONS_DICT = {Buttons.RES_ACCEL: ButtonType.accelCruise, Buttons.SET_DECEL: ButtonType.decelCruise,
                Buttons.GAP_DIST: ButtonType.gapAdjustCruise, Buttons.CANCEL: ButtonType.cancel, Buttons.LFA_BUTTON: ButtonType.lfaButton}
BUTTONS_DICT_PCM = {Buttons.RES_ACCEL: ButtonType.accelCruise, Buttons.SET_DECEL: ButtonType.decelCruise,
                Buttons.CANCEL: ButtonType.cancel, Buttons.LFA_BUTTON: ButtonType.lfaButton}


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "hyundai"
    ret.radarUnavailable = RADAR_START_ADDR not in fingerprint[1] or DBC[ret.carFingerprint]["radar"] is None

    # These cars have been put into dashcam only due to both a lack of users and test coverage.
    # These cars likely still work fine. Once a user confirms each car works and a test route is
    # added to selfdrive/car/tests/routes.py, we can remove it from this list.
    # FIXME: the Optima Hybrid 2017 uses a different SCC12 checksum
    ret.dashcamOnly = candidate in {CAR.KIA_OPTIMA_H, }

    scc2 = Params().get_int("SccConnectedBus2")
    if scc2 > 0:
      ret.extFlags |= HyundaiExtFlags.SCC_BUS2.value
      if scc2 > 1:
        ret.extFlags |= HyundaiExtFlags.ACAN_PANDA.value
    hda2 = Ecu.adas in [fw.ecu for fw in car_fw] and candidate in CANFD_CAR or Params().get_bool("CanfdHDA2")
    CAN = CanBus(None, hda2, fingerprint)

    if candidate in CANFD_CAR:
      print("$$$$CANFD_CAR")
      # detect if car is hybrid
      if 0x105 in fingerprint[CAN.ECAN]: # 0x105(261): ACCELERATOR_ALT
        ret.flags |= HyundaiFlags.HYBRID.value
        print("$$$CANFD Hybrid")
      elif candidate in EV_CAR:
        print("$$$CANFD EV")
        ret.flags |= HyundaiFlags.EV.value

      if 0x3a0 in fingerprint[CAN.ECAN]: # 0x3a0(928): TPMS
        ret.extFlags |= HyundaiExtFlags.CANFD_TPMS.value
        print("$$$CANFD TPMS")

      # detect HDA2 with ADAS Driving ECU
      if hda2:
        print("$$$CANFD HDA2")
        ret.flags |= HyundaiFlags.CANFD_HDA2.value
        if scc2 > 0:
          if 0x110 in fingerprint[CAN.ACAN]:
            ret.flags |= HyundaiFlags.CANFD_HDA2_ALT_STEERING.value
            print("$$$CANFD ALT_STEERING1")
        else:
          if 0x110 in fingerprint[CAN.CAM]: # 0x110(272): LKAS_ALT
            ret.flags |= HyundaiFlags.CANFD_HDA2_ALT_STEERING.value
            print("$$$CANFD ALT_STEERING1")
          ## carrot_todo: sorento: 뭐 이런코드가... ㅠㅠ
          if 0x2a4 not in fingerprint[CAN.CAM]: # 0x2a4(676): CAM_0x2a4
            ret.flags |= HyundaiFlags.CANFD_HDA2_ALT_STEERING.value
            print("$$$CANFD ALT_STEERING2")

        ## carrot: canival 4th, no 0x1cf
        if 0x1cf not in fingerprint[CAN.ECAN]: # 0x1cf(463): CRUISE_BUTTONS
          ret.flags |= HyundaiFlags.CANFD_ALT_BUTTONS.value
          print("$$$CANFD ALT_BUTTONS")
        ## carrot
        if 0x130 not in fingerprint[CAN.ECAN]: # 0x130(304): GEAR_SHIFTER
          if 0x40 not in fingerprint[CAN.ECAN]: # 0x40(64): GEAR_ALT
            if 112 not in fingerprint[CAN.ECAN]:  # carrot: eGV70
              if 69 in fingerprint[CAN.ECAN]:
                ret.extFlags |= HyundaiExtFlags.CANFD_GEARS_69.value
                print("$$$CANFD GEARS_69")
              else:
                ret.extFlags |= HyundaiExtFlags.CANFD_GEARS_NONE.value
                print("$$$CANFD GEARS_NONE")
            else:
              ret.flags |= HyundaiFlags.CANFD_ALT_GEARS_2.value
              print("$$$CANFD ALT_GEARS_2")
          else:
            ret.flags |= HyundaiFlags.CANFD_ALT_GEARS.value
            print("$$$CANFD ALT_GEARS")
      else:
        # non-HDA2
        print("$$$CANFD non HDA2")
        if 0x1cf not in fingerprint[CAN.ECAN]:  # 0x1cf(463): CRUISE_BUTTONS
          ret.flags |= HyundaiFlags.CANFD_ALT_BUTTONS.value
          print("$$$CANFD ALT_BUTTONS")
        # ICE cars do not have 0x130; GEARS message on 0x40 or 0x70 instead
        if 0x130 not in fingerprint[CAN.ECAN]: # 0x130(304): GEAR_SHIFTER
          if 0x40 not in fingerprint[CAN.ECAN]: # 0x40(64): GEAR_ALT
            if 112 not in fingerprint[CAN.ECAN]:  # carrot: eGV70
              if 69 in fingerprint[CAN.ECAN]:
                ret.extFlags |= HyundaiExtFlags.CANFD_GEARS_69.value
                print("$$$CANFD GEARS_69")
              else:
                ret.extFlags |= HyundaiExtFlags.CANFD_GEARS_NONE.value
                print("$$$CANFD GEARS_NONE")
            else:
              ret.flags |= HyundaiFlags.CANFD_ALT_GEARS_2.value
              print("$$$CANFD ALT_GEARS_2")
          else:
            ret.flags |= HyundaiFlags.CANFD_ALT_GEARS.value
            print("$$$CANFD ALT_GEARS")
        if candidate not in CANFD_RADAR_SCC_CAR:
          ret.flags |= HyundaiFlags.CANFD_CAMERA_SCC.value
          print("$$$CANFD CAMERA_SCC")
    else:
      # TODO: detect EV and hybrid
      if candidate in HYBRID_CAR:
        ret.flags |= HyundaiFlags.HYBRID.value
      elif candidate in EV_CAR:
        ret.flags |= HyundaiFlags.EV.value

      # Send LFA message on cars with HDA
      if 0x485 in fingerprint[2]:
        ret.flags |= HyundaiFlags.SEND_LFA.value

      # These cars use the FCA11 message for the AEB and FCW signals, all others use SCC12
      if 0x38d in fingerprint[0] or 0x38d in fingerprint[2]:
        ret.flags |= HyundaiFlags.USE_FCA.value

      if 1290 in fingerprint[2]:
        ret.extFlags |= HyundaiExtFlags.HAS_SCC13.value

      if 905 in fingerprint[2]:
        ret.extFlags |= HyundaiExtFlags.HAS_SCC14.value

    ret.steerActuatorDelay = 0.1  # Default delay
    ret.steerLimitTimer = 0.4
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    # *** longitudinal control ***
    if candidate in CANFD_CAR:
      ret.longitudinalTuning.kpV = [0.1]
      ret.longitudinalTuning.kiV = [0.0]
      ret.experimentalLongitudinalAvailable = candidate not in (CANFD_UNSUPPORTED_LONGITUDINAL_CAR | CANFD_RADAR_SCC_CAR)
      ret.experimentalLongitudinalAvailable = True # carrot: 비전 롱컨이라도 되도록... 될까?
       
    else:
      ret.longitudinalTuning.kpV = [0.5]
      ret.longitudinalTuning.kiV = [0.0]
      ret.experimentalLongitudinalAvailable = candidate not in (UNSUPPORTED_LONGITUDINAL_CAR | CAMERA_SCC_CAR)
      
      ret.experimentalLongitudinalAvailable = True

    print("***************************************************************************")
    print("sccBus = ", 2 if ret.extFlags & HyundaiExtFlags.SCC_BUS2.value else 0)
    print("adasPanda = ", 1 if ret.extFlags & HyundaiExtFlags.ACAN_PANDA.value else 0)

    ret.openpilotLongitudinalControl = experimental_long and ret.experimentalLongitudinalAvailable

    ret.stoppingControl = True
    ret.startingState = False # apilot: True
    ret.vEgoStarting = 0.1
    ret.vEgoStopping = 0.1 #0.25
    ret.startAccel = 2.0
    ret.stoppingDecelRate = 1.2 # brake_travel/s while trying to stop
    ret.longitudinalActuatorDelayLowerBound = 0.5
    ret.longitudinalActuatorDelayUpperBound = 0.5

    if Params().get_int("EnableRadarTracks") > 0:
      ret.radarTimeStep = (1.0 / 20) # 20Hz  RadarTrack 20Hz
    else:
      ret.radarTimeStep = (1.0 / 50) # 50Hz   SCC11 50Hz
    

    # *** feature detection ***
    if candidate in CANFD_CAR:
      ret.enableBsm = 0x1e5 in fingerprint[CAN.ECAN]
      if candidate in (CAR.KIA_CARNIVAL_4TH_GEN) and hda2: ##카니발4th & hda2 인경우에만 BSM이 ADAS에서 나옴.
        ret.extFlags |= HyundaiExtFlags.BSM_IN_ADAS.value
      print(f"$$$$$ CanFD ECAN = {CAN.ECAN}")
      if 0x1fa in fingerprint[CAN.ECAN]:
        ret.extFlags |= HyundaiExtFlags.NAVI_CLUSTER.value
        print("$$$$ NaviCluster = True")
      else:
        print("$$$$ NaviCluster = False")
    else:
      ret.enableBsm = 0x58b in fingerprint[0]

      if 1348 in fingerprint[0]:
        ret.extFlags |= HyundaiExtFlags.NAVI_CLUSTER.value
        print("$$$$ NaviCluster = True")
      if 1157 in fingerprint[0] or 1157 in fingerprint[2]:
        ret.extFlags |= HyundaiExtFlags.HAS_LFAHDA.value
      if 913 in fingerprint[0]:
        ret.extFlags |= HyundaiExtFlags.HAS_LFA_BUTTON.value

    print(f"$$$$ enableBsm = {ret.enableBsm}")

    # *** panda safety config ***
    if candidate in CANFD_CAR:
      cfgs = [get_safety_config(car.CarParams.SafetyModel.hyundaiCanfd), ]
      if CAN.ECAN >= 4:
       cfgs.insert(0, get_safety_config(car.CarParams.SafetyModel.noOutput))
      ret.safetyConfigs = cfgs

      if ret.flags & HyundaiFlags.CANFD_HDA2:
        print("!!!!!!!!!Panda: FLAG_HYUNDAI_CANFD_HDA2")
        ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_CANFD_HDA2
        if ret.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING:
          print("!!!!!!!!!Panda: FLAG_HYUNDAI_CANFD_HDA2_ALT_STEERING")
          ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_CANFD_HDA2_ALT_STEERING
      if ret.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
        print("!!!!!!!!!Panda: FLAG_HYUNDAI_CANFD_ALT_BUTTONS")
        ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_CANFD_ALT_BUTTONS
      if ret.flags & HyundaiFlags.CANFD_CAMERA_SCC:
        print("!!!!!!!!!Panda: FLAG_HYUNDAI_CAMERA_SCC")
        ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_CAMERA_SCC
      if ret.extFlags & HyundaiExtFlags.SCC_BUS2.value:
        print("!!!!!!!!!Panda: FLAG_HYUNDAI_SCC_BUS2")
        ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_SCC_BUS2
    else:
      if candidate in LEGACY_SAFETY_MODE_CAR:
        # these cars require a special panda safety mode due to missing counters and checksums in the messages
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiLegacy)]
      else:
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundai, 0)]

      if candidate in CAMERA_SCC_CAR:
        ret.safetyConfigs[0].safetyParam |= Panda.FLAG_HYUNDAI_CAMERA_SCC

      if ret.extFlags & HyundaiExtFlags.SCC_BUS2.value:
        ret.openpilotLongitudinalControl = True
        ret.radarUnavailable = False
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiLegacy)]

    ret.pcmCruise = not ret.openpilotLongitudinalControl
    print(f"$$$$ LongitudinalControl = {ret.openpilotLongitudinalControl}")
    if ret.openpilotLongitudinalControl:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_LONG
    if ret.flags & HyundaiFlags.HYBRID:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_HYBRID_GAS
    elif ret.flags & HyundaiFlags.EV:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_EV_GAS

    if candidate in (CAR.HYUNDAI_KONA, CAR.HYUNDAI_KONA_EV, CAR.HYUNDAI_KONA_HEV, CAR.HYUNDAI_KONA_EV_2022):
      ret.flags |= HyundaiFlags.ALT_LIMITS.value
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_ALT_LIMITS

    ret.centerToFront = ret.wheelbase * 0.4

    return ret

  @staticmethod
  def init(CP, logcan, sendcan):
    Params().put('LongitudinalPersonalityMax', "4")
    if CP.openpilotLongitudinalControl and not (CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value) and not (CP.extFlags & HyundaiExtFlags.SCC_BUS2.value):
      addr, bus = 0x7d0, CanBus(CP).ECAN if CP.carFingerprint in CANFD_CAR else 0
      if CP.flags & HyundaiFlags.CANFD_HDA2.value:
        addr, bus = 0x730, CanBus(CP).ECAN
      print(f"$$$$$$ Disable ECU : addr={addr}, bus={bus}")
      disable_ecu(logcan, sendcan, bus=bus, addr=addr, com_cont_req=b'\x28\x83\x01', debug=True)
    if Params().get_int("EnableRadarTracks") > 0:
      enable_radar_tracks(CP, logcan, sendcan) 

    # for blinkers
    if CP.flags & HyundaiFlags.ENABLE_BLINKERS:
      disable_ecu(logcan, sendcan, bus=CanBus(CP).ECAN, addr=0x7B1, com_cont_req=b'\x28\x83\x01')

    if Params().get_bool("EnableAVM"): #ajouatom
      enable_avm(logcan, sendcan)
      print("$$$$ Enable AVM = True")

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)

    if self.CS.CP.openpilotLongitudinalControl or Params().get_int("SpeedFromPCM") in [0,2]: #ajouatom: for PCM
      ret.buttonEvents = create_button_events(self.CS.cruise_buttons[-1], self.CS.prev_cruise_buttons, BUTTONS_DICT_PCM if self.CP.pcmCruise else BUTTONS_DICT)

    # On some newer model years, the CANCEL button acts as a pause/resume button based on the PCM state
    # To avoid re-engaging when openpilot cancels, check user engagement intention via buttons
    # Main button also can trigger an engagement on these cars
    allow_enable = any(btn in ENABLE_BUTTONS for btn in self.CS.cruise_buttons) or any(self.CS.main_buttons)

    allow_enable = True #carrot
    events = self.create_common_events(ret, pcm_enable=self.CS.CP.pcmCruise, allow_enable=allow_enable)

    # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)
    if ret.vEgo < (self.CP.minSteerSpeed + 2.) and self.CP.minSteerSpeed > 10.:
      self.low_speed_alert = True
    if ret.vEgo > (self.CP.minSteerSpeed + 4.):
      self.low_speed_alert = False
    if self.low_speed_alert:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    ret.events = events.to_msg()

    return ret

#ajouatom: Enable Radar tracks
def enable_radar_tracks(CP, logcan, sendcan):
  print("################ Try To Enable Radar Tracks ####################")
  
  sccBus = 2 if Params().get_int("SccConnectedBus2") > 0 else 0
  rdr_fw = None
  rdr_fw_address = 0x7d0 #
  try:
    for i in range(40):
      try:
        query = IsoTpParallelQuery(sendcan, logcan, sccBus, [rdr_fw_address], [b'\x10\x07'], [b'\x50\x07'], debug=True)
        for addr, dat in query.get_data(0.1).items(): # pylint: disable=unused-variable
          print("ecu write data by id ...")
          new_config = b"\x00\x00\x00\x01\x00\x01"
          #new_config = b"\x00\x00\x00\x00\x00\x01"
          dataId = b'\x01\x42'
          WRITE_DAT_REQUEST = b'\x2e'
          WRITE_DAT_RESPONSE = b'\x68'
          query = IsoTpParallelQuery(sendcan, logcan, sccBus, [rdr_fw_address], [WRITE_DAT_REQUEST+dataId+new_config], [WRITE_DAT_RESPONSE], debug=True)
          query.get_data(0)
          print(f"Try {i+1}")
          break
        break
      except Exception as e:
        print(f"Failed {i}: {e}") 
  except Exception as e:
    print("##############  Failed to enable tracks" + str(e))
  print("################ END Try to enable radar tracks")

def enable_avm(logcan, sendcan):
  bus = 0
  query = IsoTpParallelQuery(sendcan, logcan, bus, [0x7b1], [b'\x10\x03'], [b'\x50\x03'], debug=True)
  query.get_data(0.1) 
  query = IsoTpParallelQuery(sendcan, logcan, bus, [0x7b1], [b'\x3E\x00'], [b'\x7E\x00'], debug=True)
  query.get_data(0.1)
  query = IsoTpParallelQuery(sendcan, logcan, bus, [0x7b1], [b'\x2f\xf0\x26\x03\xff'], [b'\x6f\xf0\x26\x03'], debug=True)
  query.get_data(0.1) 

