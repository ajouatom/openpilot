from cereal import car
from panda import Panda
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.hyundai.hyundaicanfd import CanBus
from openpilot.selfdrive.car.hyundai.values import HyundaiFlags, CAR, DBC, CANFD_CAR, CAMERA_SCC_CAR, CANFD_RADAR_SCC_CAR, \
                                         CANFD_UNSUPPORTED_LONGITUDINAL_CAR, EV_CAR, HYBRID_CAR, LEGACY_SAFETY_MODE_CAR, \
                                         UNSUPPORTED_LONGITUDINAL_CAR, Buttons
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
                Buttons.GAP_DIST: ButtonType.gapAdjustCruise, Buttons.CANCEL: ButtonType.cancel}


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

      # detect HDA2 with ADAS Driving ECU
      if hda2:
        print("$$$CANFD HDA2")
        ret.flags |= HyundaiFlags.CANFD_HDA2.value
        if 0x110 in fingerprint[CAN.CAM]: # 0x110(272): LKAS_ALT
          ret.flags |= HyundaiFlags.CANFD_HDA2_ALT_STEERING.value
          print("$$$CANFD ALT_STEERING")
        ## carrot_todo: sorento
        if 0x2a4 not in fingerprint[CAN.CAM]: # 0x2a4(676): CAM_0x2a4
          ret.flags |= HyundaiFlags.CANFD_HDA2_ALT_STEERING.value
          print("$$$CANFD ALT_STEERING")
        ## carrot: canival 4th, no 0x1cf
        if 0x1cf not in fingerprint[CAN.ECAN]: # 0x1cf(463): CRUISE_BUTTONS
          ret.flags |= HyundaiFlags.CANFD_ALT_BUTTONS.value
          print("$$$CANFD ALT_BUTTONS")
        ## carrot
        if 0x130 not in fingerprint[CAN.ECAN]: # 0x130(304): GEAR_SHIFTER
          if 0x40 not in fingerprint[CAN.ECAN]: # 0x40(64): GEAR_ALT
            if 112 not in fingerprint[CAN.ECAN]:  # carrot: eGV70
              ret.flags |= HyundaiFlags.CANFD_GEARS_NONE.value
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
        ret.flags |= HyundaiFlags.HAS_SCC13.value

      if 905 in fingerprint[2]:
        ret.flags |= HyundaiFlags.HAS_SCC14.value

    ret.steerActuatorDelay = 0.1  # Default delay
    ret.steerLimitTimer = 0.4
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    if candidate in (CAR.AZERA_6TH_GEN, CAR.AZERA_HEV_6TH_GEN):
      ret.mass = 1600. if candidate == CAR.AZERA_6TH_GEN else 1675.  # ICE is ~average of 2.5L and 3.5L
      ret.wheelbase = 2.885
      ret.steerRatio = 14.5
    elif candidate in (CAR.SANTA_FE, CAR.SANTA_FE_2022, CAR.SANTA_FE_HEV_2022, CAR.SANTA_FE_PHEV_2022):
      ret.mass = 3982. * CV.LB_TO_KG
      ret.wheelbase = 2.766
      # Values from optimizer
      ret.steerRatio = 16.55  # 13.8 is spec end-to-end
      ret.tireStiffnessFactor = 0.82
    elif candidate in (CAR.SONATA, CAR.SONATA_HYBRID):
      ret.mass = 1513.
      ret.wheelbase = 2.84
      ret.steerRatio = 13.27 * 1.15   # 15% higher at the center seems reasonable
      ret.tireStiffnessFactor = 0.65
    elif candidate == CAR.SONATA_LF:
      ret.mass = 1536.
      ret.wheelbase = 2.804
      ret.steerRatio = 13.27 * 1.15   # 15% higher at the center seems reasonable
    elif candidate == CAR.PALISADE:
      ret.mass = 1999.
      ret.wheelbase = 2.90
      ret.steerRatio = 15.6 * 1.15
      ret.tireStiffnessFactor = 0.63
    elif candidate in (CAR.ELANTRA, CAR.ELANTRA_GT_I30):
      ret.mass = 1275.
      ret.wheelbase = 2.7
      ret.steerRatio = 15.4            # 14 is Stock | Settled Params Learner values are steerRatio: 15.401566348670535
      ret.tireStiffnessFactor = 0.385    # stiffnessFactor settled on 1.0081302973865127
      ret.minSteerSpeed = 32 * CV.MPH_TO_MS
    elif candidate == CAR.ELANTRA_2021:
      ret.mass = 2800. * CV.LB_TO_KG
      ret.wheelbase = 2.72
      ret.steerRatio = 12.9
      ret.tireStiffnessFactor = 0.65
    elif candidate == CAR.ELANTRA_HEV_2021:
      ret.mass = 3017. * CV.LB_TO_KG
      ret.wheelbase = 2.72
      ret.steerRatio = 12.9
      ret.tireStiffnessFactor = 0.65
    elif candidate == CAR.HYUNDAI_GENESIS:
      ret.mass = 2060.
      ret.wheelbase = 3.01
      ret.steerRatio = 16.5
      #ret.minSteerSpeed = 60 * CV.KPH_TO_MS #ajouatom
    elif candidate in (CAR.KONA, CAR.KONA_EV, CAR.KONA_HEV, CAR.KONA_EV_2022, CAR.KONA_EV_2ND_GEN):
      ret.mass = {CAR.KONA_EV: 1685., CAR.KONA_HEV: 1425., CAR.KONA_EV_2022: 1743., CAR.KONA_EV_2ND_GEN: 1740.}.get(candidate, 1275.)
      ret.wheelbase = {CAR.KONA_EV_2ND_GEN: 2.66, }.get(candidate, 2.6)
      ret.steerRatio = {CAR.KONA_EV_2ND_GEN: 13.6, }.get(candidate, 13.42)  # Spec
      ret.tireStiffnessFactor = 0.385
    elif candidate in (CAR.IONIQ, CAR.IONIQ_EV_LTD, CAR.IONIQ_PHEV_2019, CAR.IONIQ_HEV_2022, CAR.IONIQ_EV_2020, CAR.IONIQ_PHEV):
      ret.mass = 1490.  # weight per hyundai site https://www.hyundaiusa.com/ioniq-electric/specifications.aspx
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73  # Spec
      ret.tireStiffnessFactor = 0.385
      if candidate in (CAR.IONIQ, CAR.IONIQ_EV_LTD, CAR.IONIQ_PHEV_2019):
        #ret.minSteerSpeed = 32 * CV.MPH_TO_MS #ajouatom
        pass
    elif candidate in (CAR.IONIQ_5, CAR.IONIQ_6):
      ret.mass = 1948
      ret.wheelbase = 2.97
      ret.steerRatio = 14.26
      ret.tireStiffnessFactor = 0.65
    elif candidate == CAR.VELOSTER:
      ret.mass = 2917. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75 * 1.15
      ret.tireStiffnessFactor = 0.5
    elif candidate == CAR.TUCSON:
      ret.mass = 3520. * CV.LB_TO_KG
      ret.wheelbase = 2.67
      ret.steerRatio = 14.00 * 1.15
      ret.tireStiffnessFactor = 0.385
    elif candidate == CAR.TUCSON_4TH_GEN:
      ret.mass = 1630.  # average
      ret.wheelbase = 2.756
      ret.steerRatio = 16.
      ret.tireStiffnessFactor = 0.385
    elif candidate == CAR.SANTA_CRUZ_1ST_GEN:
      ret.mass = 1870.  # weight from Limited trim - the only supported trim
      ret.wheelbase = 3.000
      # steering ratio according to Hyundai News https://www.hyundainews.com/assets/documents/original/48035-2022SantaCruzProductGuideSpecsv2081521.pdf
      ret.steerRatio = 14.2
    elif candidate == CAR.CUSTIN_1ST_GEN:
      ret.mass = 1690.  # from https://www.hyundai-motor.com.tw/clicktobuy/custin#spec_0
      ret.wheelbase = 3.055
      ret.steerRatio = 17.0  # from learner
    elif candidate == CAR.STARIA_4TH_GEN:
      ret.mass = 2205.
      ret.wheelbase = 3.273
      ret.steerRatio = 11.94  # https://www.hyundai.com/content/dam/hyundai/au/en/models/staria-load/premium-pip-update-2023/spec-sheet/STARIA_Load_Spec-Table_March_2023_v3.1.pdf

    # Kia
    elif candidate == CAR.KIA_SORENTO:
      ret.mass = 1985.
      ret.wheelbase = 2.78
      ret.steerRatio = 14.4 * 1.1   # 10% higher at the center seems reasonable
    elif candidate in (CAR.KIA_NIRO_EV, CAR.KIA_NIRO_EV_2ND_GEN, CAR.KIA_NIRO_PHEV, CAR.KIA_NIRO_HEV_2021, CAR.KIA_NIRO_HEV_2ND_GEN, CAR.KIA_NIRO_PHEV_2022):
      ret.mass = 3543. * CV.LB_TO_KG  # average of all the cars
      ret.wheelbase = 2.7
      ret.steerRatio = 13.6  # average of all the cars
      ret.tireStiffnessFactor = 0.385
      if candidate == CAR.KIA_NIRO_PHEV:
        ret.minSteerSpeed = 32 * CV.MPH_TO_MS
    elif candidate == CAR.KIA_SELTOS:
      ret.mass = 1337.
      ret.wheelbase = 2.63
      ret.steerRatio = 14.56
    elif candidate == CAR.KIA_SPORTAGE_5TH_GEN:
      ret.mass = 1725.  # weight from SX and above trims, average of FWD and AWD versions
      ret.wheelbase = 2.756
      ret.steerRatio = 13.6  # steering ratio according to Kia News https://www.kiamedia.com/us/en/models/sportage/2023/specifications
    elif candidate in (CAR.KIA_OPTIMA_G4, CAR.KIA_OPTIMA_G4_FL, CAR.KIA_OPTIMA_H, CAR.KIA_OPTIMA_H_G4_FL):
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75
      ret.tireStiffnessFactor = 0.5
      if candidate == CAR.KIA_OPTIMA_G4:
        ret.minSteerSpeed = 32 * CV.MPH_TO_MS
    elif candidate in (CAR.KIA_STINGER, CAR.KIA_STINGER_2022):
      ret.mass = 1825.
      ret.wheelbase = 2.78
      ret.steerRatio = 14.4 * 1.15   # 15% higher at the center seems reasonable
    elif candidate == CAR.KIA_FORTE:
      ret.mass = 2878. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75
      ret.tireStiffnessFactor = 0.5
    elif candidate == CAR.KIA_CEED:
      ret.mass = 1450.
      ret.wheelbase = 2.65
      ret.steerRatio = 13.75
      ret.tireStiffnessFactor = 0.5
    elif candidate in (CAR.KIA_K5_2021, CAR.KIA_K5_HEV_2020):
      ret.mass = 3381. * CV.LB_TO_KG
      ret.wheelbase = 2.85
      ret.steerRatio = 13.27  # 2021 Kia K5 Steering Ratio (all trims)
      ret.tireStiffnessFactor = 0.5
    elif candidate == CAR.KIA_EV6:
      ret.mass = 2055
      ret.wheelbase = 2.9
      ret.steerRatio = 16.
      ret.tireStiffnessFactor = 0.65
    elif candidate in (CAR.KIA_SORENTO_4TH_GEN, CAR.KIA_SORENTO_HEV_4TH_GEN):
      ret.wheelbase = 2.81
      ret.steerRatio = 13.5  # average of the platforms
      if candidate == CAR.KIA_SORENTO_4TH_GEN:
        ret.mass = 3957 * CV.LB_TO_KG
      else:
        ret.mass = 4396 * CV.LB_TO_KG
    elif candidate == CAR.KIA_CARNIVAL_4TH_GEN:
      ret.mass = 2087.
      ret.wheelbase = 3.09
      ret.steerRatio = 14.23
    elif candidate == CAR.KIA_K8_HEV_1ST_GEN:
      ret.mass = 1630.  # https://carprices.ae/brands/kia/2023/k8/1.6-turbo-hybrid
      ret.wheelbase = 2.895
      ret.steerRatio = 13.27  # guesstimate from K5 platform

    # Genesis
    elif candidate == CAR.GENESIS_GV60_EV_1ST_GEN:
      ret.mass = 2205
      ret.wheelbase = 2.9
      # https://www.motor1.com/reviews/586376/2023-genesis-gv60-first-drive/#:~:text=Relative%20to%20the%20related%20Ioniq,5%2FEV6%27s%2014.3%3A1.
      ret.steerRatio = 12.6
    elif candidate == CAR.GENESIS_G70:
      ret.steerActuatorDelay = 0.1
      ret.mass = 1640.0
      ret.wheelbase = 2.84
      ret.steerRatio = 13.56
    elif candidate == CAR.GENESIS_G70_2020:
      ret.mass = 3673.0 * CV.LB_TO_KG
      ret.wheelbase = 2.83
      ret.steerRatio = 12.9
    elif candidate == CAR.GENESIS_GV70_1ST_GEN:
      ret.mass = 1950.
      ret.wheelbase = 2.87
      ret.steerRatio = 14.6
    elif candidate == CAR.GENESIS_G80:
      ret.mass = 2060.
      ret.wheelbase = 3.01
      ret.steerRatio = 16.5
    elif candidate == CAR.GENESIS_G90:
      ret.mass = 2200.
      ret.wheelbase = 3.15
      ret.steerRatio = 12.069
    elif candidate == CAR.GENESIS_GV80:
      ret.mass = 2258.
      ret.wheelbase = 2.95
      ret.steerRatio = 14.14

    elif candidate == CAR.KIA_SOUL_EV_SK3:
      ret.steerRatio = 13.7  # average of the platforms
      ret.mass = 1375.
      ret.wheelbase = 2.6      
    elif candidate in [CAR.GRANDEUR_IG, CAR.GRANDEUR_IG_HEV]:
      ret.mass = 1570.
      ret.wheelbase = 2.845
      ret.steerRatio = 16.
      ret.tireStiffnessFactor = 0.8
      ret.centerToFront = ret.wheelbase * 0.385
    elif candidate in [CAR.GRANDEUR_IG_FL, CAR.GRANDEUR_IG_FL_HEV]:
      ret.mass = 1600.
      ret.wheelbase = 2.885
      ret.steerRatio = 17.
      ret.tireStiffnessFactor = 0.8
      ret.centerToFront = ret.wheelbase * 0.385
    elif candidate == CAR.NEXO: # fix PolorBear - 22.06.05
      ret.mass = 1885.
      ret.wheelbase = 2.79
      ret.steerRatio = 15.3
      ret.tireStiffnessFactor = 0.385
    elif candidate in [CAR.K7, CAR.K7_HEV]:
      ret.mass = 1850.
      ret.wheelbase = 2.855
      ret.steerRatio = 15.5
      ret.tireStiffnessFactor = 0.7
    elif candidate == CAR.GENESIS_EQ900:
      ret.mass = 2200
      ret.wheelbase = 3.15
      ret.steerRatio = 16.0
      ret.steerActuatorDelay = 0.075
    elif candidate == CAR.GENESIS_EQ900_L:
      ret.mass = 2290
      ret.wheelbase = 3.45
      ret.steerRatio = 12.069  # carrot, TODO
    elif candidate == CAR.GENESIS_G90_2019:
      ret.mass = 2150
      ret.wheelbase = 3.16
      ret.steerRatio = 12.069  # carrot, TODO
    elif candidate == CAR.GENESIS_EGV70:
      ret.mass = 2230
      ret.wheelbase = 2.87
      ret.steerRatio = 14.6
      

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
       
      if Params().get_bool("SccConnectedBus2"):
        ret.flags |= HyundaiFlags.SCC_BUS2.value
        experimental_long = True
      
      print("***************************************************************************")
      print("sccBus = ", 2 if ret.flags & HyundaiFlags.SCC_BUS2.value else 0)
      ret.experimentalLongitudinalAvailable = True
    ret.openpilotLongitudinalControl = experimental_long and ret.experimentalLongitudinalAvailable
    ret.pcmCruise = not ret.openpilotLongitudinalControl

    ret.stoppingControl = True
    ret.startingState = False # apilot: True
    ret.vEgoStarting = 0.1
    ret.vEgoStopping = 0.25
    ret.startAccel = 2.0
    ret.stoppingDecelRate = 1.2 # brake_travel/s while trying to stop
    ret.longitudinalActuatorDelayLowerBound = 0.5
    ret.longitudinalActuatorDelayUpperBound = 0.5

    if Params().get_bool("EnableRadarTracks"):
      ret.radarTimeStep = (1.0 / 20) # 20Hz  RadarTrack 20Hz
    else:
      ret.radarTimeStep = (1.0 / 50) # 50Hz   SCC11 50Hz
    

    # *** feature detection ***
    if candidate in CANFD_CAR:
      ret.enableBsm = 0x1e5 in fingerprint[CAN.ECAN]
      print(f"$$$$$ CanFD ECAN = {CAN.ECAN}")
      if 0x1fa in fingerprint[CAN.ECAN]:
        ret.flags |= HyundaiFlags.NAVI_CLUSTER.value
        print("$$$$ NaviCluster = True")
      else:
        print("$$$$ NaviCluster = False")
    else:
      ret.enableBsm = 0x58b in fingerprint[0]

      if 1348 in fingerprint[0]:
        ret.flags |= HyundaiFlags.NAVI_CLUSTER.value
        print("$$$$ NaviCluster = True")
      if 1157 in fingerprint[0] or 1157 in fingerprint[2]:
        ret.flags |= HyundaiFlags.HAS_LFAHDA.value
      if 913 in fingerprint[0]:
        ret.flags |= HyundaiFlags.HAS_LFA_BUTTON.value

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
    else:
      if candidate in LEGACY_SAFETY_MODE_CAR:
        # these cars require a special panda safety mode due to missing counters and checksums in the messages
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiLegacy)]
      else:
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundai, 0)]

      if candidate in CAMERA_SCC_CAR:
        ret.safetyConfigs[0].safetyParam |= Panda.FLAG_HYUNDAI_CAMERA_SCC

    if ret.flags & HyundaiFlags.SCC_BUS2.value:
      ret.openpilotLongitudinalControl = True
      ret.radarUnavailable = False
      ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiLegacy)]

    print(f"$$$$ LongitudinalControl = {ret.openpilotLongitudinalControl}")
    if ret.openpilotLongitudinalControl:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_LONG
    if ret.flags & HyundaiFlags.HYBRID:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_HYBRID_GAS
    elif ret.flags & HyundaiFlags.EV:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_EV_GAS

    if candidate in (CAR.KONA, CAR.KONA_EV, CAR.KONA_HEV, CAR.KONA_EV_2022):
      ret.flags |= HyundaiFlags.ALT_LIMITS.value
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_ALT_LIMITS

    ret.centerToFront = ret.wheelbase * 0.4

    return ret

  @staticmethod
  def init(CP, logcan, sendcan):
    if CP.openpilotLongitudinalControl and not (CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value) and not (CP.flags & HyundaiFlags.SCC_BUS2.value):
      addr, bus = 0x7d0, CanBus(CP).ECAN if CP.carFingerprint in CANFD_CAR else 0
      if CP.flags & HyundaiFlags.CANFD_HDA2.value:
        addr, bus = 0x730, CanBus(CP).ECAN
      print(f"$$$$$$ Disable ECU : addr={addr}, bus={bus}")
      disable_ecu(logcan, sendcan, bus=bus, addr=addr, com_cont_req=b'\x28\x83\x01', debug=True)
    if Params().get_bool("EnableRadarTracks"): #ajouatom
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
      ret.buttonEvents = create_button_events(self.CS.cruise_buttons[-1], self.CS.prev_cruise_buttons, BUTTONS_DICT)

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

  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)

#ajouatom: Enable Radar tracks
def enable_radar_tracks(CP, logcan, sendcan):
  print("################ Try To Enable Radar Tracks ####################")
  
  sccBus = 2 if Params().get_bool("SccConnectedBus2") else 0
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

