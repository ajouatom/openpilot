import re
from dataclasses import dataclass, field
from enum import Enum, IntFlag

from cereal import car
from panda.python import uds
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from openpilot.selfdrive.car.docs_definitions import CarFootnote, CarHarness, CarInfo, CarParts, Column
from openpilot.selfdrive.car.fw_query_definitions import FwQueryConfig, Request, p16

Ecu = car.CarParams.Ecu


class CarControllerParams:
  ACCEL_MIN = -4.0 # m/s
  ACCEL_MAX = 2.5 # m/s

  def __init__(self, CP):
    self.STEER_DELTA_UP = 3
    self.STEER_DELTA_DOWN = 7
    self.STEER_DRIVER_ALLOWANCE = 50
    self.STEER_DRIVER_MULTIPLIER = 2
    self.STEER_DRIVER_FACTOR = 1
    self.STEER_THRESHOLD = 150
    self.STEER_STEP = 1  # 100 Hz

    if CP.carFingerprint in CANFD_CAR:
      self.STEER_MAX = 270
      self.STEER_DRIVER_ALLOWANCE = 250
      self.STEER_DRIVER_MULTIPLIER = 2
      self.STEER_THRESHOLD = 250
      self.STEER_DELTA_UP = 2
      self.STEER_DELTA_DOWN = 3

      if CP.carFingerprint in (CAR.KIA_CARNIVAL_4TH_GEN):
        self.STEER_MAX = 384
        self.STEER_DELTA_UP = 3
        self.STEER_DELTA_DOWN = 7

    # To determine the limit for your car, find the maximum value that the stock LKAS will request.
    # If the max stock LKAS request is <384, add your car to this list.
    elif CP.carFingerprint in (CAR.GENESIS_G80, CAR.GENESIS_G90, CAR.ELANTRA, CAR.ELANTRA_GT_I30, CAR.IONIQ,
                               CAR.IONIQ_EV_LTD, CAR.SANTA_FE_PHEV_2022, CAR.SONATA_LF, CAR.KIA_FORTE, CAR.KIA_NIRO_PHEV,
                               CAR.KIA_OPTIMA_H, CAR.KIA_OPTIMA_H_G4_FL, CAR.KIA_SORENTO):
      self.STEER_MAX = 255

    # these cars have significantly more torque than most HKG; limit to 70% of max
    elif CP.flags & HyundaiFlags.ALT_LIMITS:
      self.STEER_MAX = 384
      self.STEER_DELTA_UP = 3
      self.STEER_DELTA_DOWN = 7

    elif CP.carFingerprint in (CAR.KIA_STINGER, CAR.KONA_HEV):
      self.STEER_MAX = 384
    elif CP.carFingerprint in (CAR.SANTA_FE_HEV_2022):
      self.STEER_MAX = 409
      self.STEER_DELTA_UP = 3
      self.STEER_DELTA_DOWN = 7
      
    # Default for most HKG
    else:
      self.STEER_MAX = 409

class HyundaiFlags(IntFlag):
  # Dynamic Flags
  CANFD_HDA2 = 1
  CANFD_ALT_BUTTONS = 2
  CANFD_ALT_GEARS = 2 ** 2
  CANFD_CAMERA_SCC = 2 ** 3

  ALT_LIMITS = 2 ** 4
  ENABLE_BLINKERS = 2 ** 5
  CANFD_ALT_GEARS_2 = 2 ** 6
  SEND_LFA = 2 ** 7
  USE_FCA = 2 ** 8
  CANFD_HDA2_ALT_STEERING = 2 ** 9

  # these cars use a different gas signal
  HYBRID = 2 ** 10
  EV = 2 ** 11

  # Static flags

  # If 0x500 is present on bus 1 it probably has a Mando radar outputting radar points.
  # If no points are outputted by default it might be possible to turn it on using  selfdrive/debug/hyundai_enable_radar_points.py
  MANDO_RADAR = 2 ** 12
  CANFD = 2 ** 13

  # The radar does SCC on these cars when HDA I, rather than the camera
  RADAR_SCC = 2 ** 14
  CAMERA_SCC = 2 ** 15
  CHECKSUM_CRC8 = 2 ** 16
  CHECKSUM_6B = 2 ** 17

  # these cars require a special panda safety mode due to missing counters and checksums in the messages
  LEGACY = 2 ** 18

  # these cars have not been verified to work with longitudinal yet - radar disable, sending correct messages, etc.
  UNSUPPORTED_LONGITUDINAL = 2 ** 19

  CANFD_NO_RADAR_DISABLE = 2 ** 20

  CLUSTER_GEARS = 2 ** 21
  TCU_GEARS = 2 ** 22

  MIN_STEER_32_MPH = 2 ** 23

  HAS_SCC13 = 2 ** 33
  HAS_SCC14 = 2 ** 34
  NAVI_CLUSTER = 2 ** 35
  SCC_BUS2 = 2 ** 36
  HAS_LFAHDA = 2 ** 37
  HAS_LFA_BUTTON = 2 ** 38
  CANFD_GEARS_NONE = 2 ** 39

class CAR(StrEnum):
  # Hyundai
  AZERA_6TH_GEN = "HYUNDAI AZERA 6TH GEN"
  AZERA_HEV_6TH_GEN = "HYUNDAI AZERA HYBRID 6TH GEN"
  ELANTRA = "HYUNDAI ELANTRA 2017"
  ELANTRA_GT_I30 = "HYUNDAI I30 N LINE 2019 & GT 2018 DCT"
  ELANTRA_2021 = "HYUNDAI ELANTRA 2021"
  ELANTRA_HEV_2021 = "HYUNDAI ELANTRA HYBRID 2021"
  HYUNDAI_GENESIS = "HYUNDAI GENESIS 2015-2016"
  IONIQ = "HYUNDAI IONIQ HYBRID 2017-2019"
  IONIQ_HEV_2022 = "HYUNDAI IONIQ HYBRID 2020-2022"
  IONIQ_EV_LTD = "HYUNDAI IONIQ ELECTRIC LIMITED 2019"
  IONIQ_EV_2020 = "HYUNDAI IONIQ ELECTRIC 2020"
  IONIQ_PHEV_2019 = "HYUNDAI IONIQ PLUG-IN HYBRID 2019"
  IONIQ_PHEV = "HYUNDAI IONIQ PHEV 2020"
  KONA = "HYUNDAI KONA 2020"
  KONA_EV = "HYUNDAI KONA ELECTRIC 2019"
  KONA_EV_2022 = "HYUNDAI KONA ELECTRIC 2022"
  KONA_EV_2ND_GEN = "HYUNDAI KONA ELECTRIC 2ND GEN"
  KONA_HEV = "HYUNDAI KONA HYBRID 2020"
  SANTA_FE = "HYUNDAI SANTA FE 2019"
  SANTA_FE_2022 = "HYUNDAI SANTA FE 2022"
  SANTA_FE_HEV_2022 = "HYUNDAI SANTA FE HYBRID 2022"
  SANTA_FE_PHEV_2022 = "HYUNDAI SANTA FE PlUG-IN HYBRID 2022"
  SONATA = "HYUNDAI SONATA 2020"
  SONATA_LF = "HYUNDAI SONATA 2019"
  STARIA_4TH_GEN = "HYUNDAI STARIA 4TH GEN"
  TUCSON = "HYUNDAI TUCSON 2019"
  PALISADE = "HYUNDAI PALISADE 2020"
  VELOSTER = "HYUNDAI VELOSTER 2019"
  SONATA_HYBRID = "HYUNDAI SONATA HYBRID 2021"
  GRANDEUR_IG = "HYUNDAI GRANDEUR IG 2017"
  GRANDEUR_IG_HEV = "HYUNDAI GRANDEUR IG HEV 2019"
  GRANDEUR_IG_FL = "HYUNDAI GRANDEUR IG FL 2020"
  GRANDEUR_IG_FL_HEV = "HYUNDAI GRANDEUR IG FL HEV 2020"

  GENESIS_EQ900 = "GENESIS EQ900 2017"
  GENESIS_EQ900_L = "GENESIS EQ900 LIMOUSINE"
  GENESIS_G90_2019 = "GENESIS G90 2019"

  IONIQ_5 = "HYUNDAI IONIQ 5 2022"
  IONIQ_6 = "HYUNDAI IONIQ 6 2023"
  TUCSON_4TH_GEN = "HYUNDAI TUCSON 4TH GEN"
  TUCSON_HYBRID_4TH_GEN = "HYUNDAI TUCSON HYBRID 4TH GEN"
  SANTA_CRUZ_1ST_GEN = "HYUNDAI SANTA CRUZ 1ST GEN"
  CUSTIN_1ST_GEN = "HYUNDAI CUSTIN 1ST GEN"
  NEXO = "HYUNDAI NEXO"

  # Kia
  KIA_FORTE = "KIA FORTE E 2018 & GT 2021"
  KIA_K5_2021 = "KIA K5 2021"
  KIA_K5_HEV_2020 = "KIA K5 HYBRID 2020"
  KIA_K8_HEV_1ST_GEN = "KIA K8 HYBRID 1ST GEN"
  KIA_NIRO_EV = "KIA NIRO EV 2020"
  KIA_NIRO_EV_2ND_GEN = "KIA NIRO EV 2ND GEN"
  KIA_NIRO_PHEV = "KIA NIRO HYBRID 2019"
  KIA_NIRO_PHEV_2022 = "KIA NIRO PLUG-IN HYBRID 2022"
  KIA_NIRO_HEV_2021 = "KIA NIRO HYBRID 2021"
  KIA_NIRO_HEV_2ND_GEN = "KIA NIRO HYBRID 2ND GEN"
  KIA_OPTIMA_G4 = "KIA OPTIMA 4TH GEN"
  KIA_OPTIMA_G4_FL = "KIA OPTIMA 4TH GEN FACELIFT"
  KIA_OPTIMA_H = "KIA OPTIMA HYBRID 2017 & SPORTS 2019"
  KIA_OPTIMA_H_G4_FL = "KIA OPTIMA HYBRID 4TH GEN FACELIFT"
  KIA_SELTOS = "KIA SELTOS 2021"
  KIA_SPORTAGE_5TH_GEN = "KIA SPORTAGE 5TH GEN"
  KIA_SORENTO = "KIA SORENTO GT LINE 2018"
  KIA_SOUL_EV_SK3= "KIA SOUL EV (SK3)"
  KIA_SORENTO_4TH_GEN = "KIA SORENTO 4TH GEN"
  KIA_SORENTO_HEV_4TH_GEN = "KIA SORENTO HYBRID 4TH GEN"
  KIA_SORENTO_PHEV_4TH_GEN = "KIA SORENTO PLUG-IN HYBRID 4TH GEN"
  KIA_SPORTAGE_HYBRID_5TH_GEN = "KIA SPORTAGE HYBRID 5TH GEN"
  KIA_STINGER = "KIA STINGER GT2 2018"
  KIA_STINGER_2022 = "KIA STINGER 2022"
  KIA_CEED = "KIA CEED INTRO ED 2019"
  KIA_EV6 = "KIA EV6 2022"
  KIA_CARNIVAL_4TH_GEN = "KIA CARNIVAL 4TH GEN"

  # Genesis
  GENESIS_GV60_EV_1ST_GEN = "GENESIS GV60 ELECTRIC 1ST GEN"
  GENESIS_G70 = "GENESIS G70 2018"
  GENESIS_G70_2020 = "GENESIS G70 2020"
  GENESIS_GV70_1ST_GEN = "GENESIS GV70 1ST GEN"
  GENESIS_G80 = "GENESIS G80 2017"
  GENESIS_G90 = "GENESIS G90 2017"
  GENESIS_GV80 = "GENESIS GV80 2023"
  GENESIS_EGV70 = "GENESIS EGV70 1ST GEN"


  K7 = "KIA K7 2016-2019"
  K7_HEV = "KIA K7 HEV 2016-2019"
class Footnote(Enum):
  CANFD = CarFootnote(
    "Requires a <a href=\"https://comma.ai/shop/can-fd-panda-kit\" target=\"_blank\">CAN FD panda kit</a> if not using " +
    "comma 3X for this <a href=\"https://en.wikipedia.org/wiki/CAN_FD\" target=\"_blank\">CAN FD car</a>.",
    Column.MODEL, shop_footnote=False)


@dataclass
class HyundaiCarInfo(CarInfo):
  package: str = "Smart Cruise Control (SCC)"

  def init_make(self, CP: car.CarParams):
    if CP.flags & HyundaiFlags.CANFD:
      self.footnotes.insert(0, Footnote.CANFD)


CAR_INFO: Dict[str, Optional[Union[HyundaiCarInfo, List[HyundaiCarInfo]]]] = {
  CAR.AZERA_6TH_GEN: HyundaiCarInfo("Hyundai Azera 2022", "All", car_parts=CarParts.common([CarHarness.hyundai_k])),
  CAR.AZERA_HEV_6TH_GEN: [
    HyundaiCarInfo("Hyundai Azera Hybrid 2019", "All", car_parts=CarParts.common([CarHarness.hyundai_c])),
    HyundaiCarInfo("Hyundai Azera Hybrid 2020", "All", car_parts=CarParts.common([CarHarness.hyundai_k])),
  ],
  CAR.ELANTRA: [
    # TODO: 2017-18 could be Hyundai G
    HyundaiCarInfo("Hyundai Elantra 2017-18", min_enable_speed=19 * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_b])),
    HyundaiCarInfo("Hyundai Elantra 2019", min_enable_speed=19 * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_g])),
  ],
  CAR.ELANTRA_GT_I30: [
    HyundaiCarInfo("Hyundai Elantra GT 2017-19", car_parts=CarParts.common([CarHarness.hyundai_e])),
    HyundaiCarInfo("Hyundai i30 2017-19", car_parts=CarParts.common([CarHarness.hyundai_e])),
  ],
  CAR.ELANTRA_2021: HyundaiCarInfo("Hyundai Elantra 2021-23", video_link="https://youtu.be/_EdYQtV52-c", car_parts=CarParts.common([CarHarness.hyundai_k])),
  CAR.ELANTRA_HEV_2021: HyundaiCarInfo("Hyundai Elantra Hybrid 2021-23", video_link="https://youtu.be/_EdYQtV52-c",
                                       car_parts=CarParts.common([CarHarness.hyundai_k])),
  CAR.HYUNDAI_GENESIS: [
    # TODO: check 2015 packages
    HyundaiCarInfo("Hyundai Genesis 2015-16", min_enable_speed=19 * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_j])),
    HyundaiCarInfo("Genesis G80 2017", "All", min_enable_speed=19 * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_j])),
  ],
  CAR.IONIQ: HyundaiCarInfo("Hyundai Ioniq Hybrid 2017-19", car_parts=CarParts.common([CarHarness.hyundai_c])),
  CAR.IONIQ_HEV_2022: HyundaiCarInfo("Hyundai Ioniq Hybrid 2020-22", car_parts=CarParts.common([CarHarness.hyundai_h])),  # TODO: confirm 2020-21 harness
  CAR.IONIQ_EV_LTD: HyundaiCarInfo("Hyundai Ioniq Electric 2019", car_parts=CarParts.common([CarHarness.hyundai_c])),
  CAR.IONIQ_EV_2020: HyundaiCarInfo("Hyundai Ioniq Electric 2020", "All", car_parts=CarParts.common([CarHarness.hyundai_h])),
  CAR.IONIQ_PHEV_2019: HyundaiCarInfo("Hyundai Ioniq Plug-in Hybrid 2019", car_parts=CarParts.common([CarHarness.hyundai_c])),
  CAR.IONIQ_PHEV: HyundaiCarInfo("Hyundai Ioniq Plug-in Hybrid 2020-22", "All", car_parts=CarParts.common([CarHarness.hyundai_h])),
  CAR.KONA: HyundaiCarInfo("Hyundai Kona 2020", car_parts=CarParts.common([CarHarness.hyundai_b])),
  CAR.KONA_EV: HyundaiCarInfo("Hyundai Kona Electric 2018-21", car_parts=CarParts.common([CarHarness.hyundai_g])),
  CAR.KONA_EV_2022: HyundaiCarInfo("Hyundai Kona Electric 2022-23", car_parts=CarParts.common([CarHarness.hyundai_o])),
  CAR.KONA_HEV: HyundaiCarInfo("Hyundai Kona Hybrid 2020", car_parts=CarParts.common([CarHarness.hyundai_i])),  # TODO: check packages
  # TODO: this is the 2024 US MY, not yet released
  CAR.KONA_EV_2ND_GEN: HyundaiCarInfo("Hyundai Kona Electric (with HDA II, Korea only) 2023", video_link="https://www.youtube.com/watch?v=U2fOCmcQ8hw",
                                      car_parts=CarParts.common([CarHarness.hyundai_r])),
  CAR.SANTA_FE: HyundaiCarInfo("Hyundai Santa Fe 2019-20", "All", video_link="https://youtu.be/bjDR0YjM__s",
                               car_parts=CarParts.common([CarHarness.hyundai_d])),
  CAR.SANTA_FE_2022: HyundaiCarInfo("Hyundai Santa Fe 2021-23", "All", video_link="https://youtu.be/VnHzSTygTS4",
                                    car_parts=CarParts.common([CarHarness.hyundai_l])),
  CAR.SANTA_FE_HEV_2022: HyundaiCarInfo("Hyundai Santa Fe Hybrid 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_l])),
  CAR.SANTA_FE_PHEV_2022: HyundaiCarInfo("Hyundai Santa Fe Plug-in Hybrid 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_l])),
  CAR.SONATA: HyundaiCarInfo("Hyundai Sonata 2020-23", "All", video_link="https://www.youtube.com/watch?v=ix63r9kE3Fw",
                             car_parts=CarParts.common([CarHarness.hyundai_a])),
  CAR.STARIA_4TH_GEN: HyundaiCarInfo("Hyundai Staria 2023", "All", car_parts=CarParts.common([CarHarness.hyundai_k])),
  CAR.SONATA_LF: HyundaiCarInfo("Hyundai Sonata 2018-19", car_parts=CarParts.common([CarHarness.hyundai_e])),
  CAR.TUCSON: [
    HyundaiCarInfo("Hyundai Tucson 2021", min_enable_speed=19 * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_l])),
    HyundaiCarInfo("Hyundai Tucson Diesel 2019", car_parts=CarParts.common([CarHarness.hyundai_l])),
  ],
  CAR.PALISADE: [
    HyundaiCarInfo("Hyundai Palisade 2020-22", "All", video_link="https://youtu.be/TAnDqjF4fDY?t=456", car_parts=CarParts.common([CarHarness.hyundai_h])),
    HyundaiCarInfo("Kia Telluride 2020-22", "All", car_parts=CarParts.common([CarHarness.hyundai_h])),
  ],
  CAR.VELOSTER: HyundaiCarInfo("Hyundai Veloster 2019-20", min_enable_speed=5. * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_e])),
  CAR.SONATA_HYBRID: HyundaiCarInfo("Hyundai Sonata Hybrid 2020-23", "All", car_parts=CarParts.common([CarHarness.hyundai_a])),
  CAR.IONIQ_5: [
    HyundaiCarInfo("Hyundai Ioniq 5 (Southeast Asia only) 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_q])),
    HyundaiCarInfo("Hyundai Ioniq 5 (without HDA II) 2022-23", "Highway Driving Assist", car_parts=CarParts.common([CarHarness.hyundai_k])),
    HyundaiCarInfo("Hyundai Ioniq 5 (with HDA II) 2022-23", "Highway Driving Assist II", car_parts=CarParts.common([CarHarness.hyundai_q])),
  ],
  CAR.IONIQ_6: [
    HyundaiCarInfo("Hyundai Ioniq 6 (with HDA II) 2023", "Highway Driving Assist II", car_parts=CarParts.common([CarHarness.hyundai_p])),
  ],
  CAR.TUCSON_4TH_GEN: [
    HyundaiCarInfo("Hyundai Tucson 2022", car_parts=CarParts.common([CarHarness.hyundai_n])),
    HyundaiCarInfo("Hyundai Tucson 2023", "All", car_parts=CarParts.common([CarHarness.hyundai_n])),
    HyundaiCarInfo("Hyundai Tucson Hybrid 2022-24", "All", car_parts=CarParts.common([CarHarness.hyundai_n])),
  ],
  CAR.TUCSON_HYBRID_4TH_GEN: HyundaiCarInfo("Hyundai Tucson Hybrid 2022-24", "All", car_parts=CarParts.common([CarHarness.hyundai_n])),
  CAR.SANTA_CRUZ_1ST_GEN: HyundaiCarInfo("Hyundai Santa Cruz 2022-23", car_parts=CarParts.common([CarHarness.hyundai_n])),
  CAR.CUSTIN_1ST_GEN: HyundaiCarInfo("Hyundai Custin 2023", "All", car_parts=CarParts.common([CarHarness.hyundai_k])),

  # Kia
  CAR.KIA_FORTE: [
    HyundaiCarInfo("Kia Forte 2019-21", car_parts=CarParts.common([CarHarness.hyundai_g])),
    HyundaiCarInfo("Kia Forte 2023", car_parts=CarParts.common([CarHarness.hyundai_e])),
  ],
  CAR.KIA_K5_2021: HyundaiCarInfo("Kia K5 2021-24", car_parts=CarParts.common([CarHarness.hyundai_a])),
  CAR.KIA_K5_HEV_2020: HyundaiCarInfo("Kia K5 Hybrid 2020-22", car_parts=CarParts.common([CarHarness.hyundai_a])),
  CAR.KIA_K8_HEV_1ST_GEN: HyundaiCarInfo("Kia K8 Hybrid (with HDA II) 2023", "Highway Driving Assist II", car_parts=CarParts.common([CarHarness.hyundai_q])),
  CAR.KIA_NIRO_EV: [
    HyundaiCarInfo("Kia Niro EV 2019", "All", video_link="https://www.youtube.com/watch?v=lT7zcG6ZpGo", car_parts=CarParts.common([CarHarness.hyundai_h])),
    HyundaiCarInfo("Kia Niro EV 2020", "All", video_link="https://www.youtube.com/watch?v=lT7zcG6ZpGo", car_parts=CarParts.common([CarHarness.hyundai_f])),
    HyundaiCarInfo("Kia Niro EV 2021", "All", video_link="https://www.youtube.com/watch?v=lT7zcG6ZpGo", car_parts=CarParts.common([CarHarness.hyundai_c])),
    HyundaiCarInfo("Kia Niro EV 2022", "All", video_link="https://www.youtube.com/watch?v=lT7zcG6ZpGo", car_parts=CarParts.common([CarHarness.hyundai_h])),
  ],
  CAR.KIA_NIRO_EV_2ND_GEN: HyundaiCarInfo("Kia Niro EV 2023", "All", car_parts=CarParts.common([CarHarness.hyundai_a])),
  CAR.KIA_NIRO_PHEV: [
    HyundaiCarInfo("Kia Niro Plug-in Hybrid 2018-19", "All", min_enable_speed=10. * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_c])),
    HyundaiCarInfo("Kia Niro Plug-in Hybrid 2020", "All", car_parts=CarParts.common([CarHarness.hyundai_d])),
  ],
  CAR.KIA_NIRO_PHEV_2022: [
    HyundaiCarInfo("Kia Niro Plug-in Hybrid 2021", "All", car_parts=CarParts.common([CarHarness.hyundai_d])),
    HyundaiCarInfo("Kia Niro Plug-in Hybrid 2022", "All", car_parts=CarParts.common([CarHarness.hyundai_f])),
  ],
  CAR.KIA_NIRO_HEV_2021: [
    HyundaiCarInfo("Kia Niro Hybrid 2021", car_parts=CarParts.common([CarHarness.hyundai_d])),
    HyundaiCarInfo("Kia Niro Hybrid 2022", car_parts=CarParts.common([CarHarness.hyundai_f])),
  ],
  CAR.KIA_NIRO_HEV_2ND_GEN: HyundaiCarInfo("Kia Niro Hybrid 2023", car_parts=CarParts.common([CarHarness.hyundai_a])),
  CAR.KIA_OPTIMA_G4: HyundaiCarInfo("Kia Optima 2017", "Advanced Smart Cruise Control",
                                    car_parts=CarParts.common([CarHarness.hyundai_b])),  # TODO: may support 2016, 2018
  CAR.KIA_OPTIMA_G4_FL: HyundaiCarInfo("Kia Optima 2019-20", car_parts=CarParts.common([CarHarness.hyundai_g])),
  # TODO: may support adjacent years. may have a non-zero minimum steering speed
  CAR.KIA_OPTIMA_H: HyundaiCarInfo("Kia Optima Hybrid 2017", "Advanced Smart Cruise Control", car_parts=CarParts.common([CarHarness.hyundai_c])),
  CAR.KIA_OPTIMA_H_G4_FL: HyundaiCarInfo("Kia Optima Hybrid 2019", car_parts=CarParts.common([CarHarness.hyundai_h])),
  CAR.KIA_SELTOS: HyundaiCarInfo("Kia Seltos 2021", car_parts=CarParts.common([CarHarness.hyundai_a])),
  CAR.KIA_SPORTAGE_5TH_GEN: [
    HyundaiCarInfo("Kia Sportage 2023", car_parts=CarParts.common([CarHarness.hyundai_n])),
    HyundaiCarInfo("Kia Sportage Hybrid 2023", car_parts=CarParts.common([CarHarness.hyundai_n])),
  ],
  CAR.KIA_SORENTO: [
    HyundaiCarInfo("Kia Sorento 2018", "Advanced Smart Cruise Control & LKAS", video_link="https://www.youtube.com/watch?v=Fkh3s6WHJz8",
                   car_parts=CarParts.common([CarHarness.hyundai_e])),
    HyundaiCarInfo("Kia Sorento 2019", video_link="https://www.youtube.com/watch?v=Fkh3s6WHJz8", car_parts=CarParts.common([CarHarness.hyundai_e])),
  ],
  CAR.KIA_SORENTO_4TH_GEN: HyundaiCarInfo("Kia Sorento 2021-23", car_parts=CarParts.common([CarHarness.hyundai_k])),
  CAR.KIA_SORENTO_HEV_4TH_GEN: [
    HyundaiCarInfo("Kia Sorento Hybrid 2021-23", "All", car_parts=CarParts.common([CarHarness.hyundai_a])),
    HyundaiCarInfo("Kia Sorento Plug-in Hybrid 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_a])),
  ],
  CAR.KIA_STINGER: HyundaiCarInfo("Kia Stinger 2018-20", video_link="https://www.youtube.com/watch?v=MJ94qoofYw0",
                                  car_parts=CarParts.common([CarHarness.hyundai_c])),
  CAR.KIA_STINGER_2022: HyundaiCarInfo("Kia Stinger 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_k])),
  CAR.KIA_CEED: HyundaiCarInfo("Kia Ceed 2019", car_parts=CarParts.common([CarHarness.hyundai_e])),
  CAR.KIA_EV6: [
    HyundaiCarInfo("Kia EV6 (Southeast Asia only) 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_p])),
    HyundaiCarInfo("Kia EV6 (without HDA II) 2022-23", "Highway Driving Assist", car_parts=CarParts.common([CarHarness.hyundai_l])),
    HyundaiCarInfo("Kia EV6 (with HDA II) 2022-23", "Highway Driving Assist II", car_parts=CarParts.common([CarHarness.hyundai_p]))
  ],
  CAR.KIA_CARNIVAL_4TH_GEN: [
    HyundaiCarInfo("Kia Carnival 2022-24", car_parts=CarParts.common([CarHarness.hyundai_a])),
    HyundaiCarInfo("Kia Carnival (China only) 2023", car_parts=CarParts.common([CarHarness.hyundai_k]))
  ],

  # Genesis
  CAR.GENESIS_GV60_EV_1ST_GEN: [
    HyundaiCarInfo("Genesis GV60 (Advanced Trim) 2023", "All", car_parts=CarParts.common([CarHarness.hyundai_a])),
    HyundaiCarInfo("Genesis GV60 (Performance Trim) 2023", "All", car_parts=CarParts.common([CarHarness.hyundai_k])),
  ],
  CAR.GENESIS_G70: HyundaiCarInfo("Genesis G70 2018-19", "All", car_parts=CarParts.common([CarHarness.hyundai_f])),
  CAR.GENESIS_G70_2020: HyundaiCarInfo("Genesis G70 2020", "All", car_parts=CarParts.common([CarHarness.hyundai_f])),
  CAR.GENESIS_GV70_1ST_GEN: [
    HyundaiCarInfo("Genesis GV70 (2.5T Trim) 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_l])),
    HyundaiCarInfo("Genesis GV70 (3.5T Trim) 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_m])),
  ],
  CAR.GENESIS_G80: HyundaiCarInfo("Genesis G80 2018-19", "All", car_parts=CarParts.common([CarHarness.hyundai_h])),
  CAR.GENESIS_G90: HyundaiCarInfo("Genesis G90 2017-18", "All", car_parts=CarParts.common([CarHarness.hyundai_c])),
  CAR.GENESIS_GV80: HyundaiCarInfo("Genesis GV80 2023", "All", car_parts=CarParts.common([CarHarness.hyundai_m])),
  CAR.GENESIS_EGV70: HyundaiCarInfo("Genesis EGV70 2020-23", "All", car_parts=CarParts.common([CarHarness.hyundai_m])),
}
@dataclass
class HyundaiPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict("hyundai_kia_generic", None))

  def init(self):
    if self.flags & HyundaiFlags.MANDO_RADAR:
      self.dbc_dict = dbc_dict('hyundai_kia_generic', 'hyundai_kia_mando_front_radar_generated')

    if self.flags & HyundaiFlags.MIN_STEER_32_MPH:
      self.specs = self.specs.override(minSteerSpeed=32 * CV.MPH_TO_MS)


@dataclass
class HyundaiCanFDPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict("hyundai_canfd", None))

  def init(self):
    self.flags |= HyundaiFlags.CANFD


class CAR(Platforms):
  # Hyundai
  AZERA_6TH_GEN = HyundaiPlatformConfig(
    "HYUNDAI AZERA 6TH GEN",
    HyundaiCarInfo("Hyundai Azera 2022", "All", car_parts=CarParts.common([CarHarness.hyundai_k])),
    CarSpecs(mass=1600, wheelbase=2.885, steerRatio=14.5),
  )
  AZERA_HEV_6TH_GEN = HyundaiPlatformConfig(
    "HYUNDAI AZERA HYBRID 6TH GEN",
    [
      HyundaiCarInfo("Hyundai Azera Hybrid 2019", "All", car_parts=CarParts.common([CarHarness.hyundai_c])),
      HyundaiCarInfo("Hyundai Azera Hybrid 2020", "All", car_parts=CarParts.common([CarHarness.hyundai_k])),
    ],
    CarSpecs(mass=1675, wheelbase=2.885, steerRatio=14.5),
    flags=HyundaiFlags.HYBRID,
  )
  ELANTRA = HyundaiPlatformConfig(
    "HYUNDAI ELANTRA 2017",
    [
      # TODO: 2017-18 could be Hyundai G
      HyundaiCarInfo("Hyundai Elantra 2017-18", min_enable_speed=19 * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_b])),
      HyundaiCarInfo("Hyundai Elantra 2019", min_enable_speed=19 * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_g])),
    ],
    # steerRatio: 14 is Stock | Settled Params Learner values are steerRatio: 15.401566348670535, stiffnessFactor settled on 1.0081302973865127
    CarSpecs(mass=1275, wheelbase=2.7, steerRatio=15.4, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.LEGACY | HyundaiFlags.CLUSTER_GEARS | HyundaiFlags.MIN_STEER_32_MPH,
  )
  ELANTRA_GT_I30 = HyundaiPlatformConfig(
    "HYUNDAI I30 N LINE 2019 & GT 2018 DCT",
    [
      HyundaiCarInfo("Hyundai Elantra GT 2017-19", car_parts=CarParts.common([CarHarness.hyundai_e])),
      HyundaiCarInfo("Hyundai i30 2017-19", car_parts=CarParts.common([CarHarness.hyundai_e])),
    ],
    ELANTRA.specs,
    flags=HyundaiFlags.LEGACY | HyundaiFlags.CLUSTER_GEARS | HyundaiFlags.MIN_STEER_32_MPH,
  )
  ELANTRA_2021 = HyundaiPlatformConfig(
    "HYUNDAI ELANTRA 2021",
    HyundaiCarInfo("Hyundai Elantra 2021-23", video_link="https://youtu.be/_EdYQtV52-c", car_parts=CarParts.common([CarHarness.hyundai_k])),
    CarSpecs(mass=2800 * CV.LB_TO_KG, wheelbase=2.72, steerRatio=12.9, tireStiffnessFactor=0.65),
    flags=HyundaiFlags.CHECKSUM_CRC8,
  )
  ELANTRA_HEV_2021 = HyundaiPlatformConfig(
    "HYUNDAI ELANTRA HYBRID 2021",
    HyundaiCarInfo("Hyundai Elantra Hybrid 2021-23", video_link="https://youtu.be/_EdYQtV52-c",
                   car_parts=CarParts.common([CarHarness.hyundai_k])),
    CarSpecs(mass=3017 * CV.LB_TO_KG, wheelbase=2.72, steerRatio=12.9, tireStiffnessFactor=0.65),
    flags=HyundaiFlags.CHECKSUM_CRC8 | HyundaiFlags.HYBRID,
  )
  HYUNDAI_GENESIS = HyundaiPlatformConfig(
    "HYUNDAI GENESIS 2015-2016",
    [
      # TODO: check 2015 packages
      HyundaiCarInfo("Hyundai Genesis 2015-16", min_enable_speed=19 * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_j])),
      HyundaiCarInfo("Genesis G80 2017", "All", min_enable_speed=19 * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_j])),
    ],
    CarSpecs(mass=2060, wheelbase=3.01, steerRatio=16.5, minSteerSpeed=60 * CV.KPH_TO_MS),
    flags=HyundaiFlags.CHECKSUM_6B | HyundaiFlags.LEGACY,
  )
  IONIQ = HyundaiPlatformConfig(
    "HYUNDAI IONIQ HYBRID 2017-2019",
    HyundaiCarInfo("Hyundai Ioniq Hybrid 2017-19", car_parts=CarParts.common([CarHarness.hyundai_c])),
    CarSpecs(mass=1490, wheelbase=2.7, steerRatio=13.73, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.HYBRID | HyundaiFlags.MIN_STEER_32_MPH,
  )
  IONIQ_HEV_2022 = HyundaiPlatformConfig(
    "HYUNDAI IONIQ HYBRID 2020-2022",
    HyundaiCarInfo("Hyundai Ioniq Hybrid 2020-22", car_parts=CarParts.common([CarHarness.hyundai_h])),  # TODO: confirm 2020-21 harness,
    CarSpecs(mass=1490, wheelbase=2.7, steerRatio=13.73, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.HYBRID | HyundaiFlags.LEGACY,
  )
  IONIQ_EV_LTD = HyundaiPlatformConfig(
    "HYUNDAI IONIQ ELECTRIC LIMITED 2019",
    HyundaiCarInfo("Hyundai Ioniq Electric 2019", car_parts=CarParts.common([CarHarness.hyundai_c])),
    CarSpecs(mass=1490, wheelbase=2.7, steerRatio=13.73, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.EV | HyundaiFlags.LEGACY | HyundaiFlags.MIN_STEER_32_MPH,
  )
  IONIQ_EV_2020 = HyundaiPlatformConfig(
    "HYUNDAI IONIQ ELECTRIC 2020",
    HyundaiCarInfo("Hyundai Ioniq Electric 2020", "All", car_parts=CarParts.common([CarHarness.hyundai_h])),
    CarSpecs(mass=1490, wheelbase=2.7, steerRatio=13.73, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.EV,
  )
  IONIQ_PHEV_2019 = HyundaiPlatformConfig(
    "HYUNDAI IONIQ PLUG-IN HYBRID 2019",
    HyundaiCarInfo("Hyundai Ioniq Plug-in Hybrid 2019", car_parts=CarParts.common([CarHarness.hyundai_c])),
    CarSpecs(mass=1490, wheelbase=2.7, steerRatio=13.73, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.HYBRID | HyundaiFlags.MIN_STEER_32_MPH,
  )
  IONIQ_PHEV = HyundaiPlatformConfig(
    "HYUNDAI IONIQ PHEV 2020",
    HyundaiCarInfo("Hyundai Ioniq Plug-in Hybrid 2020-22", "All", car_parts=CarParts.common([CarHarness.hyundai_h])),
    CarSpecs(mass=1490, wheelbase=2.7, steerRatio=13.73, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.HYBRID,
  )
  KONA = HyundaiPlatformConfig(
    "HYUNDAI KONA 2020",
    HyundaiCarInfo("Hyundai Kona 2020", car_parts=CarParts.common([CarHarness.hyundai_b])),
    CarSpecs(mass=1275, wheelbase=2.6, steerRatio=13.42, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.CLUSTER_GEARS,
  )
  KONA_EV = HyundaiPlatformConfig(
    "HYUNDAI KONA ELECTRIC 2019",
    HyundaiCarInfo("Hyundai Kona Electric 2018-21", car_parts=CarParts.common([CarHarness.hyundai_g])),
    CarSpecs(mass=1685, wheelbase=2.6, steerRatio=13.42, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.EV,
  )
  KONA_EV_2022 = HyundaiPlatformConfig(
    "HYUNDAI KONA ELECTRIC 2022",
    HyundaiCarInfo("Hyundai Kona Electric 2022-23", car_parts=CarParts.common([CarHarness.hyundai_o])),
    CarSpecs(mass=1743, wheelbase=2.6, steerRatio=13.42, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.CAMERA_SCC | HyundaiFlags.EV,
  )
  KONA_EV_2ND_GEN = HyundaiCanFDPlatformConfig(
    "HYUNDAI KONA ELECTRIC 2ND GEN",
    HyundaiCarInfo("Hyundai Kona Electric (with HDA II, Korea only) 2023", video_link="https://www.youtube.com/watch?v=U2fOCmcQ8hw",
                   car_parts=CarParts.common([CarHarness.hyundai_r])),
    CarSpecs(mass=1740, wheelbase=2.66, steerRatio=13.6, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.EV | HyundaiFlags.CANFD_NO_RADAR_DISABLE,
  )
  KONA_HEV = HyundaiPlatformConfig(
    "HYUNDAI KONA HYBRID 2020",
    HyundaiCarInfo("Hyundai Kona Hybrid 2020", car_parts=CarParts.common([CarHarness.hyundai_i])),  # TODO: check packages,
    CarSpecs(mass=1425, wheelbase=2.6, steerRatio=13.42, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.HYBRID,
  )
  SANTA_FE = HyundaiPlatformConfig(
    "HYUNDAI SANTA FE 2019",
    HyundaiCarInfo("Hyundai Santa Fe 2019-20", "All", video_link="https://youtu.be/bjDR0YjM__s",
                   car_parts=CarParts.common([CarHarness.hyundai_d])),
    CarSpecs(mass=3982 * CV.LB_TO_KG, wheelbase=2.766, steerRatio=16.55, tireStiffnessFactor=0.82),
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.CHECKSUM_CRC8,
  )
  SANTA_FE_2022 = HyundaiPlatformConfig(
    "HYUNDAI SANTA FE 2022",
    HyundaiCarInfo("Hyundai Santa Fe 2021-23", "All", video_link="https://youtu.be/VnHzSTygTS4",
                   car_parts=CarParts.common([CarHarness.hyundai_l])),
    SANTA_FE.specs,
    flags=HyundaiFlags.CHECKSUM_CRC8,
  )
  SANTA_FE_HEV_2022 = HyundaiPlatformConfig(
    "HYUNDAI SANTA FE HYBRID 2022",
    HyundaiCarInfo("Hyundai Santa Fe Hybrid 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_l])),
    SANTA_FE.specs,
    flags=HyundaiFlags.CHECKSUM_CRC8 | HyundaiFlags.HYBRID,
  )
  SANTA_FE_PHEV_2022 = HyundaiPlatformConfig(
    "HYUNDAI SANTA FE PlUG-IN HYBRID 2022",
    HyundaiCarInfo("Hyundai Santa Fe Plug-in Hybrid 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_l])),
    SANTA_FE.specs,
    flags=HyundaiFlags.CHECKSUM_CRC8 | HyundaiFlags.HYBRID,
  )
  SONATA = HyundaiPlatformConfig(
    "HYUNDAI SONATA 2020",
    HyundaiCarInfo("Hyundai Sonata 2020-23", "All", video_link="https://www.youtube.com/watch?v=ix63r9kE3Fw",
                   car_parts=CarParts.common([CarHarness.hyundai_a])),
    CarSpecs(mass=1513, wheelbase=2.84, steerRatio=13.27 * 1.15, tireStiffnessFactor=0.65),  # 15% higher at the center seems reasonable
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.CHECKSUM_CRC8,
  )
  SONATA_LF = HyundaiPlatformConfig(
    "HYUNDAI SONATA 2019",
    HyundaiCarInfo("Hyundai Sonata 2018-19", car_parts=CarParts.common([CarHarness.hyundai_e])),
    CarSpecs(mass=1536, wheelbase=2.804, steerRatio=13.27 * 1.15),  # 15% higher at the center seems reasonable

    flags=HyundaiFlags.UNSUPPORTED_LONGITUDINAL | HyundaiFlags.TCU_GEARS,
  )
  STARIA_4TH_GEN = HyundaiCanFDPlatformConfig(
    "HYUNDAI STARIA 4TH GEN",
    HyundaiCarInfo("Hyundai Staria 2023", "All", car_parts=CarParts.common([CarHarness.hyundai_k])),
    CarSpecs(mass=2205, wheelbase=3.273, steerRatio=11.94),  # https://www.hyundai.com/content/dam/hyundai/au/en/models/staria-load/premium-pip-update-2023/spec-sheet/STARIA_Load_Spec-Table_March_2023_v3.1.pdf
  )
  TUCSON = HyundaiPlatformConfig(
    "HYUNDAI TUCSON 2019",
    [
      HyundaiCarInfo("Hyundai Tucson 2021", min_enable_speed=19 * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_l])),
      HyundaiCarInfo("Hyundai Tucson Diesel 2019", car_parts=CarParts.common([CarHarness.hyundai_l])),
    ],
    CarSpecs(mass=3520 * CV.LB_TO_KG, wheelbase=2.67, steerRatio=16.1, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.TCU_GEARS,
  )
  PALISADE = HyundaiPlatformConfig(
    "HYUNDAI PALISADE 2020",
    [
      HyundaiCarInfo("Hyundai Palisade 2020-22", "All", video_link="https://youtu.be/TAnDqjF4fDY?t=456", car_parts=CarParts.common([CarHarness.hyundai_h])),
      HyundaiCarInfo("Kia Telluride 2020-22", "All", car_parts=CarParts.common([CarHarness.hyundai_h])),
    ],
    CarSpecs(mass=1999, wheelbase=2.9, steerRatio=15.6 * 1.15, tireStiffnessFactor=0.63),
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.CHECKSUM_CRC8,
  )
  VELOSTER = HyundaiPlatformConfig(
    "HYUNDAI VELOSTER 2019",
    HyundaiCarInfo("Hyundai Veloster 2019-20", min_enable_speed=5. * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_e])),
    CarSpecs(mass=2917 * CV.LB_TO_KG, wheelbase=2.8, steerRatio=13.75 * 1.15, tireStiffnessFactor=0.5),
    flags=HyundaiFlags.LEGACY | HyundaiFlags.TCU_GEARS,
  )
  SONATA_HYBRID = HyundaiPlatformConfig(
    "HYUNDAI SONATA HYBRID 2021",
    HyundaiCarInfo("Hyundai Sonata Hybrid 2020-23", "All", car_parts=CarParts.common([CarHarness.hyundai_a])),
    SONATA.specs,
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.CHECKSUM_CRC8 | HyundaiFlags.HYBRID,
  )
  IONIQ_5 = HyundaiCanFDPlatformConfig(
    "HYUNDAI IONIQ 5 2022",
    [
      HyundaiCarInfo("Hyundai Ioniq 5 (Southeast Asia only) 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_q])),
      HyundaiCarInfo("Hyundai Ioniq 5 (without HDA II) 2022-23", "Highway Driving Assist", car_parts=CarParts.common([CarHarness.hyundai_k])),
      HyundaiCarInfo("Hyundai Ioniq 5 (with HDA II) 2022-23", "Highway Driving Assist II", car_parts=CarParts.common([CarHarness.hyundai_q])),
    ],
    CarSpecs(mass=1948, wheelbase=2.97, steerRatio=14.26, tireStiffnessFactor=0.65),
    flags=HyundaiFlags.EV,
  )
  IONIQ_6 = HyundaiCanFDPlatformConfig(
    "HYUNDAI IONIQ 6 2023",
    HyundaiCarInfo("Hyundai Ioniq 6 (with HDA II) 2023", "Highway Driving Assist II", car_parts=CarParts.common([CarHarness.hyundai_p])),
    IONIQ_5.specs,
    flags=HyundaiFlags.EV | HyundaiFlags.CANFD_NO_RADAR_DISABLE,
  )
  TUCSON_4TH_GEN = HyundaiCanFDPlatformConfig(
    "HYUNDAI TUCSON 4TH GEN",
    [
      HyundaiCarInfo("Hyundai Tucson 2022", car_parts=CarParts.common([CarHarness.hyundai_n])),
      HyundaiCarInfo("Hyundai Tucson 2023", "All", car_parts=CarParts.common([CarHarness.hyundai_n])),
      HyundaiCarInfo("Hyundai Tucson Hybrid 2022-24", "All", car_parts=CarParts.common([CarHarness.hyundai_n])),
    ],
    CarSpecs(mass=1630, wheelbase=2.756, steerRatio=16, tireStiffnessFactor=0.385),
  )
  SANTA_CRUZ_1ST_GEN = HyundaiCanFDPlatformConfig(
    "HYUNDAI SANTA CRUZ 1ST GEN",
    HyundaiCarInfo("Hyundai Santa Cruz 2022-23", car_parts=CarParts.common([CarHarness.hyundai_n])),
    # weight from Limited trim - the only supported trim, steering ratio according to Hyundai News https://www.hyundainews.com/assets/documents/original/48035-2022SantaCruzProductGuideSpecsv2081521.pdf
    CarSpecs(mass=1870, wheelbase=3, steerRatio=14.2),
  )
  CUSTIN_1ST_GEN = HyundaiPlatformConfig(
    "HYUNDAI CUSTIN 1ST GEN",
    HyundaiCarInfo("Hyundai Custin 2023", "All", car_parts=CarParts.common([CarHarness.hyundai_k])),
    CarSpecs(mass=1690, wheelbase=3.055, steerRatio=17),  # mass: from https://www.hyundai-motor.com.tw/clicktobuy/custin#spec_0, steerRatio: from learner
    flags=HyundaiFlags.CHECKSUM_CRC8,
  )

  # Kia
  KIA_FORTE = HyundaiPlatformConfig(
    "KIA FORTE E 2018 & GT 2021",
    [
      HyundaiCarInfo("Kia Forte 2019-21", car_parts=CarParts.common([CarHarness.hyundai_g])),
      HyundaiCarInfo("Kia Forte 2023", car_parts=CarParts.common([CarHarness.hyundai_e])),
    ],
    CarSpecs(mass=2878 * CV.LB_TO_KG, wheelbase=2.8, steerRatio=13.75, tireStiffnessFactor=0.5)
  )
  KIA_K5_2021 = HyundaiPlatformConfig(
    "KIA K5 2021",
    HyundaiCarInfo("Kia K5 2021-24", car_parts=CarParts.common([CarHarness.hyundai_a])),
    CarSpecs(mass=3381 * CV.LB_TO_KG, wheelbase=2.85, steerRatio=13.27, tireStiffnessFactor=0.5),  # 2021 Kia K5 Steering Ratio (all trims)
    flags=HyundaiFlags.CHECKSUM_CRC8,
  )
  KIA_K5_HEV_2020 = HyundaiPlatformConfig(
    "KIA K5 HYBRID 2020",
    HyundaiCarInfo("Kia K5 Hybrid 2020-22", car_parts=CarParts.common([CarHarness.hyundai_a])),
    KIA_K5_2021.specs,
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.CHECKSUM_CRC8 | HyundaiFlags.HYBRID,
  )
  KIA_K8_HEV_1ST_GEN = HyundaiCanFDPlatformConfig(
    "KIA K8 HYBRID 1ST GEN",
    HyundaiCarInfo("Kia K8 Hybrid (with HDA II) 2023", "Highway Driving Assist II", car_parts=CarParts.common([CarHarness.hyundai_q])),
    # mass: https://carprices.ae/brands/kia/2023/k8/1.6-turbo-hybrid, steerRatio: guesstimate from K5 platform
    CarSpecs(mass=1630, wheelbase=2.895, steerRatio=13.27)
  )
  KIA_NIRO_EV = HyundaiPlatformConfig(
    "KIA NIRO EV 2020",
    [
      HyundaiCarInfo("Kia Niro EV 2019", "All", video_link="https://www.youtube.com/watch?v=lT7zcG6ZpGo", car_parts=CarParts.common([CarHarness.hyundai_h])),
      HyundaiCarInfo("Kia Niro EV 2020", "All", video_link="https://www.youtube.com/watch?v=lT7zcG6ZpGo", car_parts=CarParts.common([CarHarness.hyundai_f])),
      HyundaiCarInfo("Kia Niro EV 2021", "All", video_link="https://www.youtube.com/watch?v=lT7zcG6ZpGo", car_parts=CarParts.common([CarHarness.hyundai_c])),
      HyundaiCarInfo("Kia Niro EV 2022", "All", video_link="https://www.youtube.com/watch?v=lT7zcG6ZpGo", car_parts=CarParts.common([CarHarness.hyundai_h])),
    ],
    CarSpecs(mass=3543 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=13.6, tireStiffnessFactor=0.385),  # average of all the cars
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.EV,
  )
  KIA_NIRO_EV_2ND_GEN = HyundaiCanFDPlatformConfig(
    "KIA NIRO EV 2ND GEN",
    HyundaiCarInfo("Kia Niro EV 2023", "All", car_parts=CarParts.common([CarHarness.hyundai_a])),
    KIA_NIRO_EV.specs,
    flags=HyundaiFlags.EV,
  )
  KIA_NIRO_PHEV = HyundaiPlatformConfig(
    "KIA NIRO HYBRID 2019",
    [
      HyundaiCarInfo("Kia Niro Plug-in Hybrid 2018-19", "All", min_enable_speed=10. * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_c])),
      HyundaiCarInfo("Kia Niro Plug-in Hybrid 2020", "All", car_parts=CarParts.common([CarHarness.hyundai_d])),
    ],
    KIA_NIRO_EV.specs,
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.HYBRID | HyundaiFlags.UNSUPPORTED_LONGITUDINAL | HyundaiFlags.MIN_STEER_32_MPH,
  )
  KIA_NIRO_PHEV_2022 = HyundaiPlatformConfig(
    "KIA NIRO PLUG-IN HYBRID 2022",
    [
      HyundaiCarInfo("Kia Niro Plug-in Hybrid 2021", "All", car_parts=CarParts.common([CarHarness.hyundai_d])),
      HyundaiCarInfo("Kia Niro Plug-in Hybrid 2022", "All", car_parts=CarParts.common([CarHarness.hyundai_f])),
    ],
    KIA_NIRO_EV.specs,
    flags=HyundaiFlags.HYBRID | HyundaiFlags.MANDO_RADAR,
  )
  KIA_NIRO_HEV_2021 = HyundaiPlatformConfig(
    "KIA NIRO HYBRID 2021",
    [
      HyundaiCarInfo("Kia Niro Hybrid 2021", car_parts=CarParts.common([CarHarness.hyundai_d])),
      HyundaiCarInfo("Kia Niro Hybrid 2022", car_parts=CarParts.common([CarHarness.hyundai_f])),
    ],
    KIA_NIRO_EV.specs,
    flags=HyundaiFlags.HYBRID,
  )
  KIA_NIRO_HEV_2ND_GEN = HyundaiCanFDPlatformConfig(
    "KIA NIRO HYBRID 2ND GEN",
    HyundaiCarInfo("Kia Niro Hybrid 2023", car_parts=CarParts.common([CarHarness.hyundai_a])),
    KIA_NIRO_EV.specs,
  )
  KIA_OPTIMA_G4 = HyundaiPlatformConfig(
    "KIA OPTIMA 4TH GEN",
    HyundaiCarInfo("Kia Optima 2017", "Advanced Smart Cruise Control",
                   car_parts=CarParts.common([CarHarness.hyundai_b])),  # TODO: may support 2016, 2018
    CarSpecs(mass=3558 * CV.LB_TO_KG, wheelbase=2.8, steerRatio=13.75, tireStiffnessFactor=0.5),
    flags=HyundaiFlags.LEGACY | HyundaiFlags.TCU_GEARS | HyundaiFlags.MIN_STEER_32_MPH,
  )
  KIA_OPTIMA_G4_FL = HyundaiPlatformConfig(
    "KIA OPTIMA 4TH GEN FACELIFT",
    HyundaiCarInfo("Kia Optima 2019-20", car_parts=CarParts.common([CarHarness.hyundai_g])),
    CarSpecs(mass=3558 * CV.LB_TO_KG, wheelbase=2.8, steerRatio=13.75, tireStiffnessFactor=0.5),
    flags=HyundaiFlags.UNSUPPORTED_LONGITUDINAL | HyundaiFlags.TCU_GEARS,
  )
  # TODO: may support adjacent years. may have a non-zero minimum steering speed
  KIA_OPTIMA_H = HyundaiPlatformConfig(
    "KIA OPTIMA HYBRID 2017 & SPORTS 2019",
    HyundaiCarInfo("Kia Optima Hybrid 2017", "Advanced Smart Cruise Control", car_parts=CarParts.common([CarHarness.hyundai_c])),
    CarSpecs(mass=3558 * CV.LB_TO_KG, wheelbase=2.8, steerRatio=13.75, tireStiffnessFactor=0.5),
    flags=HyundaiFlags.HYBRID | HyundaiFlags.LEGACY,
  )
  KIA_OPTIMA_H_G4_FL = HyundaiPlatformConfig(
    "KIA OPTIMA HYBRID 4TH GEN FACELIFT",
    HyundaiCarInfo("Kia Optima Hybrid 2019", car_parts=CarParts.common([CarHarness.hyundai_h])),
    CarSpecs(mass=3558 * CV.LB_TO_KG, wheelbase=2.8, steerRatio=13.75, tireStiffnessFactor=0.5),
    flags=HyundaiFlags.HYBRID | HyundaiFlags.UNSUPPORTED_LONGITUDINAL,
  )
  KIA_SELTOS = HyundaiPlatformConfig(
    "KIA SELTOS 2021",
    HyundaiCarInfo("Kia Seltos 2021", car_parts=CarParts.common([CarHarness.hyundai_a])),
    CarSpecs(mass=1337, wheelbase=2.63, steerRatio=14.56),
    flags=HyundaiFlags.CHECKSUM_CRC8,
  )
  KIA_SPORTAGE_5TH_GEN = HyundaiCanFDPlatformConfig(
    "KIA SPORTAGE 5TH GEN",
    [
      HyundaiCarInfo("Kia Sportage 2023", car_parts=CarParts.common([CarHarness.hyundai_n])),
      HyundaiCarInfo("Kia Sportage Hybrid 2023", car_parts=CarParts.common([CarHarness.hyundai_n])),
    ],
    # weight from SX and above trims, average of FWD and AWD version, steering ratio according to Kia News https://www.kiamedia.com/us/en/models/sportage/2023/specifications
    CarSpecs(mass=1725, wheelbase=2.756, steerRatio=13.6),
  )
  KIA_SORENTO = HyundaiPlatformConfig(
    "KIA SORENTO GT LINE 2018",
    [
      HyundaiCarInfo("Kia Sorento 2018", "Advanced Smart Cruise Control & LKAS", video_link="https://www.youtube.com/watch?v=Fkh3s6WHJz8",
                     car_parts=CarParts.common([CarHarness.hyundai_e])),
      HyundaiCarInfo("Kia Sorento 2019", video_link="https://www.youtube.com/watch?v=Fkh3s6WHJz8", car_parts=CarParts.common([CarHarness.hyundai_e])),
    ],
    CarSpecs(mass=1985, wheelbase=2.78, steerRatio=14.4 * 1.1),  # 10% higher at the center seems reasonable
    flags=HyundaiFlags.CHECKSUM_6B | HyundaiFlags.UNSUPPORTED_LONGITUDINAL,
  )
  KIA_SORENTO_4TH_GEN = HyundaiCanFDPlatformConfig(
    "KIA SORENTO 4TH GEN",
    HyundaiCarInfo("Kia Sorento 2021-23", car_parts=CarParts.common([CarHarness.hyundai_k])),
    CarSpecs(mass=3957 * CV.LB_TO_KG, wheelbase=2.81, steerRatio=13.5),  # average of the platforms
    flags=HyundaiFlags.RADAR_SCC,
  )
  KIA_SORENTO_HEV_4TH_GEN = HyundaiCanFDPlatformConfig(
    "KIA SORENTO HYBRID 4TH GEN",
    [
      HyundaiCarInfo("Kia Sorento Hybrid 2021-23", "All", car_parts=CarParts.common([CarHarness.hyundai_a])),
      HyundaiCarInfo("Kia Sorento Plug-in Hybrid 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_a])),
    ],
    CarSpecs(mass=4395 * CV.LB_TO_KG, wheelbase=2.81, steerRatio=13.5),  # average of the platforms
    flags=HyundaiFlags.RADAR_SCC,
  )
  KIA_STINGER = HyundaiPlatformConfig(
    "KIA STINGER GT2 2018",
    HyundaiCarInfo("Kia Stinger 2018-20", video_link="https://www.youtube.com/watch?v=MJ94qoofYw0",
                   car_parts=CarParts.common([CarHarness.hyundai_c])),
    CarSpecs(mass=1825, wheelbase=2.78, steerRatio=14.4 * 1.15)  # 15% higher at the center seems reasonable
  )
  KIA_STINGER_2022 = HyundaiPlatformConfig(
    "KIA STINGER 2022",
    HyundaiCarInfo("Kia Stinger 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_k])),
    KIA_STINGER.specs,
  )
  KIA_CEED = HyundaiPlatformConfig(
    "KIA CEED INTRO ED 2019",
    HyundaiCarInfo("Kia Ceed 2019", car_parts=CarParts.common([CarHarness.hyundai_e])),
    CarSpecs(mass=1450, wheelbase=2.65, steerRatio=13.75, tireStiffnessFactor=0.5),
    flags=HyundaiFlags.LEGACY,
  )
  KIA_EV6 = HyundaiCanFDPlatformConfig(
    "KIA EV6 2022",
    [
      HyundaiCarInfo("Kia EV6 (Southeast Asia only) 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_p])),
      HyundaiCarInfo("Kia EV6 (without HDA II) 2022-23", "Highway Driving Assist", car_parts=CarParts.common([CarHarness.hyundai_l])),
      HyundaiCarInfo("Kia EV6 (with HDA II) 2022-23", "Highway Driving Assist II", car_parts=CarParts.common([CarHarness.hyundai_p]))
    ],
    CarSpecs(mass=2055, wheelbase=2.9, steerRatio=16, tireStiffnessFactor=0.65),
    flags=HyundaiFlags.EV,
  )
  KIA_CARNIVAL_4TH_GEN = HyundaiCanFDPlatformConfig(
    "KIA CARNIVAL 4TH GEN",
    [
      HyundaiCarInfo("Kia Carnival 2022-24", car_parts=CarParts.common([CarHarness.hyundai_a])),
      HyundaiCarInfo("Kia Carnival (China only) 2023", car_parts=CarParts.common([CarHarness.hyundai_k]))
    ],
    CarSpecs(mass=2087, wheelbase=3.09, steerRatio=14.23),
    flags=HyundaiFlags.RADAR_SCC,
  )

  # Genesis
  GENESIS_GV60_EV_1ST_GEN = HyundaiCanFDPlatformConfig(
    "GENESIS GV60 ELECTRIC 1ST GEN",
    [
      HyundaiCarInfo("Genesis GV60 (Advanced Trim) 2023", "All", car_parts=CarParts.common([CarHarness.hyundai_a])),
      HyundaiCarInfo("Genesis GV60 (Performance Trim) 2023", "All", car_parts=CarParts.common([CarHarness.hyundai_k])),
    ],
    CarSpecs(mass=2205, wheelbase=2.9, steerRatio=12.6),  # steerRatio: https://www.motor1.com/reviews/586376/2023-genesis-gv60-first-drive/#:~:text=Relative%20to%20the%20related%20Ioniq,5%2FEV6%27s%2014.3%3A1.
    flags=HyundaiFlags.EV,
  )
  GENESIS_G70 = HyundaiPlatformConfig(
    "GENESIS G70 2018",
    HyundaiCarInfo("Genesis G70 2018-19", "All", car_parts=CarParts.common([CarHarness.hyundai_f])),
    CarSpecs(mass=1640, wheelbase=2.84, steerRatio=13.56),
    flags=HyundaiFlags.LEGACY,
  )
  GENESIS_G70_2020 = HyundaiPlatformConfig(
    "GENESIS G70 2020",
    HyundaiCarInfo("Genesis G70 2020", "All", car_parts=CarParts.common([CarHarness.hyundai_f])),
    CarSpecs(mass=3673 * CV.LB_TO_KG, wheelbase=2.83, steerRatio=12.9),
    flags=HyundaiFlags.MANDO_RADAR,
  )
  GENESIS_GV70_1ST_GEN = HyundaiCanFDPlatformConfig(
    "GENESIS GV70 1ST GEN",
    [
      HyundaiCarInfo("Genesis GV70 (2.5T Trim) 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_l])),
      HyundaiCarInfo("Genesis GV70 (3.5T Trim) 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_m])),
    ],
    CarSpecs(mass=1950, wheelbase=2.87, steerRatio=14.6),
    flags=HyundaiFlags.RADAR_SCC,
  )
  GENESIS_G80 = HyundaiPlatformConfig(
    "GENESIS G80 2017",
    HyundaiCarInfo("Genesis G80 2018-19", "All", car_parts=CarParts.common([CarHarness.hyundai_h])),
    CarSpecs(mass=2060, wheelbase=3.01, steerRatio=16.5),
    flags=HyundaiFlags.LEGACY,
  )
  GENESIS_G90 = HyundaiPlatformConfig(
    "GENESIS G90 2017",
    HyundaiCarInfo("Genesis G90 2017-18", "All", car_parts=CarParts.common([CarHarness.hyundai_c])),
    CarSpecs(mass=2200, wheelbase=3.15, steerRatio=12.069),
  )
  GENESIS_GV80 = HyundaiCanFDPlatformConfig(
    "GENESIS GV80 2023",
    HyundaiCarInfo("Genesis GV80 2023", "All", car_parts=CarParts.common([CarHarness.hyundai_m])),
    CarSpecs(mass=2258, wheelbase=2.95, steerRatio=14.14),
    flags=HyundaiFlags.RADAR_SCC,
  )


class Buttons:
  NONE = 0
  RES_ACCEL = 1
  SET_DECEL = 2
  GAP_DIST = 3
  CANCEL = 4  # on newer models, this is a pause/resume button


def get_platform_codes(fw_versions: list[bytes]) -> set[tuple[bytes, bytes | None]]:
  # Returns unique, platform-specific identification codes for a set of versions
  codes = set()  # (code-Optional[part], date)
  for fw in fw_versions:
    code_match = PLATFORM_CODE_FW_PATTERN.search(fw)
    part_match = PART_NUMBER_FW_PATTERN.search(fw)
    date_match = DATE_FW_PATTERN.search(fw)
    if code_match is not None:
      code: bytes = code_match.group()
      part = part_match.group() if part_match else None
      date = date_match.group() if date_match else None
      if part is not None:
        # part number starts with generic ECU part type, add what is specific to platform
        code += b"-" + part[-5:]

      codes.add((code, date))
  return codes


def match_fw_to_car_fuzzy(live_fw_versions, offline_fw_versions) -> set[str]:
  # Non-electric CAN FD platforms often do not have platform code specifiers needed
  # to distinguish between hybrid and ICE. All EVs so far are either exclusively
  # electric or specify electric in the platform code.
  fuzzy_platform_blacklist = {str(c) for c in (CANFD_CAR - EV_CAR - CANFD_FUZZY_WHITELIST)}
  candidates: set[str] = set()

  for candidate, fws in offline_fw_versions.items():
    # Keep track of ECUs which pass all checks (platform codes, within date range)
    valid_found_ecus = set()
    valid_expected_ecus = {ecu[1:] for ecu in fws if ecu[0] in PLATFORM_CODE_ECUS}
    for ecu, expected_versions in fws.items():
      addr = ecu[1:]
      # Only check ECUs expected to have platform codes
      if ecu[0] not in PLATFORM_CODE_ECUS:
        continue

      # Expected platform codes & dates
      codes = get_platform_codes(expected_versions)
      expected_platform_codes = {code for code, _ in codes}
      expected_dates = {date for _, date in codes if date is not None}

      # Found platform codes & dates
      codes = get_platform_codes(live_fw_versions.get(addr, set()))
      found_platform_codes = {code for code, _ in codes}
      found_dates = {date for _, date in codes if date is not None}

      # Check platform code + part number matches for any found versions
      if not any(found_platform_code in expected_platform_codes for found_platform_code in found_platform_codes):
        break

      if ecu[0] in DATE_FW_ECUS:
        # If ECU can have a FW date, require it to exist
        # (this excludes candidates in the database without dates)
        if not len(expected_dates) or not len(found_dates):
          break

        # Check any date within range in the database, format is %y%m%d
        if not any(min(expected_dates) <= found_date <= max(expected_dates) for found_date in found_dates):
          break

      valid_found_ecus.add(addr)

    # If all live ECUs pass all checks for candidate, add it as a match
    if valid_expected_ecus.issubset(valid_found_ecus):
      candidates.add(candidate)

  return candidates - fuzzy_platform_blacklist


HYUNDAI_VERSION_REQUEST_LONG = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(0xf100)  # Long description

HYUNDAI_VERSION_REQUEST_ALT = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(0xf110)  # Alt long description

HYUNDAI_VERSION_REQUEST_MULTI = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_SPARE_PART_NUMBER) + \
  p16(uds.DATA_IDENTIFIER_TYPE.APPLICATION_SOFTWARE_IDENTIFICATION) + \
  p16(0xf100)

HYUNDAI_ECU_MANUFACTURING_DATE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.ECU_MANUFACTURING_DATE)

HYUNDAI_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40])

# Regex patterns for parsing platform code, FW date, and part number from FW versions
PLATFORM_CODE_FW_PATTERN = re.compile(b'((?<=' + HYUNDAI_VERSION_REQUEST_LONG[1:] +
                                      b')[A-Z]{2}[A-Za-z0-9]{0,2})')
DATE_FW_PATTERN = re.compile(b'(?<=[ -])([0-9]{6}$)')
PART_NUMBER_FW_PATTERN = re.compile(b'(?<=[0-9][.,][0-9]{2} )([0-9]{5}[-/]?[A-Z][A-Z0-9]{3}[0-9])')

# We've seen both ICE and hybrid for these platforms, and they have hybrid descriptors (e.g. MQ4 vs MQ4H)
CANFD_FUZZY_WHITELIST = {CAR.KIA_SORENTO_4TH_GEN, CAR.KIA_SORENTO_HEV_4TH_GEN}

# List of ECUs expected to have platform codes, camera and radar should exist on all cars
# TODO: use abs, it has the platform code and part number on many platforms
PLATFORM_CODE_ECUS = [Ecu.fwdRadar, Ecu.fwdCamera, Ecu.eps]
# So far we've only seen dates in fwdCamera
# TODO: there are date codes in the ABS firmware versions in hex
DATE_FW_ECUS = [Ecu.fwdCamera]

ALL_HYUNDAI_ECUS = [Ecu.eps, Ecu.abs, Ecu.fwdRadar, Ecu.fwdCamera, Ecu.engine, Ecu.parkingAdas, Ecu.transmission, Ecu.adas, Ecu.hvac, Ecu.cornerRadar]

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    # TODO: minimize shared whitelists for CAN and cornerRadar for CAN-FD
    # CAN queries (OBD-II port)
    Request(
      [HYUNDAI_VERSION_REQUEST_LONG],
      [HYUNDAI_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.transmission, Ecu.eps, Ecu.abs, Ecu.fwdRadar, Ecu.fwdCamera],
    ),
    Request(
      [HYUNDAI_VERSION_REQUEST_MULTI],
      [HYUNDAI_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine, Ecu.transmission, Ecu.eps, Ecu.abs, Ecu.fwdRadar],
    ),

    # CAN-FD queries (from camera)
    # TODO: combine shared whitelists with CAN requests
    Request(
      [HYUNDAI_VERSION_REQUEST_LONG],
      [HYUNDAI_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.fwdCamera, Ecu.fwdRadar, Ecu.cornerRadar, Ecu.hvac],
      bus=0,
      auxiliary=True,
    ),
    Request(
      [HYUNDAI_VERSION_REQUEST_LONG],
      [HYUNDAI_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.fwdCamera, Ecu.adas, Ecu.cornerRadar, Ecu.hvac],
      bus=1,
      auxiliary=True,
      obd_multiplexing=False,
    ),

    # CAN & CAN FD query to understand the three digit date code
    # HDA2 cars usually use 6 digit date codes, so skip bus 1
    Request(
      [HYUNDAI_ECU_MANUFACTURING_DATE],
      [HYUNDAI_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.fwdCamera],
      bus=0,
      auxiliary=True,
      logging=True,
    ),

    # CAN & CAN FD logging queries (from camera)
    Request(
      [HYUNDAI_VERSION_REQUEST_LONG],
      [HYUNDAI_VERSION_RESPONSE],
      whitelist_ecus=ALL_HYUNDAI_ECUS,
      bus=0,
      auxiliary=True,
      logging=True,
    ),
    Request(
      [HYUNDAI_VERSION_REQUEST_MULTI],
      [HYUNDAI_VERSION_RESPONSE],
      whitelist_ecus=ALL_HYUNDAI_ECUS,
      bus=0,
      auxiliary=True,
      logging=True,
    ),
    Request(
      [HYUNDAI_VERSION_REQUEST_LONG],
      [HYUNDAI_VERSION_RESPONSE],
      whitelist_ecus=ALL_HYUNDAI_ECUS,
      bus=1,
      auxiliary=True,
      obd_multiplexing=False,
      logging=True,
    ),

    # CAN-FD alt request logging queries
    Request(
      [HYUNDAI_VERSION_REQUEST_ALT],
      [HYUNDAI_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.parkingAdas, Ecu.hvac],
      bus=0,
      auxiliary=True,
      logging=True,
    ),
    Request(
      [HYUNDAI_VERSION_REQUEST_ALT],
      [HYUNDAI_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.parkingAdas, Ecu.hvac],
      bus=1,
      auxiliary=True,
      logging=True,
      obd_multiplexing=False,
    ),
  ],
  extra_ecus=[
    (Ecu.adas, 0x730, None),         # ADAS Driving ECU on HDA2 platforms
    (Ecu.parkingAdas, 0x7b1, None),  # ADAS Parking ECU (may exist on all platforms)
    (Ecu.hvac, 0x7b3, None),         # HVAC Control Assembly
    (Ecu.cornerRadar, 0x7b7, None),
  ],
  # Custom fuzzy fingerprinting function using platform codes, part numbers + FW dates:
  match_fw_to_car_fuzzy=match_fw_to_car_fuzzy,
)

CHECKSUM = {
  "crc8": [CAR.SANTA_FE, CAR.SONATA, CAR.PALISADE, CAR.KIA_SELTOS, CAR.ELANTRA_2021, CAR.ELANTRA_HEV_2021,
           CAR.SONATA_HYBRID, CAR.SANTA_FE_2022, CAR.KIA_K5_2021, CAR.SANTA_FE_HEV_2022, CAR.SANTA_FE_PHEV_2022,
           CAR.KIA_K5_HEV_2020, CAR.CUSTIN_1ST_GEN],
  "6B": [CAR.KIA_SORENTO, CAR.HYUNDAI_GENESIS],
}

CAN_GEARS = {
  # which message has the gear. hybrid and EV use ELECT_GEAR
  "use_cluster_gears": {CAR.ELANTRA, CAR.ELANTRA_GT_I30, CAR.KONA,
                        CAR.GRANDEUR_IG, CAR.GRANDEUR_IG_FL, CAR.K7},
  "use_tcu_gears": {CAR.KIA_OPTIMA_G4, CAR.KIA_OPTIMA_G4_FL, CAR.SONATA_LF, CAR.VELOSTER, CAR.TUCSON},


 }

CANFD_CAR = {CAR.KIA_EV6, CAR.IONIQ_5, CAR.IONIQ_6, CAR.TUCSON_4TH_GEN, CAR.SANTA_CRUZ_1ST_GEN, CAR.KIA_SPORTAGE_5TH_GEN, CAR.GENESIS_GV70_1ST_GEN,
             CAR.GENESIS_GV60_EV_1ST_GEN, CAR.KIA_SORENTO_4TH_GEN, CAR.KIA_NIRO_HEV_2ND_GEN, CAR.KIA_NIRO_EV_2ND_GEN,
             CAR.GENESIS_GV80, CAR.KIA_CARNIVAL_4TH_GEN, CAR.KIA_SORENTO_HEV_4TH_GEN, CAR.KONA_EV_2ND_GEN, CAR.KIA_K8_HEV_1ST_GEN,
             CAR.STARIA_4TH_GEN, CAR.GENESIS_EGV70}

# The radar does SCC on these cars when HDA I, rather than the camera
CANFD_RADAR_SCC_CAR = {CAR.GENESIS_GV70_1ST_GEN, CAR.KIA_SORENTO_4TH_GEN, CAR.GENESIS_GV80, CAR.KIA_CARNIVAL_4TH_GEN, CAR.KIA_SORENTO_HEV_4TH_GEN, CAR.GENESIS_EGV70}

# These CAN FD cars do not accept communication control to disable the ADAS ECU,
# responds with 0x7F2822 - 'conditions not correct'
CANFD_UNSUPPORTED_LONGITUDINAL_CAR = {CAR.IONIQ_6, CAR.KONA_EV_2ND_GEN}

# The camera does SCC on these cars, rather than the radar
CAMERA_SCC_CAR = {CAR.KONA_EV_2022, }

# these cars use a different gas signal
HYBRID_CAR = {CAR.IONIQ_PHEV, CAR.ELANTRA_HEV_2021, CAR.KIA_NIRO_PHEV, CAR.KIA_NIRO_HEV_2021, CAR.SONATA_HYBRID, CAR.KONA_HEV, CAR.IONIQ,
              CAR.IONIQ_HEV_2022, CAR.SANTA_FE_HEV_2022, CAR.SANTA_FE_PHEV_2022, CAR.IONIQ_PHEV_2019, CAR.KIA_K5_HEV_2020,
              CAR.KIA_OPTIMA_H, CAR.KIA_OPTIMA_H_G4_FL, CAR.AZERA_HEV_6TH_GEN, CAR.KIA_NIRO_PHEV_2022,
              CAR.GRANDEUR_IG_HEV, CAR.GRANDEUR_IG_FL_HEV, CAR.K7_HEV,}

EV_CAR = {CAR.IONIQ_EV_2020, CAR.IONIQ_EV_LTD, CAR.KONA_EV, CAR.KIA_NIRO_EV, CAR.KIA_NIRO_EV_2ND_GEN, CAR.KONA_EV_2022,
          CAR.KIA_EV6, CAR.IONIQ_5, CAR.IONIQ_6, CAR.GENESIS_GV60_EV_1ST_GEN, CAR.KONA_EV_2ND_GEN, 
          CAR.NEXO, CAR.KIA_SOUL_EV_SK3, CAR.GENESIS_EGV70}

# these cars require a special panda safety mode due to missing counters and checksums in the messages
LEGACY_SAFETY_MODE_CAR = {CAR.HYUNDAI_GENESIS, CAR.IONIQ_EV_LTD, CAR.KIA_OPTIMA_G4,
                          CAR.VELOSTER, CAR.GENESIS_G70, CAR.GENESIS_G80, CAR.KIA_CEED, CAR.ELANTRA, CAR.IONIQ_HEV_2022,
                          CAR.KIA_OPTIMA_H, CAR.ELANTRA_GT_I30,
                          CAR.GRANDEUR_IG, CAR.GRANDEUR_IG_HEV, CAR.GRANDEUR_IG_FL, CAR.GRANDEUR_IG_FL_HEV, CAR.GENESIS_EQ900, 
                          CAR.GENESIS_EQ900_L, CAR.GENESIS_G90_2019,CAR.K7, CAR.K7_HEV}
# these cars have not been verified to work with longitudinal yet - radar disable, sending correct messages, etc.
UNSUPPORTED_LONGITUDINAL_CAR = LEGACY_SAFETY_MODE_CAR | {CAR.KIA_NIRO_PHEV, CAR.KIA_SORENTO, CAR.SONATA_LF, CAR.KIA_OPTIMA_G4_FL,
                                                         CAR.KIA_OPTIMA_H_G4_FL}

# If 0x500 is present on bus 1 it probably has a Mando radar outputting radar points.
# If no points are outputted by default it might be possible to turn it on using  selfdrive/debug/hyundai_enable_radar_points.py
DBC = {
  CAR.AZERA_6TH_GEN: dbc_dict('hyundai_kia_generic', None),
  CAR.AZERA_HEV_6TH_GEN: dbc_dict('hyundai_kia_generic', None),
  CAR.ELANTRA: dbc_dict('hyundai_kia_generic', None),
  CAR.ELANTRA_GT_I30: dbc_dict('hyundai_kia_generic', None),
  CAR.ELANTRA_2021: dbc_dict('hyundai_kia_generic', None),
  CAR.ELANTRA_HEV_2021: dbc_dict('hyundai_kia_generic', None),
  CAR.GENESIS_G70: dbc_dict('hyundai_kia_generic', None),
  CAR.GENESIS_G70_2020: dbc_dict('hyundai_kia_generic', 'hyundai_kia_mando_front_radar_generated'),
  CAR.GENESIS_G80: dbc_dict('hyundai_kia_generic', None),
  CAR.GENESIS_G90: dbc_dict('hyundai_kia_generic', None),
  CAR.HYUNDAI_GENESIS: dbc_dict('hyundai_kia_generic', None),
  CAR.IONIQ_PHEV_2019: dbc_dict('hyundai_kia_generic', None),
  CAR.IONIQ_PHEV: dbc_dict('hyundai_kia_generic', None),
  CAR.IONIQ_EV_2020: dbc_dict('hyundai_kia_generic', None),
  CAR.IONIQ_EV_LTD: dbc_dict('hyundai_kia_generic', 'hyundai_kia_mando_front_radar_generated'),
  CAR.IONIQ: dbc_dict('hyundai_kia_generic', None),
  CAR.IONIQ_HEV_2022: dbc_dict('hyundai_kia_generic', None),
  CAR.KIA_FORTE: dbc_dict('hyundai_kia_generic', None),
  CAR.KIA_K5_2021: dbc_dict('hyundai_kia_generic', None),
  CAR.KIA_K5_HEV_2020: dbc_dict('hyundai_kia_generic', 'hyundai_kia_mando_front_radar_generated'),
  CAR.KIA_NIRO_EV: dbc_dict('hyundai_kia_generic', 'hyundai_kia_mando_front_radar_generated'),
  CAR.KIA_NIRO_PHEV: dbc_dict('hyundai_kia_generic', 'hyundai_kia_mando_front_radar_generated'),
  CAR.KIA_NIRO_HEV_2021: dbc_dict('hyundai_kia_generic', None),
  CAR.KIA_OPTIMA_G4: dbc_dict('hyundai_kia_generic', None),
  CAR.KIA_OPTIMA_G4_FL: dbc_dict('hyundai_kia_generic', None),
  CAR.KIA_OPTIMA_H: dbc_dict('hyundai_kia_generic', None),
  CAR.KIA_OPTIMA_H_G4_FL: dbc_dict('hyundai_kia_generic', None),
  CAR.KIA_SELTOS: dbc_dict('hyundai_kia_generic', None),
  CAR.KIA_SORENTO: dbc_dict('hyundai_kia_generic', None), # Has 0x5XX messages, but different format
  CAR.KIA_STINGER: dbc_dict('hyundai_kia_generic', None),
  CAR.KIA_STINGER_2022: dbc_dict('hyundai_kia_generic', None),
  CAR.KONA: dbc_dict('hyundai_kia_generic', None),
  CAR.KONA_EV: dbc_dict('hyundai_kia_generic', None),
  CAR.KONA_EV_2022: dbc_dict('hyundai_kia_generic', None),
  CAR.KONA_HEV: dbc_dict('hyundai_kia_generic', None),
  CAR.SANTA_FE: dbc_dict('hyundai_kia_generic', 'hyundai_kia_mando_front_radar_generated'),
  CAR.SANTA_FE_2022: dbc_dict('hyundai_kia_generic', None),
  CAR.SANTA_FE_HEV_2022: dbc_dict('hyundai_kia_generic', None),
  CAR.SANTA_FE_PHEV_2022: dbc_dict('hyundai_kia_generic', None),
  CAR.SONATA: dbc_dict('hyundai_kia_generic', 'hyundai_kia_mando_front_radar_generated'),
  CAR.SONATA_LF: dbc_dict('hyundai_kia_generic', None), # Has 0x5XX messages, but different format
  CAR.TUCSON: dbc_dict('hyundai_kia_generic', None),
  CAR.PALISADE: dbc_dict('hyundai_kia_generic', 'hyundai_kia_mando_front_radar_generated'),
  CAR.VELOSTER: dbc_dict('hyundai_kia_generic', None),
  CAR.GRANDEUR_IG: dbc_dict('hyundai_kia_generic', None),
  CAR.GRANDEUR_IG_HEV: dbc_dict('hyundai_kia_generic', None),
  CAR.GRANDEUR_IG_FL: dbc_dict('hyundai_kia_generic', None),
  CAR.GRANDEUR_IG_FL_HEV: dbc_dict('hyundai_kia_generic', None),
  CAR.GENESIS_EQ900: dbc_dict('hyundai_kia_generic', None),
  CAR.GENESIS_EQ900_L: dbc_dict('hyundai_kia_generic', None),
  CAR.GENESIS_G90_2019: dbc_dict('hyundai_kia_generic', None),
  CAR.KIA_CEED: dbc_dict('hyundai_kia_generic', None),
  CAR.KIA_EV6: dbc_dict('hyundai_canfd', None),
  CAR.SONATA_HYBRID: dbc_dict('hyundai_kia_generic', 'hyundai_kia_mando_front_radar_generated'),
  CAR.TUCSON_4TH_GEN: dbc_dict('hyundai_canfd', None),
  CAR.IONIQ_5: dbc_dict('hyundai_canfd', None),
  CAR.IONIQ_6: dbc_dict('hyundai_canfd', None),
  CAR.SANTA_CRUZ_1ST_GEN: dbc_dict('hyundai_canfd', None),
  CAR.KIA_SOUL_EV_SK3: dbc_dict('hyundai_kia_generic', 'hyundai_kia_mando_front_radar_generated'),
  CAR.KIA_SPORTAGE_5TH_GEN: dbc_dict('hyundai_canfd', None),
  CAR.NEXO: dbc_dict('hyundai_kia_generic', 'hyundai_kia_mando_front_radar_generated'),
  CAR.K7: dbc_dict('hyundai_kia_generic', None),
  CAR.K7_HEV: dbc_dict('hyundai_kia_generic', None),
  CAR.GENESIS_GV70_1ST_GEN: dbc_dict('hyundai_canfd', None),
  CAR.GENESIS_GV60_EV_1ST_GEN: dbc_dict('hyundai_canfd', None),
  CAR.KIA_SORENTO_4TH_GEN: dbc_dict('hyundai_canfd', None),
  CAR.KIA_NIRO_HEV_2ND_GEN: dbc_dict('hyundai_canfd', None),
  CAR.KIA_NIRO_EV_2ND_GEN: dbc_dict('hyundai_canfd', None),
  CAR.GENESIS_GV80: dbc_dict('hyundai_canfd', None),
  CAR.KIA_CARNIVAL_4TH_GEN: dbc_dict('hyundai_canfd', None),
  CAR.KIA_SORENTO_HEV_4TH_GEN: dbc_dict('hyundai_canfd', None),
  CAR.KONA_EV_2ND_GEN: dbc_dict('hyundai_canfd', None),
  CAR.KIA_K8_HEV_1ST_GEN: dbc_dict('hyundai_canfd', None),
  CAR.CUSTIN_1ST_GEN: dbc_dict('hyundai_kia_generic', None),
  CAR.KIA_NIRO_PHEV_2022: dbc_dict('hyundai_kia_generic', 'hyundai_kia_mando_front_radar_generated'),
  CAR.STARIA_4TH_GEN: dbc_dict('hyundai_canfd', None),
  CAR.GENESIS_EGV70: dbc_dict('hyundai_canfd', None),
}



def main():
  for member, value in vars(CAR).items():
    if not member.startswith("_"):
      print(value)


if __name__ == "__main__":
  main()
