import fcntl
import json
import math
import os
import socket
import struct
import subprocess
import threading
import time
import numpy as np
from datetime import datetime

from ftplib import FTP
from cereal import log
import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.params import Params
from openpilot.common.filter_simple import MyMovingAverage
from openpilot.system.hardware import PC, TICI
from openpilot.selfdrive.navd.helpers import Coordinate
from opendbc.car.common.conversions import Conversions as CV
from openpilot.common.gps import get_gps_location_service

nav_type_mapping = {
  12: ("turn", "left", 1),
  16: ("turn", "sharp left", 1),
  1000: ("turn", "slight left", 1),
  1001: ("turn", "slight right", 2),
  1002: ("fork", "slight left", 3),
  1003: ("fork", "slight right", 4),
  1006: ("off ramp", "left", 3),
  1007: ("off ramp", "right", 4),
  13: ("turn", "right", 2),
  19: ("turn", "sharp right", 2),
  102: ("off ramp", "slight left", 3),
  105: ("off ramp", "slight left", 3),
  112: ("off ramp", "slight left", 3),
  115: ("off ramp", "slight left", 3),
  101: ("off ramp", "slight right", 4),
  104: ("off ramp", "slight right", 4),
  111: ("off ramp", "slight right", 4),
  114: ("off ramp", "slight right", 4),
  7: ("fork", "left", 3),
  44: ("fork", "left", 3),
  17: ("fork", "left", 3),
  75: ("fork", "left", 3),
  76: ("fork", "left", 3),
  118: ("fork", "left", 3),
  6: ("fork", "right", 4),
  43: ("fork", "right", 4),
  73: ("fork", "right", 4),
  74: ("fork", "right", 4),
  123: ("fork", "right", 4),
  124: ("fork", "right", 4),
  117: ("fork", "right", 4),
  131: ("rotary", "slight right", 5),
  132: ("rotary", "slight right", 5),
  140: ("rotary", "slight left", 5),
  141: ("rotary", "slight left", 5),
  133: ("rotary", "right", 5),
  134: ("rotary", "sharp right", 5),
  135: ("rotary", "sharp right", 5),
  136: ("rotary", "sharp left", 5),
  137: ("rotary", "sharp left", 5),
  138: ("rotary", "sharp left", 5),
  139: ("rotary", "left", 5),
  142: ("rotary", "straight", 5),
  14: ("turn", "uturn", 5),
  201: ("arrive", "straight", 5),
  51: ("notification", "straight", None),
  52: ("notification", "straight", None),
  53: ("notification", "straight", None),
  54: ("notification", "straight", None),
  55: ("notification", "straight", None),
  153: ("", "", 6),  #TG
  154: ("", "", 6),  #TG
  249: ("", "", 6)   #TG
}

import collections
class CarrotServ:
  def __init__(self):
    self.params = Params()
    self.params_memory = Params("/dev/shm/params")

    self.nRoadLimitSpeed = 30
    self.nRoadLimitSpeed_last = 30
    self.nRoadLimitSpeed_counter = 0

    self.active_carrot = 0     ## 1: CarrotMan Active, 2: sdi active , 3: speed decel active, 4: section active, 5: bump active, 6: speed limit active
    self.active_count = 0
    self.active_sdi_count = 0
    self.active_sdi_count_max = 200 # 20 sec

    self.active_kisa_count = 0

    self.nSdiType = -1
    self.nSdiSpeedLimit = 0
    self.nSdiSection = 0
    self.nSdiDist = 0
    self.nSdiBlockType = -1
    self.nSdiBlockSpeed = 0
    self.nSdiBlockDist = 0

    self.nTBTDist = 0
    self.nTBTTurnType = -1
    self.szTBTMainText = ""
    self.szNearDirName = ""
    self.szFarDirName = ""
    self.nTBTNextRoadWidth = 0

    self.nTBTDistNext = 0
    self.nTBTTurnTypeNext = -1
    self.szTBTMainTextNext = ""

    self.nGoPosDist = 0
    self.nGoPosTime = 0
    self.szPosRoadName = ""
    self.nSdiPlusType = -1
    self.nSdiPlusSpeedLimit = 0
    self.nSdiPlusDist = 0
    self.nSdiPlusBlockType = -1
    self.nSdiPlusBlockSpeed = 0
    self.nSdiPlusBlockDist = 0

    self.goalPosX = 0.0
    self.goalPosY = 0.0
    self.szGoalName = ""
    self.vpPosPointLatNavi = 0.0
    self.vpPosPointLonNavi = 0.0
    self.vpPosPointLat = 0.0
    self.vpPosPointLon = 0.0
    self.roadcate = 8

    self.nPosSpeed = 0.0
    self.nPosAngle = 0.0
    self.nPosAnglePhone = 0.0

    self.diff_angle_count = 0
    self.last_calculate_gps_time = 0
    self.last_update_gps_time = 0
    self.last_update_gps_time_phone = 0
    self.last_update_gps_time_navi = 0
    self.bearing_offset = 0.0
    self.bearing_measured = 0.0
    self.bearing = 0.0
    self.gps_valid = False

    self.phone_gps_accuracy = 0.0
    self.gps_accuracy_device = 0.0
    self.phone_latitude = 0.0
    self.phone_longitude = 0.0
    self.phone_gps_frame = 0

    self.totalDistance = 0
    self.xSpdLimit = 0
    self.xSpdDist = 0
    self.xSpdType = -1

    self.xTurnInfo = -1
    self.xDistToTurn = 0
    self.xTurnInfoNext = -1
    self.xDistToTurnNext = 0

    self.navType, self.navModifier = "invalid", ""
    self.navTypeNext, self.navModifierNext = "invalid", ""

    self.carrotIndex = 0
    self.carrotCmdIndex = 0
    self.carrotCmd = ""
    self.carrotArg = ""
    self.carrotCmdIndex_last = 0

    self.traffic_light_q = collections.deque(maxlen=int(2.0/0.1))  # 2 secnods
    self.traffic_light_count = -1
    self.traffic_state = 0

    self.left_spd_sec = 0
    self.left_tbt_sec = 0
    self.left_sec = 100
    self.max_left_sec = 100
    self.carrot_left_sec = 100
    self.sdi_inform = False


    self.atc_paused = False
    self.atc_activate_count = 0
    self.gas_override_speed = 0
    self.gas_pressed_state = False
    self.source_last = "none"

    self.debugText = ""

    # 默认语言，稍后在 update_params 中从 Params 读取覆盖，
    # 规则：main_ko -> 韩语；main_zh-CHS -> 中文；其他 -> 英文
    self.lang = "en"

    self.update_params()

  def update_params(self):
    self.autoNaviSpeedBumpSpeed = float(self.params.get_int("AutoNaviSpeedBumpSpeed"))
    self.autoNaviSpeedBumpTime = float(self.params.get_int("AutoNaviSpeedBumpTime"))
    self.autoNaviSpeedCtrlEnd = float(self.params.get_int("AutoNaviSpeedCtrlEnd"))
    self.autoNaviSpeedCtrlMode = self.params.get_int("AutoNaviSpeedCtrlMode")
    self.autoNaviSpeedSafetyFactor = float(self.params.get_int("AutoNaviSpeedSafetyFactor")) * 0.01
    self.autoNaviSpeedDecelRate = float(self.params.get_int("AutoNaviSpeedDecelRate")) * 0.01
    self.autoNaviCountDownMode = self.params.get_int("AutoNaviCountDownMode")
    self.turnSpeedControlMode= self.params.get_int("TurnSpeedControlMode")
    self.mapTurnSpeedFactor= self.params.get_float("MapTurnSpeedFactor") * 0.01

    self.autoTurnControlSpeedTurn = self.params.get_int("AutoTurnControlSpeedTurn")
    self.autoTurnMapChange = self.params.get_int("AutoTurnMapChange")
    self.autoTurnControl = self.params.get_int("AutoTurnControl")
    self.autoTurnControlTurnEnd = self.params.get_int("AutoTurnControlTurnEnd")
    #self.autoNaviSpeedDecelRate = float(self.params.get_int("AutoNaviSpeedDecelRate")) * 0.01
    self.autoCurveSpeedLowerLimit = int(self.params.get("AutoCurveSpeedLowerLimit"))
    self.is_metric = self.params.get_bool("IsMetric")
    self.autoRoadSpeedLimitOffset = self.params.get_int("AutoRoadSpeedLimitOffset")

    # 读取语言设置：优先使用 LanguageSetting，与 UI 保持一致；回退读取可能存在的 "lang"
    try:
      lang_val = self.params.get('LanguageSetting', encoding='utf8') or self.params.get('lang', encoding='utf8')
    except Exception:
      lang_val = None
    if isinstance(lang_val, bytes):
      try:
        lang_val = lang_val.decode('utf8')
      except Exception:
        lang_val = None
    if lang_val == "main_ko":
      self.lang = "ko"
    elif lang_val == "main_zh-CHS":
      self.lang = "zh"
    else:
      self.lang = "en"


  def _update_cmd(self):
    if self.carrotCmdIndex != self.carrotCmdIndex_last:
      self.carrotCmdIndex_last = self.carrotCmdIndex
      command_handlers = {
        "DETECT": self._handle_detect_command,
      }

      handler = command_handlers.get(self.carrotCmd)
      if handler:
        handler(self.carrotArg)

    self.traffic_light_q.append((-1, -1, "none", 0.0))
    self.traffic_light_count -= 1
    if self.traffic_light_count < 0:
      self.traffic_light_count = -1
      self.traffic_state = 0

  def _handle_detect_command(self, xArg):
    elements = [e.strip() for e in xArg.split(',')]
    if len(elements) >= 4:
      try:
        state = elements[0]
        value1 = float(elements[1])
        value2 = float(elements[2])
        value3 = float(elements[3])
        self.traffic_light(value1, value2, state, value3)
        self.traffic_light_count = int(0.5 / 0.1)
      except ValueError:
        pass

  def traffic_light(self, x, y, color, cnf):
    traffic_red = 0
    traffic_green = 0
    traffic_left = 0
    traffic_red_trig = 0
    traffic_green_trig = 0
    traffic_left_trig = 0
    for pdata in self.traffic_light_q:
      px, py, pcolor,pcnf = pdata
      if abs(x - px) < 0.2 and abs(y - py) < 0.2:
        if pcolor in ["Green Light", "Left turn"]:
          if color in ["Red Light", "Yellow Light"]:
            traffic_red_trig += cnf
            traffic_red += cnf
          elif color in ["Green Light", "Left turn"]:
            traffic_green += cnf
        elif pcolor in ["Red Light", "Yellow Light"]:
          if color in ["Green Light"]: #, "Left turn"]:
            traffic_green_trig += cnf
            traffic_green += cnf
          elif color in ["Left turn"]:
            traffic_left_trig += cnf
            traffic_left += cnf
          elif color in ["Red Light", "Yellow Light"]:
            traffic_red += cnf

    #print(self.traffic_light_q)
    if traffic_red_trig > 0:
      self.traffic_state = 1
      #self._add_log("Red light triggered")
      #print("Red light triggered")
    elif traffic_green_trig > 0 and traffic_green > traffic_red:  #주변에 red light의 cnf보다 더 크면 출발... 감지오류로 출발하는경우가 생김.
      self.traffic_state = 2
      #self._add_log("Green light triggered")
      #print("Green light triggered")
    elif traffic_left_trig > 0:
      self.traffic_state = 3
    elif traffic_red > 0:
      self.traffic_state = 1
      #self._add_log("Red light continued")
      #print("Red light continued")
    elif traffic_green > 0:
      self.traffic_state = 2
      #self._add_log("Green light continued")
      #print("Green light continued")
    else:
      self.traffic_state = 0
      #print("TrafficLight none")

    self.traffic_light_q.append((x,y,color,cnf))


  def calculate_current_speed(self, left_dist, safe_speed_kph, safe_time, safe_decel_rate):
    safe_speed = safe_speed_kph / 3.6
    safe_dist = safe_speed * safe_time
    decel_dist = left_dist - safe_dist

    if decel_dist <= 0:
      return safe_speed_kph

    # v_i^2 = v_f^2 + 2ad
    temp = safe_speed**2 + 2 * safe_decel_rate * decel_dist  # 공식에서 감속 적용

    if temp < 0:
      speed_mps = safe_speed
    else:
      speed_mps = math.sqrt(temp)
    return max(safe_speed_kph, min(250, speed_mps * 3.6))

  def _update_tbt(self):
    #xTurnInfo : 1: left turn, 2: right turn, 3: left lane change, 4: right lane change, 5: rotary, 6: tg, 7: arrive or uturn
    turn_type_mapping = {
      12: ("turn", "left", 1),
      16: ("turn", "sharp left", 1),
      13: ("turn", "right", 2),
      19: ("turn", "sharp right", 2),
      102: ("off ramp", "slight left", 3),
      105: ("off ramp", "slight left", 3),
      112: ("off ramp", "slight left", 3),
      115: ("off ramp", "slight left", 3),
      101: ("off ramp", "slight right", 4),
      104: ("off ramp", "slight right", 4),
      111: ("off ramp", "slight right", 4),
      114: ("off ramp", "slight right", 4),
      7: ("fork", "left", 3),
      44: ("fork", "left", 3),
      17: ("fork", "left", 3),
      75: ("fork", "left", 3),
      76: ("fork", "left", 3),
      118: ("fork", "left", 3),
      6: ("fork", "right", 4),
      43: ("fork", "right", 4),
      73: ("fork", "right", 4),
      74: ("fork", "right", 4),
      123: ("fork", "right", 4),
      124: ("fork", "right", 4),
      117: ("fork", "right", 4),
      131: ("rotary", "slight right", 5),
      132: ("rotary", "slight right", 5),
      140: ("rotary", "slight left", 5),
      141: ("rotary", "slight left", 5),
      133: ("rotary", "right", 5),
      134: ("rotary", "sharp right", 5),
      135: ("rotary", "sharp right", 5),
      136: ("rotary", "sharp left", 5),
      137: ("rotary", "sharp left", 5),
      138: ("rotary", "sharp left", 5),
      139: ("rotary", "left", 5),
      142: ("rotary", "straight", 5),
      14: ("turn", "uturn", 7),
      201: ("arrive", "straight", 8),
      51: ("notification", "straight", 0),
      52: ("notification", "straight", 0),
      53: ("notification", "straight", 0),
      54: ("notification", "straight", 0),
      55: ("notification", "straight", 0),
      153: ("", "", 6),  #TG
      154: ("", "", 6),  #TG
      249: ("", "", 6)   #TG
    }

    if self.nTBTTurnType in turn_type_mapping:
      self.navType, self.navModifier, self.xTurnInfo = turn_type_mapping[self.nTBTTurnType]
    else:
      self.navType, self.navModifier, self.xTurnInfo = "invalid", "", -1

    if self.nTBTTurnTypeNext in turn_type_mapping:
      self.navTypeNext, self.navModifierNext, self.xTurnInfoNext = turn_type_mapping[self.nTBTTurnTypeNext]
    else:
      self.navTypeNext, self.navModifierNext, self.xTurnInfoNext = "invalid", "", -1

    if self.nTBTDist > 0 and self.xTurnInfo > 0:
      self.xDistToTurn = self.nTBTDist
    if self.nTBTDistNext > 0 and self.xTurnInfoNext > 0:
      self.xDistToTurnNext = self.nTBTDistNext + self.nTBTDist

  def _get_sdi_descr(self, nSdiType):
    # 多语言映射：ko（韩语，原始），zh（简体中文），en（英文）。
    sdi_ko = {
        0: "신호과속",
        1: "과속 (고정식)",
        2: "구간단속 시작",
        3: "구간단속 끝",
        4: "구간단속중",
        5: "꼬리물기단속카메라",
        6: "신호 단속",
        7: "과속 (이동식)",
        8: "고정식 과속위험 구간(박스형)",
        9: "버스전용차로구간",
        10: "가변 차로 단속",
        11: "갓길 감시 지점",
        12: "끼어들기 금지",
        13: "교통정보 수집지점",
        14: "방범용cctv",
        15: "과적차량 위험구간",
        16: "적재 불량 단속",
        17: "주차단속 지점",
        18: "일방통행도로",
        19: "철길 건널목",
        20: "어린이 보호구역(스쿨존 시작 구간)",
        21: "어린이 보호구역(스쿨존 끝 구간)",
        22: "과속방지턱",
        23: "lpg충전소",
        24: "터널 구간",
        25: "휴게소",
        26: "톨게이트",
        27: "안개주의 지역",
        28: "유해물질 지역",
        29: "사고다발",
        30: "급커브지역",
        31: "급커브구간1",
        32: "급경사구간",
        33: "야생동물 교통사고 잦은 구간",
        34: "우측시야불량지점",
        35: "시야불량지점",
        36: "좌측시야불량지점",
        37: "신호위반다발구간",
        38: "과속운행다발구간",
        39: "교통혼잡지역",
        40: "방향별차로선택지점",
        41: "무단횡단사고다발지점",
        42: "갓길 사고 다발 지점",
        43: "과속 사발 다발 지점",
        44: "졸음 사고 다발 지점",
        45: "사고다발지점",
        46: "보행자 사고다발지점",
        47: "차량도난사고 상습발생지점",
        48: "낙석주의지역",
        49: "결빙주의지역",
        50: "병목지점",
        51: "합류 도로",
        52: "추락주의지역",
        53: "지하차도 구간",
        54: "주택밀집지역(교통진정지역)",
        55: "인터체인지",
        56: "분기점",
        57: "휴게소(lpg충전가능)",
        58: "교량",
        59: "제동장치사고다발지점",
        60: "중앙선침범사고다발지점",
        61: "통행위반사고다발지점",
        62: "목적지 건너편 안내",
        63: "졸음 쉼터 안내",
        64: "노후경유차단속",
        65: "터널내 차로변경단속",
        66: "",
    }

    sdi_en = {
        0: "Signal speed enforcement",
        1: "Speed camera (fixed)",
        2: "Section control start",
        3: "Section control end",
        4: "Under section control",
        5: "Block-the-box camera",
        6: "Signal violation enforcement",
        7: "Speed camera (mobile)",
        8: "Fixed speed camera zone (box)",
        9: "Bus-only lane zone",
        10: "Reversible/variable lane enforcement",
        11: "Shoulder surveillance point",
        12: "No cut-in",
        13: "Traffic data collection point",
        14: "Security CCTV",
        15: "Overloaded vehicle risk zone",
        16: "Improper loading enforcement",
        17: "Parking enforcement point",
        18: "One-way road",
        19: "Railroad crossing",
        20: "School zone start",
        21: "School zone end",
        22: "Speed bump",
        23: "LPG station",
        24: "Tunnel section",
        25: "Rest area",
        26: "Toll gate",
        27: "Fog caution area",
        28: "Hazardous materials area",
        29: "Accident-prone section",
        30: "Sharp curve area",
        31: "Sharp curve section 1",
        32: "Steep slope section",
        33: "Wild animal crossing area",
        34: "Poor visibility (right)",
        35: "Poor visibility",
        36: "Poor visibility (left)",
        37: "Frequent signal violations",
        38: "Frequent speeding",
        39: "Traffic congestion area",
        40: "Lane selection by direction",
        41: "Frequent jaywalking accidents",
        42: "Frequent shoulder accidents",
        43: "Frequent speeding accidents",
        44: "Frequent drowsy driving accidents",
        45: "Accident-prone spot",
        46: "Frequent pedestrian accidents",
        47: "Frequent vehicle theft",
        48: "Falling rock caution area",
        49: "Icy road caution area",
        50: "Bottleneck point",
        51: "Merging road",
        52: "Cliff/Drop caution area",
        53: "Underpass section",
        54: "Residential area (traffic calming)",
        55: "Interchange",
        56: "Junction",
        57: "Rest area (LPG available)",
        58: "Bridge",
        59: "Frequent brake failure accidents",
        60: "Center line invasion accidents",
        61: "Violation-of-passage accidents",
        62: "Destination on opposite side",
        63: "Drowsy rest area",
        64: "Old diesel control",
        65: "Lane change enforcement in tunnel",
        66: "",
    }

    sdi_zh = {
        0: "信号测速/闯灯拍照",
        1: "固定测速摄像头",
        2: "区间测速开始",
        3: "区间测速结束",
        4: "区间测速中",
        5: "路口压线摄像头",
        6: "闯红灯拍照",
        7: "流动测速摄像头",
        8: "测速拍照",
        9: "公交专用车道区间",
        10: "可变/潮汐车道拍照",
        11: "应急车道拍照",
        12: "禁止加塞",
        13: "交通信息采集点",
        14: "治安监控",
        15: "超载车辆风险区",
        16: "装载不当拍照",
        17: "违停拍照点",
        18: "单行道",
        19: "铁路道口",
        20: "学校区域开始",
        21: "学校区域结束",
        22: "减速带",
        23: "LPG加气站",
        24: "隧道区间",
        25: "服务区",
        26: "ETC计费拍照",
        27: "多雾路段",
        28: "危险品区域",
        29: "事故多发路段",
        30: "急弯路段",
        31: "急弯区段1",
        32: "陡坡路段",
        33: "野生动物出没路段",
        34: "右侧视野不良点",
        35: "视野不良点",
        36: "左侧视野不良点",
        37: "闯红灯多发",
        38: "超速多发",
        39: "交通拥堵区域",
        40: "按方向选择车道点",
        41: "行人乱穿马路多发处",
        42: "应急车道事故多发",
        43: "超速事故多发",
        44: "疲劳驾驶事故多发",
        45: "事故多发点",
        46: "行人事故多发点",
        47: "车辆盗窃多发点",
        48: "落石危险路段",
        49: "路面结冰危险",
        50: "瓶颈路段",
        51: "汇入道路",
        52: "坠落危险路段",
        53: "地下车道区间",
        54: "居民区（交通缓和）",
        55: "立交",
        56: "分岔点",
        57: "服务区（可加气）",
        58: "桥梁",
        59: "制动故障事故多发点",
        60: "越线事故多发点",
        61: "违法通行事故多发点",
        62: "目的地在对面",
        63: "瞌睡停车区",
        64: "老旧柴油车管制",
        65: "隧道内变道拍照",
        66: "",
    }

    sdi_map = sdi_en
    if self.lang == "ko":
      sdi_map = sdi_ko
    elif self.lang == "zh":
      sdi_map = sdi_zh

    return sdi_map.get(nSdiType, "")

  def _update_sdi(self):
    #sdiBlockType
    # 1: startOSEPS: 구간단속시작
    # 2: inOSEPS: 구간단속중
    # 3: endOSEPS: 구간단속종료
    # 0:감속안함,1:과속카메라,2:+사고방지턱,3:+이동식카메라
    if self.nSdiType in [0,1,2,3,4,7,8, 75, 76] and self.nSdiSpeedLimit > 0 and self.autoNaviSpeedCtrlMode > 0:
      self.xSpdLimit = self.nSdiSpeedLimit * self.autoNaviSpeedSafetyFactor
      self.xSpdDist = self.nSdiDist
      self.xSpdType = self.nSdiType
      if self.nSdiBlockType in [2,3]:
        self.xSpdDist = self.nSdiBlockDist
        self.xSpdType = 4
      elif self.nSdiType == 7 and self.autoNaviSpeedCtrlMode < 3: #이동식카메라
        self.xSpdLimit = self.xSpdDist = 0
    elif (self.nSdiPlusType == 22 or self.nSdiType == 22) and self.roadcate > 1 and self.autoNaviSpeedCtrlMode >= 2: # speed bump, roadcate:0,1: highway
      self.xSpdLimit = self.autoNaviSpeedBumpSpeed
      self.xSpdDist = self.nSdiPlusDist if self.nSdiPlusType == 22 else self.nSdiDist
      self.xSpdType = 22
    else:
      self.xSpdLimit = 0
      self.xSpdType = -1
      self.xSpdDist = 0

  def _update_gps(self, v_ego, sm, gps_service):
    gps = sm[gps_service]
    #print(f"location = {sm.valid[llk]}, {sm.updated[llk]}, {sm.recv_frame[llk]}, {sm.recv_time[llk]}")
    if not sm.updated['carState'] or not sm.updated['carControl']: # or not sm.updated[llk]:
      return self.nPosAngle
    CS = sm['carState']
    CC = sm['carControl']
    self.gps_valid = sm.updated[gps_service] and gps.hasFix

    now = time.monotonic()
    gps_updated_phone = (now - self.last_update_gps_time_phone) < 3
    gps_updated_navi = (now - self.last_update_gps_time_navi) < 3

    bearing = self.nPosAngle
    if gps_updated_phone:
      self.bearing_offset = 0.0
    elif self.gps_valid:
      bearing = self.nPosAngle = gps.bearingDeg
      if self.gps_valid:
        self.bearing_offset = 0.0
      elif self.active_carrot > 0:
        bearing = self.nPosAnglePhone
        self.bearing_offset = 0.0

    #print(f"bearing = {bearing:.1f}, posA=={self.nPosAngle:.1f}, posP=={self.nPosAnglePhone:.1f}, offset={self.bearing_offset:.1f}, {gps_updated_phone}, {gps_updated_navi}")
    gpsDelayTimeAdjust = 0.0
    if gps_updated_navi:
      gpsDelayTimeAdjust = 0 #1.0

    external_gps_update_timedout = not (gps_updated_phone or gps_updated_navi)
    #print(f"gps_valid = {self.gps_valid}, bearing = {bearing:.1f}, pos = {location.positionGeodetic.value[0]:.6f}, {location.positionGeodetic.value[1]:.6f}")
    if self.gps_valid and external_gps_update_timedout:    # 내부GPS가 자동하고 carrotman으로부터 gps신호가 없는경우
      self.vpPosPointLatNavi = gps.latitude
      self.vpPosPointLonNavi = gps.longitude
      self.last_calculate_gps_time = now #sm.recv_time[llk]
    elif gps_updated_navi:  # carrot navi로부터 gps신호가 수신되는 경우..
      if abs(self.bearing_measured - bearing) < 0.1:
          self.diff_angle_count += 1
      else:
          self.diff_angle_count = 0
      self.bearing_measured = bearing

      if self.diff_angle_count > 5: # 조향각도변화가 거의 없을때만 업데이트
        diff_angle = (self.nPosAngle - bearing) % 360
        if diff_angle > 180:
          diff_angle -= 360
        self.bearing_offset = self.bearing_offset * 0.9 + diff_angle * 0.1

    bearing_calculated = (bearing + self.bearing_offset) % 360

    dt = now - self.last_calculate_gps_time
    #print(f"dt = {dt:.1f}, {self.vpPosPointLatNavi}, {self.vpPosPointLonNavi}")
    if dt > 5.0:
      self.vpPosPointLat, self.vpPosPointLon = 0.0, 0.0
    elif dt == 0:
      self.vpPosPointLat, self.vpPosPointLon = self.vpPosPointLatNavi, self.vpPosPointLonNavi
    else:
      self.vpPosPointLat, self.vpPosPointLon = self.estimate_position(float(self.vpPosPointLatNavi), float(self.vpPosPointLonNavi), v_ego, bearing_calculated, dt + gpsDelayTimeAdjust)

    #self.debugText = " {} {:.1f},{:.1f}={:.1f}+{:.1f}".format(self.active_sdi_count, self.nPosAngle, bearing_calculated, bearing, self.bearing_offset)
    #print("nPosAngle = {:.1f},{:.1f} = {:.1f}+{:.1f}".format(self.nPosAngle, bearing_calculated, bearing, self.bearing_offset))

    return float(bearing_calculated)


  def estimate_position(self, lat, lon, speed, angle, dt):
    R = 6371000
    angle_rad = math.radians(angle)
    delta_d = speed * dt
    delta_lat = delta_d * math.cos(angle_rad) / R
    new_lat = lat + math.degrees(delta_lat)
    delta_lon = delta_d * math.sin(angle_rad) / (R * math.cos(math.radians(lat)))
    new_lon = lon + math.degrees(delta_lon)

    return new_lat, new_lon

  def update_auto_turn(self, v_ego_kph, sm, x_turn_info, x_dist_to_turn, check_steer=False):
    turn_speed = self.autoTurnControlSpeedTurn
    fork_speed = self.nRoadLimitSpeed
    stop_speed = 1
    turn_dist_for_speed = self.autoTurnControlTurnEnd * turn_speed / 3.6 # 5
    fork_dist_for_speed = self.autoTurnControlTurnEnd * fork_speed / 3.6 # 5
    stop_dist_for_speed = 5
    start_fork_dist = np.interp(self.nRoadLimitSpeed, [30, 50, 100], [160, 200, 350])
    start_turn_dist = np.interp(self.nTBTNextRoadWidth, [5, 10], [43, 60])
    turn_info_mapping = {
        1: {"type": "turn left", "speed": turn_speed, "dist": turn_dist_for_speed, "start": start_fork_dist},
        2: {"type": "turn right", "speed": turn_speed, "dist": turn_dist_for_speed, "start": start_fork_dist},
        5: {"type": "straight", "speed": turn_speed, "dist": turn_dist_for_speed, "start": start_turn_dist},
        3: {"type": "fork left", "speed": fork_speed, "dist": fork_dist_for_speed, "start": start_fork_dist},
        4: {"type": "fork right", "speed": fork_speed, "dist": fork_dist_for_speed, "start": start_fork_dist},
        6: {"type": "straight", "speed": fork_speed, "dist": fork_dist_for_speed, "start": start_fork_dist},
        7: {"type": "straight", "speed": stop_speed, "dist": stop_dist_for_speed, "start": 1000},
        8: {"type": "straight", "speed": stop_speed, "dist": stop_dist_for_speed, "start": 1000},
    }

    default_mapping = {"type": "none", "speed": 0, "dist": 0, "start": 1000}

    mapping = turn_info_mapping.get(x_turn_info, default_mapping)

    atc_type = mapping["type"]
    atc_speed = mapping["speed"]
    atc_dist = mapping["dist"]
    atc_start_dist = mapping["start"]

    if x_dist_to_turn > atc_start_dist:
      atc_type += " prepare"
      if check_steer:
        self.atc_activate_count = min(0, self.atc_activate_count - 1)
    else:
      if check_steer:
        self.atc_activate_count = max(0, self.atc_activate_count + 1)
      if atc_type in ["turn left", "turn right"] and x_dist_to_turn > start_turn_dist:
        atc_type = "atc left" if atc_type == "turn left" else "atc right"

    if self.autoTurnMapChange > 0 and check_steer:
      #print(f"x_dist_to_turn: {x_dist_to_turn}, atc_start_dist: {atc_start_dist}")
      #print(f"atc_activate_count: {self.atc_activate_count}")
      if self.atc_activate_count == 2:
        self.carrotCmdIndex += 100
        self.carrotCmd = "DISPLAY";
        self.carrotArg = "MAP";
      elif self.atc_activate_count == -50:
        self.carrotCmdIndex += 100
        self.carrotCmd = "DISPLAY";
        self.carrotArg = "ROAD";

    if check_steer:
      if 0 <= x_dist_to_turn < atc_start_dist and atc_type in ["fork left", "fork right"]:
        if not self.atc_paused:
          steering_pressed = sm["carState"].steeringPressed
          steering_torque = sm["carState"].steeringTorque
          if steering_pressed and steering_torque < 0 and atc_type in ["fork left", "atc left"]:
            self.atc_paused = True
          elif steering_pressed and steering_torque > 0 and atc_type in ["fork right", "atc right"]:
            self.atc_paused = True
      else:
        self.atc_paused = False

      if self.atc_paused:
        atc_type += " canceled"

    atc_desired = 250
    if atc_speed > 0 and x_dist_to_turn > 0:
      decel = self.autoNaviSpeedDecelRate
      safe_sec = 2.0
      atc_desired = min(atc_desired, self.calculate_current_speed(x_dist_to_turn - atc_dist, atc_speed, safe_sec, decel))


    return atc_desired, atc_type, atc_speed, atc_dist

  def update_nav_instruction(self, sm):
    if sm.alive['navInstruction'] and sm.valid['navInstruction']:
      msg_nav = sm['navInstruction']

      self.nGoPosDist = int(msg_nav.distanceRemaining)
      self.nGoPosTime = int(msg_nav.timeRemaining)
      if self.active_kisa_count <= 0 and msg_nav.speedLimit > 0:
        self.nRoadLimitSpeed = max(30, round(msg_nav.speedLimit * CV.MS_TO_KPH))
      self.xDistToTurn = int(msg_nav.maneuverDistance)
      self.szTBTMainText = msg_nav.maneuverPrimaryText
      self.xTurnInfo = -1
      for key, value in nav_type_mapping.items():
        if value[0] == msg_nav.maneuverType and value[1] == msg_nav.maneuverModifier:
          self.xTurnInfo = value[2]
          break

      self.debugText = f"{self.nRoadLimitSpeed if self.is_metric else self.nRoadLimitSpeed * CV.KPH_TO_MPH:.0f},{msg_nav.maneuverType},{msg_nav.maneuverModifier} "
      #print(msg_nav)
      #print(f"navInstruction: {self.xTurnInfo}, {self.xDistToTurn}, {self.szTBTMainText}")

  def update_kisa(self, data):
    self.active_kisa_count = 100
    if "kisawazecurrentspd" in data:
      pass
    if "kisawazeroadspdlimit" in data:
      road_limit_speed = data["kisawazeroadspdlimit"]
      if road_limit_speed > 0:
        print(f"kisawazeroadspdlimit: {road_limit_speed} km/h")
        if not self.is_metric:
          road_limit_speed *= CV.MPH_TO_KPH
        self.nRoadLimitSpeed = road_limit_speed
    if "kisawazealert" in data:
      pass
    if "kisawazeendalert" in data:
      pass
    if "kisawazeroadname" in data:
      print(f"kisawazeroadname: {data['kisawazeroadname']}")
      self.szPosRoadName = data["kisawazeroadname"]
    if "kisawazereportid" in data and "kisawazealertdist" in data:
      id_str = data["kisawazereportid"]
      dist_str = data["kisawazealertdist"].lower()
      import re
      match = re.search(r'(\d+)', dist_str)
      distance = int(match.group(1)) if match else 0
      if not self.is_metric:
        distance = int(distance * 0.3048)
      print(f"{id_str}: {distance} m")
      xSpdType = -1
      if 'camera' in id_str:
        xSpdType = 101    # 101: waze speed cam, 100: police
      elif 'police' in id_str:
        xSpdType = 100

      if xSpdType >= 0:
        offset = 5 if self.is_metric else 5 * CV.MPH_TO_KPH
        self.xSpdLimit = self.nRoadLimitSpeed + offset

        self.xSpdDist = distance
        self.xSpdType =xSpdType

  def update_navi(self, remote_ip, sm, pm, vturn_speed, coords, distances, route_speed, gps_service):

    self.debugText = ""
    self.update_params()
    if sm.alive['carState'] and sm.alive['selfdriveState']:
      CS = sm['carState']
      v_ego = CS.vEgo
      v_ego_kph = v_ego * 3.6
      distanceTraveled = sm['selfdriveState'].distanceTraveled
      delta_dist = distanceTraveled - self.totalDistance
      self.totalDistance = distanceTraveled
      if CS.speedLimit > 0 and self.active_carrot <= 1:
        self.nRoadLimitSpeed = CS.speedLimit
    else:
      v_ego = v_ego_kph = 0
      delta_dist = 0
      CS = None

    road_speed_limit_changed = True if self.nRoadLimitSpeed != self.nRoadLimitSpeed_last else False
    self.nRoadLimitSpeed_last = self.nRoadLimitSpeed
    #self.bearing = self.nPosAngle #self._update_gps(v_ego, sm)
    self.bearing = self._update_gps(v_ego, sm, gps_service)

    self.xSpdDist = max(self.xSpdDist - delta_dist, -1000)
    self.xDistToTurn = self.xDistToTurn - delta_dist
    self.xDistToTurnNext = self.xDistToTurnNext - delta_dist
    self.active_count = max(self.active_count - 1, 0)
    self.active_sdi_count = max(self.active_sdi_count - 1, 0)
    self.active_kisa_count = max(self.active_kisa_count - 1, 0)
    if self.active_kisa_count > 0:
      self.active_carrot = 2

    elif self.active_count > 0:
      self.active_carrot = 2 if self.active_sdi_count > 0 else 1
    else:
      self.active_carrot = 0

    if self.autoRoadSpeedLimitOffset >= 0 and self.active_carrot>=2:
      if self.nRoadLimitSpeed >= 30:
        road_speed_limit_offset = self.autoRoadSpeedLimitOffset
        if not self.is_metric:
          road_speed_limit_offset *= CV.KPH_TO_MPH
        limit_speed = self.nRoadLimitSpeed + road_speed_limit_offset
    else:
      limit_speed = 200

    if self.active_carrot <= 1:
      self.xSpdType = self.navType = self.xTurnInfo = self.xTurnInfoNext = -1
      self.nSdiType = self.nSdiBlockType = self.nSdiPlusBlockType = -1
      self.nTBTTurnType = self.nTBTTurnTypeNext = -1
      self.roadcate = 8
      self.nGoPosDist = 0
    if self.active_carrot <= 1 or self.active_kisa_count > 0:
      self.update_nav_instruction(sm)

    if self.xSpdType < 0 or (self.xSpdType not in [100,101] and self.xSpdDist <= 0) or (self.xSpdType in [100,101] and self.xSpdDist < -250):
      self.xSpdType = -1
      self.xSpdDist = self.xSpdLimit = 0
    if self.xTurnInfo < 0 or self.xDistToTurn < -50:
      if self.xDistToTurn > 0:
        self.xDistToTurn = 0
      self.xTurnInfo = -1
      self.xDistToTurnNext = 0
      self.xTurnInfoNext = -1

    sdi_speed = 250
    hda_active = False
    ### 과속카메라, 사고방지턱
    if (self.xSpdDist > 0 or self.xSpdType in [100, 101]) and self.active_carrot > 0:
      safe_sec = self.autoNaviSpeedBumpTime if self.xSpdType == 22 else self.autoNaviSpeedCtrlEnd
      decel = self.autoNaviSpeedDecelRate
      sdi_speed = min(sdi_speed, self.calculate_current_speed(self.xSpdDist, self.xSpdLimit, safe_sec, decel))
      self.active_carrot = 5 if self.xSpdType == 22 else 3
      if self.xSpdType == 4 or (self.xSpdType in [100, 101] and self.xSpdDist <= 0):
        sdi_speed = self.xSpdLimit
        self.active_carrot = 4
    elif CS is not None and CS.speedLimit > 0 and CS.speedLimitDistance > 0:
      sdi_speed = min(sdi_speed,
                      self.calculate_current_speed(CS.speedLimitDistance,
                                                   CS.speedLimit * self.autoNaviSpeedSafetyFactor,
                                                   self.autoNaviSpeedCtrlEnd,
                                                   self.autoNaviSpeedDecelRate))
      #self.active_carrot = 6
      hda_active = True

    #print(f"sdi_speed: {sdi_speed}, hda_active: {hda_active}, xSpdType: {self.xSpdType}, xSpdDist: {self.xSpdDist}, active_carrot: {self.active_carrot}, v_ego_kph: {v_ego_kph}, nRoadLimitSpeed: {self.nRoadLimitSpeed}")
    ### TBT 속도제어
    atc_desired, self.atcType, self.atcSpeed, self.atcDist = self.update_auto_turn(v_ego*3.6, sm, self.xTurnInfo, self.xDistToTurn, True)
    atc_desired_next, _, _, _ = self.update_auto_turn(v_ego*3.6, sm, self.xTurnInfoNext, self.xDistToTurnNext, False)

    if self.nSdiType  >= 0: # or self.active_carrot > 0:
      pass
      # self.debugText = (
      #   f"Atc:{atc_desired:.1f}," +
      #   f"{self.xTurnInfo}:{self.xDistToTurn:.1f}, " +
      #   f"I({self.nTBTNextRoadWidth},{self.roadcate}) " +
      #   f"Atc2:{atc_desired_next:.1f}," +
      #   f"{self.xTurnInfoNext},{self.xDistToTurnNext:.1f}"
      # )
      #self.debugText = "" #f" {self.nSdiType}/{self.nSdiSpeedLimit}/{self.nSdiDist},BLOCK:{self.nSdiBlockType}/{self.nSdiBlockSpeed}/{self.nSdiBlockDist}, PLUS:{self.nSdiPlusType}/{self.nSdiPlusSpeedLimit}/{self.nSdiPlusDist}"
    #elif self.nGoPosDist > 0 and self.active_carrot > 1:
    #  self.debugText = " 목적지:{:.1f}km/{:.1f}분 남음".format(self.nGoPosDist/1000., self.nGoPosTime / 60)
    else:
      #self.debugText = ""
      pass

    if self.autoTurnControl not in [2, 3]:    # auto turn speed control
      atc_desired = atc_desired_next = 250

    if self.autoTurnControl not in [1,2]:    # auto turn control
      self.atcType = "none"


    speed_n_sources = [
      (atc_desired, "atc"),
      (atc_desired_next, "atc2"),
      (sdi_speed, "hda" if hda_active else "bump" if self.xSpdType == 22 else "section" if self.xSpdType == 4 else "police" if self.xSpdType == 100 else "waze" if self.xSpdType == 101 else "cam"),
      (limit_speed, "road"),
    ]
    if self.turnSpeedControlMode in [1,2]:
      speed_n_sources.append((max(abs(vturn_speed), self.autoCurveSpeedLowerLimit), "vturn"))

    route_speed = max(route_speed * self.mapTurnSpeedFactor, self.autoCurveSpeedLowerLimit)
    if self.turnSpeedControlMode == 2:
      if -500 < self.xDistToTurn < 500:
        speed_n_sources.append((route_speed, "route"))
    elif self.turnSpeedControlMode in [3, 4]:
      speed_n_sources.append((route_speed, "route"))
      #speed_n_sources.append((self.calculate_current_speed(dist, speed * self.mapTurnSpeedFactor, 0, 1.2), "route"))

    model_turn_speed = max(sm['modelV2'].meta.modelTurnSpeed, self.autoCurveSpeedLowerLimit)
    if model_turn_speed < 200 and abs(vturn_speed) < 120:
      speed_n_sources.append((model_turn_speed, "model"))

    desired_speed, source = min(speed_n_sources, key=lambda x: x[0])

    if CS is not None:
      if source != self.source_last:
        self.gas_override_speed = 0
        self.gas_pressed_state = CS.gasPressed
      if CS.vEgo < 0.1 or desired_speed > 150 or source in ["cam", "section", "police"] or CS.brakePressed or road_speed_limit_changed:
        self.gas_override_speed = 0
      elif CS.gasPressed and not self.gas_pressed_state:
        self.gas_override_speed = max(v_ego_kph, self.gas_override_speed)
      else:
        self.gas_pressed_state = False
      self.source_last = source

      if desired_speed < self.gas_override_speed:
        source = "gas"
        desired_speed = self.gas_override_speed

      self.debugText += f"route={route_speed:.1f}"#f"desired={desired_speed:.1f},{source},g={self.gas_override_speed:.0f}"

    left_spd_sec = 100
    left_tbt_sec = 100
    if self.autoNaviCountDownMode > 0:
      if self.xSpdType == 22 and self.autoNaviCountDownMode == 1: # speed bump
        pass
      else:
        if self.xSpdDist > 0:
          left_spd_sec = min(self.left_spd_sec, int(max(self.xSpdDist - v_ego, 1) / max(1, v_ego) + 0.5))

      if self.xDistToTurn > 0:
        left_tbt_sec = min(self.left_tbt_sec, int(max(self.xDistToTurn - v_ego, 1) / max(1, v_ego) + 0.5))

    self.left_spd_sec = left_spd_sec
    self.left_tbt_sec = left_tbt_sec

    left_sec = min(left_spd_sec, left_tbt_sec)

    if left_sec > 11:
      self.left_sec = 100
      self.max_left_sec = 100
    else:
      self.sdi_inform = True if source in ["cam", "hda"] else False
      self.max_left_sec = min(11, max(6, int(v_ego_kph/10) + 1))

    if left_sec != self.left_sec:
      if left_sec == self.max_left_sec and self.sdi_inform:
        self.carrot_left_sec = 11
      elif 1 <= left_sec < self.max_left_sec:
        self.carrot_left_sec = left_sec
      elif left_sec == 0 and self.left_sec == 1:
        self.carrot_left_sec = left_sec

      self.left_sec = left_sec


    self._update_cmd()
    msg = messaging.new_message('carrotMan')
    msg.valid = True
    msg.carrotMan.activeCarrot = self.active_carrot
    msg.carrotMan.nRoadLimitSpeed = int(self.nRoadLimitSpeed)
    msg.carrotMan.remote = remote_ip
    msg.carrotMan.xSpdType = int(self.xSpdType)
    msg.carrotMan.xSpdLimit = int(self.xSpdLimit)
    msg.carrotMan.xSpdDist = int(self.xSpdDist)
    msg.carrotMan.xSpdCountDown = int(left_spd_sec)
    msg.carrotMan.xTurnInfo = int(self.xTurnInfo)
    msg.carrotMan.xDistToTurn = int(self.xDistToTurn)
    msg.carrotMan.xTurnCountDown = int(left_tbt_sec)
    msg.carrotMan.atcType = self.atcType
    msg.carrotMan.vTurnSpeed = int(vturn_speed)
    msg.carrotMan.szPosRoadName = self.szPosRoadName + self.debugText
    msg.carrotMan.szTBTMainText = self.szTBTMainText
    msg.carrotMan.desiredSpeed = int(desired_speed)
    msg.carrotMan.desiredSource = source
    msg.carrotMan.carrotCmdIndex = int(self.carrotCmdIndex)
    msg.carrotMan.carrotCmd = self.carrotCmd
    msg.carrotMan.carrotArg = self.carrotArg
    msg.carrotMan.trafficState = self.traffic_state

    msg.carrotMan.xPosSpeed = float(v_ego_kph) #float(self.nPosSpeed)
    msg.carrotMan.xPosAngle = float(self.bearing)
    msg.carrotMan.xPosLat = float(self.vpPosPointLat)
    msg.carrotMan.xPosLon = float(self.vpPosPointLon)

    msg.carrotMan.nGoPosDist = self.nGoPosDist
    msg.carrotMan.nGoPosTime = self.nGoPosTime
    msg.carrotMan.szSdiDescr = self._get_sdi_descr(-1 if self.nSdiType == 0 and self.nSdiDist == 0 else self.nSdiType)

    #coords_str = ";".join([f"{x},{y}" for x, y in coords])
    coords_str = ";".join([f"{x:.2f},{y:.2f},{d:.2f}" for (x, y), d in zip(coords, distances, strict=False)])
    msg.carrotMan.naviPaths = coords_str

    msg.carrotMan.leftSec = int(self.carrot_left_sec)
    pm.send('carrotMan', msg)

    inst = messaging.new_message('navInstructionCarrot')
    if self.active_carrot > 1 and self.active_kisa_count <= 0:
      inst.valid = True

      instruction = inst.navInstructionCarrot
      instruction.distanceRemaining = self.nGoPosDist
      instruction.timeRemaining = self.nGoPosTime
      instruction.speedLimit = self.nRoadLimitSpeed / 3.6 if self.nRoadLimitSpeed > 0 else 0
      instruction.maneuverDistance = float(self.nTBTDist)
      instruction.maneuverSecondaryText = self.szNearDirName
      if self.szFarDirName and len(self.szFarDirName):
        instruction.maneuverSecondaryText += "[{}]".format(self.szFarDirName)
      instruction.maneuverPrimaryText = self.szTBTMainText
      instruction.timeRemainingTypical = self.nGoPosTime

      navType, navModifier, xTurnInfo1 = "invalid", "", -1
      if self.nTBTTurnType in nav_type_mapping:
        navType, navModifier, xTurnInfo1 = nav_type_mapping[self.nTBTTurnType]
      navTypeNext, navModifierNext, xTurnInfoNext = "invalid", "", -1
      if self.nTBTTurnTypeNext in nav_type_mapping:
        navTypeNext, navModifierNext, xTurnInfoNext = nav_type_mapping[self.nTBTTurnTypeNext]

      instruction.maneuverType = navType
      instruction.maneuverModifier = navModifier

      maneuvers = []
      if self.nTBTTurnType >= 0:
        maneuver = {}
        maneuver['distance'] = float(self.xDistToTurn)
        maneuver['type'] = navType
        maneuver['modifier'] = navModifier
        maneuvers.append(maneuver)
        if self.nTBTDistNext >= self.nTBTDist:
          maneuver = {}
          maneuver['distance'] = float(self.nTBTDistNext)
          maneuver['type'] = navTypeNext
          maneuver['modifier'] = navModifierNext
          maneuvers.append(maneuver)

      instruction.allManeuvers = maneuvers
    elif sm.alive['navInstruction'] and sm.valid['navInstruction']:
      inst.navInstructionCarrot = sm['navInstruction']

    pm.send('navInstructionCarrot', inst)

  def _update_system_time(self, epoch_time_remote, timezone_remote):
    epoch_time = int(time.time())
    if epoch_time_remote > 0:
      epoch_time_offset = epoch_time_remote - epoch_time
      print(f"epoch_time_offset = {epoch_time_offset}")
      if abs(epoch_time_offset) > 60:
        os.system(f"sudo timedatectl set-timezone {timezone_remote}")
        formatted_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(epoch_time_remote))
        print(f"Setting system time to: {formatted_time}")
        os.system(f'sudo date -s "{formatted_time}"')

  def set_time(self, epoch_time, timezone):
    import datetime
    new_time = datetime.datetime.utcfromtimestamp(epoch_time)
    localtime_path = "/data/etc/localtime"

    no_timezone = False
    try:
      if os.path.getsize(localtime_path) == 0:
        no_timezone = True
    except (FileNotFoundError, OSError):
      no_timezone = True

    diff = datetime.datetime.utcnow() - new_time
    if abs(diff) < datetime.timedelta(seconds=10) and not no_timezone:
      #print(f"Time diff too small: {diff}")
      return

    print(f"Setting time to {new_time}, diff={diff}")
    zoneinfo_path = f"/usr/share/zoneinfo/{timezone}"
    if os.path.exists(localtime_path) or os.path.islink(localtime_path):
        try:
            subprocess.run(["sudo", "rm", "-f", localtime_path], check=True)
            print(f"Removed existing file or link: {localtime_path}")
        except subprocess.CalledProcessError as e:
            print(f"Error removing {localtime_path}: {e}")
            return
    try:
        subprocess.run(["sudo", "ln", "-s", zoneinfo_path, localtime_path], check=True)
        print(f"Timezone successfully set to: {timezone}")
    except subprocess.CalledProcessError as e:
        print(f"Failed to set timezone to {timezone}: {e}")


    try:
      subprocess.run(f"TZ=UTC date -s '{new_time}'", shell=True, check=True)
      #subprocess.run()
    except subprocess.CalledProcessError:
      print("timed.failed_setting_time")

  def update(self, json):
    if json is None:
      return
    if "carrotIndex" in json:
      self.carrotIndex = int(json.get("carrotIndex"))

    #print(json)
    if self.carrotIndex % 60 == 0 and "epochTime" in json:
      # op는 ntp를 사용하기때문에... 필요없는 루틴으로 보임.
      timezone_remote = json.get("timezone", "Asia/Seoul")

      if not PC:
        self.set_time(int(json.get("epochTime")), timezone_remote)

      #self._update_system_time(int(json.get("epochTime")), timezone_remote)

    if "carrotCmd" in json:
      #print(json.get("carrotCmd"), json.get("carrotArg"))
      self.carrotCmdIndex = self.carrotIndex
      self.carrotCmd = json.get("carrotCmd")
      self.carrotArg = json.get("carrotArg")
      print(f"carrotCmd = {self.carrotCmd}, {self.carrotArg}")

    self.active_count = 80
    now = time.monotonic()

    if "goalPosX" in json:
      self.goalPosX = float(json.get("goalPosX", self.goalPosX))
      self.goalPosY = float(json.get("goalPosY", self.goalPosY))
      self.szGoalName = json.get("szGoalName", self.szGoalName)

    if "nRoadLimitSpeed" in json:
      #print(json)
      self.active_sdi_count = self.active_sdi_count_max
      ### roadLimitSpeed
      nRoadLimitSpeed = int(json.get("nRoadLimitSpeed", 20))
      if nRoadLimitSpeed > 0:
        if nRoadLimitSpeed > 200:
          nRoadLimitSpeed = (nRoadLimitSpeed - 20) / 10
        elif nRoadLimitSpeed == 120:
          nRoadLimitSpeed = 115 # 120 -> 115 fix bug
      else:
        nRoadLimitSpeed = 30
      #self.nRoadLimitSpeed = nRoadLimitSpeed
      if self.nRoadLimitSpeed != nRoadLimitSpeed:
        self.nRoadLimitSpeed_counter += 1
        if self.nRoadLimitSpeed_counter > 5:
          self.nRoadLimitSpeed = nRoadLimitSpeed
      else:
        self.nRoadLimitSpeed_counter = 0

      ### SDI
      self.nSdiType = int(json.get("nSdiType", -1))
      self.nSdiSpeedLimit = int(json.get("nSdiSpeedLimit", 0))
      self.nSdiSection = int(json.get("nSdiSection", -1))
      self.nSdiDist = int(json.get("nSdiDist", -1))
      self.nSdiBlockType = int(json.get("nSdiBlockType", -1))
      self.nSdiBlockSpeed = int(json.get("nSdiBlockSpeed", 0))
      self.nSdiBlockDist = int(json.get("nSdiBlockDist", 0))

      self.nSdiPlusType = int(json.get("nSdiPlusType", -1))
      self.nSdiPlusSpeedLimit = int(json.get("nSdiPlusSpeedLimit", 0))
      self.nSdiPlusDist = int(json.get("nSdiPlusDist", 0))
      self.nSdiPlusBlockType = int(json.get("nSdiPlusBlockType", -1))
      self.nSdiPlusBlockSpeed = int(json.get("nSdiPlusBlockSpeed", 0))
      self.nSdiPlusBlockDist = int(json.get("nSdiPlusBlockDist", 0))
      self.roadcate = int(json.get("roadcate", 0))

      ## GuidePoint
      self.nTBTDist = int(json.get("nTBTDist", 0))
      self.nTBTTurnType = int(json.get("nTBTTurnType", -1))
      self.szTBTMainText = json.get("szTBTMainText", "")
      self.szNearDirName = json.get("szNearDirName", "")
      self.szFarDirName = json.get("szFarDirName", "")

      self.nTBTNextRoadWidth = int(json.get("nTBTNextRoadWidth", 0))
      self.nTBTDistNext = int(json.get("nTBTDistNext", 0))
      self.nTBTTurnTypeNext = int(json.get("nTBTTurnTypeNext", -1))
      self.szTBTMainTextNext = json.get("szTBTMainText", "")

      self.nGoPosDist = int(json.get("nGoPosDist", 0))
      self.nGoPosTime = int(json.get("nGoPosTime", 0))
      self.szPosRoadName = json.get("szPosRoadName", "")
      if self.szPosRoadName == "null":
        self.szPosRoadName = ""

      self.vpPosPointLatNavi = float(json.get("vpPosPointLat", 0.0))
      self.vpPosPointLonNavi = float(json.get("vpPosPointLon", 0.0))
      if self.vpPosPointLatNavi != 0.0:
        self.last_update_gps_time_navi = self.last_calculate_gps_time = now
        self.nPosAngle = float(json.get("nPosAngle", self.nPosAngle))

      self.nPosSpeed = float(json.get("nPosSpeed", self.nPosSpeed))
      self._update_tbt()
      self._update_sdi()
      print(
        f"sdi = {self.nSdiType}, {self.nSdiSpeedLimit}, {self.nSdiPlusType}, " +
        f"tbt = {self.nTBTTurnType}, {self.nTBTDist}, " +
        f"next = {self.nTBTTurnTypeNext}, {self.nTBTDistNext}"
      )
      #print(json)
    else:
      #print(json)
      pass

    # 3초간 navi 데이터가 없으면, phone gps로 업데이트
    if "latitude" in json:
      self.nPosAnglePhone = float(json.get("heading", self.nPosAngle))
      self.phone_latitude = float(json.get("latitude", self.vpPosPointLatNavi))
      self.phone_longitude = float(json.get("longitude", self.vpPosPointLonNavi))
      self.phone_gps_accuracy = float(json.get("accuracy", 0))
      if self.phone_gps_accuracy < 15.0:
        self.phone_gps_frame += 1
      if (now - self.last_update_gps_time_navi) > 3.0:
        self.vpPosPointLatNavi = self.phone_latitude
        self.vpPosPointLonNavi = self.phone_longitude

        self.nPosAngle = self.nPosAnglePhone
        # self.nPosSpeed = self.ve # TODO speed from v_ego
        self.last_update_gps_time_phone = self.last_calculate_gps_time = now        
        self.nPosSpeed = float(json.get("gps_speed", 0))
        print(f"phone gps: {self.vpPosPointLatNavi}, {self.vpPosPointLonNavi}, {self.phone_gps_accuracy}, {self.nPosSpeed}")


import traceback

def main():
  print("CarrotManager Started")
  #print("Carrot GitBranch = {}, {}".format(Params().get("GitBranch"), Params().get("GitCommitDate")))
  # 延迟导入，避免与 carrot_man 中导入 CarrotServ 的循环依赖
  from openpilot.selfdrive.carrot.carrot_man import CarrotMan
  carrot_man = CarrotMan()

  print(f"CarrotMan {carrot_man}")
  threading.Thread(target=carrot_man.kisa_app_thread).start()
  while True:
    try:
      carrot_man.carrot_man_thread()
    except Exception as e:
      print(f"carrot_man error...: {e}")
      traceback.print_exc()
      time.sleep(10)


if __name__ == "__main__":
  main()
