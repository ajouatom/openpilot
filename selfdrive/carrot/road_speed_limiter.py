#!/usr/bin/env python3
import json
import os
import random
import math

import select
import subprocess
import threading
import time
from datetime import datetime
import socket
import fcntl
import struct
from threading import Thread
from cereal import messaging, log
from openpilot.common.numpy_fast import clip
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import Ratekeeper
from openpilot.system.hardware import TICI
from openpilot.common.params import Params
import subprocess
from openpilot.selfdrive.navd.helpers import Coordinate
import traceback

CAMERA_SPEED_FACTOR = 1.05


class Port:
  BROADCAST_PORT = 7708
  RECEIVE_PORT = 7707
  LOCATION_PORT = BROADCAST_PORT


class RoadLimitSpeedServer:
  def __init__(self):
    self.json_road_limit = None
    self.json_apilot = None
    self.active = 0
    self.active_apilot = 0
    self.last_updated = 0
    self.last_updated_apilot = 0
    self.last_updated_active = 0
    self.last_exception = None
    self.lock = threading.Lock()
    self.remote_addr = None

    self.remote_gps_addr = None
    self.last_time_location = 0
    
    broadcast = Thread(target=self.broadcast_thread, args=[])
    broadcast.daemon = True
    broadcast.start()

    self.gps_sm = messaging.SubMaster(['gpsLocationExternal'], poll='gpsLocationExternal')
    self.gps_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    self.location = None

    #self.gps_event = threading.Event()
    #gps_thread = Thread(target=self.gps_thread, args=[])
    #gps_thread.daemon = True
    #gps_thread.start()


    #carrot
    #self.carrot_route_thread = threading.Thread(target=self.carrot_route, args=[])
    #self.carrot_route_thread.daemon = True
    #self.carrot_route_thread.start()

  def recvall(self, sock, n):
    """n바이트를 수신할 때까지 반복적으로 데이터를 받는 함수"""
    data = bytearray()
    while len(data) < n:
      packet = sock.recv(n - len(data))
      if not packet:
        return None
      data.extend(packet)
    return data

  def receive_double(self, sock):
    double_data = self.recvall(sock, 8)  # Double은 8바이트
    return struct.unpack('!d', double_data)[0]

  def receive_float(self, sock):
    float_data = self.recvall(sock, 4)  # Float은 4바이트
    return struct.unpack('!f', float_data)[0]

  def carrot_route(self):
    host = '0.0.0.0'  # 혹은 다른 호스트 주소
    port = 7709  # 포트 번호

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
      s.bind((host, port))
      s.listen()

      while True:
        print("################# waiting conntection from CarrotMan route #####################")
        # 클라이언트 연결 기다림
        conn, addr = s.accept()
        with conn:
          print(f"Connected by {addr}")


          # 전체 데이터 크기 수신
          total_size_bytes = self.recvall(conn, 4)
          if not total_size_bytes:
            print("Connection closed or error occurred")
            continue
          try:
            total_size = struct.unpack('!I', total_size_bytes)[0]
            # 전체 데이터를 한 번에 수신
            all_data = self.recvall(conn, total_size)
            if all_data is None:
                print("Connection closed or incomplete data received")
                continue

           # 수신된 데이터를 float 값들로 분할
            #points = []
            #for i in range(0, len(all_data), 8):  # 각 점에 대해 8바이트(4바이트 * 2)
            #  x, y = struct.unpack('!ff', all_data[i:i+8])
            #  points.append((x, y))

            points = []
            for i in range(0, len(all_data), 8):
              x, y = struct.unpack('!ff', all_data[i:i+8])
              coord = Coordinate.from_mapbox_tuple((y, x))
              points.append(coord)
            coords = [c.as_dict() for c in points]
            #points = [
            #  {'latitude': y, 'longitude': x}
            #  for i in range(0, len(all_data), 8) 
            #  for x, y in [struct.unpack('!ff', all_data[i:i+8])]
            #]
         
            print("Received points:", len(points))
            print("Received points:", coords)

          except Exception as e:
            print(e)



  def gps_thread(self):
    rk = Ratekeeper(3.0, print_delay_threshold=None)
    while True:
      self.gps_timer()
      rk.keep_time()

  def gps_timer(self):
    try:
      if self.remote_gps_addr is not None:
        self.gps_sm.update(0)
        if self.gps_sm.updated['gpsLocationExternal']:
          self.location = self.gps_sm['gpsLocationExternal']

        if self.location is not None:
          json_location = json.dumps({"location": [
            self.location.latitude,
            self.location.longitude,
            self.location.altitude,
            self.location.speed,
            self.location.bearingDeg,
            self.location.accuracy,
            self.location.unixTimestampMillis,
            # self.location.source,
            # self.location.vNED,
            self.location.verticalAccuracy,
            self.location.bearingAccuracyDeg,
            self.location.speedAccuracy,
          ]})

          address = (self.remote_gps_addr[0], Port.LOCATION_PORT)
          self.gps_socket.sendto(json_location.encode(), address)

    except:
      self.remote_gps_addr = None

  def get_broadcast_address(self):
    try:
      with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        ip = fcntl.ioctl(
          s.fileno(),
          0x8919,
          struct.pack('256s', 'wlan0'.encode('utf-8'))
        )[20:24]
        return socket.inet_ntoa(ip)
    except:
      return None

  def broadcast_thread(self):

    broadcast_address = None
    frame = 0

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
      try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  ## test.. carrot
        while True:

          try:

            if broadcast_address is None or frame % 10 == 0:
              broadcast_address = self.get_broadcast_address()

            if broadcast_address is not None and self.remote_addr is None:
              print('broadcast', broadcast_address)

              msg = self.make_msg()
              for i in range(1, 255):
                ip_tuple = socket.inet_aton(broadcast_address)
                new_ip = ip_tuple[:-1] + bytes([i])
                address = (socket.inet_ntoa(new_ip), Port.BROADCAST_PORT)
                sock.sendto(msg.encode(), address)
          except Exception as e:
            print("$$$$$$RoadSpeedLimiter Exception: " + str(e))
            pass

          time.sleep(5.)
          frame += 1

      except:
        pass

  def make_msg(self):
    msg = {}
    msg['Carrot'] = Params().get("Version").decode('utf-8')
    msg['IsOnroad'] = Params().get_bool("IsOnroad")
    msg['CarrotRouteActive'] = Params().get_bool("CarrotRouteActive")
    return json.dumps(msg)


  def send_sdp(self, sock):
    try:
      sock.sendto(self.make_msg().encode(), (self.remote_addr[0], Port.BROADCAST_PORT))
    except:
      pass

  def udp_recv(self, sock, wait_time):
    ret = False
    try:
      #print("udp_recv")
      ready = select.select([sock], [], [], wait_time)
      ret = bool(ready[0])
      if ret:
        data, self.remote_addr = sock.recvfrom(2048)
        json_obj = json.loads(data.decode())
        #print(json_obj)

        if 'cmd' in json_obj:
          try:
            os.system(json_obj['cmd'])
            ret = False
          except:
            pass

        if 'request_gps' in json_obj:
          try:
            if json_obj['request_gps'] == 1:
              self.remote_gps_addr = self.remote_addr
            else:
              self.remote_gps_addr = None
            ret = False
          except:
            pass

        if 'echo' in json_obj:
          try:
            echo = json.dumps(json_obj["echo"])
            sock.sendto(echo.encode(), (self.remote_addr[0], Port.BROADCAST_PORT))
            ret = False
          except:
            pass

        if 'echo_cmd' in json_obj:
          try:
            result = subprocess.run(json_obj['echo_cmd'], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            echo = json.dumps({"echo_cmd": json_obj['echo_cmd'], "result": result.stdout})
            sock.sendto(echo.encode(), (self.remote_addr[0], Port.BROADCAST_PORT))
            ret = False
          except:
            pass

        try:
          self.lock.acquire()
          try:
            if 'active' in json_obj:
              self.active = json_obj['active']
              self.last_updated_active = time.monotonic()
          except:
            pass

          if 'road_limit' in json_obj:
            self.json_road_limit = json_obj['road_limit']
            self.last_updated = time.monotonic()

          if 'apilot' in json_obj:
            self.json_apilot = json_obj['apilot']
            self.last_updated_apilot = time.monotonic()

        finally:
          self.lock.release()

    except:

      try:
        self.lock.acquire()
        self.json_road_limit = None
      finally:
        self.lock.release()

    return ret

  def check(self):
    now = time.monotonic()
    if now - self.last_updated > 6.:
      try:
        self.lock.acquire()
        self.json_road_limit = None
      finally:
        self.lock.release()

    if now - self.last_updated_apilot > 6.:
      try:
        self.lock.acquire()
        self.json_apilot = None
      finally:
        self.lock.release()

    if now - self.last_updated_active > 6.:
      self.active = 0
      #self.remote_addr = None
    if now - self.last_updated_apilot > 6.:
      self.active_apilot = 0
      self.remote_addr = None


  def get_limit_val(self, key, default=None):
    return self.get_json_val(self.json_road_limit, key, default)

  def get_apilot_val(self, key, default=None):
    return self.get_json_val(self.json_apilot, key, default)


  def get_json_val(self, json, key, default=None):

    try:
      if json is None:
        return default

      if key in json:
        return json[key]

    except:
      pass

    return default

def estimate_position(lat, lon, speed, angle, dt):
    R = 6371000
    angle_rad = math.radians(angle)
    delta_d = speed * dt
    delta_lat = delta_d * math.cos(angle_rad) / R
    new_lat = lat + math.degrees(delta_lat)
    delta_lon = delta_d * math.sin(angle_rad) / (R * math.cos(math.radians(lat)))
    new_lon = lon + math.degrees(delta_lon)
    
    return new_lat, new_lon

def main():
  print("RoadLimitSpeed Started.....")
  server = RoadLimitSpeedServer()

  pm = messaging.PubMaster(['roadLimitSpeed'])
  sm = messaging.SubMaster(['carState', 'liveLocationKalman', 'naviData'], poll='liveLocationKalman')
  carState = None
  CS = None
  naviData = None
  naviData_update_count = 0

  xTurnInfo = -1
  xDistToTurn = -1
  xSpdDist = -1
  xSpdLimit = -1
  xSignType = -1
  xRoadSignType = -1
  xRoadLimitSpeed = -1
  xRoadName = ""

  xBumpDistance = 0
  xTurnInfo_prev = xTurnInfo
  sdiDebugText = ""

  sdi_valid_count = 0
  apm_valid_count = 0
  sdiType = -1

  totalDistance = 0.0
  mappyMode = True
  mappyMode_valid = False

  nTBTTurnType = -1
  nTBTTurnTypeNext = -1
  nTBTNextRoadWidth = 0
  nSdiType = -1
  nSdiDist = -1
  nSdiSpeedLimit = -1
  nSdiPlusType = -1
  nSdiPlusDist = -1
  nSdiPlusSpeedLimit = -1
  nSdiBlockType = -1
  nSdiBlockSpeed = -1
  nSdiBlockDist = -1
  nTBTDist = -1
  nTBTDistNext = -1
  nRoadLimitSpeed = -1
  xIndex = 0
  roadcate = 7    # roadCategory, 0,1: highway,
  nLaneCount = 0
  szNearDirName = ""
  szFarDirName = ""
  szTBTMainText = ""
  szTBTMainTextNext = ""
  nGoPosDist = 0
  nGoPosTime = 0
  vpPosPointLat = 0
  vpPosPointLon = 0
  nPosAngle = 0
  nPosSpeed = -1
  xPosValidCount = 0
  last_update_gps_time = last_calculate_gps_time = 0
  timeStamp = 0
  
  prev_recvTime = time.monotonic()
  #autoNaviSpeedCtrl = int(Params().get("AutoNaviSpeedCtrl"))
  #sockWaitTime = 1.0 if autoNaviSpeedCtrl == 3 else 0.2
  sockWaitTime = 0.2
  send_time = time.monotonic()


  bearing = 0.0
  bearing_offset = 0.0
  location_valid = False
  diff_angle_count = 0.0

  road_category_map = {
    0: "고속도로0",
    1: "고속도로1",
    2: "넓은도로2",
    3: "넓은도로3",
    4: "일반도로4",
    5: "일반도로5",
    6: "좁은도로6",
    7: "좁은도로7",
    8: "XX도로8"
    }
  # nTBTTurnType 값을 (navType, navModifier, xTurnInfo)로 매핑하는 사전
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

  with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    try:
      sock.bind(('0.0.0.0', Port.RECEIVE_PORT))
      print("AutoNaviSpeed != 3")

      sock.setblocking(False)

      while True:

        sm.update()
        showDebugUI = Params().get_bool("ShowDebugUI")
        ret = server.udp_recv(sock, sockWaitTime)

        if sm.updated['carState']:
          CS = sm['carState']
        if sm.updated['liveLocationKalman']:
          location = sm['liveLocationKalman']
          bearing = math.degrees(location.calibratedOrientationNED.value[2])
          if (location.status == log.LiveLocationKalman.Status.valid) and location.positionGeodetic.valid and location.gpsOK:            
            location_valid = True
            bearing_offset = 0.0
          else:
            location_valid = False

        #print(Port.RECEIVE_PORT)

        msg = messaging.new_message('roadLimitSpeed', valid=True)
        roadLimitSpeed = msg.roadLimitSpeed
        roadLimitSpeed.active = server.active
        #roadLimitSpeed.roadLimitSpeed = server.get_limit_val("road_limit_speed", 0)
        #roadLimitSpeed.isHighway = server.get_limit_val("is_highway", False)
        #roadLimitSpeed.camType = server.get_limit_val("cam_type", 0)
        #roadLimitSpeed.camLimitSpeedLeftDist = server.get_limit_val("cam_limit_speed_left_dist", 0)
        #roadLimitSpeed.camLimitSpeed = server.get_limit_val("cam_limit_speed", 0)
        #roadLimitSpeed.sectionLimitSpeed = server.get_limit_val("section_limit_speed", 0)
        #roadLimitSpeed.sectionLeftDist = server.get_limit_val("section_left_dist", 0)
        #roadLimitSpeed.sectionAvgSpeed = server.get_limit_val("section_avg_speed", 0)
        #roadLimitSpeed.sectionLeftTime = server.get_limit_val("section_left_time", 0)
        #roadLimitSpeed.sectionAdjustSpeed = server.get_limit_val("section_adjust_speed", False)
        #roadLimitSpeed.camSpeedFactor = server.get_limit_val("cam_speed_factor", CAMERA_SPEED_FACTOR)

        atype = server.get_apilot_val("type")
        value = server.get_apilot_val("value")
        atype = "none" if atype is None else atype
        value = "-1" if value is None else value
        
        #if atype != 'none':
        #  print(atype, value)

        try:
          value_int = clip(int(value), -10000, 2000000000)
        except ValueError:
          value_int = -100

        xCmd = server.get_apilot_val("apilot_cmd")
        xArg = server.get_apilot_val("apilot_arg")
        #xIndex = value_int

        now = time.monotonic()
        if ret:
          prev_recvTime = now

        delta_dist = 0.0
        if CS is not None:
          delta_dist = CS.totalDistance - totalDistance
          totalDistance = CS.totalDistance
          if CS.gasPressed:
            xBumpDistance = -1
            if xSignType == 124:
              xSignType = -1
        apm_valid = True
        if atype == 'none':
          apm_valid = False
        elif atype == 'opkrturninfo':
          mappyMode_valid = True
          xTurnInfo = value_int
        elif atype == 'opkrdistancetoturn':
          xDistToTurn = value_int
        elif atype == 'opkrspddist':
          xSpdDist = value_int
        elif atype == 'opkr-spddist':
          pass
        elif atype == 'opkrspdlimit':
          mappyMode_valid = True
          xSpdLimit = value_int
        elif atype == 'opkr-spdlimit':
          pass
        elif atype == 'opkrsigntype':
          xSignType = value_int
        elif atype == 'opkr-signtype':
          pass
        elif atype == 'opkrroadsigntype':
          xRoadSignType = value_int
        elif atype == 'opkrroadlimitspeed':
          xRoadLimitSpeed = value_int
        elif atype == 'opkrwazecurrentspd':
          pass
        elif atype == 'opkrwazeroadname':
          xRoadName = value
        elif atype == 'opkrwazenavsign':
          mappyMode_valid = True
          if value == '2131230983': # 목적지
            xTurnInfo = -1
          elif value == '2131230988': # turnLeft
            xTurnInfo = 1
          elif value == '2131230989': # turnRight
            xTurnInfo = 2
          elif value == '2131230985': 
            xTurnInfo = 4
          else:
            xTurnInfo = value_int
          xTurnInfo_prev = xTurnInfo
        elif atype == 'opkrwazenavdist':
          xDistToTurn = value_int
          if xTurnInfo<0:
            xTurnInfo = xTurnInfo_prev
        elif atype == 'opkrwazeroadspdlimit':
          mappyMode_valid = True
          xRoadLimitSpeed = value_int
        elif atype == 'opkrwazealertdist':
          pass
        elif atype == 'opkrwazereportid':
          pass
        elif atype == 'apilotman':
          server.active_apilot = 1
          xIndex = value_int
        else:
          print("unknown{}={}".format(atype, value))
        #roadLimitSpeed.xRoadName = apilot_val['opkrroadname']['value']

        #for 띠맵
        #if ret or now - prev_recvTime > 2.0: # 수신값이 있거나, 2.0초가 지난경우 데이터를 초기화함.
        if sdi_valid_count <= 0: #now - prev_recvTime > 2.0: # 2.0초가 지난경우 데이터를 초기화함.
          nTBTTurnType = nSdiType = nSdiSpeedLimit = nSdiPlusType = nSdiPlusSpeedLimit = nSdiBlockType = -1
          nSdiBlockSpeed = nRoadLimitSpeed = -1
          roadcate = 8
          nLaneCount = 0
          #print("Reset roadlimit...")

        nSdiDist -= delta_dist
        nSdiPlusDist -= delta_dist
        nSdiBlockDist -= delta_dist
        nTBTDist -= delta_dist
        nTBTDistNext -= delta_dist

        if xSpdLimit >= 0:
          xSpdDist -= delta_dist
          if xSpdDist < 0:
            xSpdLimit = -1

        if True: #xTurnInfo >= 0:
          xDistToTurn -= delta_dist
          if xDistToTurn < 0:
            xTurnInfo = -1

        #print("I:{:.1f},{:.1f},{:.1f},{:.2f}".format(nSdiDist, nSdiPlusDist, nTBTDist, delta_dist))

        #vpPosPointLat = vpPosPointLon = 0
        sdi_valid = False
        if ret:
          if int(server.get_apilot_val("nRoadLimitSpeed", -1)) != -1:
            sdi_valid = True
            nTBTTurnType = nSdiType = nSdiSpeedLimit = nSdiPlusType = nSdiPlusSpeedLimit = nSdiBlockType = -1
            nSdiBlockSpeed = nRoadLimitSpeed = -1
            nPosSpeed = -1

          nTBTTurnType = int(server.get_apilot_val("nTBTTurnType", nTBTTurnType))
          nTBTTurnTypeNext = int(server.get_apilot_val("nTBTTurnTypeNext", nTBTTurnTypeNext))
          nTBTNextRoadWidth = int(server.get_apilot_val("nTBTNextRoadWidth", nTBTNextRoadWidth))          
          szNearDirName = server.get_apilot_val("szNearDirName", szNearDirName)
          szFarDirName = server.get_apilot_val("szFarDirName", szFarDirName)
          szTBTMainText = server.get_apilot_val("szTBTMainText", szTBTMainText)
          szTBTMainTextNext = server.get_apilot_val("szTBTMainTextNext", szTBTMainTextNext)
          nSdiType = int(server.get_apilot_val("nSdiType", nSdiType))
          nSdiDist = float(server.get_apilot_val("nSdiDist", nSdiDist))
          nSdiSpeedLimit = int(server.get_apilot_val("nSdiSpeedLimit", nSdiSpeedLimit))
          nSdiPlusType = int(server.get_apilot_val("nSdiPlusType", nSdiPlusType))
          nSdiPlusDist = float(server.get_apilot_val("nSdiPlusDist", nSdiPlusDist))
          nSdiPlusSpeedLimit = int(server.get_apilot_val("nSdiPlusSpeedLimit", nSdiPlusSpeedLimit))
          nSdiBlockType = int(server.get_apilot_val("nSdiBlockType", nSdiBlockType))
          nSdiBlockSpeed = int(server.get_apilot_val("nSdiBlockSpeed", nSdiBlockSpeed))
          nSdiBlockDist = float(server.get_apilot_val("nSdiBlockDist", nSdiBlockDist))
          nTBTDist = float(server.get_apilot_val("nTBTDist", nTBTDist))
          nTBTDistNext = float(server.get_apilot_val("nTBTDistNext", nTBTDistNext))
          nRoadLimitSpeed = int(server.get_apilot_val("nRoadLimitSpeed", nRoadLimitSpeed))
          roadcate = int(server.get_apilot_val("roadcate", roadcate))
          nLaneCount = int(server.get_apilot_val("nLaneCount", nLaneCount))
          nGoPosDist = int(server.get_apilot_val("nGoPosDist", nGoPosDist))
          nGoPosTime = int(server.get_apilot_val("nGoPosTime", nGoPosTime))
          vpPosPointLat = float(server.get_apilot_val("vpPosPointLat", vpPosPointLat))
          vpPosPointLon = float(server.get_apilot_val("vpPosPointLon", vpPosPointLon))
          nPosAngle = float(server.get_apilot_val("nPosAngle", nPosAngle))
          nPosSpeed = float(server.get_apilot_val("nPosSpeed", nPosSpeed))
          timeStamp = int(server.get_apilot_val("timeStamp", 0))
          if nPosSpeed >= 0:
            xPosValidCount += 1
          #roadcate = 8 if nLaneCount == 0 else roadcate
          #print("roadcate=", roadcate)

        #print("O:{:.1f},{:.1f},{:.1f},{:.2f}".format(nSdiDist, nSdiPlusDist, nTBTDist, delta_dist))

        sdiDebugText = ":"
        if nSdiType >= 0:
          sdiDebugText += "S-{}/{}/{} ".format(nSdiType, int(nSdiDist), nSdiSpeedLimit)
        if nSdiPlusType >= 0:
          sdiDebugText += "P-{}/{}/{} ".format(nSdiPlusType, int(nSdiPlusDist), nSdiPlusSpeedLimit)
        if nSdiBlockType >= 0:
          sdiDebugText += "B-{}/{}/{} ".format(nSdiBlockType, int(nSdiBlockDist), nSdiBlockSpeed)
        if nTBTTurnType >= 0:
          sdiDebugText += "T-{}/{} ".format(nTBTTurnType, int(nTBTDist))
        #if ret:
        #  print(sdiDebugText)

        navType, navModifier, xTurnInfo1 = "invalid", "", -1

        # nTBTTurnType에 따른 설정
        if nTBTTurnType in turn_type_mapping:
          navType, navModifier, xTurnInfo_temp = turn_type_mapping[nTBTTurnType]
          xTurnInfo1 = xTurnInfo_temp if xTurnInfo_temp is not None else xTurnInfo1

        if xTurnInfo1 < 0 and nTBTTurnType >= 0 and not mappyMode_valid:
          xTurnInfo = -1
        else:
          xTurnInfo = xTurnInfo1

        navTypeNext, navModifierNext, xTurnInfoNext = "invalid", "", -1
        if nTBTTurnTypeNext in turn_type_mapping:
          navTypeNext, navModifierNext, xTurnInfoNext = turn_type_mapping[nTBTTurnTypeNext]


        if nTBTDist > 0 and xTurnInfo >= 0:
          xDistToTurn = nTBTDist
        #sdi_valid = True if nRoadLimitSpeed >= 0 or nTBTTurnType > 0 or nSdiType >= 0 else False
        if nRoadLimitSpeed > 0:
          if nRoadLimitSpeed >= 200:
            nRoadLimitSpeed = (nRoadLimitSpeed - 20) / 10
        else:
          nRoadLimitSpeed = 20
        xRoadLimitSpeed = nRoadLimitSpeed
        #sdiBlockType
        # 1: startOSEPS: 구간단속시작
        # 2: inOSEPS: 구간단속중
        # 3: endOSEPS: 구간단속종료
        #sdiType: 
        # 0: speedLimit, 1: speedLimitPos, 2:SpeedBlockStartPos, 3: SpeedBlockEndPos, 4:SpeedBlockMidPos, 
        # 5: Tail, 6: SignalAccidentPos, 7: SpeedLimitDangerous, 8:BoxSpeedLimit, 9: BusLane, 
        # 10:ChangerRoadPos, 11:RoadControlPos, 12: IntruderArea, 13: TrafficInfoCollectPos, 14:CctvArea
        # 15:OverloadDangerousArea, 16:LoadBadControlPos, 17:ParkingControlPos, 18:OnewayArea, 19:RailwayCrossing
        # 20:SchoolZoneStart, 21:SchoolZoneEnd, 22:SpeedBump, 23:LpgStation, 24:TunnelArea, 
        # 25:ServiceArea
        # 66:ChangableSpeedBlockStartPos, 67:ChangableSpeedBlockEndPos
        if nSdiType in [0,1,2,3,4,7,8] and nSdiSpeedLimit > 0: # SpeedLimitPos, nSdiSection: 2,
          xSpdLimit = nSdiSpeedLimit
          xSpdDist = nSdiDist
          sdiType = nSdiType
          #if nSdiBlockType in [1,2,3]: #구간단속
          if nSdiBlockType in [2,3]: #구간단속,
            sdiType = 4
            xSpdDist = nSdiBlockDist
          #if sdiType == 4: ## 구간단속
          #  xSpdDist = nSdiBlockDist if nSdiBlockDist > 0 else 80
          elif sdiType == 7: ##이동식카메라?
            xSpdLimit = xSpdDist = -1
        elif nSdiPlusType == 22 or nSdiType == 22: # SpeedBump
          xSpdLimit = 35
          xSpdDist = nSdiPlusDist if nSdiPlusType == 22 else nSdiDist
          sdiType = 22
        elif sdi_valid and nSdiSpeedLimit <= 0 and not mappyMode: # 데이터는 수신되었으나, sdi 수신이 없으면, 감속중 다른곳으로 빠진경우... 초기화...
          xSpdLimit = xSpdDist = sdiType = -1

        if sdiType >= 0:
          roadLimitSpeed.camType = sdiType

        szPosRoadName = server.get_apilot_val("szPosRoadName", "")
        if len(szPosRoadName) > 0:
          xRoadName = szPosRoadName

        sdi_valid_count -= 1
        if sdi_valid:
          sdi_valid_count = 50
        apm_valid_count -= 1
        if apm_valid:
          apm_valid_count = 10

        if xBumpDistance > 0:
          xBumpDistance -= delta_dist
          if xBumpDistance <= 0 and xSignType == 124:
            xSignType = -1
          else:
            roadLimitSpeed.camType = 22 # bump

        if xSignType == 124: ##사고방지턱
          if xBumpDistance <= 0:
            xBumpDistance = 110
        else:
          xBumpDistance = -1

        if sdi_valid_count > 0:
          roadLimitSpeed.active = 200 + server.active
          mappyMode = False
        elif apm_valid_count > 0 and mappyMode_valid:
          roadLimitSpeed.active = 200 + server.active
        elif apm_valid_count > 0:
          roadLimitSpeed.active = 100 + server.active
        else:
          xSpdDist = xBumpDistance = xSpdLimit = -1
          mappyMode_valid = False
        #print("active=", roadLimitSpeed.active)
        #print("turn={},{}".format(xTurnInfo, xDistToTurn))
        roadLimitSpeed.xTurnInfo = int(xTurnInfo)
        roadLimitSpeed.xDistToTurn = int(xDistToTurn)
        roadLimitSpeed.xTurnInfoNext = int(xTurnInfoNext)
        roadLimitSpeed.xDistToTurnNext = int(nTBTDistNext)
        roadLimitSpeed.xSpdDist = int(xSpdDist) if xBumpDistance <= 0 else int(xBumpDistance)
        roadLimitSpeed.xSpdLimit = int(xSpdLimit) if xBumpDistance <= 0 else 35 # 속도는 추후조절해야함. 일단 35
        roadLimitSpeed.xSignType = int(xSignType) if xBumpDistance <= 0 else 22
        roadLimitSpeed.xRoadSignType = int(xRoadSignType)
        roadLimitSpeed.xRoadLimitSpeed = int(xRoadLimitSpeed)
        if xRoadLimitSpeed > 0:
          roadLimitSpeed.roadLimitSpeed = int(xRoadLimitSpeed)
        roadLimitSpeed.xRoadName = xRoadName + "[{}]".format(int(xRoadLimitSpeed))
        if showDebugUI:
          roadLimitSpeed.xRoadName += ("[{}]".format(nTBTNextRoadWidth) + "[{}]".format(road_category_map.get(roadcate,"X")) + sdiDebugText)
        #print(roadLimitSpeed.xRoadName)

        roadLimitSpeed.xCmd = "" if xCmd is None else xCmd
        roadLimitSpeed.xArg = "" if xArg is None else xArg
        roadLimitSpeed.xIndex = xIndex
        roadLimitSpeed.roadcate = roadcate
        roadLimitSpeed.xNextRoadWidth = nTBTNextRoadWidth

        instruction = roadLimitSpeed.navInstruction
        instruction.distanceRemaining = nGoPosDist
        instruction.timeRemaining = nGoPosTime
        instruction.speedLimit = nRoadLimitSpeed / 3.6 if nRoadLimitSpeed > 0 else 0
        instruction.maneuverDistance = float(nTBTDist)
        instruction.maneuverSecondaryText = szNearDirName
        if len(szFarDirName):
          instruction.maneuverSecondaryText += "[{}]".format(szFarDirName)
        instruction.maneuverPrimaryText = szTBTMainText
        instruction.timeRemainingTypical = nGoPosTime

        instruction.maneuverType = navType
        instruction.maneuverModifier = navModifier

        maneuvers = []
        if nTBTTurnType >= 0:
          maneuver = {}
          maneuver['distance'] = float(nTBTDist)
          maneuver['type'] = navType
          maneuver['modifier'] = navModifier
          maneuvers.append(maneuver)
          if nTBTDistNext >= nTBTDist:
            maneuver = {}
            maneuver['distance'] = float(nTBTDistNext)
            maneuver['type'] = navTypeNext
            maneuver['modifier'] = navModifierNext
            maneuvers.append(maneuver)

        instruction.allManeuvers = maneuvers

        #print(instruction)

        xPosValidCount = max(0, xPosValidCount - 1)
        unix_now = time.mktime(datetime.now().timetuple())
        v_ego = CS.vEgo if CS is not None else float(nPosSpeed)/3.6
        if sdi_valid:
          if not location_valid and CS is not None:
            diff_angle = nPosAngle - bearing;
            while diff_angle < 0.0:
              diff_angle += 360
            diff_angle = (diff_angle + 180) % 360 - 180;
            if abs(diff_angle) > 20 and CS.vEgo > 1.0 and abs(CS.steeringAngleDeg) < 2.0:
              diff_angle_count += 1
            else:
              diff_angle_count = 0
            print("{:.1f} bearing_diff[{}] = {:.1f} = {:.1f} - {:.1f}, v={:.1f},st={:.1f}".format(bearing_offset, diff_angle_count, diff_angle, nPosAngle, bearing, CS.vEgo*3.6, CS.steeringAngleDeg))
            if diff_angle_count > 2:
              bearing_offset = nPosAngle - bearing
              print("bearing_offset = {:.1f} = {:.1f} - {:.1f}".format(bearing_offset, nPosAngle, bearing))
          xPosValidCount = 20
          #n초 통신 지연시간이 있다고 가정하고 좀더 진행한것으로 처리함.
          dt = 0 #(unix_now - timeStamp / 1000.) if timeStamp > 0 else 0.1
          dt += 0.2  #가상으로 0.5초만큼 더 진행한것으로 
          vpPosPointLat, vpPosPointLon = estimate_position(float(vpPosPointLat), float(vpPosPointLon), v_ego, bearing + bearing_offset, dt)
          last_update_gps_time = now
          last_calculate_gps_time = now
        elif now - last_update_gps_time < 3.0:# and CS is not None:
          dt = now - last_calculate_gps_time
          last_calculate_gps_time = now
          vpPosPointLat, vpPosPointLon = estimate_position(float(vpPosPointLat), float(vpPosPointLon), v_ego, bearing + bearing_offset, dt)
        roadLimitSpeed.xPosSpeed = float(nPosSpeed)
        roadLimitSpeed.xPosAngle = float(bearing + bearing_offset)
        roadLimitSpeed.xPosLat = float(vpPosPointLat)
        roadLimitSpeed.xPosLon = float(vpPosPointLon)
        roadLimitSpeed.xPosValidCount = xPosValidCount

        if sm.updated['naviData']:
          naviData = sm['naviData']
          naviData_update_count = 20
          camLimitSpeedLeftDist = naviData.camLimitSpeedLeftDist
          sectionLeftDist = naviData.sectionLeftDist
          #print(naviData)

        if naviData_update_count > 0:
          naviData_update_count -= 1
        else:
          naviData = None
        
        if naviData is not None:
          if naviData.active:
            roadLimitSpeed.roadLimitSpeed = naviData.roadLimitSpeed
            roadLimitSpeed.isHighway = naviData.isHighway
            roadLimitSpeed.camType = naviData.camType
            roadLimitSpeed.camLimitSpeedLeftDist = int(camLimitSpeedLeftDist)
            roadLimitSpeed.camLimitSpeed = naviData.camLimitSpeed
            roadLimitSpeed.sectionLimitSpeed = naviData.sectionLimitSpeed
            roadLimitSpeed.sectionLeftDist = int(sectionLeftDist)
            roadLimitSpeed.sectionAvgSpeed = naviData.sectionAvgSpeed
            roadLimitSpeed.sectionLeftTime = naviData.sectionLeftTime
            roadLimitSpeed.sectionAdjustSpeed = naviData.sectionAdjustSpeed
            roadLimitSpeed.camSpeedFactor = naviData.camSpeedFactor
          camLimitSpeedLeftDist -= delta_dist
          sectionLeftDist -= delta_dist

        pm.send('roadLimitSpeed', msg)

        if now - send_time > 1.0:
          server.send_sdp(sock)
          send_time = now
        server.check()
        #time.sleep(0.03)

    except Exception as e:
      stack_trace = traceback.format_exc()
      print(stack_trace) 
      print(e)
      server.last_exception = e
      Params().put_bool("CarrotException", True)


if __name__ == "__main__":
  main()
