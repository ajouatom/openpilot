import numpy as np
import time
import threading
import zmq
import os
import subprocess
import json
from datetime import datetime
import socket
import select
import fcntl
import struct
import math

from ftplib import FTP
from openpilot.common.realtime import Ratekeeper
from openpilot.common.params import Params
import cereal.messaging as messaging
from cereal import log
import openpilot.selfdrive.frogpilot.fleetmanager.helpers as fleet

NetworkType = log.DeviceState.NetworkType

class CarrotMan:
  def __init__(self):
    self.params = Params()
    self.params_memory = Params("/dev/shm/params")
    self.carrot_serv = CarrotServ()
    
    self.show_panda_debug = False
    self.broadcast_ip = self.get_broadcast_address()
    self.broadcast_port = 5555
    self.carrot_man_port = 7713

    self.carrot_zmq_thread = threading.Thread(target=self.carrot_cmd_zmq, args=[])
    self.carrot_zmq_thread.daemon = True
    self.carrot_zmq_thread.start()

    self.carrot_panda_debug_thread = threading.Thread(target=self.carrot_panda_debug, args=[])
    self.carrot_panda_debug_thread.daemon = True
    self.carrot_panda_debug_thread.start()

    self.is_running = True
    threading.Thread(target=self.broadcast_version_info).start()

    self.sm = messaging.SubMaster(['deviceState', 'carState', 'controlsState', 'longitudinalPlan'])
    self.ip_address = "0.0.0.0"
    self.remote_addr = None

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
    
  # 브로드캐스트 메시지 전송
  def broadcast_version_info(self):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    #version_info = {"version": "1.0", "ip": socket.gethostbyname(socket.gethostname())}
    frame = 0
    isOnroadCount = 0
    is_tmux_sent = False
    CS = None
    self.save_toggle_values()
    rk = Ratekeeper(10, print_delay_threshold=None)
 
    while self.is_running:
      if frame % 20 == 0 or self.remote_addr is not None:
        try:
          self.broadcast_ip = self.get_broadcast_address()
          self.ip_address = socket.gethostbyname(socket.gethostname())
          #self.params_memory.put_nonblocking("NetworkAddress", self.ip_address)

          msg = self.broadcast_message()
          if self.remote_addr is None:
            sock.sendto(msg.encode('utf-8'), (self.broadcast_ip, self.broadcast_port))
            print(f"Broadcasting: {self.broadcast_ip}:{msg}")
          else:
            sock.sendto(msg.encode('utf-8'), (self.remote_addr[0], self.broadcast_port))
            #print(f"Broadcasting: {self.remote_addr}:{msg}")
            
        except Exception as e:
          print(f"##### broadcast_error...: {e}")

      self.sm.update(0)
      if self.sm.updated['carState']:
        CS = self.sm['carState']

      self.carrot_serv.update_navi(CS, self.sm)
      isOnroadCount = isOnroadCount + 1 if self.params.get_bool("IsOnroad") else 0
      if isOnroadCount == 0:
        is_tmux_sent = False
      if isOnroadCount == 1:
        self.show_panda_debug = True

      network_type = self.sm['deviceState'].networkType# if not force_wifi else NetworkType.wifi
      networkConnected = False if network_type == NetworkType.none else True

      if isOnroadCount == 500:
        self.make_tmux_data()
      if isOnroadCount > 500 and not is_tmux_sent and networkConnected:
        self.send_tmux("Ekdrmsvkdlffjt7710", "onroad", send_settings = True)
        is_tmux_sent = True
      if self.params.get_bool("CarrotException") and networkConnected:
        self.params.put_bool("CarrotException", False)
        self.make_tmux_data()
        self.send_tmux("Ekdrmsvkdlffjt7710", "exception")       
      
      rk.keep_time()
      frame += 1

  def broadcast_message(self):
    msg = {}
    msg['Carrot'] = self.params.get("Version").decode('utf-8')
    isOnroad = self.params.get_bool("IsOnroad")
    msg['IsOnroad'] = isOnroad
    msg['CarrotRouteActive'] = self.params.get_bool("CarrotRouteActive")
    msg['ip'] = self.ip_address
    if not isOnroad:
      self.controls_active = False
      self.xState = 0
      self.trafficState = 0
    else:
      if self.sm.updated['carState']:
        pass
      if self.sm.updated['controlsState']:
        controls = self.sm['controlsState']
        self.controls_active = controls.active
      if self.sm.updated['longitudinalPlan']:
        lp = self.sm['longitudinalPlan']
        self.xState = lp.xState
        self.trafficState = lp.trafficState
        
    msg['active'] = self.controls_active
    msg['xState'] = self.xState
    msg['trafficState'] = self.trafficState
    return json.dumps(msg)

  def receive_fixed_length_data(self, sock, length):
    buffer = b""
    while len(buffer) < length:
      data = sock.recv(length - len(buffer))
      if not data:
        raise ConnectionError("Connection closed before receiving all data")
      buffer += data
    return buffer

  def carrot_man_thread(self):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
      sock.bind(('0.0.0.0', self.carrot_man_port))
      sock.listen(1)
      print("#########carrot_man_thread: thread started...")

      while True:
        connection = None
        try:
          # accept에는 타임아웃이 설정되지 않음
          connection, self.remote_addr = sock.accept()
          print(self.remote_addr)

          # accept 후, recv에 대한 타임아웃을 설정
          connection.settimeout(2)  # recv 타임아웃 설정 (2초)
        
          while True:
            try:
              length_data = connection.recv(4)  # recv 타임아웃이 적용됨
              if not length_data:
                raise ConnectionError("Connection closed")
              try:
                data_length = int(length_data.decode('utf-8'))
              except ValueError:
                raise ConnectionError("Received invalid data length")

              data = self.receive_fixed_length_data(connection, data_length)
              json_obj = json.loads(data.decode())
              self.carrot_serv.update(json_obj)

            except socket.timeout:
              print("Waiting for data (timeout)...")
              time.sleep(1)
              break

            except Exception as e:
              print(f"carrot_man_thread: error...: {e}")
              break

        except Exception as e:
          print(f"carrot_man_thread: error...: {e}")
      
        finally:
          if connection:
            connection.close()
          self.remote_addr = None

        time.sleep(1)

      
  def make_tmux_data(self):
    try:
      result = subprocess.run("rm /data/media/tmux.log; tmux capture-pane -pq -S-1000 > /data/media/tmux.log", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=False)
      result = subprocess.run("/data/openpilot/selfdrive/apilot.py", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=False)
    except Exception as e:
      print("TMUX creation error")
      return

  def send_tmux(self, ftp_password, tmux_why, send_settings=False):

    ftp_server = "shind0.synology.me"
    ftp_port = 8021
    ftp_username = "carrotpilot"
    ftp = FTP()
    ftp.connect(ftp_server, ftp_port)
    ftp.login(ftp_username, ftp_password)
    car_selected = Params().get("CarName")
    if car_selected is None:
      car_selected = "none"
    else:
      car_selected = car_selected.decode('utf-8')

    directory = car_selected + " " + Params().get("DongleId").decode('utf-8')
    current_time = datetime.now().strftime("%Y%m%d-%H%M%S")
    filename = tmux_why + "-" + current_time + ".txt"

    try:
      ftp.mkd(directory)
    except Exception as e:
      print(f"Directory creation failed: {e}")
    ftp.cwd(directory)

    try:
      with open("/data/media/tmux.log", "rb") as file:
        ftp.storbinary(f'STOR {filename}', file)
    except Exception as e:
      print(f"ftp sending error...: {e}")

    if send_settings:
      self.save_toggle_values()
      try:
        #with open("/data/backup_params.json", "rb") as file:
        with open("/data/toggle_values.json", "rb") as file:
          ftp.storbinary(f'STOR toggles-{current_time}.json', file)
      except Exception as e:
        print(f"ftp params sending error...: {e}")

    ftp.quit()

  def carrot_panda_debug(self):
    #time.sleep(2)
    while True:
      if self.show_panda_debug:
        self.show_panda_debug = False
        try:
          result = subprocess.run("/data/openpilot/selfdrive/debug/debug_console_carrot.py", shell=True)
        except Exception as e:
          print("debug_console error")
          time.sleep(2)
      else:
        time.sleep(1)

  def save_toggle_values(self):
    toggle_values = fleet.get_all_toggle_values()
    file_path = os.path.join('/data', 'toggle_values.json')
    with open(file_path, 'w') as file:
      json.dump(toggle_values, file, indent=2) 

  def carrot_cmd_zmq(self):

    context = zmq.Context()
    socket = None

    print("#########carrot_cmd_zmq: thread started...")
    ip_address = "0.0.0.0"
    while True:
      try:
        if ip_address != self.ip_address:
          ip_address = self.ip_address
          if socket:
            socket.close()
          socket = context.socket(zmq.REP)
          socket.bind("tcp://*:7710")
          
        message = socket.recv()
        print(f"Received:7710 request: {message}")
        json_obj = json.loads(message.decode())
        if 'echo_cmd' in json_obj:
          try:
            result = subprocess.run(json_obj['echo_cmd'], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=False)
            try:
              stdout = result.stdout.decode('utf-8')
            except UnicodeDecodeError:
              stdout = result.stdout.decode('euc-kr', 'ignore')
                
            echo = json.dumps({"echo_cmd": json_obj['echo_cmd'], "result": stdout})
          except Exception as e:
            echo = json.dumps({"echo_cmd": json_obj['echo_cmd'], "result": f"exception error: {str(e)}"})
          #print(echo)
          socket.send(echo.encode())
        elif 'tmux_send' in json_obj:
          self.make_tmux_data()
          self.send_tmux(json_obj['tmux_send'], "tmux_send")
          echo = json.dumps({"tmux_send": json_obj['tmux_send'], "result": "success"})
          socket.send(echo.encode())
      except Exception as e:
        print("carrot_cmd_zmq error: {str{e}}")
        time.sleep(1)


class CarrotServ:
  def __init__(self):
    self.params_memory = Params("/dev/shm/params")
    
    self.nRoadLimitSpeed = 30

    self.active = False
    self.active_count = 0
    
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
    self.vpPosPointLat = 0.0
    self.vpPosPointLon = 0.0

    self.totalDistance = 0
    self.xSpdLimit = 0
    self.xSpdDist = 0
    self.xSpdType = -1

    self.xTurnInfo = -1
    self.xDistToTurn = 0

    self.safeFactor = 1.07

    self.navType, self.navModifier = "invalid", ""
    self.navTypeNext, self.navModifierNext = "invalid", ""

    self.carrotIndex = 0
    self.carrotCmdIndex = 0
    self.carrotCmd = ""
    self.carrotArg = ""

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
      201: ("arrive", "straight", 7),
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
      self.navType, self.navModifier, xTurnInfo_temp = turn_type_mapping[self.nTBTTurnType]
      xTurnInfo1 = xTurnInfo_temp if xTurnInfo_temp is not None else xTurnInfo1
    else:
      self.navType, self.navModifier, xTurnInfo1 = "invalid", "", -1

    if xTurnInfo1 < 0 and self.nTBTTurnType >= 0: #and not mappyMode_valid:
      self.xTurnInfo = -1
    else:
      self.xTurnInfo = xTurnInfo1

    if self.nTBTTurnTypeNext in turn_type_mapping:
      self.navTypeNext, self.navModifierNext, self.xTurnInfoNext = turn_type_mapping[self.nTBTTurnTypeNext]
    else:
      self.navTypeNext, self.navModifierNext, self.xTurnInfoNext = "invalid", "", -1


    if self.nTBTDist > 0 and self.xTurnInfo >= 0:
      self.xDistToTurn = self.nTBTDist
  
  def _update_sdi(self):
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
    if self.nSdiType in [0,1,2,3,4,7,8] and self.nSdiSpeedLimit > 0:
      self.xSpdLimit = self.nSdiSpeedLimit
      self.xSpdDist = self.nSdiDist
      self.xSpdType = self.nSdiType
      if self.nSdiBlockType in [2,3]:
        self.xSpdDist = self.nSdiBlockDist
        self.xSpdType = 4
      elif self.nSdiType == 7: #이동식카메라
        self.xSpdLimit = self.xSpdDist = 0
    if self.nSdiPlusType == 22 or self.nSdiType == 22: # speed bump
      self.xSpdLimit = 35
      self.xSpdDist = self.nSdiPlusDist if self.nSdiPlusType == 22 else self.nSdiDist
      self.xSpdType = 22
    pass

  def update_auto_turn(self, CS, sm):
    turn_speed = 20
    stop_speed = 5
    turn_dist = 50
    fork_dist = 60
    stop_dist = 1
    turn_info_mapping = {
        1: {"type": "turn left", "speed": turn_speed, "dist": turn_dist},
        2: {"type": "turn right", "speed": turn_speed, "dist": turn_dist},
        5: {"type": "straight", "speed": turn_speed, "dist": turn_dist},
        3: {"type": "fork left", "speed": int(self.nRoadLimitSpeed * 0.5), "dist": fork_dist},
        4: {"type": "fork right", "speed": int(self.nRoadLimitSpeed * 0.5), "dist": fork_dist},
        43: {"type": "fork right", "speed": int(self.nRoadLimitSpeed * 0.5), "dist": fork_dist},
        7: {"type": "straight", "speed": stop_speed, "dist": stop_dist},
    }

    default_mapping = {"type": "none", "speed": 0, "dist": 0}

    ## turn dist를 초과하면 lane_change(fork)로 변경
    if self.xTurnInfo in [1,2] and self.xDistToTurn > turn_dist:
      self.xTurnInfo += 2 # 1 -> 3, 2 -> 4

    mapping = turn_info_mapping.get(self.xTurnInfo, default_mapping)

    self.tbtType = mapping["type"]
    self.tbtSpeed = mapping["speed"]
    self.tbtDist = mapping["dist"]

    #nav_speed_down = True if nav_turn or self.xTurnInfo in [5, 6] else False
    #nav_direction = 1 if self.xTurnInfo in [1, 3] else 2 if self.xTurnInfo in [2, 4, 43] else 0   #nav_direction:  1: left, 2:right, 0: straight

  def update_navi(self, CS, sm):
    delta_dist = 0.0
    if CS is not None:
      delta_dist = CS.totalDistance - self.totalDistance
      self.totalDistance = CS.totalDistance
      v_ego = CS.vEgo
    else:
      v_ego = 0.0

    self.xSpdDist = max(self.xSpdDist - delta_dist, 0)
    self.xDistToTurn = max(self.xDistToTurn - delta_dist, 0)
    self.active_count = max(self.active_count - 1, 0)
    self.active = True if self.active_count > 0 else False

    if self.xSpdType < 0:
      self.xSpdDist = 0
    if self.xTurnInfo < 0:
      self.xDistToTurn = 0

    speed = 250
    ### 과속카메라, 사고방지턱
    if self.xSpdDist > 0 and self.active:
      safe_sec = 1 if self.xSpdType == 22 else 6
      decel = 1.2
      speed = min(speed, self.calculate_current_speed(self.xSpdDist, self.xSpdLimit * self.safeFactor, safe_sec, decel))
      if self.xSpdType == 4:
        speed = self.xSpdLimit
    left_spd_sec = 100
    if self.xSpdDist > 0:
      left_spd_sec = int(max(self.xSpdDist - v_ego, 1) / max(1, v_ego))

    ### TBT 속도제어
    self.update_auto_turn(CS, sm)
    if self.xDistToTurn > 0 and self.active:
      pass
    left_tbt_sec = 100
    if self.xDistToTurn > 0:
      left_tbt_sec = int(max(self.xDistToTurn - v_ego, 1) / max(1, v_ego))

    data = {
      "active" : self.active,
      "xSpdType" : self.xSpdType,
      "xSpdLimit": self.xSpdLimit,
      "xSpdDist" : self.xSpdDist,
      "xSpdCountDown" : left_spd_sec,
      "xTurnInfo" : self.xTurnInfo,
      "xDistToTurn" : self.xDistToTurn,
      "xTurnCountDown" : left_tbt_sec,
      "nRoadLimitSpeed" : self.nRoadLimitSpeed,
      "szPosRoadName" : self.szPosRoadName,
      "szTBTMainText" : self.szTBTMainText,
      "desiredSpeed" : int(speed),
      "carrotCmdIndex" : int(self.carrotCmdIndex),
      "carrotCmd" : self.carrotCmd,
      "carrotArg" : self.carrotArg,
      }
    try:
      self.params_memory.put_nonblocking("CarrotNavi", json.dumps(data))
    except Exception as e:
      print(f" error...: {e}")      
    
  def update(self, json):
    if json == None:
      return
    if "carrotIndex" in json:
      self.carrotIndex = int(json.get("carrotIndex"))

    if "carrotCmd" in json:
      print(json.get("carrotCmd"), json.get("carrotArg"))
      self.carrotCmdIndex = self.carrotIndex
      self.carrotCmd = json.get("carrotCmd")
      self.carrotArg = json.get("carrotArg")
      

    if "goalPosX" in json:      
      self.goalPosX = float(json.get("goalPosX", self.goalPosX))
      self.goalPosY = float(json.get("goalPosY", self.goalPosY))
      self.szGoalName = json.get("szGoalName", self.szGoalName)
    elif "nRoadLimitSpeed" in json:
      self.active_count = 20
      ### roadLimitSpeed
      nRoadLimitSpeed = int(json.get("nRoadLimitSpeed", 20))
      if nRoadLimitSpeed > 0:
        if nRoadLimitSpeed > 200:
          nRoadLimitSpeed = (nRoadLimitSpeed - 20) / 10
      else:
        nRoadLimitSpeed = 20
      self.nRoadLimitSpeed = nRoadLimitSpeed

      ### SDI
      sdi_count = int(json.get("sdiCount", -1))
      if sdi_count == 0:  ##수신되었으나, sdi데이터가 없음.
        self.nSdiType = -1
        self.nSdiBlockType = -1
      elif sdi_count > 0:
        sdi_info_list = json.get("sdiInfo", [])
        if isinstance(sdi_info_list, list) and sdi_info_list:
          sdi_info = sdi_info_list[0]
          self.nSdiType = int(sdi_info.get("nSdiType", self.nSdiType))
          self.nSdiSpeedLimit = int(sdi_info.get("nSdiSpeedLimit", self.nSdiSpeedLimit))
          self.nSdiSection = int(sdi_info.get("nSdiSection", self.nSdiSection))
          self.nSdiDist = int(sdi_info.get("nSdiDist", self.nSdiDist))
          self.nSdiBlockType = int(sdi_info.get("nSdiBlockType", self.nSdiBlockType))
          self.nSdiBlockSpeed = int(sdi_info.get("nSdiBlockSpeed", self.nSdiBlockSpeed))
          self.nSdiBlockDist = int(sdi_info.get("nSdiBlockDist", self.nSdiBlockDist))

      self.nSdiPlusType = int(json.get("nSdiPlusType", self.nSdiPlusType))
      self.nSdiPlusSpeedLimit = int(json.get("nSdiPlusSpeedLimit", self.nSdiPlusSpeedLimit))
      self.nSdiPlusDist = int(json.get("nSdiPlusDist", self.nSdiPlusDist))
      self.nSdiPlusBlockType = int(json.get("nSdiPlusBlockType", self.nSdiPlusBlockType))
      self.nSdiPlusBlockSpeed = int(json.get("nSdiPlusBlockSpeed", self.nSdiPlusBlockSpeed))
      self.nSdiPlusBlockDist = int(json.get("nSdiPlusBlockDist", self.nSdiPlusBlockDist))

      print(f"sdi = {self.nSdiType}, {self.nSdiSpeedLimit}, {self.nSdiPlusType}")

      ## GuidePoint
      if "stGuidePoint" in json:      
        guide_point = json.get("stGuidePoint")
        self.nTBTDist = int(guide_point.get("nTBTDist", self.nTBTDist))
        self.nTBTTurnType = int(guide_point.get("nTBTTurnType", self.nTBTTurnType))
        self.szTBTMainText = guide_point.get("szTBTMainText", self.szTBTMainText)
        self.szNearDirName = guide_point.get("szNearDirName", self.szNearDirName)
        self.szFarDirName = guide_point.get("szFarDirName", self.szFarDirName)
        self.nTBTNextRoadWidth = int(guide_point.get("nTBTNextRoadWidth", self.nTBTNextRoadWidth))
      if "stGuidePointNext" in json:
        guide_point = json.get("stGuidePointNext")
        self.nTBTDistNext = int(guide_point.get("nTBTDist", self.nTBTDistNext))
        self.nTBTTurnTypeNext = int(guide_point.get("nTBTTurnType", self.nTBTTurnTypeNext))
        self.szTBTMainTextNext = guide_point.get("szTBTMainText", self.szTBTMainTextNext)

      self.nGoPosDist = int(json.get("nGoPosDist", self.nGoPosDist))
      self.nGoPosTime = int(json.get("nGoPosTime", self.nGoPosTime))
      self.szPosRoadName = json.get("szPosRoadName", "NoRoadName")

      self.vpPosPointLat = float(json.get("vpPosPointLat", self.vpPosPointLat))
      self.vpPosPointLon = float(json.get("vpPosPointLon", self.vpPosPointLon))
      self._update_tbt()
      self._update_sdi()
    else:
      #print(json)
      pass
    

import traceback

def main():
  print("CarrotManager Started")
  #print("Carrot GitBranch = {}, {}".format(Params().get("GitBranch"), Params().get("GitCommitDate")))
  carrot_man = CarrotMan()  
  while True:
    try:
      carrot_man.carrot_man_thread()
    except Exception as e:
      print(f"carrot_man error...: {e}")
      traceback.print_exc()
      time.sleep(10)


if __name__ == "__main__":
  main()
