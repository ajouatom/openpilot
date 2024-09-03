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
from common.numpy_fast import clip, interp
#import openpilot.selfdrive.frogpilot.fleetmanager.helpers as fleet
from common.filter_simple import StreamingMovingAverage

NetworkType = log.DeviceState.NetworkType

class CarrotMan:
  def __init__(self):
    self.params = Params()
    self.params_memory = Params("/dev/shm/params")
    self.carrot_serv = CarrotServ()
    
    self.show_panda_debug = False
    self.broadcast_ip = self.get_broadcast_address()
    self.broadcast_port = 7705
    self.carrot_man_port = 7706
    self.connection = None

    self.ip_address = "0.0.0.0"
    self.remote_addr = None

    self.turn_speed_last = 250
    self.curvatureFilter = StreamingMovingAverage(20)
    self.carrot_curve_speed_params()

    self.carrot_zmq_thread = threading.Thread(target=self.carrot_cmd_zmq, args=[])
    self.carrot_zmq_thread.daemon = True
    self.carrot_zmq_thread.start()

    self.carrot_panda_debug_thread = threading.Thread(target=self.carrot_panda_debug, args=[])
    self.carrot_panda_debug_thread.daemon = True
    self.carrot_panda_debug_thread.start()

    self.is_running = True
    threading.Thread(target=self.broadcast_version_info).start()

    self.sm = messaging.SubMaster(['deviceState', 'carState', 'controlsState', 'longitudinalPlan', 'modelV2', 'selfdriveState'])
    self.pm = messaging.PubMaster(['carrotMan'])

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
    frame = 0
    self.save_toggle_values()
    rk = Ratekeeper(10, print_delay_threshold=None)

    carrotIndex_last = self.carrot_serv.carrotIndex
    while self.is_running:
      try:
        self.sm.update(0)
        remote_addr = self.remote_addr
        remote_ip = remote_addr[0] if remote_addr is not None else ""
        vturn_speed = self.carrot_curve_speed(self.sm)
        self.carrot_serv.update_navi(remote_ip, self.sm, self.pm, vturn_speed)

        if frame % 20 == 0 or remote_addr is not None:
          try:
            self.broadcast_ip = self.get_broadcast_address() if remote_addr is None else remote_addr[0]
            ip_address = socket.gethostbyname(socket.gethostname())
            if ip_address != self.ip_address:
              self.ip_address = ip_address
              self.remote_addr = None
            self.params_memory.put_nonblocking("NetworkAddress", self.ip_address)

            msg = self.make_send_message()
            if self.broadcast_ip is not None:
              dat = msg.encode('utf-8')            
              sock.sendto(dat, (self.broadcast_ip, self.broadcast_port))
            #for i in range(1, 255):
            #  ip_tuple = socket.inet_aton(self.broadcast_ip)
            #  new_ip = ip_tuple[:-1] + bytes([i])
            #  address = (socket.inet_ntoa(new_ip), self.broadcast_port)
            #  sock.sendto(dat, address)

            if remote_addr is None:
              print(f"Broadcasting: {self.broadcast_ip}:{msg}")
            
          except Exception as e:
            if self.connection:
              self.connection.close()
            self.connection = None
            print(f"##### broadcast_error...: {e}")
            traceback.print_exc()
    
        rk.keep_time()
        frame += 1
      except Exception as e:
        print(f"broadcast_version_info error...: {e}")
        traceback.print_exc()
        time.sleep(1)

  def make_send_message(self):
    msg = {}
    msg['Carrot'] = self.params.get("Version").decode('utf-8')
    isOnroad = self.params.get_bool("IsOnroad")
    msg['IsOnroad'] = isOnroad
    msg['CarrotRouteActive'] = False #self.params.get_bool("CarrotRouteActive")
    msg['ip'] = self.ip_address
    msg['port'] = self.carrot_man_port
    self.controls_active = False
    self.xState = 0
    self.trafficState = 0
    if not isOnroad:
      self.xState = 0
      self.trafficState = 0
    else:
      if self.sm.alive['carState']:
        pass
      if self.sm.alive['selfdriveState']:
        selfdrive = self.sm['selfdriveState']
        self.controls_active = selfdrive.active
      if self.sm.alive['longitudinalPlan']:
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
    while True:
      try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
          sock.settimeout(10)  # 소켓 타임아웃 설정 (10초)
          sock.bind(('0.0.0.0', self.carrot_man_port))  # UDP 포트 바인딩
          print("#########carrot_man_thread: UDP thread started...")

          while True:
            try:
              #self.remote_addr = None
              # 데이터 수신 (UDP는 recvfrom 사용)
              try:
                data, remote_addr = sock.recvfrom(4096)  # 최대 4096 바이트 수신
                #print(f"Received data from {self.remote_addr}")
              
                if not data:
                  raise ConnectionError("No data received")

                if self.remote_addr is None:
                  print("Connected to: ", remote_addr)
                self.remote_addr = remote_addr
                try:
                  json_obj = json.loads(data.decode())
                  self.carrot_serv.update(json_obj)
                except Exception as e:
                  print(f"carrot_man_thread: json error...: {e}")
                  print(data)

                # 응답 메시지 생성 및 송신 (UDP는 sendto 사용)
                #try:
                #  msg = self.make_send_message()
                #  sock.sendto(msg.encode('utf-8'), self.remote_addr)
                #except Exception as e:
                #  print(f"carrot_man_thread: send error...: {e}")

              except socket.timeout:
                print("Waiting for data (timeout)...")
                self.remote_addr = None
                time.sleep(1)

              except Exception as e:
                print(f"carrot_man_thread: error...: {e}")
                self.remote_addr = None
                break

            except Exception as e:
              print(f"carrot_man_thread: recv error...: {e}")
              self.remote_addr = None
              break

          time.sleep(1)
      except Exception as e:
        self.remote_addr = None
        print(f"Network error, retrying...: {e}")
        time.sleep(2)

  def carrot_man_thread_tcipip(self):
    while True:
      try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
          sock.settimeout(10)  # 소켓 타임아웃 설정 (10초)
          sock.bind(('0.0.0.0', self.carrot_man_port))
          sock.listen(5)
          print("#########carrot_man_thread: thread started...")

          while True:
            self.connection = None
            self.remote_addr = None
            try:
              self.connection, self.remote_addr = sock.accept()
              print(self.remote_addr)

              self.connection.settimeout(10)
            
              while self.remote_addr is not None:
                try:
                  length_data = self.connection.recv(4)
                  if not length_data:
                    raise ConnectionError("Connection closed")
                  try:
                    data_length = int(length_data.decode('utf-8'))
                  except ValueError:
                    raise ConnectionError("Received invalid data length")

                  data = self.receive_fixed_length_data(self.connection, data_length)

                  try:
                    msg = self.make_send_message()
                    length = len(msg)
                    message = f"{length:04d}" + msg
                    self.connection.send(message.encode('utf-8'))
                  except Exception as e:
                    print(f"carrot_man_thread: send error...: {e}")                    

                  try:
                    json_obj = json.loads(data.decode())
                    self.carrot_serv.update(json_obj)
                  except Exception as e:
                    print(f"carrot_man_thread: json error...: {e}")
                    print(data)

                except socket.timeout:
                  print("Waiting for data (timeout)...")
                  time.sleep(1)
                  break

                except Exception as e:
                  print(f"carrot_man_thread: error...: {e}")
                  break

            except Exception as e:
              print(f"carrot_man_thread: accept error...: {e}")
          
            finally:
              if self.connection:
                self.connection.close()              

            time.sleep(1)
      except Exception as e:
        print(f"Network error, retrying...: {e}")
        time.sleep(5)  # 네트워크 오류 발생 시 5초 후 재시도
      
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
    #toggle_values = fleet.get_all_toggle_values()
    #file_path = os.path.join('/data', 'toggle_values.json')
    #with open(file_path, 'w') as file:
    #  json.dump(toggle_values, file, indent=2) 
    pass

  def carrot_cmd_zmq(self):

    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:7710")

    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)

    isOnroadCount = 0
    is_tmux_sent = False

    print("#########carrot_cmd_zmq: thread started...")
    while True:
      try:
        socks = dict(poller.poll(100))

        if socket in socks and socks[socket] == zmq.POLLIN:
          message = socket.recv(zmq.NOBLOCK)
          #print(f"Received:7710 request: {message}")
          json_obj = json.loads(message.decode())
        else:
          json_obj = None
          
        if json_obj == None:
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
        elif 'echo_cmd' in json_obj:
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
        print(f"carrot_cmd_zmq error: {e}")
        time.sleep(1)

  def carrot_curve_speed_params(self):
    self.autoCurveSpeedLowerLimit = int(self.params.get("AutoCurveSpeedLowerLimit"))
    self.autoCurveSpeedFactor = self.params.get_int("AutoCurveSpeedFactor")*0.01
    self.autoCurveSpeedAggressiveness = self.params.get_int("AutoCurveSpeedAggressiveness")*0.01
    self.autoCurveSpeedFactorIn = self.autoCurveSpeedAggressiveness - 1.0
   
  def carrot_curve_speed(self, sm):
    self.carrot_curve_speed_params()  
    ## 국가법령정보센터: 도로설계기준
    V_CURVE_LOOKUP_BP = [0., 1./800., 1./670., 1./560., 1./440., 1./360., 1./265., 1./190., 1./135., 1./85., 1./55., 1./30., 1./15.]
    #V_CRUVE_LOOKUP_VALS = [300, 150, 120, 110, 100, 90, 80, 70, 60, 50, 40, 30, 20]
    V_CRUVE_LOOKUP_VALS = [300, 150, 120, 110, 100, 90, 80, 70, 60, 50, 45, 35, 30]

    if not sm.alive['carState'] and not sm.alive['modelV2']:
        return 250
    #print(len(sm['modelV2'].orientationRate.z))
    if len(sm['modelV2'].orientationRate.z) == 0:
        return 250

    v_ego = sm['carState'].vEgo
    # 회전속도를 선속도 나누면 : 곡률이 됨. [12:20]은 약 1.4~3.5초 앞의 곡률을 계산함.
    orientationRates = np.array(sm['modelV2'].orientationRate.z, dtype=np.float32)
    speed = min(self.turn_speed_last / 3.6, clip(v_ego, 0.5, 100.0))
    
    # 절대값이 가장 큰 요소의 인덱스를 찾습니다.
    max_index = np.argmax(np.abs(orientationRates[12:20]))
    # 해당 인덱스의 실제 값을 가져옵니다.
    max_orientation_rate = orientationRates[12 + max_index]
    # 부호를 포함한 curvature를 계산합니다.
    curvature = max_orientation_rate / speed

    curvature = self.curvatureFilter.process(curvature) * self.autoCurveSpeedFactor
    turn_speed = 250

    if abs(curvature) > 0.0001:
        # 곡률의 절대값을 사용하여 속도를 계산합니다.
        base_speed = interp(abs(curvature), V_CURVE_LOOKUP_BP, V_CRUVE_LOOKUP_VALS)
        base_speed = clip(base_speed, self.autoCurveSpeedLowerLimit, 255)
        # 곡률의 부호를 적용하여 turn_speed의 부호를 결정합니다.
        turn_speed = np.sign(curvature) * base_speed

    self.turn_speed_last = abs(turn_speed)
    speed_diff = max(0, v_ego * 3.6 - abs(turn_speed))
    turn_speed = turn_speed - np.sign(curvature) * speed_diff * self.autoCurveSpeedFactorIn
    #controls.debugText2 = 'CURVE={:5.1f},curvature={:5.4f},mode={:3.1f}'.format(self.turnSpeed_prev, curvature, self.drivingModeIndex)
    return turn_speed

import collections
class CarrotServ:
  def __init__(self):
    self.params = Params()
    self.params_memory = Params("/dev/shm/params")
    
    self.nRoadLimitSpeed = 30

    self.active = 0     ## 1: CarrotMan Active, 2: sdi active , 3: speed decel active, 4: section active
    self.active_count = 0
    self.active_sdi_count = 0
    self.active_sdi_count_max = 80
    
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
    self.roadcate = 0

    self.nPosSpeed = 0.0
    self.nPosAngle = 0.0
    
    self.diff_angle_count = 0
    self.last_update_gps_time = 0
    self.last_calculate_gps_time = 0
    self.bearing_offset = 0.0
    
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

    self.atc_paused = False

    self.debugText = ""


  def update_params(self):
    self.autoNaviSpeedBumpSpeed = float(self.params.get_int("AutoNaviSpeedBumpSpeed"))
    self.autoNaviSpeedBumpTime = float(self.params.get_int("AutoNaviSpeedBumpTime"))
    self.autoNaviSpeedCtrlEnd = float(self.params.get_int("AutoNaviSpeedCtrlEnd"))
    self.autoNaviSpeedSafetyFactor = float(self.params.get_int("AutoNaviSpeedSafetyFactor")) * 0.01
    self.autoNaviSpeedDecelRate = float(self.params.get_int("AutoNaviSpeedDecelRate")) * 0.01

    self.autoTurnControlSpeedTurn = self.params.get_int("AutoTurnControlSpeedTurn")
    #self.autoTurnMapChange = self.params.get_int("AutoTurnMapChange")
    self.autoTurnControl = self.params.get_int("AutoTurnControl")
    self.autoTurnControlTurnEnd = self.params.get_int("AutoTurnControlTurnEnd")
    #self.autoNaviSpeedDecelRate = float(self.params.get_int("AutoNaviSpeedDecelRate")) * 0.01


  def _update_cmd(self):
    if self.carrotCmdIndex != self.carrotCmdIndex_last:
      self.carrotCmdIndex_last = self.carrotCmdIndex
      command_handlers = {
        "SPEED": self._handle_speed_command,
        "CRUISE": self._handle_cruise_command,
        "LANECHANGE": self._handle_lane_change,
        "RECORD": self._handle_record_command,
        "DISPLAY": self._handle_display_command,
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

  def _handle_speed_command(self, xArg):
    if xArg == "UP":
      pass
    elif xArg == "DOWN":
      pass
    else:
      try:
        #return clip(int(xArg), self.cruiseSpeedMin, self.cruiseSpeedMax)
        pass
      except ValueError:
        pass

  def _handle_cruise_command(self, xArg):
    if xArg == "ON":
      pass
    elif xArg == "OFF":
      pass
    elif xArg == "GO":
      pass
    elif xArg == "STOP":
      pass

  def _handle_lane_change(self, xArg):
    if xArg == "RIGHT":
      pass
    elif xArg == "LEFT":
      pass

  def _handle_record_command(self, xArg):
    record_commands = {"START": "1", "STOP": "2", "TOGGLE": "3"}
    command = record_commands.get(xArg)
    if command:
      pass

  def _handle_display_command(self, xArg):
    display_commands = {"MAP": "3", "FULLMAP": "4", "DEFAULT": "1", "ROAD": "2", "TOGGLE": "5"}
    command = display_commands.get(xArg)
    if command:
      pass

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
    traffic_red_trig = 0
    traffic_green_trig = 0
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
    elif self.nSdiPlusType == 22 or self.nSdiType == 22: # speed bump
      self.xSpdLimit = self.autoNaviSpeedBumpSpeed
      self.xSpdDist = self.nSdiPlusDist if self.nSdiPlusType == 22 else self.nSdiDist
      self.xSpdType = 22
    else:
      self.xSpdLimit = 0
      self.xSpdType = -1
      self.xSpdDist = 0

  def _update_gps(self, v_ego, sm):
    if not sm.updated['carState'] or not sm.updated['liveLocationKalman']:
      return 0.0
    CS = sm['carState']
    location = sm['liveLocationKalman']
    bearing = math.degrees(location.calibratedOrientationNED.value[2])
    if (location.status == log.LiveLocationKalman.Status.valid) and location.positionGeodetic.valid and location.gpsOK:            
      location_valid = True
      self.bearing_offset = 0.0
    else:
      location_valid = False

    now = time.monotonic()
    if self.active_sdi_count > self.active_sdi_count_max - 3:
      if not location_valid and CS is not None:
        diff_angle = self.nPosAngle - bearing;
        while diff_angle < 0.0:
          diff_angle += 360
        diff_angle = (diff_angle + 180) % 360 - 180;
        if abs(diff_angle) > 20 and v_ego > 1.0 and abs(CS.steeringAngleDeg) < 2.0:
          self.diff_angle_count += 1
        else:
          self.diff_angle_count = 0
        print("{:.1f} bearing_diff[{}] = {:.1f} = {:.1f} - {:.1f}, v={:.1f},st={:.1f}".format(self.bearing_offset, self.diff_angle_count, diff_angle, self.nPosAngle, bearing, CS.vEgo*3.6, CS.steeringAngleDeg))
        if self.diff_angle_count > 2:
          self.bearing_offset = self.nPosAngle - bearing
          print("bearing_offset = {:.1f} = {:.1f} - {:.1f}".format(self.bearing_offset, self.nPosAngle, bearing))
      #n초 통신 지연시간이 있다고 가정하고 좀더 진행한것으로 처리함.
      dt = 0 #(unix_now - timeStamp / 1000.) if timeStamp > 0 else 0.1
      dt += 0.2  #가상으로 0.5초만큼 더 진행한것으로 
      self.vpPosPointLat, self.vpPosPointLon = self.estimate_position(float(self.vpPosPointLat), float(self.vpPosPointLon), v_ego, bearing + self.bearing_offset, dt)
      self.last_update_gps_time = now
      self.last_calculate_gps_time = now
    elif now - self.last_update_gps_time < 3.0:# and CS is not None:
      dt = now - self.last_calculate_gps_time
      self.last_calculate_gps_time = now
      self.vpPosPointLat, self.vpPosPointLon = self.estimate_position(float(self.vpPosPointLat), float(self.vpPosPointLon), v_ego, bearing + self.bearing_offset, dt)
    #roadLimitSpeed.xPosSpeed = float(nPosSpeed)
    #roadLimitSpeed.xPosAngle = float(bearing + self.bearing_offset)
    #roadLimitSpeed.xPosLat = float(vpPosPointLat)
    #roadLimitSpeed.xPosLon = float(vpPosPointLon)

    return float(bearing + self.bearing_offset)
    
  def estimate_position(self, lat, lon, speed, angle, dt):
    R = 6371000
    angle_rad = math.radians(angle)
    delta_d = speed * dt
    delta_lat = delta_d * math.cos(angle_rad) / R
    new_lat = lat + math.degrees(delta_lat)
    delta_lon = delta_d * math.sin(angle_rad) / (R * math.cos(math.radians(lat)))
    new_lon = lon + math.degrees(delta_lon)
    
    return new_lat, new_lon

  def update_auto_turn(self, v_ego_kph, sm, x_turn_info, x_dist_to_turn):
    turn_speed = self.autoTurnControlSpeedTurn
    stop_speed = 1
    turn_dist = 50
    fork_dist = 20
    fork_speed = self.nRoadLimitSpeed
    stop_dist = 1
    start_fork_dist = interp(self.nRoadLimitSpeed, [30, 50, 100], [160, 200, 350])
    start_turn_dist = interp(self.nTBTNextRoadWidth, [5, 10], [43, 60])
    turn_info_mapping = {
        1: {"type": "turn left", "speed": turn_speed, "dist": turn_dist, "start": start_fork_dist},
        2: {"type": "turn right", "speed": turn_speed, "dist": turn_dist, "start": start_fork_dist},
        5: {"type": "straight", "speed": turn_speed, "dist": turn_dist, "start": start_turn_dist},
        3: {"type": "fork left", "speed": fork_speed, "dist": fork_dist, "start": start_fork_dist},
        4: {"type": "fork right", "speed": fork_speed, "dist": fork_dist, "start": start_fork_dist},
        6: {"type": "straight", "speed": fork_speed, "dist": fork_dist, "start": start_fork_dist},
        7: {"type": "straight", "speed": stop_speed, "dist": stop_dist, "start": 1000},
        8: {"type": "straight", "speed": stop_speed, "dist": stop_dist, "start": 1000},
    }

    default_mapping = {"type": "none", "speed": 0, "dist": 0, "start": 1000}

    mapping = turn_info_mapping.get(x_turn_info, default_mapping)

    atc_type = mapping["type"]
    atc_speed = mapping["speed"]
    atc_dist = mapping["dist"]
    atc_start_dist = mapping["start"]

    if x_dist_to_turn > atc_start_dist:
      atc_type += " prepare"
    elif atc_type in ["turn left", "turn right"] and x_dist_to_turn > start_turn_dist:
      atc_type = "fork left" if atc_type == "turn left" else "fork right"

    if x_dist_to_turn < atc_start_dist:
      steering_pressed = sm["carState"].steeringPressed
      steering_torque = sm["carState"].steeringTorque
      if steering_pressed and steering_torque < 0 and atc_type == "fork left":
        self.atc_paused = True
      elif steering_pressed and steering_torque > 0 and atc_type == "fork right":
        self.atc_paused = True
    else:
      self.atc_paused = False

    if self.atc_paused:
      atc_type += " canceled"

    atc_desired = 250    
    if atc_speed > 0 and x_dist_to_turn > 0:
      decel = self.autoNaviSpeedDecelRate
      safe_sec = 3.0      
      atc_desired = min(atc_desired, self.calculate_current_speed(x_dist_to_turn - atc_dist, atc_speed, safe_sec, decel))


    return atc_desired, atc_type, atc_speed, atc_dist

  def update_navi(self, remote_ip, sm, pm, vturn_speed):

    self.update_params()
    if sm.alive['carState']:
      CS = sm['carState']
      v_ego = CS.vEgo
      delta_dist = v_ego * 0.1#CS.totalDistance - self.totalDistance
      #self.totalDistance = CS.totalDistance
    else:
      v_ego = 0
      delta_dist = 0
      
    bearing = self.nPosAngle #self._update_gps(v_ego, sm)

    self.xSpdDist = max(self.xSpdDist - delta_dist, 0)
    self.xDistToTurn = max(self.xDistToTurn - delta_dist, 0)
    self.xDistToTurnNext = max(self.xDistToTurnNext - delta_dist, 0)
    self.active_count = max(self.active_count - 1, 0)
    self.active_sdi_count = max(self.active_sdi_count - 1, 0)
    if self.active_count > 0:
      self.active = 2 if self.active_sdi_count > 0 else 1
    else:
      self.active = 0

    if self.active <= 0:
      self.xSpdType = self.navType = self.xTurnInfo = self.xTurnInfoNext = -1
      self.nSdiType = self.nSdiBlockType = self.nSdiPlusBlockType = -1
      self.nTBTTurnType = self.nTBTTurnTypeNext = -1
      
    if self.xSpdType < 0 or self.xSpdDist <= 0:
      self.xSpdType = -1
      self.xSpdDist = self.xSpdLimit = 0
    if self.xTurnInfo < 0 or self.xDistToTurn <= 0:
      self.xDistToTurn = 0
      self.xTurnInfo = -1
      self.xDistToTurnNext = 0
      self.xTurnInfoNext = -1

    sdi_speed = 250
    ### 과속카메라, 사고방지턱
    if self.xSpdDist > 0 and self.active > 0:
      safe_sec = self.autoNaviSpeedBumpTime if self.xSpdType == 22 else self.autoNaviSpeedCtrlEnd
      decel = self.autoNaviSpeedDecelRate
      sdi_speed = min(sdi_speed, self.calculate_current_speed(self.xSpdDist, self.xSpdLimit * self.autoNaviSpeedSafetyFactor, safe_sec, decel))
      self.active = 3
      if self.xSpdType == 4:
        sdi_speed = self.xSpdLimit
        self.active = 4

    ### TBT 속도제어
    atc_desired, self.atcType, self.atcSpeed, self.atcDist = self.update_auto_turn(v_ego*3.6, sm, self.xTurnInfo, self.xDistToTurn)
    atc_desired_next, _, _, _ = self.update_auto_turn(v_ego*3.6, sm, self.xTurnInfoNext, self.xDistToTurnNext)

    if self.xTurnInfo >= 0:      
      self.debugText = f"Atc:{atc_desired:.1f},{self.xTurnInfo}:{self.xDistToTurn:.1f}, I({self.nTBTNextRoadWidth},{self.roadcate}) Atc2:{atc_desired_next:.1f},{self.xTurnInfoNext},{self.xDistToTurnNext:.1f}"
      
    if self.autoTurnControl not in [2, 3]:    # auto turn speed control
      atc_desired = atc_desired_next = 250

    if self.autoTurnControl not in [1,2]:    # auto turn control
      self.atcType = "none"

    speed_n_sources = [
      (atc_desired, "atc"),
      (atc_desired_next, "atc2"),
      (sdi_speed, "bump" if self.xSpdType == 22 else "section" if self.xSpdType == 4 else "cam"),
      (abs(vturn_speed), "vturn"),
    ]
    desired_speed, source = min(speed_n_sources, key=lambda x: x[0])

    left_spd_sec = 100
    if self.xSpdDist > 0:
      left_spd_sec = int(max(self.xSpdDist - v_ego, 1) / max(1, v_ego))
    left_tbt_sec = 100
    if self.xDistToTurn > 0:
      left_tbt_sec = int(max(self.xDistToTurn - v_ego, 1) / max(1, v_ego))

    self._update_cmd()
    if False:
      data = {
        "remote" : remote_ip,
        "active" : self.active,
        "xSpdType" : self.xSpdType,
        "xSpdLimit": self.xSpdLimit,
        "xSpdDist" : int(self.xSpdDist),
        "xSpdCountDown" : left_spd_sec,
        "xTurnInfo" : self.xTurnInfo,
        "xDistToTurn" : int(self.xDistToTurn),
        "xTurnCountDown" : left_tbt_sec,
        "atcType" : self.atcType,
        "vTurnSpeed" : int(vturn_speed),
        "nRoadLimitSpeed" : self.nRoadLimitSpeed,
        "szPosRoadName" : self.szPosRoadName,
        "szTBTMainText" : self.szTBTMainText,
        "desiredSpeed" : int(desired_speed),
        "desiredSource" : source,
        "carrotCmdIndex" : int(self.carrotCmdIndex),
        "carrotCmd" : self.carrotCmd,
        "carrotArg" : self.carrotArg,
        }
      try:
        self.params_memory.put_nonblocking("CarrotNavi", json.dumps(data))
      except Exception as e:
        print(f" error...: {e}")

    msg = messaging.new_message('carrotMan')
    msg.valid = True
    msg.carrotMan.active = self.active
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

    msg.carrotMan.xPosSpeed = float(self.nPosSpeed)
    msg.carrotMan.xPosAngle = float(bearing)
    msg.carrotMan.xPosLat = float(self.vpPosPointLat)
    msg.carrotMan.xPosLon = float(self.vpPosPointLon)

    pm.send('carrotMan', msg)
    
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
      
    self.active_count = 80

    if "goalPosX" in json:      
      self.goalPosX = float(json.get("goalPosX", self.goalPosX))
      self.goalPosY = float(json.get("goalPosY", self.goalPosY))
      self.szGoalName = json.get("szGoalName", self.szGoalName)
    elif "nRoadLimitSpeed" in json:
      self.active_sdi_count = self.active_sdi_count_max
      ### roadLimitSpeed
      nRoadLimitSpeed = int(json.get("nRoadLimitSpeed", 20))
      if nRoadLimitSpeed > 0:
        if nRoadLimitSpeed > 200:
          nRoadLimitSpeed = (nRoadLimitSpeed - 20) / 10
      else:
        nRoadLimitSpeed = 20
      self.nRoadLimitSpeed = nRoadLimitSpeed

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

      self.vpPosPointLat = float(json.get("vpPosPointLat", self.vpPosPointLat))
      self.vpPosPointLon = float(json.get("vpPosPointLon", self.vpPosPointLon))
      self.nPosSpeed = float(json.get("nPosSpeed", self.nPosSpeed))
      self.nPosAngle = float(json.get("nPosAngle", self.nPosAngle))
      self._update_tbt()
      self._update_sdi()
      print(f"sdi = {self.nSdiType}, {self.nSdiSpeedLimit}, {self.nSdiPlusType}, tbt = {self.nTBTTurnType}, {self.nTBTDist}, next={self.nTBTTurnTypeNext},{self.nTBTDistNext}")
      #print(json)
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
