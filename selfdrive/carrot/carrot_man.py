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
import zmq
from datetime import datetime
import traceback
from typing import Any, Dict, List, Optional

from aiohttp import web
import asyncio

from ftplib import FTP
from cereal import log
import urllib.request
import urllib.error
import ssl
import requests
import psutil
import ipaddress
import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper, set_core_affinity
from openpilot.common.params import Params, ParamKeyType
from openpilot.common.filter_simple import MyMovingAverage
from openpilot.system.hardware import PC, TICI
from openpilot.selfdrive.navd.helpers import Coordinate
from opendbc.car.common.conversions import Conversions as CV

from openpilot.selfdrive.carrot.carrot_serv import CarrotServ

from openpilot.common.gps import get_gps_location_service

try:
  from shapely.geometry import LineString
  SHAPELY_AVAILABLE = True
except ImportError:
  SHAPELY_AVAILABLE = False

NetworkType = log.DeviceState.NetworkType

################ CarrotNavi
## 국가법령정보센터: 도로설계기준
#V_CURVE_LOOKUP_BP = [0., 1./800., 1./670., 1./560., 1./440., 1./360., 1./265., 1./190., 1./135., 1./85., 1./55., 1./30., 1./15.]
#V_CRUVE_LOOKUP_VALS = [300, 150, 120, 110, 100, 90, 80, 70, 60, 50, 45, 35, 30]
V_CURVE_LOOKUP_BP = [0., 1./800., 1./670., 1./560., 1./440., 1./360., 1./265., 1./190., 1./135., 1./85., 1./55., 1./30., 1./25.]
V_CRUVE_LOOKUP_VALS = [300, 150, 120, 110, 100, 90, 80, 70, 60, 50, 40, 15, 5]

# Haversine formula to calculate distance between two GPS coordinates
#haversine_cache = {}
def haversine(lon1, lat1, lon2, lat2):
    #key = (lon1, lat1, lon2, lat2)
    #if key in haversine_cache:
    #    return haversine_cache[key]

    R = 6371000  # Radius of Earth in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    distance = 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    #haversine_cache[key] = distance
    return distance


# Get the closest point on a segment between two coordinates
def closest_point_on_segment(p1, p2, current_position):
    x1, y1 = p1
    x2, y2 = p2
    px, py = current_position

    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return p1  # p1 and p2 are the same point

    # Parameter t is the projection factor onto the line segment
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0, min(1, t))  # Clamp t to the segment

    closest_x = x1 + t * dx
    closest_y = y1 + t * dy

    return (closest_x, closest_y)


# Get path after a certain distance from the current position
def get_path_after_distance(start_index, coordinates, current_position, distance_m):
    total_distance = 0
    path_after_distance = []
    closest_index = -1
    closest_point = None
    min_distance = float('inf')

    start_index = max(0, start_index - 2)

    # 가까운 점만 탐색하도록 수정
    for i in range(start_index, len(coordinates) - 1):
        p1 = coordinates[i]
        p2 = coordinates[i + 1]
        candidate_point = closest_point_on_segment(p1, p2, current_position)
        distance = haversine(current_position[0], current_position[1], candidate_point[0], candidate_point[1])

        if distance < min_distance:
            min_distance = distance
            closest_point = candidate_point
            closest_index = i
        elif distance > min_distance and min_distance < 10:
            break

    start_index = closest_index
    # Start from the closest point and calculate the path after the specified distance
    if closest_index != -1:
        path_after_distance.append(closest_point)

        path_after_distance.append(coordinates[closest_index + 1])
        total_distance = haversine(closest_point[0], closest_point[1], coordinates[closest_index + 1][0],
                                   coordinates[closest_index + 1][1])

        # Traverse the path forward from the next point
        for i in range(closest_index + 1, len(coordinates) - 1):
            coord1 = coordinates[i]
            coord2 = coordinates[i + 1]
            segment_distance = haversine(coord1[0], coord1[1], coord2[0], coord2[1])

            if total_distance + segment_distance >= distance_m and segment_distance > 0:
                remaining_distance = distance_m - total_distance
                ratio = remaining_distance / segment_distance
                interpolated_lon = coord1[0] + ratio * (coord2[0] - coord1[0])
                interpolated_lat = coord1[1] + ratio * (coord2[1] - coord1[1])
                path_after_distance.append((interpolated_lon, interpolated_lat))
                break

            total_distance += segment_distance
            path_after_distance.append(coord2)

    return path_after_distance, start_index, closest_point


def calculate_angle(point1, point2):
    delta_lon = point2[0] - point1[0]
    delta_lat = point2[1] - point1[1]
    return math.degrees(math.atan2(delta_lat, delta_lon))

# Convert GPS coordinates to relative x, y coordinates based on a reference point and heading
def gps_to_relative_xy(gps_path, reference_point, heading_deg):
    ref_lon, ref_lat = reference_point
    relative_coordinates = []

    # Convert heading from degrees to radians
    heading_rad = math.radians(heading_deg)

    for lon, lat in gps_path:
        # Convert lat/lon differences to meters (assuming small distances for simple approximation)
        x = (lon - ref_lon) * 40008000 * math.cos(math.radians(ref_lat)) / 360
        y = (lat - ref_lat) * 40008000 / 360

        # Rotate coordinates based on the heading angle to align with the car's direction
        x_rot = x * math.cos(heading_rad) - y * math.sin(heading_rad)
        y_rot = x * math.sin(heading_rad) + y * math.cos(heading_rad)

        relative_coordinates.append((y_rot, x_rot))

    return relative_coordinates


# Calculate curvature given three points using a faster vector-based method
#curvature_cache = {}
def calculate_curvature(p1, p2, p3):
    #key = (p1, p2, p3)
    #if key in curvature_cache:
    #    return curvature_cache[key]

    v1 = (p2[0] - p1[0], p2[1] - p1[1])
    v2 = (p3[0] - p2[0], p3[1] - p2[1])

    cross_product = v1[0] * v2[1] - v1[1] * v2[0]
    len_v1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2)
    len_v2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2)

    if len_v1 * len_v2 == 0:
        curvature = 0
    else:
        curvature = cross_product / (len_v1 * len_v2 * len_v1)

    #curvature_cache[key] = curvature
    return curvature

class CarrotMan:
  def __init__(self):
    print("************************************************CarrotMan init************************************************")
    self.params = Params()
    self.params_memory = Params("/dev/shm/params")
    self.gps_location_service = get_gps_location_service(self.params)
    self.sm = messaging.SubMaster(['deviceState', 'carState', 'controlsState', 'radarState', 'longitudinalPlan', 'modelV2', 'selfdriveState', 'carControl', 'navRouteNavd', self.gps_location_service, 'navInstruction'])
    self.pm = messaging.PubMaster(['carrotMan', "navRoute", "navInstructionCarrot"])

    self.carrot_serv = CarrotServ()

    self.show_panda_debug = False
    self.broadcast_ip = self.get_broadcast_address()
    self.broadcast_port = 7705
    self.carrot_man_port = 7706
    self.connection = None

    self.ip_address = "0.0.0.0"
    self.remote_addr = None

    self.turn_speed_last = 250
    self.curvatureFilter = MyMovingAverage(20)
    self.carrot_curve_speed_params()

    self.carrot_zmq_thread = threading.Thread(target=self.carrot_cmd_zmq, args=[])
    self.carrot_zmq_thread.daemon = True
    self.carrot_zmq_thread.start()

    self.carrot_panda_debug_thread = threading.Thread(target=self.carrot_panda_debug, args=[])
    self.carrot_panda_debug_thread.daemon = True
    self.carrot_panda_debug_thread.start()

    self.carrot_route_thread = threading.Thread(target=self.carrot_route, args=[])
    self.carrot_route_thread.daemon = True
    self.carrot_route_thread.start()

    self.is_running = True
    threading.Thread(target=self.broadcast_version_info).start()

    self.navi_points = []
    self.navi_points_start_index = 0
    self.navi_points_active = False
    self.navd_active = False

    self.active_carrot_last = False

    self._rgdata_ts_lock = threading.Lock()
    self._last_rgdata_timestamp_ms = 0

    self.is_metric = self.params.get_bool("IsMetric")

  def get_broadcast_address(self):
    if PC:
      iface = b'br0'
    else:
      iface = b'wlan0'
    try:
      with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        ip = fcntl.ioctl(
          s.fileno(),
          0x8919,
          struct.pack('256s', iface)
        )[20:24]
        return socket.inet_ntoa(ip)
    except (OSError, Exception):
      return None

  def get_local_ip(self):
      try:
          # 외부 서버와의 연결을 통해 로컬 IP 확인
          with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
              s.connect(("8.8.8.8", 80))  # Google DNS로 연결 시도
              return s.getsockname()[0]
      except Exception as e:
          return f"Error: {e}"


  # 브로드캐스트 메시지 전송
  def broadcast_version_info(self):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    frame = 0
    self.save_toggle_values()

    rk = Ratekeeper(20, print_delay_threshold=None)

    while self.is_running:
      try:
        self.sm.update(0)
        if self.sm.updated['navRouteNavd']:
          self.send_routes(self.sm['navRouteNavd'].coordinates, True)
        remote_addr = self.remote_addr
        remote_ip = remote_addr[0] if remote_addr is not None else ""
        vturn_speed = self.carrot_curve_speed(self.sm)
        coords, distances, route_speed = self.carrot_navi_route()

        #print("coords=", coords)
        #print("curvatures=", curvatures)
        self.carrot_serv.update_navi(remote_ip, self.sm, self.pm, vturn_speed, coords, distances, route_speed, self.gps_location_service)

        if frame % 20 == 0 or remote_addr is not None:
          try:
            self.broadcast_ip = self.get_broadcast_address() if remote_addr is None else remote_addr[0]
            if not PC:
              ip_address = socket.gethostbyname(socket.gethostname())
            else:
              ip_address = self.get_local_ip()
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
              #print(f"Broadcasting: {self.broadcast_ip}") #:{msg}")
              if not self.navd_active:
                #print("clear path_points: navd_active: ", self.navd_active)
                self.navi_points = []
                self.navi_points_active = False

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


  def carrot_navi_route(self):

    if self.carrot_serv.active_carrot > 1:
      if False and self.navd_active:  # mabox always active
        self.navd_active = False
        self.params.remove("NavDestination")
    is_onroad = self.params.get_bool("IsOnroad")
    if not is_onroad or not self.navi_points_active or not SHAPELY_AVAILABLE or (self.carrot_serv.active_carrot <= 1 and not self.navd_active):
      #print(f"navi_points_active: {self.navi_points_active}, active_carrot: {self.carrot_serv.active_carrot}")
      if self.navi_points_active:
        print("navi_points_active: ", self.navi_points_active, "active_carrot: ", self.carrot_serv.active_carrot, "navd_active: ", self.navd_active)
        #haversine_cache.clear()
        #curvature_cache.clear()
        self.navi_points = []
        self.navi_points_active = False
        if self.active_carrot_last > 1:
          #self.params.remove("NavDestination")
          pass
      self.active_carrot_last = self.carrot_serv.active_carrot
      return [],[],300

    current_position = (self.carrot_serv.vpPosPointLon, self.carrot_serv.vpPosPointLat)
    heading_deg = self.carrot_serv.bearing

    distance_interval = 10.0
    out_speed = 300
    path, self.navi_points_start_index, start_point = get_path_after_distance(self.navi_points_start_index, self.navi_points, current_position, 300)
    relative_coords = []
    if path:
        #relative_coords = gps_to_relative_xy(path, current_position, heading_deg)
        relative_coords = gps_to_relative_xy(path, start_point, heading_deg)
        # Resample relative_coords at 5m intervals using LineString
        line = LineString(relative_coords)
        resampled_points = []
        resampled_distances = []
        current_distance = 0
        while current_distance <= line.length:
            point = line.interpolate(current_distance)
            resampled_points.append((point.x, point.y))
            resampled_distances.append(current_distance)
            current_distance += distance_interval

        curvatures = []
        distances = []
        distance = 10.0
        sample = 4
        if len(resampled_points) >= sample * 2 + 1:
            # Calculate curvatures and speeds based on curvature
            speeds = []
            for i in range(len(resampled_points) - sample * 2):
                distance += distance_interval
                p1, p2, p3 = resampled_points[i], resampled_points[i + sample], resampled_points[i + sample * 2]
                curvature = calculate_curvature(p1, p2, p3)
                curvatures.append(curvature)
                speed = np.interp(abs(curvature), V_CURVE_LOOKUP_BP, V_CRUVE_LOOKUP_VALS)
                if abs(curvature) < 0.02:
                  speed = max(speed, self.carrot_serv.nRoadLimitSpeed)
                speeds.append(speed)
                distances.append(distance)
            #print(f"curvatures= {[round(s, 4) for s in curvatures]}")
            #print(f"speeds= {[round(s, 1) for s in speeds]}")
            # Apply acceleration limits in reverse to adjust speeds
            accel_limit = self.carrot_serv.autoNaviSpeedDecelRate # m/s^2
            accel_limit_kmh = accel_limit * 3.6  # Convert to km/h per second
            out_speeds = [0] * len(speeds)
            out_speeds[-1] = speeds[-1]  # Set the last speed as the initial value
            v_ego_kph = self.sm['carState'].vEgo * 3.6

            time_delay = self.carrot_serv.autoNaviSpeedCtrlEnd
            time_wait = 0
            for i in range(len(speeds) - 2, -1, -1):
                target_speed = speeds[i]
                next_out_speed = out_speeds[i + 1]

                if target_speed < next_out_speed:
                  time_delay = max(0, ((v_ego_kph - target_speed) / accel_limit_kmh))
                  time_wait = - time_delay

                # Calculate time interval for the current segment based on speed
                time_interval = distance_interval / (next_out_speed / 3.6) if next_out_speed > 0 else 0

                time_apply = min(time_interval, max(0, time_interval + time_wait))

                # Calculate maximum allowed speed with acceleration limit
                max_allowed_speed = next_out_speed + (accel_limit_kmh * time_apply)
                adjusted_speed = min(target_speed, max_allowed_speed)

                #time_wait += time_interval
                time_wait += min(2.0, time_interval)

                out_speeds[i] = adjusted_speed

            #distance_advance = self.sm['carState'].vEgo * 3.0  # Advance distance by 3.0 seconds
            #out_speed = interp(distance_advance, distances, out_speeds)
            out_speed = out_speeds[0]
            #print(f"out_speeds= {[round(s, 1) for s in out_speeds]}")
    else:
        resampled_points = []
        resampled_distances = []
        curvatures = []
        speeds = []
        distances = []
        #self.params.remove("NavDestination")

    return resampled_points, resampled_distances, out_speed #speeds, distances


  def make_send_message(self):
    msg = {}
    msg['Carrot2'] = self.params.get("Version")
    isOnroad = self.params.get_bool("IsOnroad")
    msg['IsOnroad'] = isOnroad
    msg['CarrotRouteActive'] = self.navi_points_active
    msg['ip'] = self.ip_address
    msg['port'] = self.carrot_man_port
    self.controls_active = False
    self.xState = 0
    self.trafficState = 0
    v_ego_kph = 0
    log_carrot = ""
    v_cruise_kph = 0
    carcruiseSpeed = 0
    if not isOnroad:
      self.xState = 0
      self.trafficState = 0
    else:
      if self.sm.alive['carState']:
        carState = self.sm['carState']
        v_ego_kph = int(carState.vEgoCluster * 3.6 + 0.5)
        log_carrot = carState.logCarrot
        v_cruise_kph = carState.vCruise
        carcruiseSpeed = carState.cruiseState.speed * 3.6
      if self.sm.alive['selfdriveState']:
        selfdrive = self.sm['selfdriveState']
        self.controls_active = selfdrive.active
      if self.sm.alive['longitudinalPlan']:
        lp = self.sm['longitudinalPlan']
        self.xState = lp.xState
        self.trafficState = lp.trafficState

    msg['log_carrot'] = log_carrot
    msg['v_cruise_kph'] = v_cruise_kph
    msg['carcruiseSpeed'] = carcruiseSpeed
    msg['v_ego_kph'] = v_ego_kph
    msg['tbt_dist'] = self.carrot_serv.xDistToTurn
    msg['sdi_dist'] = self.carrot_serv.xSpdDist
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

              except TimeoutError:
                #print("Waiting for data (timeout)...")
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

  def parse_kisa_data(self, data: bytes):
    result = {}

    try:
      decoded = data.decode('utf-8')
    except UnicodeDecodeError:
      print("Decoding error:", data)
      return result

    parts = decoded.split('/')
    for part in parts:
      if ':' in part:
        key, value = part.split(':', 1)
        try:
          result[key] = int(value)
        except ValueError:
          result[key] = value
    return result

  def kisa_app_thread(self):
    while True:
      try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
          sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
          sock.settimeout(10)  # 소켓 타임아웃 설정 (10초)
          sock.bind(('', 12345))  # UDP 포트 바인딩
          print("#########kisa_app_thread: UDP thread started...")

          while True:
            try:
              #self.remote_addr = None
              # 데이터 수신 (UDP는 recvfrom 사용)
              try:
                data, remote_addr = sock.recvfrom(4096)  # 최대 4096 바이트 수신
                #print(f"Received data from {self.remote_addr}")

                if not data:
                  raise ConnectionError("No data received")

                #if self.remote_addr is None:
                #  print("Connected to: ", remote_addr)
                #self.remote_addr = remote_addr
                try:
                  print(data)
                  kisa_data = self.parse_kisa_data(data)
                  self.carrot_serv.update_kisa(kisa_data)
                  #json_obj = json.loads(data.decode())
                  #print(json_obj)
                except Exception as e:
                  traceback.print_exc()
                  print(f"kisa_app_thread: json error...: {e}")
                  print(data)

              except TimeoutError:
                #print("Waiting for data (timeout)...")
                #self.remote_addr = None
                time.sleep(1)

              except Exception as e:
                print(f"kisa_app_thread: error...: {e}")
                #self.remote_addr = None
                break

            except Exception as e:
              print(f"kisa_app_thread: recv error...: {e}")
              #self.remote_addr = None
              break

          time.sleep(1)
      except Exception as e:
        #self.remote_addr = None
        print(f"Network error, retrying...: {e}")
        time.sleep(2)

  def make_tmux_data(self):
    try:
      subprocess.run("rm /data/media/tmux.log; tmux capture-pane -pq -S-1000 > /data/media/tmux.log", shell=True, capture_output=True, text=False)
      subprocess.run("/data/openpilot/selfdrive/apilot.py", shell=True, capture_output=True, text=False)
    except Exception as e:
      print(f"TMUX creation error: {e}")
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
      car_selected = car_selected

    git_branch = Params().get("GitBranch").replace("/", "__")
    try:
      ftp.mkd(git_branch)
    except Exception as e:
      print(f"Directory creation failed: {e}")
    ftp.cwd(git_branch)

    directory = car_selected + " " + Params().get("DongleId")
    current_time = datetime.now().strftime("%Y%m%d-%H%M%S")
    filename = tmux_why + "-" + current_time + "-" + git_branch + ".txt"

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

  def send_tmux_http(self, tmux_why, send_settings=False):
    def get_private_ip_by_iface(name="wlan0"):
      addrs = psutil.net_if_addrs().get(name, [])

      for addr in addrs:
          if addr.family == socket.AF_INET:
              try:
                  ip_obj = ipaddress.ip_address(addr.address)
                  if ip_obj.is_private:
                      return addr.address
              except ValueError:
                  continue
      return None

    def _pstr(key):
      v = Params().get(key) or ""
      return v.decode("utf-8", errors="ignore") if isinstance(v, bytes) else v

    url = "https://tmux.carrotpilot.app/upload"

    payload = {
      "car_name"          : f"{_pstr("CarName")}",
      "git_branch"        : f"{_pstr("GitBranch")}",
      "github_id"         : f"{_pstr("GithubUsername")}",
      "git_remote"        : f"{_pstr("GitRemote")}",
      "git_commit"        : f"{_pstr("GitCommit")}",
      "git_commit_date"   : f"{_pstr("GitCommitDate")}",
      "dongle_id"         : f"{_pstr("DongleId")}",
      "device_serial"     : f"{_pstr("HardwareSerial")}",
      "local_ip"          : f"{get_private_ip_by_iface("wlan0")}",
    }

    files = [
        ("files[0]", ("tmux.log", open("/data/media/tmux.log", "rb"), "text/plain")),
    ]

    if send_settings:
      #self.save_toggle_values()
      files.append(("files[1]",("toggle_values.json",open("/data/toggle_values.json", "rb"),"application/json")))

    params = {}
    headers = {}

    try:
      response = requests.post(
          url,
          params=params,
          headers=headers,
          data=payload,
          files=files,
          timeout=10,
      )
      print(response.status_code, response.text)
      return response
    finally:
      for _, fileinfo in files:
        fileobj = fileinfo[1]
        try:
          fileobj.close()
        except Exception:
          pass

  def carrot_panda_debug(self):
    #time.sleep(2)
    while True:
      if self.show_panda_debug:
        self.show_panda_debug = False
        try:
          subprocess.run("/data/openpilot/selfdrive/debug/debug_console_carrot.py", shell=True)
        except Exception as e:
          print(f"debug_console error: {e}")
          time.sleep(2)
      else:
        time.sleep(1)

  def get_all_toggle_values(self):
    toggle_values = {}

    for k in self.params.all_keys():
      # key 정리
      if isinstance(k, (bytes, bytearray, memoryview)):
        try:
          key = k.decode("utf-8")
        except Exception:
          continue
      else:
        key = str(k)

      # 타입 확인 + 제외
      try:
        t = self.params.get_type(key)
      except Exception:
        continue
      if t in (ParamKeyType.BYTES, ParamKeyType.JSON):
        continue

      # default 없는 키 제외
      try:
        dv = self.params.get_default_value(key)
      except Exception:
        continue
      if dv is None:
        continue

      # 값 읽기 (이미 Params.get()이 타입 변환까지 해줌)
      try:
        v = self.params.get(key, block=False, return_default=False)
      except Exception:
        v = None

      # v가 None이면 default로 채우고 싶으면 dv로 대체 (선택)
      if v is None:
        v = dv

      # 최종 stringify (jsonify 용)
      if isinstance(v, (dict, list)):
        toggle_values[key] = json.dumps(v, ensure_ascii=False)
      else:
        toggle_values[key] = str(v)

    return toggle_values

  def save_toggle_values(self):
    try:
      toggle_values = self.get_all_toggle_values()
      file_path = os.path.join('/data', 'toggle_values.json')
      with open(file_path, 'w') as file:
        json.dump(toggle_values, file, indent=2)
    except Exception as e:
      print(f"save_toggle_values error: {e}")

  def carrot_cmd_zmq(self):

    context = zmq.Context()
    def setup_socket():
        socket = context.socket(zmq.REP)
        socket.bind("tcp://*:7710")
        poller = zmq.Poller()
        poller.register(socket, zmq.POLLIN)
        return socket, poller

    socket, poller = setup_socket()
    isOnroadCount = 0
    is_tmux_sent = False

    print("#########carrot_cmd_zmq: thread started...")
    while True:
      try:
        socks = dict(poller.poll(100))

        if socket in socks and socks[socket] == zmq.POLLIN:
          message = socket.recv(zmq.NOBLOCK)
          print(f"Received:7710 request: {message}")
          json_obj = json.loads(message.decode())
        else:
          json_obj = None

        if json_obj is None:
          isOnroadCount = isOnroadCount + 1 if self.params.get_bool("IsOnroad") else 0
          if isOnroadCount == 0:
            is_tmux_sent = False
          if isOnroadCount == 1:
            self.show_panda_debug = True

          network_type = self.sm['deviceState'].networkType # if not force_wifi else NetworkType.wifi
          networkConnected = False if network_type == NetworkType.none else True

          if isOnroadCount == 500:
            self.make_tmux_data()
          if isOnroadCount > 500 and not is_tmux_sent and networkConnected:
            self.send_tmux("Ekdrmsvkdlffjt7710", "onroad", send_settings = True)
            self.send_tmux_http("onroad", send_settings = True)
            is_tmux_sent = True
          carrot_exception = self.params.get("CarrotException")
          if carrot_exception in ["exception", "log", "tmux_send"] and networkConnected:
            self.params.put_bool("CarrotException", "")
            self.make_tmux_data()
            self.send_tmux("Ekdrmsvkdlffjt7710", carrot_exception)
            self.send_tmux_http(carrot_exception, send_settings = False)
        elif 'echo_cmd' in json_obj:
          try:
            result = subprocess.run(json_obj['echo_cmd'], shell=True, capture_output=True, text=False)
            exitStatus = result.returncode
            try:
              stdout = result.stdout.decode('utf-8')
              stderr = result.stderr.decode('utf-8')
            except UnicodeDecodeError:
              stdout = result.stdout.decode('euc-kr', 'ignore')
              stderr = result.stderr.decode('euc-kr', 'ignore')

            echo = json.dumps({"echo_cmd": json_obj['echo_cmd'], "exitStatus": exitStatus, "result": stdout, "error": stderr})
          except Exception as e:
            echo = json.dumps({"echo_cmd": json_obj['echo_cmd'], "exitStatus": exitStatus, "result": "", "error": f"exception error: {str(e)}"})
          #print(echo)
          socket.send(echo.encode())
        elif 'tmux_send' in json_obj:
          self.make_tmux_data()
          self.send_tmux(json_obj['tmux_send'], "tmux_send")
          self.send_tmux_http("tmux_send")
          echo = json.dumps({"tmux_send": json_obj['tmux_send'], "result": "success"})
          socket.send(echo.encode())
      except Exception as e:
        print(f"carrot_cmd_zmq error: {e}")
        socket.close()
        time.sleep(1)
        socket, poller = setup_socket()

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


  def send_routes(self, coords, from_navd=False):
    if from_navd:
      if len(coords) > 0:
        self.navi_points = [(c.longitude, c.latitude) for c in coords]
        self.navi_points_start_index = 0
        self.navi_points_active = True
        print("Received points from navd:", len(self.navi_points))
        self.navd_active = True

        # 경로수신 -> carrotman active되고 약간의 시간지연이 발생함..
        if not from_navd:
          self.carrot_serv.active_count = 80
          self.carrot_serv.active_sdi_count = self.carrot_serv.active_sdi_count_max
          self.carrot_serv.active_carrot = 2

        coords = [{"latitude": c.latitude, "longitude": c.longitude} for c in coords]
        #print("navdNaviPoints=", self.navi_points)
      else:
        print("Received points from navd: 0")
        self.navd_active = False

    msg = messaging.new_message('navRoute', valid=True)
    msg.navRoute.coordinates = coords
    self.pm.send('navRoute', msg)

  def carrot_route(self):
    host = '0.0.0.0'  # 혹은 다른 호스트 주소
    port = 7709  # 포트 번호

    try:
      with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()

        while True:
          print("################# waiting connection from CarrotMan route #####################")
          conn, addr = s.accept()
          with conn:
            print(f"Connected by {addr}")
            #self.clear_route()

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

              self.navi_points = []
              points = []
              for i in range(0, len(all_data), 8):
                x, y = struct.unpack('!ff', all_data[i:i+8])
                self.navi_points.append((x, y))
                coord = Coordinate.from_mapbox_tuple((x, y))
                points.append(coord)
              coords = [c.as_dict() for c in points]
              self.navi_points_start_index = 0
              self.navi_points_active = True
              print("Received points:", len(self.navi_points))
              #print("Received points:", self.navi_points)

              self.send_routes(coords)
              """
              try:
                module_name = "route_engine"
                class_name = "RouteEngine"
                moduel = importlib.import_module(module_name)
                cls = getattr(moduel, class_name)
                route_engine_instance = cls(name="Loaded at Runtime")

                route_engine_instance.send_route_coords(coords, True)
              except Exception as e:
                print(f"route_engine error: {e}")

              #msg = messaging.new_message('navRoute', valid=True)
              #msg.navRoute.coordinates = coords
              #self.pm.send('navRoute', msg)
              """

              if len(coords):
                dest = coords[-1]
                dest['place_name'] = "External Navi"
                self.params.put("NavDestination", json.dumps(dest))

            except Exception as e:
              print(e)
    except Exception as e:
      print("################# CarrotMan route server error #####################")
      print(e)

  def carrot_curve_speed_params(self):
    self.autoCurveSpeedFactor = self.params.get_int("AutoCurveSpeedFactor")*0.01
    self.autoCurveSpeedAggressiveness = self.params.get_int("AutoCurveSpeedAggressiveness")*0.01

  def carrot_curve_speed(self, sm):
    self.carrot_curve_speed_params()
    if not sm.alive['carState'] and not sm.alive['modelV2']:
        return 250
    #print(len(sm['modelV2'].orientationRate.z))
    if len(sm['modelV2'].orientationRate.z) == 0:
        return 250

    return self.vturn_speed(sm['carState'], sm)

  def vturn_speed(self, CS, sm):
    TARGET_LAT_A = 1.9  # m/s^2

    modelData = sm['modelV2']
    v_ego = max(CS.vEgo, 0.1)
    # Set the curve sensitivity
    orientation_rate = np.array(modelData.orientationRate.z) * self.autoCurveSpeedFactor
    velocity = np.array(modelData.velocity.x)

    # Get the maximum lat accel from the model
    max_index = np.argmax(np.abs(orientation_rate))
    curv_direction = np.sign(orientation_rate[max_index])
    max_pred_lat_acc = np.amax(np.abs(orientation_rate) * velocity)

    # Get the maximum curve based on the current velocity
    max_curve = max_pred_lat_acc / (v_ego**2)

    # Set the target lateral acceleration
    adjusted_target_lat_a = TARGET_LAT_A * self.autoCurveSpeedAggressiveness

    # Get the target velocity for the maximum curve
    #turnSpeed = max(abs(adjusted_target_lat_a / max_curve)**0.5  * 3.6, self.autoCurveSpeedLowerLimit)
    turnSpeed = max(abs(adjusted_target_lat_a / max_curve)**0.5  * 3.6, 5)
    turnSpeed = min(turnSpeed, 250)
    return turnSpeed * curv_direction

  def carrot_navi_thread(self):
    self.carrot_navi_tcp_server(7712)

  def handle_route(self, arr: list):
    if not arr:
      print("Received route: 0")
      # navd route가 비어오면 비활성 처리
      self.navi_points = []
      self.navi_points_start_index = 0
      self.navi_points_active = False
      self.navd_active = False
      return

    # valid만 필터 (필요 없으면 제거)
    valid_pts = [p for p in arr if isinstance(p, dict) and p.get("valid", True)]
    if not valid_pts:
      print("Received route: 0 valid")
      self.navi_points = []
      self.navi_points_start_index = 0
      self.navi_points_active = False
      self.navd_active = False
      return

    # x=lon, y=lat
    coords = []
    navi_points = []

    for p in valid_pts:
      try:
        lon = float(p.get("x"))
        lat = float(p.get("y"))
      except Exception:
        continue

      navi_points.append((lon, lat))
      coords.append({"latitude": lat, "longitude": lon})

    self.navi_points = navi_points
    self.navi_points_start_index = 0
    self.navi_points_active = True
    self.navd_active = True

    print("Received points:", len(self.navi_points))

    self.send_routes(coords)

    if coords:
      dest = dict(coords[-1])
      dest["place_name"] = "External Navi"
      try:
        self.params.put("NavDestination", json.dumps(dest))
      except Exception as e:
        print("NavDestination put error:", e)

  def handle_traffic_light(self, d: dict):
    print(f"[Traffic] {d}")

    # {'distance': 120, 'greenLightRemainTime': 0, 'leftLightRemainTime': 0, 'location': {'coordString': 'x:127.045286, y:37.477032', 'latitude': 37.47703188722564, 'longitude': 127.04528634430659},
    #       'redLightRemainTime': 15, 'rightLightRemainTime': 0, 'uturnLightRemainTime': 0, 'greenLightOn': False, 'leftLightOn': False, 'redLightOn': True, 'rightLightOn': False, 'uturnLightOn': False}
    lamp = None
    remain = 0

    if d.get("redLightOn"):
      lamp = "red"
      remain = d.get("redLightRemainTime", 0)
    elif d.get("leftLightOn"):
      lamp = "left"
      remain = d.get("leftLightRemainTime", 0)
    elif d.get("greenLightOn"):
      lamp = "green"
      remain = d.get("greenLightRemainTime", 0)
    elif d.get("rightLightOn"):
      lamp = "right"
      remain = d.get("rightLightRemainTime", 0)
    elif d.get("uturnLightOn"):
      lamp = "uturn"
      remain = d.get("uturnLightRemainTime", 0)

    if lamp is None:
      return

    traffic_light = {
      "distance": int(d.get("distance", 0)),
      "lamp": lamp,
      "remain": int(remain),
    }
    self.params_memory.put("TrafficLight", json.dumps(traffic_light))


  def handle_carrot_state(self, d: dict):
    try:
      self.carrot_serv.update(d)
    except Exception as e:
      print("carrot_state update error:", e)

  def handle_unknown(self, obj: Any):
    print("[UNKNOWN]", str(obj)[:200])

  def _get_timestamp_ms(self, obj: Any) -> int:
    if not isinstance(obj, dict):
      return 0
    try:
      return int(obj.get("timestamp_ms", 0))
    except Exception:
      return 0


  def _is_stale_rgdata(self, timestamp_ms: int):
    if timestamp_ms <= 0:
      return False, 0

    with self._rgdata_ts_lock:
      last_ts = self._last_rgdata_timestamp_ms
      if timestamp_ms <= last_ts:
        return True, last_ts

      self._last_rgdata_timestamp_ms = timestamp_ms
      return False, last_ts

  def _dispatch_obj(self, obj: Any):
    if obj is None:
      return

    # obj가 str이면 여기서 JSON 파싱
    if isinstance(obj, str):
      s = obj.strip()
      if not s:
        return
      try:
        obj = json.loads(s)
      except Exception:
        # JSON 아니면 unknown 처리
        return self.handle_unknown(s[:200])

    if not isinstance(obj, dict):
      return self.handle_unknown(obj)

    if "vrtx" in obj:
      self.handle_route(obj["vrtx"])

    if "rgdata" in obj:
      timestamp_ms = self._get_timestamp_ms(obj)
      stale, last_ts = self._is_stale_rgdata(timestamp_ms)
      if stale:
        print(f"[STALE DROP] rgdata ts={timestamp_ms} <= last={last_ts}")
      else:
        self.handle_carrot_state(obj["rgdata"])

    if "sinf" in obj:
      self.handle_traffic_light(obj["sinf"])

  def carrot_navi_http_thread(self):
    asyncio.run(self.carrot_navi_http_server(7713))

  def carrot_navi_tcp_server(self, port: int = 7712):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", port))
    server.listen(5)
    print("TCP server listening", port)

    while True:
      conn, addr = server.accept()
      self.remote_addr = addr
      print("Connected:", addr)
      conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
      try:
        f = conn.makefile("r", encoding="utf-8", errors="ignore")
        while True:
          try:
            line = f.readline()
          except socket.timeout:
            print("TCP timeout: closing connection", addr)
            break

          if not line:
            break

          s = line.strip()
          if not s:
            continue

          try:
            obj = json.loads(s)
          except Exception:
            obj = s

          try:
            self._dispatch_obj(obj)
          except Exception as e:
            print("dispatch error:", e, "raw:", repr(s[:200]))

      except Exception as e:
        print("TCP error:", e)

      finally:
        try:
          conn.close()
        except Exception:
          pass
        self.remote_addr = None

  async def carrot_http_post(self, request: web.Request):
    tmap_version = request.match_info.get("tmap_version", "")

    try:
      peer = request.transport.get_extra_info("peername")
    except Exception:
      peer = None

    #print(f"[HTTP] request from={peer} version={tmap_version}")

    try:
      obj = await request.json()
      #if isinstance(obj, dict):
      #  print(f"[HTTP] json keys={list(obj.keys())[:10]}")
      #else:
      #  print(f"[HTTP] json type={type(obj).__name__}")
    except Exception as e:
      print(f"[HTTP] json parse error: {e}")
      return web.json_response({
        "ok": False,
        "error": f"invalid json: {e}"
      }, status=400)

    if isinstance(obj, dict):
      obj["_tmap_version"] = tmap_version

    try:
      self._dispatch_obj(obj)
      #print(f"[HTTP] dispatch ok version={tmap_version}")
      #print(obj)
      return web.json_response({
        "ok": True,
        "tmap_version": tmap_version
      })
    except Exception as e:
      print(f"[HTTP] dispatch error: {e}")
      traceback.print_exc()
      return web.json_response({
        "ok": False,
        "error": str(e),
        "tmap_version": tmap_version
      }, status=500)

  async def carrot_http_health(self, request: web.Request):
    return web.json_response({
      "ok": True,
      "service": "carrot_navi_http"
    })

  async def carrot_navi_http_server(self, port: int = 7713):
    app = web.Application(client_max_size=1024 * 1024)

    app.router.add_post("/api/navi/{tmap_version}", self.carrot_http_post)
    app.router.add_get("/health", self.carrot_http_health)

    runner = web.AppRunner(app, access_log=None)
    await runner.setup()

    site = web.TCPSite(runner, "0.0.0.0", port)
    await site.start()

    print("HTTP server listening", port)

    while True:
      await asyncio.sleep(3600)

def main():
  try:
    set_core_affinity([0, 1, 2, 3])
  except Exception:
    print("[carrot_man] failed to set core affinity")

  print("CarrotManager Started")
  #print("Carrot GitBranch = {}, {}".format(Params().get("GitBranch"), Params().get("GitCommitDate")))
  carrot_man = CarrotMan()

  print(f"CarrotMan {carrot_man}")
  threading.Thread(target=carrot_man.kisa_app_thread).start()
  threading.Thread(target=carrot_man.carrot_navi_thread).start()
  threading.Thread(target=carrot_man.carrot_navi_http_thread).start()

  while True:
    try:
      carrot_man.carrot_man_thread()
    except Exception as e:
      print(f"carrot_man error...: {e}")
      traceback.print_exc()
      time.sleep(10)


if __name__ == "__main__":
  main()
