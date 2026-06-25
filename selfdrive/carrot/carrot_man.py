import fcntl
import hashlib
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
NAVI_HTTP_PORT = 7713
NAVI_HTTP_MAX_BODY_SIZE = 16 * 1024 * 1024
NAVI_EVENT_TYPES = ("complexCrossroad", "rgdata", "vrtx", "ssinf", "sinf", "route")
NAVI_DEBUG_PARAM = "CarrotNaviDebug"
NAVI_IMAGE_PARAM = "CarrotNaviImage"
NAVI_IMAGE_BASE64_MAX_CHARS = 6 * 1024 * 1024
NAVI_ROUTE_MAX_POINTS = 4096
NAVI_ROUTE_SUMMARY_MAX_SCAN = 20000
AUTO_ONROAD_DIAGNOSTICS = os.environ.get("CARROT_AUTO_ONROAD_DIAGNOSTICS", "1").strip().lower() in ("1", "true", "yes", "on")
AUTO_ONROAD_TMUX_DELAY_SECONDS = float(os.environ.get("CARROT_AUTO_ONROAD_TMUX_DELAY_SECONDS", "60"))
CARROT_EXCEPTION_UPLOAD_RETRY_SECONDS = 60.0


def limit_route_points(points, max_points=NAVI_ROUTE_MAX_POINTS):
    if max_points <= 0:
        return []
    count = len(points)
    if count <= max_points:
        return list(points)

    limited = []
    last_index = count - 1
    previous_index = -1
    for i in range(max_points):
        source_index = round(i * last_index / max(1, max_points - 1))
        if source_index == previous_index:
            continue
        limited.append(points[source_index])
        previous_index = source_index
    return limited

_carrot_exception_tmux_send_lock = threading.Lock()
_carrot_exception_tmux_send_queued = False


def reset_carrot_exception_tmux_send_queue() -> None:
  global _carrot_exception_tmux_send_queued

  with _carrot_exception_tmux_send_lock:
    _carrot_exception_tmux_send_queued = False


def queue_carrot_exception_tmux_send(context: str = "") -> None:
  global _carrot_exception_tmux_send_queued

  with _carrot_exception_tmux_send_lock:
    if _carrot_exception_tmux_send_queued:
      return

    try:
      params = Params()
      current = params.get("CarrotException")
      if current in (None, "", b""):
        put_nonblocking = getattr(params, "put_nonblocking", None)
        if callable(put_nonblocking):
          put_nonblocking("CarrotException", "tmux_send")
        else:
          params.put("CarrotException", "tmux_send")
        _carrot_exception_tmux_send_queued = True
        print(f"[carrot_man] CarrotException tmux_send queued: {context or 'exception'}")
      elif current == "tmux_send":
        _carrot_exception_tmux_send_queued = True
    except Exception as e:
      print(f"[carrot_man] failed to queue CarrotException tmux_send: {e}")

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
    self.carrot_navi_http_port = NAVI_HTTP_PORT
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
    self._navi_event_lock = threading.Lock()
    self._last_navi_event: Optional[Dict[str, Any]] = None
    self._last_navi_event_by_type: Dict[str, Dict[str, Any]] = {}
    self._last_complex_crossroad: Dict[str, Any] = {}

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
            queue_carrot_exception_tmux_send("broadcast_version_info")

        rk.keep_time()
        frame += 1
      except Exception as e:
        print(f"broadcast_version_info error...: {e}")
        traceback.print_exc()
        queue_carrot_exception_tmux_send("broadcast_version_info")
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
    msg['navi_debug'] = 0
    msg['navi_http_port'] = self.carrot_navi_http_port
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
                  queue_carrot_exception_tmux_send("kisa_app_thread")
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
      subprocess.run("rm -f /data/media/tmux.log; tmux capture-pane -pq -S-1000 > /data/media/tmux.log", shell=True, capture_output=True, text=False, check=True)
      subprocess.run("/data/openpilot/selfdrive/apilot.py", shell=True, capture_output=True, text=False)
      return True
    except Exception as e:
      print(f"TMUX creation error: {e}")
      return False

  def send_tmux(self, ftp_password, tmux_why, send_settings=False):
    ftp_server = "shind0.synology.me"
    ftp_port = 8021
    ftp_username = "carrotpilot"
    ftp = FTP(timeout=10)
    try:
      ftp.connect(ftp_server, ftp_port, timeout=10)
      ftp.login(ftp_username, ftp_password)
      car_selected = Params().get("CarName") or "none"

      git_branch = (Params().get("GitBranch") or "unknown").replace("/", "__")
      try:
        ftp.mkd(git_branch)
      except Exception as e:
        print(f"Directory creation failed: {e}")
      ftp.cwd(git_branch)

      directory = car_selected + " " + (Params().get("DongleId") or "unknown")
      current_time = datetime.now().strftime("%Y%m%d-%H%M%S")
      filename = tmux_why + "-" + current_time + "-" + git_branch + ".txt"

      try:
        ftp.mkd(directory)
      except Exception as e:
        print(f"Directory creation failed: {e}")
      ftp.cwd(directory)

      with open("/data/media/tmux.log", "rb") as file:
        ftp.storbinary(f'STOR {filename}', file)

      if send_settings:
        self.save_toggle_values()
        try:
          #with open("/data/backup_params.json", "rb") as file:
          with open("/data/toggle_values.json", "rb") as file:
            ftp.storbinary(f'STOR toggles-{current_time}.json', file)
        except Exception as e:
          print(f"ftp params sending error...: {e}")
      return True
    except Exception as e:
      print(f"ftp tmux sending error...: {e}")
      traceback.print_exc()
      return False
    finally:
      try:
        ftp.quit()
      except Exception:
        try:
          ftp.close()
        except Exception:
          pass

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
      "car_name"          : _pstr("CarName"),
      "git_branch"        : _pstr("GitBranch"),
      "github_id"         : _pstr("GithubUsername"),
      "git_remote"        : _pstr("GitRemote"),
      "git_commit"        : _pstr("GitCommit"),
      "git_commit_date"   : _pstr("GitCommitDate"),
      "dongle_id"         : _pstr("DongleId"),
      "device_serial"     : _pstr("HardwareSerial"),
      "local_ip"          : get_private_ip_by_iface("wlan0"),
    }

    params = {}
    headers = {}
    files = []

    try:
      files.append(("files[0]", ("tmux.log", open("/data/media/tmux.log", "rb"), "text/plain")))

      if send_settings:
        #self.save_toggle_values()
        self.save_toggle_values()
        try:
          files.append(("files[1]",("toggle_values.json",open("/data/toggle_values.json", "rb"),"application/json")))
        except Exception as e:
          print(f"http params file open error...: {e}")

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
    except Exception as e:
      print(f"http tmux sending error...: {e}")
      traceback.print_exc()
      return None
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
    onroad_start_at = None
    onroad_tmux_captured = False
    onroad_tmux_next_attempt_at = 0.0
    pending_tmux_reason = None
    pending_tmux_next_attempt_at = 0.0

    print("#########carrot_cmd_zmq: thread started...")
    while True:
      try:
        now = time.monotonic()
        socks = dict(poller.poll(100))

        if socket in socks and socks[socket] == zmq.POLLIN:
          message = socket.recv(zmq.NOBLOCK)
          print(f"Received:7710 request: {message}")
          json_obj = json.loads(message.decode())
        else:
          json_obj = None

        if json_obj is None:
          is_onroad = self.params.get_bool("IsOnroad")
          if is_onroad:
            if onroad_start_at is None:
              onroad_start_at = now
              isOnroadCount = 1
              is_tmux_sent = False
              onroad_tmux_captured = False
              onroad_tmux_next_attempt_at = 0.0
              if AUTO_ONROAD_DIAGNOSTICS:
                self.show_panda_debug = True
            else:
              isOnroadCount += 1
          else:
            isOnroadCount = 0
            onroad_start_at = None
            is_tmux_sent = False
            onroad_tmux_captured = False
            onroad_tmux_next_attempt_at = 0.0

          network_type = self.sm['deviceState'].networkType # if not force_wifi else NetworkType.wifi
          networkConnected = False if network_type == NetworkType.none else True

          if AUTO_ONROAD_DIAGNOSTICS and onroad_start_at is not None and not is_tmux_sent:
            onroad_elapsed = now - onroad_start_at
            if not onroad_tmux_captured and onroad_elapsed >= AUTO_ONROAD_TMUX_DELAY_SECONDS and now >= onroad_tmux_next_attempt_at:
              if self.make_tmux_data():
                onroad_tmux_captured = True
                onroad_tmux_next_attempt_at = 0.0
                print(f"[carrot_man] onroad tmux captured after {onroad_elapsed:.1f}s; waiting for network upload")
              else:
                onroad_tmux_next_attempt_at = now + CARROT_EXCEPTION_UPLOAD_RETRY_SECONDS

            if onroad_tmux_captured and networkConnected and now >= onroad_tmux_next_attempt_at:
              ftp_ok = self.send_tmux("Ekdrmsvkdlffjt7710", "onroad", send_settings = True)
              http_response = self.send_tmux_http("onroad", send_settings = True)
              http_ok = http_response is not None and getattr(http_response, "ok", False)
              if ftp_ok or http_ok:
                print(f"[carrot_man] onroad tmux upload complete: ftp_ok={ftp_ok}, http_ok={http_ok}")
                is_tmux_sent = True
              else:
                onroad_tmux_next_attempt_at = now + CARROT_EXCEPTION_UPLOAD_RETRY_SECONDS
          carrot_exception = self.params.get("CarrotException")
          if carrot_exception in ["exception", "log", "tmux_send"] and pending_tmux_reason is None and now >= pending_tmux_next_attempt_at:
            if self.make_tmux_data():
              pending_tmux_reason = carrot_exception
              pending_tmux_next_attempt_at = 0.0
              print(f"[carrot_man] tmux captured for {carrot_exception}; waiting for network upload")
            else:
              pending_tmux_next_attempt_at = now + CARROT_EXCEPTION_UPLOAD_RETRY_SECONDS
              reset_carrot_exception_tmux_send_queue()

          if pending_tmux_reason is not None and networkConnected and now >= pending_tmux_next_attempt_at:
            ftp_ok = self.send_tmux("Ekdrmsvkdlffjt7710", pending_tmux_reason)
            http_response = self.send_tmux_http(pending_tmux_reason, send_settings = False)
            http_ok = http_response is not None and getattr(http_response, "ok", False)
            if ftp_ok or http_ok:
              print(f"[carrot_man] tmux upload complete for {pending_tmux_reason}: ftp_ok={ftp_ok}, http_ok={http_ok}")
              self.params.put("CarrotException", "")
              pending_tmux_reason = None
              pending_tmux_next_attempt_at = 0.0
              reset_carrot_exception_tmux_send_queue()
            else:
              pending_tmux_next_attempt_at = now + CARROT_EXCEPTION_UPLOAD_RETRY_SECONDS
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
          tmux_created = self.make_tmux_data()
          ftp_ok = self.send_tmux(json_obj['tmux_send'], "tmux_send") if tmux_created else False
          http_response = self.send_tmux_http("tmux_send") if tmux_created else None
          http_ok = http_response is not None and getattr(http_response, "ok", False)
          result = "success" if ftp_ok or http_ok else "failed"
          echo = json.dumps({"tmux_send": json_obj['tmux_send'], "result": result, "ftp_ok": ftp_ok, "http_ok": http_ok})
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
    original_count = len(coords)
    coords = limit_route_points(coords, NAVI_ROUTE_MAX_POINTS)
    if original_count > len(coords):
      print(f"Route points limited: {original_count} -> {len(coords)}")

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
        self.navi_points = []
        self.navi_points_start_index = 0
        self.navi_points_active = False
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
              self.navi_points_active = len(coords) > 0
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
              else:
                self.navi_points = []
                self.navi_points_start_index = 0
                self.navd_active = False
                self.params.remove("NavDestination")

            except Exception as e:
              print(e)
    except Exception as e:
      print("################# CarrotMan route server error #####################")
      print(e)

  def carrot_curve_speed_params(self):
    self.autoCurveSpeedFactor = self.params.get_int("AutoCurveSpeedFactor")*0.01

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
    adjusted_target_lat_a = TARGET_LAT_A

    # Get the target velocity for the maximum curve
    #turnSpeed = max(abs(adjusted_target_lat_a / max_curve)**0.5  * 3.6, self.autoCurveSpeedLowerLimit)
    turnSpeed = max(abs(adjusted_target_lat_a / max_curve)**0.5  * 3.6, 5)
    turnSpeed = min(turnSpeed, 250)
    return turnSpeed * curv_direction

  def carrot_navi_thread(self):
    self.carrot_navi_tcp_server(7712)

  def _route_point_to_lon_lat(self, point: Any):
    if isinstance(point, dict):
      if not point.get("valid", True):
        return None

      lon_value = point.get("x")
      lat_value = point.get("y")

      if lon_value is None:
        lon_value = point.get("lon", point.get("longitude"))
      if lat_value is None:
        lat_value = point.get("lat", point.get("latitude"))

      if lon_value is None or lat_value is None:
        return None

      try:
        lon = float(lon_value)
        lat = float(lat_value)
      except Exception:
        return None
      if not math.isfinite(lon) or not math.isfinite(lat):
        return None
      if not -180.0 <= lon <= 180.0 or not -90.0 <= lat <= 90.0:
        return None
      return lon, lat

    if isinstance(point, (list, tuple)) and len(point) >= 2:
      try:
        lon = float(point[0])
        lat = float(point[1])
      except Exception:
        return None
      if not math.isfinite(lon) or not math.isfinite(lat):
        return None
      if not -180.0 <= lon <= 180.0 or not -90.0 <= lat <= 90.0:
        return None
      return lon, lat

    return None

  def _extract_route_points(self, payload: Any, depth: int = 0):
    if payload is None:
      return []
    if depth > 8:
      return None

    if isinstance(payload, dict):
      for key in ("vrtx", "vertices", "vertexes", "coordinates", "coords", "points", "path", "route"):
        value = payload.get(key)
        if value is not None:
          return self._extract_route_points(value, depth + 1)

      point = self._route_point_to_lon_lat(payload)
      return [point] if point is not None else None

    if not isinstance(payload, list):
      return None

    points = []
    for point in payload:
      lon_lat = self._route_point_to_lon_lat(point)
      if lon_lat is not None:
        points.append(lon_lat)

    return points

  def _limited_route_points(self, points: List[tuple]):
    return limit_route_points(points, NAVI_ROUTE_MAX_POINTS)

  def _route_payload_for_summary(self, payload: Any, depth: int = 0):
    if payload is None or depth > 8:
      return None
    if isinstance(payload, dict):
      for key in ("vrtx", "vertices", "vertexes", "coordinates", "coords", "points", "path", "route"):
        value = payload.get(key)
        if value is not None:
          return self._route_payload_for_summary(value, depth + 1)
    return payload

  def _route_summary(self, payload: Any) -> Dict[str, Any]:
    payload = self._route_payload_for_summary(payload)
    first = None
    last = None
    count = 0
    truncated = False

    if isinstance(payload, list):
      for point in payload:
        lon_lat = self._route_point_to_lon_lat(point)
        if lon_lat is None:
          continue
        if first is None:
          first = lon_lat
        last = lon_lat
        count += 1
        if count >= NAVI_ROUTE_SUMMARY_MAX_SCAN:
          truncated = True
          break
    else:
      lon_lat = self._route_point_to_lon_lat(payload)
      if lon_lat is not None:
        first = lon_lat
        last = lon_lat
        count = 1

    summary: Dict[str, Any] = {"pointCount": count}
    if first is not None:
      summary["first"] = {"lon": first[0], "lat": first[1]}
    if last is not None:
      summary["last"] = {"lon": last[0], "lat": last[1]}
    if truncated:
      summary["truncated"] = True
    return summary

  def handle_route(self, payload: Any):
    points = self._extract_route_points(payload)
    if points is None:
      print(f"Received route: unsupported payload type={type(payload).__name__}")
      return
    points = self._limited_route_points(points)

    arr = [{"x": lon, "y": lat, "valid": True} for lon, lat in points]
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

  def _put_traffic_light(self, lamp: str, remain: Any, distance: Any = 0, lat: Any = None, lon: Any = None):
    try:
      remain_int = int(float(remain or 0))
    except Exception:
      remain_int = 0

    if remain_int <= 0:
      return

    try:
      distance_int = int(float(distance or 0))
    except Exception:
      distance_int = 0

    traffic_light = {
      "distance": distance_int,
      "lamp": lamp,
      "remain": remain_int,
      "ts": time.monotonic(),
    }

    try:
      if lat is not None:
        traffic_light["lat"] = float(lat)
      if lon is not None:
        traffic_light["lon"] = float(lon)
    except Exception:
      pass

    self.params_memory.put_nonblocking("TrafficLight", json.dumps(traffic_light))

  def handle_traffic_light(self, d: dict):
    if not isinstance(d, dict):
      return

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

    location = d.get("location", {})
    lat = None
    lon = None
    try:
      if isinstance(location, dict):
        if location.get("latitude") is not None:
          lat = float(location.get("latitude"))
        if location.get("longitude") is not None:
          lon = float(location.get("longitude"))
    except Exception:
      pass
    self._put_traffic_light(lamp, remain, d.get("distance", 0), lat, lon)

  def handle_traffic_light_detail(self, d: dict):
    if not isinstance(d, dict):
      return

    green_checks = (
      ("left", "left", "left_remain_time"),
      ("straight", "green", "straight_remain_time"),
      ("right", "right", "right_remain_time"),
      ("uturn", "uturn", "uturn_remain_time"),
    )
    for field, lamp, remain_field in green_checks:
      if str(d.get(field, "")).upper() == "GREEN_LIGHT_ON":
        self._put_traffic_light(lamp, d.get(remain_field, 0), d.get("distance", 0), d.get("lat"), d.get("lon"))
        return

    red_remain = 0
    for field in ("straight", "left", "right", "uturn"):
      if str(d.get(field, "")).upper() == "RED_LIGHT_ON":
        try:
          red_remain = max(red_remain, int(d.get(f"{field}_remain_time", 0) or 0))
        except Exception:
          pass

    if red_remain > 0:
      self._put_traffic_light("red", red_remain, d.get("distance", 0), d.get("lat"), d.get("lon"))

  def handle_complex_crossroad(self, d: dict):
    if not isinstance(d, dict):
      return

    image_base64 = d.get("imageBase64")
    image_hash = ""
    if isinstance(image_base64, str) and image_base64:
      digest = hashlib.sha256()
      for index in range(0, len(image_base64), 65536):
        digest.update(image_base64[index:index + 65536].encode("ascii", "ignore"))
      image_hash = digest.hexdigest()[:16]

    summary = {
      "show": bool(d.get("show", False)),
      "imageUrl": str(d.get("imageUrl", "")),
      "imageMime": str(d.get("imageMime", "")),
      "imageEncoding": str(d.get("imageEncoding", "")),
      "imageWidth": self._safe_int_or_none(d.get("imageWidth"), minimum=0) or 0,
      "imageHeight": self._safe_int_or_none(d.get("imageHeight"), minimum=0) or 0,
      "totalMeters": self._safe_int_or_none(d.get("totalMeters"), minimum=0) or 0,
      "remainRatio": self._safe_float_or_none(d.get("remainRatio"), minimum=0.0, maximum=1.0) or 0.0,
      "imageHash": image_hash,
      "ts": time.monotonic(),
    }
    self._last_complex_crossroad = summary
    self._write_navi_image_param(d, image_hash)


  def _safe_int_or_none(self, value: Any, minimum: Optional[int] = None, maximum: Optional[int] = None) -> Optional[int]:
    try:
      int_value = int(float(value))
    except Exception:
      return None
    if not math.isfinite(int_value):
      return None
    if minimum is not None and int_value < minimum:
      return None
    if maximum is not None and int_value > maximum:
      return None
    return int_value

  def _safe_float_or_none(self, value: Any, minimum: Optional[float] = None, maximum: Optional[float] = None) -> Optional[float]:
    try:
      float_value = float(value)
    except Exception:
      return None
    if not math.isfinite(float_value):
      return None
    if minimum is not None and float_value < minimum:
      return None
    if maximum is not None and float_value > maximum:
      return None
    return float_value

  def _remaining_time(self, value: Any) -> Optional[int]:
    return self._safe_int_or_none(value, minimum=1, maximum=999)

  def _traffic_light_debug_from_sinf(self, sinf: dict) -> Dict[str, Any]:
    return {
      "distanceM": self._safe_int_or_none(sinf.get("distance"), minimum=0),
      "redS": self._remaining_time(sinf.get("redLightRemainTime")),
      "straightS": self._remaining_time(sinf.get("greenLightRemainTime")),
      "leftS": self._remaining_time(sinf.get("leftLightRemainTime")),
      "rightS": self._remaining_time(sinf.get("rightLightRemainTime")),
      "uturnS": self._remaining_time(sinf.get("uturnLightRemainTime")),
      "redOn": bool(sinf.get("redLightOn")),
      "straightOn": bool(sinf.get("greenLightOn")),
      "leftOn": bool(sinf.get("leftLightOn")),
      "rightOn": bool(sinf.get("rightLightOn")),
      "uturnOn": bool(sinf.get("uturnLightOn")),
    }

  def _traffic_light_debug_from_ssinf(self, ssinf: dict) -> Dict[str, Any]:
    red_remaining = []
    for signal_key, remain_key in (
      ("straight", "straight_remain_time"),
      ("left", "left_remain_time"),
      ("right", "right_remain_time"),
      ("uturn", "uturn_remain_time"),
    ):
      if str(ssinf.get(signal_key, "")).upper() == "RED_LIGHT_ON":
        remaining = self._remaining_time(ssinf.get(remain_key))
        if remaining is not None:
          red_remaining.append(remaining)
    return {
      "distanceM": self._safe_int_or_none(ssinf.get("distance"), minimum=0),
      "redS": max(red_remaining) if red_remaining else None,
      "straightS": self._remaining_time(ssinf.get("straight_remain_time")),
      "leftS": self._remaining_time(ssinf.get("left_remain_time")),
      "rightS": self._remaining_time(ssinf.get("right_remain_time")),
      "uturnS": self._remaining_time(ssinf.get("uturn_remain_time")),
      "redOn": bool(red_remaining),
      "straightOn": str(ssinf.get("straight", "")).upper() == "GREEN_LIGHT_ON",
      "leftOn": str(ssinf.get("left", "")).upper() == "GREEN_LIGHT_ON",
      "rightOn": str(ssinf.get("right", "")).upper() == "GREEN_LIGHT_ON",
      "uturnOn": str(ssinf.get("uturn", "")).upper() == "GREEN_LIGHT_ON",
    }

  def _write_navi_image_param(self, crossroad: dict, image_hash: str):
    image_base64 = crossroad.get("imageBase64")
    if not isinstance(image_base64, str):
      image_base64 = ""
    image_too_large = len(image_base64) > NAVI_IMAGE_BASE64_MAX_CHARS
    if image_too_large:
      print(f"navi image too large; metadata only size={len(image_base64)} hash={image_hash}")
      image_base64 = ""
    image = {
      "receivedMono": time.monotonic(),
      "show": bool(crossroad.get("show", False)),
      "imageBase64": image_base64,
      "imageMime": str(crossroad.get("imageMime", "")),
      "imageEncoding": str(crossroad.get("imageEncoding", "")),
      "imageWidth": self._safe_int_or_none(crossroad.get("imageWidth"), minimum=0) or 0,
      "imageHeight": self._safe_int_or_none(crossroad.get("imageHeight"), minimum=0) or 0,
      "imageHash": image_hash,
      "imageUrl": str(crossroad.get("imageUrl", "")),
      "imageTooLarge": image_too_large,
    }
    try:
      self.params_memory.put_nonblocking(NAVI_IMAGE_PARAM, json.dumps(image, ensure_ascii=False))
    except Exception as e:
      print(f"navi image param error: {e}")


  def handle_carrot_state(self, d: dict):
    try:
      self.carrot_serv.update(d)
    except Exception as e:
      print("carrot_state update error:", e)

  def handle_unknown(self, obj: Any):
    print("[UNKNOWN]", str(obj)[:200])

  def _detect_navi_event_type(self, obj: Any) -> str:
    if not isinstance(obj, dict):
      return "unknown"

    for key in NAVI_EVENT_TYPES:
      if obj.get(key) is not None:
        return key
    return "unknown"

  def _get_timestamp_ms(self, obj: Any) -> int:
    if not isinstance(obj, dict):
      return 0
    try:
      return int(obj.get("timestamp_ms") or obj.get("timestamp") or 0)
    except Exception:
      return 0

  def _summarize_navi_event(self, event_type: str, obj: Any) -> Dict[str, Any]:
    summary: Dict[str, Any] = {"type": event_type}
    if not isinstance(obj, dict):
      return summary

    if event_type == "rgdata" and isinstance(obj.get("rgdata"), dict):
      rgdata = obj["rgdata"]
      summary.update({
        "lat": rgdata.get("vpPosPointLat"),
        "lon": rgdata.get("vpPosPointLon"),
        "speed": rgdata.get("nPosSpeed"),
        "roadLimitSpeed": rgdata.get("nRoadLimitSpeed"),
        "tbtDist": rgdata.get("nTBTDist"),
        "tbtTurnType": rgdata.get("nTBTTurnType"),
        "sdiType": rgdata.get("nSdiType"),
        "sdiDist": rgdata.get("nSdiDist"),
      })
    elif event_type in ("vrtx", "route"):
      summary.update(self._route_summary(obj.get(event_type)))
    elif event_type == "sinf" and isinstance(obj.get("sinf"), dict):
      sinf = obj["sinf"]
      summary.update({
        "distance": sinf.get("distance"),
        "redLightOn": sinf.get("redLightOn"),
        "greenLightOn": sinf.get("greenLightOn"),
        "leftLightOn": sinf.get("leftLightOn"),
      })
    elif event_type == "ssinf" and isinstance(obj.get("ssinf"), dict):
      ssinf = obj["ssinf"]
      summary.update({
        "distance": ssinf.get("distance"),
        "straight": ssinf.get("straight"),
        "left": ssinf.get("left"),
        "straightRemain": ssinf.get("straight_remain_time"),
        "leftRemain": ssinf.get("left_remain_time"),
      })
    elif event_type == "complexCrossroad" and isinstance(obj.get("complexCrossroad"), dict):
      crossroad = obj["complexCrossroad"]
      image_base64 = crossroad.get("imageBase64")
      summary.update({
        "show": bool(crossroad.get("show", False)),
        "imageUrl": str(crossroad.get("imageUrl", ""))[:200],
        "imageMime": str(crossroad.get("imageMime", ""))[:64],
        "imageWidth": self._safe_int_or_none(crossroad.get("imageWidth"), minimum=0) or 0,
        "imageHeight": self._safe_int_or_none(crossroad.get("imageHeight"), minimum=0) or 0,
        "totalMeters": self._safe_int_or_none(crossroad.get("totalMeters"), minimum=0) or 0,
        "remainRatio": self._safe_float_or_none(crossroad.get("remainRatio"), minimum=0.0, maximum=1.0) or 0.0,
        "hasImageBase64": isinstance(image_base64, str) and bool(image_base64),
        "imageBase64Size": len(image_base64) if isinstance(image_base64, str) else 0,
      })
    else:
      summary["keys"] = list(obj.keys())[:10]
    return summary

  def _sdi_label(self, sdi_type: Any) -> str:
    try:
      sdi_type_int = int(sdi_type)
    except Exception:
      return ""
    labels = {
      0: "Signal speed enforcement",
      1: "Fixed speed camera",
      2: "Section control start",
      3: "Section control end",
      4: "Section control",
      7: "Mobile speed camera",
      8: "Speed camera zone",
      13: "Traffic data",
      17: "Parking enforcement",
      20: "School zone start",
      21: "School zone end",
      22: "Speed bump",
      29: "Accident-prone section",
      30: "Sharp curve",
      38: "Frequent speeding",
      63: "Drowsy rest area",
      84: "Road caution",
    }
    return labels.get(sdi_type_int, f"SDI type {sdi_type_int}")

  def _navi_debug_line(self, label: str, value: Any) -> str:
    text = "" if value is None else str(value)
    return f"{label}: {text}"[:120]

  def _navi_debug_from_event(self, obj: Any, event_type: str, event_time_ms: int) -> Dict[str, Any]:
    title = f"NAVI {event_type}"
    severity = "normal"
    lines: List[str] = []
    speed_limit_kph: Optional[int] = None
    traffic_light: Optional[Dict[str, Any]] = None

    if isinstance(obj, dict) and event_type == "rgdata" and isinstance(obj.get("rgdata"), dict):
      rgdata = self._normalize_rgdata(obj["rgdata"])
      sdi_type = rgdata.get("nSdiType")
      sdi_plus_type = rgdata.get("nSdiPlusType")
      if sdi_type in (0, 1, 2, 3, 4, 7, 8, 75, 76):
        severity = "warning"
      if sdi_type == 22 or sdi_plus_type == 22:
        severity = "caution"

      road_name = rgdata.get("szPosRoadName") or rgdata.get("szNearDirName") or ""
      tbt_text = rgdata.get("szTBTMainText") or rgdata.get("szNearDirName") or ""
      speed_limit_kph = self._safe_int_or_none(rgdata.get("nRoadLimitSpeed"), minimum=1, maximum=300)
      title = "NAVI rgdata"
      lines.extend((
        self._navi_debug_line("Road", road_name),
        self._navi_debug_line("Speed", f"{rgdata.get('nPosSpeed', '--')} / limit {rgdata.get('nRoadLimitSpeed', '--')} km/h"),
        self._navi_debug_line("TBT", f"{tbt_text}  {rgdata.get('nTBTDist', '--')}m type {rgdata.get('nTBTTurnType', '--')}"),
        self._navi_debug_line("SDI", f"{self._sdi_label(sdi_type)}  {rgdata.get('nSdiDist', '--')}m limit {rgdata.get('nSdiSpeedLimit', '--')}"),
      ))
      if sdi_plus_type not in (None, 0, -1):
        lines.append(self._navi_debug_line("SDI+", f"{self._sdi_label(sdi_plus_type)}  {rgdata.get('nSdiPlusDist', '--')}m"))
      if rgdata.get("nLaneCount") is not None or rgdata.get("currentLane") is not None:
        lines.append(self._navi_debug_line("Lane", f"{rgdata.get('currentLane', '--')}/{rgdata.get('nLaneCount', '--')} rec {rgdata.get('recommendedLaneNumbers', '--')}"))

    elif isinstance(obj, dict) and event_type in ("vrtx", "route"):
      points = self._extract_route_points(obj.get(event_type))
      title = "NAVI route"
      if points:
        lines.extend((
          self._navi_debug_line("Route points", len(points)),
          self._navi_debug_line("First", f"{points[0][1]:.6f}, {points[0][0]:.6f}"),
          self._navi_debug_line("Last", f"{points[-1][1]:.6f}, {points[-1][0]:.6f}"),
        ))
      else:
        lines.append("Route points: 0")

    elif isinstance(obj, dict) and event_type == "sinf" and isinstance(obj.get("sinf"), dict):
      sinf = obj["sinf"]
      title = "Traffic light"
      traffic_light = self._traffic_light_debug_from_sinf(sinf)
      if sinf.get("redLightOn"):
        severity = "stop"
      elif sinf.get("leftLightOn") or sinf.get("greenLightOn"):
        severity = "go"
      lines.extend((
        self._navi_debug_line("Distance", f"{sinf.get('distance', '--')}m"),
        self._navi_debug_line("Red", f"{sinf.get('redLightOn')} {sinf.get('redLightRemainTime', '--')}s"),
        self._navi_debug_line("Green", f"{sinf.get('greenLightOn')} {sinf.get('greenLightRemainTime', '--')}s"),
        self._navi_debug_line("Left", f"{sinf.get('leftLightOn')} {sinf.get('leftLightRemainTime', '--')}s"),
      ))

    elif isinstance(obj, dict) and event_type == "ssinf" and isinstance(obj.get("ssinf"), dict):
      ssinf = obj["ssinf"]
      title = "Traffic light detail"
      traffic_light = self._traffic_light_debug_from_ssinf(ssinf)
      red_active = any(str(ssinf.get(key, "")).upper() == "RED_LIGHT_ON" for key in ("straight", "left", "right", "uturn"))
      green_active = any(str(ssinf.get(key, "")).upper() == "GREEN_LIGHT_ON" for key in ("straight", "left", "right", "uturn"))
      severity = "stop" if red_active else "go" if green_active else "normal"
      lines.extend((
        self._navi_debug_line("Distance", f"{ssinf.get('distance', '--')}m"),
        self._navi_debug_line("Straight", f"{ssinf.get('straight', '--')} {ssinf.get('straight_remain_time', '--')}s"),
        self._navi_debug_line("Left", f"{ssinf.get('left', '--')} {ssinf.get('left_remain_time', '--')}s"),
        self._navi_debug_line("Right", f"{ssinf.get('right', '--')} {ssinf.get('right_remain_time', '--')}s"),
      ))

    elif isinstance(obj, dict) and event_type == "complexCrossroad" and isinstance(obj.get("complexCrossroad"), dict):
      crossroad = obj["complexCrossroad"]
      title = "Complex crossroad"
      severity = "caution" if crossroad.get("show") else "normal"
      lines.extend((
        self._navi_debug_line("Show", crossroad.get("show")),
        self._navi_debug_line("Image", f"{crossroad.get('imageWidth', '--')}x{crossroad.get('imageHeight', '--')} {crossroad.get('imageMime', '')}"),
        self._navi_debug_line("Progress", f"{crossroad.get('totalMeters', '--')}m ratio {crossroad.get('remainRatio', '--')}"),
        self._navi_debug_line("URL", crossroad.get("imageUrl", "")),
      ))

    else:
      keys = list(obj.keys())[:10] if isinstance(obj, dict) else []
      lines.append(self._navi_debug_line("Keys", ", ".join(keys)))

    return {
      "receivedMono": time.monotonic(),
      "eventTimeMs": event_time_ms,
      "type": event_type,
      "title": title,
      "severity": severity,
      "lines": [line for line in lines if line],
      "speedLimitKph": speed_limit_kph,
      "trafficLight": traffic_light,
    }

  def _write_navi_debug_param(self, obj: Any, event_type: str, event_time_ms: int):
    try:
      debug = self._navi_debug_from_event(obj, event_type, event_time_ms)
      self.params_memory.put_nonblocking(NAVI_DEBUG_PARAM, json.dumps(debug, ensure_ascii=False))
    except Exception as e:
      print(f"navi debug param error: {e}")

  def _store_navi_event(self, obj: Any, event_type: str, event_time_ms: int):
    event = {
      "receivedAt": datetime.now().astimezone().isoformat(timespec="milliseconds"),
      "eventTimeMs": event_time_ms,
      "type": event_type,
      "summary": self._summarize_navi_event(event_type, obj),
    }
    with self._navi_event_lock:
      self._last_navi_event = event
      self._last_navi_event_by_type[event_type] = event

  def _normalize_rgdata(self, rgdata: Any):
    if not isinstance(rgdata, dict):
      return rgdata

    merged = dict(rgdata)
    for group_key in ("guidance", "sdi", "lane"):
      group = rgdata.get(group_key)
      if isinstance(group, dict):
        for key, value in group.items():
          merged.setdefault(key, value)
    return merged


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

    event_type = self._detect_navi_event_type(obj)
    event_time_ms = self._get_timestamp_ms(obj)
    try:
      self._store_navi_event(obj, event_type, event_time_ms)
    except Exception as e:
      print(f"navi event store error: {e}")

    handled = False

    if "complexCrossroad" in obj:
      self._safe_dispatch_handler("complexCrossroad", self.handle_complex_crossroad, obj["complexCrossroad"])
      handled = True

    if "rgdata" in obj:
      stale, last_ts = self._is_stale_rgdata(event_time_ms)
      if stale:
        print(f"[STALE DROP] rgdata ts={event_time_ms} <= last={last_ts}")
      else:
        self._safe_dispatch_handler("rgdata", self.handle_carrot_state, self._normalize_rgdata(obj["rgdata"]))
      handled = True

    if "vrtx" in obj:
      self._safe_dispatch_handler("vrtx", self.handle_route, obj["vrtx"])
      handled = True

    if "ssinf" in obj:
      self._safe_dispatch_handler("ssinf", self.handle_traffic_light_detail, obj["ssinf"])
      handled = True

    if "sinf" in obj:
      self._safe_dispatch_handler("sinf", self.handle_traffic_light, obj["sinf"])
      handled = True

    if "route" in obj:
      self._safe_dispatch_handler("route", self.handle_route, obj["route"])
      handled = True

    if handled:
      self._write_navi_debug_param(obj, event_type, event_time_ms)

    if not handled:
      self.handle_unknown({"type": event_type, "keys": list(obj.keys())[:10]})

  def _safe_dispatch_handler(self, label: str, handler: Any, *args: Any):
    try:
      return handler(*args)
    except Exception as e:
      print(f"navi {label} handler error: {e}")
      traceback.print_exc()
      queue_carrot_exception_tmux_send(f"navi {label} handler")
      return None

  def carrot_navi_http_thread(self):
    while True:
      try:
        asyncio.run(self.carrot_navi_http_server(self.carrot_navi_http_port))
      except Exception as e:
        print(f"navi http server error: {e}")
        traceback.print_exc()
        queue_carrot_exception_tmux_send("navi http server")
        time.sleep(2)

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
      raw_body = (await request.text()).strip()
      if not raw_body:
        raise ValueError("empty body")
      obj = json.loads(raw_body)
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
    if isinstance(peer, tuple) and len(peer) >= 1 and peer[0]:
      self.remote_addr = (peer[0], self.broadcast_port)

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
      queue_carrot_exception_tmux_send("navi http dispatch")
      return web.json_response({
        "ok": False,
        "error": str(e),
        "tmap_version": tmap_version
      }, status=500)

  async def carrot_http_health(self, request: web.Request):
    with self._navi_event_lock:
      last_event = self._last_navi_event
      by_type = dict(self._last_navi_event_by_type)

    last_summary = None
    if last_event is not None:
      last_summary = {
        "receivedAt": last_event["receivedAt"],
        "eventTimeMs": last_event["eventTimeMs"],
        "summary": last_event.get("summary", {}),
      }

    return web.json_response({
      "ok": True,
      "service": "carrot_navi_http",
      "lastEvent": last_summary,
      "receivedTypes": sorted(by_type.keys()),
    })

  async def carrot_navi_http_server(self, port: int = NAVI_HTTP_PORT):
    app = web.Application(client_max_size=NAVI_HTTP_MAX_BODY_SIZE)

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
  threading.Thread(target=carrot_man.kisa_app_thread, daemon=True).start()
  threading.Thread(target=carrot_man.carrot_navi_thread, daemon=True).start()
  threading.Thread(target=carrot_man.carrot_navi_http_thread, daemon=True).start()

  while True:
    try:
      carrot_man.carrot_man_thread()
    except Exception as e:
      print(f"carrot_man error...: {e}")
      traceback.print_exc()
      queue_carrot_exception_tmux_send("carrot_man_thread")
      time.sleep(10)


if __name__ == "__main__":
  main()
