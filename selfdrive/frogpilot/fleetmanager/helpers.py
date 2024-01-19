import os
import subprocess
from flask import render_template, request, session
from functools import wraps
from pathlib import Path
from openpilot.system.hardware import PC
from openpilot.system.hardware.hw import Paths
from openpilot.system.loggerd.uploader import listdir_by_creation
from tools.lib.route import SegmentName

# otisserv conversion
from common.params import Params
from urllib.parse import parse_qs, quote
import json
import requests

params = Params()

# path to openpilot screen recordings and error logs
if PC:
  SCREENRECORD_PATH = os.path.join(str(Path.home()), ".comma", "media", "0", "videos", "")
  ERROR_LOGS_PATH = os.path.join(str(Path.home()), ".comma", "community", "crashes", "")
else:
  SCREENRECORD_PATH = "/data/media/0/videos/"
  ERROR_LOGS_PATH = "/data/community/crashes/"


def list_files(path):
  return sorted(listdir_by_creation(path), reverse=True)


def is_valid_segment(segment):
  try:
    segment_to_segment_name(Paths.log_root(), segment)
    return True
  except AssertionError:
    return False


def segment_to_segment_name(data_dir, segment):
  fake_dongle = "ffffffffffffffff"
  return SegmentName(str(os.path.join(data_dir, fake_dongle + "|" + segment)))


def all_segment_names():
  segments = []
  for segment in listdir_by_creation(Paths.log_root()):
    try:
      segments.append(segment_to_segment_name(Paths.log_root(), segment))
    except AssertionError:
      pass
  return segments


def all_routes():
  segment_names = all_segment_names()
  route_names = [segment_name.route_name for segment_name in segment_names]
  route_times = [route_name.time_str for route_name in route_names]
  unique_routes = list(dict.fromkeys(route_times))
  return sorted(unique_routes, reverse=True)


def segments_in_route(route):
  segment_names = [segment_name for segment_name in all_segment_names() if segment_name.time_str == route]
  segments = [segment_name.time_str + "--" + str(segment_name.segment_num) for segment_name in segment_names]
  return segments


def ffmpeg_mp4_concat_wrap_process_builder(file_list, cameratype, chunk_size=1024*512):
  command_line = ["ffmpeg"]
  if not cameratype == "qcamera":
    command_line += ["-f", "hevc"]
  command_line += ["-r", "20"]
  command_line += ["-i", "concat:" + file_list]
  command_line += ["-c", "copy"]
  command_line += ["-map", "0"]
  if not cameratype == "qcamera":
    command_line += ["-vtag", "hvc1"]
  command_line += ["-f", "mp4"]
  command_line += ["-movflags", "empty_moov"]
  command_line += ["-"]
  return subprocess.Popen(
    command_line, stdout=subprocess.PIPE,
    bufsize=chunk_size
  )


def ffmpeg_mp4_wrap_process_builder(filename):
  """Returns a process that will wrap the given filename
     inside a mp4 container, for easier playback by browsers
     and other devices. Primary use case is streaming segment videos
     to the vidserver tool.
     filename is expected to be a pathname to one of the following
       /path/to/a/qcamera.ts
       /path/to/a/dcamera.hevc
       /path/to/a/ecamera.hevc
       /path/to/a/fcamera.hevc
  """
  basename = filename.rsplit("/")[-1]
  extension = basename.rsplit(".")[-1]
  command_line = ["ffmpeg"]
  if extension == "hevc":
    command_line += ["-f", "hevc"]
  command_line += ["-r", "20"]
  command_line += ["-i", filename]
  command_line += ["-c", "copy"]
  command_line += ["-map", "0"]
  if extension == "hevc":
    command_line += ["-vtag", "hvc1"]
  command_line += ["-f", "mp4"]
  command_line += ["-movflags", "empty_moov"]
  command_line += ["-"]
  return subprocess.Popen(
    command_line, stdout=subprocess.PIPE
  )


def ffplay_mp4_wrap_process_builder(file_name):
  command_line = ["ffmpeg"]
  command_line += ["-i", file_name]
  command_line += ["-c", "copy"]
  command_line += ["-map", "0"]
  command_line += ["-f", "mp4"]
  command_line += ["-movflags", "empty_moov"]
  command_line += ["-"]
  return subprocess.Popen(
    command_line, stdout=subprocess.PIPE
  )

def get_nav_active():
  if params.get("NavDestination", encoding='utf8') is not None:
    return True
  else:
    return False

def get_public_token():
  token = params.get("MapboxPublicKey", encoding='utf8')
  return token.strip() if token is not None else None

def get_app_token():
  token = params.get("MapboxSecretKey", encoding='utf8')
  return token.strip() if token is not None else None

def get_gmap_key():
  token = params.get("GMapKey", encoding='utf8')
  return token.strip() if token is not None else None

def get_SearchInput():
  SearchInput = params.get_int("SearchInput")
  return SearchInput

def get_PrimeType():
  PrimeType = params.get_int("PrimeType")
  return PrimeType

def get_last_lon_lat():
  last_pos = params.get("LastGPSPosition")
  l = json.loads(last_pos)
  return l["longitude"], l["latitude"]

def get_locations():
  data = params.get("ApiCache_NavDestinations", encoding='utf-8')
  return data

def parse_addr(postvars, lon, lat, valid_addr, token):
  addr = postvars.get("fav_val", [""])
  real_addr = None
  if addr != "favorites":
    try:
      dests = json.loads(params.get("ApiCache_NavDestinations", encoding='utf8').rstrip('\x00'))
    except TypeError:
      dests = json.loads("[]")
    for item in dests:
      if "label" in item and item["label"] == addr:
        lat, lon, real_addr = item["latitude"], item["longitude"], item["place_name"]
        break
  return (real_addr, lon, lat, real_addr is not None, token)

def search_addr(postvars, lon, lat, valid_addr, token):
  if "addr_val" in postvars:
    addr = postvars.get("addr_val")
    if addr != "":
      # Properly encode the address to handle spaces
      addr_encoded = quote(addr)
      query = f"https://api.mapbox.com/geocoding/v5/mapbox.places/{addr_encoded}.json?access_token={token}&limit=1"
      # focus on place around last gps position
      lngi, lati = get_last_lon_lat()
      query += "&proximity=%s,%s" % (lngi, lati)
      r = requests.get(query)
      if r.status_code != 200:
        return (addr, lon, lat, valid_addr, token)
      j = json.loads(r.text)
      if not j["features"]:
        return (addr, lon, lat, valid_addr, token)
      lon, lat = j["features"][0]["geometry"]["coordinates"]
      valid_addr = True
  return (addr, lon, lat, valid_addr, token)

def set_destination(postvars, valid_addr):
  if postvars.get("latitude") is not None and postvars.get("longitude") is not None:
    postvars["lat"] = postvars.get("latitude")
    postvars["lon"] = postvars.get("longitude")
    postvars["save_type"] = "recent"
    nav_confirmed(postvars)
    valid_addr = True
  else:
    addr = postvars.get("place_name")
    token = get_public_token()
    data, lon, lat, valid_addr, token = search_addr(addr, lon, lat, valid_addr, token)
    postvars["lat"] = lat
    postvars["lon"] = lon
    postvars["save_type"] = "recent"
    nav_confirmed(postvars)
    valid_addr= True
  return postvars, valid_addr

def nav_confirmed(postvars):
  if postvars is not None:
    lat = float(postvars.get("lat"))
    lng = float(postvars.get("lon"))
    save_type = postvars.get("save_type")
    name = postvars.get("name") if postvars.get("name") is not None else ""
    params.put("NavDestination", "{\"latitude\": %f, \"longitude\": %f, \"place_name\": \"%s\"}" % (lat, lng, name))
    if name == "":
      name =  str(lat) + "," + str(lng)
    new_dest = {"latitude": float(lat), "longitude": float(lng), "place_name": name}
    if save_type == "recent":
      new_dest["save_type"] = "recent"
    else:
      new_dest["save_type"] = "favorite"
      new_dest["label"] = save_type
    val = params.get("ApiCache_NavDestinations", encoding='utf8')
    if val is not None:
      val = val.rstrip('\x00')
    dests = [] if val is None else json.loads(val)
    # type idx
    type_label_ids = {"home": None, "work": None, "fav1": None, "fav2": None, "fav3": None, "recent": []}
    idx = 0
    for d in dests:
      if d["save_type"] == "favorite":
        type_label_ids[d["label"]] = idx
      else:
        type_label_ids["recent"].append(idx)
      idx += 1
    if save_type == "recent":
      id = None
      if len(type_label_ids["recent"]) > 10:
        dests.pop(type_label_ids["recent"][-1])
    else:
      id = type_label_ids[save_type]
    if id is None:
      dests.insert(0, new_dest)
    else:
      dests[id] = new_dest
    params.put("ApiCache_NavDestinations", json.dumps(dests).rstrip("\n\r"))

def public_token_input(postvars):
  if postvars is None or "pk_token_val" not in postvars or postvars.get("pk_token_val")[0] == "":
    return postvars
  else:
    token = postvars.get("pk_token_val").strip()
    if "pk." not in token:
      return postvars
    else:
        params.put("MapboxPublicKey", token)
  return token

def app_token_input(postvars):
  if postvars is None or "sk_token_val" not in postvars or postvars.get("sk_token_val")[0] == "":
    return postvars
  else:
    token = postvars.get("sk_token_val").strip()
    if "sk." not in token:
      return postvars
    else:
        params.put("MapboxSecretKey", token)
  return token

def gmap_key_input(postvars):
  if postvars is None or "gmap_key_val" not in postvars or postvars.get("gmap_key_val")[0] == "":
    return postvars
  else:
    token = postvars.get("gmap_key_val").strip()
    params.put("GMapKey", token)
  return token
