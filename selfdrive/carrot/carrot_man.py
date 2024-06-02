import numpy as np
import time
import threading
import zmq
import os
import subprocess
import json
from datetime import datetime

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
    self.show_panda_debug = False

    self.carrot_zmq_thread = threading.Thread(target=self.carrot_cmd_zmq, args=[])
    self.carrot_zmq_thread.daemon = True
    self.carrot_zmq_thread.start()

    self.carrot_panda_debug_thread = threading.Thread(target=self.carrot_panda_debug, args=[])
    self.carrot_panda_debug_thread.daemon = True
    self.carrot_panda_debug_thread.start()

  def carrot_man_thread(self):
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:7711")

    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)

    isOnroadCount = 0
    is_tmux_sent = False
    sm = messaging.SubMaster(['deviceState'])

    self.save_toggle_values()

    while True:
      try:
        sm.update(0)

        socks = dict(poller.poll(100))

        if socket in socks and socks[socket] == zmq.POLLIN:
          message = socket.recv(zmq.NOBLOCK)
          data = json.loads(message)
          print(f"Received request: {message}")
          response = {
              "status": "ok",
              "data": "Hello from Python ZeroMQ server!"
          }
          socket.send(json.dumps(response).encode('utf-8'))
        else:
          isOnroadCount = isOnroadCount + 1 if self.params.get_bool("IsOnroad") else 0
          if isOnroadCount == 0:
            is_tmux_sent = False
          if isOnroadCount == 1:
            self.show_panda_debug = True

          network_type = sm['deviceState'].networkType# if not force_wifi else NetworkType.wifi
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

      except Exception as e:
        print(f"carrot_man_thread: error...: {e}")

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
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:7710")

    while True:
      message = socket.recv()
      #print(f"Received request: {message}")
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

def main():
  print("CarrotManager Started")
  #print("Carrot GitBranch = {}, {}".format(Params().get("GitBranch"), Params().get("GitCommitDate")))
  carrot_man = CarrotMan()  
  while True:
    try:
      carrot_man.carrot_man_thread()
    except Exception as e:
      print(f"carrot_man error...: {e}")
      time.sleep(10)


if __name__ == "__main__":
  main()
