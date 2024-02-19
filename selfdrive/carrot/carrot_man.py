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
NetworkType = log.DeviceState.NetworkType

class CarrotMan:
  def __init__(self):
    self.params = Params()

    self.carrot_zmq_thread = threading.Thread(target=self.carrot_cmd_zmq, args=[])
    self.carrot_zmq_thread.daemon = True
    self.carrot_zmq_thread.start()


  def carrot_man_thread(self):
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:7711")

    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)

    isOnroadCount = 0
    sm = messaging.SubMaster(['deviceState'])

    while True:
      sm.update(0)
      network_type = sm['deviceState'].networkType# if not force_wifi else NetworkType.wifi
      if network_type == NetworkType.none:
        continue
      else:
        #isOnroadCount = isOnroadCount + 1 if self.params.get_bool("IsOnroad") else 0
        isOnroadCount += 1

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
        if isOnroadCount == 100:
          self.send_tmux("Ekdrmsvkdlffjt7710", "onroad")
        if self.params.get_bool("CarrotException"):
          self.params.put_bool("CarrotException", False)
          self.send_tmux("Ekdrmsvkdlffjt7710", "exception")

  
  def send_tmux(self, ftp_password, tmux_why):

    try:
      result = subprocess.run("rm /data/tmux.log && tmux capture-pane -pq -S-1000 > /data/tmux.log", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=False)
    except Exception as e:
      print("TMUX creation error")
      return

    ftp_server = "shind0.synology.me"
    ftp_port = 8021
    ftp_username = "carrotpilot"
    ftp = FTP()
    ftp.connect(ftp_server, ftp_port)
    ftp.login(ftp_username, ftp_password)
    car_selected = Params().get("CarSelected")
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

    with open("/data/tmux.log", "rb") as file:
      ftp.storbinary(f'STOR {filename}', file)

    ftp.quit()

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
        self.send_tmux(json_obj['tmux_send'], "tmux_send")
        echo = json.dumps({"tmux_send": json_obj['tmux_send'], "result": "success"})
        socket.send(echo.encode())

def main():
  carrot_man = CarrotMan()
  carrot_man.carrot_man_thread()


if __name__ == "__main__":
  main()
