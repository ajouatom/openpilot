import numpy as np
import time
import threading
import zmq
import os
import subprocess
import json

from openpilot.common.realtime import Ratekeeper
from openpilot.common.params import Params

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

    while True:
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
        pass

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

def main():
  carrot_man = CarrotMan()
  carrot_man.carrot_man_thread()


if __name__ == "__main__":
  main()
