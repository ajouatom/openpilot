#!/usr/bin/env python3
import time
from cereal import car, log, messaging
from common.params import Params
from selfdrive.hardware import HARDWARE
from selfdrive.manager.process_config import managed_processes

if __name__ == "__main__":
  CP = car.CarParams(notCar=True)
  Params().put("CarParams", CP.to_bytes())

  procs = ['camerad', 'ui', 'modeld', 'calibrationd']

  HARDWARE.set_power_save(False)

  for p in procs:
    managed_processes[p].start()
  
  pm = messaging.PubMaster(['controlsState', 'deviceState', 'pandaStates', 'carParams', 'carState'])
  
  msgs = {s: messaging.new_message(s) for s in ['controlsState', 'deviceState', 'carParams']}
  msgs['deviceState'].deviceState.started = True
  msgs['carParams'].carParams.openpilotLongitudinalControl = True
  
  msgs['pandaStates'] = messaging.new_message('pandaStates', 1)
  msgs['pandaStates'].pandaStates[0].ignitionLine = True
  msgs['pandaStates'].pandaStates[0].pandaType = log.PandaState.PandaType.uno
  
  #speed = 0.
  try:
    while True:
      time.sleep(1 / 100)  # continually send, rate doesn't matter
      
      #msgs['carState'] = messaging.new_message('carState')
      #msgs['carState'].carState.cluSpeedMs = speed
      
      #speed += 0.02
      #if speed > 40.:
      #  speed = 0.
      
      for s in msgs:
        pm.send(s, msgs[s])
  except KeyboardInterrupt:
    for p in procs:
      managed_processes[p].stop()
