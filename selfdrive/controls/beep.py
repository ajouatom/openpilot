#!/usr/bin/env python3
import subprocess
import time
from cereal import car, messaging
from openpilot.common.realtime import Ratekeeper
import threading

AudibleAlert = car.CarControl.HUDControl.AudibleAlert

class Beepd:
  def __init__(self):
    self.current_alert = AudibleAlert.none
    self.enable_gpio()
    self.startup_beep()

  def enable_gpio(self):
    # 尝试 export，忽略已 export 的错误
    try:
      subprocess.run("echo 42 | sudo tee /sys/class/gpio/export",
                     shell=True,
                     stderr=subprocess.DEVNULL,
                     stdout=subprocess.DEVNULL,
                     encoding='utf8')
    except Exception:
      pass
    subprocess.run("echo \"out\" | sudo tee /sys/class/gpio/gpio42/direction",
                   shell=True,
                   stderr=subprocess.DEVNULL,
                   stdout=subprocess.DEVNULL,
                   encoding='utf8')

  def _beep(self, on):
    val = "1" if on else "0"
    subprocess.run(f"echo \"{val}\" | sudo tee /sys/class/gpio/gpio42/value",
                   shell=True,
                   stderr=subprocess.DEVNULL,
                   stdout=subprocess.DEVNULL,
                   encoding='utf8')

  def engage(self):
    self._beep(True)
    time.sleep(0.05)
    self._beep(False)

  def disengage(self):
    for _ in range(2):
      self._beep(True)
      time.sleep(0.01)
      self._beep(False)
      time.sleep(0.01)

  def warning(self):
    for _ in range(3):
      self._beep(True)
      time.sleep(0.01)
      self._beep(False)
      time.sleep(0.01)

  def startup_beep(self):
    self._beep(True)
    time.sleep(0.1)
    self._beep(False)

  def ding(self):
    self._beep(True)
    time.sleep(0.02)
    self._beep(False)

  def dong(self):
    self._beep(True)
    time.sleep(0.03)
    self._beep(False)

  def beep(self):
    self._beep(True)
    time.sleep(0.04)
    self._beep(False)

  def dispatch_beep(self, func):
    threading.Thread(target=func, daemon=True).start()

  def update_alert(self, new_alert):
    if new_alert != self.current_alert:
      self.current_alert = new_alert
      print(f"[BEEP] New alert: {new_alert}")
      if new_alert == AudibleAlert.engage:
        self.dispatch_beep(self.engage)
      elif new_alert == AudibleAlert.disengage:
        self.dispatch_beep(self.disengage)
      elif new_alert in [AudibleAlert.refuse, AudibleAlert.prompt, AudibleAlert.warningImmediate,AudibleAlert.warningSoft]:
        self.dispatch_beep(self.warning)
      elif new_alert in [AudibleAlert.longEngaged, AudibleAlert.longDisengaged, AudibleAlert.trafficSignGreen, AudibleAlert.trafficSignChanged, AudibleAlert.trafficError, AudibleAlert.bsdWarning, AudibleAlert.laneChange]:
        self.dispatch_beep(self.ding)
      elif new_alert in [AudibleAlert.stopStop, AudibleAlert.stopping, AudibleAlert.autoHold, AudibleAlert.engage2, AudibleAlert.disengage2, AudibleAlert.speedDown, AudibleAlert.audioTurn, AudibleAlert.reverseGear]:
        self.dispatch_beep(self.dong)
      elif new_alert in [AudibleAlert.audio1, AudibleAlert.audio2, AudibleAlert.audio3, AudibleAlert.audio4, AudibleAlert.audio5,
                         AudibleAlert.audio6, AudibleAlert.audio7, AudibleAlert.audio8, AudibleAlert.audio9, AudibleAlert.audio10]:
        self.dispatch_beep(self.beep)

  def get_audible_alert(self, sm):
    if sm.updated['selfdriveState']:
      new_alert = sm['selfdriveState'].alertSound.raw
      self.update_alert(new_alert)

  def test_beepd_thread(self):
    frame = 0
    rk = Ratekeeper(20)
    pm = messaging.PubMaster(['selfdriveState'])
    while True:
      cs = messaging.new_message('selfdriveState')
      if frame == 40:
        cs.selfdriveState.alertSound = AudibleAlert.engage
      if frame == 60:
        cs.selfdriveState.alertSound = AudibleAlert.disengage
      if frame == 80:
        cs.selfdriveState.alertSound = AudibleAlert.prompt

      pm.send("selfdriveState", cs)
      frame += 1
      rk.keep_time()

  def beepd_thread(self, test=False):
    if test:
      threading.Thread(target=self.test_beepd_thread, daemon=True).start()

    sm = messaging.SubMaster(['selfdriveState'])
    rk = Ratekeeper(20)

    while True:
      sm.update(0)
      self.get_audible_alert(sm)
      rk.keep_time()

def main():
  s = Beepd()
  s.beepd_thread(test=False)  # 改成 True 可启用模拟测试数据

if __name__ == "__main__":
  main()
