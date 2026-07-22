from openpilot.cereal import car
from openpilot.cereal import messaging
from openpilot.cereal.messaging import SubMaster, PubMaster
from openpilot.selfdrive.ui.soundd import SELFDRIVE_STATE_TIMEOUT, Soundd, check_selfdrive_timeout_alert, sound_list

import os
import time
import wave

from openpilot.common.basedir import BASEDIR

AudibleAlert = car.CarControl.HUDControl.AudibleAlert


class TestSoundd:
  def test_radar_alert_sound_assets(self):
    expected = {
      AudibleAlert.radarCutin: ("radar_cutin.wav", 0.320),
      AudibleAlert.radarStationaryLead: ("radar_stationary_lead.wav", 0.280),
    }
    sound_dir = os.path.join(BASEDIR, "openpilot", "selfdrive", "assets", "sounds_eng")

    for alert, (filename, duration) in expected.items():
      assert sound_list[alert][:2] == (filename, 1)
      with wave.open(os.path.join(sound_dir, filename), "rb") as sound:
        assert sound.getnchannels() == 1
        assert sound.getsampwidth() == 2
        assert sound.getframerate() == 48000
        assert abs(sound.getnframes() / sound.getframerate() - duration) < 0.001

  def test_completed_one_shot_alert_returns_to_none(self):
    soundd = Soundd.__new__(Soundd)
    soundd.current_alert = AudibleAlert.radarCutin
    soundd.current_sound_frame = 10
    soundd.loaded_sounds = {AudibleAlert.radarCutin: [0] * 10}

    soundd.update_alert(AudibleAlert.none)

    assert soundd.current_alert == AudibleAlert.none
    assert soundd.current_sound_frame == 0

  def test_check_selfdrive_timeout_alert(self):
    sm = SubMaster(['selfdriveState'])
    pm = PubMaster(['selfdriveState'])

    for _ in range(100):
      cs = messaging.new_message('selfdriveState')
      cs.selfdriveState.enabled = True

      pm.send("selfdriveState", cs)

      time.sleep(0.01)

      sm.update(0)

      assert not check_selfdrive_timeout_alert(sm)

    for _ in range(SELFDRIVE_STATE_TIMEOUT * 110):
      sm.update(0)
      time.sleep(0.01)

    assert check_selfdrive_timeout_alert(sm)

  # TODO: add test with micd for checking that soundd actually outputs sounds

