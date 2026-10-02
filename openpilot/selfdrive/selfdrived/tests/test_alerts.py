import copy
import json
import os
import random
import string
from PIL import Image, ImageDraw, ImageFont

from openpilot.cereal import log, car
from openpilot.cereal.messaging import SubMaster
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
import openpilot.selfdrive.selfdrived.events as events_module
from openpilot.selfdrive.selfdrived.events import Alert, Events, EVENTS, ET
from openpilot.selfdrive.selfdrived.alertmanager import set_offroad_alert
from openpilot.selfdrive.test.process_replay.process_replay import CONFIGS
from openpilot.selfdrive.ui.translations.potools import extract_strings, parse_po
from openpilot.selfdrive.ui.update_translations import ALERTS_FILE, ALERT_TRANSLATION_CALL_ARGS
from openpilot.system.ui.lib.multilang import TRANSLATIONS_DIR

AlertSize = log.SelfdriveState.AlertSize

class TestElantraSteeringWarningDelay:
  @staticmethod
  def callback_args(**changes):
    from opendbc.car.hyundai.values import CAR, HyundaiFlags
    values = {"brand": "hyundai", "carFingerprint": CAR.HYUNDAI_ELANTRA,
              "steerControlType": "torque", "flags": int(HyundaiFlags.LEGACY)}
    return [car.CarParams(**(values | changes)), car.CarState(), None, False, 300, log.LongitudinalPersonality.standard]

  def test_warning_starts_after_twelve_completed_cycles(self):
    for event, kind in ((log.OnroadEvent.EventName.steerTempUnavailable, ET.SOFT_DISABLE),
                        (log.OnroadEvent.EventName.steerTempUnavailableSilent, ET.WARNING)):
      events = Events()
      args = self.callback_args()
      for cycle in range(14):
        events.add(event)
        assert events.contains(kind)  # Internal fault handling starts immediately.
        assert bool(events.create_alerts([kind], args)) == (cycle >= 12)
        events.clear()

  def test_short_pulse_clears_and_new_pulse_waits_again(self):
    events = Events()
    event = log.OnroadEvent.EventName.steerTempUnavailable
    for _ in range(3):
      for _ in range(10):
        events.add(event)
        assert not events.create_alerts([ET.SOFT_DISABLE], self.callback_args())
        events.clear()
      events.clear()  # One observed cycle with no event resets its counter.
      assert events.event_counters[event] == 0

  def test_no_entry_and_critical_takeover_are_immediate(self):
    events = Events()
    events.add(log.OnroadEvent.EventName.steerTempUnavailable)
    args = self.callback_args()
    assert events.create_alerts([ET.NO_ENTRY], args)
    args[4] = 49  # Existing soft-disable deadline is already near expiry.
    alerts = events.create_alerts([ET.SOFT_DISABLE], args)
    assert len(alerts) == 1 and alerts[0].alert_status == log.SelfdriveState.AlertStatus.critical

  def test_other_vehicles_keep_immediate_warning(self):
    from opendbc.car.hyundai.values import HyundaiFlags
    for changes in ({"brand": "kia"}, {"carFingerprint": "HYUNDAI_SONATA"}, {"steerControlType": "angle"},
                    {"flags": 0}, {"flags": int(HyundaiFlags.LEGACY | HyundaiFlags.CANFD)},
                    {"flags": int(HyundaiFlags.LEGACY | HyundaiFlags.ANGLE_CONTROL)}):
      events = Events()
      events.add(log.OnroadEvent.EventName.steerTempUnavailable)
      assert events.create_alerts([ET.SOFT_DISABLE], self.callback_args(**changes))

  def test_other_fault_warnings_are_immediate(self):
    for event, kind in ((log.OnroadEvent.EventName.steerUnavailable, ET.IMMEDIATE_DISABLE),
                        (log.OnroadEvent.EventName.canError, ET.IMMEDIATE_DISABLE),
                        (log.OnroadEvent.EventName.vehicleSensorsInvalid, ET.IMMEDIATE_DISABLE)):
      events = Events()
      events.add(event)
      assert events.create_alerts([kind], self.callback_args())


OFFROAD_ALERTS_PATH = os.path.join(BASEDIR, "openpilot/selfdrive/selfdrived/alerts_offroad.json")

# TODO: add callback alerts
ALERTS = []
for event_types in EVENTS.values():
  for alert in event_types.values():
    ALERTS.append(alert)


class TestAlerts:

  @classmethod
  def setup_class(cls):
    with open(OFFROAD_ALERTS_PATH) as f:
      cls.offroad_alerts = json.loads(f.read())

      # Create fake objects for callback
      cls.CS = car.CarState.new_message()
      cls.CP = car.CarParams.new_message()
      cfg = [c for c in CONFIGS if c.proc_name == 'selfdrived'][0]
      cls.sm = SubMaster(cfg.pubs)

  def test_events_defined(self):
    # Ensure all events in capnp schema are defined in events.py
    events = log.OnroadEvent.EventName.schema.enumerants

    for name, e in events.items():
      if not name.endswith("DEPRECATED"):
        fail_msg = f"{name} @{e} not in EVENTS"
        assert e in EVENTS.keys(), fail_msg

  def test_alerts_translated_at_creation(self, monkeypatch):
    event_name = log.OnroadEvent.EventName.startup
    source_alert = EVENTS[event_name][ET.PERMANENT]
    monkeypatch.setattr(events_module, "tr", lambda text: f"translated:{text}" if text else text)

    events = Events()
    events.add(event_name)
    translated_alert, = events.create_alerts([ET.PERMANENT])

    assert translated_alert is not source_alert
    assert translated_alert.alert_text_1 == f"translated:{source_alert.alert_text_1}"
    assert translated_alert.alert_text_2 == f"translated:{source_alert.alert_text_2}"
    assert not source_alert.alert_text_1.startswith("translated:")

  def test_alert_translation_catalogs_complete(self):
    alert_entries = extract_strings([ALERTS_FILE], BASEDIR, ALERT_TRANSLATION_CALL_ARGS)
    alert_sources = {entry.msgid for entry in alert_entries}
    formatter = string.Formatter()

    for language in ("ko", "zh-CHS"):
      _, entries = parse_po(TRANSLATIONS_DIR / f"app_{language}.po")
      catalog = {entry.msgid: entry for entry in entries}
      assert not (missing := alert_sources - catalog.keys()), f"{language} missing alert translations: {sorted(missing)}"

      untranslated = {
        source for source in alert_sources
        if not catalog[source].msgstr and not any(catalog[source].msgstr_plural.values())
      }
      assert not untranslated, f"{language} has untranslated alerts: {sorted(untranslated)}"

      for source in alert_sources:
        source_fields = {field for _, field, _, _ in formatter.parse(source) if field}
        translations = ([catalog[source].msgstr] if catalog[source].msgstr else catalog[source].msgstr_plural.values())
        for translation in translations:
          translated_fields = {field for _, field, _, _ in formatter.parse(translation) if field}
          assert source_fields == translated_fields, f"{language} placeholder mismatch: {source!r} -> {translation!r}"

  def test_no_legacy_event_source_swapping(self):
    launch_script = os.path.join(BASEDIR, "launch_chffrplus.sh")
    with open(launch_script, encoding="utf-8") as f:
      launch_source = f.read()

    for filename in ("events_ko.py", "events_zh.py", "events_en.py"):
      assert filename not in launch_source
      assert not os.path.exists(os.path.join(BASEDIR, "scripts", "add", filename))

  # ensure alert text doesn't exceed allowed width
  def test_alert_text_length(self):
    font_path = os.path.join(BASEDIR, "openpilot/selfdrive/assets/fonts")
    regular_font_path = os.path.join(font_path, "Inter-SemiBold.ttf")
    bold_font_path = os.path.join(font_path, "Inter-Bold.ttf")
    semibold_font_path = os.path.join(font_path, "Inter-SemiBold.ttf")

    max_text_width = 2160 - 300  # full screen width is usable, minus sidebar
    draw = ImageDraw.Draw(Image.new('RGB', (0, 0)))

    fonts = {
      AlertSize.small: [ImageFont.truetype(semibold_font_path, 74)],
      AlertSize.mid: [ImageFont.truetype(bold_font_path, 88),
                      ImageFont.truetype(regular_font_path, 66)],
    }

    for alert in ALERTS:
      if not isinstance(alert, Alert):
        alert = alert(self.CP, self.CS, self.sm, metric=False, soft_disable_time=100, personality=log.LongitudinalPersonality.standard)

      # for full size alerts, both text fields wrap the text,
      # so it's unlikely that they  would go past the max width
      if alert.alert_size in (AlertSize.none, AlertSize.full):
        continue

      for i, txt in enumerate([alert.alert_text_1, alert.alert_text_2]):
        if i >= len(fonts[alert.alert_size]):
          break

        font = fonts[alert.alert_size][i]
        left, _, right, _ = draw.textbbox((0, 0), txt, font)
        width = right - left
        msg = f"type: {alert.alert_type} msg: {txt}"
        assert width <= max_text_width, msg

  def test_alert_sanity_check(self):
    for event_types in EVENTS.values():
      for event_type, a in event_types.items():
        # TODO: add callback alerts
        if not isinstance(a, Alert):
          continue

        if a.alert_size == AlertSize.none:
          assert len(a.alert_text_1) == 0
          assert len(a.alert_text_2) == 0
        elif a.alert_size == AlertSize.small:
          assert len(a.alert_text_1) > 0
          assert len(a.alert_text_2) == 0
        elif a.alert_size == AlertSize.mid:
          assert len(a.alert_text_1) > 0
          assert len(a.alert_text_2) > 0
        else:
          assert len(a.alert_text_1) > 0

        assert a.duration >= 0.

        if event_type not in (ET.WARNING, ET.PERMANENT, ET.PRE_ENABLE):
          assert a.creation_delay == 0.

  def test_offroad_alerts(self):
    params = Params()
    for a in self.offroad_alerts:
      # set the alert
      alert = copy.copy(self.offroad_alerts[a])
      set_offroad_alert(a, True)
      alert['extra'] = ''
      assert alert == params.get(a)

      # then delete it
      set_offroad_alert(a, False)
      assert params.get(a) is None

  def test_offroad_alerts_extra_text(self):
    params = Params()
    for i in range(50):
      # set the alert
      a = random.choice(list(self.offroad_alerts))
      alert = self.offroad_alerts[a]
      set_offroad_alert(a, True, extra_text="a"*i)

      written_alert = params.get(a)
      assert "a"*i == written_alert['extra']
      assert alert["text"] == written_alert['text']
