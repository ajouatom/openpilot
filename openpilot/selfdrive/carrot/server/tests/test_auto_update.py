import asyncio

from openpilot.selfdrive.carrot.server.services import auto_update
from openpilot.selfdrive.carrot.server.services.auto_update import (
  AUTO_REBOOT_DISENGAGED,
  AUTO_REBOOT_OFF,
  AUTO_REBOOT_PARK,
  AutoRebootCondition,
)


def test_park_reboot_requires_valid_disengaged_park_state():
  condition = AutoRebootCondition(AUTO_REBOOT_PARK)

  assert not condition.update(now=0.0, selfdrive_valid=False, engaged=False, car_state_valid=True, gear_shifter="park")
  assert not condition.update(now=0.1, selfdrive_valid=True, engaged=True, car_state_valid=True, gear_shifter="park")
  assert not condition.update(now=0.2, selfdrive_valid=True, engaged=False, car_state_valid=True, gear_shifter="drive")
  assert condition.update(now=0.3, selfdrive_valid=True, engaged=False, car_state_valid=True, gear_shifter="park")


def test_disengaged_reboot_requires_one_continuous_second():
  condition = AutoRebootCondition(AUTO_REBOOT_DISENGAGED)

  assert not condition.update(now=10.0, selfdrive_valid=True, engaged=False)
  assert not condition.update(now=10.9, selfdrive_valid=True, engaged=False)
  assert condition.update(now=11.0, selfdrive_valid=True, engaged=False)


def test_disengaged_timer_resets_when_state_is_engaged_or_invalid():
  condition = AutoRebootCondition(AUTO_REBOOT_DISENGAGED)

  assert not condition.update(now=1.0, selfdrive_valid=True, engaged=False)
  assert not condition.update(now=1.8, selfdrive_valid=True, engaged=True)
  assert not condition.update(now=2.0, selfdrive_valid=True, engaged=False)
  assert not condition.update(now=2.8, selfdrive_valid=False, engaged=False)
  assert not condition.update(now=3.0, selfdrive_valid=True, engaged=False)
  assert condition.update(now=4.0, selfdrive_valid=True, engaged=False)


def test_off_mode_never_requests_reboot():
  condition = AutoRebootCondition(AUTO_REBOOT_OFF)

  assert not condition.update(now=100.0, selfdrive_valid=True, engaged=False, car_state_valid=True, gear_shifter="park")


def test_git_pull_reports_success_without_arming_reboot_when_nothing_changed(monkeypatch):
  responses = iter([
    (0, "old-head"),
    (0, "HEAD is now at old-head"),
    (0, "Already up to date."),
  ])

  async def fake_git(args, timeout):
    del args, timeout
    return next(responses)

  monkeypatch.setattr(auto_update, "_git", fake_git)

  assert asyncio.run(auto_update._run_git_pull()) == (True, False)


def test_git_pull_reports_actual_update_and_records_pull_time(monkeypatch):
  responses = iter([
    (0, "old-head"),
    (0, "HEAD is now at old-head"),
    (0, "Updating old-head..new-head\nFast-forward\n 2 files changed"),
  ])
  recorded = []

  async def fake_git(args, timeout):
    del args, timeout
    return next(responses)

  async def fake_notify(old_head):
    recorded.append(("notify", old_head))

  monkeypatch.setattr(auto_update, "_git", fake_git)
  monkeypatch.setattr(auto_update, "write_git_pull_time", lambda: recorded.append(("time", None)))
  monkeypatch.setattr(auto_update, "_notify_cwp", fake_notify)

  assert asyncio.run(auto_update._run_git_pull()) == (True, True)
  assert recorded == [("time", None), ("notify", "old-head")]
