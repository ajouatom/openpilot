import asyncio

from openpilot.selfdrive.carrot.server.services import auto_update, git_state
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


def test_offroad_reboot_requires_one_continuous_second_without_selfdrive_state():
  condition = AutoRebootCondition(AUTO_REBOOT_DISENGAGED)

  assert not condition.update(
    now=20.0,
    selfdrive_valid=False,
    engaged=False,
    device_state_valid=True,
    device_started=False,
  )
  assert not condition.update(
    now=20.9,
    selfdrive_valid=False,
    engaged=False,
    device_state_valid=True,
    device_started=False,
  )
  assert condition.update(
    now=21.0,
    selfdrive_valid=False,
    engaged=False,
    device_state_valid=True,
    device_started=False,
  )


def test_onroad_without_valid_selfdrive_state_does_not_reboot():
  condition = AutoRebootCondition(AUTO_REBOOT_DISENGAGED)

  assert not condition.update(
    now=30.0,
    selfdrive_valid=False,
    engaged=False,
    device_state_valid=True,
    device_started=True,
  )
  assert not condition.update(
    now=32.0,
    selfdrive_valid=False,
    engaged=False,
    device_state_valid=True,
    device_started=True,
  )


def test_off_mode_never_requests_reboot():
  condition = AutoRebootCondition(AUTO_REBOOT_OFF)

  assert not condition.update(now=100.0, selfdrive_valid=True, engaged=False, car_state_valid=True, gear_shifter="park")


def test_failed_fetch_never_uses_stale_behind_count():
  assert auto_update._verified_update_target({
    "available": False,
    "state": "fetch_error",
    "behind": 3,
    "target_head": "stale-head",
  }) == (0, "")
  assert auto_update._verified_update_target({
    "available": True,
    "state": "ok",
    "behind": 3,
    "target_head": "verified-head",
  }) == (3, "verified-head")


def test_git_pull_reports_success_without_arming_reboot_when_nothing_changed(monkeypatch):
  responses = iter([
    (0, "old-head"),
    (0, "HEAD is now at old-head"),
    (0, "Already up to date."),
    (0, "old-head"),
    (0, "old-head"),
  ])
  errors = []

  async def fake_git(args, timeout):
    del args, timeout
    return next(responses)

  monkeypatch.setattr(auto_update, "_git", fake_git)
  monkeypatch.setattr(auto_update, "read_auto_update_state", dict)
  monkeypatch.setattr(auto_update, "write_auto_update_event", lambda status, **fields: {"status": status, **fields})
  monkeypatch.setattr(auto_update, "_record_error", lambda code, detail, **fields: errors.append((code, detail, fields)))

  assert asyncio.run(auto_update._run_git_pull("old-head")) == (True, False, "old-head")
  assert errors[0][0] == "head_unchanged"


def test_git_pull_reports_actual_update_and_records_pull_time(monkeypatch):
  responses = iter([
    (0, "old-head"),
    (0, "HEAD is now at old-head"),
    (0, "Updating old-head..new-head\nFast-forward\n 2 files changed"),
    (0, "new-head"),
    (0, "new-head"),
  ])
  recorded = []

  async def fake_git(args, timeout):
    del args, timeout
    return next(responses)

  async def fake_notify(old_head):
    recorded.append(("notify", old_head))

  monkeypatch.setattr(auto_update, "_git", fake_git)
  monkeypatch.setattr(auto_update, "read_auto_update_state", dict)
  monkeypatch.setattr(auto_update, "write_auto_update_event", lambda status, **fields: {"status": status, **fields})
  monkeypatch.setattr(auto_update, "_set_auto_update_alert", lambda show, detail="": recorded.append(("alert", show)))
  monkeypatch.setattr(auto_update, "write_git_pull_time", lambda: recorded.append(("time", None)))
  monkeypatch.setattr(auto_update, "_notify_cwp", fake_notify)

  assert asyncio.run(auto_update._run_git_pull("new-head")) == (True, True, "new-head")
  assert recorded == [("alert", False), ("time", None), ("notify", "old-head")]


def test_git_pull_failure_is_persisted_and_never_reports_update(monkeypatch):
  responses = iter([
    (0, "old-head"),
    (0, "HEAD is now at old-head"),
    (1, "fatal: unable to access remote"),
  ])
  errors = []

  async def fake_git(args, timeout):
    del args, timeout
    return next(responses)

  monkeypatch.setattr(auto_update, "_git", fake_git)
  monkeypatch.setattr(auto_update, "read_auto_update_state", dict)
  monkeypatch.setattr(auto_update, "write_auto_update_event", lambda status, **fields: {"status": status, **fields})
  monkeypatch.setattr(auto_update, "_record_error", lambda code, detail, **fields: errors.append((code, detail, fields)))

  assert asyncio.run(auto_update._run_git_pull("new-head")) == (False, False, "")
  assert errors[0][0:2] == ("pull_failed", "fatal: unable to access remote")
  assert errors[0][2]["old_head"] == "old-head"
  assert errors[0][2]["target_head"] == "new-head"
  assert errors[0][2]["reset_rc"] == 0
  assert errors[0][2]["pull_rc"] == 1


def test_git_pull_does_not_modify_repo_when_attempt_cannot_be_persisted(monkeypatch):
  calls = []
  errors = []

  async def fake_git(args, timeout):
    del timeout
    calls.append(args)
    return 0, "old-head"

  monkeypatch.setattr(auto_update, "_git", fake_git)
  monkeypatch.setattr(auto_update, "read_auto_update_state", dict)
  monkeypatch.setattr(auto_update, "write_auto_update_event", lambda status, **fields: {})
  monkeypatch.setattr(auto_update, "_record_error", lambda code, detail, **fields: errors.append((code, detail, fields)))

  assert asyncio.run(auto_update._run_git_pull("new-head")) == (False, False, "")
  assert calls == [["rev-parse", "HEAD"]]
  assert errors[0][0] == "state_write_failed"


def test_same_target_that_already_requested_reboot_is_blocked_before_git_pull(monkeypatch):
  errors = []

  async def fail_git(args, timeout):
    raise AssertionError(f"git should not run: {args} {timeout}")

  monkeypatch.setattr(auto_update, "_git", fail_git)
  monkeypatch.setattr(auto_update, "read_auto_update_state", lambda: {
    "status": "reboot_requested",
    "reboot_requested_head": "same-head",
  })
  monkeypatch.setattr(auto_update, "_record_error", lambda code, detail, **fields: errors.append((code, detail, fields)))

  assert asyncio.run(auto_update._run_git_pull("same-head")) == (False, False, "")
  assert errors[0][0] == "duplicate_reboot_blocked"
  assert errors[0][2]["blocked"] is True


def test_git_pull_requires_new_head_to_match_upstream(monkeypatch):
  responses = iter([
    (0, "old-head"),
    (0, "HEAD is now at old-head"),
    (0, "Updating old-head..unexpected-head\nFast-forward"),
    (0, "unexpected-head"),
    (0, "expected-head"),
  ])
  errors = []

  async def fake_git(args, timeout):
    del args, timeout
    return next(responses)

  monkeypatch.setattr(auto_update, "_git", fake_git)
  monkeypatch.setattr(auto_update, "read_auto_update_state", dict)
  monkeypatch.setattr(auto_update, "write_auto_update_event", lambda status, **fields: {"status": status, **fields})
  monkeypatch.setattr(auto_update, "_record_error", lambda code, detail, **fields: errors.append((code, detail, fields)))

  assert asyncio.run(auto_update._run_git_pull("expected-head")) == (True, False, "unexpected-head")
  assert errors[0][0] == "head_mismatch"


def test_auto_update_state_survives_as_bounded_history(tmp_path, monkeypatch):
  state_path = tmp_path / "git.json"
  monkeypatch.setattr(git_state, "CARROT_STATE_DIR", str(tmp_path))
  monkeypatch.setattr(git_state, "CARROT_GIT_STATE_PATH", str(state_path))

  requested = git_state.write_auto_update_event(
    "reboot_requested",
    target_head="target-head",
    reboot_requested_head="target-head",
  )
  assert requested["reboot_requested_head"] == "target-head"

  for index in range(git_state.AUTO_UPDATE_HISTORY_LIMIT + 3):
    assert git_state.write_auto_update_event("error", error_code=f"error-{index}")

  data = git_state.read_git_state()
  assert data["auto_update"]["reboot_requested_head"] == "target-head"
  assert data["auto_update"]["error_code"] == f"error-{git_state.AUTO_UPDATE_HISTORY_LIMIT + 2}"
  assert len(data["auto_update_history"]) == git_state.AUTO_UPDATE_HISTORY_LIMIT
