"""Impact notice handshake. No sensor assumptions, UI imports or reboot syscalls."""
import json
import math


NOTICE_KEY = "ImpactDashcamNotice"
FEEDBACK_KEY = "ImpactDashcamFeedback"
REBOOT_KEY = "ImpactDashcamReboot"
COUNTDOWN_SECONDS = 10.0
UI_TIMEOUT = 0.5


def read_object(params, key):
  try:
    value = params.get(key)
    if isinstance(value, dict):
      return value
    value = json.loads(value or "{}")
    return value if isinstance(value, dict) else {}
  except (ValueError, TypeError):
    return {}


class ImpactDashcam:
  def __init__(self, params, memory):
    self.params = params
    self.memory = memory
    self.pending = False
    self.committed = params.get_bool(REBOOT_KEY)
    self.armed = True
    self.quiet_since = None
    self.token = ""
    self.created = 0.0
    self.shown_since = None
    self.last_visible = None
    self.memory.remove(NOTICE_KEY)
    self.memory.remove(FEEDBACK_KEY)

  def cancel(self):
    self.pending = False
    self.memory.remove(NOTICE_KEY)
    self.memory.remove(FEEDBACK_KEY)

  def update(self, *, now, allowed, trigger, quiet, details=None):
    if self.committed:
      return
    if not allowed:
      if self.pending:
        self.cancel()
      self.quiet_since = None
      return

    if not self.pending:
      if quiet:
        if self.quiet_since is None:
          self.quiet_since = now
        if now - self.quiet_since >= 1.0:
          self.armed = True
      elif quiet is not None:
        self.quiet_since = None
      if not (trigger and self.armed):
        return
      self.armed = False
      self.pending = True
      self.created = now
      self.token = str(int(now * 1e9))
      self.shown_since = self.last_visible = None
      self.memory.remove(FEEDBACK_KEY)
      self.memory.put(NOTICE_KEY, {"token": self.token, "created": now, "details": details or {}})

    feedback = read_object(self.memory, FEEDBACK_KEY)
    if feedback.get("token") == self.token:
      # Cancellation wins, including a touch in the frame at the deadline.
      if feedback.get("cancel") is True:
        self.cancel()
        return
      visible = feedback.get("visible")
      if type(visible) in (int, float) and math.isfinite(visible) and self.created <= visible <= now:
        if now - visible <= UI_TIMEOUT:
          if self.last_visible is not None and visible - self.last_visible > UI_TIMEOUT:
            self.cancel()
            return
          if self.shown_since is None:
            self.shown_since = visible
          self.last_visible = visible
          if visible - self.shown_since >= COUNTDOWN_SECONDS:
            # Synchronous durable OFF must finish before asking manager to stop
            # its workers and reboot through the existing bounded sound helper.
            self.params.put_bool("OpenpilotEnabledToggle", False)
            if self.params.get("OpenpilotEnabledToggle") is not False:
              raise OSError("Failed to save OpenpilotEnabledToggle=0")
            self.params.put_bool(REBOOT_KEY, True)
            if self.params.get(REBOOT_KEY) is not True:
              raise OSError("Failed to save impact reboot interlock")
            self.committed = True
            self.pending = False
            self.params.put_bool("DoReboot", True)
            return

    # Never reboot after an unseen notice or a frozen/hidden UI. Re-arm only
    # after a fresh quiet interval, not from stale or unavailable input.
    if ((self.last_visible is not None and now - self.last_visible > UI_TIMEOUT)
        or (self.shown_since is None and now - self.created > 2.0)):
      self.cancel()
