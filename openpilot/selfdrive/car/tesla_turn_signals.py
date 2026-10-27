"""Tesla lamp execution adapted from xpilot's bounded DAS_bodyControls sessions."""
from dataclasses import dataclass

from opendbc.car.can_definitions import CanData
from opendbc.car.tesla.values import CAR, TeslaFlags, TeslaSafetyFlags

BODY = 0x3E9
UI = 0x311
FRONT = 0x3F5
TEMPLATE_NS = 1_500_000_000
CONTEXT_NS = 200_000_000
FEEDBACK_NS = 500_000_000
FEEDBACK_DEGRADED_NS = 1_500_000_000
FRONT_FAULT_GRACE_NS = 500_000_000
ECHO_NS = 1_200_000_000
SESSION_NS = 60_000_000_000
CANCEL_NS = 5_000_000_000
OFF_STABLE_NS = 500_000_000


def body_checksum(data: bytes | bytearray) -> int:
  if len(data) != 8:
    raise ValueError("Tesla body controls require eight bytes")
  return (0xE9 + 3 + sum(data[:7])) & 0xFF


def body_frame(template: bytes, direction: int) -> bytes:
  if direction not in (1, 2, 3) or len(template) != 8 or template[1] & 3 or body_checksum(template) != template[7]:
    raise ValueError("Invalid Tesla turn request or OEM idle template")
  data = bytearray(template)
  data[1] = (data[1] & 0xFC) | direction
  data[2] = (data[2] & 0xE1) | ((4 if direction == 3 else 8) << 1)
  data[6] = (data[6] & 15) | ((((data[6] >> 4) + 1) & 15) << 4)
  data[7] = body_checksum(data)
  return bytes(data)


@dataclass
class Session:
  request_id: int
  direction: int
  started: int
  sent: int = 0
  awaiting: bytes | None = None
  submitted: int = 0
  cancelling: bool = False
  cancel_started: int = 0
  cancel_attempts: int = 0
  cancel_echo: bool = False
  active_echoes: int = 0
  cancel_echoes: int = 0
  rejected: int = 0
  manual: bool = False
  on_seen: bool = False
  off_since: int | None = None
  last_on: int = 0
  last_front_on: int = 0
  lost_since: int | None = None


class TeslaTurnSignals:
  def __init__(self, configured: bool, *, disabled_reason: str = "disabled"):
    self.configured = configured
    self.session: Session | None = None
    self.template: bytes | None = None
    self.template_time = 0
    self.template_consumed_time = 0
    self.body_rx = self.body_idle_rx = self.body_invalid_rx = self.body_non_idle_rx = 0
    self.ui = (0, 0)
    self.front = (3, 3)
    self.front_fault_since: list[int | None] = [None, None]
    self.ui_time = self.front_time = 0
    self.context_time = 0
    self.blocked_id = 0
    self.disabled_status = disabled_reason
    self.status = "idle" if configured else disabled_reason
    self.last_stalk_counts: tuple[int, int] | None = None
    self.failed = False
    self.driver_idle_time = 0
    self.driver_press_time = 0

  @classmethod
  def from_car_params(cls, CP, *, enabled: bool):
    reason = None
    if CP.brand != "tesla":
      reason = "disabled:not_tesla"
    elif not enabled:
      reason = "disabled:setting_off"
    elif CP.carFingerprint not in (CAR.TESLA_MODEL_3, CAR.TESLA_MODEL_Y):
      reason = "disabled:unsupported_platform"
    elif CP.passive:
      reason = "disabled:passive"
    elif not CP.flags & TeslaFlags.HAS_VEHICLE_BUS:
      reason = "blocked:vehicle_bus_unavailable"
    elif not CP.flags & TeslaFlags.AUTO_TURN_SIGNALS:
      reason = "blocked:restart_required"
    elif not any(str(s.safetyModel) == "tesla" and s.safetyParam & TeslaSafetyFlags.AUTO_TURN_SIGNALS for s in CP.safetyConfigs):
      reason = "blocked:safety_configuration"
    return cls(reason is None, disabled_reason=reason or "disabled")

  @property
  def owned_direction(self) -> int:
    return self.session.direction if self.session is not None and not self.session.manual else 0

  def ready_request_id(self, now: int) -> int:
    s = self.session
    if (s is None or self.failed or s.cancelling or s.manual or not s.active_echoes or not s.on_seen or
        not self.context_time or not 0 <= now - self.context_time <= CONTEXT_NS or
        not 0 <= now - s.started < SESSION_NS or not self._feedback_available(now) or self._front_fault_expired(now) or
        not 0 <= now - s.last_on < 2_500_000_000 or
        not 0 <= now - s.last_front_on < 2_500_000_000 + FEEDBACK_DEGRADED_NS):
      return 0
    return s.request_id

  def diagnostics(self, now: int) -> dict:
    s = self.session
    template_ready = self.template is not None and 0 <= now - self.template_time <= TEMPLATE_NS
    return {
      "status": self.status,
      "request_id": s.request_id if s else 0,
      "direction": s.direction if s else 0,
      "active_submissions": s.sent - s.cancel_attempts if s else 0,
      "cancel_submissions": s.cancel_attempts if s else 0,
      "active_tx_echoes": s.active_echoes if s else 0,
      "cancel_tx_echoes": s.cancel_echoes if s else 0,
      "panda_rejections": s.rejected if s else 0,
      "awaiting_echo": s is not None and s.awaiting is not None,
      "manual": s.manual if s else False,
      "on_seen": s.on_seen if s else False,
      "session_ms": (now - s.started) // 1_000_000 if s else None,
      "body_age_ms": (now - self.template_time) // 1_000_000 if self.template_time else None,
      "template_ready": template_ready,
      "template_counter": self.template[6] >> 4 if self.template is not None else None,
      "pending_counter": s.awaiting[6] >> 4 if s and s.awaiting is not None else None,
      "submission_age_ms": (now - s.submitted) // 1_000_000 if s and s.sent else None,
      "send_state": ("idle" if s is None else "fault" if self.failed else "driver_takeover" if s.manual else
                     "cancelling" if s.cancelling else "waiting:tx_echo" if s.awaiting is not None else
                     "ready" if template_ready else "waiting:body_template"),
      "ready_request_id": self.ready_request_id(now),
      "body_rx": self.body_rx,
      "body_idle_rx": self.body_idle_rx,
      "body_invalid_rx": self.body_invalid_rx,
      "body_non_idle_rx": self.body_non_idle_rx,
      "ui_age_ms": (now - self.ui_time) // 1_000_000 if self.ui_time else None,
      "front_age_ms": (now - self.front_time) // 1_000_000 if self.front_time else None,
      "physical_on_age_ms": (now - s.last_front_on) // 1_000_000 if s and s.last_front_on else None,
      "feedback": "fresh" if self._feedback_fresh(now) else "degraded" if self._feedback_available(now) else "stale",
      "context_age_ms": (now - self.context_time) // 1_000_000 if self.context_time else None,
      "ui": self.ui,
      "front": self.front,
      "front_fault_ms": [(now - t) // 1_000_000 if t is not None else None for t in self.front_fault_since],
    }

  def _front_fault_expired(self, now: int) -> bool:
    return any(t is not None and now - t >= FRONT_FAULT_GRACE_NS for t in self.front_fault_since)

  def _feedback_fresh(self, now: int) -> bool:
    return all(t > 0 and 0 <= now - t <= FEEDBACK_NS for t in (self.ui_time, self.front_time))

  def _feedback_available(self, now: int) -> bool:
    # Startup feedback may lag TX, but must still confirm actual lamps before its deadline.
    times = (self.ui_time, self.front_time)
    return self._feedback_fresh(now) or (
      self.session is not None and (self.session.on_seen or self.session.active_echoes > 0) and
      all(t > 0 and 0 <= now - t <= FEEDBACK_DEGRADED_NS for t in times) and
      any(0 <= now - t <= FEEDBACK_NS for t in times))

  def _off(self, now: int) -> bool:
    return self._feedback_fresh(now) and self.ui == (0, 0) and self.front == (0, 0)

  def _cancel(self, now: int, reason: str) -> None:
    s = self.session
    if s is None or s.cancelling:
      return
    self.blocked_id = s.request_id
    if not s.sent:
      self.session = None
      self.status = "cancelled_before_send"
      return
    s.cancelling, s.cancel_started = True, now
    self.status = f"cancelling:{reason}"

  def _fail(self, reason: str) -> None:
    self.failed = True
    self.status = f"fault:{reason}"
    if self.session is not None:
      self.blocked_id = self.session.request_id

  def _handoff(self, now: int) -> None:
    s = self.session
    if s is not None:
      # CANCEL may extinguish a human's signal. Defer Panda bookkeeping
      # cleanup until fresh physical idle and stable, actual lamp OFF.
      s.manual = True
      if self.failed:
        return
      self._cancel(now, "driver")
      if self.session is not None:
        s.cancel_started = 0
        s.off_since = None
        self.status = "driver_takeover:waiting_off"

  def observe_driver(self, state, now: int) -> None:
    counts = (int(state.leftBlinkerStalkCount), int(state.rightBlinkerStalkCount))
    if state.turnSignalDriverValid and self.last_stalk_counts is not None and counts != self.last_stalk_counts:
      self.driver_press_time = now
    self.last_stalk_counts = counts
    driver = state.turnSignalDriverValid and state.turnSignalDriverDirection != 0
    pressed = self.driver_press_time and 0 <= now - self.driver_press_time <= CONTEXT_NS
    self.driver_idle_time = now if state.turnSignalDriverValid and not (driver or pressed) else 0
    if driver or pressed:
      self._handoff(now)

  def observe(self, packets, now: int) -> None:
    if not self.configured:
      return
    for timestamp, frames in packets:
      if not 0 < timestamp <= now:
        continue
      for address, payload, source in frames:
        data = bytes(payload)
        if source == 1 and address == BODY:
          self.body_rx += 1
          if len(data) != 8 or body_checksum(data) != data[7]:
            self.body_invalid_rx += 1
          elif data[1] & 3:
            self.body_non_idle_rx += 1
          else:
            self.body_idle_rx += 1
            if timestamp >= self.template_time and timestamp > self.template_consumed_time:
              self.template, self.template_time = data, timestamp
        elif source == 0 and address == UI and len(data) == 7 and timestamp >= self.ui_time:
          # UI_warning is Motorola left at bit 25 and Intel right at bit 26.
          if data[0] == ((0x11 + 3 + sum(data[1:])) & 0xFF):
            self.ui = ((data[3] >> 0) & 3, (data[3] >> 2) & 3)
            self.ui_time = timestamp
            if all(v in (1, 2) for v in self.ui):
              self._handoff(now)
            if 3 in self.ui:
              self._cancel(now, "invalid_lamp_feedback")
        elif source == 1 and address == FRONT and len(data) == 8 and timestamp >= self.front_time:
          if self._front_fault_expired(timestamp):
            self._cancel(now, "invalid_lamp_feedback")
          self.front = ((data[6] >> 2) & 3, (data[6] >> 4) & 3)
          self.front_time = timestamp
          # Recorded lamps alternate 2/1. Debounce 2, but never use it as ON or OFF confirmation.
          for side, value in enumerate(self.front):
            if value != 2:
              self.front_fault_since[side] = None
            elif self.front_fault_since[side] is None:
              self.front_fault_since[side] = timestamp
          if 3 in self.front:
            self._cancel(now, "invalid_lamp_feedback")
        s = self.session
        if s is not None and address == BODY and data == s.awaiting and source in (0x81, 0xC1):
          s.awaiting = None
          if source == 0xC1:
            s.rejected += 1
            if s.cancelling:
              self._fail("panda_rejected_cancel")
            else:
              self._cancel(now, "panda_rejected")
          elif data[1] & 3 == 3:
            s.cancel_echoes += 1
            s.cancel_echo = s.cancelling
          else:
            s.active_echoes += 1
        if s is not None:
          if ((0 < self.ui_time <= now and now - self.ui_time <= FEEDBACK_NS and self.ui[2 - s.direction] in (1, 2)) or
              (0 < self.front_time <= now and now - self.front_time <= FEEDBACK_NS and self.front[2 - s.direction] == 1)):
            self._handoff(now)
        if s is not None and self._feedback_fresh(now):
          if self.ui[s.direction - 1] in (1, 2) and self.front[s.direction - 1] == 1:
            s.on_seen = True
            s.last_on = max(s.last_on, min(self.ui_time, self.front_time))
        if s is not None and self.front[s.direction - 1] == 1 and 0 <= now - self.front_time <= FEEDBACK_NS:
          s.last_front_on = max(s.last_front_on, self.front_time)
        if s is not None and s.on_seen and not self._feedback_fresh(now):
          if self.ui[s.direction - 1] == 2 and 0 <= now - self.ui_time <= FEEDBACK_NS:
            s.last_on = max(s.last_on, self.ui_time)
          if self.front[s.direction - 1] == 1 and 0 <= now - self.front_time <= FEEDBACK_NS:
            s.last_on = max(s.last_on, self.front_time)
    self.advance(now)

  def update(self, state, control, meta, fresh: bool, now: int) -> list[CanData]:
    self.context_time = now if fresh else 0
    self.observe_driver(state, now)
    pressed = self.driver_press_time and 0 <= now - self.driver_press_time <= CONTEXT_NS
    driver = state.turnSignalDriverValid and state.turnSignalDriverDirection != 0
    hazards = all(v in (1, 2) for v in self.ui)
    s = self.session
    if hazards:
      self._handoff(now)

    direction = meta.autoTurnSignalDirection.raw if fresh else 0
    request_id = int(meta.autoTurnSignalRequestId) if fresh else 0
    checks = ((self.configured, self.disabled_status), (fresh, "context_stale"),
              (control.latActive, "lateral_inactive"), (state.canValid, "can_invalid"),
              (not state.vehicleSensorsInvalid, "vehicle_sensors_invalid"),
              (str(state.gearShifter) == "drive", "not_in_drive"),
              (state.turnSignalDriverValid, "driver_input_unavailable"),
              (not driver and not pressed, "driver"), (not hazards, "hazards"))
    blocked = next((reason for valid, reason in checks if not valid), None)
    if blocked:
      self._cancel(now, blocked)
      if self.session is None and not self.failed:
        self.status = f"blocked:{blocked}" if self.configured else self.disabled_status
    elif direction not in (1, 2) or not request_id:
      self._cancel(now, "intent_inactive")
      if self.session is None and self.status.startswith(("blocked:", "waiting:")) and not self.failed:
        self.status = "idle"
    elif s is not None:
      if request_id != s.request_id or direction != s.direction:
        self._cancel(now, "request_changed")
    elif not self.failed and request_id != self.blocked_id:
      if not 0 < self.ui_time <= now or now - self.ui_time > FEEDBACK_NS:
        self.status = "waiting:ui_feedback"
      elif not 0 < self.front_time <= now or now - self.front_time > FEEDBACK_NS:
        self.status = "waiting:front_feedback"
      elif 3 in self.ui or any(v in (2, 3) for v in self.front):
        self.status = "waiting:invalid_lamp_feedback"
      elif self.ui != (0, 0) or self.front != (0, 0):
        self.status = "waiting:lamps_off"
      elif self.template is None or not 0 <= now - self.template_time <= TEMPLATE_NS:
        self.status = "waiting:body_template"
      else:
        self.session = Session(request_id, direction, now)
        self.status = "active"
    self.advance(now)
    return self.take_sends(now)

  def advance(self, now: int) -> None:
    s = self.session
    if s is None:
      return
    if self.failed:
      # Retain provenance after an unconfirmed cancellation, until actually off.
      if self._off(now):
        if s.off_since is None:
          s.off_since = now
        elif now - s.off_since >= OFF_STABLE_NS:
          self.session = None
      else:
        s.off_since = None
      return
    if s.manual:
      idle = self.driver_idle_time and 0 <= now - self.driver_idle_time <= CONTEXT_NS
      if not idle or not self._off(now):
        s.off_since = None
        return
      if s.off_since is None:
        s.off_since = now
      if now - s.off_since < OFF_STABLE_NS:
        return
      if not s.cancel_started:
        s.cancel_started = now
        self.status = "cancelling:driver_cleanup"
      if s.awaiting is not None and now - s.submitted >= ECHO_NS:
        s.awaiting = None
    if not s.cancelling:
      if not self.context_time or not 0 <= now - self.context_time <= CONTEXT_NS:
        self._cancel(now, "context_stale")
      elif now - s.started >= SESSION_NS:
        self._cancel(now, "session_timeout")
      elif not self._feedback_available(now):
        self._cancel(now, "feedback_stale")
      elif self._front_fault_expired(now):
        self._cancel(now, "invalid_lamp_feedback")
      elif not s.on_seen and now - s.started >= 2_500_000_000:
        self._cancel(now, "lamp_not_confirmed")
      elif s.on_seen:
        if self._off(now):
          if s.lost_since is None:
            s.lost_since = now
          elif now - s.lost_since >= OFF_STABLE_NS:
            self._cancel(now, "lamps_lost")
        else:
          s.lost_since = None
        if now - s.last_on >= 2_500_000_000:
          self._cancel(now, "lamp_feedback_lost")
        # UI activity alone must never hide a physical channel stuck OFF.
        if now - s.last_front_on >= 2_500_000_000 + FEEDBACK_DEGRADED_NS:
          self._cancel(now, "physical_lamp_feedback_lost")
    s = self.session
    if s is None:
      return
    if s.awaiting is not None and now - s.submitted >= ECHO_NS:
      s.awaiting = None
      self._cancel(now, "tx_echo_timeout")
    if s.cancelling:
      if s.cancel_echo and self._off(now):
        if s.off_since is None:
          s.off_since = now
        elif now - s.off_since >= OFF_STABLE_NS:
          self.status = "cancelled"
          self.session = None
          return
      else:
        s.off_since = None
      if now - s.cancel_started >= CANCEL_NS:
        self._fail("cancel_not_confirmed")

  def take_sends(self, now: int, *, cancel_only: bool = False) -> list[CanData]:
    s = self.session
    if (s is None or self.failed or s.awaiting is not None or s.cancel_echo or
        (s.manual and (not s.cancel_started or not self.driver_idle_time or
                       not 0 <= now - self.driver_idle_time <= CONTEXT_NS or not self._off(now))) or
        (cancel_only and not s.cancelling) or self.template is None or
        not 0 <= now - self.template_time <= TEMPLATE_NS):
      return []
    if s.cancelling and s.cancel_attempts >= 3:
      return []
    data = body_frame(self.template, 3 if s.cancelling else s.direction)
    self.template_consumed_time = self.template_time
    self.template = None
    s.awaiting, s.submitted = data, now
    s.sent += 1
    s.cancel_attempts += int(s.cancelling)
    return [CanData(BODY, data, 1)]
