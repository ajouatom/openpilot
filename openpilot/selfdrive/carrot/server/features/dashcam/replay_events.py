from __future__ import annotations

from dataclasses import dataclass
from typing import Any


REPLAY_EVENT_INDEX_VERSION = 2

_NAV_APPROACH_THRESHOLDS_M = (300, 200, 100, 50, 20)

_CATEGORY_COOLDOWN_NS = {
  "control": 500_000_000,
  "driver": 800_000_000,
  "vehicle": 350_000_000,
  "carrot": 1_000_000_000,
  "warning": 8_000_000_000,
  "turn": 1_500_000_000,
  "nav": 7_000_000_000,
}


def _get(value: Any, name: str, default: Any = None) -> Any:
  try:
    return getattr(value, name)
  except Exception:
    return default


def _bool(value: Any, name: str, default: bool = False) -> bool:
  try:
    return bool(_get(value, name, default))
  except Exception:
    return default


def _int(value: Any, name: str, default: int = 0) -> int:
  try:
    return int(_get(value, name, default))
  except Exception:
    return default


def _text_value(value: Any) -> str:
  try:
    text = str(value or "").strip()
  except Exception:
    return ""
  return text.rsplit(".", 1)[-1]


def _text(value: Any, name: str, default: str = "") -> str:
  return _text_value(_get(value, name, default))


def _float(value: Any, name: str, default: float = 0.0) -> float:
  try:
    return float(_get(value, name, default))
  except Exception:
    return default


def _present(value: Any) -> bool:
  if value is None:
    return False
  meta = _get(value, "meta")
  if meta is not None:
    return _bool(meta, "present")
  return _bool(value, "present")


def _int_list(value: Any, name: str, limit: int = 16) -> tuple[int, ...]:
  try:
    return tuple(int(item) for item in tuple(_get(value, name, ()))[:limit])
  except Exception:
    return ()


@dataclass(slots=True)
class _IndexedEvent:
  log_mono_ns: int
  category: str
  event_type: str
  params: dict[str, Any]
  source_title: str = ""
  source_detail: str = ""


@dataclass(slots=True)
class _Session:
  start_ns: int
  category: str
  start_type: str
  end_type: str
  params: dict[str, Any]
  minimum_ns: int


class ReplayEventIndexer:
  """Extracts verified, low-volume semantic markers during the existing rlog scan."""

  def __init__(self) -> None:
    self._events: list[_IndexedEvent] = []
    self._previous: dict[str, Any] = {}
    self._last_event_ns: dict[str, int] = {}
    self._sessions: dict[str, _Session] = {}
    self._last_lane_direction = ""
    self._has_carrot_navi = False
    self._finalized = False

  def _push(
    self,
    log_mono_ns: int,
    category: str,
    event_type: str,
    *,
    params: dict[str, Any] | None = None,
    source_title: str = "",
    source_detail: str = "",
    dedupe_key: str = "",
    cooldown_ns: int | None = None,
  ) -> None:
    if log_mono_ns <= 0:
      return
    key = f"{category}:{event_type}:{dedupe_key}"
    minimum_gap_ns = _CATEGORY_COOLDOWN_NS.get(category, 500_000_000) if cooldown_ns is None else cooldown_ns
    previous_ns = self._last_event_ns.get(key, 0)
    if previous_ns > 0 and log_mono_ns - previous_ns < minimum_gap_ns:
      return
    self._last_event_ns[key] = log_mono_ns
    self._events.append(_IndexedEvent(
      log_mono_ns=log_mono_ns,
      category=category,
      event_type=event_type,
      params=dict(params or {}),
      source_title=source_title.strip(),
      source_detail=source_detail.strip(),
    ))

  def _transition(
    self,
    key: str,
    value: Any,
    log_mono_ns: int,
    category: str,
    on_type: str,
    off_type: str = "",
    *,
    params: dict[str, Any] | None = None,
  ) -> None:
    previous = self._previous.get(key)
    self._previous[key] = value
    if previous is None or previous == value:
      return
    event_type = on_type if bool(value) else off_type
    if event_type:
      self._push(log_mono_ns, category, event_type, params=params)

  def _changed(
    self,
    key: str,
    value: Any,
    log_mono_ns: int,
    category: str,
    event_type: str,
    *,
    ignore: tuple[Any, ...] = (),
  ) -> None:
    previous = self._previous.get(key)
    self._previous[key] = value
    if previous is None or previous == value or value in ignore:
      return
    self._push(
      log_mono_ns,
      category,
      event_type,
      params={"from": previous, "value": value},
      dedupe_key=str(value),
    )

  def _session(
    self,
    key: str,
    active: bool,
    log_mono_ns: int,
    category: str,
    start_type: str,
    end_type: str,
    *,
    params: dict[str, Any] | None = None,
    minimum_ms: int = 120,
  ) -> None:
    previous = self._previous.get(key)
    self._previous[key] = active
    if previous is None:
      return
    if active and not previous:
      self._sessions[key] = _Session(
        start_ns=log_mono_ns,
        category=category,
        start_type=start_type,
        end_type=end_type,
        params=dict(params or {}),
        minimum_ns=max(0, minimum_ms) * 1_000_000,
      )
      return
    if not active and previous:
      session = self._sessions.pop(key, None)
      if session is None or log_mono_ns - session.start_ns < session.minimum_ns:
        return
      duration_ms = max(0, round((log_mono_ns - session.start_ns) / 1_000_000))
      event_params = {**session.params, "durationMs": duration_ms}
      self._push(session.start_ns, session.category, session.start_type, params=event_params, cooldown_ns=0)
      self._push(log_mono_ns, session.category, session.end_type, params=event_params, cooldown_ns=0)

  def ingest(self, event: Any, service: str) -> None:
    if self._finalized or not bool(_get(event, "valid", True)):
      return
    log_mono_ns = _int(event, "logMonoTime")
    if log_mono_ns <= 0:
      return
    value = _get(event, service)
    handler = getattr(self, f"_ingest_{service}", None)
    if callable(handler) and value is not None:
      try:
        handler(value, log_mono_ns)
      except Exception:
        # Event indexing must never prevent replay generation for older schemas.
        return

  def _ingest_selfdriveState(self, value: Any, log_mono_ns: int) -> None:
    self._transition(
      "selfdrive.enabled", _bool(value, "enabled"), log_mono_ns, "control", "control_engaged", "control_disengaged",
    )
    state_name = _text(value, "state").lower()
    overriding = state_name == "overriding"
    self._transition(
      "selfdrive.overriding", overriding, log_mono_ns, "control", "control_override_start", "control_override_end",
    )
    self._transition(
      "selfdrive.experimental", _bool(value, "experimentalMode"), log_mono_ns,
      "control", "experimental_mode_on", "experimental_mode_off",
    )
    self._transition(
      "selfdrive.engageable", _bool(value, "engageable"), log_mono_ns,
      "control", "control_available", "control_unavailable",
    )
    self._changed(
      "selfdrive.personality", _text(value, "personality"), log_mono_ns,
      "control", "driving_personality_changed", ignore=("",),
    )

    alert_type = _text(value, "alertType")
    alert_title = _text(value, "alertText1")
    alert_detail = _text(value, "alertText2")
    alert_status = _text(value, "alertStatus").lower()
    alert_key = f"{alert_type}|{alert_title}|{alert_detail}" if alert_type or alert_title else ""
    previous_alert = self._previous.get("selfdrive.alert", "")
    self._previous["selfdrive.alert"] = alert_key
    if alert_key and alert_key != previous_alert and alert_status not in ("", "normal", "0"):
      self._push(
        log_mono_ns,
        "warning",
        "system_alert",
        params={"alertStatus": alert_status, "alertType": alert_type},
        source_title=alert_title,
        source_detail=alert_detail,
        dedupe_key=alert_type or alert_title,
      )

  def _ingest_carControl(self, value: Any, log_mono_ns: int) -> None:
    self._transition(
      "carControl.latActive", _bool(value, "latActive"), log_mono_ns,
      "control", "lateral_control_on", "lateral_control_off",
    )
    self._transition(
      "carControl.longActive", _bool(value, "longActive"), log_mono_ns,
      "control", "longitudinal_control_on", "longitudinal_control_off",
    )
    cruise = _get(value, "cruiseControl")
    if cruise is None:
      return
    self._transition(
      "carControl.override", _bool(cruise, "override"), log_mono_ns,
      "control", "control_override_start", "control_override_end",
    )
    self._transition(
      "carControl.cancel", _bool(cruise, "cancel"), log_mono_ns, "control", "cruise_cancel_requested",
    )
    self._transition(
      "carControl.resume", _bool(cruise, "resume"), log_mono_ns, "control", "cruise_resume_requested",
    )

  def _ingest_carState(self, value: Any, log_mono_ns: int) -> None:
    self._session(
      "carState.gasPressed", _bool(value, "gasPressed"), log_mono_ns,
      "vehicle", "accelerator_pressed", "accelerator_released", minimum_ms=100,
    )
    self._session(
      "carState.brakePressed", _bool(value, "brakePressed"), log_mono_ns,
      "vehicle", "brake_pressed", "brake_released", minimum_ms=100,
    )
    self._session(
      "carState.steeringPressed", _bool(value, "steeringPressed"), log_mono_ns,
      "driver", "steering_override_start", "steering_override_end", minimum_ms=180,
    )
    self._transition(
      "carState.leftBlinker", _bool(value, "leftBlinker"), log_mono_ns, "vehicle", "left_blinker_on",
    )
    self._transition(
      "carState.rightBlinker", _bool(value, "rightBlinker"), log_mono_ns, "vehicle", "right_blinker_on",
    )
    cruise_state = _get(value, "cruiseState")
    if cruise_state is not None:
      self._transition(
        "carState.cruiseEnabled", _bool(cruise_state, "enabled"), log_mono_ns,
        "control", "cruise_enabled", "cruise_disabled",
      )
    self._changed(
      "carState.gear", _text(value, "gearShifter"), log_mono_ns,
      "vehicle", "gear_changed", ignore=("", "unknown"),
    )
    self._transition(
      "carState.standstill", _bool(value, "standstill"), log_mono_ns,
      "vehicle", "vehicle_stopped", "vehicle_moved",
    )
    self._transition(
      "carState.brakeHold", _bool(value, "brakeHoldActive"), log_mono_ns,
      "vehicle", "brake_hold_on", "brake_hold_off",
    )
    self._transition(
      "carState.parkingBrake", _bool(value, "parkingBrake"), log_mono_ns,
      "vehicle", "parking_brake_on", "parking_brake_off",
    )
    self._session(
      "carState.clutchPressed", _bool(value, "clutchPressed"), log_mono_ns,
      "vehicle", "clutch_pressed", "clutch_released", minimum_ms=100,
    )
    self._changed(
      "carState.softHold", _int(value, "softHoldActive"), log_mono_ns,
      "vehicle", "soft_hold_changed",
    )
    self._changed(
      "carState.cruiseGap", _int(value, "pcmCruiseGap"), log_mono_ns,
      "vehicle", "cruise_gap_changed", ignore=(0,),
    )
    self._changed(
      "carState.carrotCruise", _int(value, "carrotCruise"), log_mono_ns,
      "carrot", "carrot_cruise_changed",
    )
    self._transition(
      "carState.buttonEnable", _bool(value, "buttonEnable"), log_mono_ns,
      "vehicle", "enable_button_pressed",
    )

    left_blinker = _bool(value, "leftBlinker")
    right_blinker = _bool(value, "rightBlinker")
    self._warning_edge(value, "stockAeb", "stock_aeb", log_mono_ns)
    self._warning_edge(value, "stockFcw", "stock_fcw", log_mono_ns)
    self._warning_edge(value, "canTimeout", "can_timeout", log_mono_ns)
    self._warning_edge(value, "steerFaultTemporary", "steering_fault_temporary", log_mono_ns)
    self._warning_edge(value, "steerFaultPermanent", "steering_fault_permanent", log_mono_ns)
    self._warning_edge(value, "accFaulted", "acc_fault", log_mono_ns)
    self._warning_edge(value, "vehicleSensorsInvalid", "vehicle_sensors_invalid", log_mono_ns)
    self._warning_edge(value, "lowSpeedAlert", "low_speed_steering_alert", log_mono_ns)
    self._warning_edge(value, "carFaultedNonCritical", "vehicle_fault", log_mono_ns)
    self._warning_edge(value, "espDisabled", "stability_control_disabled", log_mono_ns)
    self._warning_edge(value, "doorOpen", "door_open", log_mono_ns)
    self._warning_edge(value, "seatbeltUnlatched", "seatbelt_unlatched", log_mono_ns)
    self._transition(
      "carState.leftBlindspotHazard", _bool(value, "leftBlindspot") and left_blinker, log_mono_ns,
      "warning", "left_blindspot_warning",
    )
    self._transition(
      "carState.rightBlindspotHazard", _bool(value, "rightBlindspot") and right_blinker, log_mono_ns,
      "warning", "right_blindspot_warning",
    )
    can_valid = _bool(value, "canValid", True)
    previous_can_valid = self._previous.get("carState.canValid")
    self._previous["carState.canValid"] = can_valid
    if previous_can_valid is True and not can_valid:
      self._push(log_mono_ns, "warning", "can_invalid")

    button_events = _get(value, "buttonEvents", ())
    try:
      for button in button_events:
        if not _bool(button, "pressed"):
          continue
        button_type = _text(button, "type") or "unknown"
        self._push(
          log_mono_ns,
          "vehicle",
          "vehicle_button_pressed",
          params={"button": button_type},
          dedupe_key=button_type,
          cooldown_ns=180_000_000,
        )
    except Exception:
      pass

  def _warning_edge(
    self,
    value: Any,
    field: str,
    event_type: str,
    log_mono_ns: int,
    *,
    state_key: str = "",
  ) -> None:
    active = _bool(value, field)
    key = state_key or f"carState.{field}"
    previous = self._previous.get(key)
    self._previous[key] = active
    if active and previous is False:
      self._push(log_mono_ns, "warning", event_type)

  def _ingest_controlsState(self, value: Any, log_mono_ns: int) -> None:
    self._transition(
      "controls.forceDecel", _bool(value, "forceDecel"), log_mono_ns,
      "warning", "forced_deceleration",
    )
    control_state = _text(value, "longControlState").lower()
    previous = self._previous.get("controls.longControlState")
    self._previous["controls.longControlState"] = control_state
    if previous is None or previous == control_state:
      return
    if control_state == "stopping":
      self._push(log_mono_ns, "carrot", "longitudinal_stopping")
    elif control_state == "starting":
      self._push(log_mono_ns, "carrot", "longitudinal_starting")

  def _ingest_longitudinalPlan(self, value: Any, log_mono_ns: int) -> None:
    self._transition(
      "longPlan.hasLead", _bool(value, "hasLead"), log_mono_ns,
      "carrot", "lead_acquired", "lead_lost",
    )
    self._transition(
      "longPlan.shouldStop", _bool(value, "shouldStop"), log_mono_ns,
      "carrot", "planned_stop_start", "planned_stop_end",
    )
    self._warning_edge(value, "fcw", "forward_collision_warning", log_mono_ns, state_key="longPlan.fcw")
    self._changed(
      "longPlan.drivingMode", _int(value, "myDrivingMode"), log_mono_ns,
      "carrot", "driving_mode_changed",
    )

  def _ingest_lateralPlan(self, value: Any, log_mono_ns: int) -> None:
    lane_state = _text(value, "laneChangeState").lower()
    direction = _text(value, "laneChangeDirection").lower()
    previous_state = self._previous.get("lateralPlan.laneChangeState")
    self._previous["lateralPlan.laneChangeState"] = lane_state
    if direction not in ("", "none"):
      self._last_lane_direction = direction
    was_active = previous_state not in (None, "", "off", "0")
    active = lane_state not in ("", "off", "0")
    if not was_active and active:
      event_type = "lane_change_left" if direction == "left" else "lane_change_right" if direction == "right" else "lane_change"
      self._push(log_mono_ns, "turn", event_type, params={"direction": direction}, dedupe_key=direction)
    elif was_active and not active:
      self._push(
        log_mono_ns,
        "turn",
        "lane_change_completed",
        params={"direction": self._last_lane_direction},
        dedupe_key=self._last_lane_direction,
      )
      self._last_lane_direction = ""

  @staticmethod
  def _guidance_key(guidance: Any) -> str:
    if not _present(guidance):
      return ""
    turn_type = _int(guidance, "turnType")
    if _bool(guidance, "pointValid"):
      point = f"{_float(guidance, 'latitude'):.7f},{_float(guidance, 'longitude'):.7f}"
    else:
      point = _text(guidance, "mainText") or _text(guidance, "roadName")
    return f"{turn_type}|{point}"

  def _push_guidance(self, guidance: Any, slot: str, log_mono_ns: int) -> None:
    key = self._guidance_key(guidance)
    state_key = f"carrotNavi.guidance.{slot}"
    previous_key = str(self._previous.get(state_key) or "")
    previous_next = str(self._previous.get("carrotNavi.guidance.next") or "")
    self._previous[state_key] = key

    if not key:
      if previous_key:
        self._push(
          log_mono_ns,
          "nav",
          f"navigation_maneuver_{slot}_cleared",
          dedupe_key=slot,
          cooldown_ns=0,
        )
      if slot == "current":
        self._previous.pop("carrotNavi.guidance.currentDistance", None)
      return

    distance_m = max(0, _int(guidance, "distanceM"))
    turn_type = _int(guidance, "turnType")
    title = _text(guidance, "mainText") or _text(guidance, "roadName")
    road_name = _text(guidance, "roadName")
    params = {
      "turnType": turn_type,
      "distanceM": distance_m,
      "roadName": road_name,
    }
    if slot == "current" and key == previous_next and key != previous_key:
      params["promotedFromNext"] = True
    if key != previous_key:
      self._push(
        log_mono_ns,
        "nav",
        f"navigation_maneuver_{slot}",
        params=params,
        source_title=title,
        source_detail=road_name if road_name != title else "",
        dedupe_key=key,
        cooldown_ns=0,
      )

    if slot != "current" or distance_m <= 0:
      return
    distance_key = "carrotNavi.guidance.currentDistance"
    previous_distance = self._previous.get(distance_key)
    self._previous[distance_key] = (key, distance_m)
    thresholds: list[int] = []
    if not isinstance(previous_distance, tuple) or previous_distance[0] != key:
      eligible = [threshold for threshold in _NAV_APPROACH_THRESHOLDS_M if distance_m <= threshold]
      if eligible:
        thresholds.append(min(eligible))
    else:
      before_m = int(previous_distance[1])
      if distance_m < before_m:
        thresholds.extend(
          threshold
          for threshold in _NAV_APPROACH_THRESHOLDS_M
          if before_m > threshold >= distance_m
        )
    for threshold_m in thresholds:
      self._push(
        log_mono_ns,
        "nav",
        "navigation_approach",
        params={**params, "thresholdM": threshold_m},
        source_title=title,
        source_detail=road_name if road_name != title else "",
        dedupe_key=f"{key}:{threshold_m}",
        cooldown_ns=0,
      )

  def _ingest_lane_guidance(self, lane: Any, log_mono_ns: int) -> None:
    visible = _present(lane) and _bool(lane, "visible")
    available = _int_list(lane, "available")
    lane_key = (
      _int(lane, "count"),
      _int(lane, "currentLane"),
      _int(lane, "turnCode"),
      available,
    )
    previous_visible = self._previous.get("carrotNavi.lane.visible")
    previous_key = self._previous.get("carrotNavi.lane.key")
    self._previous["carrotNavi.lane.visible"] = visible
    self._previous["carrotNavi.lane.key"] = lane_key
    if visible and (previous_visible is not True or lane_key != previous_key):
      self._push(
        log_mono_ns,
        "nav",
        "lane_guidance_shown" if previous_visible is not True else "lane_guidance_changed",
        params={
          "laneCount": lane_key[0],
          "currentLane": lane_key[1],
          "turnCode": lane_key[2],
          "available": list(available),
          "distanceM": max(0, _int(lane, "distanceM")),
        },
        dedupe_key=str(lane_key),
        cooldown_ns=0,
      )
    elif not visible and previous_visible is True:
      self._push(log_mono_ns, "nav", "lane_guidance_hidden", cooldown_ns=0)

  def _ingest_speed_alert(self, speed: Any, log_mono_ns: int, secondary: bool = False) -> None:
    prefix = "secondarySdi" if secondary else "sdi"
    source = "secondary" if secondary else "primary"
    present = _bool(speed, f"{prefix}Present")
    alert_key = (
      _int(speed, f"{prefix}Type"),
      _int(speed, f"{prefix}SpeedLimitKph"),
      _int(speed, f"{prefix}SectionType"),
      _int(speed, f"{prefix}BlockType"),
    )
    state_key = f"carrotNavi.speedAlert.{source}"
    previous = self._previous.get(state_key)
    self._previous[state_key] = (present, alert_key)
    params = {
      "source": source,
      "sdiType": alert_key[0],
      "speedLimitKph": alert_key[1],
      "distanceM": max(0, _int(speed, f"{prefix}DistanceM")),
      "sectionType": alert_key[2],
      "blockType": alert_key[3],
    }
    if present and (not isinstance(previous, tuple) or previous[0] is not True):
      self._push(log_mono_ns, "nav", "speed_alert_shown", params=params, dedupe_key=f"{source}:{alert_key}", cooldown_ns=0)
    elif present and isinstance(previous, tuple) and previous[0] is True and previous[1] != alert_key:
      self._push(log_mono_ns, "nav", "speed_alert_changed", params=params, dedupe_key=f"{source}:{alert_key}", cooldown_ns=0)
    elif not present and isinstance(previous, tuple) and previous[0] is True:
      self._push(log_mono_ns, "nav", "speed_alert_cleared", params={"source": source}, cooldown_ns=0)

  def _ingest_navigation_speed(self, speed: Any, log_mono_ns: int) -> None:
    self._ingest_speed_alert(speed, log_mono_ns)
    self._ingest_speed_alert(speed, log_mono_ns, secondary=True)

    road_limit = _int(speed, "roadLimitKph") if _bool(speed, "roadLimitValid") else 0
    previous_limit = self._previous.get("carrotNavi.roadLimitKph")
    self._previous["carrotNavi.roadLimitKph"] = road_limit
    if previous_limit is not None and previous_limit > 0 and road_limit > 0 and road_limit != previous_limit:
      self._push(
        log_mono_ns,
        "nav",
        "road_speed_limit_changed",
        params={"from": previous_limit, "value": road_limit},
        dedupe_key=str(road_limit),
        cooldown_ns=0,
      )

    section_active = _bool(speed, "sectionPresent") and _bool(speed, "sectionActive")
    previous_active = self._previous.get("carrotNavi.section.active")
    self._previous["carrotNavi.section.active"] = section_active
    section_params = {
      "speedLimitKph": _int(speed, "sectionSpeedLimitKph"),
      "averageKph": round(_float(speed, "sectionAverageKph"), 1),
      "remainingDistanceM": round(max(0.0, _float(speed, "sectionRemainingDistanceM")), 1),
    }
    if section_active and previous_active is not True:
      self._push(log_mono_ns, "nav", "section_control_started", params=section_params, cooldown_ns=0)
    elif not section_active and previous_active is True:
      self._push(log_mono_ns, "nav", "section_control_ended", cooldown_ns=0)
    if section_active:
      self._transition(
        "carrotNavi.section.suspended", _bool(speed, "sectionSuspended"), log_mono_ns,
        "nav", "section_control_suspended", "section_control_resumed", params=section_params,
      )
      self._transition(
        "carrotNavi.section.offRoute", _bool(speed, "sectionOffRoute"), log_mono_ns,
        "nav", "section_control_off_route", "section_control_recovered", params=section_params,
      )

  def _ingest_traffic_signal(self, signal: Any, log_mono_ns: int) -> None:
    visible = _bool(signal, "visible")
    signal_key = tuple(
      (_bool(signal, f"{name}Valid"), _bool(signal, f"{name}On"))
      for name in ("red", "left", "green", "right", "uturn")
    )
    previous_visible = self._previous.get("carrotNavi.signal.visible")
    previous_key = self._previous.get("carrotNavi.signal.key")
    self._previous["carrotNavi.signal.visible"] = visible
    self._previous["carrotNavi.signal.key"] = signal_key
    params = {
      "distanceM": max(0, _int(signal, "distanceM")),
      "red": signal_key[0][1] if signal_key[0][0] else None,
      "left": signal_key[1][1] if signal_key[1][0] else None,
      "green": signal_key[2][1] if signal_key[2][0] else None,
      "right": signal_key[3][1] if signal_key[3][0] else None,
      "uturn": signal_key[4][1] if signal_key[4][0] else None,
    }
    if visible and (previous_visible is not True or signal_key != previous_key):
      self._push(
        log_mono_ns,
        "nav",
        "traffic_signal_shown" if previous_visible is not True else "traffic_signal_changed",
        params=params,
        dedupe_key=str(signal_key),
        cooldown_ns=0,
      )
    elif not visible and previous_visible is True:
      self._push(log_mono_ns, "nav", "traffic_signal_hidden", cooldown_ns=0)

  def _ingest_crossroad(self, crossroad: Any, log_mono_ns: int) -> None:
    visible = _bool(crossroad, "visible")
    image_code = _int(crossroad, "imageCode")
    previous_visible = self._previous.get("carrotNavi.crossroad.visible")
    previous_code = self._previous.get("carrotNavi.crossroad.imageCode")
    self._previous["carrotNavi.crossroad.visible"] = visible
    self._previous["carrotNavi.crossroad.imageCode"] = image_code
    if visible and (previous_visible is not True or image_code != previous_code):
      self._push(
        log_mono_ns,
        "nav",
        "crossroad_guidance_shown" if previous_visible is not True else "crossroad_guidance_changed",
        params={"imageCode": image_code, "distanceM": max(0, _int(crossroad, "distanceM"))},
        dedupe_key=str(image_code),
        cooldown_ns=0,
      )
    elif not visible and previous_visible is True:
      self._push(log_mono_ns, "nav", "crossroad_guidance_hidden", cooldown_ns=0)

  def _ingest_carrotNavi(self, value: Any, log_mono_ns: int) -> None:
    connected = _bool(value, "connected")
    if connected:
      self._has_carrot_navi = True
    previous_connected = self._previous.get("carrotNavi.connected")
    self._previous["carrotNavi.connected"] = connected
    if previous_connected is not None and connected != previous_connected:
      self._push(
        log_mono_ns,
        "nav",
        "navi_connected" if connected else "navi_disconnected",
        cooldown_ns=0,
      )

    # generation advances with snapshots; only sessionId identifies a route session.
    session_key = _text(value, "sessionId")
    previous_session = self._previous.get("carrotNavi.session")
    self._previous["carrotNavi.session"] = session_key
    if previous_session and session_key and session_key != previous_session:
      self._push(log_mono_ns, "nav", "navigation_session_changed", cooldown_ns=0)

    status = _get(value, "navigationStatus")
    active = connected and _bool(status, "guidanceActive") and _bool(status, "routePresent")
    previous_active = self._previous.get("carrotNavi.navigation.active")
    self._previous["carrotNavi.navigation.active"] = active
    if active and previous_active is not True:
      self._push(log_mono_ns, "nav", "navigation_active", cooldown_ns=0)
    elif not active and previous_active is True:
      self._push(log_mono_ns, "nav", "navigation_ended", cooldown_ns=0)

    off_route = connected and _bool(status, "offRoute")
    previous_off_route = self._previous.get("carrotNavi.navigation.offRoute")
    self._previous["carrotNavi.navigation.offRoute"] = off_route
    if off_route and previous_off_route is not True:
      self._push(log_mono_ns, "nav", "navigation_off_route", cooldown_ns=0)
    elif not off_route and previous_off_route is True:
      self._push(log_mono_ns, "nav", "navigation_route_recovered", cooldown_ns=0)

    source = value if connected else None
    self._push_guidance(_get(source, "guidanceCurrent"), "current", log_mono_ns)
    self._push_guidance(_get(source, "guidanceNext"), "next", log_mono_ns)
    self._ingest_lane_guidance(_get(source, "laneCurrent"), log_mono_ns)
    self._ingest_navigation_speed(_get(source, "speed"), log_mono_ns)
    self._ingest_traffic_signal(_get(source, "trafficSignal"), log_mono_ns)
    self._ingest_crossroad(_get(source, "crossroad"), log_mono_ns)

  def _ingest_carrotMan(self, value: Any, log_mono_ns: int) -> None:
    self._changed(
      "carrotMan.activeCarrot", _int(value, "activeCarrot"), log_mono_ns,
      "carrot", "carrot_mode_changed",
    )
    self._transition(
      "carrotMan.speedControl", _int(value, "xSpdType") > 0, log_mono_ns,
      "carrot", "carrot_speed_control_start", "carrot_speed_control_end",
    )
    turn_info = _int(value, "xTurnInfo")
    self._transition(
      "carrotMan.turnControl", turn_info != 0, log_mono_ns,
      "carrot", "carrot_turn_control_start", "carrot_turn_control_end",
      params={"turnInfo": turn_info},
    )
    command_index = _int(value, "carrotCmdIndex")
    previous_command_index = self._previous.get("carrotMan.commandIndex")
    self._previous["carrotMan.commandIndex"] = command_index
    command = _text(value, "carrotCmd")
    argument = _text(value, "carrotArg")
    if previous_command_index is not None and command_index != previous_command_index and command:
      self._push(
        log_mono_ns,
        "carrot",
        "carrot_command_received",
        params={"command": command, "argument": argument},
        source_detail=" ".join(part for part in (command, argument) if part),
        dedupe_key=f"{command_index}:{command}:{argument}",
        cooldown_ns=0,
      )
    title = _text(value, "szTBTMainText") or _text(value, "szPosRoadName")
    distance_m = max(0, _int(value, "xDistToTurn"))
    nav_key = f"{turn_info}|{title}" if turn_info else ""
    previous_nav = self._previous.get("carrotMan.navigation", "")
    self._previous["carrotMan.navigation"] = nav_key
    if nav_key and nav_key != previous_nav:
      self._push(
        log_mono_ns,
        "nav",
        "navigation_maneuver",
        params={"turnInfo": turn_info, "distanceM": distance_m},
        source_title=title,
        dedupe_key=nav_key,
      )

  def _ingest_driverMonitoringState(self, value: Any, log_mono_ns: int) -> None:
    self._transition(
      "driver.lockout", _bool(value, "lockout"), log_mono_ns,
      "warning", "driver_monitoring_lockout",
    )
    alert_level = _text(value, "alertLevel").lower()
    previous_level = self._previous.get("driver.alertLevel")
    self._previous["driver.alertLevel"] = alert_level
    if previous_level is not None and previous_level != alert_level:
      if alert_level not in ("", "none", "0"):
        self._push(
          log_mono_ns,
          "driver",
          "driver_attention_warning",
          params={"level": alert_level},
          dedupe_key=alert_level,
        )
      elif previous_level not in ("", "none", "0"):
        self._push(log_mono_ns, "driver", "driver_attention_restored")

    vision_state = _get(value, "visionPolicyState")
    if vision_state is not None:
      self._session(
        "driver.distracted", _bool(vision_state, "isDistracted"), log_mono_ns,
        "driver", "driver_distracted", "driver_attention_restored", minimum_ms=750,
      )

  def finalize(self, base_log_mono_ns: int, duration_ms: int) -> list[dict[str, Any]]:
    if not self._finalized:
      self._finalized = True
      end_ns = base_log_mono_ns + max(0, duration_ms) * 1_000_000
      for _key, session in tuple(self._sessions.items()):
        if end_ns - session.start_ns < session.minimum_ns:
          continue
        duration = max(0, round((end_ns - session.start_ns) / 1_000_000))
        params = {**session.params, "durationMs": duration}
        self._push(session.start_ns, session.category, session.start_type, params=params, cooldown_ns=0)
      self._sessions.clear()

    normalized: list[dict[str, Any]] = []
    indexed_events = (
      item
      for item in self._events
      if not (self._has_carrot_navi and item.event_type == "navigation_maneuver")
    )
    for item in sorted(indexed_events, key=lambda event: event.log_mono_ns):
      time_ms = round((item.log_mono_ns - base_log_mono_ns) / 1_000_000)
      if time_ms < 0 or time_ms > duration_ms:
        continue
      normalized.append({
        "id": f"event-{time_ms}-{len(normalized)}",
        "timeMs": time_ms,
        "category": item.category,
        "type": item.event_type,
        "params": item.params,
        "sourceTitle": item.source_title,
        "sourceDetail": item.source_detail,
        "sourceTag": (
          "CarrotMan" if item.event_type in {
            "carrot_mode_changed", "carrot_speed_control_start", "carrot_speed_control_end",
            "carrot_turn_control_start", "carrot_turn_control_end", "carrot_command_received",
            "navigation_maneuver",
          }
          else "CarrotNavi" if item.event_type.startswith((
            "navigation_", "navi_", "lane_guidance_", "speed_alert_", "road_speed_",
            "section_control_", "traffic_signal_", "crossroad_guidance_",
          ))
          else ""
        ),
      })
    return normalized


def event_index_metadata(events: list[dict[str, Any]]) -> dict[str, Any]:
  category_counts: dict[str, int] = {}
  type_counts: dict[str, int] = {}
  for event in events:
    category = str(event.get("category") or "event")
    event_type = str(event.get("type") or "unknown")
    category_counts[category] = category_counts.get(category, 0) + 1
    type_counts[event_type] = type_counts.get(event_type, 0) + 1
  return {
    "eventIndexVersion": REPLAY_EVENT_INDEX_VERSION,
    "eventIndex": events,
    "eventCount": len(events),
    "eventCategoryCounts": category_counts,
    "eventTypeCounts": type_counts,
  }
