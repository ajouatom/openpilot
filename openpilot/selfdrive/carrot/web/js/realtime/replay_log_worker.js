"use strict";

self.importScripts("/js/vendor/fzstd.js?v=0.1.1", "/js/realtime/raw_capnp.js?v=2607-07");

const rawCapnp = self.CarrotRawCapnp;
const replayHudServices = new Set(rawCapnp?.HUD_SERVICES || []);
const compactServices = new Set([
  ...replayHudServices,
  "modelV2", "liveCalibration", "roadCameraState", "lateralPlan",
  "radarState", "carControl", "liveDelay", "liveTorqueParameters", "liveParameters",
  // AR anchor inputs use the same decoded display subset as live compact state.
  "liveTracks", "cameraOdometry", "livePose", "carrotNavi",
]);
// These services are already emitted at the display/camera cadence. Keep each
// recorded sample instead of forcing it through epoch-aligned 50 ms buckets:
// logger jitter around a bucket edge otherwise merges one frame and leaves the
// following video frame with stale geometry.
const frameCriticalServices = new Set([
  "modelV2", "lateralPlan", "cameraOdometry", "livePose",
]);
const serviceIntervalsNs = {
  modelV2: 50_000_000,
  liveTracks: 100_000_000,
  cameraOdometry: 50_000_000,
  livePose: 50_000_000,
  carState: 30_000_000,
  controlsState: 30_000_000,
  longitudinalPlan: 50_000_000,
  carControl: 30_000_000,
  radarState: 50_000_000,
  lateralPlan: 50_000_000,
  carrotMan: 100_000_000,
  carrotNavi: 500_000_000,
  roadCameraState: 250_000_000,
  deviceState: 500_000_000,
  peripheralState: 500_000_000,
  gpsLocationExternal: 500_000_000,
  selfdriveState: 200_000_000,
  liveCalibration: 250_000_000,
  liveParameters: 250_000_000,
  liveTorqueParameters: 250_000_000,
  liveDelay: 250_000_000,
};
const BATCH_WINDOW_MS = 12;
const MAX_DURATION_MS = 3 * 60 * 1000;
const EVENT_INDEX_VERSION = 2;
const NAV_APPROACH_THRESHOLDS_M = Object.freeze([300, 200, 100, 50, 20]);
const NAVI_PREROLL_MAX_AGE_NS = 1_500_000_000;
const NAVI_PREROLL_LOOKAHEAD_NS = 750_000_000;
const categoryCooldownNs = {
  control: 500_000_000,
  driver: 800_000_000,
  vehicle: 350_000_000,
  carrot: 1_000_000_000,
  warning: 8_000_000_000,
  turn: 1_500_000_000,
  nav: 7_000_000_000,
};

function replaySelectionKey(service, logMonoTime) {
  if (frameCriticalServices.has(service)) return logMonoTime;
  const intervalNs = serviceIntervalsNs[service] || 50_000_000;
  return Math.floor(logMonoTime / intervalNs);
}

function enumName(value, names) {
  if (typeof value === "string") return value.split(".").pop() || "";
  return names[Number(value)] || "";
}

function finiteNumber(value, fallback = 0) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function naviPresent(value) {
  return Boolean(value && typeof value === "object" && value.present === true);
}

class ReplayEventIndexer {
  constructor() {
    this.events = [];
    this.previous = new Map();
    this.lastEventAt = new Map();
    this.sessions = new Map();
    this.lastLaneDirection = "";
    this.hasCarrotNavi = false;
  }

  push(logMonoTime, category, type, options = {}) {
    if (!Number.isFinite(logMonoTime) || logMonoTime <= 0 || !type) return;
    const key = `${category}:${type}:${options.dedupeKey || ""}`;
    const cooldownNs = options.cooldownNs == null
      ? (categoryCooldownNs[category] || 500_000_000)
      : Number(options.cooldownNs);
    const previous = Number(this.lastEventAt.get(key)) || 0;
    if (previous > 0 && logMonoTime - previous < cooldownNs) return;
    this.lastEventAt.set(key, logMonoTime);
    this.events.push({
      logMonoTime,
      category,
      type,
      params: { ...(options.params || {}) },
      sourceTitle: String(options.sourceTitle || "").trim(),
      sourceDetail: String(options.sourceDetail || "").trim(),
      sourceTag: String(options.sourceTag || "").trim(),
    });
  }

  transition(key, value, time, category, onType, offType = "", params = null, sourceTag = "") {
    const previous = this.previous.get(key);
    this.previous.set(key, value);
    if (previous == null || previous === value) return;
    const type = value ? onType : offType;
    if (type) this.push(time, category, type, { params, sourceTag });
  }

  changed(key, value, time, category, type, ignore = [], sourceTag = "") {
    const previous = this.previous.get(key);
    this.previous.set(key, value);
    if (previous == null || previous === value || ignore.includes(value)) return;
    this.push(time, category, type, {
      params: { from: previous, value },
      dedupeKey: String(value),
      sourceTag,
    });
  }

  session(key, active, time, category, startType, endType, options = {}) {
    const previous = this.previous.get(key);
    this.previous.set(key, active);
    if (previous == null) return;
    if (active && !previous) {
      this.sessions.set(key, {
        startTime: time,
        category,
        startType,
        endType,
        params: { ...(options.params || {}) },
        minimumNs: Math.max(0, Number(options.minimumMs) || 120) * 1_000_000,
      });
      return;
    }
    if (!active && previous) {
      const pending = this.sessions.get(key);
      this.sessions.delete(key);
      if (!pending || time - pending.startTime < pending.minimumNs) return;
      const params = { ...pending.params, durationMs: Math.max(0, Math.round((time - pending.startTime) / 1_000_000)) };
      this.push(pending.startTime, pending.category, pending.startType, { params, cooldownNs: 0 });
      this.push(time, pending.category, pending.endType, { params, cooldownNs: 0 });
    }
  }

  warningEdge(value, field, type, time, key = `carState.${field}`) {
    const active = Boolean(value?.[field]);
    const previous = this.previous.get(key);
    this.previous.set(key, active);
    if (active && previous === false) this.push(time, "warning", type);
  }

  ingest(event) {
    const service = event?.service;
    const value = event?.decoded;
    const time = Number(event?.logMonoTime);
    if (!service || !value || !Number.isFinite(time)) return;
    if (service === "selfdriveState") this.ingestSelfdrive(value, time);
    else if (service === "carControl") this.ingestCarControl(value, time);
    else if (service === "carState") this.ingestCarState(value, time);
    else if (service === "controlsState") this.ingestControls(value, time);
    else if (service === "longitudinalPlan") this.ingestLongitudinal(value, time);
    else if (service === "lateralPlan") this.ingestLateral(value, time);
    else if (service === "carrotNavi") this.ingestCarrotNavi(value, time);
    else if (service === "carrotMan") this.ingestCarrot(value, time);
    else if (service === "driverMonitoringState") this.ingestDriver(value, time);
  }

  ingestSelfdrive(value, time) {
    this.transition("selfdrive.enabled", Boolean(value.enabled), time, "control", "control_engaged", "control_disengaged");
    const state = enumName(value.state, ["disabled", "preEnabled", "enabled", "softDisabling", "overriding"]);
    this.transition("selfdrive.overriding", state === "overriding", time, "control", "control_override_start", "control_override_end");
    this.transition("selfdrive.experimental", Boolean(value.experimentalMode), time, "control", "experimental_mode_on", "experimental_mode_off");
    this.transition("selfdrive.engageable", Boolean(value.engageable), time, "control", "control_available", "control_unavailable");
    const personality = enumName(value.personality, ["aggressive", "standard", "relaxed", "moreRelaxed"]);
    this.changed("selfdrive.personality", personality, time, "control", "driving_personality_changed", [""]);

    const alertType = String(value.alertType || "").trim();
    const alertTitle = String(value.alertText1 || "").trim();
    const alertDetail = String(value.alertText2 || "").trim();
    const alertStatus = Number(value.alertStatus || 0);
    const alertKey = alertType || alertTitle ? `${alertType}|${alertTitle}|${alertDetail}` : "";
    const previousAlert = this.previous.get("selfdrive.alert") || "";
    this.previous.set("selfdrive.alert", alertKey);
    if (alertKey && alertKey !== previousAlert && alertStatus > 0) {
      this.push(time, "warning", "system_alert", {
        params: { alertStatus, alertType },
        sourceTitle: alertTitle,
        sourceDetail: alertDetail,
        dedupeKey: alertType || alertTitle,
      });
    }
  }

  ingestCarControl(value, time) {
    this.transition("carControl.latActive", Boolean(value.latActive), time, "control", "lateral_control_on", "lateral_control_off");
    this.transition("carControl.longActive", Boolean(value.longActive), time, "control", "longitudinal_control_on", "longitudinal_control_off");
    const cruise = value.cruiseControl;
    if (!cruise) return;
    this.transition("carControl.override", Boolean(cruise.override), time, "control", "control_override_start", "control_override_end");
    this.transition("carControl.cancel", Boolean(cruise.cancel), time, "control", "cruise_cancel_requested");
    this.transition("carControl.resume", Boolean(cruise.resume), time, "control", "cruise_resume_requested");
  }

  ingestCarState(value, time) {
    this.session("carState.gasPressed", Boolean(value.gasPressed), time, "vehicle", "accelerator_pressed", "accelerator_released", { minimumMs: 100 });
    this.session("carState.brakePressed", Boolean(value.brakePressed), time, "vehicle", "brake_pressed", "brake_released", { minimumMs: 100 });
    this.session("carState.steeringPressed", Boolean(value.steeringPressed), time, "driver", "steering_override_start", "steering_override_end", { minimumMs: 180 });
    this.transition("carState.leftBlinker", Boolean(value.leftBlinker), time, "vehicle", "left_blinker_on");
    this.transition("carState.rightBlinker", Boolean(value.rightBlinker), time, "vehicle", "right_blinker_on");
    if (value.cruiseState) {
      this.transition("carState.cruiseEnabled", Boolean(value.cruiseState.enabled), time, "control", "cruise_enabled", "cruise_disabled");
    }
    this.changed("carState.gear", String(value.gearShifter || ""), time, "vehicle", "gear_changed", ["", "unknown"]);
    this.transition("carState.standstill", Boolean(value.standstill), time, "vehicle", "vehicle_stopped", "vehicle_moved");
    this.transition("carState.brakeHold", Boolean(value.brakeHoldActive), time, "vehicle", "brake_hold_on", "brake_hold_off");
    this.transition("carState.parkingBrake", Boolean(value.parkingBrake), time, "vehicle", "parking_brake_on", "parking_brake_off");
    this.session("carState.clutchPressed", Boolean(value.clutchPressed), time, "vehicle", "clutch_pressed", "clutch_released", { minimumMs: 100 });
    this.changed("carState.softHold", Number(value.softHoldActive) || 0, time, "vehicle", "soft_hold_changed");
    this.changed("carState.cruiseGap", Number(value.pcmCruiseGap) || 0, time, "vehicle", "cruise_gap_changed", [0]);
    this.changed("carState.carrotCruise", Number(value.carrotCruise) || 0, time, "carrot", "carrot_cruise_changed");
    this.transition("carState.buttonEnable", Boolean(value.buttonEnable), time, "vehicle", "enable_button_pressed");

    this.warningEdge(value, "stockAeb", "stock_aeb", time);
    this.warningEdge(value, "stockFcw", "stock_fcw", time);
    this.warningEdge(value, "canTimeout", "can_timeout", time);
    this.warningEdge(value, "steerFaultTemporary", "steering_fault_temporary", time);
    this.warningEdge(value, "steerFaultPermanent", "steering_fault_permanent", time);
    this.warningEdge(value, "accFaulted", "acc_fault", time);
    this.warningEdge(value, "vehicleSensorsInvalid", "vehicle_sensors_invalid", time);
    this.warningEdge(value, "lowSpeedAlert", "low_speed_steering_alert", time);
    this.warningEdge(value, "carFaultedNonCritical", "vehicle_fault", time);
    this.warningEdge(value, "espDisabled", "stability_control_disabled", time);
    this.warningEdge(value, "doorOpen", "door_open", time);
    this.warningEdge(value, "seatbeltUnlatched", "seatbelt_unlatched", time);
    this.transition("carState.leftBlindspotHazard", Boolean(value.leftBlindspot && value.leftBlinker), time, "warning", "left_blindspot_warning");
    this.transition("carState.rightBlindspotHazard", Boolean(value.rightBlindspot && value.rightBlinker), time, "warning", "right_blindspot_warning");
    const canValid = Boolean(value.canValid);
    const previousCanValid = this.previous.get("carState.canValid");
    this.previous.set("carState.canValid", canValid);
    if (previousCanValid === true && !canValid) this.push(time, "warning", "can_invalid");

    for (const button of Array.isArray(value.buttonEvents) ? value.buttonEvents : []) {
      if (!button?.pressed) continue;
      const buttonType = String(button.type || "unknown");
      this.push(time, "vehicle", "vehicle_button_pressed", {
        params: { button: buttonType },
        dedupeKey: buttonType,
        cooldownNs: 180_000_000,
      });
    }
  }

  ingestControls(value, time) {
    this.transition("controls.forceDecel", Boolean(value.forceDecel), time, "warning", "forced_deceleration");
    const controlState = enumName(value.longControlState, ["off", "pid", "stopping", "starting"]);
    const previous = this.previous.get("controls.longControlState");
    this.previous.set("controls.longControlState", controlState);
    if (previous == null || previous === controlState) return;
    if (controlState === "stopping") this.push(time, "carrot", "longitudinal_stopping");
    else if (controlState === "starting") this.push(time, "carrot", "longitudinal_starting");
  }

  ingestLongitudinal(value, time) {
    this.transition("longPlan.hasLead", Boolean(value.hasLead), time, "carrot", "lead_acquired", "lead_lost");
    this.transition("longPlan.shouldStop", Boolean(value.shouldStop), time, "carrot", "planned_stop_start", "planned_stop_end");
    this.warningEdge(value, "fcw", "forward_collision_warning", time, "longPlan.fcw");
    this.changed("longPlan.drivingMode", Number(value.myDrivingMode) || 0, time, "carrot", "driving_mode_changed");
  }

  ingestLateral(value, time) {
    const state = enumName(value.laneChangeState, ["off", "preLaneChange", "laneChangeStarting", "laneChangeFinishing"]);
    const direction = enumName(value.laneChangeDirection, ["none", "left", "right"]);
    const previous = this.previous.get("lateralPlan.laneChangeState");
    this.previous.set("lateralPlan.laneChangeState", state);
    if (direction && direction !== "none") this.lastLaneDirection = direction;
    const wasActive = previous != null && previous !== "" && previous !== "off";
    const active = state !== "" && state !== "off";
    if (!wasActive && active) {
      const type = direction === "left" ? "lane_change_left" : direction === "right" ? "lane_change_right" : "lane_change";
      this.push(time, "turn", type, { params: { direction }, dedupeKey: direction });
    } else if (wasActive && !active) {
      this.push(time, "turn", "lane_change_completed", {
        params: { direction: this.lastLaneDirection },
        dedupeKey: this.lastLaneDirection,
      });
      this.lastLaneDirection = "";
    }
  }

  pushNavi(time, type, options = {}) {
    this.push(time, "nav", type, { ...options, sourceTag: "CarrotNavi" });
  }

  guidanceKey(guidance) {
    if (!naviPresent(guidance)) return "";
    const turnType = Math.trunc(finiteNumber(guidance.turnType));
    const point = guidance.pointValid
      ? `${finiteNumber(guidance.latitude).toFixed(7)},${finiteNumber(guidance.longitude).toFixed(7)}`
      : String(guidance.mainText || guidance.roadName || "").trim();
    return `${turnType}|${point}`;
  }

  ingestNaviGuidance(guidance, slot, time) {
    const key = this.guidanceKey(guidance);
    const stateKey = `carrotNavi.guidance.${slot}`;
    const previousKey = String(this.previous.get(stateKey) || "");
    const previousNext = String(this.previous.get("carrotNavi.guidance.next") || "");
    this.previous.set(stateKey, key);

    if (!key) {
      if (previousKey) this.pushNavi(time, `navigation_maneuver_${slot}_cleared`, { dedupeKey: slot, cooldownNs: 0 });
      if (slot === "current") this.previous.delete("carrotNavi.guidance.currentDistance");
      return;
    }

    const distanceM = Math.max(0, Math.trunc(finiteNumber(guidance.distanceM)));
    const turnType = Math.trunc(finiteNumber(guidance.turnType));
    const title = String(guidance.mainText || guidance.roadName || "").trim();
    const roadName = String(guidance.roadName || "").trim();
    const params = { turnType, distanceM, roadName };
    if (slot === "current" && key === previousNext && key !== previousKey) params.promotedFromNext = true;
    if (key !== previousKey) {
      this.pushNavi(time, `navigation_maneuver_${slot}`, {
        params,
        sourceTitle: title,
        sourceDetail: roadName !== title ? roadName : "",
        dedupeKey: key,
        cooldownNs: 0,
      });
    }

    if (slot !== "current" || distanceM <= 0) return;
    const distanceKey = "carrotNavi.guidance.currentDistance";
    const previousDistance = this.previous.get(distanceKey);
    this.previous.set(distanceKey, [key, distanceM]);
    const thresholds = [];
    if (!Array.isArray(previousDistance) || previousDistance[0] !== key) {
      const eligible = NAV_APPROACH_THRESHOLDS_M.filter((threshold) => distanceM <= threshold);
      if (eligible.length) thresholds.push(Math.min(...eligible));
    } else if (distanceM < previousDistance[1]) {
      thresholds.push(...NAV_APPROACH_THRESHOLDS_M.filter((threshold) => previousDistance[1] > threshold && threshold >= distanceM));
    }
    for (const thresholdM of thresholds) {
      this.pushNavi(time, "navigation_approach", {
        params: { ...params, thresholdM },
        sourceTitle: title,
        sourceDetail: roadName !== title ? roadName : "",
        dedupeKey: `${key}:${thresholdM}`,
        cooldownNs: 0,
      });
    }
  }

  ingestNaviLane(lane, time) {
    const visible = naviPresent(lane) && Boolean(lane.visible);
    const available = Array.isArray(lane?.available) ? lane.available.map((item) => Math.trunc(finiteNumber(item))).slice(0, 16) : [];
    const laneKey = JSON.stringify([Math.trunc(finiteNumber(lane?.count)), available]);
    const previousVisible = this.previous.get("carrotNavi.lane.visible");
    const previousKey = this.previous.get("carrotNavi.lane.key");
    this.previous.set("carrotNavi.lane.visible", visible);
    this.previous.set("carrotNavi.lane.key", laneKey);
    if (visible && (previousVisible !== true || laneKey !== previousKey)) {
      this.pushNavi(time, previousVisible !== true ? "lane_guidance_shown" : "lane_guidance_changed", {
        params: {
          laneCount: Math.trunc(finiteNumber(lane.count)),
          available,
          distanceM: Math.max(0, Math.trunc(finiteNumber(lane.distanceM))),
        },
        dedupeKey: laneKey,
        cooldownNs: 0,
      });
    } else if (!visible && previousVisible === true) {
      this.pushNavi(time, "lane_guidance_hidden", { cooldownNs: 0 });
    }
  }

  ingestNaviSpeed(speed, time) {
    const present = Boolean(speed?.sdiPresent);
    const alertKey = JSON.stringify([
      Math.trunc(finiteNumber(speed?.sdiType)),
      Math.trunc(finiteNumber(speed?.sdiSpeedLimitKph)),
    ]);
    const previousAlert = this.previous.get("carrotNavi.speedAlert.primary");
    this.previous.set("carrotNavi.speedAlert.primary", [present, alertKey]);
    const params = {
      source: "primary",
      sdiType: Math.trunc(finiteNumber(speed?.sdiType)),
      speedLimitKph: Math.trunc(finiteNumber(speed?.sdiSpeedLimitKph)),
      distanceM: Math.max(0, Math.trunc(finiteNumber(speed?.sdiDistanceM))),
    };
    if (present && (!Array.isArray(previousAlert) || previousAlert[0] !== true)) {
      this.pushNavi(time, "speed_alert_shown", { params, dedupeKey: `primary:${alertKey}`, cooldownNs: 0 });
    } else if (present && previousAlert[0] === true && previousAlert[1] !== alertKey) {
      this.pushNavi(time, "speed_alert_changed", { params, dedupeKey: `primary:${alertKey}`, cooldownNs: 0 });
    } else if (!present && Array.isArray(previousAlert) && previousAlert[0] === true) {
      this.pushNavi(time, "speed_alert_cleared", { params: { source: "primary" }, cooldownNs: 0 });
    }

    const roadLimit = speed?.roadLimitValid ? Math.trunc(finiteNumber(speed.roadLimitKph)) : 0;
    const previousLimit = this.previous.get("carrotNavi.roadLimitKph");
    this.previous.set("carrotNavi.roadLimitKph", roadLimit);
    if (previousLimit > 0 && roadLimit > 0 && roadLimit !== previousLimit) {
      this.pushNavi(time, "road_speed_limit_changed", {
        params: { from: previousLimit, value: roadLimit },
        dedupeKey: String(roadLimit),
        cooldownNs: 0,
      });
    }

    const sectionActive = Boolean(speed?.sectionPresent && speed?.sectionActive);
    const previousActive = this.previous.get("carrotNavi.section.active");
    this.previous.set("carrotNavi.section.active", sectionActive);
    const sectionParams = {
      speedLimitKph: Math.trunc(finiteNumber(speed?.sectionSpeedLimitKph)),
      averageKph: Math.round(finiteNumber(speed?.sectionAverageKph) * 10) / 10,
      remainingDistanceM: Math.round(Math.max(0, finiteNumber(speed?.sectionRemainingDistanceM)) * 10) / 10,
    };
    if (sectionActive && previousActive !== true) this.pushNavi(time, "section_control_started", { params: sectionParams, cooldownNs: 0 });
    else if (!sectionActive && previousActive === true) this.pushNavi(time, "section_control_ended", { cooldownNs: 0 });
  }

  ingestNaviSignal(signal, time) {
    const visible = Boolean(signal?.visible);
    const names = ["red", "left", "green", "right", "uturn"];
    const signalKey = JSON.stringify(names.map((name) => [Boolean(signal?.[`${name}Valid`]), Boolean(signal?.[`${name}On`])]));
    const previousVisible = this.previous.get("carrotNavi.signal.visible");
    const previousKey = this.previous.get("carrotNavi.signal.key");
    this.previous.set("carrotNavi.signal.visible", visible);
    this.previous.set("carrotNavi.signal.key", signalKey);
    const params = { distanceM: Math.max(0, Math.trunc(finiteNumber(signal?.distanceM))) };
    for (const name of names) params[name] = signal?.[`${name}Valid`] ? Boolean(signal[`${name}On`]) : null;
    if (visible && (previousVisible !== true || signalKey !== previousKey)) {
      this.pushNavi(time, previousVisible !== true ? "traffic_signal_shown" : "traffic_signal_changed", {
        params,
        dedupeKey: signalKey,
        cooldownNs: 0,
      });
    } else if (!visible && previousVisible === true) {
      this.pushNavi(time, "traffic_signal_hidden", { cooldownNs: 0 });
    }
  }

  ingestNaviCrossroad(crossroad, time) {
    const visible = Boolean(crossroad?.visible);
    const imageCode = Math.trunc(finiteNumber(crossroad?.imageCode));
    const previousVisible = this.previous.get("carrotNavi.crossroad.visible");
    const previousCode = this.previous.get("carrotNavi.crossroad.imageCode");
    this.previous.set("carrotNavi.crossroad.visible", visible);
    this.previous.set("carrotNavi.crossroad.imageCode", imageCode);
    if (visible && (previousVisible !== true || imageCode !== previousCode)) {
      this.pushNavi(time, previousVisible !== true ? "crossroad_guidance_shown" : "crossroad_guidance_changed", {
        params: { imageCode, distanceM: Math.max(0, Math.trunc(finiteNumber(crossroad?.distanceM))) },
        dedupeKey: String(imageCode),
        cooldownNs: 0,
      });
    } else if (!visible && previousVisible === true) {
      this.pushNavi(time, "crossroad_guidance_hidden", { cooldownNs: 0 });
    }
  }

  ingestCarrotNavi(value, time) {
    const connected = Boolean(value.connected);
    if (connected) this.hasCarrotNavi = true;
    const previousConnected = this.previous.get("carrotNavi.connected");
    this.previous.set("carrotNavi.connected", connected);
    if (previousConnected != null && connected !== previousConnected) {
      this.pushNavi(time, connected ? "navi_connected" : "navi_disconnected", { cooldownNs: 0 });
    }

    // generation advances with published snapshots; it is not a route/session
    // identity and must never create one visible event every few seconds.
    const sessionKey = String(value.sessionId || "").trim();
    const previousSession = this.previous.get("carrotNavi.session");
    this.previous.set("carrotNavi.session", sessionKey);
    if (previousSession && sessionKey && sessionKey !== previousSession) this.pushNavi(time, "navigation_session_changed", { cooldownNs: 0 });

    const status = value.navigationStatus || {};
    const active = connected && Boolean(status.guidanceActive) && Boolean(status.routePresent);
    const previousActive = this.previous.get("carrotNavi.navigation.active");
    this.previous.set("carrotNavi.navigation.active", active);
    if (active && previousActive !== true) this.pushNavi(time, "navigation_active", { cooldownNs: 0 });
    else if (!active && previousActive === true) this.pushNavi(time, "navigation_ended", { cooldownNs: 0 });

    const offRoute = connected && Boolean(status.offRoute);
    const previousOffRoute = this.previous.get("carrotNavi.navigation.offRoute");
    this.previous.set("carrotNavi.navigation.offRoute", offRoute);
    if (offRoute && previousOffRoute !== true) this.pushNavi(time, "navigation_off_route", { cooldownNs: 0 });
    else if (!offRoute && previousOffRoute === true) this.pushNavi(time, "navigation_route_recovered", { cooldownNs: 0 });

    const source = connected ? value : {};
    this.ingestNaviGuidance(source.guidanceCurrent, "current", time);
    this.ingestNaviGuidance(source.guidanceNext, "next", time);
    this.ingestNaviLane(source.laneCurrent, time);
    this.ingestNaviSpeed(source.speed, time);
    this.ingestNaviSignal(source.trafficSignal, time);
    this.ingestNaviCrossroad(source.crossroad, time);
  }

  ingestCarrot(value, time) {
    this.changed("carrotMan.activeCarrot", Number(value.activeCarrot) || 0, time, "carrot", "carrot_mode_changed", [], "CarrotMan");
    this.transition("carrotMan.speedControl", Number(value.xSpdType) > 0, time, "carrot", "carrot_speed_control_start", "carrot_speed_control_end", null, "CarrotMan");
    const turnInfo = Number(value.xTurnInfo) || 0;
    this.transition("carrotMan.turnControl", turnInfo !== 0, time, "carrot", "carrot_turn_control_start", "carrot_turn_control_end", { turnInfo }, "CarrotMan");
    const commandIndex = Number(value.carrotCmdIndex) || 0;
    const previousCommand = this.previous.get("carrotMan.commandIndex");
    this.previous.set("carrotMan.commandIndex", commandIndex);
    const command = String(value.carrotCmd || "").trim();
    const argument = String(value.carrotArg || "").trim();
    if (previousCommand != null && commandIndex !== previousCommand && command) {
      this.push(time, "carrot", "carrot_command_received", {
        params: { command, argument },
        sourceDetail: [command, argument].filter(Boolean).join(" "),
        sourceTag: "CarrotMan",
        dedupeKey: `${commandIndex}:${command}:${argument}`,
        cooldownNs: 0,
      });
    }
    const title = String(value.szTBTMainText || value.szPosRoadName || "").trim();
    const navKey = turnInfo ? `${turnInfo}|${title}` : "";
    const previousNav = this.previous.get("carrotMan.navigation") || "";
    this.previous.set("carrotMan.navigation", navKey);
    if (navKey && navKey !== previousNav) {
      this.push(time, "nav", "navigation_maneuver", {
        params: { turnInfo, distanceM: Math.max(0, Number(value.xDistToTurn) || 0) },
        sourceTitle: title,
        sourceTag: "CarrotMan",
        dedupeKey: navKey,
      });
    }
  }

  ingestDriver(value, time) {
    this.transition("driver.lockout", Boolean(value.lockout), time, "warning", "driver_monitoring_lockout");
    const level = enumName(value.alertLevel, ["none", "one", "two", "three"]);
    const previousLevel = this.previous.get("driver.alertLevel");
    this.previous.set("driver.alertLevel", level);
    if (previousLevel != null && previousLevel !== level) {
      if (level && level !== "none") {
        this.push(time, "driver", "driver_attention_warning", { params: { level }, dedupeKey: level });
      } else if (previousLevel && previousLevel !== "none") {
        this.push(time, "driver", "driver_attention_restored");
      }
    }
    this.session("driver.distracted", Boolean(value.visionPolicyState?.isDistracted), time, "driver", "driver_distracted", "driver_attention_restored", { minimumMs: 750 });
  }

  finalize(baseMonoTime, durationMs) {
    const endTime = baseMonoTime + Math.max(0, durationMs) * 1_000_000;
    for (const pending of this.sessions.values()) {
      if (endTime - pending.startTime < pending.minimumNs) continue;
      const params = { ...pending.params, durationMs: Math.max(0, Math.round((endTime - pending.startTime) / 1_000_000)) };
      this.push(pending.startTime, pending.category, pending.startType, { params, cooldownNs: 0 });
    }
    this.sessions.clear();
    return this.events
      .filter((event) => !(this.hasCarrotNavi && event.type === "navigation_maneuver"))
      .sort((left, right) => left.logMonoTime - right.logMonoTime)
      .map((event, index) => ({
        id: `event-${Math.max(0, Math.round((event.logMonoTime - baseMonoTime) / 1_000_000))}-${index}`,
        timeMs: Math.round((event.logMonoTime - baseMonoTime) / 1_000_000),
        category: event.category,
        type: event.type,
        params: event.params,
        sourceTitle: event.sourceTitle,
        sourceDetail: event.sourceDetail,
        sourceTag: event.sourceTag,
      }))
      .filter((event) => event.timeMs >= 0 && event.timeMs <= durationMs);
  }
}

function appendBytes(left, right) {
  if (!left?.byteLength) return right;
  if (!right?.byteLength) return left;
  const joined = new Uint8Array(left.byteLength + right.byteLength);
  joined.set(left, 0);
  joined.set(right, left.byteLength);
  return joined;
}

class ReplayLogBuilder {
  constructor(expectedSegment) {
    this.expectedSegment = Number(expectedSegment) || 0;
    this.pending = new Uint8Array(0);
    this.selected = new Map();
    for (const service of compactServices) this.selected.set(service, new Map());
    this.cameraIndexes = [];
    this.rawEventCount = 0;
    this.selectedEventCount = 0;
    this.decodeErrorCount = 0;
    this.partialReason = "";
    this.truncatedTailBytes = 0;
    this.eventIndexer = new ReplayEventIndexer();
  }

  markPartial(reason, tailBytes = 0) {
    if (!this.partialReason) this.partialReason = String(reason || "Recorded log ended early").slice(0, 320);
    this.truncatedTailBytes = Math.max(this.truncatedTailBytes, Number(tailBytes) || 0);
  }

  push(chunk) {
    this.pending = appendBytes(this.pending, chunk);
    let offset = 0;
    while (offset < this.pending.byteLength) {
      let messageLength = 0;
      try {
        messageLength = rawCapnp.framedMessageByteLength(this.pending.subarray(offset));
      } catch (error) {
        throw new Error(`Recorded log framing failed: ${error?.message || error}`);
      }
      if (!messageLength) break;
      const message = this.pending.subarray(offset, offset + messageLength);
      this.ingest(message);
      offset += messageLength;
    }
    this.pending = offset ? this.pending.slice(offset) : this.pending;
  }

  ingest(message) {
    this.rawEventCount += 1;
    let event = null;
    try {
      // Raw radar tracks are intentionally deferred to the sensor worker so
      // the first replay load does not decode and clone thousands of points.
      event = rawCapnp.decodeReplayEvent(message, "", "liveTracks");
    } catch {
      this.decodeErrorCount += 1;
      return;
    }
    if (!event?.service || !Number.isFinite(event.logMonoTime) || event.logMonoTime <= 0) return;
    if (event.service === "qRoadEncodeIdx") {
      const value = event.decoded || {};
      this.cameraIndexes.push({
        logMonoTime: event.logMonoTime,
        frameId: Number(value.frameId) || 0,
        segmentNum: Number(value.segmentNum),
        segmentId: Number(value.segmentId) || 0,
        timestampSof: Number(value.timestampSof) || 0,
      });
      return;
    }
    if (event.valid) this.eventIndexer.ingest(event);
    if (!compactServices.has(event.service)) return;
    // carrot-wip route replay keeps the latest decoded HUD state even when an
    // Event validity bit is down. Validity describes source health; discarding
    // the value here leaves the entire replay HUD permanently uninitialized.
    if (!event.valid && !replayHudServices.has(event.service)) return;
    const bucket = replaySelectionKey(event.service, event.logMonoTime);
    const serviceBuckets = this.selected.get(event.service);
    const previous = serviceBuckets.get(bucket);
    // Keep the latest timestamp in each selection slot even when logger socket
    // drain order is not globally monotonic. Frame-critical slots are exact
    // timestamps; slower HUD services still use bounded display buckets.
    if (!previous || event.logMonoTime >= previous.logMonoTime) {
      serviceBuckets.set(bucket, event);
    }
  }

  finish() {
    if (this.pending.byteLength) {
      this.markPartial("Recorded log ended with an incomplete Cap'n Proto message", this.pending.byteLength);
      this.pending = new Uint8Array(0);
    }
    const matchingIndexes = this.cameraIndexes.filter((item) => item.segmentNum === this.expectedSegment);
    const indexes = (matchingIndexes.length ? matchingIndexes : this.cameraIndexes)
      .sort((left, right) => (left.segmentId - right.segmentId) || (left.logMonoTime - right.logMonoTime));

    const events = [];
    for (const serviceBuckets of this.selected.values()) events.push(...serviceBuckets.values());
    events.sort((left, right) => left.logMonoTime - right.logMonoTime);
    if (!indexes.length && !events.length) throw new Error("Recorded log has no Carrot Vision display data");

    const baseMonoTime = indexes[0]?.logMonoTime || events[0].logMonoTime;
    const timestampBase = indexes[0]?.timestampSof || 0;
    const naviEvents = events.filter((event) => event.service === "carrotNavi");
    const previousNaviCandidates = naviEvents.filter((event) => (
      event.logMonoTime <= baseMonoTime && baseMonoTime - event.logMonoTime <= NAVI_PREROLL_MAX_AGE_NS
    ));
    const previousNavi = previousNaviCandidates[previousNaviCandidates.length - 1];
    const lookaheadNavi = naviEvents.find((event) => (
      event.logMonoTime > baseMonoTime && event.logMonoTime - baseMonoTime <= NAVI_PREROLL_LOOKAHEAD_NS
    ));
    const naviSeedEvent = previousNavi || lookaheadNavi || null;
    const frameVideoTime = new Map();
    let cameraDurationMs = 0;
    for (const item of indexes) {
      const videoTimeMs = timestampBase > 0 && item.timestampSof >= timestampBase
        ? Math.round((item.timestampSof - timestampBase) / 1_000_000)
        : Math.round((item.logMonoTime - baseMonoTime) / 1_000_000);
      if (videoTimeMs < 0 || videoTimeMs > MAX_DURATION_MS) continue;
      frameVideoTime.set(item.frameId, videoTimeMs);
      cameraDurationMs = Math.max(cameraDurationMs, videoTimeMs);
    }

    const sequences = new Map();
    const batches = new Map();
    const serviceCounts = {};
    let naviSeedFrame = null;
    let naviSeedSourceTimeMs = null;
    let lastSampleMs = 0;
    for (const event of events) {
      let videoTimeMs = Math.round((event.logMonoTime - baseMonoTime) / 1_000_000);
      if ((event.service === "modelV2" || event.service === "roadCameraState")
          && frameVideoTime.has(Number(event.decoded?.frameId))) {
        videoTimeMs = frameVideoTime.get(Number(event.decoded.frameId));
      }
      videoTimeMs = Math.max(0, Math.min(MAX_DURATION_MS, videoTimeMs));
      lastSampleMs = Math.max(lastSampleMs, videoTimeMs);
      const sequence = ((sequences.get(event.service) || 0) + 1) & 0xffff;
      sequences.set(event.service, sequence);
      serviceCounts[event.service] = (serviceCounts[event.service] || 0) + 1;
      const frame = { service: event.service, sequence, decoded: event.decoded, byteLength: 0 };
      if (event === naviSeedEvent) {
        naviSeedFrame = frame;
        naviSeedSourceTimeMs = Math.round((event.logMonoTime - baseMonoTime) / 1_000_000);
      }
      const batchKey = Math.floor(videoTimeMs / BATCH_WINDOW_MS);
      if (!batches.has(batchKey)) batches.set(batchKey, new Map());
      const pending = batches.get(batchKey);
      pending.delete(event.service);
      pending.set(event.service, { videoTimeMs, frame });
    }

    const records = [];
    for (const batchKey of Array.from(batches.keys()).sort((left, right) => left - right)) {
      const pending = batches.get(batchKey);
      let timeMs = 0;
      const frames = [];
      for (const item of pending.values()) {
        timeMs = Math.max(timeMs, item.videoTimeMs);
        frames.push(item.frame);
      }
      records.push({ timeMs, frames });
    }
    const hasZeroNavi = records.some((record) => (
      record.timeMs === 0 && record.frames.some((frame) => frame.service === "carrotNavi")
    ));
    if (naviSeedFrame && !hasZeroNavi) {
      records.unshift({ timeMs: 0, frames: [naviSeedFrame] });
    }
    this.selectedEventCount = events.length;
    const durationMs = Math.max(cameraDurationMs, lastSampleMs);
    const eventIndex = this.eventIndexer.finalize(baseMonoTime, durationMs);
    const eventCategoryCounts = {};
    const eventTypeCounts = {};
    for (const event of eventIndex) {
      eventCategoryCounts[event.category] = (eventCategoryCounts[event.category] || 0) + 1;
      eventTypeCounts[event.type] = (eventTypeCounts[event.type] || 0) + 1;
    }
    return {
      durationMs,
      records,
      metadata: {
        mode: "client",
        sampleCount: events.length,
        recordCount: records.length,
        cameraFrameCount: indexes.length,
        syncMode: frameVideoTime.size ? "camera-frame" : "log-time",
        services: serviceCounts,
        rawEventCount: this.rawEventCount,
        decodeErrorCount: this.decodeErrorCount,
        eventIndexVersion: EVENT_INDEX_VERSION,
        eventIndex,
        eventCategoryCounts,
        eventTypeCounts,
        preRollServices: naviSeedFrame ? ["carrotNavi"] : [],
        preRollMode: naviSeedFrame
          ? (naviSeedEvent.logMonoTime <= baseMonoTime ? "previous" : "bounded-lookahead")
          : "none",
        preRollSourceTimeMs: naviSeedSourceTimeMs,
        rawFirstMonoTimeNanos: String(baseMonoTime),
        rawParseStatus: this.partialReason || this.decodeErrorCount ? "partial" : "complete",
        rawParseError: this.partialReason,
        rawTailBytes: this.truncatedTailBytes,
      },
    };
  }
}

function isRecoverableTailError(error) {
  const message = String(error?.message || error || "");
  return Number(error?.code) === 5 || /unexpected\s+eof|unexpected\s+end|incomplete\s+(?:frame|stream)/i.test(message);
}

async function streamResponse(url, onChunk, signal) {
  const response = await fetch(url, { cache: "no-store", signal });
  if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
  const total = Number(response.headers.get("Content-Length")) || 0;
  let loaded = 0;
  if (response.body?.getReader) {
    const reader = response.body.getReader();
    while (true) {
      const result = await reader.read();
      if (result.done) break;
      loaded += result.value.byteLength;
      onChunk(result.value, false);
      self.postMessage({ type: "progress", loaded, total });
    }
  } else {
    const bytes = new Uint8Array(await response.arrayBuffer());
    loaded = bytes.byteLength;
    onChunk(bytes, false);
    self.postMessage({ type: "progress", loaded, total: total || loaded });
  }
  onChunk(new Uint8Array(0), true);
}

async function bufferedResponse(url, signal) {
  const response = await fetch(url, { cache: "no-store", signal });
  if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
  const bytes = new Uint8Array(await response.arrayBuffer());
  self.postMessage({ type: "progress", loaded: bytes.byteLength, total: bytes.byteLength });
  return bytes;
}

function decodeBzipToBuilder(compressed, builder) {
  if (!self.CarrotBzip?.decode) self.importScripts("/js/vendor/seek-bzip.js");
  if (!self.CarrotBzip?.decode) throw new Error("Client Bzip2 decoder is unavailable");
  const outputSize = 256 * 1024;
  let output = new Uint8Array(outputSize);
  let used = 0;
  const sink = {
    writeByte(value) {
      output[used] = value;
      used += 1;
      if (used === output.byteLength) {
        builder.push(output);
        output = new Uint8Array(outputSize);
        used = 0;
      }
    },
  };
  self.CarrotBzip.decode(compressed, sink, true);
  if (used) builder.push(output.subarray(0, used));
}

async function buildReplay(options) {
  if (!rawCapnp) throw new Error("Client Cap'n Proto decoder is unavailable");
  const builder = new ReplayLogBuilder(options.expectedSegment);
  const compression = String(options.compression || "");
  const controller = new AbortController();
  self.replayAbortController = controller;
  try {
    if (compression === "zstd") {
      if (!self.fzstd?.Decompress) throw new Error("Client Zstandard decoder is unavailable");
      const decoder = new self.fzstd.Decompress((chunk) => builder.push(chunk));
      await streamResponse(options.url, (chunk, final) => decoder.push(chunk, final), controller.signal);
    } else if (compression === "none") {
      await streamResponse(options.url, (chunk) => builder.push(chunk), controller.signal);
    } else if (compression === "bzip2") {
      // Historical rlogs use bzip2. Keep even this compatibility path on the
      // visiting phone/PC; output is fed to the parser in bounded chunks.
      const compressed = await bufferedResponse(options.url, controller.signal);
      decodeBzipToBuilder(compressed, builder);
    } else {
      throw new Error(`unsupported-client-compression:${compression || "unknown"}`);
    }
  } catch (error) {
    if (!isRecoverableTailError(error)) throw error;
    builder.markPartial(error?.message || error);
  }
  return builder.finish();
}

self.onmessage = async (event) => {
  const payload = event?.data || {};
  if (payload.type === "abort") {
    self.replayAbortController?.abort?.();
    return;
  }
  if (payload.type !== "start") return;
  try {
    const result = await buildReplay(payload);
    self.postMessage({ type: "complete", ...result });
  } catch (error) {
    if (error?.name === "AbortError") return;
    self.postMessage({ type: "error", error: error?.message || String(error) });
  }
};
