"use strict";

import {
  resolveLaneModeFields,
  resolveLaneModeState,
} from "../lane_mode.js";

export { resolveLaneModeState };

function finite(value) {
  if (value === null || value === undefined || value === "") return null;
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

function optionalBoolean(value) {
  return value == null ? null : Boolean(value);
}

function validCruiseKph(value) {
  const number = finite(value);
  return number != null && number > 0 && number < 250 ? number : null;
}

function level(value) {
  const number = finite(value);
  return number != null && number >= 0 && number <= 1 ? number : null;
}

function pressure(value) {
  const number = finite(value);
  return number != null && number >= 5 && number <= 100 ? number : null;
}

function trafficSignalState(value) {
  const state = finite(value);
  return state === 1 || state === 2 ? state : 0;
}

// Cluster driving-mode telltale: longitudinalPlan.myDrivingMode 1..4 (연비/안전/일반/고속).
// Anything outside 1..4 means "no badge".
function drivingModeState(value) {
  const mode = finite(value);
  return mode === 1 || mode === 2 || mode === 3 || mode === 4 ? mode : null;
}

function gearLabel(value) {
  const raw = String(value || "").trim().toLowerCase();
  const labels = {
    park: "P",
    reverse: "R",
    neutral: "N",
    drive: "D",
    sport: "S",
    low: "L",
  };
  if (!raw) return null;
  return labels[raw] || raw.slice(0, 2).toUpperCase();
}

function tpmsPayload(value) {
  if (!value || typeof value !== "object") return null;
  const payload = {
    fl: pressure(value.fl),
    fr: pressure(value.fr),
    rl: pressure(value.rl),
    rr: pressure(value.rr),
  };
  return Object.values(payload).some((pressure) => pressure != null) ? payload : null;
}

function cruiseOverridePayload(value) {
  if (!value || typeof value !== "object") return null;
  const kph = finite(value.kph);
  if (kph == null || kph <= 0) return null;
  return {
    kph,
    label: value.label == null ? "" : String(value.label),
    mode: finite(value.mode) ?? 0,
  };
}

export const CRUISE_OVERRIDE_HOLD_MS = 2500;

// Presentation-only grace for short service/sample gaps. The caller supplies
// the live monotonic clock or replay media clock, so pausing a replay does not
// expire the telltale and seeking never carries an old value into a new scene.
export function createCruiseOverrideHold(options = {}) {
  const configuredHoldMs = finite(options.holdMs);
  const holdMs = configuredHoldMs != null && configuredHoldMs >= 0
    ? configuredHoldMs
    : CRUISE_OVERRIDE_HOLD_MS;
  let held = null;
  let lastSeenMs = null;
  let lastClockMs = null;
  let clockKey = null;

  function reset() {
    held = null;
    lastSeenMs = null;
    lastClockMs = null;
    clockKey = null;
  }

  function update(value, context = {}) {
    const next = cruiseOverridePayload(value);
    const clockMs = finite(context.clockMs);
    const nextClockKey = String(context.clockKey || "live");
    if (context.active === false || clockMs == null) {
      reset();
      return context.active === false ? null : next;
    }

    if (clockKey !== nextClockKey || (lastClockMs != null && clockMs < lastClockMs)) {
      reset();
    }
    clockKey = nextClockKey;
    lastClockMs = clockMs;

    if (next) {
      if (next.mode === 2) {
        held = next;
        lastSeenMs = clockMs;
      } else {
        held = null;
        lastSeenMs = null;
      }
      return next;
    }

    const ageMs = lastSeenMs == null ? null : clockMs - lastSeenMs;
    if (held && ageMs != null && ageMs >= 0 && ageMs <= holdMs) return held;
    held = null;
    lastSeenMs = null;
    return null;
  }

  function status() {
    return Object.freeze({
      held: held ? { ...held } : null,
      lastSeenMs,
      lastClockMs,
      clockKey,
      holdMs,
    });
  }

  return Object.freeze({ update, reset, status });
}

// The classic realtime bridge rebuilds a compact payload before presenting it.
// Keep the cluster-only fields in one shared rule so live and replay cannot
// silently drop them at that boundary.
export function withVehicleHudFields(payload = {}, source = {}) {
  const laneMode = resolveLaneModeFields({
    requested: source.laneModeRequested,
    planned: source.laneModePlanned,
    controlled: source.activeLaneLine,
  });
  return {
    ...payload,
    evActive: source.evActive === true,
    activeLaneLine: laneMode.controlled,
    laneModeRequested: laneMode.requested,
    laneModePlanned: laneMode.planned,
    laneModePresentation: laneMode.presentation,
    cruiseOverride: cruiseOverridePayload(source.cruiseOverride),
    trafficState: trafficSignalState(source.trafficState ?? payload.trafficState),
    drivingMode: drivingModeState(source.drivingMode ?? payload.drivingMode),
  };
}

// Stable fragment for the classic HUD change detector. activeLaneLine is
// deliberately tri-state: unknown and explicitly disabled are different.
export function vehicleHudSignature(payload = {}) {
  const override = cruiseOverridePayload(payload.cruiseOverride);
  return [
    payload.evActive === true ? 1 : 0,
    payload.activeLaneLine == null ? "-" : (payload.activeLaneLine === true ? 1 : 0),
    payload.laneModeRequested == null ? "-" : (payload.laneModeRequested === true ? 1 : 0),
    payload.laneModePlanned == null ? "-" : (payload.laneModePlanned === true ? 1 : 0),
    payload.laneModePresentation ?? "-",
    override?.kph ?? "-",
    override?.label ?? "-",
    override?.mode ?? "-",
    drivingModeState(payload.drivingMode) ?? "-",
  ].join(":");
}

// Cluster parity: cluster_live.py deceleration_source_display_label.
const DECEL_SOURCE_LABELS = Object.freeze({
  cam: "cam:n", section: "section:n", bump: "bump:n", police: "police:n",
  waze: "waze:n", road: "road:n", atc: "turn:n", atc2: "turn:n",
  hda: "cam:v", route: "route:v", gas: "gas:v",
  vturn: "turn:c", model: "turn:c", turn: "turn:c",
});

function decelerationSourceLabel(source) {
  const normalized = String(source || "").trim().toLowerCase();
  if (!normalized) return "apply";
  if (normalized.endsWith(":n") || normalized.endsWith(":v") || normalized.endsWith(":c")) return normalized;
  return DECEL_SOURCE_LABELS[normalized] || normalized.slice(0, 8);
}

// Cluster parity: prefer the cluster-facing set speed, but treat zero/sentinel
// values as unavailable instead of letting nullish-coalescing pin the HUD to 0.
// The final cruiseState fallbacks also support non-compact/raw callers.
export function resolveCruiseKph(state = {}) {
  const carState = state.carState || {};
  const controlsState = state.controlsState || {};
  const cruiseState = carState.cruiseState || {};
  if (cruiseState.available === false) return null;
  const direct = [
    carState.vCruiseCluster,
    carState.vCruise,
    controlsState.vCruiseCluster,
    controlsState.vCruise,
  ];
  for (const value of direct) {
    const kph = validCruiseKph(value);
    if (kph != null) return kph;
  }

  for (const value of [cruiseState.speedCluster, cruiseState.speed]) {
    const speedMps = finite(value);
    if (speedMps != null && speedMps > 0.1 && speedMps < 70) return speedMps * 3.6;
  }
  return null;
}

// Cluster cruise_display_state is off when the current control state explicitly
// says disabled. With no control sample, a valid set speed remains a paused
// display and may still carry the cluster override telltale.
export function isCruiseDisplayVisible(state = {}, cruiseKph = resolveCruiseKph(state)) {
  if (validCruiseKph(cruiseKph) == null) return false;
  const selfdriveEnabled = optionalBoolean(state.selfdriveState?.enabled);
  if (selfdriveEnabled != null) return selfdriveEnabled;
  const controlsEnabled = optionalBoolean(state.controlsState?.enabled);
  if (controlsEnabled != null) return controlsEnabled;
  return state.carState?.cruiseState?.available !== false;
}

// Cruise set-speed override telltale. Mirrors cluster_live.py priority/thresholds:
//   mode 1 (eco, green)    = longitudinalPlan.cruiseTarget above the set speed
//   mode 2 (decel, orange) = carrotMan.desiredSpeed below the set speed, with source label
// Returns { kph, label, mode } in kph, or null. Shared by live and replay.
export function deriveCruiseOverride(state = {}) {
  const cruiseKph = resolveCruiseKph(state);
  if (!isCruiseDisplayVisible(state, cruiseKph)) return null;

  const cruiseTarget = finite(state.longitudinalPlan?.cruiseTarget);
  if (cruiseTarget != null && cruiseTarget > cruiseKph + 0.5) {
    return { kph: cruiseTarget, label: "eco", mode: 1 };
  }

  const desiredSpeed = finite(state.carrotMan?.desiredSpeed);
  if (desiredSpeed != null && desiredSpeed > 0 && desiredSpeed < 200 && desiredSpeed < cruiseKph) {
    return { kph: desiredSpeed, label: decelerationSourceLabel(state.carrotMan?.desiredSource), mode: 2 };
  }
  return null;
}

// The cluster follows longitudinalPlan while the comma HUD follows carrotMan.
// Compact/replay samples can update those services at different moments, and
// carrotMan commonly carries an explicit 0 that must not mask an active planner
// signal. Prefer an active cluster state, then fall back to an active HUD state.
export function resolveTrafficState(state = {}) {
  const plannerState = trafficSignalState(state.longitudinalPlan?.trafficState);
  if (plannerState !== 0) return plannerState;
  return trafficSignalState(state.carrotMan?.trafficState);
}

// Decoded compact and raw replay state intentionally converge here. UI widgets
// consume this stable payload and never need to know the cereal layout.
export function deriveVehicleHudPayload(state = {}) {
  const carState = state.carState || {};
  const controlsState = state.controlsState || {};
  const selfdriveState = state.selfdriveState || {};
  const carControl = state.carControl || {};
  const gear = gearLabel(carState.gearShifter);
  const rawGearStep = finite(carState.gearStep);
  const gearStep = gear === "D" && rawGearStep != null && rawGearStep > 0
    ? Math.round(rawGearStep)
    : null;

  // Keep user intent, planner selection and final control activation separate.
  // They update on different service clocks in both live and replay.
  const evActive = Boolean(carState.evModeValid) && Boolean(carState.evModeActive);
  const laneMode = resolveLaneModeState(state);
  const latActive = optionalBoolean(carControl.latActive);

  return {
    gear,
    gearStep,
    evActive,
    activeLaneLine: laneMode.controlled,
    laneModeRequested: laneMode.requested,
    laneModePlanned: laneMode.planned,
    laneModePresentation: laneMode.presentation,
    cruiseOverride: deriveCruiseOverride(state),
    trafficState: resolveTrafficState(state),
    drivingMode: drivingModeState(state.longitudinalPlan?.myDrivingMode),
    // The cluster wheel follows lateral actuation, not overall engagement.
    // Older recordings without carControl retain the legacy state fallback.
    lfaActive: latActive ?? Boolean(selfdriveState.enabled ?? controlsState.enabled),
    steeringAngleDeg: finite(carState.steeringAngleDeg ?? carControl.actuators?.steeringAngleDeg),
    aEgo: finite(carState.aEgo ?? carControl.actuators?.accel),
    steerOutput: finite(
      controlsState.lateralOutput
      ?? controlsState.torqueState?.lateralOutput
      ?? controlsState.lateralControlState?.torqueState?.output,
    ),
    leftBlinker: Boolean(carState.leftBlinker),
    rightBlinker: Boolean(carState.rightBlinker),
    fuelGauge: level(carState.fuelGauge),
    ureaGauge: level(carState.ureaGauge),
    tpms: tpmsPayload(carState.tpms),
  };
}

export const CarrotHudDataBridge = Object.freeze({
  deriveVehicleHudPayload,
  resolveLaneModeState,
  deriveCruiseOverride,
  createCruiseOverrideHold,
  resolveTrafficState,
  resolveCruiseKph,
  isCruiseDisplayVisible,
  withVehicleHudFields,
  vehicleHudSignature,
});
