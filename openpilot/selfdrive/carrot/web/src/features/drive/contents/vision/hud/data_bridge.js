"use strict";

function finite(value) {
  if (value === null || value === undefined || value === "") return null;
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

function level(value) {
  const number = finite(value);
  return number != null && number >= 0 && number <= 1 ? number : null;
}

function pressure(value) {
  const number = finite(value);
  return number != null && number >= 5 && number <= 100 ? number : null;
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

// Cruise set-speed override telltale. Mirrors cluster_live.py priority/thresholds:
//   mode 1 (eco, green)    = longitudinalPlan.cruiseTarget above the set speed
//   mode 2 (decel, orange) = carrotMan.desiredSpeed below the set speed, with source label
// Returns { kph, label, mode } in kph, or null. Shared by live and replay.
export function deriveCruiseOverride(state = {}) {
  const carState = state.carState || {};
  const controlsState = state.controlsState || {};
  const cruiseKph = finite(carState.vCruiseCluster ?? controlsState.vCruiseCluster);
  // Cruise off/paused shows 0 or a large sentinel; only override an engaged set speed.
  if (cruiseKph == null || cruiseKph <= 0 || cruiseKph >= 250) return null;

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

  // Cluster parity: EV telltale (carState.evModeValid & evModeActive) and the
  // LFA lane-line lateral mode (controlsState.activeLaneLine → green lane wings).
  const evActive = Boolean(carState.evModeValid) && Boolean(carState.evModeActive);
  const activeLaneLine = controlsState.activeLaneLine == null
    ? null
    : Boolean(controlsState.activeLaneLine);

  return {
    gear,
    gearStep,
    evActive,
    activeLaneLine,
    cruiseOverride: deriveCruiseOverride(state),
    lfaActive: Boolean(selfdriveState.enabled ?? controlsState.enabled ?? carControl.latActive),
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

export const CarrotHudDataBridge = Object.freeze({ deriveVehicleHudPayload, deriveCruiseOverride });
