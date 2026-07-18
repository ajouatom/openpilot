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

  return {
    gear,
    gearStep,
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

export const CarrotHudDataBridge = Object.freeze({ deriveVehicleHudPayload });
