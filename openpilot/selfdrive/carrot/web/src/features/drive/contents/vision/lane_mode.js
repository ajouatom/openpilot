"use strict";

export const LANE_MODE_PRESENTATION = Object.freeze({
  UNKNOWN: "unknown",
  LANELESS: "laneless",
  ARMED: "armed",
  ACTIVE: "active",
});

function optionalBoolean(value) {
  return value == null ? null : Boolean(value);
}

function optionalNumber(value) {
  if (value == null || value === "") return null;
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

export function resolveLaneModeFields(fields = {}) {
  const requested = optionalBoolean(fields.requested);
  const planned = optionalBoolean(fields.planned);
  const controlled = optionalBoolean(fields.controlled);
  const pathActive = planned ?? controlled ?? false;
  let presentation = LANE_MODE_PRESENTATION.UNKNOWN;

  // A direct request to leave lane mode must be acknowledged immediately,
  // while the slower planner/control services may still carry an older sample.
  if (requested === false) {
    presentation = LANE_MODE_PRESENTATION.LANELESS;
  } else if (controlled === true) {
    presentation = LANE_MODE_PRESENTATION.ACTIVE;
  } else if (controlled === false) {
    presentation = requested === true || planned === true
      ? LANE_MODE_PRESENTATION.ARMED
      : LANE_MODE_PRESENTATION.LANELESS;
  } else if (planned === true) {
    presentation = LANE_MODE_PRESENTATION.ACTIVE;
  } else if (requested === true) {
    presentation = LANE_MODE_PRESENTATION.ARMED;
  } else if (planned === false) {
    presentation = LANE_MODE_PRESENTATION.LANELESS;
  }

  return Object.freeze({
    requested,
    planned,
    controlled,
    pathActive: pathActive === true,
    presentation,
  });
}

export function resolveLaneModeState(state = {}) {
  const useLaneLineSpeed = optionalNumber(state.carState?.useLaneLineSpeed);
  return resolveLaneModeFields({
    requested: useLaneLineSpeed == null ? null : useLaneLineSpeed > 0,
    planned: state.lateralPlan?.useLaneLines,
    controlled: state.controlsState?.activeLaneLine,
  });
}

export const DriveVisionLaneMode = Object.freeze({
  PRESENTATION: LANE_MODE_PRESENTATION,
  resolveFields: resolveLaneModeFields,
  resolveState: resolveLaneModeState,
});
