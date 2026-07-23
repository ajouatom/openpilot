/* Continuous route -> model-road world-anchor handoff.
 *
 * A far route cue is already a fixed world object. When the event enters the
 * model horizon, replacing that object in one frame creates the exact screen
 * jump that a world anchor is meant to avoid. This module validates the new
 * observation and blends position plus road orientation in world space.
 */

import { blendRoadFrames } from "./road_frame.js";

export const AR_ANCHOR_HANDOFF_POLICY = Object.freeze({
  durationMs: 1200,
  maxPositionInnovationM: 10,
  maxOrientationInnovationRad: Math.PI / 3,
});

function finite(value, fallback = null) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function vector(value) {
  if (!Array.isArray(value) || value.length < 3) return null;
  const out = value.slice(0, 3).map(Number);
  return out.every(Number.isFinite) ? out : null;
}

function normalized(value) {
  const out = vector(value);
  const length = out ? Math.hypot(...out) : 0;
  return length > 1e-9 ? out.map((component) => component / length) : null;
}

function angleBetween(aValue, bValue) {
  const a = normalized(aValue);
  const b = normalized(bValue);
  if (!a || !b) return Infinity;
  const cosine = clamp(a.reduce((sum, value, index) => sum + value * b[index], 0), -1, 1);
  return Math.acos(cosine);
}

function positionDistance(aValue, bValue) {
  const a = vector(aValue);
  const b = vector(bValue);
  return a && b ? Math.hypot(...a.map((value, index) => b[index] - value)) : Infinity;
}

function frozenVector(value) {
  return Object.freeze([...value]);
}

export function handoffProgress(handoff, nowMs) {
  if (!handoff) return 1;
  const durationMs = Math.max(1, finite(handoff.durationMs, AR_ANCHOR_HANDOFF_POLICY.durationMs));
  return clamp((finite(nowMs, handoff.startedAtMs) - handoff.startedAtMs) / durationMs, 0, 1);
}

/** Validate a same-event correction before it is allowed to move the object. */
export function planWorldAnchorHandoff(from, to, nowMs, options = {}) {
  const policy = Object.freeze({ ...AR_ANCHOR_HANDOFF_POLICY, ...(options.policy || {}) });
  const positionInnovationM = positionDistance(from?.position, to?.position);
  const orientationInnovationRad = angleBetween(from?.roadForward, to?.roadForward);
  const sameFrame = from?.coordinateFrame && from.coordinateFrame === to?.coordinateFrame;
  const sameEpoch = from?.epoch !== undefined && String(from.epoch) === String(to?.epoch);
  let reason = "";
  if (!sameFrame) reason = "coordinate-frame-mismatch";
  else if (!sameEpoch) reason = "world-epoch-mismatch";
  else if (positionInnovationM > policy.maxPositionInnovationM) reason = "position-innovation-rejected";
  else if (orientationInnovationRad > policy.maxOrientationInnovationRad) reason = "orientation-innovation-rejected";

  const metrics = Object.freeze({
    positionInnovationM,
    orientationInnovationRad,
    lateralInnovationM: finite(options.lateralInnovationM, 0),
    heightInnovationM: finite(options.heightInnovationM, 0),
  });
  if (reason) return Object.freeze({ accepted: false, reason, ...metrics });

  return Object.freeze({
    accepted: true,
    reason: "",
    ...metrics,
    transition: Object.freeze({
      from,
      to,
      startedAtMs: finite(nowMs, 0),
      durationMs: Math.max(1, finite(policy.durationMs, AR_ANCHOR_HANDOFF_POLICY.durationMs)),
      ...metrics,
    }),
  });
}

/** Sample with smoothstep so neither endpoint acquires an instantaneous velocity. */
export function sampleWorldAnchorHandoff(handoff, nowMs) {
  if (!handoff?.from || !handoff?.to) return null;
  const progress = handoffProgress(handoff, nowMs);
  if (progress <= 0) return handoff.from;
  if (progress >= 1) return handoff.to;
  const alpha = progress * progress * (3 - 2 * progress);
  const mix = (aValue, bValue) => {
    const a = vector(aValue);
    const b = vector(bValue);
    return a && b ? a.map((value, index) => value + (b[index] - value) * alpha) : null;
  };
  const road = blendRoadFrames(
    { forward: handoff.from.roadForward, up: handoff.from.roadUp },
    { forward: handoff.to.roadForward, up: handoff.to.roadUp },
    alpha,
  );
  const hasFace = handoff.from.faceForward && handoff.from.faceUp
    && handoff.to.faceForward && handoff.to.faceUp;
  const face = hasFace
    ? blendRoadFrames(
        { forward: handoff.from.faceForward, up: handoff.from.faceUp },
        { forward: handoff.to.faceForward, up: handoff.to.faceUp },
        alpha,
      )
    : null;

  return Object.freeze({
    coordinateFrame: handoff.from.coordinateFrame,
    epoch: handoff.from.epoch,
    position: frozenVector(mix(handoff.from.position, handoff.to.position)),
    roadForward: frozenVector(road.forward),
    roadUp: frozenVector(road.up),
    ...(face ? {
      faceForward: frozenVector(face.forward),
      faceUp: frozenVector(face.up),
    } : {}),
    createdAtTimestampNs: finite(handoff.to.createdAtTimestampNs, handoff.from.createdAtTimestampNs),
  });
}

export function publicHandoffState(handoff, nowMs, state = "blending") {
  if (!handoff) return null;
  return Object.freeze({
    state,
    progress: state === "complete" ? 1 : handoffProgress(handoff, nowMs),
    durationMs: handoff.durationMs,
    positionInnovationM: handoff.positionInnovationM,
    orientationInnovationRad: handoff.orientationInnovationRad,
    lateralInnovationM: handoff.lateralInnovationM,
    heightInnovationM: handoff.heightInnovationM,
  });
}
