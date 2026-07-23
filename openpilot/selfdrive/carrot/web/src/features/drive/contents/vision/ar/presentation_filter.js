/* Temporal presentation filter for an already-approved world anchor.
 *
 * The anchor store owns world identity and odometry propagation. This filter
 * only removes sub-frame lateral/elevation/orientation/scale noise before the
 * Three matrix is written. Forward distance stays unsmoothed so a sign never
 * lags behind the road while the vehicle approaches it.
 */

import { AR_RENDER } from "./tokens.js";
import {
  blendRoadFrames,
  faceFrameFields,
  faceFrameForAnchor,
  roadFrameFields,
  roadFrameForAnchor,
} from "./road_frame.js";

function finite(value, fallback = 0) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function halfLifeAlpha(dtMs, halfLifeMs) {
  const dt = Math.max(0, finite(dtMs));
  const halfLife = Math.max(1, finite(halfLifeMs, 1));
  return 1 - Math.exp(-Math.LN2 * dt / halfLife);
}

function snapshot(anchor, scale) {
  return Object.freeze({
    anchor: Object.freeze({ ...anchor }),
    scale: Math.max(0.001, finite(scale, 1)),
  });
}

export function createMarkerPresentationFilter(options = {}) {
  const limits = Object.freeze({
    ...AR_RENDER.presentation,
    ...(options.limits || {}),
  });
  let state = null;
  let lastAtMs = null;

  function reset(anchor = null, scale = 1, nowMs = null) {
    state = anchor ? snapshot(anchor, scale) : null;
    lastAtMs = Number.isFinite(Number(nowMs)) ? Number(nowMs) : null;
    return state;
  }

  function update(anchor, scale = 1, nowMs = 0) {
    if (!anchor) return reset();
    const now = finite(nowMs, 0);
    const dtMs = lastAtMs === null ? 0 : now - lastAtMs;
    const lateralJumpM = state
      ? Math.hypot(
        finite(anchor.y) - finite(state.anchor.y),
        finite(anchor.z) - finite(state.anchor.z),
      )
      : 0;
    if (
      !state
      || dtMs < 0
      || dtMs > limits.maxStepMs
      || lateralJumpM > limits.snapDistanceM
    ) return reset(anchor, scale, now);

    const positionAlpha = halfLifeAlpha(dtMs, limits.positionHalfLifeMs);
    const orientationAlpha = halfLifeAlpha(dtMs, limits.orientationHalfLifeMs);
    const scaleAlpha = halfLifeAlpha(dtMs, limits.scaleHalfLifeMs);
    const hasFaceFrame = Array.isArray(anchor.faceForward);
    const frame = blendRoadFrames(
      hasFaceFrame ? faceFrameForAnchor(state.anchor) : roadFrameForAnchor(state.anchor),
      hasFaceFrame ? faceFrameForAnchor(anchor) : roadFrameForAnchor(anchor),
      orientationAlpha,
    );
    const filteredAnchor = Object.freeze({
      ...anchor,
      // Forward motion is frame-linked odometry and must not trail the video.
      x: finite(anchor.x),
      y: finite(state.anchor.y) + (finite(anchor.y) - finite(state.anchor.y)) * positionAlpha,
      z: finite(state.anchor.z) + (finite(anchor.z) - finite(state.anchor.z)) * positionAlpha,
      ...(hasFaceFrame ? faceFrameFields(frame) : roadFrameFields(frame)),
    });
    const targetScale = Math.max(0.001, finite(scale, state.scale));
    const filteredScale = state.scale + (targetScale - state.scale) * scaleAlpha;
    state = snapshot(filteredAnchor, filteredScale);
    lastAtMs = now;
    return state;
  }

  return Object.freeze({ update, reset, status: () => state, limits });
}
