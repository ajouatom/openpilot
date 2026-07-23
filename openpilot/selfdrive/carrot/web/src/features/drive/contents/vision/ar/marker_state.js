/** Explicit renderer visibility states for one semantic AR marker. */
export const AR_MARKER_VISIBILITY_STATE = Object.freeze({
  ACTIVE_CANDIDATE: "active-candidate",
  ACTIVE_VISIBLE: "active-visible",
  ACTIVE_OFFSCREEN: "active-offscreen",
  ACTIVE_SUPPRESSED: "active-suppressed",
  PASSING: "passing",
  BEHIND_CAMERA: "behind-camera",
  NEAR_PLANE: "near-plane",
  PLAN_REJECTED: "plan-rejected",
  PROJECTION_UNAVAILABLE: "projection-unavailable",
  INACTIVE_FADING: "inactive-fading",
});

const REASONS = Object.freeze({
  [AR_MARKER_VISIBILITY_STATE.ACTIVE_CANDIDATE]: "active marker candidate",
  [AR_MARKER_VISIBILITY_STATE.ACTIVE_VISIBLE]: "visible",
  [AR_MARKER_VISIBILITY_STATE.ACTIVE_OFFSCREEN]: "outside camera viewport",
  [AR_MARKER_VISIBILITY_STATE.ACTIVE_SUPPRESSED]: "overlap selection suppressed",
  [AR_MARKER_VISIBILITY_STATE.PASSING]: "marker passing camera",
  [AR_MARKER_VISIBILITY_STATE.BEHIND_CAMERA]: "behind camera",
  [AR_MARKER_VISIBILITY_STATE.NEAR_PLANE]: "inside near-plane passing zone",
  [AR_MARKER_VISIBILITY_STATE.PLAN_REJECTED]: "visibility plan rejected",
  [AR_MARKER_VISIBILITY_STATE.PROJECTION_UNAVAILABLE]: "projection unavailable",
  [AR_MARKER_VISIBILITY_STATE.INACTIVE_FADING]: "inactive marker fading",
});

export function markerStateReason(state) {
  return REASONS[state] || "unknown marker state";
}

function finite(value) {
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

/** Classify screen bounds before overlap/max-count selection. */
export function markerViewportState(bounds, canvas, options = {}) {
  const left = finite(bounds?.left);
  const right = finite(bounds?.right);
  const top = finite(bounds?.top);
  const bottom = finite(bounds?.bottom);
  const width = finite(canvas?.width);
  const height = finite(canvas?.height);
  if ([left, right, top, bottom, width, height].some((value) => value === null)) {
    return AR_MARKER_VISIBILITY_STATE.PROJECTION_UNAVAILABLE;
  }
  const paddingPx = Math.max(0, finite(options.paddingPx) ?? 0);
  if (
    right < -paddingPx || left > width + paddingPx
    || bottom < -paddingPx || top > height + paddingPx
  ) return AR_MARKER_VISIBILITY_STATE.ACTIVE_OFFSCREEN;
  return AR_MARKER_VISIBILITY_STATE.ACTIVE_CANDIDATE;
}
