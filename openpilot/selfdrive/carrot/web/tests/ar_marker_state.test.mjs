import assert from "node:assert/strict";
import test from "node:test";

import {
  AR_MARKER_VISIBILITY_STATE,
  markerStateReason,
  markerViewportState,
} from "../src/features/drive/contents/vision/ar/marker_state.js";

test("marker viewport state distinguishes partial visibility and offscreen bounds", () => {
  const canvas = { width: 1200, height: 700 };
  assert.equal(
    markerViewportState({ left: 100, right: 300, top: 100, bottom: 250 }, canvas),
    AR_MARKER_VISIBILITY_STATE.ACTIVE_CANDIDATE,
  );
  assert.equal(
    markerViewportState({ left: -80, right: 20, top: 100, bottom: 250 }, canvas),
    AR_MARKER_VISIBILITY_STATE.ACTIVE_CANDIDATE,
  );
  assert.equal(
    markerViewportState(
      { left: 1225, right: 1400, top: 100, bottom: 250 }, canvas, { paddingPx: 20 },
    ),
    AR_MARKER_VISIBILITY_STATE.ACTIVE_OFFSCREEN,
  );
  assert.equal(
    markerViewportState({ left: NaN, right: 20, top: 0, bottom: 20 }, canvas),
    AR_MARKER_VISIBILITY_STATE.PROJECTION_UNAVAILABLE,
  );
  assert.equal(markerStateReason(AR_MARKER_VISIBILITY_STATE.NEAR_PLANE), "inside near-plane passing zone");
});
