import assert from "node:assert/strict";
import test from "node:test";

import {
  AR_TRACKING_STATE,
  createArTrackingState,
} from "../src/features/drive/contents/vision/ar/tracking_state.js";

function good(nowMs, sampleId, overrides = {}) {
  return {
    nowMs,
    sampleId,
    presentedTimestampNs: 10_000_000_000 + nowMs * 1e6,
    spatialClockReady: true,
    sync: { canDrawPrecise: true, canHoldAnchor: true, reasons: [] },
    odometry: {
      trans: [15, 0, 0],
      rot: [0, 0, 0.01],
      transStd: [0.1, 0.02, 0.02],
      rotStd: [0.002, 0.002, 0.002],
    },
    egoSpeedMps: 15,
    anchorDistanceM: 200,
    ...overrides,
  };
}

// 공간 클럭 락은 world 고정/전파의 조건이지 앵커 생성의 조건이 아니다.
// 클럭이 아직 매핑되지 않아도 precise sync면 표지는 만들어 놓는다.
test("tracking locks only from a mapped frame, but creation follows precise sync", () => {
  const tracker = createArTrackingState();
  const waiting = tracker.update(good(0, null, {
    spatialClockReady: false,
    presentedTimestampNs: null,
    odometry: null,
  }));
  assert.equal(waiting.state, AR_TRACKING_STATE.INITIALIZING);
  assert.equal(waiting.canCreateAnchor, true);
  assert.equal(waiting.canPropagateAnchor, false);

  const tracking = tracker.update(good(50, 1));
  assert.equal(tracking.state, AR_TRACKING_STATE.TRACKING);
  assert.equal(tracking.canCreateAnchor, true);
  assert.equal(tracking.alpha, 1);
});

test("the same 20Hz video frame stays tracking across 30Hz presentation ticks", () => {
  const tracker = createArTrackingState();
  tracker.update(good(0, 1));
  const repeated = tracker.update(good(66, 1));

  assert.equal(repeated.state, AR_TRACKING_STATE.TRACKING);
  assert.equal(repeated.predictionAgeMs, 66);
  assert.equal(repeated.transition, null);
});

test("a 400ms input gap coasts without deleting the anchor lifecycle", () => {
  const tracker = createArTrackingState();
  tracker.update(good(0, 1));
  const coast = tracker.update(good(400, 2, {
    spatialClockReady: false,
    presentedTimestampNs: null,
    sync: { canDrawPrecise: false, canHoldAnchor: false, reasons: ["frame gap"] },
    odometry: null,
  }));

  assert.equal(coast.state, AR_TRACKING_STATE.COASTING);
  assert.equal(coast.retainAnchor, true);
  assert.equal(coast.canCreateAnchor, false);
  assert.ok(coast.alpha > 0.7);
});

test("uncertainty and age advance through degraded to lost with a recorded reason", () => {
  const tracker = createArTrackingState();
  tracker.update(good(0, 1));
  const unavailable = {
    spatialClockReady: false,
    presentedTimestampNs: null,
    sync: { canDrawPrecise: false, canHoldAnchor: false, reasons: ["model stale"] },
    odometry: null,
  };
  const degraded = tracker.update(good(2000, 2, unavailable));
  const lost = tracker.update(good(5000, 3, unavailable));

  assert.equal(degraded.state, AR_TRACKING_STATE.DEGRADED);
  assert.equal(degraded.retainAnchor, true);
  assert.equal(lost.state, AR_TRACKING_STATE.LOST);
  assert.equal(lost.retainAnchor, false);
  assert.equal(lost.alpha, 0);
  assert.match(lost.reasons.join(" | "), /model stale/);
  assert.match(lost.reasons.join(" | "), /tracking budget exceeded/);
});

test("a healthy fix recovers from lost without reusing its uncertainty", () => {
  const tracker = createArTrackingState();
  tracker.update(good(0, 1));
  tracker.update(good(10, 1, { discontinuity: true, discontinuityReason: "replay seek" }));
  // Segment-local frame IDs may repeat after seek; discontinuity must clear the
  // previous sample identity so that the first new fix can recover tracking.
  const recovered = tracker.update(good(60, 1));

  assert.equal(recovered.state, AR_TRACKING_STATE.TRACKING);
  assert.equal(recovered.recovered, true);
  assert.equal(recovered.transition, "lost->tracking");
  assert.equal(recovered.uncertainty.lateralM, 0);
});
