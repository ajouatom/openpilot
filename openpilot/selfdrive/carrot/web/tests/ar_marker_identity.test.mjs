import assert from "node:assert/strict";
import test from "node:test";

import { collectSigns } from "../src/features/drive/contents/vision/ar/compose.js";
import {
  AR_MARKER_IDENTITY_LIMITS,
  createMarkerIdentityTracker,
} from "../src/features/drive/contents/vision/ar/marker_identity.js";

function guidance({
  distanceM,
  turnType = 12,
  roadName = "test road",
  latitude = 37.5,
  longitude = 127,
  pointValid = true,
  generation = 0,
  sequence = 0,
} = {}) {
  return {
    present: true,
    distanceM,
    turnType,
    roadName,
    pointValid,
    latitude,
    longitude,
    generation,
    sequence,
  };
}

function navi(sessionId, current, next = null) {
  return {
    sessionId,
    navigationStatus: { guidanceActive: true, offRoute: false },
    guidanceCurrent: current,
    guidanceNext: next,
  };
}

function turnMarker(signs, source) {
  return signs.find((item) => item.source === source && item.descriptor.kind === "turn_gate");
}

test("guidance updates repaint one occurrence across distance, text, phase, and GPS jitter", () => {
  const tracker = createMarkerIdentityTracker();
  const frame = (distanceM, roadName, latitude, generation) => turnMarker(collectSigns({
    navi: navi("route-a", guidance({
      distanceM,
      roadName,
      latitude,
      longitude: 127.00002,
      generation,
      sequence: generation * 10,
    })),
    naviUsable: true,
    egoSpeedMps: 20,
  }, { identityTracker: tracker }), "guidanceCurrent");

  const far = frame(300, "first label", 37.5, 1);
  const approach = frame(150, "refined label", 37.50003, 2);
  const near = frame(70, "turn soon", 37.49998, 3);
  const commit = frame(15, "final text", 37.50002, 4);

  assert.equal(approach.markerId, far.markerId);
  assert.equal(near.markerId, far.markerId);
  assert.equal(commit.markerId, far.markerId);
  // One turn gate plus its separately rendered commit arrow.
  assert.equal(tracker.status().occurrenceCount, 2);
  assert.equal(tracker.status().tracked, 2);
  assert.notEqual(far.descriptor.phase, commit.descriptor.phase);
});

test("guidanceNext promotion keeps its occurrence while lifecycle presentation changes", () => {
  const tracker = createMarkerIdentityTracker();
  const pointA = { latitude: 37.5, longitude: 127 };
  const pointB = { latitude: 37.501, longitude: 127.001 };
  const pointC = { latitude: 37.502, longitude: 127.002 };
  const item = (point, distanceM, turnType) => guidance({ ...point, distanceM, turnType });

  const first = collectSigns({
    navi: navi("route-a", item(pointA, 50, 12), item(pointB, 300, 13)),
    naviUsable: true,
  }, { identityTracker: tracker });
  const promoted = collectSigns({
    navi: navi(
      "route-a",
      item({ latitude: pointB.latitude + 0.00003, longitude: pointB.longitude }, 270, 13),
      item(pointC, 290, 12),
    ),
    naviUsable: true,
  }, { identityTracker: tracker });

  const firstCurrent = turnMarker(first, "guidanceCurrent");
  const firstNext = turnMarker(first, "guidanceNext");
  const promotedCurrent = turnMarker(promoted, "guidanceCurrent");
  const promotedNext = turnMarker(promoted, "guidanceNext");

  assert.equal(promotedCurrent.markerId, firstNext.markerId);
  assert.notEqual(promotedCurrent.lifecycleSlot, firstNext.lifecycleSlot);
  assert.notEqual(firstCurrent.markerId, promotedCurrent.markerId);
  assert.notEqual(promotedNext.markerId, promotedCurrent.markerId);
});

test("a same-looking maneuver beyond point tolerance becomes a new occurrence", () => {
  const tracker = createMarkerIdentityTracker();
  const at = (latitude) => turnMarker(collectSigns({
    navi: navi("route-a", guidance({ distanceM: 100, latitude })),
    naviUsable: true,
  }, { identityTracker: tracker }), "guidanceCurrent");

  const first = at(37.5);
  const fartherThanTolerance = at(37.5002);

  assert.notEqual(first.markerId, fartherThanTolerance.markerId);
  assert.ok(0.0002 * 111_320 > AR_MARKER_IDENTITY_LIMITS.pointToleranceM);
});

test("coordinate-less markers survive approach but not a large forward distance reset", () => {
  const tracker = createMarkerIdentityTracker();
  const assign = (distanceM) => tracker.assign([{
    source: "sdi",
    kind: "caution_sign",
    sdiFamily: "camera",
    distanceM,
  }], { sessionId: "route-a" })[0];

  const far = assign(180);
  const near = assign(120);
  const reset = assign(190);

  assert.equal(near, far);
  assert.notEqual(reset, far);
});

test("route session is the hard identity boundary", () => {
  const tracker = createMarkerIdentityTracker();
  const marker = {
    source: "guidanceCurrent",
    kind: "turn_gate",
    turnCode: 12,
    distanceM: 100,
  };

  const first = tracker.assign([marker], { sessionId: "route-a" })[0];
  const reroute = tracker.assign([marker], { sessionId: "route-b" })[0];

  assert.notEqual(reroute, first);
});

test("provided durable event ID has priority over mutable navigation fields", () => {
  const tracker = createMarkerIdentityTracker();
  const assign = (distanceM, label, generation) => tracker.assign([{
    source: "guidanceCurrent",
    kind: "turn_gate",
    stableEventId: "maneuver-42",
    distanceM,
    label,
    generation,
    sequence: generation * 10,
  }], { sessionId: "route-a" })[0];

  assert.equal(assign(280, "old", 1), assign(40, "new", 9));
});
