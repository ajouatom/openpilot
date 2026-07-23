import assert from "node:assert/strict";
import test from "node:test";

import { createContinuousAnchorStore } from "../src/features/drive/contents/vision/ar/anchor_store.js";
import { routeDistanceAnchor } from "../src/features/drive/contents/vision/ar/geo.js";
import { pointOnPath } from "../src/features/drive/contents/vision/ar/projection.js";
import {
  AR_ROAD_HEIGHT_SOURCE,
  AR_ROAD_OBSERVATION_LIMITS,
  limitedRoadLateralCorrection,
} from "../src/features/drive/contents/vision/ar/road_observation.js";

test("model road height removes origin bias and bounds a horizon grade spike", () => {
  const anchor = pointOnPath({
    x: [0, 10, 20],
    y: [0, 0, 0],
    z: [7, 9, 109],
  }, 15);

  assert.ok(anchor);
  assert.equal(anchor.heightSource, AR_ROAD_HEIGHT_SOURCE.MODEL_PATH);
  assert.equal(anchor.heightLimited, true);
  assert.ok(Math.abs(anchor.z - 3.25) < 1e-12);
  assert.ok(Math.abs(anchor.roadGrade - AR_ROAD_OBSERVATION_LIMITS.maxAbsGrade) < 1e-12);
  assert.ok(anchor.roadForward[2] > 0);
  assert.ok(anchor.roadForward[2] < 0.25);
});

test("missing model z remains explicit unknown-flat without corrupting x/y", () => {
  const anchor = pointOnPath({
    x: [0, 20, 40],
    y: [0, 2, 4],
    z: [0, null, 2],
  }, 30);

  assert.ok(anchor);
  assert.equal(anchor.x, 30);
  assert.equal(anchor.y, 3);
  assert.equal(anchor.z, 0);
  assert.equal(anchor.heightSource, AR_ROAD_HEIGHT_SOURCE.UNKNOWN_FLAT);
  assert.equal(anchor.heightConfidence, 0);
  assert.equal(anchor.roadForward[2], 0);
});

test("far model elevation stops at the bounded near-field horizon", () => {
  const anchor = pointOnPath({
    x: [0, 100, 200],
    y: [0, 0, 0],
    z: [0, 20, 60],
  }, 150);

  assert.ok(anchor);
  assert.equal(anchor.z, 25);
  assert.equal(anchor.heightConfidence, 0);
  assert.equal(anchor.heightLimited, true);
  assert.equal(anchor.roadGrade, 0);
  assert.equal(anchor.roadForward[2], 0);
});

test("non-monotonic model geometry is rejected instead of creating a reversed tangent", () => {
  assert.equal(pointOnPath({
    x: [0, 20, 10, 40],
    y: [0, 0, 0, 0],
    z: [0, 0, 0, 0],
  }, 30), null);
});

test("route lateral correction is small and rejects a different branch", () => {
  const near = limitedRoadLateralCorrection(1, 0, { enabled: true });
  assert.equal(near.accepted, true);
  assert.ok(Math.abs(near.y - 0.65) < 1e-12);
  assert.ok(Math.abs(near.correctionM + 0.35) < 1e-12);

  const branch = limitedRoadLateralCorrection(10, 0, { enabled: true });
  assert.equal(branch.accepted, false);
  assert.equal(branch.y, 10);
  assert.equal(branch.correctionM, 0);
});

test("route anchor keeps route provenance while receiving bounded road height", () => {
  const store = createContinuousAnchorStore();
  const result = store.update({
    nowMs: 0,
    valid: true,
    precise: true,
    canHold: true,
    navi: { sessionId: "road-height", generation: 1, publishMonoTimeNanos: 1 },
    candidates: [{
      source: "guidanceCurrent",
      markerId: "route-height",
      descriptor: { kind: "turn_gate", primary: "40m" },
      distanceM: 40,
      placedBy: "geo",
      anchor: {
        x: 40,
        y: 1,
        z: 0,
        headingRad: 0.4,
        routeDerived: true,
        routeStartIndex: 2,
      },
    }],
    modelPosition: {
      x: [0, 20, 40, 60],
      y: [0, 0, 0, 0],
      z: [4, 6, 8, 10],
    },
  });
  const anchor = result.anchors[0].anchor;

  assert.equal(anchor.routeDerived, true);
  assert.equal(anchor.routeStartIndex, 2);
  assert.equal(anchor.heightSource, AR_ROAD_HEIGHT_SOURCE.MODEL_PATH);
  assert.equal(anchor.z, 4);
  assert.ok(Math.abs(anchor.y - 0.65) < 1e-12);
  assert.ok(Math.abs(anchor.headingRad - 0.4) < 1e-12);
});

test("route-only elevation is explicitly unknown rather than GNSS altitude", () => {
  const latitude = 37.5;
  const longitude = 127;
  const east = longitude + 100 / (111320 * Math.cos(latitude * Math.PI / 180));
  const anchor = routeDistanceAnchor(
    { present: true, latitude, longitude, headingDeg: 90 },
    [{ latitude, longitude }, { latitude, longitude: east }],
    40,
  );

  assert.ok(anchor);
  assert.equal(anchor.z, 0);
  assert.equal(anchor.heightSource, AR_ROAD_HEIGHT_SOURCE.UNKNOWN_FLAT);
  assert.equal(anchor.heightConfidence, 0);
});
