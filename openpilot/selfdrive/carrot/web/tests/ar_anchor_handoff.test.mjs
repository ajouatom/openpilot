import assert from "node:assert/strict";
import test from "node:test";

import {
  AR_ANCHOR_HANDOFF_POLICY,
  planWorldAnchorHandoff,
  sampleWorldAnchorHandoff,
} from "../src/features/drive/contents/vision/ar/anchor_handoff.js";
import {
  createContinuousAnchorStore,
  worldAnchorCreationGate,
} from "../src/features/drive/contents/vision/ar/anchor_store.js";
import { AR_COORDINATE_FRAME } from "../src/features/drive/contents/vision/ar/coordinate_frames.js";

function pose(position = [0, 0, 0], timestampNs = 1) {
  return Object.freeze({
    epoch: "route-a",
    initialized: true,
    timestampNs,
    position: Object.freeze(position),
    orientation: Object.freeze([1, 0, 0, 0]),
    worldCoordinateFrame: AR_COORDINATE_FRAME.LOCAL_WORLD_DEVICE,
    deviceCoordinateFrame: AR_COORDINATE_FRAME.DEVICE_FRD,
  });
}

function worldAnchor(position, roadForward = [1, 0, 0]) {
  return Object.freeze({
    coordinateFrame: AR_COORDINATE_FRAME.LOCAL_WORLD_DEVICE,
    epoch: "route-a",
    position: Object.freeze(position),
    roadForward: Object.freeze(roadForward),
    roadUp: Object.freeze([0, 0, -1]),
    createdAtTimestampNs: 1,
  });
}

function routeCandidate(distanceM, y = 0) {
  return {
    source: "guidanceCurrent",
    markerId: "guidance:primary|guidanceCurrent|turn_gate||12",
    lifecycleSlot: "guidance:primary",
    descriptor: { kind: "turn_gate", primary: `${distanceM}m` },
    distanceM,
    placedBy: "geo",
    anchor: {
      x: distanceM,
      y,
      z: 0,
      headingRad: 0.4,
      routeDerived: true,
      heightSource: "unknown-flat",
      heightConfidence: 0,
    },
  };
}

function navi(publishMonoTimeNanos) {
  return { sessionId: "route-a", generation: 1, publishMonoTimeNanos };
}

test("new geo anchors wait for a settled corrected world frame", () => {
  assert.equal(worldAnchorCreationGate(pose(), undefined, {
    trackingState: "initializing",
  }).reason, "pose-propagation-settling");

  assert.equal(worldAnchorCreationGate({
    ...pose(),
    geoCorrection: { referenceReady: false, accepted: 0 },
  }).reason, "geo-reference-unavailable");

  const correcting = {
    ...pose(),
    geoCorrection: {
      referenceReady: true,
      accepted: 10,
      lastPositionInnovationM: 10.656,
      lastYawInnovationRad: -0.36777,
    },
  };
  assert.deepEqual(worldAnchorCreationGate(correcting), {
    accepted: false,
    reason: "geo-yaw-settling",
    yawInnovationRad: -0.36777,
  });

  const settled = {
    ...correcting,
    geoCorrection: {
      ...correcting.geoCorrection,
      lastYawInnovationRad: -0.18585,
    },
  };
  assert.equal(worldAnchorCreationGate(settled).accepted, true);

  const store = createContinuousAnchorStore();
  const blocked = store.update({
    nowMs: 0,
    navi: navi(1),
    candidates: [routeCandidate(160)],
    modelPosition: { x: [0, 50, 100], y: [0, 0, 0], z: [0, 0, 0] },
    valid: true,
    precise: true,
    canHold: true,
    trackingState: "tracking",
    worldPose: correcting,
  });
  assert.equal(blocked.anchors, null);
  assert.equal(store.status(0).creationDeferredCount, 1);
  assert.equal(store.status(0).creationGateReason, "geo-yaw-settling");

  const created = store.update({
    nowMs: 50,
    navi: navi(1),
    candidates: [routeCandidate(160)],
    modelPosition: { x: [0, 50, 100], y: [0, 0, 0], z: [0, 0, 0] },
    valid: true,
    precise: true,
    canHold: true,
    trackingState: "tracking",
    worldPose: settled,
  });
  assert.equal(created.anchors.length, 1);
  assert.equal(store.status(50).creationDeferredCount, 0);
});

test("world handoff blends position and orientation with fixed endpoints", () => {
  const from = worldAnchor([100, 0, 0]);
  const to = worldAnchor([100, -2, -4], [Math.cos(0.4), -Math.sin(0.4), 0]);
  const decision = planWorldAnchorHandoff(from, to, 100);

  assert.equal(decision.accepted, true);
  assert.strictEqual(sampleWorldAnchorHandoff(decision.transition, 100), from);
  const midpoint = sampleWorldAnchorHandoff(
    decision.transition,
    100 + AR_ANCHOR_HANDOFF_POLICY.durationMs / 2,
  );
  assert.deepEqual(midpoint.position, [100, -1, -2]);
  assert.ok(midpoint.roadForward[0] < 1 && midpoint.roadForward[0] > Math.cos(0.4));
  assert.strictEqual(
    sampleWorldAnchorHandoff(decision.transition, 100 + AR_ANCHOR_HANDOFF_POLICY.durationMs),
    to,
  );
});

test("an implausible model correction is rejected instead of moving the route anchor", () => {
  const decision = planWorldAnchorHandoff(worldAnchor([100, 0, 0]), worldAnchor([100, 0, -30]), 0);
  assert.equal(decision.accepted, false);
  assert.equal(decision.reason, "position-innovation-rejected");
});

test("route cue hands off inside the model horizon without changing marker identity", () => {
  const store = createContinuousAnchorStore();
  const far = store.update({
    nowMs: 0,
    navi: navi(1),
    candidates: [routeCandidate(180)],
    modelPosition: { x: [0, 50, 100], y: [0, 0, 0], z: [0, 0, 0] },
    valid: true,
    precise: true,
    canHold: true,
    trackingState: "tracking",
    worldPose: pose(),
  });
  const markerId = far.anchors[0].markerId;
  assert.equal(far.anchors[0].sourceMode, "geo-only");

  const entered = store.update({
    nowMs: 100,
    navi: navi(2),
    candidates: [routeCandidate(100, 2)],
    modelPosition: { x: [0, 50, 100, 120], y: [0, 0, 0, 0], z: [0, 2, 4, 5] },
    valid: true,
    precise: true,
    canHold: true,
    trackingState: "tracking",
    worldPose: pose([80, 0, 0], 2),
  });
  assert.equal(entered.anchors[0].markerId, markerId);
  assert.equal(entered.anchors[0].anchor.y, 0, "handoff starts at the exact prior screen pose");
  assert.equal(entered.anchors[0].anchor.z, 0);
  assert.equal(entered.anchors[0].handoff.state, "blending");
  assert.equal(entered.anchors[0].handoff.progress, 0);
  assert.equal(store.status(100).handoffCount, 1);

  const middle = store.update({
    nowMs: 700,
    // A newer 2Hz Navi publish must repaint without restarting the blend.
    navi: navi(3),
    candidates: [routeCandidate(100, 2)],
    modelPosition: { x: [0, 50, 100, 120], y: [0, 0, 0, 0], z: [0, 2, 4, 5] },
    valid: true,
    precise: true,
    canHold: true,
    trackingState: "tracking",
    worldPose: pose([80, 0, 0], 3),
  });
  assert.equal(middle.anchors[0].markerId, markerId);
  assert.ok(Math.abs(middle.anchors[0].anchor.y - 0.65) < 1e-9);
  assert.ok(Math.abs(middle.anchors[0].anchor.z - 2) < 1e-9);
  assert.equal(middle.anchors[0].handoff.progress, 0.5);

  const completed = store.update({
    nowMs: 1300,
    navi: navi(3),
    candidates: [routeCandidate(100, 2)],
    modelPosition: { x: [0, 50, 100, 120], y: [0, 0, 0, 0], z: [0, 2, 4, 5] },
    valid: true,
    precise: true,
    canHold: true,
    trackingState: "tracking",
    worldPose: pose([80, 0, 0], 4),
  });
  assert.equal(completed.anchors[0].markerId, markerId);
  assert.ok(Math.abs(completed.anchors[0].anchor.y - 1.3) < 1e-9);
  assert.ok(Math.abs(completed.anchors[0].anchor.z - 4) < 1e-9);
  assert.equal(completed.anchors[0].sourceMode, "geo+path");
  assert.equal(completed.anchors[0].handoff.state, "complete");
  assert.equal(store.status(1300).handoffCount, 0);
});

test("a real guidance point receives road height without route lateral snapping", () => {
  const store = createContinuousAnchorStore();
  const actualPoint = (distanceM) => {
    const candidate = routeCandidate(distanceM, 2);
    delete candidate.anchor.routeDerived;
    return candidate;
  };
  const far = store.update({
    nowMs: 0,
    navi: navi(10),
    candidates: [actualPoint(180)],
    modelPosition: { x: [0, 50, 100], y: [0, 0, 0], z: [0, 0, 0] },
    valid: true,
    precise: true,
    canHold: true,
    trackingState: "tracking",
    worldPose: pose(),
  });
  assert.equal(far.anchors[0].sourceMode, "geo-only");

  const entered = store.update({
    nowMs: 100,
    navi: navi(11),
    candidates: [actualPoint(100)],
    modelPosition: { x: [0, 50, 100, 120], y: [0, 0, 0, 0], z: [0, 2, 4, 5] },
    valid: true,
    precise: true,
    canHold: true,
    trackingState: "tracking",
    worldPose: pose([80, 42, 0], 2),
  });
  assert.equal(entered.anchors[0].markerId, far.anchors[0].markerId);
  assert.equal(entered.anchors[0].handoff.state, "blending");
  assert.equal(entered.anchors[0].anchor.y, 44);
  assert.equal(entered.anchors[0].anchor.z, 0);
  assert.ok(Math.abs(entered.anchors[0].handoff.lateralInnovationM + 42) < 1e-9);
  assert.equal(entered.anchors[0].handoff.positionInnovationM, 4);

  const completed = store.update({
    nowMs: 1300,
    navi: navi(11),
    candidates: [actualPoint(100)],
    modelPosition: { x: [0, 50, 100, 120], y: [0, 0, 0, 0], z: [0, 2, 4, 5] },
    valid: true,
    precise: true,
    canHold: true,
    trackingState: "tracking",
    worldPose: pose([80, 42, 0], 3),
  });
  assert.equal(completed.anchors[0].anchor.y, 44, "actual geo world point must not snap to the model lane");
  assert.equal(completed.anchors[0].anchor.z, 4);
  assert.equal(completed.anchors[0].anchor.heightSource, "model-path-bounded");
  assert.equal(completed.anchors[0].sourceMode, "geo+path");
});
