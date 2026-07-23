import assert from "node:assert/strict";
import test from "node:test";

import { AR_COORDINATE_FRAME } from "../src/features/drive/contents/vision/ar/coordinate_frames.js";
import { createContinuousAnchorStore } from "../src/features/drive/contents/vision/ar/anchor_store.js";
import {
  createImmutableWorldAnchor,
  projectWorldAnchorToRoute,
} from "../src/features/drive/contents/vision/ar/world_anchor.js";

function pose({ epoch = "route-a", position = [0, 0, 0], orientation = [1, 0, 0, 0], timestampNs = 1 } = {}) {
  return Object.freeze({
    epoch,
    initialized: true,
    timestampNs,
    position: Object.freeze(position),
    orientation: Object.freeze(orientation),
    worldCoordinateFrame: AR_COORDINATE_FRAME.LOCAL_WORLD_DEVICE,
    deviceCoordinateFrame: AR_COORDINATE_FRAME.DEVICE_FRD,
  });
}

function candidate(distanceM, primary = `${distanceM}m`) {
  return {
    source: "guidanceCurrent",
    markerId: "guidance:primary|guidanceCurrent|turn_gate||13",
    descriptor: { kind: "turn_gate", primary },
    distanceM,
    placedBy: "path",
    anchor: { x: distanceM, y: 0, z: 0, headingRad: 0 },
  };
}

function path() {
  return { x: [0, 100, 200, 300], y: [0, 0, 0, 0], z: [0, 0, 0, 0] };
}

function navi(publishMonoTimeNanos) {
  return { sessionId: "route-a", generation: 1, publishMonoTimeNanos };
}

test("moving the device changes only the projected relative pose", () => {
  const origin = pose();
  const worldAnchor = createImmutableWorldAnchor(
    { x: 40, y: 2, z: 1, headingRad: 0 },
    origin,
  );
  const moved = pose({ position: [10, 0, 0], timestampNs: 2 });
  const projected = projectWorldAnchorToRoute(worldAnchor, moved);

  assert.deepEqual(worldAnchor.position, [40, -2, -1]);
  assert.equal(projected.x, 30);
  assert.equal(projected.y, 2);
  assert.equal(projected.z, 1);
  assert.deepEqual(worldAnchor.position, [40, -2, -1]);
});

test("device yaw moves a fixed marker oppositely and preserves world orientation", () => {
  const worldAnchor = createImmutableWorldAnchor(
    { x: 10, y: 0, z: 0, headingRad: 0 },
    pose(),
  );
  const halfSqrt = Math.SQRT1_2;
  const turned = pose({ orientation: [halfSqrt, 0, 0, halfSqrt], timestampNs: 2 });
  const projected = projectWorldAnchorToRoute(worldAnchor, turned);

  assert.ok(Math.abs(projected.x) < 1e-9);
  assert.ok(Math.abs(projected.y - 10) < 1e-9);
  assert.ok(Math.abs(projected.headingRad - Math.PI / 2) < 1e-9);
  assert.deepEqual(worldAnchor.roadForward, [1, 0, 0]);
});

test("store keeps one immutable world anchor across presented frames and Navi repaint", () => {
  const store = createContinuousAnchorStore();
  const first = store.update({
    nowMs: 0,
    navi: navi(1),
    candidates: [candidate(80)],
    modelPosition: path(),
    valid: true,
    precise: true,
    canHold: true,
    trackingState: "tracking",
    worldPose: pose(),
  });
  const immutable = first.anchors[0].worldAnchor;
  const moved = store.update({
    nowMs: 100,
    navi: navi(1),
    candidates: [candidate(80)],
    modelPosition: path(),
    valid: true,
    precise: true,
    canHold: true,
    trackingState: "tracking",
    worldPose: pose({ position: [10, 0, 0], timestampNs: 2 }),
  });
  const repainted = store.update({
    nowMs: 500,
    navi: navi(2),
    candidates: [candidate(75, "75m")],
    modelPosition: path(),
    valid: true,
    precise: true,
    canHold: true,
    trackingState: "tracking",
    worldPose: pose({ position: [10, 0, 0], timestampNs: 3 }),
  });

  assert.equal(first.sourceMode, "world-anchor");
  assert.equal(moved.anchors[0].anchor.x, 70);
  assert.strictEqual(moved.anchors[0].worldAnchor, immutable);
  assert.strictEqual(repainted.anchors[0].worldAnchor, immutable);
  assert.equal(repainted.anchors[0].anchor.x, 70);
  assert.equal(repainted.anchors[0].descriptor.primary, "75m");
  assert.equal(store.status().worldLockedCount, 1);
});

test("world epoch change discards the prior anchor before accepting a new one", () => {
  const store = createContinuousAnchorStore();
  const first = store.update({
    nowMs: 0,
    navi: navi(1),
    candidates: [candidate(80)],
    modelPosition: path(),
    valid: true,
    precise: true,
    canHold: true,
    trackingState: "tracking",
    worldPose: pose(),
  });
  const changed = store.update({
    nowMs: 100,
    navi: { sessionId: "route-b", generation: 1, publishMonoTimeNanos: 2 },
    candidates: [candidate(50)],
    modelPosition: path(),
    valid: true,
    precise: true,
    canHold: true,
    trackingState: "tracking",
    worldPose: pose({ epoch: "route-b", position: [0, 0, 0], timestampNs: 2 }),
  });

  assert.notStrictEqual(changed.anchors[0].worldAnchor, first.anchors[0].worldAnchor);
  assert.equal(changed.anchors[0].anchor.x, 50);
  assert.equal(store.status().worldEpoch, "route-b");
});
