import assert from "node:assert/strict";
import test from "node:test";

import { createContinuousAnchorStore } from "../src/features/drive/contents/vision/ar/anchor_store.js";

function probe(anchor) {
  return {
    source: "calibrationProbe",
    eventKey: "calibrationProbe|turn_gate",
    descriptor: { kind: "turn_gate" },
    distanceM: 40,
    placedBy: "path",
    anchor,
  };
}

function path(y) {
  return {
    x: [0, 20, 40, 60],
    y: [0, y / 2, y, y * 1.5],
    z: [0, 0, 0, 0],
  };
}

test("calibration probe is one continuous world anchor instead of a new 40m anchor every frame", () => {
  const store = createContinuousAnchorStore();
  const first = store.update({
    nowMs: 0,
    probe: true,
    candidates: [probe({ x: 40, y: 0, z: 0, headingRad: 0 })],
    modelPosition: path(0),
    precise: true,
    canHold: true,
    odometry: { trans: [10, 0, 0], rot: [0, 0, 0] },
  });
  assert.equal(first.anchors[0].anchor.x, 40);

  const next = store.update({
    nowMs: 50,
    probe: true,
    // A freshly composed probe still says "40m ahead" and the model path jumps
    // sideways. Neither value may replace the already established world anchor.
    candidates: [probe({ x: 40, y: 2, z: 0, headingRad: 0 })],
    modelPosition: path(2),
    precise: true,
    canHold: true,
    odometry: { trans: [10, 0, 0], rot: [0, 0, 0] },
  });

  assert.ok(Math.abs(next.anchors[0].anchor.x - 39.5) < 1e-9);
  assert.equal(next.anchors[0].distanceM, 39.5);
  assert.equal(next.anchors[0].descriptor.primary, "40m");
  assert.equal(next.anchors[0].anchor.y, 0);
  assert.equal(next.anchors[0].anchor.headingRad, 0);
  assert.equal(next.reason, "odometry world lock");
  assert.equal(next.fixId, "calibrationProbe");
});

test("a transient imprecise frame keeps the approved anchor in HELD state", () => {
  const store = createContinuousAnchorStore();
  store.update({
    nowMs: 0,
    probe: true,
    candidates: [probe({ x: 40, y: 0, z: 0, headingRad: 0 })],
    modelPosition: path(0),
    precise: true,
    canHold: true,
    odometry: { trans: [10, 0, 0], rot: [0, 0, 0] },
  });
  const held = store.update({
    nowMs: 50,
    probe: true,
    candidates: [],
    modelPosition: path(4),
    precise: false,
    canHold: true,
    odometry: { trans: [10, 0, 0], rot: [0, 0, 0], transStd: [0, 0.01, 0], rotStd: [0, 0, 0.001] },
  });

  assert.equal(held.state, "held");
  assert.ok(Math.abs(held.anchors[0].anchor.x - 39.5) < 1e-9);
  assert.equal(held.anchors[0].anchor.y, 0);
});

test("a one-frame invalid source holds instead of deleting the world anchor", () => {
  const store = createContinuousAnchorStore();
  store.update({
    nowMs: 0,
    probe: true,
    candidates: [probe({ x: 40, y: 0, z: 0, headingRad: 0 })],
    modelPosition: path(0),
    precise: true,
    canHold: true,
    odometry: { trans: [5, 0, 0], rot: [0, 0, 0] },
  });
  const held = store.update({
    nowMs: 50,
    valid: false,
    candidates: [],
    precise: false,
    canHold: true,
    odometry: { trans: [5, 0, 0], rot: [0, 0, 0], transStd: [0, 0, 0], rotStd: [0, 0, 0] },
  });

  assert.equal(held.state, "held");
  assert.equal(held.anchors.length, 1);
  assert.ok(Math.abs(held.anchors[0].anchor.x - 39.75) < 1e-9);
});

test("an inactive Navi object does not turn the calibration probe into a 2Hz Navi fix", () => {
  const store = createContinuousAnchorStore();
  const navi = { vehicle: { latitude: 37, longitude: 127 } };
  store.update({
    nowMs: 0,
    navi,
    probe: true,
    candidates: [probe({ x: 40, y: 0, z: 0, headingRad: 0 })],
    modelPosition: path(0),
    canHold: true,
    odometry: { trans: [5, 0, 0], rot: [0, 0, 0] },
  });
  const next = store.update({
    nowMs: 100,
    navi: { vehicle: { latitude: 37.00001, longitude: 127.00001 } },
    probe: true,
    candidates: [probe({ x: 40, y: 0, z: 0, headingRad: 0 })],
    modelPosition: path(0),
    canHold: true,
    odometry: { trans: [5, 0, 0], rot: [0, 0, 0] },
  });

  assert.ok(Math.abs(next.anchors[0].anchor.x - 39.5) < 1e-9);
  assert.equal(next.sourceId, "calibrationProbe");
});

test("a conservative odometry hold gap does not recreate a precise probe at 40m", () => {
  const store = createContinuousAnchorStore();
  store.update({
    nowMs: 0,
    probe: true,
    candidates: [probe({ x: 40, y: 0, z: 0, headingRad: 0 })],
    modelPosition: path(0),
    canHold: true,
    precise: true,
    odometry: { trans: [10, 0, 0], rot: [0, 0, 0] },
  });
  const next = store.update({
    nowMs: 50,
    probe: true,
    candidates: [probe({ x: 40, y: 0, z: 0, headingRad: 0 })],
    modelPosition: path(0),
    canHold: false,
    precise: true,
    odometry: { trans: [10, 0, 0], rot: [0, 0, 0] },
  });

  assert.ok(Math.abs(next.anchors[0].anchor.x - 39.5) < 1e-9);
  assert.equal(next.fixId, "calibrationProbe");
});

test("a new Navi publish repaints but does not relocate the same world marker", () => {
  const store = createContinuousAnchorStore();
  const candidate = (distanceM, anchor, primary) => ({
    source: "guidanceCurrent",
    markerId: "guidance:primary|guidanceCurrent|turn_gate||13",
    lifecycleSlot: "guidance:primary",
    descriptor: { kind: "turn_gate", primary },
    distanceM,
    placedBy: "path",
    anchor,
  });
  const navi = (publishMonoTimeNanos) => ({
    sessionId: "route-a",
    generation: 1,
    publishMonoTimeNanos,
  });

  store.update({
    nowMs: 0,
    navi: navi(1),
    candidates: [candidate(40, { x: 40, y: 0, z: 0, headingRad: 0 }, "40m")],
    modelPosition: path(0),
    valid: true,
    precise: true,
    canHold: true,
    odometry: { trans: [10, 0, 0], rot: [0, 0, 0] },
  });
  const propagated = store.update({
    nowMs: 50,
    navi: navi(1),
    candidates: [candidate(40, { x: 40, y: 0, z: 0, headingRad: 0 }, "40m")],
    modelPosition: path(0),
    valid: true,
    precise: true,
    canHold: true,
    odometry: { trans: [10, 0, 0], rot: [0, 0, 0] },
  });
  const refreshed = store.update({
    nowMs: 100,
    navi: navi(2),
    candidates: [candidate(38, { x: 38, y: 2, z: 1, headingRad: 1.2 }, "38m")],
    modelPosition: path(2),
    valid: true,
    precise: true,
    canHold: true,
    odometry: { trans: [10, 0, 0], rot: [0, 0, 0] },
  });

  assert.equal(propagated.anchors[0].anchor.x, 39.5);
  assert.equal(refreshed.anchors[0].anchor.x, 39.5);
  assert.equal(refreshed.anchors[0].anchor.headingRad, 0);
  assert.equal(refreshed.anchors[0].descriptor.primary, "38m");
});

test("a large forward reset starts the next coordinate-less event occurrence", () => {
  const store = createContinuousAnchorStore();
  const marker = (distanceM) => ({
    source: "sdi",
    markerId: "sdi:caution_sign|sdi|caution_sign||camera",
    lifecycleSlot: "sdi:caution_sign",
    descriptor: { kind: "caution_sign" },
    distanceM,
    placedBy: "path",
    anchor: { x: distanceM, y: 0, z: 0, headingRad: 0 },
  });
  const navi = (publishMonoTimeNanos) => ({
    sessionId: "route-a",
    generation: 1,
    publishMonoTimeNanos,
  });

  store.update({
    nowMs: 0,
    navi: navi(1),
    candidates: [marker(12)],
    modelPosition: path(0),
    valid: true,
    precise: true,
    canHold: true,
    odometry: { trans: [0, 0, 0], rot: [0, 0, 0] },
  });
  const next = store.update({
    nowMs: 100,
    navi: navi(2),
    candidates: [marker(90)],
    modelPosition: { x: [0, 30, 60, 90, 120], y: [0, 0, 0, 0, 0], z: [0, 0, 0, 0, 0] },
    valid: true,
    precise: true,
    canHold: true,
    odometry: { trans: [0, 0, 0], rot: [0, 0, 0] },
  });

  assert.equal(next.anchors[0].anchor.x, 90);
});
