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

test("a 400ms scheduling gap keeps the world anchor without pretending it was a seek", () => {
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
  const delayed = store.update({
    nowMs: 400,
    probe: true,
    candidates: [],
    modelPosition: path(0),
    precise: true,
    canHold: true,
    odometry: { trans: [10, 0, 0], rot: [0, 0, 0] },
  });

  assert.equal(delayed.state, "held");
  assert.equal(delayed.anchors.length, 1);
  assert.equal(delayed.anchors[0].anchor.x, 40);
  assert.match(delayed.reason, /앵커 유지/);
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

test("untrusted odometry freezes briefly without integrating its motion", () => {
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

  assert.equal(next.state, "held");
  assert.equal(next.anchors[0].anchor.x, 40);
  assert.equal(next.reason, "odometry untracked hold 50ms");
  assert.equal(next.fixId, "calibrationProbe");

  const dropped = store.update({
    nowMs: 1_200,
    probe: true,
    candidates: [],
    modelPosition: path(0),
    canHold: false,
    precise: true,
    odometry: { trans: [10, 0, 0], rot: [0, 0, 0] },
  });
  assert.equal(dropped.state, "dropped");
  assert.equal(dropped.anchors, null);
});

test("tracking coasting owns the gap budget instead of the legacy 150ms deletion", () => {
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
  const coast = store.update({
    nowMs: 400,
    probe: true,
    candidates: [],
    trackingState: "coasting",
    retainAnchor: true,
    precise: false,
    canHold: false,
    odometry: null,
  });

  assert.equal(coast.state, "held");
  assert.equal(coast.anchors.length, 1);
  assert.equal(coast.anchors[0].anchor.x, 40);
  assert.match(coast.reason, /coasting freeze/);
});

test("tracking recovery replaces a frozen pose with a fresh fix without deleting identity", () => {
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
  store.update({
    nowMs: 400,
    probe: true,
    candidates: [],
    trackingState: "coasting",
    retainAnchor: true,
    precise: false,
    canHold: false,
    odometry: null,
  });
  const recovered = store.update({
    nowMs: 450,
    probe: true,
    candidates: [probe({ x: 35, y: 1, z: 0, headingRad: 0.1 })],
    modelPosition: path(1),
    trackingState: "tracking",
    trackingRecovered: true,
    precise: true,
    canHold: true,
    odometry: { trans: [10, 0, 0], rot: [0, 0, 0] },
  });

  assert.equal(recovered.state, "live");
  assert.equal(recovered.fixId, "calibrationProbe");
  assert.equal(recovered.anchors[0].anchor.x, 35);
  assert.match(recovered.reason, /fresh fix/);
});

test("same Navi marker keeps its world placement across coasting and a fresh publish", () => {
  const store = createContinuousAnchorStore();
  const candidate = (distanceM, lateralM) => ({
    source: "guidanceCurrent",
    markerId: "guidance:primary|guidanceCurrent|turn_gate||13",
    lifecycleSlot: "guidance:primary",
    descriptor: { kind: "turn_gate", primary: `${distanceM}m` },
    distanceM,
    placedBy: "geo",
    anchor: { x: distanceM, y: lateralM, z: 0.4, headingRad: 0.2 },
  });
  const navi = (publishMonoTimeNanos) => ({
    sessionId: "route-a", generation: 1, publishMonoTimeNanos,
  });

  store.update({
    nowMs: 0, navi: navi(1), candidates: [candidate(154.9, -18.1)],
    modelPosition: path(0), valid: true, precise: true, canHold: true,
    odometry: { trans: [0, 0, 0], rot: [0, 0, 0] },
  });
  store.update({
    nowMs: 400, navi: navi(1), candidates: [], trackingState: "coasting",
    retainAnchor: true, valid: false, precise: false, canHold: false, odometry: null,
  });
  const recovered = store.update({
    nowMs: 500, navi: navi(2), candidates: [candidate(153.5, -0.1)],
    modelPosition: path(0), trackingState: "tracking", trackingRecovered: true,
    valid: true, precise: true, canHold: true,
    odometry: { trans: [0, 0, 0], rot: [0, 0, 0] },
  });

  assert.equal(recovered.anchors.length, 1);
  assert.equal(recovered.anchors[0].anchor.y, -18.1);
  assert.equal(recovered.anchors[0].anchor.headingRad, 0.2);
  assert.equal(recovered.anchors[0].descriptor.primary, "153.5m");
});

test("lane surface snaps at its explicit event distance instead of the ego origin", () => {
  const store = createContinuousAnchorStore();
  const lane = {
    source: "lane",
    markerId: "lane:lane_band",
    descriptor: { kind: "lane_band", surface: true, laneOffsetM: 3.5 },
    distanceM: 60,
    placedBy: "path",
    anchor: { x: 60, y: 1.8, z: 0.15, headingRad: 0.04 },
  };
  const result = store.update({
    nowMs: 0, navi: { sessionId: "route-a", publishMonoTimeNanos: 1 },
    candidates: [lane], modelPosition: path(1.2), valid: true, precise: true,
    canHold: true, odometry: { trans: [0, 0, 0], rot: [0, 0, 0] },
  });

  assert.equal(result.anchors[0].anchor.x, 60);
  assert.equal(result.anchors[0].distanceM, 60);
  assert.equal(result.anchors[0].descriptor.laneOffsetM, 3.5);
});

test("tracking lost releases store anchors so the renderer can fade its cache", () => {
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
  const lost = store.update({
    nowMs: 1600,
    candidates: [],
    trackingState: "lost",
    retainAnchor: false,
    precise: false,
    canHold: false,
    reason: "uncertainty budget exceeded",
  });

  assert.equal(lost.state, "dropped");
  assert.equal(lost.anchors, null);
  assert.match(lost.reason, /uncertainty budget exceeded/);
});

test("a new Navi fix does not resnap the same marker after an untracked interval", () => {
  const store = createContinuousAnchorStore();
  const candidate = (distanceM) => ({
    source: "guidanceCurrent",
    markerId: "guidance:primary|guidanceCurrent|turn_gate||13",
    descriptor: { kind: "turn_gate", primary: `${distanceM}m` },
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
    candidates: [candidate(40)],
    modelPosition: path(0),
    valid: true,
    precise: true,
    canHold: true,
    odometry: { trans: [10, 0, 0], rot: [0, 0, 0] },
  });
  const held = store.update({
    nowMs: 50,
    navi: navi(1),
    candidates: [candidate(40)],
    modelPosition: path(0),
    valid: true,
    precise: true,
    canHold: false,
    odometry: { trans: [10, 0, 0], rot: [0, 0, 0] },
  });
  const refreshed = store.update({
    nowMs: 100,
    navi: navi(2),
    candidates: [candidate(38)],
    modelPosition: path(0),
    valid: true,
    precise: true,
    canHold: true,
    odometry: { trans: [10, 0, 0], rot: [0, 0, 0] },
  });

  assert.equal(held.anchors[0].anchor.x, 40);
  assert.equal(refreshed.state, "live");
  assert.equal(refreshed.anchors[0].anchor.x, 40);
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

test("Navi generation updates repaint within one route without resetting the world anchor", () => {
  const store = createContinuousAnchorStore();
  const candidate = (distanceM, lateralM, primary) => ({
    source: "guidanceCurrent",
    markerId: "guidance:primary|guidanceCurrent|turn_gate||13",
    descriptor: { kind: "turn_gate", primary },
    distanceM,
    placedBy: "path",
    anchor: { x: distanceM, y: lateralM, z: 0, headingRad: 0.8 },
  });
  const navi = (generation, publishMonoTimeNanos) => ({
    sessionId: "route-a",
    generation,
    publishMonoTimeNanos,
  });

  store.update({
    nowMs: 0,
    navi: navi(10, 100),
    candidates: [candidate(80, 0, "80m")],
    modelPosition: path(0),
    valid: true,
    precise: true,
    canHold: true,
    odometry: { trans: [10, 0, 0], rot: [0, 0, 0] },
  });
  const propagated = store.update({
    nowMs: 50,
    navi: navi(10, 100),
    candidates: [candidate(80, 0, "80m")],
    modelPosition: path(0),
    valid: true,
    precise: true,
    canHold: true,
    odometry: { trans: [10, 0, 0], rot: [0, 0, 0] },
  });
  const refreshed = store.update({
    nowMs: 100,
    navi: navi(11, 200),
    candidates: [candidate(78, 3, "78m")],
    modelPosition: path(3),
    valid: true,
    precise: true,
    canHold: true,
    odometry: { trans: [10, 0, 0], rot: [0, 0, 0] },
  });

  assert.equal(propagated.anchors[0].anchor.x, 79.5);
  assert.equal(refreshed.anchors[0].anchor.x, 79.5);
  assert.equal(refreshed.anchors[0].anchor.y, 0);
  assert.equal(refreshed.anchors[0].descriptor.primary, "78m");
  assert.equal(refreshed.sourceId, "session:route-a");
  assert.equal(refreshed.reason, "");
});

test("a real Navi session change starts a fresh world anchor", () => {
  const store = createContinuousAnchorStore();
  const candidate = (distanceM, lateralM) => ({
    source: "guidanceCurrent",
    markerId: "guidance:primary|guidanceCurrent|turn_gate||13",
    descriptor: { kind: "turn_gate", primary: `${distanceM}m` },
    distanceM,
    placedBy: "path",
    anchor: { x: distanceM, y: lateralM, z: 0, headingRad: 0 },
  });
  const update = (nowMs, sessionId, generation, item, modelPosition) => store.update({
    nowMs,
    navi: { sessionId, generation, publishMonoTimeNanos: nowMs + 1 },
    candidates: [item],
    modelPosition,
    valid: true,
    precise: true,
    canHold: true,
    odometry: { trans: [0, 0, 0], rot: [0, 0, 0] },
  });

  update(0, "route-a", 10, candidate(80, 0), path(0));
  const changed = update(100, "route-b", 11, candidate(60, 4), path(4));

  assert.equal(changed.anchors[0].anchor.x, 60);
  assert.equal(changed.anchors[0].anchor.y, 4);
  assert.equal(changed.sourceId, "session:route-b");
  assert.equal(changed.reason, "session 변경 — 이전 앵커 reset");
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

test("an active marker keeps its world anchor through a temporary placement miss", () => {
  const store = createContinuousAnchorStore();
  const marker = (markerId, source, distanceM, withAnchor = true) => ({
    source,
    markerId,
    lifecycleSlot: source === "guidanceNext" ? "guidance:next" : "guidance:primary",
    descriptor: { kind: "turn_gate", primary: `${distanceM}m` },
    distanceM,
    placedBy: "path",
    ...(withAnchor ? { anchor: { x: distanceM, y: 0, z: 0, headingRad: 0 } } : {}),
  });
  const current = marker("turn-current", "guidanceCurrent", 42);
  const next = marker("turn-next", "guidanceNext", 220);
  const update = (nowMs, publish, candidates, activeMarkers) => store.update({
    nowMs,
    navi: { sessionId: "route-a", generation: publish, publishMonoTimeNanos: publish },
    candidates,
    activeMarkers,
    lifecycleAuthoritative: true,
    modelPosition: { x: [0, 50, 100, 150, 200, 250], y: [0, 0, 0, 0, 0, 0], z: [0, 0, 0, 0, 0, 0] },
    valid: true,
    precise: true,
    canHold: true,
    odometry: { trans: [0, 0, 0], rot: [0, 0, 0] },
  });

  update(0, 1, [current, next], [current, next]);
  const missed = update(500, 2, [marker("turn-current", "guidanceCurrent", 38)], [
    marker("turn-current", "guidanceCurrent", 38, false),
    marker("turn-next", "guidanceNext", 198, false),
  ]);
  const heldNext = missed.anchors.find((item) => item.markerId === "turn-next");
  assert.equal(missed.anchors.length, 2);
  assert.equal(heldNext.lifecycleState, "active-unresolved");
  assert.equal(heldNext.anchor.x, 220);
  assert.equal(store.status(500).unresolvedCount, 1);

  const promoted = marker("turn-next", "guidanceCurrent", 190);
  const resumed = update(2500, 3, [promoted], [promoted]);
  assert.equal(resumed.anchors.length, 1);
  assert.equal(resumed.anchors[0].markerId, "turn-next");
  assert.equal(resumed.anchors[0].lifecycleState, "active");
  assert.equal(resumed.anchors[0].anchor.x, 220);
});

test("an authoritative empty marker snapshot retires stale anchors", () => {
  const store = createContinuousAnchorStore();
  const marker = {
    source: "guidanceCurrent",
    markerId: "turn-a",
    lifecycleSlot: "guidance:primary",
    descriptor: { kind: "turn_gate" },
    distanceM: 40,
    placedBy: "path",
    anchor: { x: 40, y: 0, z: 0, headingRad: 0 },
  };
  const common = {
    modelPosition: path(0), valid: true, precise: true, canHold: true,
    lifecycleAuthoritative: true,
    odometry: { trans: [0, 0, 0], rot: [0, 0, 0] },
  };
  store.update({
    ...common, nowMs: 0,
    navi: { sessionId: "route-a", generation: 1, publishMonoTimeNanos: 1 },
    candidates: [marker], activeMarkers: [marker],
  });
  const retired = store.update({
    ...common, nowMs: 500,
    navi: { sessionId: "route-a", generation: 2, publishMonoTimeNanos: 2 },
    candidates: [], activeMarkers: [],
  });

  assert.equal(retired.anchors, null);
  assert.equal(store.status(500).anchorCount, 0);
});

test("an unresolved active marker expires after its bounded fix grace", () => {
  const store = createContinuousAnchorStore({
    limits: { unresolvedGraceMs: 1000, unresolvedGraceFixes: 2 },
  });
  const anchored = {
    source: "guidanceNext", markerId: "turn-next", lifecycleSlot: "guidance:next",
    descriptor: { kind: "turn_gate" }, distanceM: 100, placedBy: "path",
    anchor: { x: 100, y: 0, z: 0, headingRad: 0 },
  };
  const active = { ...anchored };
  delete active.anchor;
  const update = (nowMs, generation, candidates) => store.update({
    nowMs,
    navi: { sessionId: "route-a", generation, publishMonoTimeNanos: generation },
    candidates,
    activeMarkers: [active],
    lifecycleAuthoritative: true,
    modelPosition: path(0), valid: true, precise: true, canHold: true,
    odometry: { trans: [0, 0, 0], rot: [0, 0, 0] },
  });

  update(0, 1, [anchored]);
  assert.equal(update(400, 2, []).anchors.length, 1);
  assert.equal(update(800, 3, []).anchors.length, 1);
  assert.equal(update(1200, 4, []).anchors, null);
});
