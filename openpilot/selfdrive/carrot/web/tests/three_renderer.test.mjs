import assert from "node:assert/strict";
import test from "node:test";

import {
  AR_MARKER_KIND,
  markerSupportHeightM,
} from "../src/features/drive/contents/vision/ar/tokens.js";
import { AR_TONE } from "../src/features/drive/contents/vision/ar/design_tokens.js";
import {
  describeSignboard,
  signboardFromMarker,
} from "../src/features/drive/contents/vision/ar/signboard.js";
import { pointOnPath, projectRouteFluPoint } from "../src/features/drive/contents/vision/ar/projection.js";
import { createMarkerPresentationFilter } from "../src/features/drive/contents/vision/ar/presentation_filter.js";
import {
  createThreeArRenderer,
  markerProjectionState,
  markerScreenMetrics,
  uprightPresentationAnchor,
} from "../src/features/drive/contents/vision/ar/three_adapter.js";

function fakeSurface() {
  const listeners = new Map();
  return {
    width: 1920,
    height: 1080,
    addEventListener(type, listener) { listeners.set(type, listener); },
    removeEventListener(type, listener) {
      if (listeners.get(type) === listener) listeners.delete(type);
    },
    dispatch(type, event = {}) { listeners.get(type)?.(event); },
  };
}

function fakeWebglRenderer() {
  return {
    renders: 0,
    clears: 0,
    sizes: [],
    disposed: false,
    contextReleased: false,
    setClearColor() {},
    setSize(width, height, updateStyle) { this.sizes.push([width, height, updateStyle]); },
    render() { this.renders += 1; },
    clear() { this.clears += 1; },
    dispose() { this.disposed = true; },
    forceContextLoss() { this.contextReleased = true; },
  };
}

function frame({ laneWidthM = 3.5, nowMs = 0, diagnosticsEnabled = false } = {}) {
  const modelPosition = {
    x: [0, 20, 40, 60, 80],
    y: [0, 0.2, 0.8, 1.8, 3.2],
    z: [0, 0, 0.1, 0.15, 0.2],
  };
  const sign = describeSignboard({
    tone: AR_TONE.GUIDE,
    primary: "40m",
    secondary: "우회전",
    turnSign: 1,
    phase: "precise",
  });
  const lane = signboardFromMarker({
    kind: AR_MARKER_KIND.LANE_BAND,
    distanceM: 60,
    laneWidthM,
    laneOffsetM: 0.4,
    phase: "precise",
  });
  return {
    nowMs,
    diagnosticsEnabled,
    stage: {
      calibTransform: [
        [960, 1000, 0],
        [540, 0, 1000],
        [1, 0, 0],
      ],
      scale: 1,
      tx: 0,
      ty: 0,
      stageWidth: 1920,
      stageHeight: 1080,
    },
    sync: { canDrawPrecise: true, reasons: [] },
    held: false,
    modelPosition,
    egoSpeedMps: 10,
    signs: [
      {
        eventKey: "guidanceCurrent|turn",
        descriptor: Object.freeze({ ...sign, kind: AR_MARKER_KIND.TURN_GATE }),
        distanceM: 40,
        anchor: { x: 40, y: 0.8, z: 0.1, headingRad: 0.04 },
      },
      {
        eventKey: "lane",
        descriptor: lane,
        distanceM: 60,
        anchor: { x: 60, y: 1.6, z: 0.15, headingRad: 0.04 },
      },
    ],
  };
}

function fakePaintContext() {
  const gradient = { addColorStop() {} };
  return {
    clearRect() {}, save() {}, restore() {}, beginPath() {}, moveTo() {}, lineTo() {},
    arcTo() {}, closePath() {}, fill() {}, stroke() {}, strokeText() {},
    fillText() {}, translate() {}, rotate() {}, arc() {}, fillRect() {},
    createLinearGradient() { return gradient; },
    createRadialGradient() { return gradient; },
    measureText(text) { return { width: String(text || "").length * 24 }; },
  };
}

function fakeTexture() {
  const context = fakePaintContext();
  return {
    image: { width: 0, height: 0, getContext: () => context },
    dispose() {},
  };
}

test("presentation filter smooths lateral pose, road direction and scale without delaying distance", () => {
  const filter = createMarkerPresentationFilter();
  const initial = filter.update({ x: 40, y: 0, z: 0, headingRad: 0 }, 1, 0);
  const next = filter.update({ x: 39, y: 1, z: 0.5, headingRad: 0.3 }, 1.8, 50);

  assert.equal(initial.anchor.x, 40);
  assert.equal(next.anchor.x, 39);
  assert.ok(next.anchor.y > 0 && next.anchor.y < 1);
  assert.ok(next.anchor.z > 0 && next.anchor.z < 0.5);
  assert.ok(next.anchor.headingRad > 0 && next.anchor.headingRad < 0.3);
  assert.ok(next.scale > 1 && next.scale < 1.8);
});

test("presentation filter eases logged route corrections but snaps a severe discontinuity", () => {
  const filter = createMarkerPresentationFilter();
  filter.update({ x: 40, y: 0, z: 0, headingRad: 0 }, 1, 0);
  const eased = filter.update({ x: 39, y: 5.5, z: 0, headingRad: 0.4 }, 1.5, 50);
  const snapped = filter.update({ x: 38, y: 20, z: 0, headingRad: 0.8 }, 2, 100);

  assert.ok(eased.anchor.y > 0 && eased.anchor.y < 5.5);
  assert.ok(eased.anchor.headingRad > 0 && eased.anchor.headingRad < 0.4);
  assert.ok(eased.scale > 1 && eased.scale < 1.5);
  assert.equal(snapped.anchor.y, 20);
  assert.equal(snapped.anchor.headingRad, 0.8);
  assert.equal(snapped.scale, 2);
});

test("screen metrics follow the marker road-up axis on an incline", () => {
  const currentFrame = frame();
  const descriptor = currentFrame.signs[0].descriptor;
  const anchor = pointOnPath({
    x: [0, 20, 40, 60],
    y: [0, 1, 2, 3],
    z: [0, 3, 6, 9],
  }, 40);
  const metrics = markerScreenMetrics(descriptor, anchor, currentFrame.stage);

  assert.ok(metrics);
  assert.ok(metrics.heightPx > 0.5);
  assert.ok(metrics.widthPx > metrics.heightPx);
  assert.equal(metrics.opticalHeightM, descriptor.heightM);
  assert.equal(metrics.opticalWidthM, descriptor.widthM);
});

test("product support and face scaling lower the sign without moving its anchor", () => {
  const currentFrame = frame();
  const descriptor = currentFrame.signs[0].descriptor;
  const anchor = currentFrame.signs[0].anchor;
  const approved = markerScreenMetrics(descriptor, anchor, currentFrame.stage);
  const supportHeightM = markerSupportHeightM(descriptor);
  const product = markerScreenMetrics(descriptor, anchor, currentFrame.stage, { supportHeightM });
  const compact = markerScreenMetrics(descriptor, anchor, currentFrame.stage, {
    supportHeightM,
    presentationScale: 0.5,
  });

  assert.ok(approved && product && compact);
  assert.ok(product.centerY > approved.centerY);
  assert.equal(product.heightPx, approved.heightPx);
  assert.ok(compact.centerY > product.centerY);
  assert.ok(compact.heightPx < product.heightPx);
  assert.deepEqual(anchor, currentFrame.signs[0].anchor);
});

test("explicit FLU projection keeps face above support without screen-space correction", () => {
  const currentFrame = frame();
  const stage = {
    ...currentFrame.stage,
    calibTransform: [
      [960, 1000, 0],
      [540, 0, 1000],
      [1, 0, 0],
    ],
  };
  const source = { x: 40, y: 0.8, z: 0.1, headingRad: 0.04 };
  const corrected = uprightPresentationAnchor(source, 0);
  const projectStage = (point) => {
    const projected = projectRouteFluPoint(stage.calibTransform, point.x, point.y, point.z);
    return {
      x: projected.x * stage.scale + stage.tx,
      y: projected.y * stage.scale + stage.ty,
    };
  };
  const base = projectStage(corrected);
  const top = projectStage({
    x: corrected.x + corrected.faceUp[0] * 4,
    y: corrected.y + corrected.faceUp[1] * 4,
    z: corrected.z + corrected.faceUp[2] * 4,
  });
  const right = projectStage({
    x: corrected.x + corrected.faceRight[0],
    y: corrected.y + corrected.faceRight[1],
    z: corrected.z + corrected.faceRight[2],
  });

  assert.equal("stageUpFlipped" in corrected, false);
  assert.equal(corrected.headingRad, source.headingRad);
  assert.ok(top.y < base.y, "the sign face must render above its support");
  assert.ok(right.x > base.x, "local right must remain screen-right so labels are readable");
});

test("screen metrics remove an upright sign before it grows across the lower driver view", () => {
  const currentFrame = frame();
  const descriptor = currentFrame.signs[0].descriptor;
  const near = markerScreenMetrics(
    descriptor,
    { x: 2.5, y: 0, z: 0, headingRad: 0 },
    currentFrame.stage,
  );
  const stillTooNear = markerScreenMetrics(
    descriptor,
    { x: 4, y: 0, z: 0, headingRad: 0 },
    currentFrame.stage,
  );
  const safe = markerScreenMetrics(
    descriptor,
    { x: 9, y: 0, z: 0, headingRad: 0 },
    currentFrame.stage,
  );

  assert.equal(near, null);
  assert.equal(stillTooNear, null);
  assert.ok(safe);
});

test("projection diagnostics distinguish behind-camera and near-plane markers", () => {
  const currentFrame = frame();
  const descriptor = currentFrame.signs[0].descriptor;
  assert.equal(
    markerProjectionState(descriptor, { x: -4, y: 0, z: 0, headingRad: 0 }, currentFrame.stage).state,
    "behind-camera",
  );
  assert.equal(
    markerProjectionState(descriptor, { x: 4, y: 0, z: 0, headingRad: 0 }, currentFrame.stage).state,
    "near-plane",
  );
  assert.equal(
    markerProjectionState(descriptor, { x: 40, y: 0, z: 0, headingRad: 0 }, currentFrame.stage).state,
    "active-candidate",
  );
});

test("BAND projection measures its longitudinal road footprint", () => {
  const currentFrame = frame();
  const lane = currentFrame.signs[1];
  const projection = markerProjectionState(lane.descriptor, {
    ...lane.anchor,
    z: -1,
    roadForward: [0.995, 0, -0.1],
    roadUp: [0.1, 0, 0.995],
  }, currentFrame.stage);

  assert.equal(projection.state, "active-candidate");
  assert.equal(projection.metrics.opticalHeightM, lane.descriptor.heightM);
  assert.equal(projection.metrics.opticalWidthM, lane.descriptor.widthM);
  assert.ok(projection.metrics.heightPx > 0.5);
  assert.ok(projection.metrics.widthPx > 0.5);
});

test("BAND far-legibility scaling grows only its road length", () => {
  const currentFrame = frame();
  const lane = currentFrame.signs[1];
  const base = markerProjectionState(lane.descriptor, lane.anchor, currentFrame.stage);
  const extended = markerProjectionState(lane.descriptor, lane.anchor, currentFrame.stage, {
    presentationHeightScale: 5,
    presentationWidthScale: 1,
  });

  assert.equal(base.state, "active-candidate");
  assert.equal(extended.state, "active-candidate");
  assert.equal(extended.metrics.opticalHeightM, base.metrics.opticalHeightM * 5);
  assert.equal(extended.metrics.opticalWidthM, base.metrics.opticalWidthM);
  // Scaling extends the far edge away from the fixed road anchor, so screen
  // growth is intentionally sub-linear and the unchanged physical width is
  // slightly smaller at the new face centre.
  assert.ok(extended.metrics.heightPx > base.metrics.heightPx * 3);
  assert.ok(extended.metrics.widthPx < base.metrics.widthPx);
});

test("LANE_BAND is legible far away and cannot remain visible after projection explodes", () => {
  const renderer = createThreeArRenderer({
    surface: fakeSurface(),
    rendererFactory: fakeWebglRenderer,
    textureFactory: fakeTexture,
    shadowTextureFactory: fakeTexture,
  });
  const distant = frame({ nowMs: 0, diagnosticsEnabled: true });
  distant.signs = [{
    ...distant.signs[1],
    distanceM: 280,
    anchor: { ...distant.signs[1].anchor, x: 280 },
  }];
  const unassisted = markerScreenMetrics(
    distant.signs[0].descriptor,
    distant.signs[0].anchor,
    distant.stage,
  );
  assert.equal(renderer.render(distant), true);
  const distantBand = renderer.status().markers[0];
  assert.equal(distantBand.visibilityState, "active-visible");
  assert.ok(distantBand.screen.heightPx >= 24);
  assert.ok(distantBand.screen.heightPx > unassisted.heightPx * 10);
  assert.ok(distantBand.screen.widthPx < 100);

  const near = frame({ nowMs: 100, diagnosticsEnabled: true });
  near.signs = [{
    ...near.signs[1],
    distanceM: 10,
    anchor: { ...near.signs[1].anchor, x: 10, y: 30 },
  }];
  assert.equal(renderer.render(near), true);
  assert.notEqual(renderer.status().markers[0]?.visibilityState, "active-visible");
  renderer.destroy();
});

test("Three renderer draws approved upright and BAND components in one scene", () => {
  const surface = fakeSurface();
  const webgl = fakeWebglRenderer();
  let faceTextureCount = 0;
  const renderer = createThreeArRenderer({
    surface,
    rendererFactory: () => webgl,
    textureFactory: () => { faceTextureCount += 1; return fakeTexture(); },
    shadowTextureFactory: fakeTexture,
  });

  assert.equal(renderer.render(frame({ nowMs: 0, diagnosticsEnabled: true })), true);
  assert.equal(webgl.renders, 1);
  assert.deepEqual(webgl.sizes, [[1920, 1080, false]]);
  assert.equal(renderer.status().backend, "three");
  assert.equal(renderer.status().drawn, 2);
  assert.equal(renderer.status().textureCount, 2);
  assert.equal(renderer.status().cacheCreates, 2);
  assert.equal(renderer.status().markers.length, 2);
  assert.ok(renderer.status().markers.some((marker) => marker.visible && marker.screen?.centerX !== null));
  assert.equal(renderer.status().visibilityStates["active-visible"], 2);
  assert.equal(faceTextureCount, 2);

  // Mutable BAND dimensions update inside the existing marker entry.
  assert.equal(renderer.render(frame({ laneWidthM: 4.0, nowMs: 50 })), true);
  assert.equal(renderer.status().textureCount, 2);
  assert.equal(renderer.status().cacheCreates, 2);
  assert.equal(renderer.status().cacheDisposes, 0);
  assert.equal(renderer.status().geometryUpdates, 1);
  assert.equal(renderer.status().groupRebuilds, 0);
  assert.equal(faceTextureCount, 2);

  // 사용하지 않은 geometry는 한 프레임 누락에 폐기하지 않고 grace 뒤 정리한다.
  assert.equal(renderer.render(frame({ laneWidthM: 4.0, nowMs: 1_000 })), true);
  assert.equal(renderer.status().textureCount, 2);

  assert.equal(renderer.destroy(), true);
  assert.equal(webgl.disposed, true);
  assert.equal(webgl.contextReleased, true);
});

test("one marker repaints phase, text and geometry without restarting its cache lifecycle", () => {
  const surface = fakeSurface();
  const webgl = fakeWebglRenderer();
  let faceTextureCount = 0;
  const renderer = createThreeArRenderer({
    surface,
    rendererFactory: () => webgl,
    textureFactory: () => { faceTextureCount += 1; return fakeTexture(); },
    shadowTextureFactory: fakeTexture,
  });
  const first = frame({ nowMs: 0 });
  first.signs = [{
    ...first.signs[0],
    markerId: "maneuver-42",
    eventKey: "mutable-alias-a",
    lifecycleSlot: "guidance:next",
  }];
  const settled = frame({ nowMs: 220 });
  settled.signs = [{ ...first.signs[0] }];
  const updated = frame({ nowMs: 270 });
  updated.signs = [{
    ...first.signs[0],
    eventKey: "mutable-alias-b",
    lifecycleSlot: "guidance:primary",
    distanceM: 15,
    descriptor: Object.freeze({
      ...describeSignboard({
        tone: AR_TONE.GUIDE,
        primary: "15m",
        secondary: "updated road",
        turnSign: 1,
        phase: "commit",
      }),
      kind: AR_MARKER_KIND.TURN_GATE,
    }),
  }];

  assert.equal(renderer.render(first), true);
  assert.equal(renderer.render(settled), true);
  const beforeUpdateAlpha = renderer.status().minimumVisibilityAlpha;
  assert.ok(beforeUpdateAlpha > 0.01);
  assert.equal(renderer.render(updated), true);

  const status = renderer.status();
  assert.equal(status.textureCount, 1);
  assert.equal(status.cacheCreates, 1);
  assert.equal(status.cacheDisposes, 0);
  assert.equal(status.contentRepaints, 1);
  assert.equal(status.geometryUpdates, 1);
  assert.equal(status.groupRebuilds, 0);
  assert.equal(status.lifecycleTransitions, 1);
  assert.equal(status.lifecycleReplacements, 0);
  assert.ok(status.minimumVisibilityAlpha >= beforeUpdateAlpha);
  assert.equal(faceTextureCount, 1);
  renderer.destroy();
});

test("Three renderer keeps a sharp-turn sign readable while applying overlap selection", () => {
  const surface = fakeSurface();
  const webgl = fakeWebglRenderer();
  const renderer = createThreeArRenderer({
    surface,
    rendererFactory: () => webgl,
    textureFactory: fakeTexture,
    shadowTextureFactory: fakeTexture,
  });
  const currentFrame = frame({ nowMs: 0, diagnosticsEnabled: true });
  const primary = {
    ...currentFrame.signs[0],
    eventKey: "primary",
    source: "guidanceCurrent",
    anchor: { x: 40, y: 0, z: 0, headingRad: Math.PI / 2, routeDerived: true },
  };
  const overlappingNext = {
    ...currentFrame.signs[0],
    eventKey: "next",
    source: "guidanceNext",
    anchor: { x: 40, y: 0, z: 0, headingRad: Math.PI / 2 },
  };
  currentFrame.signs = [overlappingNext, primary];

  assert.equal(renderer.render(currentFrame), true);
  assert.equal(renderer.status().drawn, 1);
  assert.equal(renderer.status().selectionSuppressed, 1);
  assert.equal(renderer.status().billboarded, 1);
  assert.equal(renderer.status().farRouteAnchors, 1);
  assert.equal(renderer.status().textureCount, 1);
  assert.equal(renderer.status().visibilityStates["active-suppressed"], 1);
  assert.equal(
    renderer.status().markers.some((marker) => marker.visibilityState === "active-suppressed"),
    true,
  );
  renderer.destroy();
});

test("offscreen markers retain their cache and resume the existing fade state", () => {
  const surface = fakeSurface();
  const renderer = createThreeArRenderer({
    surface,
    rendererFactory: fakeWebglRenderer,
    textureFactory: fakeTexture,
    shadowTextureFactory: fakeTexture,
  });
  const visible = frame({ nowMs: 0, diagnosticsEnabled: true });
  visible.signs = [visible.signs[0]];
  const settled = { ...visible, nowMs: 220 };
  assert.equal(renderer.render(visible), true);
  assert.equal(renderer.render(settled), true);
  const before = renderer.status().minimumVisibilityAlpha;

  const offscreen = {
    ...visible,
    nowMs: 270,
    signs: [{ ...visible.signs[0], anchor: { x: 40, y: 100, z: 0, headingRad: 0 } }],
  };
  assert.equal(renderer.render(offscreen), true);
  assert.equal(renderer.status().drawn, 0);
  assert.equal(renderer.status().visibilityStates["active-offscreen"], 1);
  assert.equal(renderer.status().cacheDisposes, 0);
  assert.equal(renderer.status().textureCount, 1);

  assert.equal(renderer.render({ ...visible, nowMs: 320 }), true);
  assert.equal(renderer.status().cacheCreates, 1);
  assert.equal(renderer.status().cacheDisposes, 0);
  assert.ok(renderer.status().minimumVisibilityAlpha >= before);
  renderer.destroy();
});

test("an offscreen cached marker cannot resurrect as a passing fade", () => {
  const renderer = createThreeArRenderer({
    surface: fakeSurface(),
    rendererFactory: fakeWebglRenderer,
    textureFactory: fakeTexture,
    shadowTextureFactory: fakeTexture,
  });
  const visible = frame({ nowMs: 0, diagnosticsEnabled: true });
  visible.signs = [{
    ...visible.signs[0], markerId: "no-resurrection", eventKey: "no-resurrection",
    anchor: { x: 40, y: 0, z: 0, headingRad: 0 },
  }];
  assert.equal(renderer.render(visible), true);
  assert.equal(renderer.render({ ...visible, nowMs: 220 }), true);

  const offscreen = {
    ...visible,
    nowMs: 270,
    signs: [{ ...visible.signs[0], anchor: { x: 40, y: 100, z: 0, headingRad: 0 } }],
  };
  assert.equal(renderer.render(offscreen), true);
  assert.equal(renderer.status().drawn, 0);

  const nearPlane = {
    ...visible,
    nowMs: 320,
    signs: [{ ...visible.signs[0], anchor: { x: 6, y: 0, z: 0, headingRad: 0 } }],
  };
  assert.equal(renderer.render(nearPlane), true);
  assert.equal(renderer.status().drawn, 0);
  assert.equal(renderer.status().visibilityStates["near-plane"], 1);
  assert.equal(renderer.status().visibilityStates.passing, undefined);
  assert.equal(renderer.status().cacheDisposes, 0);
  renderer.destroy();
});

test("overlap priority changes wait for the bounded selection hysteresis", () => {
  const renderer = createThreeArRenderer({
    surface: fakeSurface(),
    rendererFactory: fakeWebglRenderer,
    textureFactory: fakeTexture,
    shadowTextureFactory: fakeTexture,
  });
  const initial = frame({ nowMs: 0, diagnosticsEnabled: true });
  const next = {
    ...initial.signs[0], markerId: "next", eventKey: "next",
    lifecycleSlot: "guidance:next", source: "guidanceNext",
  };
  initial.signs = [next];
  assert.equal(renderer.render(initial), true);

  const current = {
    ...next, markerId: "current", eventKey: "current",
    lifecycleSlot: "guidance:primary", source: "guidanceCurrent",
  };
  const takeover = { ...initial, nowMs: 100, signs: [next, current] };
  assert.equal(renderer.render(takeover), true);
  assert.equal(
    renderer.status().markers.find((marker) => marker.visible)?.markerId,
    "next",
  );

  assert.equal(renderer.render({ ...takeover, nowMs: 500 }), true);
  assert.equal(
    renderer.status().markers.find((marker) => marker.visible)?.markerId,
    "current",
  );
  renderer.destroy();
});

test("a near-plane upright marker hides its stale pre-clip pose but keeps its cache", () => {
  const renderer = createThreeArRenderer({
    surface: fakeSurface(),
    rendererFactory: fakeWebglRenderer,
    textureFactory: fakeTexture,
    shadowTextureFactory: fakeTexture,
  });
  const visible = frame({ nowMs: 0, diagnosticsEnabled: true });
  visible.signs = [{
    ...visible.signs[0], markerId: "passing", eventKey: "passing",
    anchor: { x: 9, y: 0, z: 0, headingRad: 0 },
  }];
  assert.equal(renderer.render(visible), true);
  assert.equal(renderer.render({ ...visible, nowMs: 220 }), true);

  const near = {
    ...visible,
    signs: [{ ...visible.signs[0], anchor: { x: 6, y: 0, z: 0, headingRad: 0 } }],
  };
  for (const nowMs of [270, 520, 770]) renderer.render({ ...near, nowMs });
  assert.equal(renderer.status().drawn, 0);
  assert.equal(renderer.status().visibilityStates["near-plane"], 1);
  assert.equal(renderer.status().visibilityStates.passing, undefined);
  assert.equal(renderer.status().cacheDisposes, 0);
  assert.equal(renderer.status().textureCount, 1);

  assert.equal(renderer.render({ ...visible, nowMs: 1800, signs: [] }), true);
  assert.equal(renderer.status().textureCount, 0);
  assert.equal(renderer.status().cacheDisposes, 1);
  renderer.destroy();
});

test("Three renderer fades the last anchored scene instead of clearing one bad frame", () => {
  const surface = fakeSurface();
  const webgl = fakeWebglRenderer();
  const renderer = createThreeArRenderer({
    surface,
    rendererFactory: () => webgl,
    textureFactory: fakeTexture,
    shadowTextureFactory: fakeTexture,
  });

  assert.equal(renderer.render(frame({ nowMs: 0 })), true);
  assert.equal(renderer.render(frame({ nowMs: 220 })), true);
  const invalid = frame({ nowMs: 270 });
  invalid.sync = { canDrawPrecise: false, reasons: ["transient model gap"] };
  invalid.held = false;
  invalid.signs = [];

  assert.equal(renderer.render(invalid), false);
  assert.equal(webgl.renders, 3);
  assert.equal(webgl.clears, 0);
  assert.equal(renderer.status().lastReason, "transient model gap");
  renderer.destroy();
});

test("tracking confidence reduces the anchored scene without using the abrupt legacy held alpha", () => {
  const surface = fakeSurface();
  const webgl = fakeWebglRenderer();
  const renderer = createThreeArRenderer({
    surface,
    rendererFactory: () => webgl,
    textureFactory: fakeTexture,
    shadowTextureFactory: fakeTexture,
  });
  const tracked = frame({ nowMs: 0 });
  tracked.tracking = { state: "tracking", alpha: 1 };
  const coasting = frame({ nowMs: 100 });
  coasting.sync = { canDrawPrecise: false, reasons: ["short input gap"] };
  coasting.held = true;
  coasting.tracking = { state: "coasting", alpha: 0.84 };

  assert.equal(renderer.render(tracked), true);
  assert.equal(renderer.status().trackingAlpha, 1);
  assert.equal(renderer.render(coasting), true);
  assert.equal(renderer.status().trackingAlpha, 0.84);
  assert.equal(renderer.status().cacheCreates, 2);
  assert.equal(renderer.status().cacheDisposes, 0);
  renderer.destroy();
});

test("a marker replacement in the same lifecycle slot never cross-fades two signs", () => {
  const surface = fakeSurface();
  const webgl = fakeWebglRenderer();
  const renderer = createThreeArRenderer({
    surface,
    rendererFactory: () => webgl,
    textureFactory: fakeTexture,
    shadowTextureFactory: fakeTexture,
  });
  const first = frame({ nowMs: 0 });
  first.signs = [{
    ...first.signs[0],
    markerId: "turn-a",
    eventKey: "turn-a",
    lifecycleSlot: "guidance:primary",
  }];
  const replacement = frame({ nowMs: 50 });
  replacement.signs = [{
    ...replacement.signs[0],
    markerId: "turn-b",
    eventKey: "turn-b",
    lifecycleSlot: "guidance:primary",
    anchor: { x: 80, y: 1, z: 0, headingRad: 0.2 },
  }];

  assert.equal(renderer.render(first), true);
  assert.equal(renderer.status().drawn, 1);
  assert.equal(renderer.render(replacement), true);
  assert.equal(renderer.status().drawn, 1);
  assert.equal(renderer.status().textureCount, 1);
  assert.equal(renderer.status().cacheCreates, 2);
  assert.equal(renderer.status().cacheDisposes, 1);
  assert.equal(renderer.status().lifecycleReplacements, 1);
  renderer.destroy();
});

test("Three renderer fails closed on WebGL context loss", () => {
  const surface = fakeSurface();
  const webgl = fakeWebglRenderer();
  const failures = [];
  const renderer = createThreeArRenderer({
    surface,
    rendererFactory: () => webgl,
    textureFactory: fakeTexture,
    shadowTextureFactory: fakeTexture,
    onFatal: (reason) => failures.push(reason),
  });
  let prevented = false;

  surface.dispatch("webglcontextlost", { preventDefault() { prevented = true; } });

  assert.equal(prevented, true);
  assert.deepEqual(failures, ["WebGL context lost"]);
  assert.equal(renderer.status().failed, "WebGL context lost");
  assert.equal(renderer.status().contextLost, true);
  assert.equal(renderer.render(frame()), false);
  renderer.destroy();
});

test("Three renderer initialization errors are not replaced by Canvas2D", () => {
  assert.throws(
    () => createThreeArRenderer({
      surface: fakeSurface(),
      rendererFactory: () => { throw new Error("webgl2 unavailable"); },
    }),
    /webgl2 unavailable/,
  );
});
