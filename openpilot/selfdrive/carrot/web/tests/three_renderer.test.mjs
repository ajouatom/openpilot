import assert from "node:assert/strict";
import test from "node:test";

import { AR_MARKER_KIND } from "../src/features/drive/contents/vision/ar/tokens.js";
import { AR_TONE } from "../src/features/drive/contents/vision/ar/design_tokens.js";
import {
  describeSignboard,
  signboardFromMarker,
} from "../src/features/drive/contents/vision/ar/signboard.js";
import { pointOnPath, projectPoint } from "../src/features/drive/contents/vision/ar/projection.js";
import { createMarkerPresentationFilter } from "../src/features/drive/contents/vision/ar/presentation_filter.js";
import {
  createThreeArRenderer,
  markerScreenMetrics,
  stageUprightAnchor,
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

function frame({ laneWidthM = 3.5, nowMs = 0 } = {}) {
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
    stage: {
      calibTransform: [
        [960, -1000, 0],
        [540, 0, -1000],
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

function fakeTexture() {
  return { dispose() {} };
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

test("presentation filter snaps instead of dragging across a large lateral discontinuity", () => {
  const filter = createMarkerPresentationFilter();
  filter.update({ x: 40, y: 0, z: 0, headingRad: 0 }, 1, 0);
  const snapped = filter.update({ x: 39, y: 4, z: 0, headingRad: 0.4 }, 1.5, 50);

  assert.equal(snapped.anchor.y, 4);
  assert.equal(snapped.anchor.headingRad, 0.4);
  assert.equal(snapped.scale, 1.5);
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

test("stage-up correction keeps face above support on Carrot Vision mirrored axes", () => {
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
  const corrected = stageUprightAnchor(source, stage, 0);
  const projectStage = (point) => {
    const projected = projectPoint(stage.calibTransform, point.x, point.y, point.z);
    return {
      x: projected.x * stage.scale + stage.tx,
      y: projected.y * stage.scale + stage.ty,
    };
  };
  const base = projectStage(corrected);
  const top = projectStage({
    x: corrected.x + corrected.roadUp[0] * 4,
    y: corrected.y + corrected.roadUp[1] * 4,
    z: corrected.z + corrected.roadUp[2] * 4,
  });
  const right = projectStage({
    x: corrected.x + corrected.roadRight[0],
    y: corrected.y + corrected.roadRight[1],
    z: corrected.z + corrected.roadRight[2],
  });

  assert.equal(corrected.stageUpFlipped, true);
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

  assert.equal(renderer.render(frame({ nowMs: 0 })), true);
  assert.equal(webgl.renders, 1);
  assert.deepEqual(webgl.sizes, [[1920, 1080, false]]);
  assert.equal(renderer.status().backend, "three");
  assert.equal(renderer.status().drawn, 2);
  assert.equal(renderer.status().textureCount, 2);
  assert.equal(faceTextureCount, 2);

  // 같은 slot에서 geometry가 바뀌면 새 BAND로 원자 교체하고 이전 것은 남기지 않는다.
  assert.equal(renderer.render(frame({ laneWidthM: 4.0, nowMs: 50 })), true);
  assert.equal(renderer.status().textureCount, 2);
  assert.equal(faceTextureCount, 3);

  // 사용하지 않은 geometry는 한 프레임 누락에 폐기하지 않고 grace 뒤 정리한다.
  assert.equal(renderer.render(frame({ laneWidthM: 4.0, nowMs: 1_000 })), true);
  assert.equal(renderer.status().textureCount, 2);

  assert.equal(renderer.destroy(), true);
  assert.equal(webgl.disposed, true);
  assert.equal(webgl.contextReleased, true);
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
  const currentFrame = frame({ nowMs: 0 });
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
