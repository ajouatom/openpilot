import assert from "node:assert/strict";
import test from "node:test";

import { AR_EMPHASIS } from "../src/features/drive/contents/vision/ar/design_tokens.js";
import {
  distanceEmphasisScale,
  planMarker,
  resolveWorldScale,
  selectVisibleMarkers,
} from "../src/features/drive/contents/vision/ar/responsive.js";

test("wide upright markers are clamped by viewport width as well as height", () => {
  const result = resolveWorldScale({
    worldHeightM: 2.56,
    worldWidthM: 5.12,
    distanceM: 40,
    focalPx: 2648,
    canvas: { width: 500, height: 720 },
  });

  assert.equal(result.clamped, "width");
  assert.ok(result.projectedWidthPx <= result.maxWidthPx + 1e-9);
  assert.ok(result.projectedPx < result.rawPx);
});

test("a marker near the viewport edge receives a stronger bounded width clamp", () => {
  const centered = resolveWorldScale({
    worldHeightM: 2.56,
    worldWidthM: 5.12,
    distanceM: 40,
    focalPx: 2648,
    centerX: 250,
    canvas: { width: 500, height: 720 },
  });
  const nearEdge = resolveWorldScale({
    worldHeightM: 2.56,
    worldWidthM: 5.12,
    distanceM: 40,
    focalPx: 2648,
    centerX: 450,
    canvas: { width: 500, height: 720 },
  });

  assert.equal(nearEdge.clamped, "width-edge");
  assert.ok(nearEdge.scale < centered.scale);
  assert.ok(nearEdge.projectedWidthPx >= 112);
});

test("preview phase geometry is normalized to one physical world size", () => {
  const speedMps = 16.7;
  assert.equal(distanceEmphasisScale(300, speedMps), AR_EMPHASIS.preview.scale);
  assert.equal(distanceEmphasisScale(150, speedMps), AR_EMPHASIS.approach.scale);
  assert.equal(distanceEmphasisScale(70, speedMps), AR_EMPHASIS.precise.scale);
  assert.equal(distanceEmphasisScale(15, speedMps), AR_EMPHASIS.commit.scale);

  const baseHeightM = 2.56;
  const focalPx = 2648;
  const canvas = { width: 1920, height: 1080 };
  const plan = (distanceM, descriptorScale) => planMarker({
    distanceM,
    egoSpeedMps: speedMps,
    worldHeightM: baseHeightM * descriptorScale,
    worldWidthM: baseHeightM * 2 * descriptorScale,
    descriptorScale,
    focalPx,
    projectedHeightPx: baseHeightM * descriptorScale * focalPx / distanceM,
    projectedWidthPx: baseHeightM * 2 * descriptorScale * focalPx / distanceM,
    canvas,
  });
  const justBefore = plan(150.001, AR_EMPHASIS.preview.scale);
  const justAfter = plan(149.999, AR_EMPHASIS.approach.scale);

  assert.ok(Math.abs(justBefore.projectedPx - justAfter.projectedPx) < 0.01);
  assert.ok(Math.abs(justBefore.projectedWidthPx - justAfter.projectedWidthPx) < 0.02);
  assert.notEqual(justBefore.geometryScale, justAfter.geometryScale);
  assert.equal(justBefore.clamped, "none");
  assert.equal(justAfter.clamped, "none");
});

test("a nearby marker grows with perspective instead of shrinking into a HUD card", () => {
  const descriptorScale = AR_EMPHASIS.commit.scale;
  const plan = planMarker({
    distanceM: 12,
    egoSpeedMps: 10,
    worldHeightM: 2.56 * descriptorScale,
    worldWidthM: 5.12 * descriptorScale,
    descriptorScale,
    focalPx: 2648,
    projectedHeightPx: 2.56 * descriptorScale * 2648 / 12,
    projectedWidthPx: 5.12 * descriptorScale * 2648 / 12,
    canvas: { width: 1920, height: 1080 },
  });

  assert.equal(plan.clamped, "world-fixed");
  assert.equal(plan.visible, true);
  assert.ok(plan.projectedPx > 500);
  assert.ok(Math.abs(plan.worldScale - 1 / descriptorScale) < 1e-12);
});

test("an upright marker may use a bounded near-view scale instead of disappearing", () => {
  const descriptorScale = AR_EMPHASIS.commit.scale;
  const plan = planMarker({
    distanceM: 10,
    egoSpeedMps: 10,
    worldHeightM: 2.56 * descriptorScale,
    worldWidthM: 5.12 * descriptorScale,
    descriptorScale,
    focalPx: 2648,
    projectedHeightPx: 2.56 * descriptorScale * 2648 / 10,
    projectedWidthPx: 5.12 * descriptorScale * 2648 / 10,
    minimumWorldScale: 0.62,
    canvas: { width: 1920, height: 1080 },
  });

  assert.equal(plan.visible, true);
  assert.ok(plan.worldScale < 1 / descriptorScale);
  assert.ok(plan.projectedWidthPx <= 1920 * 0.9 + 1e-9);
});

test("a 200m preview stays legible while its base remains on the world anchor", () => {
  const descriptorScale = AR_EMPHASIS.preview.scale;
  const baseHeightM = 2.56;
  const focalPx = 2648;
  const distanceM = 200;
  const plan = planMarker({
    distanceM,
    egoSpeedMps: 16.7,
    worldHeightM: baseHeightM * descriptorScale,
    worldWidthM: baseHeightM * 2 * descriptorScale,
    descriptorScale,
    focalPx,
    projectedHeightPx: baseHeightM * descriptorScale * focalPx / distanceM,
    projectedWidthPx: baseHeightM * 2 * descriptorScale * focalPx / distanceM,
    canvas: { width: 1920, height: 1080 },
  });

  assert.equal(plan.visible, true);
  assert.equal(plan.phase, "preview");
  assert.ok(plan.projectedPx >= 30);
  assert.ok(plan.worldScale > 1);
});

test("measured 3D projection overrides the flat focal-distance approximation", () => {
  const result = resolveWorldScale({
    worldHeightM: 2.56,
    worldWidthM: 5.12,
    distanceM: 40,
    focalPx: 2648,
    rawHeightPx: 80,
    rawWidthPx: 160,
    canvas: { width: 1920, height: 1080 },
  });

  assert.equal(result.rawPx, 80);
  assert.equal(result.rawWidthPx, 160);
});

test("near marker overflow is detected after an edge-width clamp overwrites the height clamp", () => {
  const plan = planMarker({
    distanceM: 5,
    egoSpeedMps: 20,
    worldHeightM: 2.56,
    worldWidthM: 5.12,
    focalPx: 2648,
    centerX: 450,
    projectedHeightPx: 2_000,
    projectedWidthPx: 4_000,
    canvas: { width: 500, height: 720 },
  });

  assert.equal(plan.clamped, "world-fixed");
  assert.equal(plan.overflow, true);
  assert.equal(plan.visible, false);
});

test("overlapping markers keep the higher-priority semantic event at its world position", () => {
  const candidate = (key, source, bounds, distanceM = 40) => ({
    key,
    item: { source },
    distanceM,
    plan: { visible: true, phase: "precise" },
    screenBounds: bounds,
  });
  const selected = selectVisibleMarkers([
    candidate("next", "guidanceNext", { left: 500, right: 700, top: 300, bottom: 450 }, 35),
    candidate("primary", "guidanceCurrent", { left: 520, right: 720, top: 310, bottom: 460 }, 45),
    candidate("camera", "sdi", { left: 900, right: 1040, top: 300, bottom: 440 }, 60),
  ], { width: 1920, height: 1080 });

  assert.deepEqual(selected.map((item) => item.key), ["primary", "camera"]);
});

test("overlap selection keeps the previous equal-priority marker across distance jitter", () => {
  const candidate = (key, distanceM) => ({
    key,
    item: { source: "guidanceCurrent" },
    distanceM,
    plan: { visible: true, phase: "precise" },
    screenBounds: { left: 500, right: 700, top: 300, bottom: 450 },
  });
  const selected = selectVisibleMarkers([
    candidate("stable", 40.2),
    candidate("jitter", 40),
  ], { width: 1920, height: 1080 }, {
    preferredKeys: new Set(["stable"]),
  });

  assert.deepEqual(selected.map((item) => item.key), ["stable"]);
});

test("semantic priority still overrides an overlap preference", () => {
  const candidate = (key, source) => ({
    key,
    item: { source },
    distanceM: 40,
    plan: { visible: true, phase: "precise" },
    screenBounds: { left: 500, right: 700, top: 300, bottom: 450 },
  });
  const selected = selectVisibleMarkers([
    candidate("next", "guidanceNext"),
    candidate("current", "guidanceCurrent"),
  ], { width: 1920, height: 1080 }, {
    preferredKeys: new Set(["next"]),
  });

  assert.deepEqual(selected.map((item) => item.key), ["current"]);
});

test("distant markers use the preview LOD before the 300m visibility boundary", () => {
  const plan = planMarker({
    distanceM: 280,
    egoSpeedMps: 16.7,
    worldHeightM: 2,
    worldWidthM: 4,
    focalPx: 2648,
    canvas: { width: 1920, height: 1080 },
  });

  assert.equal(plan.visible, true);
  assert.equal(plan.phase, "preview");
  assert.equal(plan.lod, "flat");
  assert.equal(plan.chevronCount, 0);
});
