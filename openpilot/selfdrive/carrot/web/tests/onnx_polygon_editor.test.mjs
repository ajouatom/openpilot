import assert from "node:assert/strict";
import test from "node:test";

import {
  canvasPointFromEvent,
  hasFreshOnnxVisionSnapshot,
  isPolygonConfigSavable,
  mirroredPolygon,
  movePolygonPoint,
  polygonCanvasGesture,
  polygonHandleScreenMetrics,
  polygonPointLabel,
} from "../src/features/settings/extensions/onnx_vision/components/polygon_editor.js";
import {
  fallbackRoadSceneMarkup,
  layoutFallbackRoadScene,
} from "../src/features/settings/extensions/onnx_vision/components/fallback_road_scene.js";
import { createOnnxSnapshotCache } from "../src/features/settings/extensions/onnx_vision/components/snapshot_cache.js";

const canvas = {
  getBoundingClientRect() {
    return { left: 10, top: 20, width: 200, height: 100 };
  },
};

test("ONNX polygon pointer coordinates scale to camera pixels", () => {
  assert.deepEqual(
    canvasPointFromEvent({ clientX: 110, clientY: 70 }, canvas, 1928, 1208),
    [964, 604],
  );
});

test("ONNX polygon pointer coordinates stay inside the camera frame", () => {
  assert.deepEqual(
    canvasPointFromEvent({ clientX: -100, clientY: 500 }, canvas, 1928, 1208),
    [0, 1207],
  );
});

test("ONNX polygon points can be nudged without leaving the image", () => {
  assert.deepEqual(movePolygonPoint([5, 8], -10, 7, 100, 50), [0, 15]);
  assert.deepEqual(movePolygonPoint([95, 48], 10, 7, 100, 50), [99, 49]);
});

test("ONNX canvas adds on empty taps and moves the selected point", () => {
  assert.equal(polygonCanvasGesture(), "add-point");
  assert.equal(polygonCanvasGesture({ hasSelectedPoint: true }), "move-selected-point");
});

test("ONNX polygon point labels are simple one-based numbers", () => {
  assert.equal(polygonPointLabel(0), "1");
  assert.equal(polygonPointLabel(3), "4");
});

test("ONNX polygon handles use bounded responsive screen-pixel targets", () => {
  assert.deepEqual(polygonHandleScreenMetrics(360), {
    dotRadius: 4.5,
    labelSize: 12,
    labelHeight: 20,
    labelWidth: 24,
    labelWideWidth: 30,
    labelGap: 5,
    labelCornerRadius: 5,
    hitRadius: 22,
    hitInset: 4,
    selectionGap: 3.5,
  });
  assert.equal(polygonHandleScreenMetrics(720).dotRadius, 5);
  assert.equal(polygonHandleScreenMetrics(720).labelSize, 13);
  assert.equal(polygonHandleScreenMetrics(0).hitRadius, 22);
});

test("ONNX polygon mirrors across the image and preserves winding", () => {
  assert.deepEqual(
    mirroredPolygon([[10, 1], [20, 2], [30, 3]], 100),
    [[69, 3], [79, 2], [89, 1]],
  );
});

test("ONNX polygon save validation accepts empty or complete sides only", () => {
  assert.equal(isPolygonConfigSavable({ poly_left: [], poly_right: [] }), false);
  assert.equal(isPolygonConfigSavable({ poly_left: [[1, 1]], poly_right: [[1, 1], [2, 2], [3, 3]] }), false);
  assert.equal(isPolygonConfigSavable({ poly_left: [], poly_right: [[1, 1], [2, 2], [3, 3]] }), true);
  assert.equal(isPolygonConfigSavable({
    poly_left: [[1, 1], [2, 2], [3, 3]],
    poly_right: [[4, 4], [5, 5], [6, 6]],
  }), true);
});

test("ONNX polygon save requires a newly received real camera snapshot", () => {
  assert.equal(hasFreshOnnxVisionSnapshot("ready"), true);
  assert.equal(hasFreshOnnxVisionSnapshot("cached"), false);
  assert.equal(hasFreshOnnxVisionSnapshot("fallback"), false);
  assert.equal(hasFreshOnnxVisionSnapshot("loading"), false);
});

test("ONNX snapshot cache keeps one JPEG in memory and persistent browser cache", async () => {
  const entries = new Map();
  const target = {
    location: { origin: "http://carrot.local" },
    Response,
    caches: {
      async open() {
        return {
          async put(key, response) { entries.set(key, response.clone()); },
          async match(key) { return entries.get(key)?.clone() || null; },
        };
      },
    },
  };
  const source = new Blob(["jpeg-frame"], { type: "image/jpeg" });
  const writer = createOnnxSnapshotCache();
  assert.equal(await writer.store(source, target), true);
  assert.equal(await writer.load({}), source);

  const reader = createOnnxSnapshotCache();
  const restored = await reader.load(target);
  assert.equal(restored.type, "image/jpeg");
  assert.equal(await restored.text(), "jpeg-frame");
  assert.equal(await reader.store(new Blob([], { type: "image/jpeg" }), target), false);
});

test("ONNX fallback scene lays out a simple perspective road", () => {
  const attributes = new Map();
  const nodes = new Map(["sky", "ground", "horizon", "road", "shoulders", "lanes"].map((name) => [name, {
    setAttribute(attribute, value) { attributes.set(`${name}.${attribute}`, value); },
  }]));
  const root = {
    setAttribute(attribute, value) { attributes.set(`root.${attribute}`, value); },
    querySelector(selector) {
      return nodes.get(selector.match(/data-fallback-part="([^"]+)"/)?.[1]) || null;
    },
  };
  layoutFallbackRoadScene(root, 800, 450);
  assert.match(fallbackRoadSceneMarkup(), /<svg[^>]+preserveAspectRatio="none"/);
  assert.match(fallbackRoadSceneMarkup(), /onnx-fallback-road__road/);
  assert.equal(attributes.get("root.viewBox"), "0 0 800 450");
  assert.equal(attributes.get("sky.width"), "800");
  assert.equal(attributes.get("sky.height"), "0");
  assert.equal(attributes.get("ground.height"), "450");
  assert.match(attributes.get("road.d"), /M 356 0 L 444 0 L 800 450 L 0 450/);
  assert.match(attributes.get("lanes.d"), /M 400 0 L 400 450/);
});
