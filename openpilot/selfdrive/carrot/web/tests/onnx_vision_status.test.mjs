import assert from "node:assert/strict";
import test from "node:test";

import {
  ONNX_VISION_STATE,
  normalizeOnnxVisionStatus,
} from "../src/features/settings/extensions/onnx_vision/model.js";

test("ONNX status stays quiet while the existing ShareData toggle is off", () => {
  const status = normalizeOnnxVisionStatus({ model: { loaded: true } }, { enabled: false });
  assert.equal(status.state, ONNX_VISION_STATE.DISABLED);
  assert.equal(status.service, null);
});

test("ONNX status presents ready models, lanes, BSD gate, and performance", () => {
  const status = normalizeOnnxVisionStatus({
    model: { loaded: true },
    camera: { available: true },
    configuredSides: ["left", "right"],
    gate: { active: true, side: "left" },
    inference: { fps: 3.4, latencyMs: 31.2 },
    lane: {
      loaded: true,
      cameraAvailable: true,
      resultFresh: true,
      result: { leftLine: 1, rightLine: 0 },
      inference: { fps: 2.5, latencyMs: 42.8 },
    },
  }, { enabled: true });

  assert.equal(status.state, ONNX_VISION_STATE.READY);
  assert.equal(status.lane.left.line, "solid");
  assert.equal(status.lane.right.line, "dashed");
  assert.equal(status.lane.fps, 2.5);
  assert.equal(status.bsd.active, true);
  assert.equal(status.bsd.side, "left");
  assert.equal(status.bsd.configured, true);
});

test("ONNX status distinguishes startup, unavailable service, and a camera wait", () => {
  const starting = normalizeOnnxVisionStatus({}, { enabled: true, loading: true });
  const unavailable = normalizeOnnxVisionStatus({}, { enabled: true, error: new Error("unavailable") });
  const waiting = normalizeOnnxVisionStatus({
    model: { loaded: true },
    camera: { available: false },
    lane: { loaded: true, cameraAvailable: false },
  }, { enabled: true });

  assert.equal(starting.state, ONNX_VISION_STATE.STARTING);
  assert.equal(unavailable.state, ONNX_VISION_STATE.UNAVAILABLE);
  assert.equal(waiting.state, ONNX_VISION_STATE.WAITING);
});

test("ONNX status preserves confidence, gate, camera, and diagnostic values", () => {
  const status = normalizeOnnxVisionStatus({
    model: { loaded: true, path: "/model.onnx" },
    camera: { available: true, lastFrameAgeSeconds: 0.2 },
    configuredSides: ["right"],
    gate: { active: false, side: "right", reason: "target lane width below 3.0 m", laneWidth: 2.8 },
    vehicleSide: { right: { valid: true, active: true, confidence: 0.91 } },
    inference: { fps: 4, latencyMs: 21, threadCpuMs: 18, count: 12 },
    lane: {
      loaded: true,
      cameraAvailable: true,
      resultFresh: true,
      result: { leftLine: 0, rightLine: 1, leftConf: 0.77, rightConf: 0.83, candidatesCount: 4 },
      inference: { fps: 2.5, latencyMs: 43, threadCpuMs: 39, count: 8 },
    },
  }, { enabled: true });

  assert.equal(status.lane.left.confidence, 0.77);
  assert.equal(status.lane.candidates, 4);
  assert.equal(status.bsd.right.confidence, 0.91);
  assert.equal(status.bsd.gateReason, "target lane width below 3.0 m");
  assert.equal(status.service.bsdModelPath, "/model.onnx");
});
