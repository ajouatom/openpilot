import assert from "node:assert/strict";
import test from "node:test";

import { createReplayRenderController } from "../src/features/drive/contents/vision/replay_render_controller.js";

function fixture(overrides = {}) {
  const calls = [];
  const controller = createReplayRenderController({
    isReplayActive: () => true,
    isStageVisible: () => true,
    cancelScheduledRender: () => calls.push("cancel"),
    resetFrameTemporalState: () => calls.push("reset-frame"),
    resetReplayState: () => calls.push("reset-replay"),
    mergePendingRenderState: (value) => calls.push(["merge", value]),
    flushScheduledRender: () => calls.push("flush"),
    notifyPresentedFrame: (value) => calls.push(["present", value]),
    ...overrides,
  });
  return { calls, controller };
}

test("replay presents only after the exact frame has flushed", () => {
  const { calls, controller } = fixture();
  assert.equal(controller.renderVideoFrame({
    force: true,
    resetTemporal: true,
    mediaTime: 2.25,
    reason: "seek",
  }), true);
  assert.deepEqual(calls, [
    "cancel",
    "reset-frame",
    ["merge", { force: true, overlayDirty: true, hudDirty: true }],
    "flush",
    ["present", {
      source: "replay",
      mediaTime: 2.25,
      reason: "seek",
      discontinuity: true,
      discontinuityReason: "replay-seek",
    }],
  ]);
});

test("ordinary replay frames do not announce a timeline discontinuity", () => {
  const { calls, controller } = fixture();
  controller.renderVideoFrame({ mediaTime: 2.5, reason: "replay video frame" });
  assert.deepEqual(calls.at(-1), ["present", {
    source: "replay",
    mediaTime: 2.5,
    reason: "replay video frame",
    discontinuity: false,
    discontinuityReason: null,
  }]);
});

test("presentation-only replay frames notify AR without repainting unchanged 2D overlays", () => {
  const { calls, controller } = fixture();
  controller.renderVideoFrame({
    mediaTime: 2.55,
    reason: "replay video frame",
    overlayDirty: false,
    hudDirty: false,
  });
  assert.deepEqual(calls, [["present", {
    source: "replay",
    mediaTime: 2.55,
    reason: "replay video frame",
    discontinuity: false,
    discontinuityReason: null,
  }]]);
});

test("replay does not publish a frame while its surface is inactive", () => {
  const { calls, controller } = fixture({ isReplayActive: () => false });
  assert.equal(controller.renderVideoFrame(), false);
  assert.deepEqual(calls, []);
});

test("replay controller requires a presented-frame bridge", () => {
  const { controller } = fixture({ notifyPresentedFrame: null });
  assert.equal(controller, null);
});
