import assert from "node:assert/strict";
import test from "node:test";

import { createRenderPerformancePolicy } from "../src/features/drive/contents/vision/ar/performance_policy.js";

function observeMany(policy, count, workMs) {
  let status = policy.status();
  for (let i = 0; i < count; i += 1) status = policy.observe(workMs);
  return status;
}

test("isolated slow frames do not lower the Three AR cadence", () => {
  const policy = createRenderPerformancePolicy();
  for (let i = 0; i < 24; i += 1) policy.observe(i % 4 === 0 ? 36 : 18);

  assert.equal(policy.status().level, "target");
  assert.equal(policy.status().fps, 30);
  assert.equal(policy.status().failed, false);
});

test("sustained 30fps budget misses degrade once to 20fps", () => {
  const policy = createRenderPerformancePolicy();
  const status = observeMany(policy, 12, 36);

  assert.equal(status.level, "degraded");
  assert.equal(status.fps, 20);
  assert.equal(status.sampledFrames, 0);
  assert.equal(status.failed, false);

  assert.equal(observeMany(policy, 60, 18).fps, 20);
});

test("sustained 20fps budget misses stay degraded instead of killing AR", () => {
  const policy = createRenderPerformancePolicy();
  observeMany(policy, 12, 36);
  const status = observeMany(policy, 30, 55);

  assert.equal(status.level, "degraded");
  assert.equal(status.fps, 20);
  assert.equal(status.failed, false);
});
