import assert from "node:assert/strict";
import test from "node:test";

import {
  createRouteSummaryProgressEmitter,
  ROUTE_SUMMARY_PROGRESS_INTERVAL_MS,
} from "../src/features/logs/route_summary/progress.js";

test("route summary progress coalesces chunk updates to the configured cadence", () => {
  let timestamp = 0;
  const sent = [];
  const progress = createRouteSummaryProgressEmitter((payload) => sent.push(payload), {
    now: () => timestamp,
  });

  assert.equal(ROUTE_SUMMARY_PROGRESS_INTERVAL_MS, 150);
  assert.equal(progress.update({ loadedBytes: 10 }), true);
  timestamp = 25;
  assert.equal(progress.update({ loadedBytes: 20 }), false);
  timestamp = 149;
  assert.equal(progress.update({ loadedBytes: 30 }), false);
  timestamp = 150;
  assert.equal(progress.update({ loadedBytes: 40 }), true);
  assert.deepEqual(sent, [{ loadedBytes: 10 }, { loadedBytes: 40 }]);
});

test("route summary progress flush delivers the latest pending and final states", () => {
  let timestamp = 0;
  const sent = [];
  const progress = createRouteSummaryProgressEmitter((payload) => sent.push(payload), {
    now: () => timestamp,
  });

  progress.update({ completed: 0, total: 2, loadedBytes: 10 });
  timestamp = 20;
  progress.update({ completed: 0, total: 2, loadedBytes: 30 });
  timestamp = 30;
  assert.equal(progress.flush(), true);
  timestamp = 35;
  assert.equal(progress.flush({ completed: 1, total: 2, loadedBytes: 50 }), true);
  assert.equal(progress.flush(), false);

  assert.deepEqual(sent, [
    { completed: 0, total: 2, loadedBytes: 10 },
    { completed: 0, total: 2, loadedBytes: 30 },
    { completed: 1, total: 2, loadedBytes: 50 },
  ]);
});

test("route summary progress interval remains inside the worker policy range", () => {
  let timestamp = 0;
  const fast = [];
  const slow = [];
  const fastProgress = createRouteSummaryProgressEmitter((payload) => fast.push(payload), {
    intervalMs: 1,
    now: () => timestamp,
  });
  const slowProgress = createRouteSummaryProgressEmitter((payload) => slow.push(payload), {
    intervalMs: 1_000,
    now: () => timestamp,
  });

  fastProgress.update(1);
  slowProgress.update(1);
  timestamp = 99;
  fastProgress.update(2);
  timestamp = 100;
  fastProgress.update(3);
  timestamp = 199;
  slowProgress.update(2);
  timestamp = 200;
  slowProgress.update(3);

  assert.deepEqual(fast, [1, 3]);
  assert.deepEqual(slow, [1, 3]);
});
