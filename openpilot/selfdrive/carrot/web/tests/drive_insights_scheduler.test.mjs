import assert from "node:assert/strict";
import test from "node:test";

import {
  DRIVE_INSIGHTS_RENDER_CADENCE_MS,
  createDriveInsightsRenderScheduler,
} from "../src/features/drive/contents/drive_insights/scheduler.js";

function fakeClock() {
  let now = 0;
  let nextId = 0;
  const timers = new Map();
  return {
    nowMs: () => now,
    setNow(value) { now = value; },
    setTimeout(callback, delay) {
      const id = ++nextId;
      timers.set(id, { callback, delay, due: now + delay });
      return id;
    },
    clearTimeout(id) { timers.delete(id); },
    timers,
    runNext() {
      const next = [...timers.entries()].sort((left, right) => left[1].due - right[1].due)[0];
      assert.ok(next, "expected a scheduled timer");
      const [id, timer] = next;
      timers.delete(id);
      now = timer.due;
      timer.callback();
    },
  };
}

test("Drive Insights scheduler coalesces service bursts and enforces view cadence", () => {
  const clock = fakeClock();
  let flushes = 0;
  const scheduler = createDriveInsightsRenderScheduler({
    nowMs: clock.nowMs,
    setTimeout: clock.setTimeout,
    clearTimeout: clock.clearTimeout,
    onFlush() { flushes += 1; },
  });

  scheduler.request(DRIVE_INSIGHTS_RENDER_CADENCE_MS.graph);
  scheduler.request(DRIVE_INSIGHTS_RENDER_CADENCE_MS.graph);
  scheduler.request(DRIVE_INSIGHTS_RENDER_CADENCE_MS.graph);
  assert.equal(clock.timers.size, 1);
  clock.runNext();
  assert.equal(flushes, 1);

  clock.setNow(20);
  scheduler.request(DRIVE_INSIGHTS_RENDER_CADENCE_MS.graph);
  clock.setNow(40);
  scheduler.request(DRIVE_INSIGHTS_RENDER_CADENCE_MS.graph);
  assert.equal(clock.timers.size, 1);
  assert.equal([...clock.timers.values()][0].due, 100);
  clock.runNext();
  assert.equal(flushes, 2);

  clock.setNow(110);
  scheduler.request(DRIVE_INSIGHTS_RENDER_CADENCE_MS.forward);
  assert.equal([...clock.timers.values()][0].due, 167);
  clock.runNext();
  assert.equal(flushes, 3);
  assert.deepEqual(scheduler.status(), {
    pending: false,
    scheduled: false,
    cadenceMs: 67,
    requests: 6,
    flushes: 3,
    lastFlushAtMs: 167,
    lastFlushDurationMs: 0,
    maxFlushDurationMs: 0,
    slowFlushes: 0,
  });
});

test("Drive Insights scheduler cancellation prevents a stale paint", () => {
  const clock = fakeClock();
  let flushes = 0;
  const scheduler = createDriveInsightsRenderScheduler({
    nowMs: clock.nowMs,
    setTimeout: clock.setTimeout,
    clearTimeout: clock.clearTimeout,
    onFlush() { flushes += 1; },
  });
  scheduler.request(100);
  assert.equal(scheduler.cancel(), true);
  assert.equal(clock.timers.size, 0);
  assert.equal(flushes, 0);
});

test("Drive Insights yields an auxiliary paint to browser idle time", () => {
  const clock = fakeClock();
  let nextIdleId = 0;
  const idleCallbacks = new Map();
  let flushes = 0;
  const scheduler = createDriveInsightsRenderScheduler({
    nowMs: clock.nowMs,
    setTimeout: clock.setTimeout,
    clearTimeout: clock.clearTimeout,
    requestIdleCallback(callback, options) {
      const id = ++nextIdleId;
      idleCallbacks.set(id, { callback, options });
      return id;
    },
    cancelIdleCallback(id) { idleCallbacks.delete(id); },
    onFlush() { flushes += 1; },
  });

  scheduler.request(DRIVE_INSIGHTS_RENDER_CADENCE_MS.forward);
  clock.runNext();
  assert.equal(flushes, 0, "cadence timer must not paint inside its own task");
  assert.equal(idleCallbacks.size, 1);
  assert.equal([...idleCallbacks.values()][0].options.timeout, 12);

  const [idleId, idle] = [...idleCallbacks.entries()][0];
  idleCallbacks.delete(idleId);
  idle.callback({ didTimeout: false, timeRemaining: () => 8 });
  assert.equal(flushes, 1);

  scheduler.request(DRIVE_INSIGHTS_RENDER_CADENCE_MS.forward);
  clock.runNext();
  assert.equal(idleCallbacks.size, 1);
  assert.equal(scheduler.cancel(), true);
  assert.equal(idleCallbacks.size, 0);
  assert.equal(flushes, 1);
});

test("Drive Insights records slow auxiliary paints for on-device diagnosis", () => {
  const clock = fakeClock();
  const scheduler = createDriveInsightsRenderScheduler({
    nowMs: clock.nowMs,
    setTimeout: clock.setTimeout,
    clearTimeout: clock.clearTimeout,
    onFlush() { clock.setNow(clock.nowMs() + 9); },
  });

  scheduler.request(100);
  clock.runNext();
  assert.equal(scheduler.status().lastFlushDurationMs, 9);
  assert.equal(scheduler.status().maxFlushDurationMs, 9);
  assert.equal(scheduler.status().slowFlushes, 1);
});
