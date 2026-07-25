import assert from "node:assert/strict";
import test from "node:test";

import {
  VISION_SESSION_DESIRED,
  VISION_SESSION_EFFECT,
  VISION_SESSION_EVENT,
  VISION_SESSION_NETWORK,
  VISION_SESSION_PAGE,
  createVisionLifecycleController,
} from "../src/features/drive/contents/vision/index.js";

function createManualScheduler() {
  const tasks = [];
  return {
    schedule(task) {
      tasks.push(task);
    },
    async flush() {
      while (tasks.length > 0) {
        const task = tasks.shift();
        task();
        await Promise.resolve();
      }
    },
    size() {
      return tasks.length;
    },
  };
}

function createHarness(overrides = {}) {
  const scheduler = createManualScheduler();
  const effects = [];
  const handlers = {};
  Object.values(VISION_SESSION_EFFECT).forEach((effect) => {
    handlers[effect] = async (context) => {
      effects.push({ effect, context });
      await overrides.handlers?.[effect]?.(context);
    };
  });
  const controller = createVisionLifecycleController({
    schedule: scheduler.schedule,
    handlers,
    initialState: overrides.initialState,
  });
  return { controller, effects, scheduler };
}

test("duplicate foreground events coalesce into one running effect", async () => {
  const { controller, effects, scheduler } = createHarness({
    initialState: {
      requested: true,
      page: VISION_SESSION_PAGE.HIDDEN,
    },
  });

  controller.updateInputs({ page: VISION_SESSION_PAGE.VISIBLE }, { reason: "visibility" });
  controller.updateInputs({ page: VISION_SESSION_PAGE.VISIBLE }, { reason: "pageshow" });
  controller.updateInputs({ page: VISION_SESSION_PAGE.VISIBLE }, { reason: "focus" });

  assert.equal(scheduler.size(), 1);
  await scheduler.flush();
  await controller.whenIdle();

  assert.deepEqual(effects.map((entry) => entry.effect), [VISION_SESSION_EFFECT.ENSURE_RUNNING]);
  assert.equal(controller.snapshot().desired, VISION_SESSION_DESIRED.RUNNING);
});

test("hidden and frozen signals suspend an active session once", async () => {
  const { controller, effects, scheduler } = createHarness({
    initialState: {
      requested: true,
      page: VISION_SESSION_PAGE.VISIBLE,
    },
  });

  controller.updateInputs({ page: VISION_SESSION_PAGE.HIDDEN }, { reason: "visibility hidden" });
  controller.updateInputs({ page: VISION_SESSION_PAGE.FROZEN }, { reason: "page freeze" });

  await scheduler.flush();
  await controller.whenIdle();

  assert.deepEqual(effects.map((entry) => entry.effect), [VISION_SESSION_EFFECT.ENSURE_SUSPENDED]);
  assert.equal(controller.snapshot().desired, VISION_SESSION_DESIRED.SUSPENDED);
});

test("latest desired state replaces an unstarted intermediate effect", async () => {
  const { controller, effects, scheduler } = createHarness({
    initialState: {
      requested: true,
      page: VISION_SESSION_PAGE.VISIBLE,
    },
  });

  controller.updateInputs({ page: VISION_SESSION_PAGE.HIDDEN }, { reason: "transient hidden" });
  controller.updateInputs({ page: VISION_SESSION_PAGE.VISIBLE }, { reason: "immediate return" });

  assert.equal(scheduler.size(), 1);
  await scheduler.flush();
  await controller.whenIdle();

  assert.deepEqual(effects.map((entry) => entry.effect), [VISION_SESSION_EFFECT.ENSURE_RUNNING]);
  assert.equal(controller.snapshot().desired, VISION_SESSION_DESIRED.RUNNING);
});

test("effects stay serialized when visibility changes during an active effect", async () => {
  let releaseRunning;
  const runningGate = new Promise((resolve) => {
    releaseRunning = resolve;
  });
  const { controller, effects, scheduler } = createHarness({
    handlers: {
      [VISION_SESSION_EFFECT.ENSURE_RUNNING]: async () => runningGate,
    },
  });

  controller.updateInputs({ requested: true }, { reason: "user start" });
  await scheduler.flush();
  assert.deepEqual(effects.map((entry) => entry.effect), [VISION_SESSION_EFFECT.ENSURE_RUNNING]);

  controller.updateInputs({ page: VISION_SESSION_PAGE.HIDDEN }, { reason: "backgrounded" });
  assert.equal(controller.status().pendingEffect, VISION_SESSION_EFFECT.ENSURE_SUSPENDED);

  releaseRunning();
  await controller.whenIdle();

  assert.deepEqual(effects.map((entry) => entry.effect), [
    VISION_SESSION_EFFECT.ENSURE_RUNNING,
    VISION_SESSION_EFFECT.ENSURE_SUSPENDED,
  ]);
  assert.equal(controller.status().processing, false);
});

test("user stop wins over source and page return signals", async () => {
  const { controller, effects, scheduler } = createHarness({
    initialState: {
      requested: true,
      page: VISION_SESSION_PAGE.HIDDEN,
      sourceAvailable: false,
    },
  });

  controller.updateInputs({
    requested: false,
    page: VISION_SESSION_PAGE.VISIBLE,
    sourceAvailable: true,
  }, { reason: "stopped while returning" });

  await scheduler.flush();
  await controller.whenIdle();

  assert.deepEqual(effects.map((entry) => entry.effect), [VISION_SESSION_EFFECT.ENSURE_STOPPED]);
  assert.equal(controller.snapshot().desired, VISION_SESSION_DESIRED.INACTIVE);
});

test("source and network gating preserve the user request", async () => {
  const { controller, effects, scheduler } = createHarness();

  controller.updateInputs({
    requested: true,
    sourceAvailable: false,
    network: VISION_SESSION_NETWORK.UNREACHABLE,
  }, { reason: "source and network unavailable" });
  await scheduler.flush();
  await controller.whenIdle();

  assert.equal(controller.snapshot().requested, true);
  assert.equal(controller.snapshot().desired, VISION_SESSION_DESIRED.WAITING);
  assert.deepEqual(effects.map((entry) => entry.effect), [VISION_SESSION_EFFECT.ENSURE_SUSPENDED]);

  controller.updateInputs({
    sourceAvailable: true,
    network: VISION_SESSION_NETWORK.REACHABLE,
  }, { reason: "source and network restored" });
  await scheduler.flush();
  await controller.whenIdle();

  assert.equal(controller.snapshot().desired, VISION_SESSION_DESIRED.RUNNING);
  assert.deepEqual(effects.map((entry) => entry.effect), [
    VISION_SESSION_EFFECT.ENSURE_SUSPENDED,
    VISION_SESSION_EFFECT.ENSURE_RUNNING,
  ]);
});

test("a restored frozen page requests one fresh session", async () => {
  const { controller, effects, scheduler } = createHarness({
    initialState: {
      requested: true,
      page: VISION_SESSION_PAGE.FROZEN,
    },
  });

  controller.dispatch(VISION_SESSION_EVENT.PAGE_RESTORED, { reason: "bfcache restore" });
  controller.dispatch(VISION_SESSION_EVENT.WINDOW_FOCUS, { reason: "duplicate focus" });
  await scheduler.flush();
  await controller.whenIdle();

  assert.deepEqual(effects.map((entry) => entry.effect), [VISION_SESSION_EFFECT.ENSURE_RUNNING]);
  assert.equal(effects[0].context.detail.freshSession, true);
  assert.equal(controller.snapshot().page, VISION_SESSION_PAGE.VISIBLE);
});
