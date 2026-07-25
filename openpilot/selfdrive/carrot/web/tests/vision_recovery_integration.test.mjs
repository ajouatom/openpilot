import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

import {
  VISION_SESSION_DESIRED,
  VISION_SESSION_EFFECT,
  VISION_SESSION_NETWORK,
  VISION_SESSION_PAGE,
  createVisionLifecycleController,
  createVisionNetworkRecoveryController,
} from "../src/features/drive/contents/vision/index.js";

const appRealtimeSource = await readFile(
  new URL("../js/realtime/app_realtime.js", import.meta.url),
  "utf8",
);
const visionRtcSource = await readFile(
  new URL("../js/realtime/vision_rtc.js", import.meta.url),
  "utf8",
);

function createTaskScheduler() {
  const tasks = [];
  return {
    schedule(task) {
      tasks.push(task);
    },
    async flush() {
      while (tasks.length) {
        tasks.shift()();
        await Promise.resolve();
      }
    },
    size: () => tasks.length,
  };
}

function createTimerScheduler() {
  let nextId = 1;
  const timers = new Map();
  return {
    schedule(task, delayMs) {
      const id = nextId++;
      timers.set(id, { task, delayMs });
      return id;
    },
    cancel(id) {
      timers.delete(id);
    },
    runNext() {
      const entry = timers.entries().next().value;
      if (!entry) return false;
      const [id, timer] = entry;
      timers.delete(id);
      timer.task();
      return true;
    },
    size: () => timers.size,
  };
}

async function flushMicrotasks() {
  await Promise.resolve();
  await Promise.resolve();
}

function createIntegratedHarness(options = {}) {
  const lifecycleTasks = createTaskScheduler();
  const networkTimers = createTimerScheduler();
  const effects = [];
  let serverReachable = options.serverReachable !== false;
  let probeCount = 0;
  let requested = false;
  let page = VISION_SESSION_PAGE.VISIBLE;
  let sourceAvailable = true;

  const lifecycle = createVisionLifecycleController({
    schedule: lifecycleTasks.schedule,
    handlers: Object.fromEntries(
      Object.values(VISION_SESSION_EFFECT).map((effect) => [
        effect,
        async (context) => {
          effects.push({ effect, context });
          await options.handlers?.[effect]?.(context);
        },
      ]),
    ),
  });

  const network = createVisionNetworkRecoveryController({
    probe: async () => {
      probeCount += 1;
      if (!serverReachable) throw new Error("local origin unreachable");
      return { ok: true };
    },
    schedule: networkTimers.schedule,
    cancel: networkTimers.cancel,
    backoffMs: [300, 600, 1200, 2400, 4000],
    probeTimeoutMs: 0,
    onState(status) {
      if (!status.enabled || status.reachable === null) return;
      lifecycle.updateInputs({
        network: status.reachable
          ? VISION_SESSION_NETWORK.REACHABLE
          : VISION_SESSION_NETWORK.UNREACHABLE,
      }, { reason: `network ${status.state}` });
    },
  });

  function sync(next = {}) {
    if (Object.hasOwn(next, "requested")) requested = Boolean(next.requested);
    if (Object.hasOwn(next, "page")) page = next.page;
    if (Object.hasOwn(next, "sourceAvailable")) sourceAvailable = next.sourceAvailable !== false;
    const networkEnabled = requested && sourceAvailable && page === VISION_SESSION_PAGE.VISIBLE;
    network.setEnabled(networkEnabled, { reason: next.reason || "integration sync" });
    const networkStatus = network.snapshot();
    lifecycle.updateInputs({
      requested,
      page,
      sourceAvailable,
      ...(networkEnabled ? {
        network: networkStatus.reachable
          ? VISION_SESSION_NETWORK.REACHABLE
          : VISION_SESSION_NETWORK.UNREACHABLE,
      } : {}),
    }, { reason: next.reason || "integration sync" });
  }

  async function settle() {
    await flushMicrotasks();
    await lifecycleTasks.flush();
    await lifecycle.whenIdle();
  }

  return {
    lifecycle,
    network,
    effects,
    lifecycleTasks,
    networkTimers,
    sync,
    settle,
    setServerReachable(value) {
      serverReachable = Boolean(value);
    },
    probeCount: () => probeCount,
  };
}

test("classic runtime wiring probes only the current local origin and treats browser state as a hint", () => {
  assert.match(appRealtimeSource, /new URL\("\/index\.html", window\.location\.href\)/);
  assert.doesNotMatch(appRealtimeSource, /\/api\/heartbeat_status/);
  assert.match(appRealtimeSource, /noteBrowserHint\(false, "browser offline hint"\)/);
  assert.match(appRealtimeSource, /noteBrowserHint\(true, "browser online hint"\)/);
  assert.match(appRealtimeSource, /window\.CarrotVisionNetworkRecovery = _carrotVisionNetworkRecoveryController/);
  assert.match(visionRtcSource, /CarrotVisionNetworkRecovery\?\.reportTransportFailure/);
});

test("5 second and 60 second background stays use the same suspend and fresh resume path", async () => {
  const harness = createIntegratedHarness();
  harness.sync({ requested: true, reason: "user start" });
  await harness.settle();
  harness.effects.length = 0;

  for (const durationMs of [5000, 60000]) {
    harness.sync({ page: VISION_SESSION_PAGE.HIDDEN, reason: `background ${durationMs}` });
    await harness.settle();
    assert.equal(harness.network.snapshot().enabled, false);
    assert.equal(harness.lifecycle.snapshot().desired, VISION_SESSION_DESIRED.SUSPENDED);

    harness.sync({ page: VISION_SESSION_PAGE.VISIBLE, reason: `return ${durationMs}` });
    await harness.settle();
    assert.equal(harness.network.snapshot().reachable, true);
    assert.equal(harness.lifecycle.snapshot().desired, VISION_SESSION_DESIRED.RUNNING);
  }

  assert.deepEqual(harness.effects.map((entry) => entry.effect), [
    VISION_SESSION_EFFECT.ENSURE_SUSPENDED,
    VISION_SESSION_EFFECT.ENSURE_RUNNING,
    VISION_SESSION_EFFECT.ENSURE_SUSPENDED,
    VISION_SESSION_EFFECT.ENSURE_RUNNING,
  ]);
});

test("backgrounding during connection setup serializes cleanup behind the active attempt", async () => {
  let releaseRunning;
  const runningGate = new Promise((resolve) => {
    releaseRunning = resolve;
  });
  const harness = createIntegratedHarness({
    handlers: {
      [VISION_SESSION_EFFECT.ENSURE_RUNNING]: async () => runningGate,
    },
  });

  harness.sync({ requested: true, reason: "connect begins" });
  await flushMicrotasks();
  await harness.lifecycleTasks.flush();
  assert.deepEqual(harness.effects.map((entry) => entry.effect), [
    VISION_SESSION_EFFECT.ENSURE_RUNNING,
  ]);

  harness.sync({ page: VISION_SESSION_PAGE.HIDDEN, reason: "app backgrounded during connect" });
  assert.equal(harness.network.snapshot().enabled, false);
  assert.equal(harness.lifecycle.status().pendingEffect, VISION_SESSION_EFFECT.ENSURE_SUSPENDED);

  releaseRunning();
  await harness.lifecycle.whenIdle();
  assert.deepEqual(harness.effects.map((entry) => entry.effect), [
    VISION_SESSION_EFFECT.ENSURE_RUNNING,
    VISION_SESSION_EFFECT.ENSURE_SUSPENDED,
  ]);
});

test("Wi-Fi or server loss suspends once and the single retry queue resumes once", async () => {
  const harness = createIntegratedHarness();
  harness.sync({ requested: true, reason: "user start" });
  await harness.settle();
  harness.effects.length = 0;

  harness.setServerReachable(false);
  await harness.network.noteBrowserHint(false, "wifi lost");
  await harness.settle();
  assert.equal(harness.lifecycle.snapshot().desired, VISION_SESSION_DESIRED.WAITING);
  assert.equal(harness.networkTimers.size(), 1);

  harness.setServerReachable(true);
  assert.equal(harness.networkTimers.runNext(), true);
  await harness.settle();
  assert.equal(harness.lifecycle.snapshot().desired, VISION_SESSION_DESIRED.RUNNING);
  assert.deepEqual(harness.effects.map((entry) => entry.effect), [
    VISION_SESSION_EFFECT.ENSURE_SUSPENDED,
    VISION_SESSION_EFFECT.ENSURE_RUNNING,
  ]);
});

test("repeated online and foreground signals cannot create parallel reconnects", async () => {
  const harness = createIntegratedHarness({ serverReachable: false });
  harness.sync({ requested: true, reason: "user start" });
  await harness.settle();
  const probesBefore = harness.probeCount();
  harness.setServerReachable(true);

  const checks = [
    harness.network.noteBrowserHint(true, "online"),
    harness.network.noteBrowserHint(true, "pageshow"),
    harness.network.noteBrowserHint(true, "focus"),
  ];
  await Promise.all(checks);
  await harness.settle();

  assert.equal(harness.probeCount(), probesBefore + 1);
  assert.equal(harness.lifecycle.snapshot().desired, VISION_SESSION_DESIRED.RUNNING);
  assert.equal(
    harness.effects.filter((entry) => entry.effect === VISION_SESSION_EFFECT.ENSURE_RUNNING).length,
    1,
  );
});

test("user stop remains authoritative across page and network recovery signals", async () => {
  const harness = createIntegratedHarness();
  harness.sync({ requested: true, reason: "user start" });
  await harness.settle();
  harness.effects.length = 0;

  harness.sync({ requested: false, reason: "user stop" });
  await harness.settle();
  await harness.network.noteBrowserHint(true, "late online");
  harness.sync({ page: VISION_SESSION_PAGE.HIDDEN, reason: "late pagehide" });
  harness.sync({ page: VISION_SESSION_PAGE.VISIBLE, reason: "late pageshow" });
  await harness.settle();

  assert.equal(harness.network.snapshot().enabled, false);
  assert.equal(harness.lifecycle.snapshot().requested, false);
  assert.equal(harness.lifecycle.snapshot().desired, VISION_SESSION_DESIRED.INACTIVE);
  assert.deepEqual(harness.effects.map((entry) => entry.effect), [
    VISION_SESSION_EFFECT.ENSURE_STOPPED,
  ]);
});
