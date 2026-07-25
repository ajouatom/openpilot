import assert from "node:assert/strict";
import test from "node:test";

import {
  VISION_NETWORK_RECOVERY_STATE,
  createVisionNetworkRecoveryController,
} from "../src/features/drive/contents/vision/index.js";

function createScheduler() {
  let nextId = 1;
  const queue = new Map();
  return {
    schedule(task, delayMs) {
      const id = nextId++;
      queue.set(id, { task, delayMs });
      return id;
    },
    cancel(id) {
      queue.delete(id);
    },
    runNext() {
      const entry = queue.entries().next().value;
      if (!entry) return false;
      const [id, item] = entry;
      queue.delete(id);
      item.task();
      return true;
    },
    items() {
      return Array.from(queue.values());
    },
  };
}

async function flush() {
  await Promise.resolve();
  await Promise.resolve();
}

test("initial server check gates recovery until the current origin responds", async () => {
  const scheduler = createScheduler();
  const states = [];
  const controller = createVisionNetworkRecoveryController({
    probe: async () => ({ ok: true }),
    schedule: scheduler.schedule,
    cancel: scheduler.cancel,
    onState: (state) => states.push(state),
  });

  controller.setEnabled(true, { reason: "vision requested" });
  assert.equal(controller.snapshot().state, VISION_NETWORK_RECOVERY_STATE.CHECKING);

  await flush();
  assert.equal(controller.snapshot().state, VISION_NETWORK_RECOVERY_STATE.REACHABLE);
  assert.equal(controller.snapshot().reachable, true);
  assert.equal(controller.snapshot().originClassification, "current-origin-reachable");
  assert.deepEqual(states.map((state) => state.state), ["checking", "reachable"]);
});

test("repeated online hints share one in-flight probe", async () => {
  const scheduler = createScheduler();
  let resolveProbe;
  let probeCount = 0;
  const controller = createVisionNetworkRecoveryController({
    probe: () => {
      probeCount += 1;
      return new Promise((resolve) => {
        resolveProbe = resolve;
      });
    },
    schedule: scheduler.schedule,
    cancel: scheduler.cancel,
  });

  controller.setEnabled(true);
  controller.noteBrowserHint(true, "online 1");
  controller.noteBrowserHint(true, "online 2");
  assert.equal(probeCount, 1);
  assert.equal(controller.snapshot().inFlight, true);

  resolveProbe({ ok: true });
  await flush();
  assert.equal(controller.snapshot().reachable, true);
});

test("failed probes keep exactly one bounded backoff check queued", async () => {
  const scheduler = createScheduler();
  let probeCount = 0;
  const controller = createVisionNetworkRecoveryController({
    probe: async () => {
      probeCount += 1;
      throw new Error("unreachable");
    },
    schedule: scheduler.schedule,
    cancel: scheduler.cancel,
    backoffMs: [400, 800],
    probeTimeoutMs: 0,
  });

  controller.setEnabled(true);
  await flush();
  assert.equal(controller.snapshot().state, VISION_NETWORK_RECOVERY_STATE.WAITING);
  assert.equal(controller.snapshot().queueScheduled, true);
  assert.equal(scheduler.items().length, 1);
  assert.equal(scheduler.items()[0].delayMs, 400);

  controller.noteBrowserHint(false, "offline hint");
  await flush();
  assert.equal(probeCount, 2);
  assert.equal(scheduler.items().length, 1);
  assert.equal(scheduler.items()[0].delayMs, 800);
});

test("same-origin recovery resumes automatically from the queued check", async () => {
  const scheduler = createScheduler();
  let available = false;
  const controller = createVisionNetworkRecoveryController({
    probe: async () => {
      if (!available) throw new Error("route unavailable");
      return { ok: true };
    },
    schedule: scheduler.schedule,
    cancel: scheduler.cancel,
    backoffMs: [250],
    probeTimeoutMs: 0,
  });

  controller.setEnabled(true);
  await flush();
  assert.equal(controller.snapshot().reachable, false);

  available = true;
  assert.equal(scheduler.runNext(), true);
  await flush();
  assert.equal(controller.snapshot().state, VISION_NETWORK_RECOVERY_STATE.REACHABLE);
  assert.equal(controller.snapshot().failureCount, 0);
});

test("browser online state is advisory and cannot mark the server reachable", async () => {
  const scheduler = createScheduler();
  const controller = createVisionNetworkRecoveryController({
    probe: async () => {
      throw new Error("server still unreachable");
    },
    schedule: scheduler.schedule,
    cancel: scheduler.cancel,
    probeTimeoutMs: 0,
  });

  controller.setEnabled(true);
  await flush();
  await controller.noteBrowserHint(true, "browser online");

  assert.equal(controller.snapshot().browserOnlineHint, true);
  assert.equal(controller.snapshot().reachable, false);
  assert.equal(controller.snapshot().state, VISION_NETWORK_RECOVERY_STATE.WAITING);
});

test("persistent failure identifies a possible changed address or hotspot route", async () => {
  const scheduler = createScheduler();
  const controller = createVisionNetworkRecoveryController({
    probe: async () => {
      throw new Error("origin unreachable");
    },
    schedule: scheduler.schedule,
    cancel: scheduler.cancel,
    backoffMs: [10],
    probeTimeoutMs: 0,
  });

  controller.setEnabled(true);
  await flush();
  scheduler.runNext();
  await flush();
  scheduler.runNext();
  await flush();

  assert.equal(controller.snapshot().failureCount, 3);
  assert.equal(
    controller.snapshot().originClassification,
    "possible-origin-address-change-or-hotspot-route",
  );
});

test("disabling aborts the probe and removes every scheduled check", async () => {
  const scheduler = createScheduler();
  let aborted = false;
  const controller = createVisionNetworkRecoveryController({
    probe: ({ signal }) => new Promise((resolve, reject) => {
      signal.addEventListener("abort", () => {
        aborted = true;
        reject(signal.reason || new DOMException("Aborted", "AbortError"));
      }, { once: true });
    }),
    schedule: scheduler.schedule,
    cancel: scheduler.cancel,
  });

  controller.setEnabled(true);
  controller.setEnabled(false, { reason: "page hidden" });
  await flush();

  assert.equal(aborted, true);
  assert.equal(controller.snapshot().state, VISION_NETWORK_RECOVERY_STATE.DISABLED);
  assert.equal(controller.snapshot().queueScheduled, false);
  assert.equal(controller.snapshot().inFlight, false);
  assert.equal(scheduler.items().length, 0);
});
