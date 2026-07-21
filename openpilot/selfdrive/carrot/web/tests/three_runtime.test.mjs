import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";

import { createArRuntime } from "../src/features/drive/contents/vision/ar/runtime.js";

test("production AR entry graph cannot import the retired Canvas2D renderer", () => {
  for (const relative of [
    "../src/features/drive/contents/vision/ar/index.js",
    "../src/features/drive/contents/vision/ar/runtime.js",
    "../src/features/drive/contents/vision/ar/worker.js",
  ]) {
    const source = readFileSync(new URL(relative, import.meta.url), "utf8");
    assert.doesNotMatch(source, /(?:from|import\s*)\s*[('"`]\.\/renderer\.js/);
    assert.doesNotMatch(source, /createArRenderer/);
  }
});

function runtimeFixture(backend, overrides = {}) {
  const workers = [];
  const canvases = [];
  const frameMessages = [];
  const { target: targetOverrides = {}, ...runtimeOverrides } = overrides;

  class FakeWorker {
    constructor() {
      this.terminated = false;
      workers.push(this);
    }

    postMessage(message) {
      if (message.type === "init") {
        this.onmessage?.({ data: { type: "ready", backend } });
      } else if (message.type === "frame") {
        frameMessages.push(message);
        const response = overrides.workerFrameReply?.(message);
        if (response) this.onmessage?.({ data: response });
      }
    }

    terminate() { this.terminated = true; }
  }

  const host = {
    appendChild(node) { node.parent = this; },
  };
  const documentRoot = {
    createElement() {
      const canvas = {
        className: "",
        style: { cssText: "", display: "unset" },
        dataset: {},
        removed: false,
        transferControlToOffscreen() { return {}; },
        getBoundingClientRect() { return { width: 1280, height: 720 }; },
        remove() { this.removed = true; },
      };
      canvases.push(canvas);
      return canvas;
    },
    getElementById() { return host; },
  };
  const target = {
    Worker: FakeWorker,
    devicePixelRatio: 1,
    console: { warn() {} },
    ...targetOverrides,
  };
  const runtime = createArRuntime({
    target,
    document: documentRoot,
    host,
    resolveAssetUrl: () => "/ar-worker.js",
    ...runtimeOverrides,
  });
  return { runtime, workers, canvases, frameMessages };
}

test("runtime exposes the AR canvas only after a Three worker handshake", () => {
  const fixture = runtimeFixture("three");
  assert.ok(fixture.runtime);
  assert.equal(fixture.runtime.status().worker.ready, true);
  assert.equal(fixture.runtime.status().worker.broken, null);
  assert.equal(fixture.canvases[0].style.display, "");
  assert.equal(fixture.canvases[0].dataset.arReady, "true");
  assert.equal(fixture.canvases[0].dataset.arBackend, "three");
  assert.equal(fixture.workers[0].terminated, false);
  fixture.runtime.destroy();
});

test("AR canvas diagnostics keep sync, hold, renderer, and clock causes separate", () => {
  const fixture = runtimeFixture("three", {
    workerFrameReply: () => ({
      type: "drawn",
      ok: false,
      hold: {
        state: "held",
        sourceMode: "held",
        anchorCount: 1,
        holdMs: 150,
        driftM: 0.12,
        reason: "bounded hold",
      },
      renderer: {
        drawn: 1,
        skipped: 0,
        lastReason: "transient projection gap",
      },
      performance: { level: "target", fps: 20, failed: false },
    }),
  });

  assert.equal(fixture.runtime.activate(), true);
  const data = fixture.canvases[0].dataset;
  assert.equal(data.arClockDomain, "live-monotonic");
  assert.equal(data.arHoldState, "held");
  assert.equal(data.arAnchorMode, "held");
  assert.equal(data.arAnchorCount, "1");
  assert.equal(data.arComposedAnchorCount, "0");
  assert.equal(data.arHoldMs, "150");
  assert.equal(data.arDriftM, "0.12");
  assert.equal(data.arHoldReason, "bounded hold");
  assert.equal(data.arRenderReason, "transient projection gap");
  assert.ok(data.arSyncReason);
  assert.equal(data.arReason, "transient projection gap");
  fixture.runtime.destroy();
});

test("runtime hides and terminates any non-Three worker without a 2D fallback", () => {
  const fixture = runtimeFixture("canvas2d");
  assert.ok(fixture.runtime);
  assert.equal(fixture.runtime.status().worker.ready, false);
  assert.match(fixture.runtime.status().worker.broken, /unexpected AR renderer backend/);
  assert.equal(fixture.canvases[0].style.display, "none");
  assert.equal(fixture.canvases[0].dataset.arReady, "false");
  assert.equal(fixture.canvases[0].dataset.arBackend, "failed");
  assert.equal(fixture.workers[0].terminated, true);
  fixture.runtime.destroy();
});

test("asynchronous Three failure releases the AR data lease immediately", () => {
  let released = false;
  const fixture = runtimeFixture("three", {
    activity: {
      acquire() {
        return {
          active: true,
          release() { this.active = false; released = true; },
        };
      },
    },
  });

  assert.equal(fixture.runtime.activate(), true);
  assert.equal(fixture.runtime.status().leaseActive, true);
  fixture.workers[0].onerror?.({ message: "gpu failure" });

  assert.equal(released, true);
  assert.equal(fixture.runtime.status().active, false);
  assert.equal(fixture.runtime.status().leaseActive, false);
  assert.equal(fixture.canvases[0].style.display, "none");
  fixture.runtime.destroy();
});

test("runtime applies the Worker-reported 20fps cadence without a server change", () => {
  let nowMs = 0;
  let presented;
  const fixture = runtimeFixture("three", {
    target: {
      performance: { now: () => nowMs },
    },
    frameSync: {
      subscribePresented(callback) { presented = callback; return () => {}; },
    },
    workerFrameReply: () => ({
      type: "drawn",
      ok: true,
      performance: { level: "degraded", fps: 20, failed: false },
    }),
  });

  assert.equal(fixture.runtime.activate(), true);
  assert.equal(fixture.runtime.status().currentFps, 20);
  assert.equal(fixture.canvases[0].dataset.arOk, "true");
  assert.equal(fixture.frameMessages.length, 1);

  nowMs = 45;
  presented();
  assert.equal(fixture.frameMessages.length, 1);

  nowMs = 55;
  presented({ source: "replay" });
  assert.equal(fixture.frameMessages.length, 2);
  assert.equal(fixture.runtime.status().frameSignalMode, "presented-channel");
  assert.equal(fixture.runtime.status().presentedSignals, 2);
  assert.equal(fixture.runtime.status().lastPresentedSource, "replay");
  fixture.runtime.destroy();
});

test("runtime resamples synchronized state on a browser 30Hz presentation clock", () => {
  let nowMs = 0;
  let nextFrameId = 0;
  let presented;
  const scheduled = new Map();
  const cancelled = [];
  const fixture = runtimeFixture("three", {
    target: {
      performance: { now: () => nowMs },
      requestAnimationFrame(callback) {
        const id = ++nextFrameId;
        scheduled.set(id, callback);
        return id;
      },
      cancelAnimationFrame(id) {
        cancelled.push(id);
        scheduled.delete(id);
      },
    },
    frameSync: {
      subscribePresented(callback) { presented = callback; return () => {}; },
    },
    workerFrameReply: () => ({
      type: "drawn",
      ok: true,
      performance: { level: "target", fps: 30, failed: false },
    }),
  });

  assert.equal(fixture.runtime.activate(), true);
  assert.equal(fixture.frameMessages.length, 1);
  assert.equal(fixture.runtime.status().presentationClock, "raf");
  assert.equal(fixture.runtime.status().presentationLoopActive, true);

  nowMs = 10;
  presented({ source: "live" });
  assert.equal(fixture.frameMessages.length, 1);
  assert.equal(fixture.runtime.status().presentedSignals, 1);

  const tick = () => {
    const [id, callback] = scheduled.entries().next().value;
    scheduled.delete(id);
    callback(nowMs);
  };
  nowMs = 16;
  tick();
  assert.equal(fixture.frameMessages.length, 1);
  nowMs = 34;
  tick();
  assert.equal(fixture.frameMessages.length, 2);

  fixture.runtime.deactivate();
  assert.equal(fixture.runtime.status().presentationLoopActive, false);
  assert.ok(cancelled.length > 0);
  fixture.runtime.destroy();
});

test("paused replay keeps AR on the media timeline instead of expiring on wall clock", () => {
  let monotonicMs = 100_000;
  let mediaSeconds = 12;
  let presented;
  const modelV2 = {
    frameId: 100,
    frameAge: 0,
    position: { x: [0, 40], y: [0, 0], z: [0, 0] },
  };
  const fixture = runtimeFixture("three", {
    target: {
      performance: { now: () => monotonicMs },
      CarrotVisionReplay: {
        isActive: () => true,
        status: () => ({ active: true, currentTime: mediaSeconds }),
      },
    },
    stage: { stageWidth: 1280, stageHeight: 720 },
    frameSync: {
      subscribePresented(callback) { presented = callback; return () => {}; },
      selectModel() { return modelV2; },
    },
    provider: {
      snapshot() {
        return {
          receivedAtMonotonic: { modelV2: 1, liveCalibration: 1 },
          hudState: { carState: { vEgo: 0 } },
          overlayState: {
            modelV2,
            roadCameraState: { frameId: 100, timestampEof: 12_000_000_000 },
            liveCalibration: { calStatus: "calibrated", rpyCalib: [0, 0, 0] },
          },
        };
      },
    },
    workerFrameReply: () => ({
      type: "drawn",
      ok: true,
      performance: { level: "target", fps: 20, failed: false },
    }),
  });

  assert.equal(fixture.runtime.activate(), true);
  assert.equal(fixture.frameMessages[0].payload.nowMs, 12_000);
  assert.equal(fixture.frameMessages[0].payload.clockDomain, "replay-media");
  assert.equal(fixture.frameMessages[0].payload.sync.canDrawPrecise, true);

  // 실제 시간은 5초 흘렀지만 일시정지된 media time은 그대로다.
  monotonicMs += 5_000;
  presented({ source: "replay" });
  assert.equal(fixture.frameMessages.length, 2);
  assert.equal(fixture.frameMessages[1].payload.nowMs, 12_000);
  assert.equal(fixture.frameMessages[1].payload.sync.canDrawPrecise, true);
  assert.deepEqual(fixture.runtime.status().timeline, {
    domain: "replay-media",
    nowMs: 12_000,
  });

  mediaSeconds = 12.05;
  fixture.runtime.destroy();
});

test("runtime compatibility fallback listens to the canonical render request", () => {
  const listeners = new Map();
  const fixture = runtimeFixture("three", {
    target: {
      addEventListener(name, listener) { listeners.set(name, listener); },
      removeEventListener(name, listener) {
        if (listeners.get(name) === listener) listeners.delete(name);
      },
    },
  });

  assert.equal(fixture.runtime.activate(), true);
  assert.equal(fixture.runtime.status().frameSignalMode, "render-request-fallback");
  assert.equal(typeof listeners.get("carrot:render-request"), "function");
  assert.equal(listeners.has("carrot:renderrequest"), false);
  listeners.get("carrot:render-request")();
  assert.equal(fixture.runtime.status().presentedSignals, 1);
  fixture.runtime.destroy();
  assert.equal(listeners.size, 0);
});

test("runtime renders the model synchronized to the presented camera frame", () => {
  const latestModel = {
    frameId: 105,
    position: { x: [0, 40], y: [0, 2], z: [0, 0] },
  };
  const presentedModel = {
    frameId: 100,
    position: { x: [0, 40], y: [0, 1], z: [0, 0] },
  };
  const receipts = {
    roadCameraState: 0,
    modelV2: 0,
    liveCalibration: 0,
    carState: 0,
    selfdriveState: 0,
    radarState: 0,
  };
  const fixture = runtimeFixture("three", {
    target: { performance: { now: () => 0 } },
    stage: { videoWidth: 1928, videoHeight: 1208, stageWidth: 1280, stageHeight: 720 },
    frameSync: {
      subscribePresented() { return () => {}; },
      selectModel() { return presentedModel; },
    },
    provider: {
      snapshot() {
        return {
          receivedAtMonotonic: receipts,
          hudState: {
            carState: { gearShifter: "drive", aEgo: 0, vEgo: 15 },
            selfdriveState: { alertStatus: "normal" },
          },
          overlayState: {
            modelV2: latestModel,
            roadCameraState: { frameId: 100 },
            liveCalibration: { calStatus: "calibrated", rpyCalib: [0, 0, 0] },
            radarState: {},
          },
        };
      },
    },
  });

  assert.equal(fixture.runtime.activate(), true);
  assert.equal(fixture.frameMessages.length, 1);
  assert.equal(fixture.frameMessages[0].payload.composeInput.modelPosition, presentedModel.position);
  assert.equal(fixture.frameMessages[0].payload.sync.frameIdGap, 0);
  assert.equal(fixture.frameMessages[0].payload.sync.canDrawPrecise, true);
  assert.equal(fixture.runtime.diagnose().modelFrameId, 100);
  fixture.runtime.destroy();
});

test("runtime diagnostics expose marker composition and Navi state", () => {
  const fixture = runtimeFixture("three", {
    provider: {
      snapshot() {
        return {
          hudState: {},
          overlayState: {
            carrotNavi: {
              connected: true,
              navigationStatus: {
                guidanceActive: true,
                offRoute: false,
                routePresent: true,
              },
            },
          },
          receivedAtMonotonic: {},
        };
      },
    },
    workerFrameReply: () => ({
      type: "drawn",
      ok: true,
      composition: {
        signCount: 1,
        anchoredCount: 1,
        sources: ["guidanceCurrent"],
      },
      renderer: { drawn: 1, lastReason: "" },
    }),
  });

  assert.equal(fixture.runtime.activate(), true);
  assert.deepEqual(fixture.runtime.status().composition, {
    signCount: 1,
    anchoredCount: 1,
    sources: ["guidanceCurrent"],
  });
  const diagnosis = fixture.runtime.diagnose();
  assert.equal(diagnosis.drawn, 1);
  assert.deepEqual(diagnosis.composition.sources, ["guidanceCurrent"]);
  assert.deepEqual(diagnosis.navi, {
    present: true,
    connected: true,
    guidanceActive: true,
    offRoute: false,
    routePresent: true,
    usable: false,
    ageMs: null,
  });
  fixture.runtime.destroy();
});

test("runtime does not hide AR for gear, FCW, deceleration, or critical alerts", () => {
  const modelV2 = {
    frameId: 100,
    position: { x: [0, 40], y: [0, 0], z: [0, 0] },
  };
  const fixture = runtimeFixture("three", {
    target: { performance: { now: () => 0 } },
    stage: { videoWidth: 1928, videoHeight: 1208, stageWidth: 1280, stageHeight: 720 },
    frameSync: {
      subscribePresented() { return () => {}; },
      selectModel() { return modelV2; },
    },
    provider: {
      snapshot() {
        return {
          hudState: {
            carState: { gearShifter: "park", vEgo: 0, aEgo: -5 },
            selfdriveState: { enabled: false, alertStatus: "critical", alertType: "fcw" },
          },
          overlayState: {
            modelV2,
            roadCameraState: { frameId: 100 },
            liveCalibration: { calStatus: "calibrated", rpyCalib: [0, 0, 0] },
            radarState: { leadOne: { fcw: true } },
          },
          receivedAtMonotonic: {
            roadCameraState: 0,
            modelV2: 0,
            liveCalibration: 0,
          },
        };
      },
    },
    workerFrameReply: () => ({ type: "drawn", ok: true }),
  });

  assert.equal(fixture.runtime.activate(), true);
  const payload = fixture.frameMessages[0].payload;
  assert.equal(payload.sync.canDrawPrecise, true);
  assert.doesNotMatch(payload.sync.reasons.join(" | "), /gate|gear|FCW|critical/i);
  assert.equal(Object.hasOwn(payload.sync, "safetyAllowed"), false);
  assert.equal(Object.hasOwn(fixture.runtime.status(), "safety"), false);
  fixture.runtime.destroy();
});
