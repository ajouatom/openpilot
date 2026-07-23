import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";

import { createArRuntime } from "../src/features/drive/contents/vision/ar/runtime.js";
import { createDeviceWorldPose } from "../src/features/drive/contents/vision/ar/world_pose.js";

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

test("AR worker forwards capture diagnostics into the Three renderer frame", () => {
  const source = readFileSync(new URL(
    "../src/features/drive/contents/vision/ar/worker.js",
    import.meta.url,
  ), "utf8");
  assert.match(source, /renderer\.render\(\{[\s\S]*diagnosticsEnabled:\s*payload\.diagnosticsEnabled\s*===\s*true/);
  assert.match(source, /activeMarkers:\s*composition\.signs/);
  assert.match(source, /lifecycleAuthoritative:\s*isProbe/);
  assert.match(source, /retainedComposeSources/);
  assert.match(source, /hasOwnProperty\.call\(sourceUpdate,\s*"navi"\)/);
  assert.match(source, /hasOwnProperty\.call\(sourceUpdate,\s*"modelPosition"\)/);
});

function runtimeFixture(backend, overrides = {}) {
  const workers = [];
  const canvases = [];
  const frameMessages = [];
  const resizeMessages = [];
  const { target: targetOverrides = {}, ...runtimeOverrides } = overrides;

  class FakeWorker {
    constructor() {
      this.terminated = false;
      workers.push(this);
    }

    postMessage(message) {
      if (message.type === "init") {
        this.onmessage?.({ data: { type: "ready", backend } });
      } else if (message.type === "resize") {
        resizeMessages.push(message);
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
  return { runtime, workers, canvases, frameMessages, resizeMessages };
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

test("runtime trace is opt-in, bounded, and available without server logging", () => {
  const fixture = runtimeFixture("three", { traceCapacity: 4 });
  assert.deepEqual(fixture.runtime.trace.status(), {
    enabled: false, capacity: 4, size: 0, dropped: 0,
  });

  fixture.runtime.trace.enable({ clear: true });
  assert.equal(fixture.runtime.activate(), true);
  const trace = fixture.runtime.trace.snapshot();
  assert.equal(trace.enabled, true);
  assert.equal(trace.entries.some((entry) => entry.kind === "activate"), true);
  assert.equal(trace.entries.some((entry) => entry.kind === "frame-submit"), true);
  assert.equal(trace.entries.at(-1).payload.traceFrameId, 1);

  fixture.runtime.destroy();
});

test("full AR frame diagnostics exist only while a capture listener is attached", () => {
  const fixture = runtimeFixture("three", {
    workerFrameReply: (message) => ({
      type: "drawn",
      traceFrameId: message.payload.traceFrameId,
      debugFrame: message.payload.debugFrame,
      ok: true,
      composition: { signCount: 0, anchoredCount: 0, sources: [], diag: null },
      hold: { state: "dropped", anchorCount: 0 },
      tracking: message.payload.tracking,
      worldPose: { initialized: false, integrations: 0 },
      renderer: { drawn: 0, skipped: 0, markers: [] },
      performance: { level: "target", fps: 30 },
    }),
  });
  const observed = [];
  const unsubscribe = fixture.runtime.subscribeDiagnostics((frame) => observed.push(frame));

  assert.equal(fixture.runtime.activate(), true);
  assert.equal(fixture.frameMessages[0].payload.diagnosticsEnabled, true);
  assert.ok(fixture.frameMessages[0].payload.debugFrame);
  assert.equal(observed.length, 1);

  unsubscribe();
  fixture.runtime.render(true);
  assert.equal(fixture.frameMessages.at(-1).payload.diagnosticsEnabled, false);
  assert.equal(fixture.frameMessages.at(-1).payload.debugFrame, null);
  assert.equal(observed.length, 1);
  fixture.runtime.destroy();
});

test("the synthetic marker requires the diagnosticProbe option, not the legacy product flag", () => {
  const legacy = runtimeFixture("three", { calibrationProbe: true });
  assert.equal(legacy.runtime.status().probeEnabled, false);
  assert.equal(legacy.runtime.setProbeDistance(60), false);
  legacy.runtime.destroy();

  const diagnostic = runtimeFixture("three", { diagnosticProbe: true });
  assert.equal(diagnostic.runtime.status().probeEnabled, true);
  assert.equal(diagnostic.runtime.setProbeDistance(60), true);
  assert.equal(diagnostic.runtime.status().probeDistanceM, 60);
  diagnostic.runtime.destroy();
});

test("worker AR canvas uses the calibrated render viewport instead of the whole stage", () => {
  const fixture = runtimeFixture("three", {
    target: { devicePixelRatio: 3 },
    stage: {
      stageWidth: 1104,
      stageHeight: 711,
      viewportLeft: 0,
      viewportTop: 52,
    },
  });

  assert.equal(fixture.runtime.activate(), true);
  const canvas = fixture.canvases[0];
  assert.equal(canvas.style.left, "0px");
  assert.equal(canvas.style.top, "52px");
  assert.equal(canvas.style.width, "1104px");
  assert.equal(canvas.style.height, "711px");
  assert.equal(canvas.dataset.arViewport, "0,52,1104x711");
  assert.deepEqual(fixture.resizeMessages.at(-1), {
    type: "resize",
    pixelWidth: 2208,
    pixelHeight: 1422,
  });
  assert.equal(fixture.frameMessages[0].payload.stage.devicePixelRatio, 2);
  assert.equal(fixture.runtime.status().worker.dpr, 2);
  assert.deepEqual(fixture.runtime.status().worker.viewport, {
    left: 0,
    top: 52,
    width: 1104,
    height: 711,
  });
  fixture.runtime.destroy();
});

test("degraded AR lowers only its own OffscreenCanvas DPR", () => {
  const fixture = runtimeFixture("three", {
    target: { devicePixelRatio: 3 },
    stage: { stageWidth: 1000, stageHeight: 600 },
    workerFrameReply: () => ({
      type: "drawn",
      ok: true,
      performance: { level: "degraded", fps: 15, failed: false },
    }),
  });

  fixture.runtime.activate();
  assert.deepEqual(fixture.resizeMessages[0], {
    type: "resize", pixelWidth: 2000, pixelHeight: 1200,
  });
  fixture.runtime.render(true);
  assert.deepEqual(fixture.resizeMessages.at(-1), {
    type: "resize", pixelWidth: 1500, pixelHeight: 900,
  });
  assert.equal(fixture.frameMessages.at(-1).payload.stage.devicePixelRatio, 1.5);
  assert.equal(fixture.runtime.status().worker.dpr, 1.5);
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
  assert.equal(fixture.frameMessages.length, 2, "the first real video frame is never throttled by activation");

  nowMs = 70;
  presented({ source: "live" });
  assert.equal(fixture.frameMessages.length, 2);

  nowMs = 100;
  presented({ source: "replay" });
  assert.equal(fixture.frameMessages.length, 3);
  assert.equal(fixture.runtime.status().frameSignalMode, "presented-channel");
  assert.equal(fixture.runtime.status().presentedSignals, 3);
  assert.equal(fixture.runtime.status().presentedSubmissions, 2);
  assert.equal(fixture.runtime.status().lastPresentedSource, "replay");
  fixture.runtime.destroy();
});

test("worker backpressure keeps one in-flight frame and one latest pending frame", () => {
  const fixture = runtimeFixture("three", {
    stage: { stageWidth: 1280, stageHeight: 720 },
  });

  fixture.runtime.activate();
  fixture.runtime.render(true);
  fixture.runtime.render(true);
  assert.equal(fixture.frameMessages.length, 1);
  assert.deepEqual(fixture.runtime.status().worker.queue, {
    posted: 1,
    completed: 0,
    deferred: 2,
    replaced: 1,
    discarded: 0,
    inFlight: true,
    pending: true,
    maxPending: 1,
  });

  fixture.workers[0].onmessage?.({
    data: { type: "drawn", ok: true, performance: { level: "target", fps: 30 } },
  });
  assert.equal(fixture.frameMessages.length, 2);
  assert.equal(fixture.runtime.status().worker.queue.posted, 2);
  assert.equal(fixture.runtime.status().worker.queue.completed, 1);
  assert.equal(fixture.runtime.status().worker.queue.pending, false);
  fixture.runtime.destroy();
});

test("runtime never advances spatial presentation on an independent browser rAF", () => {
  let nowMs = 0;
  let presented;
  let rafRequests = 0;
  const fixture = runtimeFixture("three", {
    target: {
      performance: { now: () => nowMs },
      requestAnimationFrame() { rafRequests += 1; return rafRequests; },
      cancelAnimationFrame() {},
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
  assert.equal(fixture.runtime.status().presentationClock, "presented-frame");
  assert.equal(fixture.runtime.status().presentationLoopActive, false);
  assert.equal(rafRequests, 0);

  nowMs = 40;
  presented({ source: "live" });
  assert.equal(fixture.frameMessages.length, 2);
  assert.equal(fixture.runtime.status().presentedSignals, 1);
  assert.equal(fixture.runtime.status().presentedSubmissions, 1);

  fixture.runtime.deactivate();
  assert.equal(fixture.runtime.status().presentationLoopActive, false);
  assert.equal(rafRequests, 0);
  fixture.runtime.destroy();
});

test("one presented video frame produces one spatial submission and no between-frame slide", () => {
  let nowMs = 0;
  let presented;
  const fixture = runtimeFixture("three", {
    target: {
      performance: { now: () => nowMs },
      requestAnimationFrame() { throw new Error("AR must not schedule an independent rAF"); },
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
  fixture.runtime.activate();
  nowMs = 5;
  presented({
    source: "live",
    sequence: 1,
    frameId: 200,
    cameraTimestampEof: 10_000_000_000,
    clockMappingConfidence: "exact-frame",
    metadata: { mediaTime: 1 },
  });
  const firstPresentedSubmission = fixture.frameMessages.at(-1);
  assert.equal(firstPresentedSubmission.payload.nowMs, 10_000);
  assert.equal(fixture.runtime.status().presentedSubmissions, 1);
  assert.equal(fixture.runtime.status().lastSubmittedPresentedSequence, 1);

  // Browser time can advance arbitrarily while the video frame is frozen.
  // Without a new presented-frame callback no spatial payload is submitted.
  const submissionsAfterFirstFrame = fixture.frameMessages.length;
  nowMs = 40;
  nowMs = 80;
  assert.equal(fixture.frameMessages.length, submissionsAfterFirstFrame);

  presented({ source: "live", sequence: 2, metadata: { mediaTime: 1.05 } });
  assert.equal(fixture.frameMessages.at(-1).payload.nowMs, 10_000);
  assert.equal(fixture.frameMessages.at(-1).payload.canHoldAnchor, false);
  assert.equal(fixture.frameMessages.at(-1).payload.composeInput.canDrawPrecise, false);
  assert.equal(fixture.runtime.status().presentedSubmissions, 2);
  assert.equal(fixture.runtime.status().lastSubmittedPresentedSequence, 2);
  fixture.runtime.destroy();
});

test("runtime sends tracking lifecycle and retains anchors through a 400ms unmapped gap", () => {
  let nowMs = 0;
  let presented;
  let cameraFrameId = 200;
  let cameraTimestampNs = 10_000_000_000;
  const modelV2 = {
    frameId: cameraFrameId,
    frameAge: 0,
    position: { x: [0, 40], y: [0, 0], z: [0, 0] },
  };
  const workerWorldPose = createDeviceWorldPose();
  let workerWorldEpoch = null;
  const fixture = runtimeFixture("three", {
    target: { performance: { now: () => nowMs } },
    stage: { videoWidth: 1928, videoHeight: 1208, stageWidth: 1280, stageHeight: 720 },
    frameSync: {
      subscribePresented(callback) { presented = callback; return () => {}; },
      selectModel() { return modelV2; },
    },
    provider: {
      snapshot() {
        return {
          receivedAtMonotonic: {
            modelV2: nowMs,
            cameraOdometry: nowMs,
            livePose: nowMs,
            liveCalibration: nowMs,
          },
          hudState: { carState: { vEgo: 15 } },
          overlayState: {
            modelV2,
            roadCameraState: { frameId: cameraFrameId, timestampEof: cameraTimestampNs },
            cameraOdometry: {
              frameId: cameraFrameId,
              timestampEof: cameraTimestampNs + 100_000_000,
              trans: [15, 0, 0],
              rot: [0, 0, 0.01],
              transStd: [0.1, 0.02, 0.02],
              rotStd: [0.002, 0.002, 0.002],
            },
            livePose: {
              timestamp: cameraTimestampNs,
              orientationNED: { x: 0, y: 0, z: 0, xStd: 0.01, yStd: 0.01, zStd: 0.02, valid: true },
              velocityDevice: { x: 15, y: 0, z: 0, xStd: 0.1, yStd: 0.1, zStd: 0.1, valid: true },
              accelerationDevice: { x: 0, y: 0, z: 0, xStd: 0.2, yStd: 0.2, zStd: 0.2, valid: true },
              angularVelocityDevice: { x: 0, y: 0, z: 0.01, xStd: 0.01, yStd: 0.01, zStd: 0.01, valid: true },
              inputsOK: true,
              posenetOK: true,
              sensorsOK: true,
            },
            liveCalibration: { calStatus: "calibrated", rpyCalib: [0, 0, 0] },
          },
        };
      },
    },
    workerFrameReply: (message) => {
      if (workerWorldEpoch !== message.payload.worldPoseEpoch) {
        workerWorldEpoch = message.payload.worldPoseEpoch;
        workerWorldPose.reset({ epoch: workerWorldEpoch });
      }
      const worldPose = message.payload.deviceOdometry
        ? workerWorldPose.update({
          timestampNs: message.payload.worldPoseTargetTimestampNs,
          odometry: message.payload.deviceOdometry,
          livePose: message.payload.livePose,
          geographicObservation: message.payload.geographicObservation,
          trackingState: message.payload.tracking?.state,
        })
        : workerWorldPose.status();
      return {
        type: "drawn",
        ok: true,
        tracking: message.payload.tracking,
        worldPose,
        performance: { level: "target", fps: 20, failed: false },
      };
    },
  });

  fixture.runtime.activate();
  nowMs = 50;
  presented({
    source: "live",
    sequence: 1,
    frameId: 200,
    cameraTimestampEof: 10_000_000_000,
    clockMappingConfidence: "exact-frame",
  });
  assert.equal(fixture.frameMessages.at(-1).payload.tracking.state, "tracking");
  assert.equal(fixture.frameMessages.at(-1).payload.tracking.canCreateAnchor, true);
  assert.equal(fixture.canvases[0].dataset.arTracking, "tracking");
  assert.equal(fixture.frameMessages.at(-1).payload.deviceOdometry.coordinateFrame, "openpilot-device-frd");
  assert.equal(fixture.frameMessages.at(-1).payload.livePose.velocityDevice.x, 15);
  assert.equal(fixture.frameMessages.at(-1).payload.livePose.accelerationDevice.valid, true);
  assert.equal(fixture.runtime.status().worldPose.initialized, true);
  assert.deepEqual(fixture.runtime.status().worldPose.position, [0, 0, 0]);

  nowMs = 100;
  cameraFrameId = 201;
  cameraTimestampNs = 10_050_000_000;
  modelV2.frameId = cameraFrameId;
  presented({
    source: "live",
    sequence: 2,
    frameId: cameraFrameId,
    cameraTimestampEof: cameraTimestampNs,
    clockMappingConfidence: "exact-frame",
  });
  assert.equal(fixture.runtime.status().worldPose.integrations, 1);
  assert.ok(Math.abs(Math.hypot(...fixture.runtime.status().worldPose.position) - 0.75) < 1e-9);
  assert.equal(fixture.runtime.status().worldPose.deviceCoordinateFrame, "openpilot-device-frd");

  nowMs = 500;
  presented({ source: "live", sequence: 3 });
  const gap = fixture.frameMessages.at(-1).payload;
  assert.equal(gap.tracking.state, "coasting");
  assert.equal(gap.tracking.retainAnchor, true);
  // 새 영상 프레임이 없는 갭이어도 cereal 입력은 신선하므로 앵커 "생성"은 계속
  // 허용하고, odometry가 살아 있으므로 기존 앵커 "전파"도 계속된다. 그것이 coast의
  // 목적이다. 새 관측이 없다는 사실은 worldPose.integrations가 늘지 않는 것으로 확인한다.
  assert.equal(gap.composeInput.canDrawPrecise, true);
  assert.equal(gap.tracking.canPropagateAnchor, true);
  assert.equal(gap.nowMs, 10_050);
  assert.equal(gap.presentationNowMs, 500);
  assert.equal(fixture.runtime.status().tracking.state, "coasting");
  assert.equal(fixture.runtime.status().worldPose.integrations, 1);
  fixture.runtime.destroy();
});

/* replay는 roadCameraState를 frameId/sensor만 실어 보낸다(timestampEof 없음).
 * 그때 spatial clock이 서지 않으면 world pose가 초기화되지 않고 tracking이 LOST에
 * 머물러 AR이 통째로 사라진다. 같은 카메라 프레임 시각인 cameraOdometry.timestampEof로
 * media↔cereal offset을 세워 그 사슬을 끊는다. */
test("replay seeds the spatial clock from cameraOdometry when roadCameraState carries no time", () => {
  let monotonicMs = 100_000;
  let presented;
  const mediaSeconds = 12;
  const cameraTimestampNs = 30_000_000_000;
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
          receivedAtMonotonic: { modelV2: 1, cameraOdometry: 1, liveCalibration: 1 },
          hudState: { carState: { vEgo: 15 } },
          overlayState: {
            modelV2,
            // 실제 replay 형태: 시간 필드가 없다.
            roadCameraState: { frameId: 100, sensor: "ox03c10" },
            cameraOdometry: {
              frameId: 100,
              timestampEof: cameraTimestampNs,
              trans: [15, 0, 0],
              rot: [0, 0, 0],
              transStd: [0.1, 0.02, 0.02],
              rotStd: [0.002, 0.002, 0.002],
            },
            liveCalibration: { calStatus: "calibrated", rpyCalib: [0, 0, 0] },
          },
        };
      },
    },
    workerFrameReply: () => ({
      type: "drawn", ok: true,
      performance: { level: "target", fps: 20, failed: false },
    }),
  });

  assert.equal(fixture.runtime.activate(), true);
  // 실제 replay 프레임: cereal timestamp 없이 media 시각만 실려 온다.
  presented({ source: "replay", metadata: { mediaTime: mediaSeconds } });
  fixture.runtime.render(true);
  const clock = fixture.frameMessages.at(-1).payload.presentedClock;
  assert.equal(clock.confidence, "estimated-frame");
  assert.equal(clock.targetTimestampNs, cameraTimestampNs);
  // offset이 서면 spatial clock이 살아나 world pose 적분이 가능해진다.
  assert.equal(clock.mediaToCerealOffsetNs, cameraTimestampNs - mediaSeconds * 1e9);
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
    epoch: "replay",
    deltaMs: 0,
    discontinuity: false,
    discontinuityReason: null,
  });

  mediaSeconds = 12.05;
  fixture.runtime.destroy();
});

test("runtime forwards explicit replay seek once and ignores a 400ms forward gap", () => {
  let mediaSeconds = 10;
  let presented;
  const fixture = runtimeFixture("three", {
    target: {
      performance: { now: () => mediaSeconds * 1000 },
      CarrotVisionReplay: {
        isActive: () => true,
        status: () => ({ active: true, currentTime: mediaSeconds, segment: "segment-a" }),
        videoSourceKey: () => "video-a",
      },
    },
    frameSync: {
      subscribePresented(listener) { presented = listener; return () => {}; },
    },
    workerFrameReply: () => ({
      type: "drawn",
      ok: true,
      performance: { level: "target", fps: 20, failed: false },
    }),
  });

  fixture.runtime.activate();
  mediaSeconds = 10.4;
  presented({ source: "replay", metadata: { mediaTime: 10.4 } });
  assert.equal(fixture.frameMessages.at(-1).payload.timelineDiscontinuity, false);

  mediaSeconds = 20;
  presented({
    source: "replay",
    sequence: 3,
    metadata: {
      mediaTime: 20,
      discontinuity: true,
      discontinuityReason: "replay-seek",
    },
  });
  assert.equal(fixture.frameMessages.at(-1).payload.timelineDiscontinuity, true);
  assert.equal(fixture.frameMessages.at(-1).payload.timelineDiscontinuityReason, "replay-seek");

  mediaSeconds = 20.05;
  presented({ source: "replay", metadata: { mediaTime: 20.05 } });
  assert.equal(fixture.frameMessages.at(-1).payload.timelineDiscontinuity, false);
  fixture.runtime.destroy();
});

test("runtime uses the presented frame target instead of rereading a newer replay time", () => {
  let presented;
  let monotonicMs = 100_000;
  const fixture = runtimeFixture("three", {
    target: {
      performance: { now: () => monotonicMs },
      CarrotVisionReplay: {
        isActive: () => true,
        status: () => ({ active: true, currentTime: 99, segment: "segment-a" }),
        videoSourceKey: () => "video-a",
      },
    },
    frameSync: {
      subscribePresented(listener) { presented = listener; return () => {}; },
    },
    workerFrameReply: () => ({
      type: "drawn",
      ok: true,
      performance: { level: "target", fps: 20, failed: false },
    }),
  });

  fixture.runtime.activate();
  monotonicMs += 100;
  presented({
    source: "replay",
    sequence: 2,
    frameId: 300,
    cameraTimestampEof: 15_000_000_000,
    clockMappingConfidence: "exact-frame",
    metadata: { mediaTime: 15 },
  });

  const payload = fixture.frameMessages.at(-1).payload;
  assert.equal(payload.nowMs, 15_000);
  assert.equal(fixture.runtime.status().presentedClock.targetTimeMs, 15_000);
  assert.equal(fixture.runtime.status().presentedClock.confidence, "exact-frame");
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
  assert.notEqual(fixture.frameMessages[0].payload.sourceUpdate.modelPosition, presentedModel.position);
  assert.deepEqual(
    fixture.frameMessages[0].payload.sourceUpdate.modelPosition.y,
    presentedModel.position.y.map((value) => (value === 0 ? 0 : -value)),
  );
  assert.equal(fixture.frameMessages[0].payload.sourceUpdate.modelPosition.coordinateFrame, "route-local-flu");
  assert.equal(fixture.frameMessages[0].payload.sync.frameIdGap, 0);
  assert.equal(fixture.frameMessages[0].payload.sync.canDrawPrecise, true);
  assert.equal(fixture.runtime.diagnose().modelFrameId, 100);
  fixture.runtime.destroy();
});

test("stable Navi and model geometry are retained in the Worker instead of cloned every frame", () => {
  let nowMs = 0;
  let presented;
  const model = {
    frameId: 100,
    position: { x: [0, 40], y: [0, 1], z: [0, 0] },
  };
  const navi = {
    publishMonoTimeNanos: 1_000,
    sessionId: "route-a",
    route: { polyline: [{ latitude: 37.5, longitude: 127.0 }] },
  };
  const fixture = runtimeFixture("three", {
    target: { performance: { now: () => nowMs } },
    stage: { stageWidth: 1280, stageHeight: 720 },
    frameSync: {
      subscribePresented(callback) { presented = callback; return () => {}; },
      selectModel() { return model; },
    },
    provider: {
      snapshot() {
        return {
          receivedAtMonotonic: {},
          hudState: {},
          overlayState: { modelV2: model, carrotNavi: navi },
        };
      },
    },
    workerFrameReply: () => ({
      type: "drawn", ok: true,
      performance: { level: "target", fps: 30, failed: false },
    }),
  });

  fixture.runtime.activate();
  assert.ok(fixture.frameMessages[0].payload.sourceUpdate.navi);
  assert.ok(fixture.frameMessages[0].payload.sourceUpdate.modelPosition);
  nowMs = 40;
  presented({ source: "live", sequence: 1 });
  assert.equal(fixture.frameMessages[1].payload.sourceUpdate, undefined);
  assert.deepEqual(fixture.runtime.status().worker.sourceTransfers, {
    navi: 1,
    modelPosition: 1,
    reusedNavi: 1,
    reusedModelPosition: 1,
  });

  navi.publishMonoTimeNanos = 2_000;
  model.frameId = 101;
  nowMs = 80;
  presented({ source: "live", sequence: 2 });
  assert.equal(fixture.frameMessages[2].payload.sourceUpdate.navi, navi);
  assert.equal(fixture.frameMessages[2].payload.sourceUpdate.modelPosition.coordinateFrame, "route-local-flu");
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
