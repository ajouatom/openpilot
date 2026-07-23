import assert from "node:assert/strict";
import test from "node:test";

import {
  compactArDiagnosticFrame,
  createArReplayDiagnosticCapture,
  formatArReplayDiagnosticReport,
  markerContinuitySummary,
} from "../src/features/drive/contents/vision/ar/debug/capture.js";

function packet(mediaTimeMs, options = {}) {
  return {
    traceFrameId: options.traceFrameId ?? Math.round(mediaTimeMs / 50) + 1,
    ok: options.drawn !== false,
    debugFrame: {
      replayTimeMs: mediaTimeMs,
      presented: { sequence: options.sequence ?? 1, frameId: options.frameId ?? 10 },
      presentedClock: {
        sourceFrameId: options.frameId ?? 10,
        confidence: options.mapped === false ? "unmapped" : "estimated-frame",
        mapped: options.mapped !== false,
        targetTimestampNs: options.mapped === false ? null : 1_000_000_000 + mediaTimeMs * 1_000_000,
      },
      sources: { cameraFrameId: 10, modelFrameId: 11, odometryFrameId: 10 },
      sync: {
        state: "ok", canDrawPrecise: true, canHoldAnchor: true,
        frameIdGap: 1, naviUsable: true, reasons: [],
      },
      navi: {
        present: true, sessionId: "route-a", guidanceActive: true, routePresent: true,
        vehicle: { present: true }, route: { present: true, pointCount: 3 },
        publishMonoTimeNanos: options.publishMonoTimeNanos,
      },
      positionQuality: { canUseGeo: true, canUseRoute: true, fallback: "geo" },
      worldEpoch: "replay|route-a",
    },
    tracking: {
      state: options.tracking || "tracking",
      canCreateAnchor: true, canPropagateAnchor: true, retainAnchor: true,
      alpha: 1, uncertainty: { lateralM: 0.1, budgetM: 2.5 }, reasons: [],
    },
    composition: {
      signCount: 1, anchoredCount: 1, sources: ["guidanceCurrent"],
      diag: { precise: true, route: true, routePts: 3, matchIdx: options.matchIdx ?? 1 },
    },
    hold: {
      state: options.hold || "live", sourceMode: "world-anchor", anchorCount: 1,
      unresolvedCount: 0,
      creationDeferredCount: options.creationGateReason ? 1 : 0,
      creationGateReason: options.creationGateReason || null,
      fixId: "fix-a", sourceId: "route-a", worldEpoch: "replay|route-a",
      sample: {
        x: 40 - mediaTimeMs / 1000, y: options.y ?? 1, z: 0,
        markerId: options.markerId || "turn-a",
        lifecycleSlot: "guidance:primary",
        source: "guidanceCurrent",
      },
      ...(options.handoff ? {
        handoffCount: 1,
        handoff: {
          state: "blending", progress: 0.5, durationMs: 1200,
          positionInnovationM: 2.4, orientationInnovationRad: 0.1,
          lateralInnovationM: 0.7, heightInnovationM: 2,
        },
      } : {}),
    },
    worldPose: {
      initialized: true, epoch: "replay|route-a", integrations: Math.round(mediaTimeMs / 50),
      position: [mediaTimeMs / 1000, 0, 0], orientation: [1, 0, 0, 0], reason: "integrated",
      geoCorrection: {
        referenceReady: true,
        accepted: 3,
        rejected: 1,
        yawAccepted: 2,
        yawRejected: 0,
        appliedFrames: 8,
        lastPositionInnovationM: 1.25,
        lastYawInnovationRad: 0.02,
        lastPositionSigmaM: 12,
        lastHeadingSigmaDeg: 18,
        lastReason: "geographic position correction accepted",
      },
    },
    renderer: {
      drawn: options.drawn === false ? 0 : 1, skipped: options.drawn === false ? 1 : 0,
      trackingAlpha: 1, minimumVisibilityAlpha: 1,
      markers: [{
        markerId: options.markerId || "turn-a",
        lifecycleSlot: "guidance:primary", source: "guidanceCurrent",
        kind: "turn-gate", distanceM: 40, visible: options.drawn !== false,
        visibilityState: options.drawn === false ? "active-offscreen" : "active-visible",
        anchor: { x: 40, y: options.y ?? 1, z: 0 },
        screen: { centerX: 600, centerY: 350, widthPx: 220, heightPx: 120, depth: 40 },
      }],
      visibilityStates: {
        [options.drawn === false ? "active-offscreen" : "active-visible"]: 1,
      },
    },
    performance: { fps: 30, averageWorkMs: 0.4, lastWorkMs: 0.5, slowFrames: 0, level: "target" },
    transport: {
      dpr: 1.5,
      queue: { posted: 25, completed: 24, deferred: 4, replaced: 2, discarded: 0, inFlight: true, pending: true, maxPending: 1 },
      sourceTransfers: { navi: 2, modelPosition: 8, reusedNavi: 23, reusedModelPosition: 17 },
    },
  };
}

test("AR diagnostic frame retains clock, anchor and projected marker coordinates", () => {
  const frame = compactArDiagnosticFrame(packet(1250, {
    y: -12.3456,
    handoff: true,
    creationGateReason: "geo-yaw-settling",
  }));

  assert.equal(frame.mediaTimeMs, 1250);
  assert.equal(frame.presented.confidence, "estimated-frame");
  assert.equal(frame.composition.gate.matchIdx, 1);
  assert.equal(frame.hold.sample.y, -12.346);
  assert.equal(frame.hold.sample.markerId, "turn-a");
  assert.equal(frame.hold.creationDeferredCount, 1);
  assert.equal(frame.hold.creationGateReason, "geo-yaw-settling");
  assert.deepEqual(frame.hold.handoff, {
    state: "blending", reason: undefined, progress: 0.5, durationMs: 1200,
    positionInnovationM: 2.4, orientationInnovationRad: 0.1,
    lateralInnovationM: 0.7, heightInnovationM: 2,
  });
  assert.deepEqual(frame.renderer.markers[0].screen, {
    centerX: 600, centerY: 350, widthPx: 220, heightPx: 120, depth: 40,
  });
  assert.deepEqual(frame.transport, {
    dpr: 1.5,
    queue: { posted: 25, completed: 24, deferred: 4, replaced: 2, discarded: 0, inFlight: true, pending: true, maxPending: 1 },
    sourceTransfers: { navi: 2, modelPosition: 8, reusedNavi: 23, reusedModelPosition: 17 },
  });
  assert.equal(frame.renderer.markers[0].visibilityState, "active-visible");
  assert.deepEqual(frame.renderer.visibilityStates, { "active-visible": 1 });
  assert.deepEqual(frame.worldPose.geoCorrection, {
    referenceReady: true,
    accepted: 3,
    rejected: 1,
    yawAccepted: 2,
    yawRejected: 0,
    appliedFrames: 8,
    positionInnovationM: 1.25,
    yawInnovationRad: 0.02,
    positionSigmaM: 12,
    headingSigmaDeg: 18,
    reason: "geographic position correction accepted",
  });
});

test("diagnostics ignore cross-marker displacement and unmapped zero-clock samples", async () => {
  let diagnosticListener = null;
  const downloads = [];
  const playback = {
    active: true, ready: true, loading: false,
    route: "route", segment: "segment--0", currentTime: 0,
    duration: 60, paused: true, ended: false, playbackRate: 1,
  };
  const control = {
    snapshot: () => ({ ...playback }),
    pause: () => { playback.paused = true; return true; },
    seek: (seconds) => { playback.currentTime = seconds; return true; },
    setRate: (rate) => { playback.playbackRate = rate; return true; },
    play: async () => { playback.paused = false; },
    restore: async () => true,
  };
  const capture = createArReplayDiagnosticCapture({
    target: {
      CarrotVisionReplay: { diagnosticPlayback: control, status: () => ({ ...playback }) },
      CarrotVisionAr: {
        subscribeDiagnostics(listener) {
          diagnosticListener = listener;
          return () => { diagnosticListener = null; };
        },
      },
      navigator: { userAgent: "test" },
    },
    setInterval: () => 1,
    clearInterval() {},
    download(filename, payload) { downloads.push({ filename, payload }); return true; },
  });

  assert.equal(await capture.start(), true);
  diagnosticListener(packet(0, { markerId: "turn-a", y: 1 }));
  diagnosticListener(packet(50, {
    markerId: "turn-b", y: 100, mapped: false,
    publishMonoTimeNanos: 90_000_000_000,
  }));
  assert.equal(await capture.stop(), true);
  assert.equal(downloads[0].payload.summary.eventTypes["anchor-position-jump"], undefined);
  assert.equal(downloads[0].payload.summary.navi.clockMismatchFrames, 0);
  assert.equal(downloads[0].payload.summary.flags.includes("mixed-clock-domain"), false);
  assert.equal(downloads[0].payload.frames[1].presented.targetTimestampNs, null);
});

test("blink detection follows one marker identity instead of global scene occupancy", () => {
  const frame = (mediaTimeMs, markerId, visible) => ({
    mediaTimeMs,
    renderer: {
      markers: [{ markerId, visible }],
    },
  });
  const normalSceneChanges = markerContinuitySummary([
    frame(0, "turn-a", true),
    frame(50, "turn-a", false),
    frame(1000, "turn-b", true),
    frame(1050, "turn-b", false),
    frame(3000, "lane-c", true),
  ]);
  assert.deepEqual(normalSceneChanges.blinkingIds, []);

  const sameMarkerBlink = markerContinuitySummary([
    frame(0, "turn-a", true),
    frame(50, "turn-a", false),
    frame(100, "turn-a", true),
    frame(150, "turn-a", false),
  ]);
  assert.deepEqual(sameMarkerBlink.blinkingIds, ["turn-a"]);
  assert.equal(sameMarkerBlink.maxRapidTransitions, 3);
});

test("one-click replay capture saves one file and restores the prior playback state", async () => {
  let diagnosticListener = null;
  let intervalCallback = null;
  let restored = null;
  const downloads = [];
  const playback = {
    active: true, ready: true, loading: false,
    route: "route", segment: "segment--0", currentTime: 19.288,
    duration: 60.04, paused: true, ended: false, playbackRate: 2,
  };
  const control = {
    snapshot: () => ({ ...playback }),
    pause: () => { playback.paused = true; return true; },
    seek: (seconds) => { playback.currentTime = seconds; return true; },
    setRate: (rate) => { playback.playbackRate = rate; return true; },
    play: async () => { playback.paused = false; },
    restore: async (snapshot) => { restored = snapshot; Object.assign(playback, snapshot); return true; },
  };
  const target = {
    CarrotVisionReplay: {
      diagnosticPlayback: control,
      status: () => ({ ...playback }),
    },
    CarrotVisionAr: {
      subscribeDiagnostics(listener) {
        diagnosticListener = listener;
        return () => { diagnosticListener = null; };
      },
    },
    CarrotDriveLiveStateProvider: {
      snapshot: () => ({ overlayState: { carrotNavi: {
        sessionId: "route-a",
        route: { polyline: [
          { latitude: 37.5, longitude: 127 },
          { latitude: 37.5001, longitude: 127.0001 },
        ] },
      } } }),
    },
    navigator: { userAgent: "test" },
    innerWidth: 1200,
    innerHeight: 800,
    devicePixelRatio: 1,
  };
  const capture = createArReplayDiagnosticCapture({
    target,
    setInterval(callback) { intervalCallback = callback; return 1; },
    clearInterval() { intervalCallback = null; },
    download(filename, payload) { downloads.push({ filename, payload }); return true; },
  });

  assert.equal(await capture.start(), true);
  assert.equal(playback.currentTime, 0);
  assert.equal(playback.playbackRate, 1);
  diagnosticListener(packet(0));
  diagnosticListener(packet(50, { tracking: "lost", drawn: false, y: 12 }));
  assert.equal(capture.snapshot().frames, 2);
  assert.equal(typeof intervalCallback, "function");

  assert.equal(await capture.stop(), true);
  assert.equal(downloads.length, 1);
  assert.equal(downloads[0].payload.kind, "carrot-ar-replay-diagnostic");
  assert.equal(downloads[0].payload.frames.length, 2);
  assert.equal(downloads[0].payload.summary.eventTypes["tracking-transition"], 1);
  assert.equal(downloads[0].payload.summary.eventTypes["marker-hidden"], 1);
  assert.equal(downloads[0].payload.summary.eventTypes["anchor-position-jump"], 1);
  assert.equal(downloads[0].payload.summary.worldPose.geoAccepted, 3);
  assert.equal(downloads[0].payload.summary.worldPose.geoRejected, 1);
  assert.equal(downloads[0].payload.summary.worldPose.geoAppliedFrames, 8);
  assert.deepEqual(downloads[0].payload.summary.markerStates, {
    "active-visible": 1,
    "active-offscreen": 1,
  });
  assert.equal(downloads[0].payload.routeGeometries.length, 1);
  assert.equal(downloads[0].payload.summary.coverage.complete, false);
  assert.match(downloads[0].payload.report, /AR 리플레이 진단 요약/);
  assert.match(capture.snapshot().lastResult.report, /capture-incomplete/);
  assert.equal(restored.currentTime, 19.288);
  assert.equal(restored.playbackRate, 2);
  assert.equal(restored.paused, true);
});

test("capture waits for the renderer tail and never labels a short prefix complete", async () => {
  let diagnosticListener = null;
  let intervalCallback = null;
  let nowMs = 0;
  const downloads = [];
  const playback = {
    active: true, ready: true, loading: false,
    route: "route", segment: "segment--0", currentTime: 0,
    duration: 60, paused: true, ended: false, playbackRate: 1,
  };
  const control = {
    snapshot: () => ({ ...playback }),
    pause: () => { playback.paused = true; return true; },
    seek: (seconds) => { playback.currentTime = seconds; return true; },
    setRate: (rate) => { playback.playbackRate = rate; return true; },
    play: async () => { playback.paused = false; },
    restore: async () => true,
  };
  const capture = createArReplayDiagnosticCapture({
    target: {
      CarrotVisionReplay: { diagnosticPlayback: control, status: () => ({ ...playback }) },
      CarrotVisionAr: {
        subscribeDiagnostics(listener) {
          diagnosticListener = listener;
          return () => { diagnosticListener = null; };
        },
      },
      navigator: { userAgent: "test" },
    },
    now: () => nowMs,
    setInterval(callback) { intervalCallback = callback; return 1; },
    clearInterval() { intervalCallback = null; },
    download(filename, payload) { downloads.push({ filename, payload }); return true; },
  });

  assert.equal(await capture.start(), true);
  diagnosticListener(packet(0));
  diagnosticListener(packet(18_831));
  playback.currentTime = 60;
  intervalCallback();
  assert.equal(downloads.length, 0);
  assert.equal(playback.paused, true);

  nowMs = 2501;
  intervalCallback();
  await new Promise((resolve) => setImmediate(resolve));
  assert.equal(downloads.length, 1);
  assert.equal(downloads[0].payload.stopReason, "incomplete-render-tail");
  assert.equal(downloads[0].payload.replay.captureEnd, 18.831);
  assert.equal(downloads[0].payload.summary.coverage.complete, false);
  assert.equal(capture.snapshot().phase, "warning");
});

test("pasteable report keeps counts and representative times without raw frames", () => {
  const report = formatArReplayDiagnosticReport({
    stopReason: "complete",
    replay: { segment: "route--0" },
    summary: {
      frames: 1200, events: 2,
      coverage: { complete: true, firstMediaTimeMs: 0, lastMediaTimeMs: 60_000, requestedEndMs: 60_000, effectiveFps: 20 },
      transitions: { naviUsable: 1, visibility: 2, tracking: 1, hold: 1 },
      navi: { usableFrames: 1100, unusableFrames: 100, clockMismatchFrames: 0 },
      worldPose: { integratedFrames: 1000, maxIntegrations: 999 },
      anchorRange: { x: [1, 200], y: [-3, 4] },
      flags: [], reasons: { tracking: { lost: 3 }, sync: {} }, eventTypes: { "marker-hidden": 2 },
    },
    events: [{ type: "marker-hidden", mediaTimeMs: 12_340 }],
    frames: Array.from({ length: 1200 }),
  });
  assert.match(report, /수집 완료/);
  assert.match(report, /12\.34초 marker-hidden/);
  assert.doesNotMatch(report, /"frames"/);
  assert.ok(report.split("\n").length < 40);
});
