import assert from "node:assert/strict";
import test from "node:test";

import {
  AR_SYNC_STATE,
  evaluateFrameSync,
} from "../src/features/drive/contents/vision/ar/frame_sync.js";

function freshInput() {
  return {
    nowMs: 10_000,
    receivedAtMonotonic: {
      modelV2: 9_950,
      cameraOdometry: 9_950,
      livePose: 9_950,
      liveCalibration: 8_000,
      carrotNavi: 9_000,
    },
    roadCameraState: { frameId: 100, timestampEof: 10_000_000_000 },
    modelV2: { frameId: 100, frameAge: 1 },
    cameraOdometry: {
      frameId: 100,
      timestampEof: 10_000_000_000,
      trans: [10, 0, 0],
      rot: [0, 0, 0.01],
      transStd: [0.1, 0.01, 0.01],
      rotStd: [0.01, 0.01, 0.01],
    },
    livePose: {
      timestamp: 10_000_000_000,
      orientationNED: { x: 0, y: 0, z: 0 },
      inputsOK: true,
      posenetOK: true,
      sensorsOK: true,
    },
    liveCalibration: { calStatus: "calibrated", rpyCalib: [0, 0.02, 0] },
    carrotNavi: {
      publishMonoTimeNanos: 9_000_000_000,
      connected: true,
      navigationStatus: { guidanceActive: true, offRoute: false },
    },
  };
}

test("fresh frame-linked sources allow precise draw and bounded hold", () => {
  const sync = evaluateFrameSync(freshInput());

  assert.equal(sync.state, AR_SYNC_STATE.OK);
  assert.equal(sync.canDrawPrecise, true);
  assert.equal(sync.canHoldAnchor, true);
  assert.equal(sync.naviUsable, true);
  assert.equal(sync.modelAgeMs, 50);
  assert.equal(sync.odometryAgeMs, 100);
  assert.equal(sync.odometryPoseDelayMs, 100);
  assert.equal(sync.naviAgeMs, 1_000);
});

test("presented camera time evaluates the delayed odometry observation instead of raw EOF", () => {
  const input = freshInput();
  input.presentedClock = {
    targetTimestampNs: 10_020_000_000,
    confidence: "exact-frame",
  };
  const sync = evaluateFrameSync(input);

  assert.equal(sync.presentedTargetTimestampNs, 10_020_000_000);
  assert.equal(sync.presentedClockConfidence, "exact-frame");
  assert.equal(sync.odometryAgeMs, 120);
  assert.equal(sync.canHoldAnchor, true);
});

test("presented frame id is the model sync target when live state has moved ahead", () => {
  const input = freshInput();
  input.modelV2.frameId = 97;
  input.presentedClock = {
    sourceFrameId: 97,
    targetTimestampNs: 10_000_000_000,
    confidence: "exact-frame",
  };
  const sync = evaluateFrameSync(input);

  assert.equal(sync.frameIdGap, 0);
  assert.equal(sync.canDrawPrecise, true);
});

test("a three-frame replay model gap remains inside the precise AR window", () => {
  const input = freshInput();
  input.modelV2.frameId = 97;
  const sync = evaluateFrameSync(input);

  assert.equal(sync.frameIdGap, 3);
  assert.equal(sync.state, AR_SYNC_STATE.OK);
  assert.equal(sync.canDrawPrecise, true);
});

test("a nine-frame model gap hides precise AR and enters bounded hold", () => {
  const input = freshInput();
  input.modelV2.frameId = 91;
  const sync = evaluateFrameSync(input);

  assert.equal(sync.frameIdGap, 9);
  assert.equal(sync.state, AR_SYNC_STATE.FRAME_GAP);
  assert.equal(sync.canDrawPrecise, false);
  assert.equal(sync.canHoldAnchor, true);
  assert.match(sync.reasons.join(" | "), /frame gap 9 > 8/);
});

test("a stale model receipt hides precise AR even when frame ids still match", () => {
  const input = freshInput();
  input.receivedAtMonotonic.modelV2 = 9_600;
  const sync = evaluateFrameSync(input);

  assert.equal(sync.state, AR_SYNC_STATE.STALE);
  assert.equal(sync.canDrawPrecise, false);
  assert.equal(sync.canHoldAnchor, true);
  assert.match(sync.reasons.join(" | "), /model age 400ms/);
});

test("replay media time does not expire paused frame receipts on wall clock", () => {
  const input = freshInput();
  input.clockDomain = "replay-media";
  input.nowMs = 30_000;
  input.receivedAtMonotonic = {
    modelV2: 1,
    cameraOdometry: 1,
    livePose: 1,
    liveCalibration: 1,
    carrotNavi: 1,
  };
  const sync = evaluateFrameSync(input);

  assert.equal(sync.clockDomain, "replay-media");
  assert.equal(sync.state, AR_SYNC_STATE.OK);
  assert.equal(sync.canDrawPrecise, true);
  assert.equal(sync.canHoldAnchor, true);
  assert.doesNotMatch(sync.reasons.join(" | "), /수신시각|age 30000ms/);
});

test("stale calibration is not replaced by the last known projection", () => {
  const input = freshInput();
  input.receivedAtMonotonic.liveCalibration = -15_000;
  const sync = evaluateFrameSync(input);

  assert.equal(sync.state, AR_SYNC_STATE.STALE);
  assert.equal(sync.calibrated, true);
  assert.equal(sync.canDrawPrecise, false);
  assert.match(sync.reasons.join(" | "), /calibration age 25000ms/);
});

test("calibrated status without a valid current rpy still hides precise AR", () => {
  const input = freshInput();
  input.liveCalibration.rpyCalib = [];
  const sync = evaluateFrameSync(input);

  assert.equal(sync.calibrated, true);
  assert.equal(sync.calibrationPlausible, false);
  assert.equal(sync.canDrawPrecise, false);
  assert.match(sync.reasons.join(" | "), /calibration rpy 비정상\/누락/);
});

test("a repeatedly received but old Navi payload cannot create markers", () => {
  const input = freshInput();
  input.receivedAtMonotonic.carrotNavi = 9_990;
  input.carrotNavi.publishMonoTimeNanos = 1_000_000_000;
  const sync = evaluateFrameSync(input);

  assert.equal(sync.state, AR_SYNC_STATE.OK);
  assert.equal(sync.canDrawPrecise, true);
  assert.equal(sync.naviUsable, false);
  assert.equal(sync.naviAgeMs, 9_000);
  assert.match(sync.reasons.join(" | "), /navi age 9000ms/);
});

test("missing odometry receipt disables hold without blocking a fresh anchor", () => {
  const input = freshInput();
  delete input.receivedAtMonotonic.cameraOdometry;
  const sync = evaluateFrameSync(input);

  assert.equal(sync.canDrawPrecise, true);
  assert.equal(sync.odometryUsable, false);
  assert.equal(sync.canHoldAnchor, false);
  assert.match(sync.reasons.join(" | "), /odometry 수신시각 없음/);
});
