import assert from "node:assert/strict";
import test from "node:test";

import {
  CAMERA_ODOMETRY_POSE_DELAY_MS,
  cameraOdometryObservationTimestampNs,
  createCameraOdometryTimeline,
  createPresentedFrameClockMapper,
} from "../src/features/drive/contents/vision/ar/pose_timeline.js";

function odometry(timestampEof, speed) {
  return {
    timestampEof,
    trans: [speed, 0, 0],
    rot: [0, 0, speed / 100],
    transStd: [0.1, 0.01, 0.01],
    rotStd: [0.01, 0.01, 0.01],
    coordinateFrame: "route-flu",
  };
}

test("cameraOdometry observation clock matches locationd's 100ms pose delay", () => {
  assert.equal(CAMERA_ODOMETRY_POSE_DELAY_MS, 100);
  assert.equal(
    cameraOdometryObservationTimestampNs(odometry(2_000_000_000, 10)),
    1_900_000_000,
  );
});

test("presented frame mapping prefers its cereal frame time and never invents wall time", () => {
  const mapper = createPresentedFrameClockMapper();
  const mapped = mapper.map({
    source: "live",
    sequence: 7,
    frameId: 200,
    cameraTimestampEof: 10_000_000_000,
    mediaTime: 2.5,
    clockMappingConfidence: "exact-frame",
  }, { domain: "live-monotonic", nowMs: 99_000 });
  assert.equal(mapped.domain, "cereal-monotonic");
  assert.equal(mapped.targetTimeMs, 10_000);
  assert.equal(mapped.mediaToCerealOffsetNs, 7_500_000_000);
  assert.equal(mapped.confidence, "exact-frame");

  mapper.reset();
  mapper.map({
    source: "replay",
    sequence: 7,
    frameId: 200,
    cameraTimestampEof: 10_000_000_000,
    mediaTime: 2.5,
    clockMappingConfidence: "exact-frame",
  }, { domain: "replay-media", nowMs: 2_500 });
  const estimated = mapper.map({
    source: "replay",
    sequence: 8,
    mediaTime: 2.55,
  }, { domain: "replay-media", nowMs: 2_550 });
  assert.equal(estimated.targetTimestampNs, 10_050_000_000);
  assert.equal(estimated.confidence, "estimated-frame");

  const unmapped = mapper.map(null, { domain: "live-monotonic", nowMs: 100_000 });
  assert.equal(unmapped.mapped, false);
  assert.equal(unmapped.targetTimeMs, null);
});

// replay 영상 프레임에는 cereal timestamp가 실려 오지 않는다. 그때 spatial clock이
// 영영 서지 않으면 world pose가 초기화되지 않고 tracking이 LOST에 머물러 AR이
// 통째로 사라진다. 같은 tick의 roadCameraState로 offset을 세워 그 사슬을 끊는다.
test("replay without frame-carried cereal time still gains a spatial clock from roadCameraState", () => {
  const mapper = createPresentedFrameClockMapper();
  const seeded = mapper.map(
    { source: "replay", sequence: 1, mediaTime: 2.5 },
    { domain: "replay-media", nowMs: 2_500 },
    { cameraTimestampEof: 10_000_000_000 },
  );
  assert.equal(seeded.targetTimestampNs, 10_000_000_000);
  assert.equal(seeded.confidence, "estimated-frame");
  assert.equal(seeded.mediaToCerealOffsetNs, 7_500_000_000);

  // 한 번 선 offset은 이후 프레임에서 참조 없이도 media 시각만으로 이어진다.
  const next = mapper.map(
    { source: "replay", sequence: 2, mediaTime: 2.55 },
    { domain: "replay-media", nowMs: 2_550 },
  );
  assert.equal(next.targetTimestampNs, 10_050_000_000);

  // seek 직후 cereal이 뒤처진 순간에 잡힌 나쁜 씨앗은 영구히 고정되면 안 된다.
  // 정상 지터를 넘는 불일치가 관측되면 한 번 재동기한다.
  const drifted = mapper.map(
    { source: "replay", sequence: 3, mediaTime: 3.0 },
    { domain: "replay-media", nowMs: 3_000 },
    { cameraTimestampEof: 11_000_000_000 },   // 함의 offset 8.0s (기존 7.5s에서 500ms 차)
  );
  assert.equal(drifted.mediaToCerealOffsetNs, 8_000_000_000);
  assert.equal(drifted.targetTimestampNs, 11_000_000_000);

  // 반대로 정상 지터(<250ms)에서는 offset을 흔들지 않는다 — 같은 영상 프레임 동안
  // spatial target이 고정되어야 하기 때문이다.
  const jittered = mapper.map(
    { source: "replay", sequence: 4, mediaTime: 3.05 },
    { domain: "replay-media", nowMs: 3_050 },
    { cameraTimestampEof: 11_100_000_000 },   // 함의 offset 8.05s (50ms 차)
  );
  assert.equal(jittered.mediaToCerealOffsetNs, 8_000_000_000);

  // presented 이벤트가 media 시각을 싣지 않아도(리플레이 컨트롤러가 항상 넣지는
  // 않는다) 동시 관측된 cereal 시각으로 spatial clock을 유지한다.
  const withoutMedia = mapper.map(
    { source: "replay", sequence: 5 },
    { domain: "replay-media", nowMs: 3_100 },
    { cameraTimestampEof: 11_200_000_000 },
  );
  assert.equal(withoutMedia.confidence, "estimated-frame");
  assert.equal(withoutMedia.targetTimestampNs, 11_200_000_000);

  // 근거가 하나도 없을 때만 unmapped로 남는다.
  const nothing = mapper.map(
    { source: "replay", sequence: 6 },
    { domain: "replay-media", nowMs: 3_200 },
  );
  assert.equal(nothing.confidence, "unmapped");
  assert.equal(nothing.targetTimestampNs, null);

  // seek 후에는 offset을 버리고 다시 세운다.
  mapper.reset();
  const afterSeek = mapper.map(
    { source: "replay", sequence: 3, mediaTime: 40 },
    { domain: "replay-media", nowMs: 40_000 },
  );
  assert.equal(afterSeek.targetTimestampNs, null);
  assert.equal(afterSeek.confidence, "media-frame");
});

test("delay-aware timeline chooses the sample whose observation matches the video frame", () => {
  const timeline = createCameraOdometryTimeline();
  timeline.push(odometry(1_000_000_000, 10)); // observation 0.9s
  timeline.push(odometry(1_100_000_000, 20)); // observation 1.0s

  const aligned = timeline.sampleAt(1_000_000_000);
  assert.equal(aligned.trans[0], 20);
  assert.equal(aligned.temporalAlignment.mode, "exact");
  assert.equal(aligned.temporalAlignment.poseDelayMs, 100);
  const expectedSpeed = 20;
  const rawTimestampSelectionError = Math.abs(10 - expectedSpeed);
  const delayAwareSelectionError = Math.abs(aligned.trans[0] - expectedSpeed);
  assert.ok(delayAwareSelectionError < rawTimestampSelectionError);
});

test("pose timeline interpolates bracketing samples and bounds extrapolation", () => {
  const timeline = createCameraOdometryTimeline({ maxExtrapolationMs: 120 });
  timeline.push(odometry(1_000_000_000, 10)); // observation 0.9s
  timeline.push(odometry(1_100_000_000, 20)); // observation 1.0s

  const middle = timeline.sampleAt(950_000_000);
  assert.equal(middle.trans[0], 15);
  assert.equal(middle.temporalAlignment.mode, "interpolated");
  assert.equal(middle.temporalAlignment.alpha, 0.5);

  assert.equal(timeline.sampleAt(1_121_000_000), null);
});
