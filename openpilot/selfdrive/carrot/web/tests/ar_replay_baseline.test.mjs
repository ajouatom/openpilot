import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

const fixtureUrl = new URL("./fixtures/ar_replay_baseline.json", import.meta.url);

async function readFixture() {
  return JSON.parse(await readFile(fixtureUrl, "utf8"));
}

test("the AR replay baseline pins the user-reviewed route and maneuver windows", async () => {
  const fixture = await readFixture();

  assert.equal(fixture.$schema, "carrot-ar-replay-baseline/v1");
  assert.equal(fixture.baseline.route, "00001d9f--28e0608712--0");
  assert.equal(fixture.baseline.capturedFromCommit, "ca2f48b8");
  assert.equal(fixture.baseline.nominalVideoFps, 20);
  assert.equal(fixture.baseline.visualReviewRequired, true);

  assert.deepEqual(
    fixture.reviewWindows.map(({ id, maneuver, startMs, endMs }) => ({ id, maneuver, startMs, endMs })),
    [
      { id: "left-turn-at-segment-start", maneuver: "left", startMs: 0, endMs: 8000 },
      { id: "right-turn-around-23s", maneuver: "right", startMs: 18000, endMs: 30000 },
    ],
  );
  assert.equal(fixture.reviewWindows[0].preRollAvailable, false);
  assert.equal(fixture.reviewWindows[1].preRollAvailable, true);
});

test("the baseline records every input needed for later frame and pose comparison", async () => {
  const fixture = await readFixture();
  const services = new Set(fixture.requiredServices);

  for (const service of [
    "roadCameraState",
    "modelV2",
    "cameraOdometry",
    "liveCalibration",
    "livePose",
    "carrotNavi",
  ]) {
    assert.equal(services.has(service), true, `missing baseline service ${service}`);
  }

  const issueCodes = new Set(fixture.reviewWindows.flatMap((window) => window.observedIssues));
  for (const issue of [
    "late-appearance",
    "mirror-or-axis-inversion",
    "vertical-placement",
    "spatial-jitter",
    "blink",
    "not-world-locked",
  ]) {
    assert.equal(issueCodes.has(issue), true, `missing observed issue ${issue}`);
  }
});

test("the baseline describes current faults without accepting them as target behavior", async () => {
  const fixture = await readFixture();
  const current = fixture.currentImplementation;
  const target = fixture.comparisonContract;

  assert.equal(current.openpilotCoordinateContract, "device-frd");
  assert.equal(current.coordinateLabel, "device-flu-in-ar-code");
  assert.equal(current.cameraOdometryPoseDelayAppliedMs, 0);
  assert.equal(current.presentationClock, "independent-raf-over-presented-frame");
  assert.equal(current.anchorModel, "mutable-vehicle-relative");

  assert.equal(target.sameRouteAndWindows, true);
  assert.equal(target.sameViewportAndDisplayMode, true);
  assert.equal(target.fakeCalibrationProbeAllowed, false);
  assert.equal(target.screenPixelClampAllowed, false);
  assert.match(target.sameArTokenPreview, /carrot_ar_token_preview\.html$/);
  assert.equal(target.manualChecks.length >= 5, true);
});
