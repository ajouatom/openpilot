import assert from "node:assert/strict";
import test from "node:test";

import { AR_COORDINATE_FRAME } from "../src/features/drive/contents/vision/ar/coordinate_frames.js";
import {
  createDeviceWorldPose,
  devicePointToWorld,
  quaternionFromEuler,
  quaternionFromRotationVector,
  quaternionToEuler,
  quaternionToRotationMatrix,
  worldPointToDevice,
} from "../src/features/drive/contents/vision/ar/world_pose.js";

const close = (actual, expected, tolerance = 1e-9) => {
  assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} != ${expected}`);
};

function odometry(trans, rot = [0, 0, 0], timestampNs = null) {
  return {
    trans,
    rot,
    targetTimestampNs: timestampNs,
    coordinateFrame: AR_COORDINATE_FRAME.DEVICE_FRD,
  };
}

function measurement(values, std = [0.01, 0.01, 0.01], valid = true) {
  return {
    x: values[0], y: values[1], z: values[2],
    xStd: std[0], yStd: std[1], zStd: std[2], valid,
  };
}

function livePose(timestamp, options = {}) {
  return {
    orientationNED: measurement(options.orientation ?? [0, 0, 0], options.orientationStd),
    velocityDevice: measurement(options.velocity ?? [10, 0, 0], options.velocityStd),
    accelerationDevice: measurement(options.acceleration ?? [0, 0, 0], options.accelerationStd),
    angularVelocityDevice: measurement(options.angularVelocity ?? [0, 0, 0], options.angularVelocityStd),
    inputsOK: options.inputsOK ?? true,
    posenetOK: options.posenetOK ?? true,
    sensorsOK: options.sensorsOK ?? true,
    timestamp,
  };
}

function geographicObservation(key, latitude, longitude, overrides = {}) {
  return {
    key,
    valid: true,
    latitude,
    longitude,
    headingDeg: 90,
    positionSigmaM: 2,
    headingSigmaDeg: 2,
    yawUsable: true,
    ...overrides,
  };
}

function longitudeAtEastM(latitude, longitude, meters) {
  return longitude + meters / (111320 * Math.cos(latitude * Math.PI / 180));
}

test("livePose Euler orientation uses openpilot Z-Y-X quaternion order", () => {
  const quaternion = quaternionFromEuler([0, 0, Math.PI / 2]);
  close(quaternion[0], Math.SQRT1_2);
  close(quaternion[1], 0);
  close(quaternion[2], 0);
  close(quaternion[3], Math.SQRT1_2);
});

test("livePose establishes a continuous NED reference then corrects orientation drift", () => {
  const tracker = createDeviceWorldPose({
    epoch: "live-pose-orientation",
    limits: { maxIntegrationStepMs: 1100 },
  });
  tracker.update({
    timestampNs: 1_000_000_000,
    odometry: odometry([0, 0, 0]),
    livePose: livePose(1_000_000_000, { orientation: [0, 0, 1.0], velocity: [0, 0, 0] }),
  });
  const initial = tracker.status();
  tracker.update({
    timestampNs: 2_000_000_000,
    odometry: odometry([0, 0, 0]),
    livePose: livePose(2_000_000_000, { orientation: [0, 0, 1.1], velocity: [0, 0, 0] }),
  });
  const corrected = tracker.status();

  assert.deepEqual(initial.orientation, [1, 0, 0, 0]);
  assert.equal(initial.fusion.orientationReferenceReady, true);
  assert.equal(corrected.fusion.orientationAccepted, 1);
  assert.ok(corrected.orientation[3] > 0);
  assert.ok(corrected.fusion.lastOrientationInnovationRad > 0.09);
});

test("fresh filtered livePose velocity and angular velocity assist Device prediction", () => {
  const tracker = createDeviceWorldPose({
    epoch: "live-pose-motion",
    limits: { maxIntegrationStepMs: 1100 },
  });
  tracker.update({
    timestampNs: 1_000_000_000,
    odometry: odometry([10, 0, 0]),
    livePose: livePose(1_000_000_000, { velocity: [14, 0, 0], angularVelocity: [0, 0, 0.2] }),
  });
  tracker.update({
    timestampNs: 2_000_000_000,
    odometry: odometry([10, 0, 0]),
    livePose: livePose(2_000_000_000, {
      velocity: [14, 0, 0],
      angularVelocity: [0, 0, 0.2],
      acceleration: [1, 0, 0],
    }),
  });
  const pose = tracker.status();

  assert.ok(pose.position[0] > 11 && pose.position[0] < 12);
  assert.equal(pose.fusion.velocityAccepted, 1);
  assert.equal(pose.fusion.angularVelocityAccepted, 1);
  assert.equal(pose.fusion.accelerationAccepted, 1);
  close(pose.fusion.lastVelocityInnovationMps, 4);
  close(pose.fusion.lastAngularInnovationRadps, 0.2);
});

test("stale or implausible livePose cannot perturb camera odometry integration", () => {
  const tracker = createDeviceWorldPose({
    epoch: "live-pose-gate",
    limits: { maxIntegrationStepMs: 1100 },
  });
  tracker.update({
    timestampNs: 1_000_000_000,
    odometry: odometry([10, 0, 0]),
    livePose: livePose(100_000_000, { velocity: [14, 0, 0] }),
  });
  tracker.update({
    timestampNs: 2_000_000_000,
    odometry: odometry([10, 0, 0]),
    livePose: livePose(100_000_000, { velocity: [14, 0, 0] }),
  });
  const pose = tracker.status();

  close(pose.position[0], 10);
  assert.equal(pose.fusion.velocityAccepted, 0);
  assert.equal(pose.fusion.orientationAccepted, 0);
  assert.equal(pose.fusion.lastLivePoseFresh, false);
});

test("Device world pose integrates 10m forward while its fixed anchor stays unchanged", () => {
  const tracker = createDeviceWorldPose({ epoch: "route-a" });
  let timestampNs = 1_000_000_000;
  tracker.update({ timestampNs, odometry: odometry([10, 0, 0]) });
  const worldAnchor = Object.freeze([40, 0, 0]);
  for (let index = 0; index < 20; index += 1) {
    timestampNs += 50_000_000;
    tracker.update({ timestampNs, odometry: odometry([10, 0, 0]) });
  }
  const pose = tracker.status();
  const relative = worldPointToDevice(pose, worldAnchor);

  close(pose.position[0], 10);
  close(relative[0], 30);
  assert.deepEqual(worldAnchor, [40, 0, 0]);
  assert.equal(pose.integrations, 20);
  assert.equal(pose.worldCoordinateFrame, AR_COORDINATE_FRAME.LOCAL_WORLD_DEVICE);
});

test("positive Device-down yaw moves a fixed forward marker to camera-left", () => {
  const tracker = createDeviceWorldPose({
    epoch: "turn",
    limits: { maxIntegrationStepMs: 1100 },
  });
  tracker.update({ timestampNs: 1_000_000_000, odometry: odometry([0, 0, 0]) });
  tracker.update({
    timestampNs: 2_000_000_000,
    odometry: odometry([0, 0, 0], [0, 0, Math.PI / 2]),
  });
  const pose = tracker.status();
  const relative = worldPointToDevice(pose, [10, 0, 0]);

  close(relative[0], 0, 1e-8);
  close(relative[1], -10, 1e-8);
  assert.deepEqual(devicePointToWorld(pose, relative).map((value) => +value.toFixed(8)), [10, 0, 0]);
});

test("translation uses midpoint orientation instead of end-frame Euler addition", () => {
  const tracker = createDeviceWorldPose({
    epoch: "curve",
    limits: { maxIntegrationStepMs: 1100 },
  });
  tracker.update({ timestampNs: 1_000_000_000, odometry: odometry([10, 0, 0]) });
  tracker.update({
    timestampNs: 2_000_000_000,
    odometry: odometry([10, 0, 0], [0, 0, Math.PI / 2]),
  });
  const expected = 10 / Math.sqrt(2);

  close(tracker.status().position[0], expected, 1e-8);
  close(tracker.status().position[1], expected, 1e-8);
});

test("the same presented timestamp is idempotent and a large gap is not invented as motion", () => {
  const tracker = createDeviceWorldPose({ epoch: "gap" });
  tracker.update({ timestampNs: 1_000_000_000, odometry: odometry([20, 0, 0]) });
  const duplicate = tracker.update({ timestampNs: 1_000_000_000, odometry: odometry([20, 0, 0]) });
  const gap = tracker.update({ timestampNs: 1_400_000_000, odometry: odometry([20, 0, 0]) });
  const resumed = tracker.update({ timestampNs: 1_450_000_000, odometry: odometry([20, 0, 0]) });

  assert.equal(duplicate.reason, "same presented frame");
  assert.equal(gap.accepted, false);
  assert.match(gap.reason, /integration gap 400ms/);
  close(resumed.position[0], 1);
  assert.equal(resumed.skippedGaps, 1);
});

test("world pose rejects FLU input instead of silently flipping it twice", () => {
  const tracker = createDeviceWorldPose();
  assert.throws(() => tracker.update({
    timestampNs: 1_000_000_000,
    odometry: {
      ...odometry([10, 0, 0]),
      coordinateFrame: AR_COORDINATE_FRAME.ROUTE_FLU,
    },
  }), /openpilot-device-frd/);
});

test("T_world_device is row-major, rigid, and contains no invented camera translation", () => {
  const tracker = createDeviceWorldPose({
    epoch: "matrix",
    limits: { maxIntegrationStepMs: 1100 },
  });
  tracker.update({ timestampNs: 1_000_000_000, odometry: odometry([2, 0, 0]) });
  tracker.update({ timestampNs: 2_000_000_000, odometry: odometry([2, 0, 0]) });
  const pose = tracker.status();
  const matrix = pose.matrixWorldFromDevice;

  assert.deepEqual(matrix, [
    1, 0, 0, 2,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1,
  ]);
  assert.equal(Object.hasOwn(pose, "cameraTranslation"), false);
});

test("quaternion exponential stays unit length and produces an orthonormal rotation", () => {
  const quaternion = quaternionFromRotationVector([0.2, -0.3, 0.4]);
  const rotation = quaternionToRotationMatrix(quaternion);
  close(Math.hypot(...quaternion), 1, 1e-12);
  for (let row = 0; row < 3; row += 1) {
    close(Math.hypot(...rotation[row]), 1, 1e-12);
    for (let other = row + 1; other < 3; other += 1) {
      close(rotation[row].reduce((sum, value, index) => sum + value * rotation[other][index], 0), 0, 1e-12);
    }
  }
});

test("navigation epoch reset returns Device pose to the shared local origin", () => {
  const tracker = createDeviceWorldPose({
    epoch: "route-a",
    limits: { maxIntegrationStepMs: 1100 },
  });
  tracker.update({ timestampNs: 1_000_000_000, odometry: odometry([5, 0, 0]) });
  tracker.update({ timestampNs: 2_000_000_000, odometry: odometry([5, 0, 0]) });
  const reset = tracker.reset({ epoch: "route-b", reason: "navigation session changed" });

  assert.equal(reset.epoch, "route-b");
  assert.equal(reset.initialized, false);
  assert.deepEqual(reset.position, [0, 0, 0]);
  assert.deepEqual(reset.orientation, [1, 0, 0, 0]);
  assert.match(reset.reason, /navigation session changed/);
});

test("TMap position corrects Device world pose gradually without snapping", () => {
  const latitude = 37.5;
  const longitude = 127;
  const tracker = createDeviceWorldPose({
    epoch: "geo-position",
    limits: { maxIntegrationStepMs: 1100 },
  });
  tracker.update({
    timestampNs: 1_000_000_000,
    odometry: odometry([10, 0, 0]),
    geographicObservation: geographicObservation("geo-1", latitude, longitude),
  });
  const corrected = tracker.update({
    timestampNs: 2_000_000_000,
    odometry: odometry([10, 0, 0]),
    geographicObservation: geographicObservation(
      "geo-2",
      latitude,
      longitudeAtEastM(latitude, longitude, 12),
    ),
  });

  assert.ok(corrected.position[0] > 10.2 && corrected.position[0] <= 10.25 + 1e-9);
  assert.equal(corrected.geoCorrection.referenceReady, true);
  assert.equal(corrected.geoCorrection.accepted, 1);
  assert.ok(corrected.geoCorrection.pendingPosition[0] > 0);

  const continued = tracker.update({
    timestampNs: 3_000_000_000,
    odometry: odometry([0, 0, 0]),
    geographicObservation: geographicObservation(
      "geo-2",
      latitude,
      longitudeAtEastM(latitude, longitude, 12),
    ),
  });
  assert.ok(continued.position[0] > corrected.position[0]);
  assert.ok(continued.position[0] < 11);
  assert.equal(continued.geoCorrection.appliedFrames, 2);
});

test("duplicate presented frames cannot consume a pending geographic correction", () => {
  const latitude = 37.5;
  const longitude = 127;
  const tracker = createDeviceWorldPose({
    epoch: "geo-idempotent",
    limits: { maxIntegrationStepMs: 1100 },
  });
  tracker.update({
    timestampNs: 1_000_000_000,
    odometry: odometry([10, 0, 0]),
    geographicObservation: geographicObservation("geo-1", latitude, longitude),
  });
  const corrected = tracker.update({
    timestampNs: 2_000_000_000,
    odometry: odometry([10, 0, 0]),
    geographicObservation: geographicObservation(
      "geo-2",
      latitude,
      longitudeAtEastM(latitude, longitude, 12),
    ),
  });
  const duplicate = tracker.update({
    timestampNs: 2_000_000_000,
    odometry: odometry([10, 0, 0]),
    geographicObservation: geographicObservation(
      "geo-2",
      latitude,
      longitudeAtEastM(latitude, longitude, 12),
    ),
  });

  assert.equal(duplicate.reason, "same presented frame");
  assert.deepEqual(duplicate.position, corrected.position);
  assert.deepEqual(duplicate.geoCorrection.pendingPosition, corrected.geoCorrection.pendingPosition);
});

test("large TMap innovation is rejected instead of moving the local world", () => {
  const latitude = 37.5;
  const longitude = 127;
  const tracker = createDeviceWorldPose({
    epoch: "geo-reject",
    limits: { maxIntegrationStepMs: 1100 },
  });
  tracker.update({
    timestampNs: 1_000_000_000,
    odometry: odometry([10, 0, 0]),
    geographicObservation: geographicObservation("geo-1", latitude, longitude),
  });
  const rejected = tracker.update({
    timestampNs: 2_000_000_000,
    odometry: odometry([10, 0, 0]),
    geographicObservation: geographicObservation(
      "geo-2",
      latitude,
      longitudeAtEastM(latitude, longitude, 100),
    ),
  });

  close(rejected.position[0], 10);
  assert.equal(rejected.geoCorrection.accepted, 0);
  assert.equal(rejected.geoCorrection.rejected, 1);
  assert.match(rejected.geoCorrection.lastReason, /innovation rejected/);
});

test("trusted TMap heading applies only a bounded yaw correction and reset clears its reference", () => {
  const latitude = 37.5;
  const longitude = 127;
  const tracker = createDeviceWorldPose({
    epoch: "geo-yaw",
    limits: { maxIntegrationStepMs: 1100 },
  });
  tracker.update({
    timestampNs: 1_000_000_000,
    odometry: odometry([0, 0, 0]),
    geographicObservation: geographicObservation("geo-1", latitude, longitude),
  });
  const corrected = tracker.update({
    timestampNs: 2_000_000_000,
    odometry: odometry([0, 0, 0], [0, 0, 5 * Math.PI / 180]),
    geographicObservation: geographicObservation("geo-2", latitude, longitude, { headingDeg: 100 }),
  });
  const yawDeg = quaternionToEuler(corrected.orientation)[2] * 180 / Math.PI;

  assert.ok(yawDeg > 5 && yawDeg <= 5.5 + 1e-9);
  assert.equal(corrected.geoCorrection.yawAccepted, 1);
  assert.ok(corrected.geoCorrection.pendingYawRad > 0);

  const reset = tracker.reset({ epoch: "geo-yaw-next" });
  assert.equal(reset.geoCorrection.referenceReady, false);
  assert.deepEqual(reset.geoCorrection.pendingPosition, [0, 0, 0]);
  assert.equal(reset.geoCorrection.pendingYawRad, 0);
});
