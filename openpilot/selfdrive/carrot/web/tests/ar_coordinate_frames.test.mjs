import assert from "node:assert/strict";
import test from "node:test";

import {
  AR_COORDINATE_FRAME,
  AR_LIVE_POSE_FRAMES,
  assertCoordinateFrame,
  deviceFrdVectorToRouteFlu,
  routeFluVectorToDeviceFrd,
  modelPositionFrdToRouteFlu,
  deviceOdometryFrdToRouteFlu,
} from "../src/features/drive/contents/vision/ar/coordinate_frames.js";

test("openpilot FRD and AR route FLU basis signs are explicit and reversible", () => {
  assert.deepEqual(deviceFrdVectorToRouteFlu([1, 0, 0]), [1, 0, 0]);
  assert.deepEqual(deviceFrdVectorToRouteFlu([0, 1, 0]), [0, -1, 0]);
  assert.deepEqual(deviceFrdVectorToRouteFlu([0, 0, 1]), [0, 0, -1]);
  assert.deepEqual(routeFluVectorToDeviceFrd(deviceFrdVectorToRouteFlu([4, -2, 3])), [4, -2, 3]);
});

test("model position crosses from Calibrated FRD into route FLU once", () => {
  const source = { x: [0, 10], y: [1, -2], z: [0.5, -1] };
  const converted = modelPositionFrdToRouteFlu(source);

  assert.equal(converted.coordinateFrame, AR_COORDINATE_FRAME.ROUTE_FLU);
  assert.equal(converted.sourceCoordinateFrame, AR_COORDINATE_FRAME.CALIBRATED_FRD);
  assert.equal(converted.x, source.x, "forward samples are shared without an allocation");
  assert.deepEqual(converted.y, [-1, 2]);
  assert.deepEqual(converted.z, [-0.5, 1]);
});

test("Device FRD odometry crosses into route FLU without flipping uncertainty", () => {
  const converted = deviceOdometryFrdToRouteFlu({
    trans: [12, 2, -1],
    rot: [0.1, -0.2, 0.3],
    transStd: [0.4, 0.2, 0.1],
    rotStd: [0.04, 0.02, 0.01],
    coordinateFrame: AR_COORDINATE_FRAME.DEVICE_FRD,
  });

  assert.deepEqual(converted.trans, [12, -2, 1]);
  assert.deepEqual(converted.rot, [0.1, 0.2, -0.3]);
  assert.deepEqual(converted.transStd, [0.4, 0.2, 0.1]);
  assert.deepEqual(converted.rotStd, [0.04, 0.02, 0.01]);
  assert.equal(converted.coordinateFrame, AR_COORDINATE_FRAME.ROUTE_FLU);
});

test("declared module boundaries reject missing or mismatched coordinate frames", () => {
  assert.throws(
    () => assertCoordinateFrame({}, AR_COORDINATE_FRAME.DEVICE_FRD, "sample"),
    /sample coordinateFrame/,
  );
  assert.throws(
    () => deviceOdometryFrdToRouteFlu({
      trans: [1, 0, 0],
      rot: [0, 0, 0],
      coordinateFrame: AR_COORDINATE_FRAME.CALIBRATED_FRD,
    }),
    /cameraOdometry coordinateFrame/,
  );
  assert.deepEqual(AR_LIVE_POSE_FRAMES, {
    orientationNED: AR_COORDINATE_FRAME.NED,
    velocityDevice: AR_COORDINATE_FRAME.DEVICE_FRD,
    accelerationDevice: AR_COORDINATE_FRAME.DEVICE_FRD,
    angularVelocityDevice: AR_COORDINATE_FRAME.DEVICE_FRD,
  });
});
