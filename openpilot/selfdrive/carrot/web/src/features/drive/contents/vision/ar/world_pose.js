/* Device-frame SE(3) dead reckoning for Carrot AR.
 *
 * State is T_world_device: a fixed navigation-session world origin and the
 * moving openpilot Device frame. The initial world axes equal Device FRD; a
 * later livePose/geo phase may align this world to NED/ENU explicitly.
 *
 * cameraOdometry must already be Calibrated FRD -> Device FRD. This module
 * never accepts route FLU, avoiding a sign flip round-trip before integration.
 */

import {
  AR_COORDINATE_FRAME,
  assertCoordinateFrame,
  routeFluVectorToDeviceFrd,
} from "./coordinate_frames.js";
import { enuOffset, toVehicleFrame } from "./geo.js";

export const AR_WORLD_POSE_LIMITS = Object.freeze({
  maxIntegrationStepMs: 250,
  maxLivePoseAgeMs: 180,
  maxOrientationStdRad: 0.35,
  maxOrientationInnovationRad: 0.45,
  orientationCorrectionGain: 0.08,
  maxVelocityStdMps: 5,
  maxVelocityInnovationMps: 8,
  velocityFusionGain: 0.35,
  maxAngularVelocityStdRadps: 0.5,
  maxAngularVelocityInnovationRadps: 0.7,
  angularVelocityFusionGain: 0.35,
  maxAccelerationStdMps2: 8,
  maxAccelerationMps2: 20,
  maxGeoPositionSigmaM: 25,
  maxGeoPositionInnovationM: 25,
  geoPositionCorrectionGain: 0.25,
  maxGeoPositionCorrectionM: 3,
  maxGeoPositionStepM: 0.25,
  maxGeoHeadingSigmaDeg: 25,
  maxGeoYawInnovationRad: 35 * Math.PI / 180,
  geoYawCorrectionGain: 0.2,
  maxGeoYawCorrectionRad: 4 * Math.PI / 180,
  maxGeoYawStepRad: 0.5 * Math.PI / 180,
});

const IDENTITY_QUATERNION = Object.freeze([1, 0, 0, 0]);
const ZERO_VECTOR = Object.freeze([0, 0, 0]);
const NS_PER_MS = 1e6;

function finite(value, fallback = null) {
  if (value === null || value === undefined || value === "") return fallback;
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function vector3(value) {
  if (!Array.isArray(value) || value.length < 3) return null;
  const out = value.slice(0, 3).map(Number);
  return out.every(Number.isFinite) ? out : null;
}

function quaternion4(value) {
  if (!Array.isArray(value) || value.length < 4) return null;
  const out = value.slice(0, 4).map(Number);
  return out.every(Number.isFinite) ? out : null;
}

export function normalizeQuaternion(value) {
  const quaternion = quaternion4(value);
  if (!quaternion) return null;
  const norm = Math.hypot(...quaternion);
  if (!(norm > 0)) return null;
  const out = quaternion.map((item) => item / norm);
  // q and -q encode the same rotation. A non-negative scalar component keeps
  // diagnostics/interpolation deterministic across the pi boundary.
  if (out[0] < 0) return Object.freeze(out.map((item) => -item));
  return Object.freeze(out);
}

export function multiplyQuaternions(leftValue, rightValue) {
  const left = quaternion4(leftValue);
  const right = quaternion4(rightValue);
  if (!left || !right) return null;
  const [aw, ax, ay, az] = left;
  const [bw, bx, by, bz] = right;
  return normalizeQuaternion([
    aw * bw - ax * bx - ay * by - az * bz,
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
  ]);
}

export function quaternionFromEuler(value) {
  const euler = vector3(value);
  if (!euler) return null;
  const [roll, pitch, yaw] = euler;
  const cr = Math.cos(roll / 2);
  const sr = Math.sin(roll / 2);
  const cp = Math.cos(pitch / 2);
  const sp = Math.sin(pitch / 2);
  const cy = Math.cos(yaw / 2);
  const sy = Math.sin(yaw / 2);
  return normalizeQuaternion([
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
  ]);
}

function conjugateQuaternion(value) {
  const quaternion = normalizeQuaternion(value);
  return quaternion
    ? Object.freeze([quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]])
    : null;
}

export function slerpQuaternions(fromValue, toValue, alphaValue) {
  const from = normalizeQuaternion(fromValue);
  let to = normalizeQuaternion(toValue);
  if (!from || !to) return null;
  const alpha = Math.max(0, Math.min(1, finite(alphaValue, 0)));
  let cosine = from.reduce((sum, value, index) => sum + value * to[index], 0);
  if (cosine < 0) {
    to = to.map((value) => -value);
    cosine = -cosine;
  }
  if (cosine > 0.9995) {
    return normalizeQuaternion(from.map((value, index) => value + (to[index] - value) * alpha));
  }
  const angle = Math.acos(Math.max(-1, Math.min(1, cosine)));
  const sine = Math.sin(angle);
  const fromScale = Math.sin((1 - alpha) * angle) / sine;
  const toScale = Math.sin(alpha * angle) / sine;
  return normalizeQuaternion(from.map((value, index) => value * fromScale + to[index] * toScale));
}

function quaternionDistanceRad(leftValue, rightValue) {
  const left = normalizeQuaternion(leftValue);
  const right = normalizeQuaternion(rightValue);
  if (!left || !right) return null;
  const dot = Math.abs(left.reduce((sum, value, index) => sum + value * right[index], 0));
  return 2 * Math.acos(Math.max(-1, Math.min(1, dot)));
}

function measurementVector(measurement, maxStd) {
  if (!measurement || measurement.valid !== true) return null;
  const value = vector3([measurement.x, measurement.y, measurement.z]);
  const std = vector3([measurement.xStd, measurement.yStd, measurement.zStd]);
  if (!value || !std || std.some((item) => item < 0 || item > maxStd)) return null;
  return Object.freeze({ value, std, worstStd: Math.max(...std) });
}

function vectorDistance(left, right) {
  return Math.hypot(...left.map((value, index) => value - right[index]));
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function wrapAngleRad(value) {
  return Math.atan2(Math.sin(value), Math.cos(value));
}

export function quaternionToEuler(value) {
  const quaternion = normalizeQuaternion(value);
  if (!quaternion) return null;
  const [w, x, y, z] = quaternion;
  return Object.freeze([
    Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
    Math.asin(clamp(2 * (w * y - z * x), -1, 1)),
    Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
  ]);
}

function blendVectors(primary, observation, gain) {
  return Object.freeze(primary.map((value, index) => (
    value + (observation[index] - value) * gain
  )));
}

function livePoseFresh(livePose, targetTimestampNs, limits) {
  const timestampNs = finite(livePose?.timestamp, null);
  return timestampNs !== null
    && timestampNs > 0
    && Math.abs(targetTimestampNs - timestampNs) / NS_PER_MS <= limits.maxLivePoseAgeMs;
}

function fusedMotion(odometry, livePose, targetTimestampNs, limits) {
  const velocity = vector3(odometry?.trans);
  const angularVelocity = vector3(odometry?.rot);
  const usable = livePose?.inputsOK === true
    && livePose?.posenetOK === true
    && livePose?.sensorsOK !== false
    && livePoseFresh(livePose, targetTimestampNs, limits);
  const poseVelocity = usable
    ? measurementVector(livePose.velocityDevice, limits.maxVelocityStdMps)
    : null;
  const poseAngular = usable
    ? measurementVector(livePose.angularVelocityDevice, limits.maxAngularVelocityStdRadps)
    : null;
  const poseAcceleration = usable
    ? measurementVector(livePose.accelerationDevice, limits.maxAccelerationStdMps2)
    : null;

  const velocityInnovation = velocity && poseVelocity
    ? vectorDistance(velocity, poseVelocity.value)
    : null;
  const angularInnovation = angularVelocity && poseAngular
    ? vectorDistance(angularVelocity, poseAngular.value)
    : null;
  const accelerationMagnitude = poseAcceleration ? Math.hypot(...poseAcceleration.value) : null;
  const velocityAccepted = velocityInnovation !== null
    && velocityInnovation <= limits.maxVelocityInnovationMps;
  const angularAccepted = angularInnovation !== null
    && angularInnovation <= limits.maxAngularVelocityInnovationRadps;
  const accelerationAccepted = accelerationMagnitude !== null
    && accelerationMagnitude <= limits.maxAccelerationMps2;

  return Object.freeze({
    velocity: velocityAccepted
      ? blendVectors(velocity, poseVelocity.value, limits.velocityFusionGain)
      : velocity,
    angularVelocity: angularAccepted
      ? blendVectors(angularVelocity, poseAngular.value, limits.angularVelocityFusionGain)
      : angularVelocity,
    velocityAccepted,
    angularAccepted,
    accelerationAccepted,
    velocityInnovationMps: velocityInnovation,
    angularInnovationRadps: angularInnovation,
    accelerationMagnitudeMps2: accelerationMagnitude,
    livePoseFresh: usable,
  });
}

export function quaternionFromRotationVector(value) {
  const rotation = vector3(value);
  if (!rotation) return null;
  const angle = Math.hypot(...rotation);
  if (angle < 1e-12) return IDENTITY_QUATERNION;
  const half = angle / 2;
  const scale = Math.sin(half) / angle;
  return normalizeQuaternion([
    Math.cos(half),
    rotation[0] * scale,
    rotation[1] * scale,
    rotation[2] * scale,
  ]);
}

export function rotateVectorByQuaternion(quaternionValue, vectorValue) {
  const q = normalizeQuaternion(quaternionValue);
  const vector = vector3(vectorValue);
  if (!q || !vector) return null;
  const [w, x, y, z] = q;
  const [vx, vy, vz] = vector;
  // q * [0,v] * conjugate(q), expanded without temporary quaternions.
  const tx = 2 * (y * vz - z * vy);
  const ty = 2 * (z * vx - x * vz);
  const tz = 2 * (x * vy - y * vx);
  return Object.freeze([
    vx + w * tx + (y * tz - z * ty),
    vy + w * ty + (z * tx - x * tz),
    vz + w * tz + (x * ty - y * tx),
  ]);
}

export function quaternionToRotationMatrix(value) {
  const q = normalizeQuaternion(value);
  if (!q) return null;
  const [w, x, y, z] = q;
  return Object.freeze([
    Object.freeze([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)]),
    Object.freeze([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)]),
    Object.freeze([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]),
  ]);
}

/** Row-major T_world_device. Three's Matrix4.set consumes this same ordering. */
export function worldFromDeviceMatrix(positionValue, orientationValue) {
  const position = vector3(positionValue);
  const rotation = quaternionToRotationMatrix(orientationValue);
  if (!position || !rotation) return null;
  return Object.freeze([
    rotation[0][0], rotation[0][1], rotation[0][2], position[0],
    rotation[1][0], rotation[1][1], rotation[1][2], position[1],
    rotation[2][0], rotation[2][1], rotation[2][2], position[2],
    0, 0, 0, 1,
  ]);
}

export function devicePointToWorld(pose, pointValue) {
  const point = vector3(pointValue);
  const position = vector3(pose?.position);
  const rotated = rotateVectorByQuaternion(pose?.orientation, point);
  if (!point || !position || !rotated) return null;
  return Object.freeze(rotated.map((value, index) => value + position[index]));
}

export function worldPointToDevice(pose, pointValue) {
  const point = vector3(pointValue);
  const position = vector3(pose?.position);
  const orientation = normalizeQuaternion(pose?.orientation);
  if (!point || !position || !orientation) return null;
  const relative = point.map((value, index) => value - position[index]);
  return rotateVectorByQuaternion(
    [orientation[0], -orientation[1], -orientation[2], -orientation[3]],
    relative,
  );
}

function publicPose(state, details = {}) {
  const position = Object.freeze([...state.position]);
  const orientation = Object.freeze([...state.orientation]);
  return Object.freeze({
    epoch: state.epoch,
    initialized: state.timestampNs !== null,
    timestampNs: state.timestampNs,
    position,
    orientation,
    matrixWorldFromDevice: worldFromDeviceMatrix(position, orientation),
    worldCoordinateFrame: AR_COORDINATE_FRAME.LOCAL_WORLD_DEVICE,
    deviceCoordinateFrame: AR_COORDINATE_FRAME.DEVICE_FRD,
    samples: state.samples,
    integrations: state.integrations,
    skippedGaps: state.skippedGaps,
    fusion: Object.freeze({ ...state.fusion }),
    geoCorrection: Object.freeze({
      ...state.geoCorrection,
      pendingPosition: Object.freeze([...state.geoPendingPosition]),
    }),
    ...details,
  });
}

export function createDeviceWorldPose(options = {}) {
  const limits = Object.freeze({ ...AR_WORLD_POSE_LIMITS, ...(options.limits || {}) });
  const state = {
    epoch: String(options.epoch || "uninitialized"),
    timestampNs: null,
    position: [...ZERO_VECTOR],
    orientation: [...IDENTITY_QUATERNION],
    samples: 0,
    integrations: 0,
    skippedGaps: 0,
    worldFromNed: null,
    lastLivePoseTimestampNs: null,
    fusion: {
      orientationReferenceReady: false,
      orientationAccepted: 0,
      orientationRejected: 0,
      velocityAccepted: 0,
      angularVelocityAccepted: 0,
      accelerationAccepted: 0,
      lastOrientationInnovationRad: null,
      lastVelocityInnovationMps: null,
      lastAngularInnovationRadps: null,
      lastAccelerationMagnitudeMps2: null,
      lastLivePoseFresh: false,
      lastReason: "not observed",
    },
    geoReference: null,
    geoLastObservationKey: null,
    geoPendingPosition: [...ZERO_VECTOR],
    geoPendingYawRad: 0,
    geoCorrection: {
      referenceReady: false,
      accepted: 0,
      rejected: 0,
      yawAccepted: 0,
      yawRejected: 0,
      appliedFrames: 0,
      lastPositionInnovationM: null,
      lastYawInnovationRad: null,
      lastPositionSigmaM: null,
      lastHeadingSigmaDeg: null,
      pendingYawRad: 0,
      lastReason: "not observed",
    },
  };
  let last = publicPose(state, { accepted: false, reason: "not initialized", dtMs: null });

  function reset(input = {}) {
    state.epoch = String(input.epoch || "uninitialized");
    state.timestampNs = null;
    state.position = [...ZERO_VECTOR];
    state.orientation = [...IDENTITY_QUATERNION];
    state.samples = 0;
    state.integrations = 0;
    state.skippedGaps = 0;
    state.worldFromNed = null;
    state.lastLivePoseTimestampNs = null;
    state.fusion = {
      orientationReferenceReady: false,
      orientationAccepted: 0,
      orientationRejected: 0,
      velocityAccepted: 0,
      angularVelocityAccepted: 0,
      accelerationAccepted: 0,
      lastOrientationInnovationRad: null,
      lastVelocityInnovationMps: null,
      lastAngularInnovationRadps: null,
      lastAccelerationMagnitudeMps2: null,
      lastLivePoseFresh: false,
      lastReason: "reset",
    };
    state.geoReference = null;
    state.geoLastObservationKey = null;
    state.geoPendingPosition = [...ZERO_VECTOR];
    state.geoPendingYawRad = 0;
    state.geoCorrection = {
      referenceReady: false,
      accepted: 0,
      rejected: 0,
      yawAccepted: 0,
      yawRejected: 0,
      appliedFrames: 0,
      lastPositionInnovationM: null,
      lastYawInnovationRad: null,
      lastPositionSigmaM: null,
      lastHeadingSigmaDeg: null,
      pendingYawRad: 0,
      lastReason: "reset",
    };
    last = publicPose(state, {
      accepted: false,
      reason: String(input.reason || "reset"),
      dtMs: null,
    });
    return last;
  }

  function observeLiveOrientation(livePose, timestampNs) {
    const poseTimestampNs = finite(livePose?.timestamp, null);
    const usable = livePose?.inputsOK === true
      && livePose?.posenetOK === true
      && livePose?.sensorsOK !== false
      && livePoseFresh(livePose, timestampNs, limits);
    const orientation = usable
      ? measurementVector(livePose.orientationNED, limits.maxOrientationStdRad)
      : null;
    if (!orientation) {
      state.fusion.lastReason = usable ? "orientation invalid/std rejected" : "livePose stale/state rejected";
      return false;
    }
    if (poseTimestampNs === state.lastLivePoseTimestampNs) {
      state.fusion.lastReason = "livePose sample already fused";
      return false;
    }
    state.lastLivePoseTimestampNs = poseTimestampNs;

    const nedFromDevice = quaternionFromEuler(orientation.value);
    if (!nedFromDevice) {
      state.fusion.orientationRejected += 1;
      state.fusion.lastReason = "orientation conversion failed";
      return false;
    }
    if (!state.worldFromNed) {
      state.worldFromNed = multiplyQuaternions(
        state.orientation,
        conjugateQuaternion(nedFromDevice),
      );
      state.fusion.orientationReferenceReady = true;
      state.fusion.lastReason = "livePose NED reference initialized";
      return false;
    }

    const observedWorldFromDevice = multiplyQuaternions(state.worldFromNed, nedFromDevice);
    const innovationRad = quaternionDistanceRad(state.orientation, observedWorldFromDevice);
    state.fusion.lastOrientationInnovationRad = innovationRad;
    if (innovationRad === null || innovationRad > limits.maxOrientationInnovationRad) {
      state.fusion.orientationRejected += 1;
      state.fusion.lastReason = "orientation innovation rejected";
      return false;
    }
    const stdGain = limits.orientationCorrectionGain / (1 + orientation.worstStd * 8);
    const corrected = slerpQuaternions(state.orientation, observedWorldFromDevice, stdGain);
    if (!corrected) {
      state.fusion.orientationRejected += 1;
      state.fusion.lastReason = "orientation correction failed";
      return false;
    }
    state.orientation = [...corrected];
    state.fusion.orientationAccepted += 1;
    state.fusion.lastReason = "livePose orientation corrected";
    return true;
  }

  function applyPendingGeographicCorrection() {
    const pendingDistanceM = Math.hypot(...state.geoPendingPosition);
    let applied = false;
    if (pendingDistanceM > 1e-6) {
      const stepM = Math.min(pendingDistanceM, limits.maxGeoPositionStepM);
      const scale = stepM / pendingDistanceM;
      const step = state.geoPendingPosition.map((value) => value * scale);
      state.position = state.position.map((value, index) => value + step[index]);
      state.geoPendingPosition = state.geoPendingPosition.map((value, index) => value - step[index]);
      applied = true;
    }

    if (Math.abs(state.geoPendingYawRad) > 1e-8) {
      const yawStep = clamp(
        state.geoPendingYawRad,
        -limits.maxGeoYawStepRad,
        limits.maxGeoYawStepRad,
      );
      const corrected = multiplyQuaternions(
        state.orientation,
        quaternionFromRotationVector([0, 0, yawStep]),
      );
      if (corrected) state.orientation = [...corrected];
      state.geoPendingYawRad -= yawStep;
      applied = true;
    }
    state.geoCorrection.pendingYawRad = state.geoPendingYawRad;
    if (applied) state.geoCorrection.appliedFrames += 1;
  }

  function observeGeographicPosition(observation) {
    const key = String(observation?.key || "");
    const isNew = key && key !== state.geoLastObservationKey;
    if (!isNew) {
      applyPendingGeographicCorrection();
      return false;
    }
    state.geoLastObservationKey = key;

    const latitude = finite(observation?.latitude, null);
    const longitude = finite(observation?.longitude, null);
    const headingDeg = finite(observation?.headingDeg, null);
    const positionSigmaM = finite(observation?.positionSigmaM, null);
    const headingSigmaDeg = finite(observation?.headingSigmaDeg, null);
    state.geoCorrection.lastPositionSigmaM = positionSigmaM;
    state.geoCorrection.lastHeadingSigmaDeg = headingSigmaDeg;
    if (observation?.valid !== true
        || latitude === null || longitude === null || headingDeg === null
        || positionSigmaM === null || positionSigmaM < 0
        || positionSigmaM > limits.maxGeoPositionSigmaM) {
      state.geoCorrection.rejected += 1;
      state.geoCorrection.lastReason = observation?.valid === true
        ? "geographic covariance rejected"
        : "geographic quality gate rejected";
      applyPendingGeographicCorrection();
      return false;
    }

    if (!state.geoReference) {
      state.geoReference = Object.freeze({
        latitude,
        longitude,
        headingDeg,
        position: Object.freeze([...state.position]),
        orientation: Object.freeze([...state.orientation]),
      });
      state.geoCorrection.referenceReady = true;
      state.geoCorrection.lastReason = "geographic reference initialized";
      applyPendingGeographicCorrection();
      return false;
    }

    const offset = enuOffset(
      state.geoReference.latitude,
      state.geoReference.longitude,
      latitude,
      longitude,
    );
    const routeDisplacement = offset && toVehicleFrame(
      offset.east,
      offset.north,
      state.geoReference.headingDeg,
    );
    const deviceDisplacement = routeDisplacement
      ? routeFluVectorToDeviceFrd([
          routeDisplacement.x,
          routeDisplacement.y,
          routeDisplacement.z,
        ])
      : null;
    const worldDisplacement = deviceDisplacement
      ? rotateVectorByQuaternion(state.geoReference.orientation, deviceDisplacement)
      : null;
    if (!worldDisplacement) {
      state.geoCorrection.rejected += 1;
      state.geoCorrection.lastReason = "geographic frame conversion failed";
      applyPendingGeographicCorrection();
      return false;
    }

    const observedPosition = state.geoReference.position.map(
      (value, index) => value + worldDisplacement[index],
    );
    const innovation = observedPosition.map((value, index) => value - state.position[index]);
    const innovationM = Math.hypot(...innovation);
    state.geoCorrection.lastPositionInnovationM = innovationM;
    if (innovationM > limits.maxGeoPositionInnovationM) {
      state.geoCorrection.rejected += 1;
      state.geoPendingPosition = [...ZERO_VECTOR];
      state.geoCorrection.lastReason = "geographic position innovation rejected";
    } else {
      const confidence = 1 / (1 + positionSigmaM / 5);
      const correctionM = Math.min(
        limits.maxGeoPositionCorrectionM,
        innovationM * limits.geoPositionCorrectionGain * confidence,
      );
      const scale = innovationM > 1e-9 ? correctionM / innovationM : 0;
      state.geoPendingPosition = innovation.map((value) => value * scale);
      state.geoCorrection.accepted += 1;
      state.geoCorrection.lastReason = "geographic position correction accepted";
    }

    if (observation?.yawUsable === true
        && headingSigmaDeg !== null
        && headingSigmaDeg >= 0
        && headingSigmaDeg <= limits.maxGeoHeadingSigmaDeg) {
      const referenceFromWorld = conjugateQuaternion(state.geoReference.orientation);
      const relativeOrientation = multiplyQuaternions(referenceFromWorld, state.orientation);
      const relativeEuler = quaternionToEuler(relativeOrientation);
      const observedYawRad = wrapAngleRad(
        (headingDeg - state.geoReference.headingDeg) * Math.PI / 180,
      );
      const yawInnovationRad = relativeEuler
        ? wrapAngleRad(observedYawRad - relativeEuler[2])
        : null;
      state.geoCorrection.lastYawInnovationRad = yawInnovationRad;
      if (yawInnovationRad !== null
          && Math.abs(yawInnovationRad) <= limits.maxGeoYawInnovationRad) {
        const confidence = 1 / (1 + headingSigmaDeg / 10);
        state.geoPendingYawRad = clamp(
          yawInnovationRad * limits.geoYawCorrectionGain * confidence,
          -limits.maxGeoYawCorrectionRad,
          limits.maxGeoYawCorrectionRad,
        );
        state.geoCorrection.yawAccepted += 1;
      } else {
        state.geoPendingYawRad = 0;
        state.geoCorrection.yawRejected += 1;
        state.geoCorrection.lastReason = "geographic yaw innovation rejected";
      }
    } else {
      state.geoPendingYawRad = 0;
      state.geoCorrection.lastYawInnovationRad = null;
    }

    applyPendingGeographicCorrection();
    return true;
  }

  function update(input = {}) {
    const odometry = input.odometry;
    if (!odometry) {
      last = publicPose(state, { accepted: false, reason: "odometry missing", dtMs: null });
      return last;
    }
    assertCoordinateFrame(odometry, AR_COORDINATE_FRAME.DEVICE_FRD, "world pose odometry");
    const timestampNs = finite(input.timestampNs ?? odometry.targetTimestampNs, null);
    const velocity = vector3(odometry.trans);
    const angularVelocity = vector3(odometry.rot);
    if (!velocity || !angularVelocity || timestampNs === null || timestampNs <= 0) {
      last = publicPose(state, { accepted: false, reason: "invalid pose sample", dtMs: null });
      return last;
    }

    if (state.timestampNs === null) {
      state.timestampNs = timestampNs;
      state.samples += 1;
      const motion = fusedMotion(odometry, input.livePose, timestampNs, limits);
      state.fusion.lastLivePoseFresh = motion.livePoseFresh;
      state.fusion.lastVelocityInnovationMps = motion.velocityInnovationMps;
      state.fusion.lastAngularInnovationRadps = motion.angularInnovationRadps;
      state.fusion.lastAccelerationMagnitudeMps2 = motion.accelerationMagnitudeMps2;
      observeLiveOrientation(input.livePose, timestampNs);
      observeGeographicPosition(input.geographicObservation);
      last = publicPose(state, { accepted: true, reason: "origin initialized", dtMs: 0 });
      return last;
    }

    const dtMs = (timestampNs - state.timestampNs) / NS_PER_MS;
    if (dtMs < 0) {
      last = publicPose(state, { accepted: false, reason: "timestamp regression", dtMs });
      return last;
    }
    if (dtMs === 0) {
      last = publicPose(state, { accepted: true, reason: "same presented frame", dtMs: 0 });
      return last;
    }

    state.timestampNs = timestampNs;
    state.samples += 1;
    if (dtMs > limits.maxIntegrationStepMs || input.trackingState === "lost") {
      state.skippedGaps += 1;
      last = publicPose(state, {
        accepted: false,
        reason: input.trackingState === "lost"
          ? "tracking lost — integration skipped"
          : `integration gap ${Math.round(dtMs)}ms`,
        dtMs,
      });
      return last;
    }

    const motion = fusedMotion(odometry, input.livePose, timestampNs, limits);
    const fusedVelocity = motion.velocity || velocity;
    const fusedAngularVelocity = motion.angularVelocity || angularVelocity;
    state.fusion.lastLivePoseFresh = motion.livePoseFresh;
    state.fusion.lastVelocityInnovationMps = motion.velocityInnovationMps;
    state.fusion.lastAngularInnovationRadps = motion.angularInnovationRadps;
    state.fusion.lastAccelerationMagnitudeMps2 = motion.accelerationMagnitudeMps2;
    if (motion.velocityAccepted) state.fusion.velocityAccepted += 1;
    if (motion.angularAccepted) state.fusion.angularVelocityAccepted += 1;
    if (motion.accelerationAccepted) state.fusion.accelerationAccepted += 1;

    const dtSeconds = dtMs / 1000;
    const fullDelta = quaternionFromRotationVector(fusedAngularVelocity.map((value) => value * dtSeconds));
    const halfDelta = quaternionFromRotationVector(fusedAngularVelocity.map((value) => value * dtSeconds * 0.5));
    const midpointOrientation = multiplyQuaternions(state.orientation, halfDelta);
    const worldDisplacement = rotateVectorByQuaternion(
      midpointOrientation,
      fusedVelocity.map((value) => value * dtSeconds),
    );
    const nextOrientation = multiplyQuaternions(state.orientation, fullDelta);
    if (!worldDisplacement || !nextOrientation) {
      last = publicPose(state, { accepted: false, reason: "SE(3) integration failed", dtMs });
      return last;
    }
    state.position = state.position.map((value, index) => value + worldDisplacement[index]);
    state.orientation = [...nextOrientation];
    observeLiveOrientation(input.livePose, timestampNs);
    observeGeographicPosition(input.geographicObservation);
    state.integrations += 1;
    last = publicPose(state, { accepted: true, reason: "integrated", dtMs });
    return last;
  }

  function status() {
    return last;
  }

  return Object.freeze({ update, reset, status, limits });
}
