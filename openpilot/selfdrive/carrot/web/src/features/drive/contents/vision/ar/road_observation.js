/* Bounded model-path road observation.
 *
 * modelV2.position is a short-range observation of the road/vehicle path, not
 * an absolute altitude source. Raw z samples can contain a biased origin,
 * missing values, or a steep horizon spike. Those values must never lift a
 * world marker directly. This module turns them into a vehicle-local road
 * height with an explicit source/confidence and bounded grade.
 */

export const AR_ROAD_HEIGHT_SOURCE = Object.freeze({
  MODEL_PATH: "model-path-bounded",
  UNKNOWN_FLAT: "unknown-flat",
});

export const AR_ROAD_OBSERVATION_LIMITS = Object.freeze({
  /** Road height is trusted only in the model's useful near-field. */
  fullHeightConfidenceM: 70,
  maxHeightDistanceM: 120,
  /** 25% keeps real steep ramps while rejecting horizon spikes. */
  maxAbsGrade: 0.25,
  /** A route/model lateral disagreement is an innovation, not a snap target. */
  maxLateralInnovationM: 3,
  lateralCorrectionGain: 0.35,
  maxLateralCorrectionM: 0.75,
});

export const UNKNOWN_ROAD_HEIGHT = Object.freeze({
  z: 0,
  roadGrade: 0,
  heightSource: AR_ROAD_HEIGHT_SOURCE.UNKNOWN_FLAT,
  heightConfidence: 0,
  heightLimited: false,
});

function sampleNumber(value) {
  if (value === null || value === undefined || value === "" || typeof value === "boolean") return null;
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function heightAt(points, targetM, limits) {
  if (!points.length || points[0].z === null) return null;
  let boundedZ = 0; // model z is relative; remove any constant origin bias.
  let limited = Math.abs(points[0].z) > 1e-6;

  for (let index = 1; index < points.length; index += 1) {
    const previous = points[index - 1];
    const current = points[index];
    if (previous.z === null || current.z === null) return null;
    const dx = current.x - previous.x;
    const rawGrade = (current.z - previous.z) / dx;
    const grade = clamp(rawGrade, -limits.maxAbsGrade, limits.maxAbsGrade);
    if (Math.abs(grade - rawGrade) > 1e-9) limited = true;
    const segmentM = Math.min(targetM, current.x) - previous.x;
    if (segmentM > 0) boundedZ += grade * segmentM;
    if (targetM <= current.x) return { z: boundedZ, grade, limited };
  }
  return null;
}

/**
 * Sample x/y and a bounded local road height at a forward distance.
 * Invalid/non-monotonic geometry is rejected. Missing z keeps the usable x/y
 * sample but labels height as unknown-flat instead of fabricating elevation.
 */
export function modelRoadObservation(position, distanceM, options = {}) {
  const limits = options.limits
    ? { ...AR_ROAD_OBSERVATION_LIMITS, ...options.limits }
    : AR_ROAD_OBSERVATION_LIMITS;
  const xs = Array.isArray(position?.x) ? position.x : null;
  const ys = Array.isArray(position?.y) ? position.y : null;
  const zs = Array.isArray(position?.z) ? position.z : null;
  const distance = sampleNumber(distanceM);
  if (!xs || !ys || xs.length < 2 || distance === null || distance < 0) return null;

  const count = Math.min(xs.length, ys.length);
  const points = [];
  let bracket = null;
  for (let index = 0; index < count; index += 1) {
    const point = {
      x: sampleNumber(xs[index]),
      y: sampleNumber(ys[index]),
      z: zs ? sampleNumber(zs[index]) : null,
      index,
    };
    if (point.x === null || point.y === null) return null;
    if (points.length && point.x <= points.at(-1).x) return null;
    points.push(point);
    if (points.length >= 2 && point.x >= distance) {
      bracket = [points.at(-2), point];
      break;
    }
  }
  if (!bracket || distance < points[0].x) return null;

  const [before, after] = bracket;
  const span = after.x - before.x;
  const t = clamp((distance - before.x) / span, 0, 1);
  const y = before.y + (after.y - before.y) * t;
  const heightDistanceM = Math.min(distance, limits.maxHeightDistanceM);
  const bounded = heightAt(points, heightDistanceM, limits);
  const heightKnown = Boolean(bounded);
  const confidence = !heightKnown || distance >= limits.maxHeightDistanceM
    ? 0
    : distance <= limits.fullHeightConfidenceM
      ? 1
      : (limits.maxHeightDistanceM - distance)
        / (limits.maxHeightDistanceM - limits.fullHeightConfidenceM);
  const grade = heightKnown && distance < limits.maxHeightDistanceM ? bounded.grade : 0;

  return Object.freeze({
    x: distance,
    y,
    z: heightKnown ? bounded.z : 0,
    tangent: Object.freeze([span, after.y - before.y, grade * span]),
    index: after.index,
    roadGrade: grade,
    heightSource: heightKnown
      ? AR_ROAD_HEIGHT_SOURCE.MODEL_PATH
      : AR_ROAD_HEIGHT_SOURCE.UNKNOWN_FLAT,
    heightConfidence: clamp(confidence, 0, 1),
    heightLimited: Boolean(bounded?.limited || (heightKnown && distance >= limits.maxHeightDistanceM)),
  });
}

/** Apply only a small, gated route-to-model lateral innovation. */
export function limitedRoadLateralCorrection(routeY, modelY, options = {}) {
  const limits = options.limits
    ? { ...AR_ROAD_OBSERVATION_LIMITS, ...options.limits }
    : AR_ROAD_OBSERVATION_LIMITS;
  const route = sampleNumber(routeY);
  const model = sampleNumber(modelY);
  if (route === null || model === null || options.enabled !== true) {
    return Object.freeze({ y: route ?? model ?? 0, correctionM: 0, accepted: false });
  }
  const innovationM = model - route;
  if (Math.abs(innovationM) > limits.maxLateralInnovationM) {
    return Object.freeze({ y: route, correctionM: 0, accepted: false });
  }
  const correctionM = clamp(
    innovationM * limits.lateralCorrectionGain,
    -limits.maxLateralCorrectionM,
    limits.maxLateralCorrectionM,
  );
  return Object.freeze({ y: route + correctionM, correctionM, accepted: true });
}
