/* Pure 3D road-frame math for world-anchored AR markers.
 *
 * Road coordinates are route-local FLU: +x forward, +y left, +z up.
 * This is deliberately not called openpilot Device, which is FRD.
 * Marker coordinates follow the approved preview: +x right, +y up, -z
 * forward. Keeping this contract in one module prevents projection, anchor
 * propagation, and Three placement from inventing different axis rules.
 */

const EPSILON = 1e-9;
const ROUTE_UP = Object.freeze([0, 0, 1]);

function finite(value, fallback = 0) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function vector3(value, fallback = null) {
  if (!Array.isArray(value) || value.length < 3) return fallback;
  const vector = [Number(value[0]), Number(value[1]), Number(value[2])];
  return vector.every(Number.isFinite) ? vector : fallback;
}

function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function cross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function normalize(value, fallback) {
  const vector = vector3(value, fallback);
  const length = vector ? Math.hypot(vector[0], vector[1], vector[2]) : 0;
  if (!(length > EPSILON)) return [...fallback];
  return vector.map((component) => component / length);
}

export function roadFrameFromForward(forwardValue, preferredUpValue = ROUTE_UP) {
  const forward = normalize(forwardValue, [1, 0, 0]);
  let preferredUp = normalize(preferredUpValue, ROUTE_UP);

  // Remove the forward component from up. A near-vertical path needs a
  // deterministic fallback, even though a real road should never reach it.
  let up = preferredUp.map((component, index) => (
    component - forward[index] * dot(preferredUp, forward)
  ));
  if (Math.hypot(...up) <= EPSILON) {
    preferredUp = Math.abs(forward[2]) < 0.9 ? ROUTE_UP : [0, 1, 0];
    up = preferredUp.map((component, index) => (
      component - forward[index] * dot(preferredUp, forward)
    ));
  }
  up = normalize(up, ROUTE_UP);

  // forward x up points to the driver's right (negative y) in route FLU.
  const right = normalize(cross(forward, up), [0, -1, 0]);
  const orthogonalUp = normalize(cross(right, forward), ROUTE_UP);
  return Object.freeze({
    forward: Object.freeze(forward),
    right: Object.freeze(right),
    up: Object.freeze(orthogonalUp),
  });
}

export function roadFrameFromHeading(headingRad = 0, pitchRad = 0) {
  const heading = finite(headingRad);
  const pitch = finite(pitchRad);
  const horizontal = Math.cos(pitch);
  return roadFrameFromForward([
    Math.cos(heading) * horizontal,
    Math.sin(heading) * horizontal,
    Math.sin(pitch),
  ]);
}

export function roadFrameForAnchor(anchor) {
  if (!anchor) return roadFrameFromHeading();
  const forward = vector3(anchor.roadForward);
  const up = vector3(anchor.roadUp, ROUTE_UP);
  return forward
    ? roadFrameFromForward(forward, up)
    : roadFrameFromHeading(anchor.headingRad, anchor.pitchRad);
}

/**
 * Presentation-only face frame. Route fields remain the immutable semantic
 * direction; upright signs may add a separately bounded view-facing frame.
 */
export function faceFrameForAnchor(anchor) {
  if (!anchor) return roadFrameFromHeading();
  const forward = vector3(anchor.faceForward);
  const up = vector3(anchor.faceUp, ROUTE_UP);
  return forward
    ? roadFrameFromForward(forward, up)
    : roadFrameForAnchor(anchor);
}

/**
 * Build the presentation frame for a standing marker.
 *
 * The anchor and route yaw remain world-locked, but the support/face axis is
 * gravity-up. A model-path normal is useful for road bands; it is not a safe
 * vertical axis for a sign because accumulated pose rotation can tilt it past
 * the horizon and place the pole above the face on screen.
 */
export function uprightRoadFrameForAnchor(anchor) {
  const roadFrame = roadFrameForAnchor(anchor);
  const horizontalForward = [roadFrame.forward[0], roadFrame.forward[1], 0];
  if (Math.hypot(horizontalForward[0], horizontalForward[1]) <= EPSILON) {
    const heading = finite(anchor?.headingRad);
    horizontalForward[0] = Math.cos(heading);
    horizontalForward[1] = Math.sin(heading);
  }
  return roadFrameFromForward(horizontalForward, ROUTE_UP);
}

export function roadFrameFields(frame) {
  const normalized = roadFrameFromForward(frame?.forward, frame?.up);
  return Object.freeze({
    roadForward: normalized.forward,
    roadRight: normalized.right,
    roadUp: normalized.up,
    headingRad: Math.atan2(normalized.forward[1], normalized.forward[0]),
    pitchRad: Math.atan2(
      normalized.forward[2],
      Math.hypot(normalized.forward[0], normalized.forward[1]),
    ),
  });
}

export function faceFrameFields(frame) {
  const normalized = roadFrameFromForward(frame?.forward, frame?.up);
  return Object.freeze({
    faceForward: normalized.forward,
    faceRight: normalized.right,
    faceUp: normalized.up,
    faceHeadingRad: Math.atan2(normalized.forward[1], normalized.forward[0]),
  });
}

/** Rotate a vector by an axis-angle rotation vector (Rodrigues formula). */
export function rotateByRotationVector(vectorValue, rotationVectorValue) {
  const vector = vector3(vectorValue, [0, 0, 0]);
  const rotation = vector3(rotationVectorValue, [0, 0, 0]);
  const angle = Math.hypot(rotation[0], rotation[1], rotation[2]);
  if (!(angle > EPSILON)) return Object.freeze([...vector]);

  const axis = rotation.map((component) => component / angle);
  const cosine = Math.cos(angle);
  const sine = Math.sin(angle);
  const axisCrossVector = cross(axis, vector);
  const axisDotVector = dot(axis, vector);
  return Object.freeze(vector.map((component, index) => (
    component * cosine
    + axisCrossVector[index] * sine
    + axis[index] * axisDotVector * (1 - cosine)
  )));
}

export function rotateRoadFrame(frameValue, rotationVector) {
  const frame = roadFrameFromForward(frameValue?.forward, frameValue?.up);
  return roadFrameFromForward(
    rotateByRotationVector(frame.forward, rotationVector),
    rotateByRotationVector(frame.up, rotationVector),
  );
}

export function blendRoadFrames(currentValue, targetValue, alphaValue) {
  const current = roadFrameFromForward(currentValue?.forward, currentValue?.up);
  const target = roadFrameFromForward(targetValue?.forward, targetValue?.up);
  const alpha = Math.max(0, Math.min(1, finite(alphaValue)));
  const mix = (a, b) => a.map((component, index) => (
    component + (b[index] - component) * alpha
  ));
  return roadFrameFromForward(
    mix(current.forward, target.forward),
    mix(current.up, target.up),
  );
}

/**
 * Turn an upright road marker toward the device without changing its anchor
 * or handedness. The support uses gravity-up and only yaw is allowed, so the
 * sign stays aligned with the route and can never pitch below its own pole.
 */
export function constrainedBillboardAnchor(anchor, maxYawRad = Math.PI / 10) {
  if (!anchor) return null;
  const frame = uprightRoadFrameForAnchor(anchor);
  const toAnchor = [finite(anchor.x), finite(anchor.y), finite(anchor.z)];
  const vertical = dot(toAnchor, frame.up);
  const planar = toAnchor.map((component, index) => component - frame.up[index] * vertical);
  if (Math.hypot(...planar) <= EPSILON) {
    return Object.freeze({
      ...anchor,
      ...faceFrameFields(frame),
      billboardYawRad: 0,
    });
  }

  const targetForward = normalize(planar, frame.forward);
  const sine = dot(frame.up, cross(frame.forward, targetForward));
  const cosine = clamp(dot(frame.forward, targetForward), -1, 1);
  const desiredYaw = Math.atan2(sine, cosine);
  const limit = Math.max(0, Math.min(Math.PI / 2 - 1e-3, Math.abs(finite(maxYawRad, Math.PI / 10))));
  const appliedYaw = clamp(desiredYaw, -limit, limit);
  const billboardFrame = roadFrameFromForward(
    rotateByRotationVector(frame.forward, frame.up.map((value) => value * appliedYaw)),
    frame.up,
  );
  return Object.freeze({
    ...anchor,
    ...faceFrameFields(billboardFrame),
    billboardYawRad: appliedYaw,
  });
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}
