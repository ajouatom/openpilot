/* Named coordinate-frame boundaries for the AR pipeline.
 *
 * openpilot Device/Calibrated vectors are FRD: [forward, right, down].
 * The existing AR road/marker math is FLU: [forward, left, up]. The two are
 * related by a proper 180 degree rotation about +x, diag(1, -1, -1). Keeping
 * that conversion here prevents source modules from silently sharing numbers
 * that have opposite lateral and vertical meanings.
 */

export const AR_COORDINATE_FRAME = Object.freeze({
  GEODETIC_WGS84: "geodetic-wgs84",
  LOCAL_WORLD_ENU: "local-world-enu",
  // Before livePose/geo alignment, the fixed local world starts with the
  // Device axes at the navigation-session origin. It must not pretend to be
  // ENU until an explicit orientation observation aligns it.
  LOCAL_WORLD_DEVICE: "local-world-device-origin",
  NED: "ned",
  DEVICE_FRD: "openpilot-device-frd",
  CALIBRATED_FRD: "openpilot-calibrated-frd",
  ROUTE_FLU: "route-local-flu",
  VIEW_RDF: "openpilot-view-rdf",
  MARKER_RUB: "marker-local-right-up-backward",
  STAGE_CSS_PIXEL: "stage-css-pixel",
});

export const AR_LIVE_POSE_FRAMES = Object.freeze({
  orientationNED: AR_COORDINATE_FRAME.NED,
  velocityDevice: AR_COORDINATE_FRAME.DEVICE_FRD,
  accelerationDevice: AR_COORDINATE_FRAME.DEVICE_FRD,
  angularVelocityDevice: AR_COORDINATE_FRAME.DEVICE_FRD,
});

function finiteVector3(value) {
  if (!Array.isArray(value) || value.length < 3) return null;
  const vector = value.slice(0, 3).map(Number);
  return vector.every(Number.isFinite) ? vector : null;
}

export function assertCoordinateFrame(value, expected, label = "value") {
  const actual = value?.coordinateFrame;
  if (actual !== expected) {
    throw new TypeError(`${label} coordinateFrame must be ${expected}; received ${String(actual)}`);
  }
  return value;
}

/** FRD <-> FLU is an involution: [x, y, z] -> [x, -y, -z]. */
export function flipFrdFluVector(value) {
  const vector = finiteVector3(value);
  if (!vector) return null;
  const flip = (number) => (number === 0 ? 0 : -number);
  return Object.freeze([vector[0], flip(vector[1]), flip(vector[2])]);
}

export const deviceFrdVectorToRouteFlu = flipFrdFluVector;
export const routeFluVectorToDeviceFrd = flipFrdFluVector;

function flipSeries(value) {
  return Array.isArray(value)
    ? Object.freeze(value.map((item) => {
      const number = Number(item);
      return Number.isFinite(number) ? (number === 0 ? 0 : -number) : item;
    }))
    : value;
}

/** Tag and convert modelV2.position from Calibrated FRD into AR's route FLU. */
export function modelPositionFrdToRouteFlu(position) {
  if (!position || !Array.isArray(position.x) || !Array.isArray(position.y)) return null;
  if (position.coordinateFrame && position.coordinateFrame !== AR_COORDINATE_FRAME.CALIBRATED_FRD) {
    throw new TypeError(`modelV2.position coordinateFrame must be ${AR_COORDINATE_FRAME.CALIBRATED_FRD}`);
  }
  return Object.freeze({
    ...position,
    x: position.x,
    y: flipSeries(position.y),
    ...(Array.isArray(position.z) ? { z: flipSeries(position.z) } : {}),
    sourceCoordinateFrame: AR_COORDINATE_FRAME.CALIBRATED_FRD,
    coordinateFrame: AR_COORDINATE_FRAME.ROUTE_FLU,
  });
}

/** Convert odometry already expressed in Device FRD once at the AR boundary. */
export function deviceOdometryFrdToRouteFlu(odometry) {
  if (!odometry) return null;
  assertCoordinateFrame(odometry, AR_COORDINATE_FRAME.DEVICE_FRD, "cameraOdometry");
  const trans = deviceFrdVectorToRouteFlu(odometry.trans);
  const rot = deviceFrdVectorToRouteFlu(odometry.rot);
  if (!trans || !rot) return null;
  return Object.freeze({
    ...odometry,
    trans,
    rot,
    // Axis signs do not change standard-deviation magnitudes.
    sourceCoordinateFrame: AR_COORDINATE_FRAME.DEVICE_FRD,
    coordinateFrame: AR_COORDINATE_FRAME.ROUTE_FLU,
  });
}
