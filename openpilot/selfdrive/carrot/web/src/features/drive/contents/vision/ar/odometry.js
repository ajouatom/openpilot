/* cameraOdometry pose vectors are expressed in the calibrated/model frame.
 * World anchors live in openpilot's device frame (+x forward, +y left, +z up),
 * so applying the raw vectors directly mixes pitch/yaw into the wrong axes.
 *
 * This is the JavaScript equivalent of locationd's:
 *   device_from_calib = rot_from_euler(liveCalibration.rpyCalib)
 *   vector_device = device_from_calib @ vector_calib
 */

function finiteVector3(value) {
  if (!Array.isArray(value) || value.length < 3) return null;
  const vector = value.slice(0, 3).map(Number);
  return vector.every(Number.isFinite) ? vector : null;
}

export function deviceFromCalibMatrix(rpyCalib) {
  const rpy = finiteVector3(rpyCalib);
  if (!rpy) return null;
  const [roll, pitch, yaw] = rpy;
  const cr = Math.cos(roll), sr = Math.sin(roll);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const cy = Math.cos(yaw), sy = Math.sin(yaw);

  // Rz(yaw) @ Ry(pitch) @ Rx(roll), matching rot_from_euler().
  return Object.freeze([
    Object.freeze([cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr]),
    Object.freeze([sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr]),
    Object.freeze([-sp, cp * sr, cp * cr]),
  ]);
}

function rotateVector(matrix, value) {
  const vector = finiteVector3(value);
  if (!matrix || !vector) return null;
  return Object.freeze(matrix.map((row) => (
    row[0] * vector[0] + row[1] * vector[1] + row[2] * vector[2]
  )));
}

function rotateIndependentStd(matrix, value) {
  const std = finiteVector3(value);
  if (!matrix || !std) return null;
  // Transform a diagonal covariance and expose its device-axis standard
  // deviations. Signs do not apply to uncertainty, hence the squared terms.
  return Object.freeze(matrix.map((row) => Math.sqrt(
    (row[0] * std[0]) ** 2
    + (row[1] * std[1]) ** 2
    + (row[2] * std[2]) ** 2,
  )));
}

export function odometryInDeviceFrame(odometry, rpyCalib) {
  if (!odometry) return null;
  const matrix = deviceFromCalibMatrix(rpyCalib);
  const trans = rotateVector(matrix, odometry.trans);
  const rot = rotateVector(matrix, odometry.rot);
  if (!matrix || !trans || !rot) return null;

  const transStd = rotateIndependentStd(matrix, odometry.transStd);
  const rotStd = rotateIndependentStd(matrix, odometry.rotStd);
  return Object.freeze({
    ...odometry,
    trans,
    rot,
    ...(transStd ? { transStd } : {}),
    ...(rotStd ? { rotStd } : {}),
    coordinateFrame: "device",
  });
}
