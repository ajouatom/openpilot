import { faceFrameForAnchor } from "./road_frame.js";
import {
  AR_COORDINATE_FRAME,
  assertCoordinateFrame,
  routeFluVectorToDeviceFrd,
} from "./coordinate_frames.js";

/* Pure matrix contracts shared by the Three adapter and static tests.
 *
 * Matrices are returned in row-major order. No Three.js type crosses this
 * module boundary, keeping camera alignment independently testable.
 */

function finite(value, fallback = null) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function validMat3(matrix) {
  return Array.isArray(matrix)
    && matrix.length === 3
    && matrix.every((row) => Array.isArray(row)
      && row.length === 3
      && row.every((value) => Number.isFinite(Number(value))));
}

function combineRow(a, aScale, b, bScale) {
  return [
    Number(a[0]) * aScale + Number(b[0]) * bScale,
    Number(a[1]) * aScale + Number(b[1]) * bScale,
    Number(a[2]) * aScale + Number(b[2]) * bScale,
  ];
}

/**
 * Convert the existing calibrated pixel projection and stage crop into one
 * WebGL clip-space matrix. This preserves renderer.js exactly:
 *   stageX = projectedX * scale + tx
 *   stageY = projectedY * scale + ty
 */
export function stageProjectionMatrix(stage, viewport = {}) {
  const frdTransform = stage?.calibTransform;
  if (!validMat3(frdTransform)) return null;

  // Three world objects use route FLU. Compose calibTransform(FRD) with the
  // one explicit FLU->FRD basis conversion before building clip space.
  const transform = frdTransform.map((row) => [Number(row[0]), -Number(row[1]), -Number(row[2])]);

  const width = Math.max(1, finite(viewport.width, stage?.stageWidth) || 1);
  const height = Math.max(1, finite(viewport.height, stage?.stageHeight) || 1);
  const scale = finite(stage?.scale, 1);
  const tx = finite(stage?.tx, 0);
  const ty = finite(stage?.ty, 0);
  const near = Math.max(0.001, finite(viewport.near, 0.1));
  const far = Math.max(near + 0.001, finite(viewport.far, 600));

  const depthRow = transform[2].map(Number);
  const clipX = combineRow(transform[0], (2 * scale) / width, depthRow, (2 * tx) / width - 1);
  const clipY = combineRow(transform[1], (-2 * scale) / height, depthRow, 1 - (2 * ty) / height);
  const depthScale = (far + near) / (far - near);
  const depthOffset = (-2 * far * near) / (far - near);
  const clipZ = depthRow.map((value) => value * depthScale);

  return Object.freeze([
    clipX[0], clipX[1], clipX[2], 0,
    clipY[0], clipY[1], clipY[2], 0,
    clipZ[0], clipZ[1], clipZ[2], depthOffset,
    depthRow[0], depthRow[1], depthRow[2], 0,
  ]);
}

/** Map approved-preview local axes into the AR route-local FLU world. */
export function markerWorldMatrix(anchor, scale = 1) {
  if (!anchor) return null;
  if (anchor.coordinateFrame !== undefined) {
    assertCoordinateFrame(anchor, AR_COORDINATE_FRAME.ROUTE_FLU, "marker anchor");
  }
  const x = finite(anchor.x);
  const y = finite(anchor.y);
  const z = finite(anchor.z, 0);
  if (x === null || y === null) return null;

  const worldScale = Math.max(0.001, finite(scale, 1));
  const frame = faceFrameForAnchor(anchor);
  const right = frame.right.map((value) => value * worldScale);
  const up = frame.up.map((value) => value * worldScale);
  const backward = frame.forward.map((value) => -value * worldScale);

  // local +X = driver's right, local +Y = up, local -Z = route forward.
  return Object.freeze([
    right[0], up[0], backward[0], x,
    right[1], up[1], backward[1], y,
    right[2], up[2], backward[2], z,
    0, 0, 0, 1,
  ]);
}

export function transformPointRowMajor(matrix, point, w = 1) {
  if (!Array.isArray(matrix) || matrix.length !== 16 || !point) return null;
  const vector = [finite(point.x, 0), finite(point.y, 0), finite(point.z, 0), finite(w, 1)];
  return Object.freeze([0, 1, 2, 3].map((row) => (
    matrix[row * 4] * vector[0]
    + matrix[row * 4 + 1] * vector[1]
    + matrix[row * 4 + 2] * vector[2]
    + matrix[row * 4 + 3] * vector[3]
  )));
}

/** Test/diagnostic helper exposing the exact FLU -> FRD projection boundary. */
export function routeFluPointInDeviceFrd(point) {
  const vector = routeFluVectorToDeviceFrd([point?.x, point?.y, point?.z]);
  return vector && Object.freeze({ x: vector[0], y: vector[1], z: vector[2] });
}
