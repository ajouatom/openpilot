import assert from "node:assert/strict";
import test from "node:test";
import { FrontSide, Vector3 } from "three";

import { AR_OPACITY, AR_SHAPE } from "../src/features/drive/contents/vision/ar/design_tokens.js";
import { advanceAnchor, driftIncrement } from "../src/features/drive/contents/vision/ar/anchor.js";
import {
  constrainedBillboardAnchor,
  faceFrameForAnchor,
  uprightRoadFrameForAnchor,
} from "../src/features/drive/contents/vision/ar/road_frame.js";
import { odometryInDeviceFrame } from "../src/features/drive/contents/vision/ar/odometry.js";
import { markerIdentity, markerLifecycleSlot } from "../src/features/drive/contents/vision/ar/marker_identity.js";
import { projectPoint } from "../src/features/drive/contents/vision/ar/projection.js";
import { pointOnPath } from "../src/features/drive/contents/vision/ar/projection.js";
import { describeSignboard } from "../src/features/drive/contents/vision/ar/signboard.js";
import { AR_BILLBOARD, AR_RENDER } from "../src/features/drive/contents/vision/ar/tokens.js";
import {
  applyThreeStageProjection,
  createThreeSignboardGroup,
  createThreeStageCamera,
  placeThreeSignboardGroup,
} from "../src/features/drive/contents/vision/ar/three_adapter.js";
import {
  markerWorldMatrix,
  routeFluPointInDeviceFrd,
  stageProjectionMatrix,
  transformPointRowMajor,
} from "../src/features/drive/contents/vision/ar/three_projection.js";

const STAGE = Object.freeze({
  calibTransform: Object.freeze([
    Object.freeze([1000, 0, 960]),
    Object.freeze([0, 1000, 540]),
    Object.freeze([0, 0, 1]),
  ]),
  scale: 0.5,
  tx: 20,
  ty: 30,
  stageWidth: 1000,
  stageHeight: 600,
});

test("production AR policy is Three-only and fail-closed", () => {
  assert.equal(AR_RENDER.backend, "three");
  assert.equal(AR_RENDER.canvas2dFallback, false);
  assert.equal(AR_RENDER.failureMode, "hidden");
  assert.equal(AR_RENDER.targetFps, 30);
  assert.equal(AR_RENDER.degradedFps, 15);
  assert.equal(AR_RENDER.devicePixelRatioMax, 2);
  assert.equal(AR_RENDER.degradedDevicePixelRatioMax, 1.5);
  assert.ok(AR_BILLBOARD.maxYawRad > Math.PI / 3);
  assert.ok(AR_BILLBOARD.maxYawRad < Math.PI / 2);
});

test("Three clip matrix matches the existing calibrated stage projection", () => {
  const point = { x: 0.2, y: -0.1, z: -2 };
  const frd = routeFluPointInDeviceFrd(point);
  const projected = projectPoint(STAGE.calibTransform, frd.x, frd.y, frd.z);
  const expectedStageX = projected.x * STAGE.scale + STAGE.tx;
  const expectedStageY = projected.y * STAGE.scale + STAGE.ty;
  const matrix = stageProjectionMatrix(STAGE);
  const clip = transformPointRowMajor(matrix, point);

  assert.ok(Math.abs(clip[0] / clip[3] - (2 * expectedStageX / STAGE.stageWidth - 1)) < 1e-12);
  assert.ok(Math.abs(clip[1] / clip[3] - (1 - 2 * expectedStageY / STAGE.stageHeight)) < 1e-12);
  assert.ok(clip[2] / clip[3] >= -1 && clip[2] / clip[3] <= 1);

  const camera = createThreeStageCamera();
  assert.equal(applyThreeStageProjection(camera, STAGE), true);
  assert.deepEqual(camera.projectionMatrix.toArray(), new camera.projectionMatrix.constructor().set(...matrix).toArray());
});

test("route heading maps preview right/up/forward axes into route-local FLU", () => {
  const anchor = { x: 20, y: 3, z: 1, headingRad: 0 };
  const matrix = markerWorldMatrix(anchor, 2);
  assert.deepEqual(transformPointRowMajor(matrix, { x: 1, y: 0, z: 0 }), [20, 1, 1, 1]);
  assert.deepEqual(transformPointRowMajor(matrix, { x: 0, y: 1, z: 0 }), [20, 3, 3, 1]);
  assert.deepEqual(transformPointRowMajor(matrix, { x: 0, y: 0, z: -1 }), [22, 3, 1, 1]);

  const turned = markerWorldMatrix({ ...anchor, headingRad: Math.PI / 2 }, 1);
  const forward = transformPointRowMajor(turned, { x: 0, y: 0, z: -1 });
  assert.ok(Math.abs(forward[0] - 20) < 1e-12);
  assert.ok(Math.abs(forward[1] - 4) < 1e-12);
});

test("model path elevation supplies a full orthonormal road frame", () => {
  const anchor = pointOnPath({
    x: [0, 10, 20],
    y: [0, 1, 2],
    z: [0, 2, 4],
  }, 5);
  assert.ok(anchor);
  assert.equal(anchor.z, 1);
  assert.ok(anchor.roadForward[2] > 0);

  const dot = (a, b) => a.reduce((sum, value, index) => sum + value * b[index], 0);
  const length = (vector) => Math.hypot(...vector);
  assert.ok(Math.abs(length(anchor.roadForward) - 1) < 1e-12);
  assert.ok(Math.abs(length(anchor.roadRight) - 1) < 1e-12);
  assert.ok(Math.abs(length(anchor.roadUp) - 1) < 1e-12);
  assert.ok(Math.abs(dot(anchor.roadForward, anchor.roadRight)) < 1e-12);
  assert.ok(Math.abs(dot(anchor.roadForward, anchor.roadUp)) < 1e-12);

  const matrix = markerWorldMatrix(anchor, 1);
  const forwardTip = transformPointRowMajor(matrix, { x: 0, y: 0, z: -1 });
  assert.ok(Math.abs(forwardTip[0] - (anchor.x + anchor.roadForward[0])) < 1e-12);
  assert.ok(Math.abs(forwardTip[1] - (anchor.y + anchor.roadForward[1])) < 1e-12);
  assert.ok(Math.abs(forwardTip[2] - (anchor.z + anchor.roadForward[2])) < 1e-12);
});

test("camera odometry applies one inverse 3D ego transform to pose and orientation", () => {
  const original = pointOnPath({ x: [0, 10], y: [0, 0], z: [0, 0] }, 10);
  const moved = advanceAnchor(original, {
    trans: [2, 0, 0],
    rot: [0, Math.PI / 2, 0],
  }, 1);

  assert.ok(Math.abs(moved.x) < 1e-12);
  assert.ok(Math.abs(moved.y) < 1e-12);
  assert.ok(Math.abs(moved.z - 8) < 1e-12);
  assert.ok(Math.abs(moved.roadForward[0]) < 1e-12);
  assert.ok(Math.abs(moved.roadForward[2] - 1) < 1e-12);

  const matrix = markerWorldMatrix(moved, 1);
  const right = [matrix[0], matrix[4], matrix[8]];
  const up = [matrix[1], matrix[5], matrix[9]];
  const backward = [matrix[2], matrix[6], matrix[10]];
  const determinant = (
    right[0] * (up[1] * backward[2] - up[2] * backward[1])
    - up[0] * (right[1] * backward[2] - right[2] * backward[1])
    + backward[0] * (right[1] * up[2] - right[2] * up[1])
  );
  assert.ok(Math.abs(determinant - 1) < 1e-12, "pose must stay right-handed, not mirrored");
});

test("camera odometry is rotated from calibrated coordinates into device coordinates", () => {
  const pitch = Math.PI / 6;
  const converted = odometryInDeviceFrame({
    trans: [10, 0, 0],
    rot: [0, 0, 0.2],
    transStd: [0.3, 0.2, 0.1],
    rotStd: [0.03, 0.02, 0.01],
  }, [0, pitch, 0]);

  assert.ok(converted);
  assert.ok(Math.abs(converted.trans[0] - 10 * Math.cos(pitch)) < 1e-12);
  assert.ok(Math.abs(converted.trans[2] + 10 * Math.sin(pitch)) < 1e-12);
  assert.ok(Math.abs(converted.rot[0] - 0.2 * Math.sin(pitch)) < 1e-12);
  assert.ok(Math.abs(converted.rot[2] - 0.2 * Math.cos(pitch)) < 1e-12);
  assert.equal(converted.coordinateFrame, "openpilot-device-frd");
  assert.ok(converted.transStd.every(Number.isFinite));
  assert.ok(converted.rotStd.every(Number.isFinite));
});

test("one TMap maneuver keeps its identity when next guidance becomes current", () => {
  const point = { pointValid: true, latitude: 37.1234567, longitude: 127.7654321 };
  const current = { source: "guidanceCurrent", kind: "turn_gate", point, turn: { code: 13 } };
  const next = { source: "guidanceNext", kind: "turn_gate", point, turn: { code: 13 } };

  assert.equal(markerIdentity(current), markerIdentity(next));
  assert.notEqual(markerLifecycleSlot(current), markerLifecycleSlot(next));
  assert.notEqual(
    markerIdentity({ ...current, point: null }),
    markerIdentity({ ...next, point: null }),
    "coordinate-free guidance remains source-scoped because consecutive turns are ambiguous",
  );
});

test("hold drift budget includes vertical and roll/pitch uncertainty", () => {
  const drift = driftIncrement(
    { x: 100, y: 0, z: 0 },
    { transStd: [0, 0, 0.2], rotStd: [0.01, 0.02, 0] },
    0.1,
  );
  assert.ok(drift > 0.2);
});

test("production upright markers use bounded yaw to stay readable at turns", () => {
  const maxYaw = AR_BILLBOARD.maxYawRad;
  const anchor = {
    x: 40,
    y: 0,
    z: 2,
    headingRad: Math.PI / 2,
  };
  const billboard = constrainedBillboardAnchor(anchor, maxYaw);
  const beforeFacing = Math.cos(anchor.headingRad);
  const afterFacing = billboard.faceForward[0];

  assert.ok(Math.abs(billboard.billboardYawRad) <= maxYaw + 1e-12);
  assert.ok(Math.abs(billboard.billboardYawRad) > Math.PI / 3);
  assert.ok(afterFacing > beforeFacing);
  assert.equal(billboard.headingRad, anchor.headingRad, "billboard must not overwrite route yaw");
  assert.ok(Math.abs(billboard.faceUp[2] - 1) < 1e-12);
  assert.ok(Math.abs(Math.hypot(...billboard.faceForward) - 1) < 1e-12);
});

test("upright marker frame rejects an inverted road normal without moving its anchor", () => {
  const anchor = {
    x: 42,
    y: -3,
    z: 1.2,
    roadForward: [0.96, 0.28, 0.08],
    roadUp: [0.08, 0.02, -0.99],
  };
  const frame = uprightRoadFrameForAnchor(anchor);
  const marker = constrainedBillboardAnchor(anchor, 0);
  const faceFrame = faceFrameForAnchor(marker);

  assert.deepEqual([marker.x, marker.y, marker.z], [anchor.x, anchor.y, anchor.z]);
  assert.ok(frame.up[2] > 0.999999);
  assert.deepEqual(marker.roadForward, anchor.roadForward, "route frame remains source-owned");
  assert.deepEqual(marker.roadUp, anchor.roadUp, "billboard cannot rewrite route up");
  assert.ok(marker.faceUp[2] > 0.999999);
  assert.ok(Math.abs(marker.faceForward[2]) < 1e-12);
  assert.ok(Math.abs(Math.atan2(marker.faceForward[1], marker.faceForward[0])
    - Math.atan2(anchor.roadForward[1], anchor.roadForward[0])) < 1e-12);
  assert.deepEqual(faceFrame.forward, marker.faceForward);

  const world = markerWorldMatrix(marker, 1);
  const deviceProjection = [
    [960, -1000, 0],
    [540, 0, -1000],
    [1, 0, 0],
  ];
  const screenY = (localY) => {
    const point = transformPointRowMajor(world, { x: 0, y: localY, z: 0 });
    return projectPoint(deviceProjection, point[0], point[1], point[2]).y;
  };
  assert.ok(screenY(4) < screenY(0), "the sign face must project above its pole base");
});

test("upright sign local left/right and top/bottom stay readable on screen", () => {
  const world = markerWorldMatrix({ x: 40, y: 0, z: 0, headingRad: 0 }, 1);
  const deviceProjection = [
    [960, -1000, 0],
    [540, 0, -1000],
    [1, 0, 0],
  ];
  const screen = (point) => {
    const transformed = transformPointRowMajor(world, point);
    const projected = projectPoint(
      deviceProjection,
      transformed[0],
      transformed[1],
      transformed[2],
    );
    return { x: projected.x, y: projected.y };
  };

  assert.ok(screen({ x: -1, y: 0, z: 0 }).x < screen({ x: 1, y: 0, z: 0 }).x);
  assert.ok(screen({ x: 0, y: 1, z: 0 }).y < screen({ x: 0, y: -1, z: 0 }).y);
});

test("Three placement consumes heading and applies held opacity from shared tokens", () => {
  const descriptor = describeSignboard({ shape: AR_SHAPE.BAR, primary: "test" });
  const textureFactory = () => ({
    repeat: { x: 1, y: 1 },
    offset: { x: 0, y: 0 },
    dispose() {},
  });
  const group = createThreeSignboardGroup(descriptor, {
    textureFactory,
    shadowTextureFactory: textureFactory,
  });

  assert.equal(placeThreeSignboardGroup(group, {
    x: 30, y: -1, z: 0.2, headingRad: 0.25,
  }, { scale: 0.9, alpha: 0.8, held: true }), true);
  assert.equal(group.matrixAutoUpdate, false);
  assert.equal(group.matrix.elements[12], 30);
  assert.equal(group.matrix.elements[13], -1);
  assert.equal(group.matrix.elements[14], 0.2);

  const faceOpacity = group.getObjectByName("signboard:face").material.opacity;
  assert.equal(group.getObjectByName("signboard:face").material.map.flipY, true);
  assert.equal(group.getObjectByName("signboard:face").material.map.repeat.x, 1);
  assert.equal(group.getObjectByName("signboard:face").material.map.offset.x, 0);
  assert.equal(group.getObjectByName("signboard:face").material.side, FrontSide);
  assert.equal(group.getObjectByName("signboard:face-back").material.side, FrontSide);
  assert.equal(group.getObjectByName("signboard:face-back").rotation.y, Math.PI);
  assert.equal(
    group.getObjectByName("signboard:face-back").material.map,
    group.getObjectByName("signboard:face").material.map,
  );
  assert.ok(Math.abs(faceOpacity - descriptor.opacity * AR_OPACITY.group * 0.8 * AR_OPACITY.held) < 1e-12);
});

test("sign texture maps canvas pixel corners to the same visible face corners", () => {
  const descriptor = describeSignboard({ shape: AR_SHAPE.BAR, primary: "test" });
  const textureFactory = () => ({
    repeat: { x: 1, y: 1 },
    offset: { x: 0, y: 0 },
    dispose() {},
  });
  const group = createThreeSignboardGroup(descriptor, {
    textureFactory,
    shadowTextureFactory: textureFactory,
  });
  const face = group.getObjectByName("signboard:face");
  const texture = face.material.map;
  const positions = face.geometry.attributes.position.array;
  const uvs = face.geometry.attributes.uv.array;

  const sourcePixelAt = (localX, localY) => {
    let vertex = -1;
    for (let index = 0; index < positions.length / 3; index += 1) {
      if (positions[index * 3] === localX && positions[index * 3 + 1] === localY) {
        vertex = index;
        break;
      }
    }
    assert.notEqual(vertex, -1);
    const u = uvs[vertex * 2] * texture.repeat.x + texture.offset.x;
    const v = uvs[vertex * 2 + 1];
    return { x: u, y: texture.flipY ? 1 - v : v };
  };

  const localXs = Array.from({ length: positions.length / 3 }, (_, index) => positions[index * 3]);
  const localYs = Array.from({ length: positions.length / 3 }, (_, index) => positions[index * 3 + 1]);
  const left = Math.min(...localXs);
  const right = Math.max(...localXs);
  const top = Math.max(...localYs);
  const bottom = Math.min(...localYs);
  assert.deepEqual(sourcePixelAt(left, top), { x: 0, y: 0 });
  assert.deepEqual(sourcePixelAt(right, top), { x: 1, y: 0 });
  assert.deepEqual(sourcePixelAt(left, bottom), { x: 0, y: 1 });
  assert.deepEqual(sourcePixelAt(right, bottom), { x: 1, y: 1 });
});

test("opposite route headings select a readable FrontSide face without mirrored UVs", () => {
  const descriptor = describeSignboard({ shape: AR_SHAPE.BAR, primary: "좌회전" });
  const textureFactory = () => ({
    repeat: { x: 1, y: 1 },
    offset: { x: 0, y: 0 },
    dispose() {},
  });
  const group = createThreeSignboardGroup(descriptor, {
    textureFactory,
    shadowTextureFactory: textureFactory,
  });
  const projection = stageProjectionMatrix({
    calibTransform: [
      [964, 2648, 0],
      [604, 0, 2648],
      [1, 0, 0],
    ],
    scale: 1,
    tx: 0,
    ty: 0,
    stageWidth: 1928,
    stageHeight: 1208,
  });

  const projectedFace = (mesh) => {
    mesh.updateMatrix();
    const positions = mesh.geometry.attributes.position;
    const uvs = mesh.geometry.attributes.uv;
    return Array.from({ length: positions.count }, (_, index) => {
      const local = new Vector3(positions.getX(index), positions.getY(index), positions.getZ(index));
      local.applyMatrix4(mesh.matrix).applyMatrix4(group.matrix);
      const clip = transformPointRowMajor(projection, local);
      return {
        x: clip[0] / clip[3],
        y: clip[1] / clip[3],
        u: uvs.getX(index),
        v: uvs.getY(index),
      };
    });
  };
  const winding = (mesh, points) => {
    const [aIndex, bIndex, cIndex] = mesh.geometry.index.array;
    const [a, b, c] = [points[aIndex], points[bIndex], points[cIndex]];
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  };
  const assertReadable = (points) => {
    const sourceLeft = points.find((point) => point.u === 0 && point.v === 1);
    const sourceRight = points.find((point) => point.u === 1 && point.v === 1);
    assert.ok(sourceLeft.x < sourceRight.x, "canvas left must remain screen-left");
  };

  const front = group.getObjectByName("signboard:face");
  const back = group.getObjectByName("signboard:face-back");
  placeThreeSignboardGroup(group, { x: 40, y: 0, z: 0, headingRad: 0 });
  let frontPoints = projectedFace(front);
  let backPoints = projectedFace(back);
  assert.ok(winding(front, frontPoints) > 0);
  assert.ok(winding(back, backPoints) < 0);
  assertReadable(frontPoints);

  placeThreeSignboardGroup(group, { x: 40, y: 0, z: 0, headingRad: Math.PI });
  frontPoints = projectedFace(front);
  backPoints = projectedFace(back);
  assert.ok(winding(front, frontPoints) < 0);
  assert.ok(winding(back, backPoints) > 0);
  assertReadable(backPoints);
});
