import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { spawnSync } from "node:child_process";
import test from "node:test";
import { fileURLToPath } from "node:url";

import {
  projectPoint,
  projectRouteFluPoint,
} from "../src/features/drive/contents/vision/ar/projection.js";
import {
  stageProjectionMatrix,
  transformPointRowMajor,
} from "../src/features/drive/contents/vision/ar/three_projection.js";
import { createRoadOverlayProjection } from "../src/features/drive/contents/vision/road_overlay_projection.js";

const TEST_DIR = dirname(fileURLToPath(import.meta.url));
const WEB_DIR = resolve(TEST_DIR, "..");
const REPO_ROOT = resolve(TEST_DIR, "../../../../..");
const FIXTURE_PATH = resolve(TEST_DIR, "fixtures/ar_projection_golden.json");
const GENERATOR_PATH = resolve(TEST_DIR, "generate_ar_projection_golden.py");
const fixture = JSON.parse(readFileSync(FIXTURE_PATH, "utf8"));

function nearlyEqual(actual, expected, tolerance = 1e-9) {
  assert.ok(
    Math.abs(actual - expected) <= tolerance,
    `expected ${actual} to be within ${tolerance} of ${expected}`,
  );
}

test("projection fixture stays generated from canonical openpilot Python transforms", () => {
  const result = spawnSync(process.env.PYTHON || "python", [GENERATOR_PATH], {
    cwd: REPO_ROOT,
    encoding: "utf8",
  });
  assert.equal(result.status, 0, result.stderr || result.stdout);
  assert.deepEqual(JSON.parse(result.stdout), fixture);
  assert.equal(fixture.schemaVersion, 1);
  assert.deepEqual(fixture.coordinateContract.frdFromFlu, [
    [1, 0, 0],
    [0, -1, 0],
    [0, 0, -1],
  ]);
});

test("Carrot Vision owns the browser camera profiles and AR does not duplicate them", () => {
  const visionSource = readFileSync(resolve(WEB_DIR, "js/realtime/home_drive.js"), "utf8");
  const arProjectionSource = readFileSync(
    resolve(WEB_DIR, "src/features/drive/contents/vision/ar/projection.js"),
    "utf8",
  );

  assert.match(visionSource, /ar_ox:\s*\{\s*width:\s*1928,\s*height:\s*1208,\s*focal:\s*2648\.0\s*\}/);
  assert.match(visionSource, /os04c10:\s*\{\s*width:\s*1344,\s*height:\s*760,\s*focal:\s*1141\.5\s*\}/);
  assert.match(visionSource, /window\.CarrotVisionStageTransform\s*=\s*Object\.freeze/);

  for (const duplicate of [
    "AR_CAMERA_PROFILES",
    "cameraProfile(",
    "intrinsicsFor(",
    "calibrationMatrix(",
    "calibratedTransform(",
    "2648",
    "1141.5",
  ]) {
    assert.equal(arProjectionSource.includes(duplicate), false, `AR projection duplicates ${duplicate}`);
  }
});

test("route FLU projects to the Python golden pixels exactly once through FRD", () => {
  for (const sample of fixture.cases) {
    sample.cameraFromDeviceFrd.forEach((row, rowIndex) => {
      row.forEach((value, columnIndex) => {
        nearlyEqual(
          value * fixture.coordinateContract.frdFromFlu[columnIndex][columnIndex],
          sample.cameraFromRouteFlu[rowIndex][columnIndex],
        );
      });
    });

    for (const point of sample.points) {
      const [x, y, z] = point.routeFlu;
      const projected = projectRouteFluPoint(sample.cameraFromDeviceFrd, x, y, z);
      assert.ok(projected, `${sample.id} point must project`);
      nearlyEqual(projected.x, point.pixel[0]);
      nearlyEqual(projected.y, point.pixel[1]);
      nearlyEqual(projected.depth, point.depth);
    }
  }
});

test("Three clip projection matches the same Python golden pixels", () => {
  for (const sample of fixture.cases) {
    const profile = fixture.profiles[sample.sensor];
    const stage = {
      calibTransform: sample.cameraFromDeviceFrd,
      scale: 1,
      tx: 0,
      ty: 0,
      stageWidth: profile.width,
      stageHeight: profile.height,
    };
    const matrix = stageProjectionMatrix(stage);
    assert.ok(matrix, `${sample.id} projection matrix must exist`);

    sample.points.forEach((point) => {
      const [x, y, z] = point.routeFlu;
      const clip = transformPointRowMajor(matrix, { x, y, z });
      nearlyEqual(clip[0] / clip[3], (2 * point.pixel[0]) / profile.width - 1);
      nearlyEqual(clip[1] / clip[3], 1 - (2 * point.pixel[1]) / profile.height);
      nearlyEqual(clip[3], point.depth);
    });
  }
});

test("AR ground anchors and Carrot Vision road geometry share the exact calibrated stage", () => {
  for (const sample of fixture.cases) {
    const profile = fixture.profiles[sample.sensor];
    const stage = {
      calibTransform: sample.cameraFromDeviceFrd,
      scale: 0.73,
      tx: -41.25,
      ty: 28.5,
      stageWidth: 1280,
      stageHeight: 720,
    };
    const roadProjection = createRoadOverlayProjection({
      maxDrawDistance: 120,
      projectPoint: (transform, x, y, z) => projectPoint(transform, x, y, z),
      projectPointPrecise: (transform, x, y, z) => projectPoint(transform, x, y, z),
    });
    const threeMatrix = stageProjectionMatrix(stage);

    for (const point of sample.points) {
      const [x, routeY, routeZ] = point.routeFlu;
      const road = roadProjection.buildRibbon(
        sample.cameraFromDeviceFrd,
        { x: [x], y: [-routeY], z: [-routeZ] },
        0.02,
        0,
        120,
      );
      assert.equal(road.center.length, 1, `${sample.id} road point must project`);
      const roadStageX = road.center[0].x * stage.scale + stage.tx;
      const roadStageY = road.center[0].y * stage.scale + stage.ty;

      const clip = transformPointRowMajor(threeMatrix, { x, y: routeY, z: routeZ });
      const arStageX = ((clip[0] / clip[3]) + 1) * stage.stageWidth / 2;
      const arStageY = (1 - clip[1] / clip[3]) * stage.stageHeight / 2;
      nearlyEqual(arStageX, roadStageX);
      nearlyEqual(arStageY, roadStageY);
    }
  }
});
