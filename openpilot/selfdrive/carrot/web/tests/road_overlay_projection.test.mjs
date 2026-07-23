import assert from "node:assert/strict";
import test from "node:test";

import { createRoadOverlayProjection } from "../src/features/drive/contents/vision/road_overlay_projection.js";

function ribbon(x) {
  const left = [{ x, y: 0 }, { x, y: 10 }];
  const right = [{ x: x + 2, y: 0 }, { x: x + 2, y: 10 }];
  return { left, right, center: [{ x: x + 1, y: 0 }], polygon: [...left, ...right] };
}

const TAU_MS = 140;

function makeProjection(recorded) {
  return createRoadOverlayProjection({
    projectPoint() {},
    projectPointPrecise() {},
    isRecordedReplayActive: () => recorded,
  });
}

/** 두 프레임을 dt 간격으로 흘려 보낸 뒤의 ribbon. */
function stepRibbon(projection, dtMs, tauMs = TAU_MS) {
  projection.resetFrame(1_000);
  projection.smoothRibbon("path", ribbon(0), tauMs);
  projection.resetFrame(1_000 + dtMs);
  return projection.smoothRibbon("path", ribbon(10), tauMs);
}

test("recorded replay uses the same temporal ribbon smoothing as live vision", () => {
  const recorded = stepRibbon(makeProjection(true), 50);
  const live = stepRibbon(makeProjection(false), 50);

  assert.deepEqual(recorded.left, live.left);
  assert.deepEqual(recorded.right, live.right);
  assert.deepEqual(recorded.center, live.center);
  // 필터를 우회하지 않는다: 목표(10)로 즉시 가지 않고 이전(0)에서 이동한다.
  assert.ok(recorded.left[0].x > 0 && recorded.left[0].x < 10);
});

// 렌더 간격은 20Hz 데이터·30fps 상한·스케줄러 지터로 계속 달라진다. 고정 alpha면
// 그때마다 실효 시정수가 흔들려 "툭툭 끊기는" 체감이 된다. 같은 tau라면 간격이
// 길수록 더 많이 접근해야 시간상 같은 속도가 된다.
test("temporal smoothing follows elapsed time, not call count", () => {
  const short = stepRibbon(makeProjection(false), 16);
  const long = stepRibbon(makeProjection(false), 50);

  assert.ok(long.left[0].x > short.left[0].x);

  // 지수 스무딩은 합성된다: 16ms씩 세 번은 48ms 한 번과 같아야 한다.
  // 프레임이 몇 번 나뉘어 들어오든 경과 시간이 같으면 같은 결과라는 뜻이다.
  const projection = makeProjection(false);
  projection.resetFrame(1_000);
  projection.smoothRibbon("path", ribbon(0), TAU_MS);
  let stepped = null;
  for (let i = 1; i <= 3; i += 1) {
    projection.resetFrame(1_000 + i * 16);
    stepped = projection.smoothRibbon("path", ribbon(10), TAU_MS);
  }
  const single = stepRibbon(makeProjection(false), 48);
  assert.ok(Math.abs(stepped.left[0].x - single.left[0].x) < 1e-9);
});

// 탭 복귀·seek처럼 긴 공백 뒤에는 과거 값을 끌고 오지 않고 목표로 스냅한다.
test("a long gap snaps instead of dragging stale geometry", () => {
  const snapped = stepRibbon(makeProjection(false), 400);
  assert.equal(snapped.left[0].x, 10);
});

// Distant model points were decimated up to 5:1 so the Canvas2D path would pay
// fewer per-point path operations. On the WebGL2 path those vertices are
// effectively free, and the decimation is precisely what makes a curve read as
// a few straight facets. Full detail must be used whenever the GPU path drives.
function ribbonPointCount(fullDetail) {
  const line = { x: [], y: [], z: [] };
  for (let i = 0; i < 40; i += 1) {
    line.x.push(i * 2.5);
    line.y.push(Math.sin(i / 6) * 3);
    line.z.push(0);
  }
  // Screen y must fall away monotonically with distance, as a real camera
  // projection does; buildRibbon drops samples that move back toward the camera.
  const project = (_calib, x, y) => ({ x: 640 + y * 12, y: 1000 - x * 8 });
  const projection = createRoadOverlayProjection({
    projectPoint: project,
    projectPointPrecise: project,
    maxDrawDistance: 100,
    isFullDetailActive: () => fullDetail,
  });
  projection.resetFrame(0);
  return projection.buildRibbon(null, line, 0.9, 0, 100).center.length;
}

test("the GPU geometry path keeps every model point instead of decimating", () => {
  const decimated = ribbonPointCount(false);
  const full = ribbonPointCount(true);

  assert.ok(full > decimated);
  // Every in-range model sample survives, so a curve stays a curve.
  assert.ok(full >= 40);
});

/** Largest turn between consecutive spans: the faceting a viewer actually sees. */
function worstKinkDegrees(points) {
  let worst = 0;
  for (let i = 1; i < points.length - 1; i += 1) {
    const a = Math.atan2(points[i].y - points[i - 1].y, points[i].x - points[i - 1].x);
    const b = Math.atan2(points[i + 1].y - points[i].y, points[i + 1].x - points[i].x);
    let delta = Math.abs(b - a);
    if (delta > Math.PI) delta = 2 * Math.PI - delta;
    worst = Math.max(worst, delta);
  }
  return worst * 180 / Math.PI;
}

function ribbonCenter(fullDetail) {
  const line = { x: [], y: [], z: [] };
  for (let i = 0; i < 40; i += 1) {
    line.x.push(i * 2.5);
    line.y.push(Math.sin(i / 6) * 3);
    line.z.push(0);
  }
  const project = (_calib, x, y) => ({ x: 640 + y * 12, y: 1000 - x * 8 });
  const projection = createRoadOverlayProjection({
    projectPoint: project,
    projectPointPrecise: project,
    maxDrawDistance: 100,
    isFullDetailActive: () => fullDetail,
  });
  projection.resetFrame(0);
  return projection.buildRibbon(null, line, 0.9, 0, 100).center;
}

// The model publishes only ~33 path samples, so a curve still breaks into
// visible straight spans even at full density. Catmull-Rom midpoints follow the
// curve those samples describe instead of the chord between them.
test("curve subdivision removes visible faceting on the GPU path", () => {
  const chord = worstKinkDegrees(ribbonCenter(false));
  const dense = worstKinkDegrees(ribbonCenter(true));

  assert.ok(dense < chord / 3, `expected a much smoother curve, got ${dense} vs ${chord}`);
  assert.ok(dense < 3);
  // Endpoints stay put; subdivision must not shorten or shift the ribbon.
  const plain = ribbonCenter(true);
  assert.ok(plain.length > 40);
});

/* Uniform Catmull-Rom overshoots when samples are unevenly spaced, and projected
 * road points bunch up hard toward the horizon. That overshoot drew as large
 * streaks across the frame. Centripetal knots plus a clamp keep every resampled
 * point inside the span it belongs to.
 *
 * buildRibbon only keeps samples that recede from the camera, so screen y must
 * stay monotonic through the ribbon. An overshooting spline breaks exactly that.
 */
test("curve resampling never overshoots its span", () => {
  const project = (_calib, x, y) => ({ x: 640 + y * 12, y: 1000 - x * 8 });
  // Horizon-like spacing: far samples collapse toward each other while the
  // lateral offset keeps swinging, which is where overshoot appeared.
  const line = { x: [], y: [], z: [] };
  for (let i = 0; i < 34; i += 1) {
    line.x.push(Math.pow(i / 33, 0.35) * 95);
    line.y.push(Math.sin(i / 3) * 6);
    line.z.push(0);
  }
  const projection = createRoadOverlayProjection({
    projectPoint: project,
    projectPointPrecise: project,
    maxDrawDistance: 100,
    isFullDetailActive: () => true,
  });
  projection.resetFrame(0);
  const center = projection.buildRibbon(null, line, 0.9, 0, 100).center;

  for (let i = 1; i < center.length; i += 1) {
    assert.ok(
      center[i].y <= center[i - 1].y + 1e-6,
      `resampled point ${i} moved back toward the camera: ${center[i - 1].y} -> ${center[i].y}`,
    );
  }
  assert.ok(center.length > 34);
});
