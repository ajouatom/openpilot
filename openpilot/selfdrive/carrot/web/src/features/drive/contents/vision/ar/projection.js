import {
  roadFrameFields,
  roadFrameForAnchor,
  roadFrameFromForward,
} from "./road_frame.js";

/* Phase 3 — 도로 좌표 → 화면 픽셀 투영.
 *
 * home_drive.js 의 투영과 **같은 수식**을 쓴다. 다른 수식을 쓰면 AR 표지가
 * 기존 차선/경로 오버레이와 미세하게 어긋나 보인다.
 *   화면 = intrinsics · calibration · [x, y, z]
 * 좌표계는 device frame: +x 전방, +y 좌, +z 상.
 *
 * 이 파일은 DOM/three 에 의존하지 않는다. 순수 수학이라 테스트가 쉽다.
 */

/* 센서별 기준 해상도와 초점거리. home_drive.js 의 BASE_CAMERA 와 동일해야 한다. */
export const AR_CAMERA_PROFILES = Object.freeze({
  ar0231: Object.freeze({ width: 1928, height: 1208, focalX: 2648, focalY: 2648 }),
  ox03c10: Object.freeze({ width: 1928, height: 1208, focalX: 2648, focalY: 2648 }),
  os04c10: Object.freeze({ width: 2688, height: 1520, focalX: 2648 * (2688 / 1928), focalY: 2648 * (2688 / 1928) }),
  unknown: Object.freeze({ width: 1928, height: 1208, focalX: 2648, focalY: 2648 }),
});

export function cameraProfile(sensor) {
  const key = String(sensor || "").trim();
  return AR_CAMERA_PROFILES[key] || AR_CAMERA_PROFILES.unknown;
}

function finite(v, fallback = null) {
  const n = Number(v);
  return Number.isFinite(n) ? n : fallback;
}

/** 3x3 곱. row-major 배열. */
export function mat3Multiply(a, b) {
  const out = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let r = 0; r < 3; r += 1) {
    for (let c = 0; c < 3; c += 1) {
      out[r][c] = a[r][0] * b[0][c] + a[r][1] * b[1][c] + a[r][2] * b[2][c];
    }
  }
  return out;
}

export function mat3Vector(m, v) {
  return [
    m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
    m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
    m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
  ];
}

/** 영상 해상도에 맞춘 intrinsics. */
export function intrinsicsFor(sensor, videoWidth, videoHeight) {
  const p = cameraProfile(sensor);
  const w = Math.max(1, finite(videoWidth, p.width));
  const h = Math.max(1, finite(videoHeight, p.height));
  const sx = w / p.width;
  const sy = h / p.height;
  return [
    [p.focalX * sx, 0, w / 2],
    [0, p.focalY * sy, h / 2],
    [0, 0, 1],
  ];
}

/**
 * liveCalibration.rpyCalib → device→camera 회전 행렬.
 * openpilot 의 view_frame_from_device_frame 과 같은 순서(roll·pitch·yaw)를 쓴다.
 */
export function calibrationMatrix(rpyCalib) {
  const src = Array.isArray(rpyCalib) ? rpyCalib : null;
  if (!src) return null;
  const roll = finite(src[0]);
  const pitch = finite(src[1]);
  const yaw = finite(src[2]);
  if (roll === null || pitch === null || yaw === null) return null;

  const cr = Math.cos(roll), sr = Math.sin(roll);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const cy = Math.cos(yaw), sy = Math.sin(yaw);

  // device → view. 카메라 축(우=+x, 하=+y, 전=+z)으로 재배열까지 포함한다.
  const deviceFromView = [
    [cp * cy, -sy * cp, sp],
    [cr * sy + sr * sp * cy, cr * cy - sr * sp * sy, -sr * cp],
    [sr * sy - cr * sp * cy, sr * cy + cr * sp * sy, cr * cp],
  ];
  // view_frame_from_device_frame: [[0,-1,0],[0,0,-1],[1,0,0]]
  const viewFromDevice = [[0, -1, 0], [0, 0, -1], [1, 0, 0]];
  // 전치(=역행렬, 회전이므로)를 곱해 device→camera 를 만든다.
  const t = [
    [deviceFromView[0][0], deviceFromView[1][0], deviceFromView[2][0]],
    [deviceFromView[0][1], deviceFromView[1][1], deviceFromView[2][1]],
    [deviceFromView[0][2], deviceFromView[1][2], deviceFromView[2][2]],
  ];
  return mat3Multiply(viewFromDevice, t);
}

/** intrinsics · calibration. 매 프레임 재계산하지 말고 캐시해서 쓴다. */
export function calibratedTransform(sensor, videoWidth, videoHeight, rpyCalib) {
  const calib = calibrationMatrix(rpyCalib);
  if (!calib) return null;
  return mat3Multiply(intrinsicsFor(sensor, videoWidth, videoHeight), calib);
}

/**
 * device 좌표 → 영상 픽셀. 카메라 뒤(z<=0)면 null.
 * home_drive.js 의 projectPointPrecise 와 동일한 판정.
 */
export function projectPoint(transform, x, y, z) {
  if (!transform) return null;
  const v = mat3Vector(transform, [x, y, z]);
  if (!Number.isFinite(v[2]) || v[2] <= 1e-3) return null;
  return { x: v[0] / v[2], y: v[1] / v[2], depth: v[2] };
}

/** 영상 픽셀 → 스테이지(표시 영역) 픽셀. 영상이 스케일/오프셋되어 표시될 때. */
export function toStage(point, stage) {
  if (!point || !stage) return null;
  const scale = finite(stage.scale, 1);
  return {
    x: (point.x - finite(stage.videoWidth, 0) / 2) * scale + finite(stage.width, 0) / 2 + finite(stage.offsetX, 0),
    y: (point.y - finite(stage.videoHeight, 0) / 2) * scale + finite(stage.height, 0) / 2 + finite(stage.offsetY, 0),
    depth: point.depth,
  };
}

/* ── model path 위의 앵커 지점 ─────────────────────────────── */

/**
 * modelV2.position(x/y/z 배열) 에서 전방 distanceM 지점을 보간해 찾는다.
 * Phase 3 의 핵심: GPS 를 화면에 찍지 않고 "내 경로 위 N미터" 를 쓴다.
 * 드리프트가 0 인 이유가 여기다 — 매 프레임 새 model 에서 다시 뽑는다.
 */
export function pointOnPath(position, distanceM) {
  const xs = Array.isArray(position?.x) ? position.x : null;
  const ys = Array.isArray(position?.y) ? position.y : null;
  const zs = Array.isArray(position?.z) ? position.z : null;
  const d = finite(distanceM);
  if (!xs || !ys || xs.length < 2 || d === null || d < 0) return null;

  const n = Math.min(xs.length, ys.length);
  if (d > finite(xs[n - 1], 0)) return null;   // 경로보다 먼 지점은 만들지 않는다

  for (let i = 1; i < n; i += 1) {
    const x0 = finite(xs[i - 1], 0);
    const x1 = finite(xs[i], 0);
    if (x1 < d) continue;
    const span = x1 - x0;
    const t = span > 1e-6 ? (d - x0) / span : 0;
    const y0 = finite(ys[i - 1], 0), y1 = finite(ys[i], 0);
    const z0 = zs ? finite(zs[i - 1], 0) : 0;
    const z1 = zs ? finite(zs[i], 0) : 0;
    // The full x/y/z tangent keeps the marker aligned on grades as well as
    // curves. roadFrameFields also provides headingRad for older consumers.
    const dy = y1 - y0;
    const dz = z1 - z0;
    const roadFrame = roadFrameFromForward([
      span > 1e-6 ? span : 1e-6,
      dy,
      dz,
    ]);
    return Object.freeze({
      x: d,
      y: y0 + (y1 - y0) * t,
      z: z0 + (z1 - z0) * t,
      ...roadFrameFields(roadFrame),
      index: i,
    });
  }
  return null;
}

/**
 * 앵커 지점에 세운 "문(gate)" 의 네 모서리를 device 좌표로 만든다.
 * 도로 접선 방향으로 회전시켜 도로에 정합시킨다.
 */
export function gateCorners(anchor, { widthM = 3.6, heightM = 2.56, mountM = 0 } = {}) {
  if (!anchor) return null;
  const hw = widthM / 2;
  const frame = roadFrameForAnchor(anchor);
  const center = [anchor.x, anchor.y, anchor.z].map((value, index) => (
    finite(value, 0) + frame.up[index] * mountM
  ));
  const corner = (rightScale, upScale) => ({
    x: center[0] + frame.right[0] * rightScale + frame.up[0] * upScale,
    y: center[1] + frame.right[1] * rightScale + frame.up[1] * upScale,
    z: center[2] + frame.right[2] * rightScale + frame.up[2] * upScale,
  });
  return Object.freeze([
    corner(-hw, 0),
    corner(hw, 0),
    corner(hw, heightM),
    corner(-hw, heightM),
  ]);
}

/** 코너들을 화면 픽셀로. 하나라도 카메라 뒤면 null(부분 클리핑은 Phase 6). */
export function projectCorners(transform, corners, stage = null) {
  if (!transform || !corners) return null;
  const out = [];
  for (const c of corners) {
    const p = projectPoint(transform, c.x, c.y, c.z);
    if (!p) return null;
    out.push(stage ? toStage(p, stage) : p);
  }
  return Object.freeze(out);
}
