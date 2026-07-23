import {
  roadFrameFields,
  roadFrameForAnchor,
  roadFrameFromForward,
} from "./road_frame.js";
import { AR_COORDINATE_FRAME, routeFluVectorToDeviceFrd } from "./coordinate_frames.js";
import { modelRoadObservation } from "./road_observation.js";

/* Phase 3 — 도로 좌표 → 화면 픽셀 투영.
 *
 * home_drive.js 의 투영과 **같은 수식**을 쓴다. 다른 수식을 쓰면 AR 표지가
 * 기존 차선/경로 오버레이와 미세하게 어긋나 보인다.
 *   화면 = intrinsics · calibration · [x, y, z]
 * camera transform 입력은 openpilot FRD, AR road math는 route-local FLU다.
 * 둘 사이의 부호 변환은 projectRouteFluPoint()에서만 수행한다.
 *
 * 이 파일은 DOM/three 에 의존하지 않는다. 순수 수학이라 테스트가 쉽다.
 */

function finite(v, fallback = null) {
  const n = Number(v);
  return Number.isFinite(n) ? n : fallback;
}

export function mat3Vector(m, v) {
  return [
    m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
    m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
    m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
  ];
}

/**
 * 기존 Carrot Vision이 제공한 calibrated FRD transform으로 영상에 투영한다.
 * AR은 sensor/profile/calibration을 다시 만들지 않고 stage snapshot만 소비한다.
 */
export function projectPoint(transform, x, y, z) {
  if (!transform) return null;
  const v = mat3Vector(transform, [x, y, z]);
  if (!Number.isFinite(v[2]) || v[2] <= 1e-3) return null;
  return { x: v[0] / v[2], y: v[1] / v[2], depth: v[2] };
}

/** AR route-local FLU 좌표를 명시적으로 FRD로 바꾼 뒤 영상에 투영한다. */
export function projectRouteFluPoint(transform, x, y, z) {
  const point = routeFluVectorToDeviceFrd([x, y, z]);
  return point ? projectPoint(transform, point[0], point[1], point[2]) : null;
}

/** Raw calibrated depth for lifecycle classification before projection clips. */
export function routeFluProjectionDepth(transform, x, y, z) {
  if (!transform) return null;
  const point = routeFluVectorToDeviceFrd([x, y, z]);
  if (!point) return null;
  const projected = mat3Vector(transform, point);
  return Number.isFinite(projected[2]) ? projected[2] : null;
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
 * route-local FLU로 변환된 modelV2.position(x/y/z 배열)에서 전방 지점을 찾는다.
 * GPS를 화면에 직접 찍지 않고 "내 경로 위 N미터"라는 road observation을
 * 만든다. 이 값은 최초 world anchor 또는 명시적 handoff에만 쓰며, 이미 승인된
 * marker를 매 model frame마다 끌어당기는 좌표가 아니다.
 */
export function pointOnPath(position, distanceM) {
  const observation = modelRoadObservation(position, distanceM);
  if (!observation) return null;
  // Bounded x/y/z tangent keeps the marker aligned with valid road grades
  // without letting a single model horizon spike rotate or lift the marker.
  const roadFrame = roadFrameFromForward(observation.tangent);
  const { tangent, ...fields } = observation;
  return Object.freeze({
    ...fields,
    ...roadFrameFields(roadFrame),
    coordinateFrame: AR_COORDINATE_FRAME.ROUTE_FLU,
  });
}

/**
 * 앵커 지점에 세운 "문(gate)" 의 네 모서리를 route-local FLU로 만든다.
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
    const p = projectRouteFluPoint(transform, c.x, c.y, c.z);
    if (!p) return null;
    out.push(stage ? toStage(p, stage) : p);
  }
  return Object.freeze(out);
}
