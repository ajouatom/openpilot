/* AR 마커 반응형/적응형 계층.
 *
 * AR 마커는 "실제 미터 크기"를 가지므로 원근에 따라 자동으로 커지고 작아진다.
 * 그대로 두면 두 가지 실패가 생긴다.
 *   1) 멀면 몇 px 로 뭉개져 안 보인다.
 *   2) 가까우면 화면을 가려 도로가 안 보인다(안전 문제).
 * 그래서 "월드 크기는 진짜로 두되, 투영된 화면 크기를 clamp" 한다.
 *
 * 추가로 주행 워크스페이스는 분할 시 50% 이하로 줄고 미니HUD 모드에서는 253px
 * 폭까지 내려간다. 그 구간에서는 디테일을 줄이고 마커 수를 제한한다.
 */

import {
  AR_PHASE,
  AR_RENDER,
  arClamp,
  phaseBoundaries,
  phaseForDistance,
  viewportTier,
} from "./tokens.js";
import { AR_EMPHASIS } from "./design_tokens.js";

function finite(value, fallback) {
  const n = Number(value);
  return Number.isFinite(n) ? n : fallback;
}

/** 캔버스 크기 + DPR 을 AR 용으로 정규화. 2D HUD 보다 DPR 상한이 낮다. */
export function resolveArCanvasSize(rect = {}, devicePixelRatio = 1) {
  const width = Math.max(1, Math.round(finite(rect.width, 1)));
  const height = Math.max(1, Math.round(finite(rect.height, 1)));
  const dpr = arClamp(finite(devicePixelRatio, 1), 1, AR_RENDER.devicePixelRatioMax);
  return Object.freeze({
    width, height, dpr,
    pixelWidth: Math.max(1, Math.round(width * dpr)),
    pixelHeight: Math.max(1, Math.round(height * dpr)),
    tier: viewportTier(width, height),
  });
}

/**
 * 월드 높이(m)가 주어진 거리에서 화면 몇 px 로 보이는지.
 * focalPx 는 카메라 intrinsics 의 초점거리(픽셀). projection.js 가 제공한다.
 */
export function projectedHeightPx(worldHeightM, distanceM, focalPx) {
  const h = finite(worldHeightM, 0);
  const d = Math.max(0.5, finite(distanceM, 1));
  const f = Math.max(1, finite(focalPx, 1));
  return (h * f) / d;
}

function smoothstep(value) {
  const t = arClamp(finite(value, 0), 0, 1);
  return t * t * (3 - 2 * t);
}

function mix(from, to, value) {
  return from + (to - from) * smoothstep(value);
}

/**
 * Continuous counterpart of the approved discrete phase scales.
 *
 * Descriptors still use the exact preview token for their current phase. The
 * runtime matrix compensates between those token values so crossing a phase
 * boundary never makes the whole marker pop to a different size.
 */
export function distanceEmphasisScale(distanceM, egoSpeedMps) {
  const distance = finite(distanceM, NaN);
  const boundaries = phaseBoundaries(egoSpeedMps);
  const preview = AR_EMPHASIS.preview.scale;
  const approach = AR_EMPHASIS.approach.scale;
  const precise = AR_EMPHASIS.precise.scale;
  const commit = AR_EMPHASIS.commit.scale;
  if (!Number.isFinite(distance) || distance >= boundaries.preview) return preview;
  if (distance > boundaries.approach) {
    return mix(
      preview,
      approach,
      (boundaries.preview - distance) / Math.max(1e-6, boundaries.preview - boundaries.approach),
    );
  }
  if (distance > boundaries.precise) {
    return mix(
      approach,
      precise,
      (boundaries.approach - distance) / Math.max(1e-6, boundaries.approach - boundaries.precise),
    );
  }
  if (distance > boundaries.commit) {
    return mix(
      precise,
      commit,
      (boundaries.precise - distance) / Math.max(1e-6, boundaries.precise - boundaries.commit),
    );
  }
  return commit;
}

/**
 * 화면 크기 clamp 를 만족시키는 월드 스케일 배수를 구한다.
 * 반환 1.0 이면 보정 없음. 1.0 초과면 실제보다 크게(멀리서도 보이게),
 * 미만이면 작게(가까이서 시야를 덜 가리게) 그린다.
 *
 * 과장을 막기 위해 보정 배수 자체에도 상한/하한을 둔다. 이 한도를 넘으면
 * 마커를 억지로 키우는 대신 상위 계층이 phase 를 낮추거나 숨겨야 한다.
 */
export function resolveWorldScale({
  worldHeightM,
  worldWidthM = null,
  distanceM,
  focalPx,
  centerX = null,
  rawHeightPx = null,
  rawWidthPx = null,
  canvas,
  scaleMin = 0.35,
  scaleMax = 2.2,
} = {}) {
  const size = canvas && canvas.tier ? canvas : resolveArCanvasSize(canvas || {}, 1);
  const tier = size.tier;
  const measuredHeightPx = finite(rawHeightPx, null);
  const raw = measuredHeightPx !== null && measuredHeightPx > 0
    ? measuredHeightPx
    : projectedHeightPx(worldHeightM, distanceM, focalPx);
  if (!(raw > 0)) return Object.freeze({
    scale: 1,
    projectedPx: 0,
    projectedWidthPx: 0,
    clamped: "none",
    tier: tier.id,
  });

  const maxPx = size.height * tier.maxHeightRatio;
  const minPx = tier.minHeightPx;
  const aspect = Math.max(0, finite(worldWidthM, 0)) / Math.max(0.001, finite(worldHeightM, 0));
  const measuredWidthPx = finite(rawWidthPx, null);
  const rawWidth = measuredWidthPx !== null && measuredWidthPx > 0
    ? measuredWidthPx
    : aspect > 0 ? raw * aspect : 0;
  let maxWidthPx = size.width * finite(tier.maxWidthRatio, 1);
  const projectedCenterX = centerX === null || centerX === undefined || centerX === ""
    ? null
    : finite(centerX, null);
  if (projectedCenterX !== null && rawWidth > 0) {
    const margin = Math.max(0, finite(tier.safeMarginPx, 0));
    const leftRoom = projectedCenterX - margin;
    const rightRoom = size.width - margin - projectedCenterX;
    const centeredCapacity = Math.max(0, Math.min(leftRoom, rightRoom) * 2);
    maxWidthPx = Math.min(
      maxWidthPx,
      Math.max(finite(tier.minWidthPx, 0), centeredCapacity),
    );
  }

  let scale = 1;
  let clamped = "none";
  if (raw > maxPx) {
    scale = maxPx / raw;
    clamped = "max";
  } else if (raw < minPx) {
    scale = minPx / raw;
    clamped = "min";
  }
  if (rawWidth > 0 && rawWidth * scale > maxWidthPx) {
    scale = maxWidthPx / rawWidth;
    clamped = projectedCenterX === null ? "width" : "width-edge";
  }
  const bounded = arClamp(scale, scaleMin, scaleMax);
  return Object.freeze({
    scale: bounded,
    projectedPx: raw * bounded,
    projectedWidthPx: rawWidth * bounded,
    rawPx: raw,
    rawWidthPx: rawWidth,
    maxWidthPx,
    clamped: bounded === scale ? clamped : `${clamped}-limited`,
    tier: tier.id,
  });
}

/**
 * 한 마커의 표시 계획을 만든다. 지오메트리를 만들기 전에 호출해
 * lod / chevron 수 / 라벨 표시 여부까지 결정한다.
 */
export function planMarker({
  distanceM,
  egoSpeedMps,
  worldHeightM,
  worldWidthM = null,
  focalPx,
  centerX = null,
  projectedHeightPx: measuredHeightPx = null,
  projectedWidthPx: measuredWidthPx = null,
  descriptorScale = null,
  minimumWorldScale = 1,
  maximumWorldScale = AR_RENDER.farLegibilityScaleMax,
  canvas,
  confidence = "high",
} = {}) {
  const size = canvas && canvas.tier ? canvas : resolveArCanvasSize(canvas || {}, 1);
  const tier = size.tier;
  const phase = phaseForDistance(distanceM, egoSpeedMps);
  let visible = phase !== AR_PHASE.PASSED
    && confidence !== "hidden"
    && finite(distanceM, NaN) === finite(distanceM, NaN);

  // preview 단계는 정밀 정합을 약속하지 않으므로 항상 단순 표현.
  const lodByPhase = phase === AR_PHASE.PREVIEW || phase === AR_PHASE.APPROACH ? "flat" : tier.lod;
  const tokenScale = finite(descriptorScale, null);
  const emphasisScale = distanceEmphasisScale(distanceM, egoSpeedMps);
  // Preview descriptors encode a discrete phase emphasis in their geometry.
  // Production replaces it with the continuous distance emphasis so phase
  // boundaries cannot make one world marker pop to a new physical size.
  const geometryScale = tokenScale !== null && tokenScale > 0
    ? emphasisScale / tokenScale
    : 1;
  const effectiveHeightM = finite(worldHeightM, 0) * geometryScale;
  const effectiveWidthM = worldWidthM === null
    ? null
    : finite(worldWidthM, 0) * geometryScale;
  const scale = resolveWorldScale({
    worldHeightM: effectiveHeightM,
    worldWidthM: effectiveWidthM,
    distanceM,
    focalPx,
    centerX,
    rawHeightPx: finite(measuredHeightPx, null) === null
      ? null
      : finite(measuredHeightPx, 0) * geometryScale,
    rawWidthPx: finite(measuredWidthPx, null) === null
      ? null
      : finite(measuredWidthPx, 0) * geometryScale,
    canvas: size,
    // The scale remains attached to world geometry, but each viewport tier may
    // bound a near object and enlarge a distant one for driver legibility.
    scaleMin: arClamp(finite(minimumWorldScale, 1), 0.2, 1),
    scaleMax: Math.max(1, finite(maximumWorldScale, AR_RENDER.farLegibilityScaleMax)),
  });
  let alpha = AR_RENDER.alphaByConfidence[confidence] ?? AR_RENDER.alphaByConfidence.low;

  // 축소 한도(scaleMin)에 걸리고도 상한을 넘으면, 마커가 도로를 가릴 만큼 크다는 뜻이다.
  // 억지로 그리지 말고 물러난다. 통과 직전(COMMIT)에는 어차피 앵커를 지나치는 중이므로
  // 숨기고, 그 밖에는 알파를 낮춰 시야를 확보한다.
  const overflowRatio = size.height > 0 ? scale.projectedPx / size.height : 0;
  const widthOverflow = size.width > 0
    && scale.projectedWidthPx / size.width > AR_RENDER.worldObjectMaxWidthRatio;
  const heightOverflow = overflowRatio > AR_RENDER.worldObjectMaxHeightRatio;
  // 높이 clamp 뒤 폭 clamp가 적용되면 clamped 값은 width-edge-limited가 된다.
  // 문자열이 max-limited인지로 판정하면 근거리 표지가 scaleMin에 걸린 채 화면을
  // 덮어도 놓친다. 최종 투영 크기 자체로 안전 한도 초과를 판정한다.
  const overflow = heightOverflow || widthOverflow;
  if (overflow) {
    // A near COMMIT marker transitions into the renderer's explicit PASSING
    // fade. Hiding it here would produce a one-frame disappearance first.
    alpha = Math.min(alpha, AR_RENDER.alphaByConfidence.low);
  }

  // 원거리에서는 셰브런을 줄여 시각적 노이즈를 낮춘다.
  const chevronCount = phase === AR_PHASE.PREVIEW
    ? 0
    : Math.min(tier.chevronCount, phase === AR_PHASE.COMMIT ? tier.chevronCount : tier.chevronCount - 1 || 1);

  return Object.freeze({
    visible,
    phase,
    lod: lodByPhase,
    chevronCount,
    // The descriptor geometry already contains its discrete token scale. This
    // factor replaces it with continuous phase emphasis, then applies the
    // bounded viewport legibility scale.
    worldScale: geometryScale * scale.scale,
    // Filter the final physical emphasis rather than the inverse descriptor
    // scale. Runtime adapters divide by the current token scale only at draw
    // time, so a discrete phase descriptor update cannot resize the marker.
    presentationScale: emphasisScale * scale.scale,
    distanceScale: emphasisScale,
    geometryScale,
    projectedPx: scale.projectedPx,
    projectedWidthPx: scale.projectedWidthPx,
    clamped: scale.scale === 1 && String(scale.clamped).endsWith("-limited")
      ? "world-fixed"
      : scale.clamped,
    overflow,
    overflowRatio,
    // preview 는 바닥 정합을 주장하지 않으므로 살짝 띄워 그린다.
    groundLocked: phase === AR_PHASE.PRECISE || phase === AR_PHASE.COMMIT,
    showLabel: tier.labels && phase !== AR_PHASE.PREVIEW,
    alpha,
    tier: tier.id,
    maxMarkers: tier.maxMarkers,
  });
}

/**
 * 여러 마커 후보에서 표시할 것만 고른다.
 * 우선순위: COMMIT > PRECISE > APPROACH > PREVIEW, 동률이면 가까운 것.
 * tier.maxMarkers 로 개수를 제한해 작은 화면에서 겹침을 막는다.
 */
export function selectVisibleMarkers(candidates = [], canvas, options = {}) {
  const size = canvas && canvas.tier ? canvas : resolveArCanvasSize(canvas || {}, 1);
  const preferredKeys = options.preferredKeys instanceof Set
    ? options.preferredKeys
    : new Set(options.preferredKeys || []);
  const preferContinuity = options.preferContinuity === true;
  const order = {
    [AR_PHASE.COMMIT]: 0,
    [AR_PHASE.PRECISE]: 1,
    [AR_PHASE.APPROACH]: 2,
    [AR_PHASE.PREVIEW]: 3,
  };
  const sourceOrder = {
    guidanceCurrent: 0,
    sdi: 1,
    section: 2,
    trafficSignal: 3,
    destination: 4,
    guidanceNext: 5,
    crossroad: 6,
    lane: 7,
  };
  const sorted = candidates
    .filter((candidate) => candidate && candidate.plan && candidate.plan.visible)
    .sort((a, b) => {
      const preferredA = preferredKeys.has(a.key) ? 0 : 1;
      const preferredB = preferredKeys.has(b.key) ? 0 : 1;
      if (preferContinuity && preferredA !== preferredB) return preferredA - preferredB;
      const phaseA = order[a.plan.phase] ?? 9;
      const phaseB = order[b.plan.phase] ?? 9;
      if (phaseA !== phaseB) return phaseA - phaseB;
      const sourceA = sourceOrder[a.item?.source || a.source] ?? 9;
      const sourceB = sourceOrder[b.item?.source || b.source] ?? 9;
      if (sourceA !== sourceB) return sourceA - sourceB;
      if (preferredA !== preferredB) return preferredA - preferredB;
      const distanceDelta = finite(a.distanceM, Infinity) - finite(b.distanceM, Infinity);
      if (distanceDelta !== 0) return distanceDelta;
      return String(a.key || "").localeCompare(String(b.key || ""));
    });

  const padding = Math.max(4, finite(size.tier.safeMarginPx, 0) * 0.5);
  const selected = [];
  const overlaps = (a, b) => {
    const first = a?.screenBounds;
    const second = b?.screenBounds;
    if (!first || !second) return false;
    return !(
      first.right + padding <= second.left
      || second.right + padding <= first.left
      || first.bottom + padding <= second.top
      || second.bottom + padding <= first.top
    );
  };
  for (const candidate of sorted) {
    if (selected.length >= size.tier.maxMarkers) break;
    if (selected.some((visible) => overlaps(candidate, visible))) continue;
    selected.push(candidate);
  }
  return Object.freeze(selected);
}
