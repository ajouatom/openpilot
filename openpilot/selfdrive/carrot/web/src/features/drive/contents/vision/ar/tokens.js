/* AR 마커 디자인 토큰.
 *
 * 크기는 전부 "미터"다. AR 마커는 도로 위 실제 크기를 가져야 원근이 자연스럽고,
 * 화면 크기에 맞춘 보정은 responsive 계층에서 별도로 clamp 한다(아래 VIEWPORT_TIERS).
 * 색은 기존 Carrot HUD 계열(초록 accent)과 맞추되, 영상 위에서 뭉개지지 않도록
 * 순백 대신 약간 채도가 있는 값을 쓴다.
 */

export const AR_MARKER_KIND = Object.freeze({
  TURN_GATE: "turn_gate",             // 회전/분기/램프/로터리 게이트
  COMMIT_ARROW: "commit_arrow",       // 진입 화살표
  DESTINATION_PIN: "destination_pin", // 목적지
  CAUTION_SIGN: "caution_sign",       // 단속카메라/과속방지턱/경찰
  LANE_BAND: "lane_band",             // 권장 차선군
  SECTION_GATE: "section_gate",       // 구간단속 진입/진행
  SIGNAL_HEAD: "signal_head",         // 신호등 램프
  CROSSROAD_CARD: "crossroad_card",   // 교차로 프리뷰 카드
  SPEED_SIGN: "speed_sign",           // 제한속도 표지
});

/* 마커는 두 부류다. 그리는 방법이 근본적으로 다르므로 섞으면 안 된다.
 *
 *   BILLBOARD  카메라를 향한 표지판. 도로 위 한 점에 세우고 거리에 따라 크기만
 *              변한다. 원근 왜곡이 없어 2D 로 정확히 그릴 수 있다.
 *   SURFACE    노면에 눕는 짧은 권장 차선 BAND. model/geo 경로가 anchor와
 *              방향을 정하지만 형상은 승인 프리뷰 component 그대로다.
 *
 * 이 구분이 없으면 차선 밴드가 표지판으로 취급되어 조용히 사라진다.
 */
export const AR_MARKER_FORM = Object.freeze({ BILLBOARD: "billboard", SURFACE: "surface" });

const SURFACE_KINDS = Object.freeze(new Set([
  AR_MARKER_KIND.LANE_BAND,
]));

export function markerForm(kind) {
  return SURFACE_KINDS.has(kind) ? AR_MARKER_FORM.SURFACE : AR_MARKER_FORM.BILLBOARD;
}

/* 차량-이벤트 거리에 따른 표시 단계.
 * 원거리는 "방향 미리보기"이고 바닥에 정밀하게 붙이지 않는다(과장 금지 원칙). */
export const AR_PHASE = Object.freeze({
  PREVIEW: "preview",     // 멀다. 소실점 근처 빌보드만.
  APPROACH: "approach",   // 접근 중. 평면 게이트.
  PRECISE: "precise",     // 정밀 정합 구간. 완전 3D.
  COMMIT: "commit",       // 진입 직전. 화살표 강조.
  PASSED: "passed",       // 통과. fade-out.
});

/* 경계값은 속도로 스케일된다(고속일수록 더 일찍 보여야 함).
 * baseMps 기준 거리이고, 실제 경계는 phaseBoundaries()가 계산한다. */
export const AR_PHASE_BASE = Object.freeze({
  baseMps: 16.7,          // 60km/h 기준
  previewEnterM: 300,
  approachEnterM: 150,
  preciseEnterM: 70,
  commitEnterM: 15,
  passedExitM: -8,        // 이벤트를 지나친 뒤 여유
  speedScaleMin: 0.6,
  speedScaleMax: 1.8,
});

export const AR_COLOR = Object.freeze({
  // 회전 안내: Carrot accent 계열
  turn: Object.freeze({ edge: [0.21, 0.95, 0.63], fill: [0.09, 0.45, 0.30], glow: [0.35, 1.0, 0.70] }),
  // 주의/단속: 앰버. 빨강은 FCW 등 기존 경고와 충돌하므로 피한다.
  caution: Object.freeze({ edge: [1.0, 0.78, 0.25], fill: [0.42, 0.30, 0.06], glow: [1.0, 0.86, 0.45] }),
  // 목적지
  destination: Object.freeze({ edge: [1.0, 0.45, 0.78], fill: [0.42, 0.12, 0.28], glow: [1.0, 0.60, 0.85] }),
  // 차선 안내
  lane: Object.freeze({ edge: [0.35, 0.82, 1.0], fill: [0.08, 0.30, 0.44], glow: [0.55, 0.90, 1.0] }),
  // 신뢰도 낮음(preview 등)에서 덧씌우는 감쇠
  uncertain: Object.freeze({ edge: [0.72, 0.76, 0.78], fill: [0.20, 0.22, 0.24], glow: [0.80, 0.84, 0.86] }),
});

/* 실제 도로 치수(m). 차선폭은 lateralPlan.laneWidth 로 덮어쓰되 아래 범위로 clamp. */
export const AR_GEOMETRY = Object.freeze({
  laneWidthDefaultM: 3.5,
  laneWidthMinM: 2.8,
  laneWidthMaxM: 4.2,

  gate: Object.freeze({
    heightM: 4.2,          // 상단 보 높이
    postWidthM: 0.26,      // 기둥 두께
    beamHeightM: 0.34,     // 상단 보 두께
    depthM: 0.22,          // 앞뒤 두께(3D 감)
    sideMarginM: 0.15,     // 차선폭 대비 여유
    chevronCount: 3,       // 상단 보의 방향 셰브런 수
  }),

  arrow: Object.freeze({
    chevronCount: 3,
    chevronWidthM: 2.2,
    chevronDepthM: 1.1,
    chevronThickM: 0.42,
    spacingM: 2.6,
    liftM: 0.06,           // 노면 z-fighting 방지
  }),

  pin: Object.freeze({
    heightM: 5.0,
    headRadiusM: 0.85,
    stemRadiusM: 0.12,
    radialSegments: 12,
  }),

  sign: Object.freeze({
    diameterM: 1.7,
    heightM: 3.2,
    ringThickM: 0.22,
    radialSegments: 20,
  }),

  laneBand: Object.freeze({
    lengthM: 26,
    liftM: 0.03,
    fadeStartRatio: 0.55,  // 길이의 이 지점부터 알파 감쇠
  }),
});

/* 반응형 계층. 주행 워크스페이스는 분할 시 50% 이하로 줄고, 미니HUD 모드에서는
 * 253px 폭까지 내려간다. 그래서 뷰포트 최소변 기준으로 단계를 나눈다. */
export const AR_VIEWPORT_TIERS = Object.freeze([
  Object.freeze({
    id: "xs", maxMinSidePx: 380,
    maxHeightRatio: 0.34,   // 마커 화면 높이 상한(뷰포트 높이 대비)
    maxWidthRatio: 0.56,
    minWidthPx: 112,
    safeMarginPx: 12,
    minHeightPx: 18,
    labels: false,          // 텍스트 라벨 숨김(공간 없음)
    chevronCount: 2,
    lod: "flat",            // 3D 디테일 생략
    maxMarkers: 1,
  }),
  Object.freeze({
    id: "sm", maxMinSidePx: 560,
    maxHeightRatio: 0.38, maxWidthRatio: 0.54, minWidthPx: 136, safeMarginPx: 14,
    minHeightPx: 22, labels: true,
    chevronCount: 2, lod: "flat", maxMarkers: 2,
  }),
  Object.freeze({
    id: "md", maxMinSidePx: 900,
    maxHeightRatio: 0.42, maxWidthRatio: 0.52, minWidthPx: 164, safeMarginPx: 16,
    minHeightPx: 26, labels: true,
    chevronCount: 3, lod: "solid", maxMarkers: 2,
  }),
  Object.freeze({
    id: "lg", maxMinSidePx: Infinity,
    maxHeightRatio: 0.46, maxWidthRatio: 0.50, minWidthPx: 192, safeMarginPx: 18,
    minHeightPx: 30, labels: true,
    chevronCount: 3, lod: "solid", maxMarkers: 3,
  }),
]);

/* AR 오버레이는 2D HUD보다 무거우므로 DPR 상한을 더 낮게 둔다. */
export const AR_RENDER = Object.freeze({
  backend: "three",
  failureMode: "hidden",
  canvas2dFallback: false,
  devicePixelRatioMax: 2,
  // The camera/model snapshots remain frame-synchronised, while the browser
  // presentation loop resamples their odometry at 30 Hz for fluid motion.
  targetFps: 30,
  degradedFps: 20,
  performance: Object.freeze({
    slowBudgetRatio: 0.90,
    degradeWindowFrames: 12,
    degradeSlowFrames: 9,
  }),
  // 신뢰도에 따른 전체 알파. quality_gate 가 confidence 를 준다.
  alphaByConfidence: Object.freeze({ high: 1.0, medium: 0.72, low: 0.42, hidden: 0 }),
  // 15fps에서도 한두 프레임의 sync 흔들림이 완전 점멸로 보이지 않게 한다.
  // 같은 cache entry가 돌아오면 현재 alpha에서 즉시 반대 방향으로 이어진다.
  fadeInMs: 280,
  fadeOutMs: 420,
  cacheGraceMs: 900,
  timelineDiscontinuityMs: 250,
  // Fixed world geometry gets only a small far-distance readability assist.
  farLegibilityScaleMax: 1.25,
  // Natural perspective may fill most of the view before the near-plane hide.
  worldObjectMaxHeightRatio: 0.82,
  worldObjectMaxWidthRatio: 0.90,
  // 카메라 근평면을 스치면 원근 투영이 폭발하고 표지/지주의 화면 순서도
  // 뒤집혀 보일 수 있다. 이 깊이 안에서는 이미 통과 중인 앵커로 처리한다.
  nearMarkerDepthM: 3.0,
  nearUprightMarkerDepthM: 8.0,
  presentation: Object.freeze({
    positionHalfLifeMs: 160,
    orientationHalfLifeMs: 220,
    scaleHalfLifeMs: 180,
    maxStepMs: 250,
    snapDistanceM: 2.5,
    // Product-only mounting corrections. Preview geometry and component tokens
    // stay unchanged; the road support grows upward from the same anchor.
    uprightScale: 0.82,
    mountLiftM: 1.05,
    nearWorldScaleMin: 0.62,
  }),
});

/* Upright markers retain their anchor and vertical axis. Bounded yaw keeps a
 * 90-degree maneuver from showing only the sign's edge to the driver. */
export const AR_BILLBOARD = Object.freeze({
  maxYawRad: Math.PI * 0.39,
});

/* Comma GPS는 TMap 좌표를 대체하지 않고 geo anchor의 교차검증에만 쓴다. */
export const AR_POSITION_QUALITY = Object.freeze({
  maxGpsAgeMs: 1500,
  maxHorizontalAccuracyM: 15,
  maxPositionSeparationM: 30,
  separationAccuracyMultiplier: 3,
  minBearingCheckSpeedMps: 2,
  maxBearingAccuracyDeg: 20,
  maxHeadingDisagreementDeg: 35,
});

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function finite(value, fallback) {
  const n = Number(value);
  return Number.isFinite(n) ? n : fallback;
}

/** 속도에 따라 스케일된 단계 경계(m). */
export function phaseBoundaries(egoSpeedMps) {
  const speed = Math.max(0, finite(egoSpeedMps, AR_PHASE_BASE.baseMps));
  const scale = clamp(
    speed / AR_PHASE_BASE.baseMps,
    AR_PHASE_BASE.speedScaleMin,
    AR_PHASE_BASE.speedScaleMax,
  );
  return Object.freeze({
    preview: AR_PHASE_BASE.previewEnterM * scale,
    approach: AR_PHASE_BASE.approachEnterM * scale,
    precise: AR_PHASE_BASE.preciseEnterM * scale,
    commit: AR_PHASE_BASE.commitEnterM,   // 진입 직전은 속도로 늘리지 않는다
    passed: AR_PHASE_BASE.passedExitM,
  });
}

/** 거리 → 표시 단계. */
export function phaseForDistance(distanceM, egoSpeedMps) {
  const d = finite(distanceM, NaN);
  if (!Number.isFinite(d)) return AR_PHASE.PASSED;
  const b = phaseBoundaries(egoSpeedMps);
  if (d < b.passed) return AR_PHASE.PASSED;
  if (d <= b.commit) return AR_PHASE.COMMIT;
  if (d <= b.precise) return AR_PHASE.PRECISE;
  if (d <= b.approach) return AR_PHASE.APPROACH;
  if (d <= b.preview) return AR_PHASE.PREVIEW;
  return AR_PHASE.PASSED;   // 아직 너무 멀다 → 표시하지 않음
}

/** 뷰포트 최소변으로 반응형 계층 선택. */
export function viewportTier(widthPx, heightPx) {
  const minSide = Math.max(1, Math.min(finite(widthPx, 1), finite(heightPx, 1)));
  for (const tier of AR_VIEWPORT_TIERS) {
    if (minSide <= tier.maxMinSidePx) return tier;
  }
  return AR_VIEWPORT_TIERS[AR_VIEWPORT_TIERS.length - 1];
}

/** lateralPlan.laneWidth 를 안전 범위로 clamp. */
export function resolveLaneWidthM(laneWidthM) {
  return clamp(
    finite(laneWidthM, AR_GEOMETRY.laneWidthDefaultM),
    AR_GEOMETRY.laneWidthMinM,
    AR_GEOMETRY.laneWidthMaxM,
  );
}

export { clamp as arClamp };
