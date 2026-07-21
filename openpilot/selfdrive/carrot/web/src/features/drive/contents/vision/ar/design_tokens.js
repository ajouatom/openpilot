/* Carrot AR 표지 디자인 토큰.
 *
 * 설계 원칙 세 가지.
 *
 * 1) 모든 치수는 하나의 배율에서 파생한다.
 *    BASE_M 과 RATIO 만 바꾸면 전체가 같은 비율로 커지고 작아진다.
 *    개별 값을 손으로 조정하지 않는다.
 *
 * 2) 구분 축을 셋으로 늘린다. 이전 디자인은 "텍스트"만 달랐다.
 *    - 색   : 의미(안내/주의/규제/차선/목적지)
 *    - 실루엣: 형태(막대/마름모/원/밴드/핀)
 *    - 크기 : 긴급도
 *    운전 중에는 글자를 읽기 전에 색과 실루엣으로 먼저 판단한다.
 *
 * 3) 색·형태는 한국 도로표지 관습을 따른다. 운전자가 이미 학습한 코드를 쓰는 것이
 *    새 코드를 가르치는 것보다 항상 빠르다.
 *      안내 = 초록 사각   /   주의 = 노랑 마름모   /   규제 = 빨강 테두리 원
 */

/** 사용자 승인 self-contained preview의 고정 출처.
 * application code를 읽을 수 있게 나눈 것이 이 모듈이며, 값이나 파생 관계를
 * 바꿀 때에는 preview와 이 hash 계약을 함께 갱신해야 한다. */
export const AR_TOKEN_PREVIEW_CONTRACT = Object.freeze({
  file: "carrot_ar_token_preview.html",
  sha256: "f5395c5eabfeb6bfb55fb2a14e0e10665fee23076e3220187fac54abbe42336f",
  threeRevision: "185",
});

/* ── 배율 ────────────────────────────────────────────────── */

export const AR_BASE_M = 1.0;
export const AR_RATIO = 1.6;

function step(n) {
  return AR_BASE_M * Math.pow(AR_RATIO, n);
}

/** 치수 스케일(미터). 모든 크기는 여기서만 나온다. */
export const AR_SCALE = Object.freeze({
  xxs: +step(-3).toFixed(4),   // 0.2441
  xs: +step(-2).toFixed(4),    // 0.3906
  sm: +step(-1).toFixed(4),    // 0.6250
  md: +step(0).toFixed(4),     // 1.0000
  lg: +step(1).toFixed(4),     // 1.6000
  xl: +step(2).toFixed(4),     // 2.5600
  xxl: +step(3).toFixed(4),    // 4.0960
});

/* ── 의미 역할(tone) ─────────────────────────────────────── */

export const AR_TONE = Object.freeze({
  GUIDE: "guide",             // 방향 안내: 회전·직진·분기·램프
  CAUTION: "caution",         // 주의: 단속카메라·과속방지턱·경찰
  RESTRICT: "restrict",       // 규제: 제한속도·구간단속
  LANE: "lane",               // 차선 안내
  DESTINATION: "destination", // 목적지
});

/* 색은 "밝은 아스팔트 + 하늘"이 배경이라는 전제로 고른다.
 * WebRTC 압축 영상은 대비가 뭉개지므로 테두리(edge)를 반드시 함께 쓴다. */
export const AR_PALETTE = Object.freeze({
  [AR_TONE.GUIDE]: Object.freeze({
    surface: "#0E7A46", surfaceTop: "#17A45E",
    ink: "#FFFFFF", inkMuted: "rgba(255,255,255,0.78)",
    edge: "#7DFFC0", accent: "#3BE38B",
  }),
  [AR_TONE.CAUTION]: Object.freeze({
    surface: "#FFC53D", surfaceTop: "#FFD976",
    ink: "#1A1200", inkMuted: "rgba(26,18,0,0.72)",
    edge: "#FFE9A8", accent: "#FF8A00",
  }),
  [AR_TONE.RESTRICT]: Object.freeze({
    surface: "#FFFFFF", surfaceTop: "#FFFFFF",
    ink: "#10141A", inkMuted: "rgba(16,20,26,0.68)",
    edge: "#FF5A5E", accent: "#E5262B",
  }),
  [AR_TONE.LANE]: Object.freeze({
    surface: "#0B6E8C", surfaceTop: "#1394B5",
    ink: "#FFFFFF", inkMuted: "rgba(255,255,255,0.78)",
    edge: "#9BE8FF", accent: "#4DD2FF",
  }),
  [AR_TONE.DESTINATION]: Object.freeze({
    surface: "#8E2A6B", surfaceTop: "#B93C8D",
    ink: "#FFFFFF", inkMuted: "rgba(255,255,255,0.80)",
    edge: "#FFA8DC", accent: "#FF6BC4",
  }),
});

/* ── 실루엣 ──────────────────────────────────────────────── */

export const AR_SHAPE = Object.freeze({
  BAR: "bar",         // 가로 막대(안내). 한국 안내표지 = 초록 사각
  DIAMOND: "diamond", // 마름모(주의). 한국 주의표지
  CIRCLE: "circle",   // 원(규제). 한국 규제표지
  BAND: "band",       // 노면 밴드(차선)
  PIN: "pin",         // 핀(목적지)
});

/** tone → 기본 실루엣. 형태만으로 의미가 읽히게 1:1 로 묶는다. */
export const AR_TONE_SHAPE = Object.freeze({
  [AR_TONE.GUIDE]: AR_SHAPE.BAR,
  [AR_TONE.CAUTION]: AR_SHAPE.DIAMOND,
  [AR_TONE.RESTRICT]: AR_SHAPE.CIRCLE,
  [AR_TONE.LANE]: AR_SHAPE.BAND,
  [AR_TONE.DESTINATION]: AR_SHAPE.PIN,
});

/* ── 형상 치수(전부 AR_SCALE 파생) ───────────────────────── */

const S = AR_SCALE;

/* 공통분모: 모든 표지는 "같은 높이"를 갖는다.
 *
 * 이전에는 막대 1.6m / 마름모 2.56m / 원 2.0m / 핀 1.25m 로 제각각이라
 * 시각적 무게가 어긋나고 핀은 읽히지도 않았다. 높이를 하나로 묶으면
 *   - 같은 거리에서 같은 크기로 읽히고
 *   - 같은 눈높이 띠에 정렬되며
 *   - 텍스트도 같은 배율을 쓸 수 있다.
 * 폭만 형태별로 배율에서 파생한다. */
export const AR_SIGN_HEIGHT_M = S.xl;    // 2.56 — 모든 표지의 공통 높이

/** 막대 표지의 종횡비. RATIO(1.6)는 글자가 넘쳐 2.0 으로 넓혔다. */
export const AR_BAR_ASPECT = 2.0;

/** 배지형(마름모·원·핀) 공통 크기. 단속/방지턱/구간/제한속도/목적지가 전부 같다. */
export const AR_BADGE_M = S.xl;          // 2.56

const H = AR_SIGN_HEIGHT_M;
const B = AR_BADGE_M;

export const AR_FORM = Object.freeze({
  /** 노면에서 표지 하단까지. 실제 갠트리 높이대. */
  mountHeightM: S.xl,                    // 2.56

  [AR_SHAPE.BAR]: Object.freeze({
    widthM: H * AR_BAR_ASPECT,           // 5.120
    heightM: H,                          // 2.560
    radiusM: S.xs,                       // 0.391
    depthM: S.xxs,                       // 0.244
  }),
  [AR_SHAPE.DIAMOND]: Object.freeze({
    widthM: B, heightM: B,               // 2.560
    radiusM: S.xxs,
    depthM: S.xxs,
    borderM: S.xxs * 0.9,
  }),
  [AR_SHAPE.CIRCLE]: Object.freeze({
    diameterM: B,                        // 2.560
    depthM: S.xxs,
    ringM: S.xxs * 1.15,
  }),
  [AR_SHAPE.BAND]: Object.freeze({
    lengthM: S.xxl * 2,                  // 8.19 (노면이라 높이 개념 없음)
    liftM: 0.03,
    insetRatio: 0.72,
  }),
  [AR_SHAPE.PIN]: Object.freeze({
    heightM: S.xxl + B / 2,              // 지주 포함 전체
    headDiameterM: B,                    // 2.560 — 배지형과 동일
    headRadiusM: B / 2,
    stemRadiusM: S.xxs * 0.45,
  }),

  /** 배지형 3종이 텍스처 안에서 차지하는 비율. 같아야 광학적 크기가 맞는다. */
  badgeFillRatio: 0.92,

  /** 방향 셰브런: 표지판에 "통합"한다. 별도 오브젝트로 띄우지 않는다. */
  chevron: Object.freeze({
    countDefault: 3,
    widthRatio: 0.115,                   // 보드 폭 대비
    gapRatio: 0.028,
    insetRatio: 0.055,                   // 보드 안쪽 여백
  }),
});

/* ── 타이포 ──────────────────────────────────────────────── */

/* 텍스처 해상도도 배율에서 파생한다(2k 이하 — 구글 AR 가이드라인 권고).
 * BAR 종횡비 2.56 → 1024 x 400. */
export const AR_TEXTURE = Object.freeze({
  barWidthPx: 1280,      // 1280/640 = 2.0 = AR_BAR_ASPECT
  barHeightPx: 640,
  squarePx: 640,         // 배지형도 같은 높이 해상도
  maxPx: 2048,           // 구글 AR 가이드라인 권고 상한
});

/* 2단 위계. 이전 디자인은 "우회전"과 "80m"가 같은 크기라 무엇을 먼저 볼지
 * 알 수 없었다. primary 를 확실히 키워 한눈에 읽히게 한다. */
/* 글자 크기도 텍스처 높이에서 파생한다. 값을 손으로 정하지 않는다.
 * 보드 높이가 1.6 → 2.56m 로 커졌으므로 글자도 같은 배율로 커진다. */
const PRIMARY_PX = Math.round(AR_TEXTURE.barHeightPx * 0.34);        // 218 (0.42 → 0.34 로 축소)
const SECONDARY_PX = Math.round(PRIMARY_PX / Math.pow(AR_RATIO, 2)); // 85

export const AR_TYPE = Object.freeze({
  primaryPx: PRIMARY_PX,                 // 핵심 수치(거리/속도)
  secondaryPx: SECONDARY_PX,             // 설명(도로명/부가)
  badgePx: Math.round(PRIMARY_PX / AR_RATIO),   // 원형 표지 안 숫자
  primaryWeight: 900,
  secondaryWeight: 700,
  tracking: -0.02,
  family: '"Pretendard", "Noto Sans KR", system-ui, -apple-system, sans-serif',
  /** primary:secondary = RATIO^2 여야 한다. 배율 일관성 확인용. */
  get ratio() { return +(this.primaryPx / this.secondaryPx).toFixed(2); },
});

/* ── 가시성(영상 위 대비) ────────────────────────────────── */

/* 구글 AR 가이드라인: "Employ contrast and visual hierarchy to separate UI
 * from live feed content." WebRTC 압축 영상은 대비가 더 나쁘므로 강하게 잡는다. */
/* 투명도. AR 표지는 도로를 가리면 안 되므로 전 요소가 반투명이다.
 * 다만 글자는 가독성이 우선이라 거의 불투명하게 둔다. */
export const AR_OPACITY = Object.freeze({
  surface: 0.86,      // 표지 면
  surfaceTop: 0.90,   // 상단 그라디언트
  edge: 0.92,         // 테두리
  ink: 0.98,          // 주 텍스트
  inkMuted: 0.80,     // 보조 텍스트
  chevron: 0.88,      // 셰브런
  pole: 0.72,         // 지주
  shadow: 0.38,       // 접지 그림자
  /** 렌더러 material.opacity 로 곱해질 전체 투명도. */
  group: 0.94,
  /** Phase 4 유지(hold) 중 곱해지는 계수. 새 model 없이 odometry 적분으로만
   *  버티는 상태라, 확신이 낮다는 것을 사용자가 볼 수 있어야 한다. */
  held: 0.72,
});

/** "#RRGGBB" + alpha → "rgba(...)". 토큰 색을 투명도와 합성할 때 쓴다. */
export function withAlpha(hex, alpha) {
  const s = String(hex || "").trim();
  if (s.startsWith("rgba") || s.startsWith("rgb")) return s;
  const m = /^#([0-9a-f]{6})$/i.exec(s);
  if (!m) return s;
  const n = parseInt(m[1], 16);
  const a = Math.max(0, Math.min(1, Number(alpha)));
  return `rgba(${(n >> 16) & 255}, ${(n >> 8) & 255}, ${n & 255}, ${a})`;
}

export const AR_LEGIBILITY = Object.freeze({
  outlineM: 0.055,          // 표지 외곽 테두리 두께
  outlineOpacity: 0.95,
  textStrokePx: 10,         // 글자 외곽선(텍스처 픽셀)
  dropShadowPx: 18,
  shadowPlaneOpacity: 0.42, // 접지 그림자 — 떠 보이지 않게(구글: shadow plane 필수)
  shadowPlaneScale: 1.35,
  emissiveIntensity: 0.28,
});

/* ── 거리 단계별 강조 ────────────────────────────────────── */

export const AR_EMPHASIS = Object.freeze({
  preview: Object.freeze({ scale: 0.78, opacity: 0.62, chevrons: 0, showSecondary: false }),
  approach: Object.freeze({ scale: 0.9, opacity: 0.85, chevrons: 2, showSecondary: true }),
  precise: Object.freeze({ scale: 1.0, opacity: 1.0, chevrons: 3, showSecondary: true }),
  commit: Object.freeze({ scale: 1.12, opacity: 1.0, chevrons: 3, showSecondary: false }),
});

/** tone 팔레트 조회. 모르는 tone 은 안내색으로 떨어뜨린다. */
export function paletteFor(tone) {
  return AR_PALETTE[tone] || AR_PALETTE[AR_TONE.GUIDE];
}

/** tone → 실루엣. */
export function shapeFor(tone) {
  return AR_TONE_SHAPE[tone] || AR_SHAPE.BAR;
}

/** 배율 일관성 자체 검사. 빌드/테스트에서 호출해 토큰이 어긋나지 않았는지 본다. */
export function assertScaleConsistency() {
  const issues = [];
  const bar = AR_FORM[AR_SHAPE.BAR];
  const aspect = +(bar.widthM / bar.heightM).toFixed(3);
  if (Math.abs(aspect - AR_BAR_ASPECT) > 1e-6) {
    issues.push(`BAR 종횡비 ${aspect} != AR_BAR_ASPECT ${AR_BAR_ASPECT}`);
  }

  const texAspect = +(AR_TEXTURE.barWidthPx / AR_TEXTURE.barHeightPx).toFixed(2);
  if (texAspect !== +aspect.toFixed(2)) issues.push(`텍스처 종횡비 ${texAspect} != 보드 ${aspect}`);

  // 공통분모 1: 모든 표지가 같은 높이여야 한다.
  const heights = {
    bar: bar.heightM,
    diamond: AR_FORM[AR_SHAPE.DIAMOND].heightM,
    circle: AR_FORM[AR_SHAPE.CIRCLE].diameterM,
    pinHead: AR_FORM[AR_SHAPE.PIN].headDiameterM,
  };
  for (const [k, v] of Object.entries(heights)) {
    if (Math.abs(v - AR_SIGN_HEIGHT_M) > 1e-6) {
      issues.push(`${k} 높이 ${v} != 공통 ${AR_SIGN_HEIGHT_M}`);
    }
  }

  // 공통분모 2: 배지형(단속/방지턱/구간/제한속도/목적지)은 폭까지 완전히 같아야 한다.
  const badges = {
    diamond: AR_FORM[AR_SHAPE.DIAMOND].widthM,
    circle: AR_FORM[AR_SHAPE.CIRCLE].diameterM,
    pinHead: AR_FORM[AR_SHAPE.PIN].headDiameterM,
  };
  for (const [k, v] of Object.entries(badges)) {
    if (Math.abs(v - AR_BADGE_M) > 1e-6) issues.push(`${k} 폭 ${v} != AR_BADGE_M ${AR_BADGE_M}`);
  }

  // 투명도 토큰 범위
  for (const [k, v] of Object.entries(AR_OPACITY)) {
    if (!(v > 0 && v <= 1)) issues.push(`AR_OPACITY.${k} 범위 밖: ${v}`);
  }

  const typeRatio = +(AR_TYPE.primaryPx / AR_TYPE.secondaryPx).toFixed(2);
  const wantType = +Math.pow(AR_RATIO, 2).toFixed(2);
  if (Math.abs(typeRatio - wantType) > 0.06) {
    issues.push(`타이포 비 ${typeRatio} != RATIO^2 ${wantType}`);
  }

  for (const [k, v] of Object.entries(AR_SCALE)) {
    if (!Number.isFinite(v) || v <= 0) issues.push(`AR_SCALE.${k} 비정상: ${v}`);
  }
  if (AR_TEXTURE.barWidthPx > AR_TEXTURE.maxPx) issues.push("텍스처가 2k 권고를 넘음");

  return Object.freeze({
    ok: issues.length === 0, issues: Object.freeze(issues),
    aspect, texAspect, typeRatio, heights: Object.freeze(heights),
    signHeightM: AR_SIGN_HEIGHT_M,
  });
}
