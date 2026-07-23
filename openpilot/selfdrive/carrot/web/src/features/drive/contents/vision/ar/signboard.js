/* AR 표지 컴포넌트 빌더.
 *
 * 토큰(design_tokens.js) → 표지 descriptor + 텍스처 페인터.
 * 렌더러(three/webgl2)에 종속되지 않는다. 페인터는 CanvasRenderingContext2D 만
 * 요구하므로 브라우저·오프스크린 어디서든 쓸 수 있다.
 *
 * 이전 디자인의 실패를 여기서 구조적으로 막는다.
 *   - 7종이 전부 같은 초록 알약  → tone 이 색과 실루엣을 동시에 결정
 *   - 셰브런이 따로 떠 있음      → 보드 텍스처 안에 통합해서 그린다
 *   - 거리/설명이 같은 크기      → primary/secondary 2단 고정
 */

import { AR_MARKER_FORM, AR_MARKER_KIND, markerForm } from "./tokens.js";
import {
  AR_EMPHASIS,
  AR_FORM,
  AR_LEGIBILITY,
  AR_OPACITY,
  AR_SHAPE,
  AR_TEXTURE,
  AR_TONE,
  AR_TYPE,
  paletteFor,
  shapeFor,
  withAlpha,
} from "./design_tokens.js";

/* ── descriptor ─────────────────────────────────────────── */

/**
 * 표지 하나의 완전한 기술. 지오메트리/텍스처를 만들기 전 단계.
 * @param {object} input
 *   tone      : AR_TONE
 *   primary   : 큰 글자 (거리/속도 등 핵심 수치)
 *   secondary : 작은 글자 (도로명/부가 설명)
 *   turnSign  : -1 좌 / +1 우 / 0 없음 → 셰브런 방향
 *   phase     : preview | approach | precise | commit
 *   distanceM : 앵커까지 거리(배치용)
 */
export function describeSignboard(input = {}) {
  const tone = Object.values(AR_TONE).includes(input.tone) ? input.tone : AR_TONE.GUIDE;
  const shape = input.shape || shapeFor(tone);
  const phase = AR_EMPHASIS[input.phase] ? input.phase : "precise";
  const emphasis = AR_EMPHASIS[phase];
  const turnSign = Math.sign(Number(input.turnSign) || 0);

  const form = AR_FORM[shape] || AR_FORM[AR_SHAPE.BAR];
  // 형태마다 치수 이름이 다르다(원=diameter, 밴드=length, 핀=headRadius).
  // 여기서 한 번에 width/height 로 정규화해 두면 렌더러가 분기하지 않아도 된다.
  const laneWidthM = Number(input.laneWidthM) || 3.5;
  let baseW, baseH;
  if (shape === AR_SHAPE.BAND) {
    baseW = laneWidthM * form.insetRatio;
    baseH = form.lengthM;
  } else if (shape === AR_SHAPE.CIRCLE) {
    baseW = baseH = form.diameterM;
  } else if (shape === AR_SHAPE.PIN) {
    baseW = form.headDiameterM;   // 다른 표지와 같은 높이/폭
    baseH = form.heightM;         // 지주 포함 전체 높이
  } else {
    baseW = form.widthM;
    baseH = form.heightM;
  }
  const widthM = baseW * emphasis.scale;
  const heightM = baseH * emphasis.scale;

  return Object.freeze({
    tone, shape, phase,
    palette: paletteFor(tone),
    primary: String(input.primary ?? ""),
    secondary: emphasis.showSecondary ? String(input.secondary ?? "") : "",
    turnSign,
    chevronCount: turnSign === 0 ? 0 : emphasis.chevrons,
    distanceM: Number(input.distanceM) || 0,
    widthM, heightM,
    mountHeightM: AR_FORM.mountHeightM,
    depthM: form.depthM ?? 0.24,
    radiusM: form.radiusM ?? 0,
    opacity: emphasis.opacity,
    scale: emphasis.scale,
  });
}
/* ── 캔버스 페인터 ──────────────────────────────────────── */

function roundRect(ctx, x, y, w, h, r) {
  const rr = Math.min(r, w / 2, h / 2);
  ctx.beginPath();
  ctx.moveTo(x + rr, y);
  ctx.arcTo(x + w, y, x + w, y + h, rr);
  ctx.arcTo(x + w, y + h, x, y + h, rr);
  ctx.arcTo(x, y + h, x, y, rr);
  ctx.arcTo(x, y, x + w, y, rr);
  ctx.closePath();
}

function fillText(ctx, text, x, y, { px, weight, color, stroke, align = "center", maxWidth = 0 }) {
  if (!text) return;
  let size = px;
  // 상자를 넘치면 줄인다. 배율을 키워도 글자가 잘리지 않게 하는 안전장치.
  if (maxWidth > 0) {
    ctx.font = `${weight} ${size}px ${AR_TYPE.family}`;
    const w = ctx.measureText(text).width;
    if (w > maxWidth) size = Math.max(12, Math.floor(size * (maxWidth / w)));
  }
  const px2 = size;
  ctx.font = `${weight} ${px2}px ${AR_TYPE.family}`;
  ctx.textAlign = align;
  ctx.textBaseline = "middle";
  ctx.lineJoin = "round";
  if (stroke) {
    ctx.lineWidth = AR_LEGIBILITY.textStrokePx * (px2 / px);
    ctx.strokeStyle = stroke;
    ctx.strokeText(text, x, y);
  }
  ctx.fillStyle = color;
  ctx.fillText(text, x, y);
}

/** 보드 안에 통합되는 방향 셰브런. 별도 오브젝트로 띄우지 않는 것이 핵심. */
function paintChevrons(ctx, d, box) {
  if (!d.chevronCount) return;
  const c = AR_FORM.chevron;
  const w = box.w * c.widthRatio;
  const gap = box.w * c.gapRatio;
  const inset = box.w * c.insetRatio;
  const h = box.h * 0.46;
  const cy = box.y + box.h / 2;
  const dir = d.turnSign;
  // 회전 방향 쪽 가장자리에 붙인다.
  const startX = dir > 0
    ? box.x + box.w - inset - (w * d.chevronCount + gap * (d.chevronCount - 1))
    : box.x + inset;

  ctx.save();
  for (let i = 0; i < d.chevronCount; i += 1) {
    const x = startX + i * (w + gap);
    // 진행 방향으로 갈수록 진하게 → 흐름이 읽힌다
    ctx.globalAlpha = (0.42 + (i / Math.max(1, d.chevronCount - 1)) * 0.58) * AR_OPACITY.chevron;
    ctx.fillStyle = d.palette.accent;
    ctx.beginPath();
    if (dir > 0) {
      ctx.moveTo(x, cy - h / 2);
      ctx.lineTo(x + w * 0.62, cy);
      ctx.lineTo(x, cy + h / 2);
      ctx.lineTo(x + w * 0.28, cy);
    } else {
      ctx.moveTo(x + w * 0.62, cy - h / 2);
      ctx.lineTo(x, cy);
      ctx.lineTo(x + w * 0.62, cy + h / 2);
      ctx.lineTo(x + w * 0.34, cy);
    }
    ctx.closePath();
    ctx.fill();
  }
  ctx.restore();
}

/** 가로 막대(안내). */
function paintBar(ctx, d, W, H) {
  const p = d.palette;
  const pad = H * 0.06;
  const box = { x: pad, y: pad, w: W - pad * 2, h: H - pad * 2 };
  const r = (d.radiusM / d.heightM) * box.h;

  const grad = ctx.createLinearGradient(0, box.y, 0, box.y + box.h);
  grad.addColorStop(0, withAlpha(p.surfaceTop, AR_OPACITY.surfaceTop));
  grad.addColorStop(1, withAlpha(p.surface, AR_OPACITY.surface));
  roundRect(ctx, box.x, box.y, box.w, box.h, r);
  ctx.fillStyle = grad;
  ctx.fill();

  ctx.lineWidth = H * 0.030;
  ctx.strokeStyle = withAlpha(p.edge, AR_OPACITY.edge);
  ctx.globalAlpha = AR_LEGIBILITY.outlineOpacity;
  ctx.stroke();
  ctx.globalAlpha = 1;

  paintChevrons(ctx, d, box);

  // 셰브런이 차지한 쪽을 피해 텍스트 영역을 잡는다.
  const chevW = d.chevronCount
    ? box.w * (AR_FORM.chevron.widthRatio * d.chevronCount
      + AR_FORM.chevron.gapRatio * (d.chevronCount - 1) + AR_FORM.chevron.insetRatio * 1.5)
    : 0;
  const textLeft = d.turnSign < 0 ? box.x + chevW : box.x;
  const textW = box.w - chevW;
  const cx = textLeft + textW / 2;
  const hasSecondary = Boolean(d.secondary);
  const cy = box.y + box.h / 2;

  const inner = textW * 0.90;
  fillText(ctx, d.primary, cx, hasSecondary ? cy - H * 0.105 : cy, {
    px: AR_TYPE.primaryPx, weight: AR_TYPE.primaryWeight,
    color: withAlpha(p.ink, AR_OPACITY.ink), stroke: "rgba(0,0,0,0.45)", maxWidth: inner,
  });
  if (hasSecondary) {
    fillText(ctx, d.secondary, cx, cy + H * 0.205, {
      px: AR_TYPE.secondaryPx, weight: AR_TYPE.secondaryWeight,
      color: p.inkMuted, stroke: "rgba(0,0,0,0.35)", maxWidth: inner,
    });
  }
}

/** 마름모(주의) — 한국 주의표지 형태. */
function paintDiamond(ctx, d, W, H) {
  const p = d.palette;
  const cx = W / 2, cy = H / 2;
  const f = AR_FORM.badgeFillRatio / 2;
  const rx = W * f, ry = H * f;
  const rr = Math.min(rx, ry) * 0.13;

  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(Math.PI / 4);
  const s = Math.min(rx, ry) * Math.SQRT1_2 * 2;
  roundRect(ctx, -s / 2, -s / 2, s, s, rr);
  const grad = ctx.createLinearGradient(0, -s / 2, 0, s / 2);
  grad.addColorStop(0, withAlpha(p.surfaceTop, AR_OPACITY.surfaceTop));
  grad.addColorStop(1, withAlpha(p.surface, AR_OPACITY.surface));
  ctx.fillStyle = grad;
  ctx.fill();
  ctx.lineWidth = s * 0.055;
  ctx.strokeStyle = withAlpha(p.accent, AR_OPACITY.edge);
  ctx.stroke();
  ctx.restore();

  const hasSecondary = Boolean(d.secondary);
  const dInner = W * 0.62;
  fillText(ctx, d.primary, cx, hasSecondary ? cy - H * 0.075 : cy, {
    px: AR_TYPE.primaryPx * 0.78, weight: AR_TYPE.primaryWeight,
    color: withAlpha(p.ink, AR_OPACITY.ink), stroke: "rgba(255,255,255,0.30)", maxWidth: dInner,
  });
  if (hasSecondary) {
    fillText(ctx, d.secondary, cx, cy + H * 0.155, {
      px: AR_TYPE.secondaryPx * 0.86, weight: AR_TYPE.secondaryWeight,
      color: p.inkMuted, maxWidth: dInner,
    });
  }
}

/** 원(규제) — 한국 규제표지: 흰 바탕 + 빨간 링. */
function paintCircle(ctx, d, W, H) {
  const p = d.palette;
  const cx = W / 2, cy = H / 2;
  const R = Math.min(W, H) * (AR_FORM.badgeFillRatio / 2);
  const ring = R * 0.17;

  ctx.beginPath();
  ctx.arc(cx, cy, R, 0, Math.PI * 2);
  ctx.fillStyle = withAlpha(p.surface, AR_OPACITY.surface);
  ctx.fill();

  ctx.beginPath();
  ctx.arc(cx, cy, R - ring / 2, 0, Math.PI * 2);
  ctx.lineWidth = ring;
  ctx.strokeStyle = withAlpha(p.accent, AR_OPACITY.edge);
  ctx.stroke();

  const hasSecondary = Boolean(d.secondary);
  const cInner = (R - ring) * 1.55;
  fillText(ctx, d.primary, cx, hasSecondary ? cy - H * 0.055 : cy, {
    px: AR_TYPE.badgePx * 1.55, weight: AR_TYPE.primaryWeight,
    color: withAlpha(p.ink, AR_OPACITY.ink), maxWidth: cInner,
  });
  if (hasSecondary) {
    fillText(ctx, d.secondary, cx, cy + H * 0.205, {
      px: AR_TYPE.secondaryPx * 0.78, weight: AR_TYPE.secondaryWeight,
      color: p.inkMuted, maxWidth: cInner,
    });
  }
}

/** 핀(목적지) — 글자를 핀 머리 안에 크게. 이전엔 너무 작아 안 읽혔다. */
function paintPin(ctx, d, W, H) {
  const p = d.palette;
  const cx = W / 2, cy = H * 0.42;
  const R = Math.min(W, H) * (AR_FORM.badgeFillRatio / 2) * 0.86;
  ctx.beginPath();
  ctx.arc(cx, cy, R, 0, Math.PI * 2);
  const grad = ctx.createRadialGradient(cx, cy - R * 0.4, R * 0.1, cx, cy, R);
  grad.addColorStop(0, withAlpha(p.surfaceTop, AR_OPACITY.surfaceTop));
  grad.addColorStop(1, withAlpha(p.surface, AR_OPACITY.surface));
  ctx.fillStyle = grad;
  ctx.fill();
  ctx.lineWidth = R * 0.12;
  ctx.strokeStyle = withAlpha(p.edge, AR_OPACITY.edge);
  ctx.stroke();

  const pInner = R * 1.45;
  fillText(ctx, d.primary, cx, d.secondary ? cy - R * 0.18 : cy, {
    px: AR_TYPE.primaryPx * 0.62, weight: AR_TYPE.primaryWeight,
    color: withAlpha(p.ink, AR_OPACITY.ink), stroke: "rgba(0,0,0,0.35)", maxWidth: pInner,
  });
  if (d.secondary) {
    fillText(ctx, d.secondary, cx, cy + R * 0.44, {
      px: AR_TYPE.secondaryPx * 0.9, weight: AR_TYPE.secondaryWeight,
      color: p.inkMuted, maxWidth: pInner,
    });
  }
}

const PAINTERS = Object.freeze({
  [AR_SHAPE.BAR]: paintBar,
  [AR_SHAPE.DIAMOND]: paintDiamond,
  [AR_SHAPE.CIRCLE]: paintCircle,
  [AR_SHAPE.PIN]: paintPin,
});

/** descriptor 에 맞는 텍스처 크기(px). */
export function textureSizeFor(descriptor) {
  if (descriptor.shape === AR_SHAPE.BAR) {
    return { width: AR_TEXTURE.barWidthPx, height: AR_TEXTURE.barHeightPx };
  }
  return { width: AR_TEXTURE.squarePx, height: AR_TEXTURE.squarePx };
}

/**
 * descriptor 를 2D 캔버스에 그린다. 반환값은 그린 크기.
 * 폰트·색·레이아웃이 전부 평범한 웹 코드라 디자인 반복이 빠르다.
 */
export function paintSignboard(ctx, descriptor) {
  const { width, height } = textureSizeFor(descriptor);
  ctx.clearRect(0, 0, width, height);
  ctx.save();
  ctx.shadowColor = "rgba(0,0,0,0.55)";
  ctx.shadowBlur = AR_LEGIBILITY.dropShadowPx;
  ctx.shadowOffsetY = AR_LEGIBILITY.dropShadowPx * 0.35;
  const painter = PAINTERS[descriptor.shape] || paintBar;
  painter(ctx, descriptor, width, height);
  ctx.restore();
  return { width, height };
}

/* ── TMap descriptor → 표지 입력 ─────────────────────────── */

/** 거리 표기. 1km 미만은 m, 이상은 km. */
export function formatDistance(meters) {
  const m = Number(meters);
  if (!Number.isFinite(m) || m < 0) return "";
  if (m < 1000) return `${Math.round(m / 10) * 10}m`;
  return `${(m / 1000).toFixed(m >= 10000 ? 0 : 1)}km`;
}

const TURN_LABEL = Object.freeze({
  left: "좌회전", right: "우회전",
  slight_left: "좌측", slight_right: "우측",
  sharp_left: "급좌회전", sharp_right: "급우회전",
  uturn: "유턴", straight: "직진",
});

/**
 * tmap_catalog 의 marker descriptor → 표지 descriptor.
 * 여기서 "무엇을 크게 보여줄지"를 정한다. 거리를 primary 로 올리는 것이 원칙.
 */
export function signboardFromMarker(marker = {}) {
  const dist = formatDistance(marker.distanceM);
  const turn = marker.turn || {};
  const label = TURN_LABEL[turn.direction] || "";
  const common = { distanceM: marker.distanceM, phase: marker.phase };

  /* 승인 preview에서 LANE/BAND도 다른 종류와 같은 descriptor→painter→Three
   * component 경로를 탄다. 제품 renderer가 임의의 단색 path 면으로 바꾸지 않는다. */
  if (marker.kind === AR_MARKER_KIND.LANE_BAND) {
    const descriptor = describeSignboard({
      ...common,
      tone: AR_TONE.LANE,
      primary: dist,
      secondary: "권장 차선",
      laneWidthM: Number(marker.laneWidthM) || 3.5,
    });
    return Object.freeze({
      ...descriptor,
      kind: marker.kind,
      laneOffsetM: Number(marker.laneOffsetM) || 0,
      surface: true,
    });
  }

  /* 향후 다른 노면 종류가 생겨도 승인 component가 없으면 임의 스타일을 만들지 않는다. */
  if (markerForm(marker.kind) === AR_MARKER_FORM.SURFACE) return null;

  const withKind = (d) => (d ? Object.freeze({ ...d, kind: marker.kind }) : null);

  switch (marker.kind) {
    case "turn_gate":
      return withKind(describeSignboard({
        ...common, tone: AR_TONE.GUIDE, primary: dist || label,
        secondary: [label, marker.label].filter(Boolean).join(" · "),
        turnSign: turn.sign,
      }));
    case "commit_arrow":
      // 진입 직전의 방향 지시. 거리 텍스트 없이 방향만 크게.
      return withKind(describeSignboard({
        ...common, tone: AR_TONE.GUIDE, primary: label || "직진",
        secondary: "", turnSign: turn.sign,
      }));
    case "destination_pin":
      return withKind(describeSignboard({
        ...common, tone: AR_TONE.DESTINATION, primary: "도착", secondary: dist,
      }));
    case "caution_sign":
      return withKind(describeSignboard({
        ...common, tone: AR_TONE.CAUTION, primary: dist,
        secondary: marker.sdiFamily === "bump" ? "과속방지턱"
          : marker.sdiFamily === "police" ? "경찰"
            : marker.sdiFamily === "camera" ? "단속카메라" : "도로 주의",
      }));
    case "section_gate":
      return withKind(describeSignboard({
        ...common, tone: AR_TONE.RESTRICT, primary: String(marker.limitKph || ""),
        secondary: marker.suspended
          ? "구간단속 일시중지"
          : `구간 평균 ${Math.round(marker.averageKph || marker.overallAverageKph || 0)}`,
      }));
    case "speed_sign":
      return withKind(describeSignboard({
        ...common, tone: AR_TONE.RESTRICT, primary: String(marker.speedLimitKph || ""),
        secondary: "",
      }));
    case "signal_head": {
      // 켜져 있는 램프를 우선 보여 준다. 없으면 신호 자체를 알리지 않는다.
      const lamps = Array.isArray(marker.lamps) ? marker.lamps : [];
      const on = lamps.find((l) => l.on);
      if (!on) return null;
      const NAME = { red: "정지", green: "직진", left: "좌회전", right: "우회전", uturn: "유턴" };
      return withKind(describeSignboard({
        ...common, tone: AR_TONE.CAUTION, primary: NAME[on.id] || "신호",
        secondary: (on.remainSec > 0 || marker.counterRemainSec > 0)
          ? `${Math.round(on.remainSec || marker.counterRemainSec)}초`
          : dist,
      }));
    }
    case "crossroad_card":
      return withKind(describeSignboard({
        ...common, tone: AR_TONE.GUIDE, primary: dist, secondary: "교차로",
      }));
    default:
      return null;
  }
}
