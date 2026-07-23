/* AR 표지 렌더러 — 독립 2D 캔버스 오버레이.
 *
 * 왜 별도 canvas 인가:
 *   AR 은 HUD 와 다른 기능이다. vision_webgl2.js 를 건드리지 않으면 기존
 *   오버레이에 회귀가 생길 수 없고, AR 만 켜고 끌 수 있다.
 *
 * 왜 그래도 타이밍이 안 어긋나는가:
 *   같은 문서·같은 UI 스레드에서 **같은 프레임**에 그리면 브라우저가 두 캔버스를
 *   함께 합성한다. 어긋나는 것은 워커/독립 타이머로 그릴 때다. 그래서 이 렌더러는
 *   자체 rAF 루프를 돌리지 않고 runtime 이 presented-frame 신호로 호출해 준다.
 *
 * 왜 2D 인가:
 *   Phase 3 의 목표는 "정합이 되는가" 하나다. 표지는 카메라를 향한 빌보드라
 *   원근 왜곡(사다리꼴)이 필요 없고, 투영된 중심·크기만 맞으면 정확하다.
 *   3D/Three.js 는 정합이 검증된 뒤 Phase 6 에서 올린다.
 */

import { AR_LEGIBILITY, AR_OPACITY, paletteFor } from "./design_tokens.js";
import { AR_GEOMETRY, AR_MARKER_FORM, AR_MARKER_KIND, markerForm } from "./tokens.js";

/* 리본 형태 토큰. 멀리 갈수록 옅어지는 정도까지 여기서 정한다. */
const AR_RIBBON = Object.freeze({
  maxDistanceM: 120,   // 이보다 먼 경로점은 신뢰도가 낮아 그리지 않는다
  nearAlpha: 0.42,     // 가장 가까운 구간의 불투명도
  fadePower: 1.6,      // 클수록 빨리 옅어진다
});
import { paintSignboard, textureSizeFor } from "./signboard.js";
import { projectRouteFluPoint, pointOnPath } from "./projection.js";
import { planMarker, resolveArCanvasSize } from "./responsive.js";

function finite(v, fallback = null) {
  const n = Number(v);
  return Number.isFinite(n) ? n : fallback;
}

/** 영상 픽셀 → 스테이지 픽셀. home_drive 의 transform 과 같은 식이어야 한다. */
function toStage(point, st) {
  if (!point || !st) return null;
  return { x: point.x * st.scale + st.tx, y: point.y * st.scale + st.ty, depth: point.depth };
}

/** 표지 텍스처를 캐시한다. 같은 내용이면 다시 그리지 않는다. */
function createTextureCache(documentRoot, limit = 12) {
  const cache = new Map();
  function key(d) {
    return [d.tone, d.shape, d.primary, d.secondary, d.turnSign, d.chevronCount, d.phase].join("|");
  }
  return {
    get(descriptor) {
      const k = key(descriptor);
      const hit = cache.get(k);
      if (hit) return hit;
      const size = textureSizeFor(descriptor);
      const canvas = documentRoot.createElement("canvas");
      canvas.width = size.width;
      canvas.height = size.height;
      const ctx = canvas.getContext("2d");
      if (!ctx) return null;
      paintSignboard(ctx, descriptor);
      if (cache.size >= limit) cache.delete(cache.keys().next().value);
      cache.set(k, canvas);
      return canvas;
    },
    clear() { cache.clear(); },
    get size() { return cache.size; },
  };
}

export function createArRenderer(options = {}) {
  const root = options.root;
  const documentRoot = options.document || root?.ownerDocument || globalThis.document;
  const target = options.target || documentRoot?.defaultView || globalThis;
  if (!root || !documentRoot?.createElement) return null;

  /* surface 가 주어지면 그것을 그대로 쓴다(워커의 OffscreenCanvas).
   * 아니면 DOM 에 캔버스를 만들어 붙인다(메인 스레드 폴백). */
  const offscreen = Boolean(options.surface);
  let canvas;
  if (offscreen) {
    canvas = options.surface;
  } else {
    canvas = documentRoot.createElement("canvas");
    canvas.className = "carrot-ar__canvas";
    canvas.style.cssText = "position:absolute;inset:0;width:100%;height:100%;pointer-events:none;z-index:6";
    root.appendChild(canvas);
  }
  const ctx = canvas.getContext("2d");
  const textures = createTextureCache(documentRoot);

  let destroyed = false;
  let lastSize = null;
  let stats = { drawn: 0, skipped: 0, lastReason: "", skips: [] };

  function syncSize(stage) {
    /* 워커에서는 레이아웃을 잴 수 없다(getBoundingClientRect 없음). 대신 메인이
     * 이미 정한 캔버스 픽셀 크기와 stage 스냅샷을 그대로 신뢰한다. 크기 변경은
     * "resize" 메시지로 메인이 알려 준다. */
    if (offscreen) {
      const dpr = finite(stage?.devicePixelRatio, 1) || 1;
      const size = {
        pixelWidth: canvas.width, pixelHeight: canvas.height,
        width: canvas.width / dpr, height: canvas.height / dpr, dpr,
      };
      lastSize = size;
      return size;
    }
    const rect = canvas.getBoundingClientRect?.() || {};
    const size = resolveArCanvasSize(
      { width: finite(rect.width, stage?.stageWidth) || 1, height: finite(rect.height, stage?.stageHeight) || 1 },
      target.devicePixelRatio || 1,
    );
    if (!lastSize || lastSize.pixelWidth !== size.pixelWidth || lastSize.pixelHeight !== size.pixelHeight) {
      canvas.width = size.pixelWidth;
      canvas.height = size.pixelHeight;
      lastSize = size;
    }
    return size;
  }

  /**
   * 경로 리본 — 노면에 깔리는 주행 경로.
   *
   * 빌보드가 아니다. modelV2 경로점을 좌우로 폭만큼 벌려 투영한 뒤 그 다각형을
   * 채운다. 원근은 투영이 알아서 만들어 준다(먼 쪽이 저절로 좁아진다).
   *
   * 멀수록 옅어지게 해서 "어디까지 확실한가"를 그대로 보이게 한다. 리본이
   * 끝까지 진하면 모델이 실제로 보고 있는 거리보다 더 아는 것처럼 보인다.
   */
  function drawRibbon(modelPosition, stage, options = {}) {
    const T = stage?.calibTransform;
    const xs = modelPosition?.x;
    const ys = modelPosition?.y;
    const zs = modelPosition?.z;
    if (!T || !Array.isArray(xs) || !Array.isArray(ys) || xs.length < 2) return false;

    const widthM = finite(options.widthM, 3.5);
    const offsetM = finite(options.offsetM, 0);
    const startM = Math.max(1, finite(options.startM, 1));
    const endM = Math.max(startM, finite(options.endM, AR_RIBBON.maxDistanceM));
    const nearAlpha = finite(options.nearAlpha, AR_RIBBON.nearAlpha);
    const fadePower = finite(options.fadePower, AR_RIBBON.fadePower);
    const color = options.color;
    const half = Math.max(0.4, widthM / 2);
    const left = [];
    const right = [];
    const samples = [];
    const startPoint = pointOnPath(modelPosition, startM);
    if (startPoint) samples.push(startPoint);
    const limit = Math.min(xs.length, ys.length);
    for (let i = 0; i < limit; i += 1) {
      const x = Number(xs[i]);
      const y = Number(ys[i]);
      const z = Number(Array.isArray(zs) ? zs[i] : 0) || 0;
      if (!Number.isFinite(x) || !Number.isFinite(y) || x <= startM) continue;
      if (x >= endM) break;
      samples.push({ x, y, z });
    }
    const endPoint = pointOnPath(modelPosition, endM);
    if (endPoint && (!samples.length || endPoint.x > samples.at(-1).x + 1e-3)) samples.push(endPoint);

    for (const { x, y, z } of samples) {
      const centerY = y + offsetM;
      const l = toStage(projectRouteFluPoint(T, x, centerY + half, z), stage);
      const r = toStage(projectRouteFluPoint(T, x, centerY - half, z), stage);
      if (!l || !r) continue;
      left.push(l);
      right.push(r);
    }
    if (left.length < 2) return false;

    // 가까운 쪽부터 구간별로 채운다. 구간마다 알파를 달리하려면 통짜 path 로는 안 된다.
    ctx.save();
    for (let i = 0; i < left.length - 1; i += 1) {
      const t = i / (left.length - 1);
      ctx.globalAlpha = AR_OPACITY.group * nearAlpha * (1 - t) ** fadePower;
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.moveTo(left[i].x, left[i].y);
      ctx.lineTo(left[i + 1].x, left[i + 1].y);
      ctx.lineTo(right[i + 1].x, right[i + 1].y);
      ctx.lineTo(right[i].x, right[i].y);
      ctx.closePath();
      ctx.fill();
    }
    ctx.restore();
    return true;
  }

  /** 접지 그림자 — 구글 AR 가이드라인의 shadow plane. 떠 보임 방지. */
  function drawShadow(groundPt, widthPx) {
    if (!groundPt || !(widthPx > 0)) return;
    const rx = widthPx * 0.55;
    const ry = Math.max(2, rx * 0.22);
    const g = ctx.createRadialGradient(groundPt.x, groundPt.y, 0, groundPt.x, groundPt.y, rx);
    g.addColorStop(0, `rgba(0,0,0,${AR_LEGIBILITY.shadowPlaneOpacity})`);
    g.addColorStop(1, "rgba(0,0,0,0)");
    ctx.save();
    ctx.translate(groundPt.x, groundPt.y);
    ctx.scale(1, ry / rx);
    ctx.fillStyle = g;
    ctx.beginPath();
    ctx.arc(0, 0, rx, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
  }

  /** 지주 — 표지가 공중에 뜬 것처럼 보이지 않게 노면과 잇는다. */
  function drawPole(groundPt, basePt, color) {
    if (!groundPt || !basePt) return;
    ctx.save();
    ctx.globalAlpha = AR_OPACITY.pole;
    ctx.strokeStyle = color;
    ctx.lineWidth = Math.max(1.5, Math.abs(groundPt.y - basePt.y) * 0.035);
    ctx.beginPath();
    ctx.moveTo(basePt.x, basePt.y);
    ctx.lineTo(groundPt.x, groundPt.y);
    ctx.stroke();
    ctx.restore();
  }

  /** 표지의 화면 높이를 실제 투영으로 잰다. 초점거리 가정이 필요 없다. */
  function measureScreenHeight(descriptor, anchor, stage) {
    const T = stage?.calibTransform;
    if (!T || !descriptor || !anchor) return null;
    const mount = finite(descriptor.mountHeightM, 2.56);
    const half = descriptor.heightM / 2;
    const centerPt = toStage(projectRouteFluPoint(T, anchor.x, anchor.y, anchor.z + mount + half), stage);
    const basePt = toStage(projectRouteFluPoint(T, anchor.x, anchor.y, anchor.z + mount), stage);
    const groundPt = toStage(projectRouteFluPoint(T, anchor.x, anchor.y, anchor.z), stage);
    if (!centerPt || !basePt) return null;
    const halfPx = Math.abs(centerPt.y - basePt.y);
    if (!(halfPx > 0.5)) return null;
    const distanceM = Math.max(0.5, anchor.x);
    return {
      centerPt, basePt, groundPt,
      heightPx: halfPx * 2,
      // planMarker 는 focal 로 크기를 예측한다. 실측 높이에서 역산해 넘겨주면
      // 행렬 구조에 대한 가정 없이 같은 판단을 할 수 있다.
      impliedFocalPx: (halfPx * 2 * distanceM) / Math.max(1e-6, descriptor.heightM),
    };
  }

  /**
   * 표지 하나를 그린다.
   * @param descriptor signboard.js 의 표지 descriptor
   * @param anchor     projection.pointOnPath 결과 (도로 위 지점)
   * @param stage      window.CarrotVisionStageTransform 스냅샷
   */
  function drawSign(descriptor, anchor, stage, plan, measured, held = false) {
    const m = measured || measureScreenHeight(descriptor, anchor, stage);
    if (!m) return false;
    const { centerPt, basePt, groundPt } = m;
    const heightPx = m.heightPx * (plan?.worldScale ?? 1);
    const widthPx = heightPx * (descriptor.widthM / descriptor.heightM);

    const tex = textures.get(descriptor);
    if (!tex) return false;

    drawShadow(groundPt, widthPx);
    drawPole(groundPt, basePt, descriptor.palette.surface);

    ctx.save();
    ctx.globalAlpha = (plan?.alpha ?? 1) * (descriptor.opacity ?? 1) * AR_OPACITY.group
      * (held ? AR_OPACITY.held : 1);
    ctx.drawImage(tex, centerPt.x - widthPx / 2, centerPt.y - heightPx / 2, widthPx, heightPx);
    ctx.restore();
    return true;
  }

  /**
   * 한 프레임 그리기. runtime 이 presented-frame 신호로 호출한다.
   * @param frame { stage, modelPosition, signs:[{descriptor, distanceM}], sync }
   */
  function render(frame = {}) {
    if (destroyed || !ctx) return false;
    const stage = frame.stage;
    const size = syncSize(stage);
    ctx.setTransform(size.dpr, 0, 0, size.dpr, 0, 0);
    ctx.clearRect(0, 0, size.width, size.height);
    stats.drawn = 0;
    stats.skipped = 0;

    /* 동기화 게이트가 막으면 아무것도 그리지 않는다. 어긋난 표지보다 없는 편이 낫다.
     * 단 Phase 4 의 "유지" 프레임은 예외다 — model 은 없지만 odometry 로 앵커
     * 위치를 계산해 둔 상태이고, 예산 안이라 아직 믿을 수 있다. 이걸 막으면
     * 표지가 깜빡인다. */
    const held = frame.held === true;
    if (!stage || (frame.sync?.canDrawPrecise !== true && !held)) {
      stats.lastReason = frame.sync?.reasons?.[0] || "stage/sync 없음";
      return false;
    }
    stats.lastReason = "";
    stats.held = held;

    stats.skips = [];
    const items = Array.isArray(frame.signs) ? frame.signs : [];

    /* 노면 오버레이를 먼저 깐다. 표지판보다 아래에 있어야 자연스럽다.
     * 리본은 마커의 distanceM 이 아니라 경로 전체를 쓰므로 별도 경로로 그린다. */
    for (const item of items) {
      if (markerForm(item.descriptor?.kind) !== AR_MARKER_FORM.SURFACE) continue;
      const descriptor = item.descriptor;
      const laneWidthM = finite(descriptor?.laneWidthM, AR_GEOMETRY.laneWidthDefaultM);
      const isLaneBand = descriptor?.kind === AR_MARKER_KIND.LANE_BAND;
      const surfaceOptions = isLaneBand
        ? {
          widthM: finite(descriptor.widthM, laneWidthM),
          offsetM: finite(descriptor.laneOffsetM, 0),
          startM: Math.max(1, finite(item.distanceM, 0) - AR_GEOMETRY.laneBand.lengthM),
          endM: finite(item.distanceM, 0),
          nearAlpha: 0.62,
          fadePower: 1.1,
          color: paletteFor(descriptor.tone).surface,
        }
        : {
          widthM: laneWidthM,
          startM: 1,
          endM: AR_RIBBON.maxDistanceM,
          color: paletteFor(descriptor?.tone).surface,
        };
      if (drawRibbon(frame.modelPosition, stage, surfaceOptions)) {
        stats.drawn += 1;
      } else {
        stats.skipped += 1;
        stats.skips.push(`${item.descriptor?.kind}: 경로점 부족`);
      }
    }

    for (const item of items) {
      if (markerForm(item.descriptor?.kind) === AR_MARKER_FORM.SURFACE) continue;
      // 유지 프레임은 anchor.js 가 적분해 둔 좌표를 그대로 쓴다.
      const anchor = item.anchor || pointOnPath(frame.modelPosition, item.distanceM);
      if (!anchor) {
        stats.skipped += 1;
        stats.skips.push(`${item.distanceM}m: 경로가 짧음(pathLen=${
          Array.isArray(frame.modelPosition?.x) ? Math.round(frame.modelPosition.x.at(-1)) : "?"}m)`);
        continue;
      }
      // 초점거리는 calibTransform 에서 뽑을 수 없다(그 행렬의 [0][0]은 cx다).
      // 대신 실제로 투영해서 화면 높이를 재고, 그 값으로 clamp 판단을 한다.
      const measured = measureScreenHeight(item.descriptor, anchor, stage);
      if (!measured) {
        stats.skipped += 1;
        // 좌표를 함께 남긴다. y가 크면 화면 밖(FOV), x가 작거나 음수면 카메라 뒤다.
        stats.skips.push(`${item.distanceM}m: 투영 실패(x${
          Math.round(finite(anchor.x, 0))} y${Math.round(finite(anchor.y, 0))})`);
        continue;
      }
      const plan = planMarker({
        distanceM: item.distanceM,
        egoSpeedMps: frame.egoSpeedMps,
        worldHeightM: item.descriptor?.heightM,
        focalPx: measured.impliedFocalPx,
        canvas: size,
      });
      if (!plan.visible) {
        stats.skipped += 1;
        stats.skips.push(`${item.distanceM}m: 표시 억제(${plan.phase}/${plan.clamped}, ${Math.round(plan.projectedPx)}px)`);
        continue;
      }
      if (drawSign(item.descriptor, anchor, stage, plan, measured, held)) stats.drawn += 1;
      else { stats.skipped += 1; stats.skips.push(`${item.distanceM}m: 그리기 실패`); }
    }
    return true;
  }

  function status() {
    return Object.freeze({
      destroyed,
      drawn: stats.drawn,
      held: stats.held === true,
      skipped: stats.skipped,
      skips: Object.freeze((stats.skips || []).slice(0, 6)),
      lastReason: stats.lastReason,
      textureCount: textures.size,
      canvas: lastSize ? `${lastSize.width}x${lastSize.height}@${lastSize.dpr}` : "-",
    });
  }

  function destroy() {
    if (destroyed) return false;
    destroyed = true;
    textures.clear();
    canvas.remove?.();
    return true;
  }

  return Object.freeze({ canvas, render, status, destroy });
}
