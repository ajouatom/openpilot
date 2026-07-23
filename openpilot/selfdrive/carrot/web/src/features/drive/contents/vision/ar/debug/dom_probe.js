"use strict";

/* AR 캔버스의 실제 화면 상태를 DOM에서 직접 확인한다.
 *
 * 워커는 OffscreenCanvas에 그리므로 "그렸다(drawn>0)"와 "사람이 볼 수 있다"는
 * 별개다. 캔버스가 0 크기이거나, 스테이지 밖에 있거나, 다른 요소에 덮이면
 * 렌더러는 정상인데 화면에는 아무것도 없다. 그 구간을 여기서 가른다.
 */

export const AR_CANVAS_SELECTOR = ".carrot-ar__canvas";

function rectOf(element) {
  const r = element.getBoundingClientRect?.();
  if (!r) return null;
  return {
    x: Math.round(r.left), y: Math.round(r.top),
    w: Math.round(r.width), h: Math.round(r.height),
  };
}

/** 짧게. id가 있으면 id만, 없으면 첫 클래스 하나만 쓴다(패널이 좁다). */
function describe(element) {
  if (!element) return "없음";
  const tag = String(element.tagName || "?").toLowerCase();
  if (element.id) return `${tag}#${element.id}`;
  const cls = typeof element.className === "string" && element.className.trim()
    ? element.className.trim().split(/\s+/)[0]
    : "";
  return cls ? `${tag}.${cls}` : tag;
}

/**
 * @returns {{rect, style, covering, coveredBy, ready, backend}|null}
 *   covering: 캔버스 중심점에서 실제로 hit-test되는 요소가 캔버스 자신인가.
 */
export function probeArCanvas(target = globalThis, options = {}) {
  const documentRoot = options.document || target?.document;
  const canvas = options.canvas
    || documentRoot?.querySelector?.(options.selector || AR_CANVAS_SELECTOR);
  if (!canvas) return null;

  const rect = rectOf(canvas);
  const view = canvas.ownerDocument?.defaultView || target;
  const computed = view?.getComputedStyle?.(canvas) || {};
  const style = {
    display: computed.display || "",
    visibility: computed.visibility || "",
    opacity: computed.opacity || "",
    zIndex: computed.zIndex || "",
  };

  // 중심점에 무엇이 올라와 있는지. pointer-events:none이라 캔버스 자신은 잡히지
  // 않으므로, 잡힌 요소가 캔버스를 덮는 형제인지 스테이지 자체인지로 판단한다.
  let coveredBy = null;
  if (rect && rect.w > 0 && rect.h > 0 && documentRoot?.elementFromPoint) {
    const hit = documentRoot.elementFromPoint(rect.x + rect.w / 2, rect.y + rect.h / 2);
    coveredBy = hit === canvas ? null : hit;
  }

  return {
    rect,
    style,
    coveredBy: describe(coveredBy),
    ready: canvas.dataset?.arReady ?? null,
    backend: canvas.dataset?.arBackend ?? null,
    drawn: canvas.dataset?.arDrawn ?? null,
  };
}

/** 패널/로그가 함께 쓰는 한 줄 표현. */
export function formatCanvasProbe(probe) {
  if (!probe) return "캔버스 없음";
  const { rect, style } = probe;
  if (!rect) return "rect 없음";
  const size = `${rect.w}x${rect.h}@${rect.x},${rect.y}`;
  const hidden = style.display === "none" || style.visibility === "hidden"
    || Number(style.opacity) === 0;
  const bits = [size, `z${style.zIndex || "auto"}`];
  if (hidden) bits.push(`숨김(${style.display}/${style.visibility}/${style.opacity})`);
  if (!(rect.w > 0 && rect.h > 0)) bits.push("크기0");
  if (probe.coveredBy && probe.coveredBy !== "없음") bits.push(`위:${probe.coveredBy}`);
  return bits.join(" ");
}
