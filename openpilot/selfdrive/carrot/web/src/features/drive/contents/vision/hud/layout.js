"use strict";

/* 실제 앵커 사각형을 기준으로 낮은 우선순위부터 제거한다.
 * P0 좌하단 속도판과 활성 이벤트(방향지시등/비상등)는 숨기지 않는다.
 * 방향지시등/비상등(topCenter)은 순간적으로 켜지는 이벤트라 다른 위젯의
 * 축소·숨김도 유발하지 않는다 → 충돌 판정 peer에서 제외한다. */
const COLLISION_GAP_PX = 8;

function rect(node) {
  if (!node || node.hasAttribute?.("hidden")) return null;
  const value = node.getBoundingClientRect?.();
  if (!value || value.width <= 0 || value.height <= 0) return null;
  return value;
}

function intersects(a, b, gap = COLLISION_GAP_PX) {
  if (!a || !b) return false;
  return a.left < b.right + gap
    && a.right + gap > b.left
    && a.top < b.bottom + gap
    && a.bottom + gap > b.top;
}

function exceeds(nodeRect, boundary) {
  if (!nodeRect || !boundary) return false;
  return nodeRect.left < boundary.left
    || nodeRect.top < boundary.top
    || nodeRect.right > boundary.right
    || nodeRect.bottom > boundary.bottom;
}

export function applyHudDegradation(root, zones = {}) {
  if (!root) return "";
  const shed = [];
  const apply = (token) => {
    shed.push(token);
    root.dataset.shed = shed.join(" ");
  };
  root.dataset.shed = "";
  const boundary = rect(root);
  if (!boundary) return "";

  const conflicts = (key, peers) => {
    const source = rect(zones[key]);
    return exceeds(source, boundary)
      || peers.some((peer) => intersects(source, rect(zones[peer])));
  };

  // P5 TPMS: 보호 속도판 또는 우측 상단과 충돌하면 가장 먼저 제거.
  if (conflicts("bottomRight", ["bottomLeft", "topRight"])) apply("p5");
  // P4 accel/steer/fuel/DEF: 좌측·속도판 침범 시 우측 HUD 전체 제거.
  //   (방향지시등/비상등=topCenter은 peer에서 제외 — 다른 UI를 가리지 않는다.)
  if (conflicts("topRight", ["topLeft", "bottomLeft", "bottomRight"])) apply("p4");
  // 이후에도 상단이 좁으면 좌측의 낮은 우선순위부터 단계적으로 제거.
  if (conflicts("topLeft", ["topRight", "bottomLeft"])) apply("p3");
  if (conflicts("topLeft", ["topRight", "bottomLeft"])) apply("p2");
  if (conflicts("topLeft", ["topRight", "bottomLeft"])) apply("p1");

  return root.dataset.shed;
}

export function createHudLayoutObserver(root, onLayout, target = globalThis) {
  if (!root || typeof onLayout !== "function") return null;
  let frameId = 0;
  let destroyed = false;

  function flush() {
    frameId = 0;
    if (!destroyed) onLayout();
  }

  function schedule() {
    if (destroyed || frameId) return;
    if (typeof target.requestAnimationFrame === "function") {
      frameId = target.requestAnimationFrame(flush);
    } else {
      frameId = target.setTimeout?.(flush, 0) || 0;
    }
  }

  const ResizeObserverCtor = target.ResizeObserver;
  const resizeObserver = typeof ResizeObserverCtor === "function"
    ? new ResizeObserverCtor(schedule)
    : null;
  resizeObserver?.observe(root);

  const fonts = target.document?.fonts;
  fonts?.ready?.then(schedule).catch?.(() => {});
  fonts?.addEventListener?.("loadingdone", schedule);
  for (const image of root.querySelectorAll?.("img") || []) {
    if (!image.complete) image.addEventListener("load", schedule, { once: true });
  }
  schedule();

  function destroy() {
    if (destroyed) return;
    destroyed = true;
    resizeObserver?.disconnect();
    fonts?.removeEventListener?.("loadingdone", schedule);
    if (frameId && typeof target.cancelAnimationFrame === "function") {
      target.cancelAnimationFrame(frameId);
    } else if (frameId) {
      target.clearTimeout?.(frameId);
    }
    frameId = 0;
  }

  return Object.freeze({ destroy, schedule });
}
