/* AR 오버레이 워커 — 투영·기하 연산과 그리기를 접속 디바이스의 워커에서 한다.
 *
 * 왜 워커인가:
 *   기존 HUD(vision_webgl2)와 같은 원칙이다. 주행 화면은 20Hz 로 계속 도는데,
 *   투영·마커 배치·텍스처 생성을 메인 스레드에서 하면 UI 입력과 경쟁한다.
 *   OffscreenCanvas 를 넘겨 받아 여기서 전부 처리하고, 메인 스레드는 상태
 *   스냅샷만 던진다.
 *
 * 백프레셔:
 *   vision_webgl2_worker 와 같은 규약. 한 프레임을 그리면 "drawn" 을 보낸다.
 *   메인은 in-flight 중이면 최신 프레임 하나만 들고 있다가 교체해서 보낸다.
 *   → 워커가 밀려도 큐가 자라지 않고 항상 최신 상태만 그린다.
 *
 * 이 파일은 DOM 에 접근하지 않는다. 텍스처도 OffscreenCanvas 로 만든다.
 */

import { createThreeArRenderer } from "./three_adapter.js";
import { AR_HOLD_STATE } from "./anchor.js";
import { createContinuousAnchorStore } from "./anchor_store.js";
import { composeFrame } from "./compose.js";
import { createRenderPerformancePolicy } from "./performance_policy.js";

let renderer = null;
let anchorStore = null;
let performancePolicy = null;
let failed = "";

function reply(message) {
  self.postMessage(message);
}

function workerNowMs() {
  return self.performance?.now?.() ?? Date.now();
}

function init(canvas, performanceOptions) {
  try {
    renderer = createThreeArRenderer({
      surface: canvas,
      canvasFactory: (width, height) => new OffscreenCanvas(width, height),
      onFatal: (reason) => {
        failed = String(reason || "Three renderer failed");
        reply({ type: "error", error: failed, backend: "three" });
      },
    });
    if (!renderer) throw new Error("renderer 생성 실패");
    anchorStore = createContinuousAnchorStore();
    performancePolicy = createRenderPerformancePolicy(performanceOptions);
    reply({ type: "ready", backend: "three" });
  } catch (error) {
    failed = error?.message || String(error);
    reply({ type: "error", error: failed, backend: "three" });
  }
}

function drawFrame(payload = {}) {
  if (!renderer || failed) {
    reply({ type: "drawn", ok: false });
    return;
  }
  const workStartedAt = workerNowMs();
  if (payload.timelineDiscontinuity === true) {
    anchorStore.reset();
    renderer.reset?.();
  }
  const composition = composeFrame(payload.composeInput || {});
  const composeInput = payload.composeInput || {};
  const isProbe = composition.signs.length > 0
    && composition.signs.every((item) => item?.source === "calibrationProbe");
  const heldResult = anchorStore.update({
    nowMs: payload.nowMs,
    navi: composeInput.navi,
    probe: isProbe,
    candidates: composition.fresh,
    modelPosition: composeInput.modelPosition,
    valid: isProbe || !composeInput.navi || composeInput.naviUsable !== false,
    canHold: payload.canHoldAnchor === true,
    precise: payload.sync?.canDrawPrecise === true,
    odometry: payload.cameraOdometry,
  });

  const ok = renderer.render({
    nowMs: payload.nowMs,
    clockDomain: payload.clockDomain,
    stage: payload.stage,
    sync: payload.sync,
    held: heldResult.state === AR_HOLD_STATE.HELD,
    modelPosition: composeInput.modelPosition,
    egoSpeedMps: composeInput.egoSpeedMps,
    // Three는 반드시 store가 승인한 월드 앵커만 소비한다. store가 놓친 순간
    // 현재 model path의 40m 앞에 새 표지를 만드는 fallback은 위치 점프를 만든다.
    signs: heldResult.anchors || [],
  });
  const performance = performancePolicy.observe(workerNowMs() - workStartedAt);
  reply({
    type: "drawn",
    ok,
    composition: Object.freeze({
      signCount: composition.signs.length,
      anchoredCount: composition.fresh?.length || 0,
      sources: Object.freeze(composition.signs.map((item) => item.source || "unknown")),
    }),
    hold: anchorStore.status(payload.nowMs),
    renderer: renderer.status(),
    performance,
  });
}

self.onmessage = (event) => {
  const data = event.data || {};
  switch (data.type) {
    case "init":
      init(data.canvas, data.performance);
      break;
    case "frame":
      drawFrame(data.payload);
      break;
    case "resize":
      // OffscreenCanvas 는 워커가 소유하므로 크기도 여기서 바꾼다.
      if (renderer?.canvas && data.pixelWidth > 0 && data.pixelHeight > 0) {
        renderer.canvas.width = data.pixelWidth;
        renderer.canvas.height = data.pixelHeight;
      }
      break;
    case "reset":
      anchorStore?.reset();
      renderer?.reset?.();
      break;
    case "destroy":
      renderer?.destroy();
      renderer = null;
      anchorStore = null;
      performancePolicy = null;
      break;
    default:
      break;
  }
};
