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
import { createMarkerIdentityTracker } from "./marker_identity.js";
import { createRouteMatcher } from "./route_matcher.js";
import { createRenderPerformancePolicy } from "./performance_policy.js";
import { createDeviceWorldPose } from "./world_pose.js";

let renderer = null;
let anchorStore = null;
let performancePolicy = null;
let identityTracker = null;
let routeMatcher = null;
let worldPoseTracker = null;
let worldPoseEpoch = null;
let retainedComposeSources = { navi: null, modelPosition: null };
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
    identityTracker = createMarkerIdentityTracker();
    routeMatcher = createRouteMatcher();
    worldPoseTracker = createDeviceWorldPose();
    worldPoseEpoch = null;
    retainedComposeSources = { navi: null, modelPosition: null };
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
    identityTracker.reset();
    routeMatcher.reset();
    renderer.reset?.();
  }
  const requestedWorldEpoch = String(payload.worldPoseEpoch || "uninitialized");
  if (payload.timelineDiscontinuity === true || worldPoseEpoch !== requestedWorldEpoch) {
    worldPoseTracker.reset({
      epoch: requestedWorldEpoch,
      reason: payload.timelineDiscontinuityReason || "navigation/session epoch initialized",
    });
    worldPoseEpoch = requestedWorldEpoch;
  }
  const worldPose = payload.deviceOdometry
    ? worldPoseTracker.update({
      timestampNs: payload.worldPoseTargetTimestampNs,
      odometry: payload.deviceOdometry,
      livePose: payload.livePose,
      geographicObservation: payload.geographicObservation,
      trackingState: payload.tracking?.state,
    })
    : worldPoseTracker.status();
  const sourceUpdate = payload.sourceUpdate || {};
  if (Object.prototype.hasOwnProperty.call(sourceUpdate, "navi")) {
    retainedComposeSources.navi = sourceUpdate.navi;
  }
  if (Object.prototype.hasOwnProperty.call(sourceUpdate, "modelPosition")) {
    retainedComposeSources.modelPosition = sourceUpdate.modelPosition;
  }
  const composeInput = {
    ...(payload.composeInput || {}),
    navi: retainedComposeSources.navi,
    modelPosition: retainedComposeSources.modelPosition,
  };
  const composition = composeFrame(composeInput, { identityTracker, routeMatcher });
  const isProbe = composition.signs.length > 0
    && composition.signs.every((item) => item?.source === "calibrationProbe");
  const heldResult = anchorStore.update({
    nowMs: payload.nowMs,
    navi: composeInput.navi,
    probe: isProbe,
    candidates: composition.fresh,
    activeMarkers: composition.signs,
    lifecycleAuthoritative: isProbe || (
      Boolean(composeInput.navi) && composeInput.naviUsable !== false
    ),
    modelPosition: composeInput.modelPosition,
    valid: isProbe || !composeInput.navi || composeInput.naviUsable !== false,
    canHold: payload.canHoldAnchor === true,
    precise: payload.tracking?.canCreateAnchor === true,
    trackingState: payload.tracking?.state,
    trackingRecovered: payload.tracking?.recovered === true,
    retainAnchor: payload.tracking?.retainAnchor === true,
    trackingUncertaintyM: payload.tracking?.uncertainty?.lateralM,
    reason: payload.tracking?.reasons?.[0] || "",
    odometry: payload.routeOdometry,
    worldPose,
  });

  const ok = renderer.render({
    nowMs: payload.presentationNowMs ?? payload.nowMs,
    diagnosticsEnabled: payload.diagnosticsEnabled === true,
    clockDomain: payload.clockDomain,
    stage: payload.stage,
    sync: payload.sync,
    tracking: payload.tracking,
    deviceWorldPose: worldPose,
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
    traceFrameId: payload.traceFrameId ?? null,
    debugFrame: payload.debugFrame || null,
    ok,
    composition: Object.freeze({
      signCount: composition.signs.length,
      anchoredCount: composition.fresh?.length || 0,
      sources: Object.freeze(composition.signs.map((item) => item.source || "unknown")),
      diag: composition.diag || null,
    }),
    hold: anchorStore.status(payload.nowMs),
    tracking: payload.tracking || null,
    worldPose,
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
      identityTracker?.reset();
      routeMatcher?.reset();
      renderer?.reset?.();
      worldPoseTracker?.reset({ epoch: "uninitialized", reason: "worker reset" });
      worldPoseEpoch = null;
      retainedComposeSources = { navi: null, modelPosition: null };
      break;
    case "destroy":
      renderer?.destroy();
      renderer = null;
      anchorStore = null;
      identityTracker = null;
      routeMatcher = null;
      worldPoseTracker = null;
      worldPoseEpoch = null;
      retainedComposeSources = { navi: null, modelPosition: null };
      performancePolicy = null;
      break;
    default:
      break;
  }
};
