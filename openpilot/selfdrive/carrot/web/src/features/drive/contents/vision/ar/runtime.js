/* AR 오버레이 런타임 — 생명주기·데이터 임대·프레임 동기화 소유.
 *
 * 구조는 drive_insights/runtime.js 와 같은 패턴을 따른다.
 *   createArRuntime(...) → { activate, deactivate, resize, status, destroy }
 *   getOrCreate... 싱글턴 + install...Facade
 *
 * 이 런타임이 지키는 것 세 가지.
 *   1) 자체 rAF 루프를 돌리지 않는다. presented-frame 신호(rVFC)로만 그린다.
 *      → 독립 canvas 여도 영상과 같은 프레임에 합성된다.
 *   2) 스테이지 변환을 스스로 계산하지 않는다. home_drive 가 노출한 스냅샷을 쓴다.
 *      → 표지가 기존 차선/경로 오버레이와 어긋나지 않는다.
 *   3) "ar" 데이터 임대를 활성 중에만 쥔다.
 *      → cameraOdometry/livePose/carrotNavi 구독이 AR 을 켤 때만 열린다.
 */

import { AR_HOLD_STATE } from "./anchor.js";
import { createContinuousAnchorStore } from "./anchor_store.js";
import { evaluateFrameSync } from "./frame_sync.js";
import { composeFrame } from "./compose.js";
import { AR_RENDER } from "./tokens.js";
import { evaluateGeoPositionQuality } from "./position_quality.js";
import { odometryInDeviceFrame } from "./odometry.js";

const runtimeSingletons = new WeakMap();

export const AR_WORKER_ASSET_ID = "drive.vision.ar.worker";

/**
 * 워커 파이프.
 *
 * 캔버스 소유권을 워커에 넘기고(transferControlToOffscreen) 이후 상태만 보낸다.
 * 백프레셔는 vision_webgl2 와 같은 규약이다 — 그린 프레임마다 "drawn" 이 오고,
 * 아직 안 왔으면 최신 프레임 하나만 들고 있다가 교체한다. 큐가 자라지 않는다.
 *
 * 실패하면 null 을 돌려준다. 제품 경로에는 메인 스레드 Canvas2D fallback이 없다.
 */
function createWorkerPipe({ host, documentRoot, target, options, onBroken }) {
  if (typeof target.Worker !== "function") return null;
  if (typeof documentRoot.createElement !== "function") return null;

  const canvas = documentRoot.createElement("canvas");
  if (typeof canvas.transferControlToOffscreen !== "function") return null;
  canvas.className = "carrot-ar__canvas";
  canvas.style.cssText = "position:absolute;inset:0;width:100%;height:100%;pointer-events:none;z-index:6;display:none";
  host.appendChild(canvas);

  let worker;
  try {
    const resolve = options.resolveAssetUrl
      || ((id) => target.CarrotAssetUrl?.resolve?.(id));
    const url = String(resolve(AR_WORKER_ASSET_ID) || "");
    if (!url) throw new Error("AR worker asset URL 없음");
    worker = new target.Worker(url);
  } catch (error) {
    canvas.remove?.();
    target.console?.warn?.("[carrot] AR 워커 시작 실패 — AR 숨김:", error?.message || error);
    return null;
  }

  let ready = false;
  let broken = "";
  let inFlight = false;
  let pending = null;
  let lastStatus = null;
  let lastSyncReason = "";
  let lastSize = { pixelWidth: 0, pixelHeight: 0 };

  // Keep the last worker decision on the actual surface.  This is deliberately
  // DOM-only (no visible debug UI and no console spam) so a remote browser can
  // distinguish "canvas exists" from "Three drew a frame" even though the
  // renderer itself runs in an isolated OffscreenCanvas worker.
  function setCanvasDiagnostic(name, value) {
    if (!canvas.dataset) return;
    if (value === null || value === undefined || value === "") delete canvas.dataset[name];
    else canvas.dataset[name] = String(value);
  }

  setCanvasDiagnostic("arReady", false);
  setCanvasDiagnostic("arBackend", "pending");

  function failClosed(reason) {
    if (broken) return;
    broken = String(reason || "Three AR worker failed");
    ready = false;
    inFlight = false;
    pending = null;
    canvas.style.display = "none";
    setCanvasDiagnostic("arReady", false);
    setCanvasDiagnostic("arBackend", "failed");
    setCanvasDiagnostic("arReason", broken);
    worker.terminate?.();
    onBroken?.(broken);
  }

  worker.onmessage = (event) => {
    const data = event.data || {};
    if (data.type === "ready") {
      if (data.backend !== "three") { failClosed("unexpected AR renderer backend"); return; }
      ready = true;
      canvas.style.display = "";
      setCanvasDiagnostic("arReady", true);
      setCanvasDiagnostic("arBackend", data.backend);
      return;
    }
    if (data.type === "error") { failClosed(data.error || "worker error"); return; }
    if (data.type === "drawn") {
      if (broken) return;
      inFlight = false;
      lastStatus = data;
      setCanvasDiagnostic("arOk", data.ok === true);
      setCanvasDiagnostic("arDrawn", data.renderer?.drawn ?? 0);
      setCanvasDiagnostic("arSkipped", data.renderer?.skipped ?? 0);
      const renderReason = String(data.renderer?.lastReason || "");
      const holdReason = String(data.hold?.reason || "");
      setCanvasDiagnostic("arRenderReason", renderReason || null);
      setCanvasDiagnostic("arHoldState", data.hold?.state || null);
      setCanvasDiagnostic("arHoldReason", holdReason || null);
      setCanvasDiagnostic("arAnchorMode", data.hold?.sourceMode || null);
      setCanvasDiagnostic("arAnchorCount", data.hold?.anchorCount ?? null);
      setCanvasDiagnostic("arHoldMs", data.hold?.holdMs ?? null);
      setCanvasDiagnostic("arDriftM", data.hold?.driftM ?? null);
      // arReason은 기존 원격 점검 도구와의 호환용 요약이고, 위 필드들이
      // 실제 원인별 source of truth다.
      setCanvasDiagnostic("arReason", renderReason || lastSyncReason || holdReason || null);
      setCanvasDiagnostic("arSignCount", data.composition?.signCount ?? 0);
      setCanvasDiagnostic("arComposedAnchorCount", data.composition?.anchoredCount ?? 0);
      if (pending) { const next = pending; pending = null; post(next); }
    }
  };
  worker.onerror = (event) => { failClosed(event?.message || "worker failed"); };

  function post(payload) {
    if (!ready || broken) return false;
    if (inFlight) { pending = payload; return true; }   // 최신 것만 남긴다
    inFlight = true;
    worker.postMessage({ type: "frame", payload });
    return true;
  }

  try {
    const offscreen = canvas.transferControlToOffscreen();
    worker.postMessage({
      type: "init",
      canvas: offscreen,
      performance: {
        targetFps: options.targetFps,
        degradedFps: options.degradedFps,
      },
    }, [offscreen]);
  } catch (error) {
    canvas.remove?.();
    worker.terminate?.();
    target.console?.warn?.("[carrot] OffscreenCanvas 이전 실패 — AR 숨김:", error?.message || error);
    return null;
  }

  /** 레이아웃 측정은 메인만 할 수 있다. 캔버스 픽셀 크기를 워커에 알려 준다. */
  function syncSize() {
    const rect = canvas.getBoundingClientRect?.() || {};
    const dpr = target.devicePixelRatio || 1;
    const pixelWidth = Math.max(1, Math.round((rect.width || 1) * dpr));
    const pixelHeight = Math.max(1, Math.round((rect.height || 1) * dpr));
    if (pixelWidth === lastSize.pixelWidth && pixelHeight === lastSize.pixelHeight) return;
    lastSize = { pixelWidth, pixelHeight };
    worker.postMessage({ type: "resize", pixelWidth, pixelHeight });
  }

  return Object.freeze({
    canvas,
    render(payload) {
      syncSize();
      lastSyncReason = String(payload?.sync?.reasons?.[0] || "");
      setCanvasDiagnostic("arSync", payload?.sync?.state || "missing");
      setCanvasDiagnostic("arCanDraw", payload?.sync?.canDrawPrecise === true);
      setCanvasDiagnostic("arClockDomain", payload?.clockDomain || null);
      setCanvasDiagnostic("arSyncReason", lastSyncReason || null);
      setCanvasDiagnostic("arReason", lastSyncReason || null);
      return post(payload);
    },
    reset() { worker.postMessage({ type: "reset" }); },
    status() {
      return Object.freeze({
        mode: "worker", ready, broken: broken || null,
        composition: lastStatus?.composition || null,
        hold: lastStatus?.hold || null,
        renderer: lastStatus?.renderer || null,
        performance: lastStatus?.performance || null,
        canvas: `${lastSize.pixelWidth}x${lastSize.pixelHeight}`,
      });
    },
    destroy() {
      try { worker.postMessage({ type: "destroy" }); } catch (_) {}
      worker.terminate?.();
      canvas.remove?.();
    },
  });
}

function finite(v, fallback = null) {
  const n = Number(v);
  return Number.isFinite(n) ? n : fallback;
}

/**
 * AR 계산 시간축을 고른다.
 *
 * 라이브 수신시각은 performance.now() 도메인이지만, 녹화 리플레이는 영상의
 * media timeline이 유일한 기준이다. 두 값을 섞으면 일시정지한 리플레이가
 * 120ms 뒤 stale로 바뀌며 표지가 깜빡인다. wall clock은 제출 cadence에만 쓰고
 * 동기화·앵커 적분에는 이 결과만 사용한다.
 */
export function resolveArTimeline(target = globalThis, monotonicNowMs = 0) {
  const replay = target?.CarrotVisionReplay;
  const replayStatus = replay?.status?.() || null;
  const replayActive = typeof replay?.isActive === "function"
    ? replay.isActive()
    : replayStatus?.active === true;
  const replaySeconds = finite(replayStatus?.currentTime, null);
  if (replayActive && replaySeconds !== null && replaySeconds >= 0) {
    return Object.freeze({
      domain: "replay-media",
      nowMs: replaySeconds * 1000,
    });
  }
  return Object.freeze({
    domain: "live-monotonic",
    nowMs: Math.max(0, finite(monotonicNowMs, 0)),
  });
}

export function createArRuntime(options = {}) {
  const target = options.target || globalThis;
  const documentRoot = options.document || target.document;
  if (!documentRoot?.createElement) return null;

  const host = options.host
    || documentRoot.getElementById?.("carrotVisionStage")
    || documentRoot.querySelector?.(".carrot-stage");
  if (!host) return null;

  let workerFailureHandler = () => {};
  /* 제품 renderer는 Worker Three.js만 사용한다. renderer injection은 정적 테스트 seam이다. */
  const workerPipe = options.renderer ? null : createWorkerPipe({
    host,
    documentRoot,
    target,
    options,
    onBroken: (reason) => workerFailureHandler(reason),
  });
  const renderer = options.renderer || null;
  if (!renderer && !workerPipe) return null;

  let active = false;
  let destroyed = false;
  let lease = null;
  let unsubscribeFrame = null;
  let lastSync = null;
  let lastPositionQuality = null;
  let lastFrameAt = 0;
  let lastSubmitAt = null;
  let frames = 0;
  let activatedAt = 0;
  let frameSignalMode = "none";
  let presentedSignals = 0;
  let lastPresentedSource = null;
  let presentationFrameId = null;
  const requestPresentationFrame = options.requestAnimationFrame
    || target.requestAnimationFrame?.bind(target)
    || null;
  const cancelPresentationFrame = options.cancelAnimationFrame
    || target.cancelAnimationFrame?.bind(target)
    || (() => {});
  const targetFps = Math.max(1, finite(options.targetFps, AR_RENDER.targetFps));
  const degradedFps = Math.min(
    targetFps,
    Math.max(1, finite(options.degradedFps, AR_RENDER.degradedFps)),
  );
  let probeDistanceM = finite(options.probeDistanceM, 40);
  let lastAnchor = null;
  let lastTimeline = null;
  const anchorStore = options.anchorStore
    || createContinuousAnchorStore({ limits: options.holdLimits });

  workerFailureHandler = () => {
    if (!active || destroyed) return;
    active = false;
    stopPresentationLoop();
    unsubscribeFrame?.();
    unsubscribeFrame = null;
    frameSignalMode = "none";
    anchorStore.reset();
    lease?.release?.();
    lease = null;
  };

  function stopPresentationLoop() {
    if (presentationFrameId === null) return false;
    cancelPresentationFrame(presentationFrameId);
    presentationFrameId = null;
    return true;
  }

  function schedulePresentationLoop() {
    if (!active || destroyed || !requestPresentationFrame || presentationFrameId !== null) {
      return false;
    }
    presentationFrameId = requestPresentationFrame(() => {
      presentationFrameId = null;
      if (!active || destroyed) return;
      // State selection remains tied to the latest presented camera frame.
      // Only the vehicle-relative pose is advanced at the presentation rate.
      drawFrame(false);
      schedulePresentationLoop();
    });
    return true;
  }

  function drawFrame(force = false) {
    if (!active || destroyed) return false;
    const frameNowMs = target.performance?.now?.() ?? Date.now();
    const currentFps = workerPipe
      ? Math.max(1, finite(workerPipe.status().performance?.fps, targetFps))
      : targetFps;
    const minFrameIntervalMs = 1000 / currentFps;
    if (!force && lastSubmitAt !== null && frameNowMs - lastSubmitAt < minFrameIntervalMs) return false;
    lastSubmitAt = frameNowMs;
    const provider = options.provider || target.CarrotDriveLiveStateProvider;
    const stage = options.stage || target.CarrotVisionStageTransform;
    const snapshot = provider?.snapshot?.();
    const overlayState = snapshot?.overlayState || {};
    const hudState = snapshot?.hudState || {};
    // modelV2 is retained at 20 Hz while roadCameraState is a lightweight
    // 4 Hz sync sample. Resolve the model belonging to the presented video
    // frame, just like the existing road overlay, instead of comparing the
    // latest model with an older camera sample and permanently fail-closing.
    const frameSync = options.frameSync || target.CarrotVisionFrameSync;
    const modelV2 = frameSync?.selectModel?.() || overlayState.modelV2;

    const timeline = resolveArTimeline(target, frameNowMs);
    const timelineDeltaMs = lastTimeline?.domain === timeline.domain
      ? timeline.nowMs - lastTimeline.nowMs
      : null;
    const timelineDiscontinuity = Boolean(
      lastTimeline
      && (
        lastTimeline.domain !== timeline.domain
        || timelineDeltaMs < 0
        || timelineDeltaMs > AR_RENDER.timelineDiscontinuityMs
      ),
    );
    lastTimeline = timeline;
    const inputSync = evaluateFrameSync({
      roadCameraState: overlayState.roadCameraState,
      modelV2,
      cameraOdometry: overlayState.cameraOdometry,
      livePose: overlayState.livePose,
      liveCalibration: overlayState.liveCalibration,
      carrotNavi: overlayState.carrotNavi,
      nowMs: timeline.nowMs,
      clockDomain: timeline.domain,
      receivedAtMonotonic: snapshot?.receivedAtMonotonic,
    });
    const positionQuality = evaluateGeoPositionQuality({
      nowMs: timeline.nowMs,
      clockDomain: timeline.domain,
      receivedAtMonotonic: snapshot?.receivedAtMonotonic,
      naviUsable: inputSync.naviUsable,
      naviVehicle: overlayState.carrotNavi?.vehicle,
      gpsLocationExternal: hudState.gpsLocationExternal,
    });
    lastPositionQuality = positionQuality;
    const sync = inputSync;
    lastSync = sync;

    const posX = modelV2?.position?.x;
    const posY = modelV2?.position?.y;
    lastAnchor = Array.isArray(posX) && Array.isArray(posY)
      ? Object.freeze({
        pathLengthM: posX.at(-1) ?? null,
        lateralAtProbeM: (() => {
          for (let i = 1; i < Math.min(posX.length, posY.length); i += 1) {
            if (posX[i] >= probeDistanceM) return +Number(posY[i]).toFixed(2);
          }
          return null;
        })(),
        lateralSpread: +(Math.max(...posY.map(Number)) - Math.min(...posY.map(Number))).toFixed(2),
      })
      : null;

    const modelPosition = modelV2?.position;
    const nowMs = timeline.nowMs;
    const egoSpeedMps = finite(hudState.carState?.vEgo, 0);
    // cameraOdometry is published in the calibrated/model frame. The anchor
    // store owns device-frame world poses, so convert once before either the
    // main seam or the production worker integrates it.
    const deviceOdometry = odometryInDeviceFrame(
      overlayState.cameraOdometry,
      overlayState.liveCalibration?.rpyCalib,
    );
    /* 순수 입력만 만든다. 워커 모드에서는 마커 선택·위경도 변환·경로 배치까지
     * 모두 워커가 composeFrame()으로 처리한다. 메인 폴백도 같은 함수를 써서
     * 두 경로의 결과가 갈라지지 않는다. */
    const composeInput = {
      navi: overlayState.carrotNavi || null,
      naviUsable: sync.naviUsable,
      laneWidthM: finite(overlayState.lateralPlan?.laneWidth, 3.5),
      egoSpeedMps,
      calibrationProbe: options.calibrationProbe === true,
      probeDistanceM,
      canDrawPrecise: sync.canDrawPrecise,
      geoAllowed: positionQuality.canUseGeo,
      modelPosition,
    };
    // 실제 Navi의 off-route 여부는 worker/store의 valid 입력이 맡는다. 여기서
    // canHold까지 끄면 비활성 Navi 객체와 함께 그리는 probe도 매번 재생성된다.
    const canHoldAnchor = sync.canHoldAnchor;

    let ok;
    if (workerPipe) {
      /* 유지(hold) 판단도 워커가 한다. 메인은 상태만 넘긴다 — 그래야 계산이
       * 정말로 워커에서 일어난다. stage 에 dpr 을 실어 보내야 워커가 레이아웃을
       * 재지 않고도 논리 크기를 안다. */
      ok = workerPipe.render({
        nowMs,
        stage: stage ? { ...stage, devicePixelRatio: target.devicePixelRatio || 1 } : null,
        sync,
        clockDomain: timeline.domain,
        timelineDiscontinuity,
        canHoldAnchor,
        cameraOdometry: deviceOdometry,
        composeInput,
      });
    } else {
      if (timelineDiscontinuity) {
        anchorStore.reset();
        renderer.reset?.();
      }
      const composition = composeFrame(composeInput);
      const isProbe = composition.signs.length > 0
        && composition.signs.every((item) => item?.source === "calibrationProbe");
      const heldResult = anchorStore.update({
        nowMs,
        navi: composeInput.navi,
        probe: isProbe,
        candidates: composition.fresh,
        modelPosition,
        valid: isProbe || !composeInput.navi || sync.naviUsable,
        canHold: canHoldAnchor,
        precise: sync.canDrawPrecise,
        odometry: deviceOdometry,
      });
      ok = renderer.render({
        stage,
        sync,
        nowMs,
        clockDomain: timeline.domain,
        held: heldResult.state === AR_HOLD_STATE.HELD,
        modelPosition,
        egoSpeedMps,
        signs: heldResult.anchors || [],
      });
    }
    frames += 1;
    lastFrameAt = target.performance?.now?.() ?? Date.now();
    return ok;
  }

  /** presented-frame 신호 구독. 자체 rAF 를 돌리지 않는 것이 핵심이다. */
  function subscribeFrames() {
    const sync = options.frameSync || target.CarrotVisionFrameSync;
    const onPresented = (detail = {}) => {
      presentedSignals += 1;
      lastPresentedSource = detail?.source || null;
      // With a browser presentation clock, the next regular 30 Hz tick reads
      // this newly presented state. Drawing here as well would create uneven
      // 20/30 Hz intervals. Older clients without rAF remain frame-driven.
      if (!requestPresentationFrame) drawFrame(false);
    };
    if (typeof sync?.subscribePresented === "function") {
      frameSignalMode = "presented-channel";
      return sync.subscribePresented(onPresented);
    }
    // Compatibility path for a partially cached client. The canonical path is
    // the presented channel above; use the real event spelling here so AR can
    // still update until all generated assets have refreshed.
    frameSignalMode = "render-request-fallback";
    const handler = () => onPresented({ source: "render-request" });
    target.addEventListener?.("carrot:render-request", handler);
    return () => target.removeEventListener?.("carrot:render-request", handler);
  }

  function activate() {
    if (destroyed || active) return false;
    if (workerPipe?.status().broken) return false;
    const activity = options.activity || target.CarrotDriveDataActivity;
    if (activity?.acquire) {
      // cameraOdometry / livePose / carrotNavi 구독을 여는 임대.
      lease = activity.acquire({ owner: "vision_ar", ar: true });
    }
    active = true;
    activatedAt = target.performance?.now?.() ?? Date.now();
    presentedSignals = 0;
    lastPresentedSource = null;
    lastSubmitAt = null;
    unsubscribeFrame = subscribeFrames();
    drawFrame(true);
    schedulePresentationLoop();
    return true;
  }

  function deactivate() {
    if (destroyed || !active) return false;
    active = false;
    stopPresentationLoop();
    unsubscribeFrame?.();
    unsubscribeFrame = null;
    frameSignalMode = "none";
    anchorStore.reset();
    workerPipe?.reset();
    lease?.release?.();
    lease = null;
    if (workerPipe) workerPipe.render({ sync: null, signs: [] });
    else renderer.render({});   // 화면 지우기
    return true;
  }

  function resize() {
    return active ? drawFrame(true) : false;
  }

  /** 프로브 거리 변경. 투영이 실제로 동작하는지 확인하는 가장 빠른 방법이다.
   *  거리를 줄이면 표지가 커지면서 화면 아래로 내려와야 한다. */
  function setProbeDistance(meters) {
    const next = finite(meters, null);
    if (next === null || next <= 0) return false;
    probeDistanceM = next;
    // 이전 거리의 앵커가 남아 한 프레임 튀는 것을 막는다
    anchorStore.reset();
    workerPipe?.reset();
    drawFrame(true);
    return true;
  }

  /** 왜 안 그려지는지 한 줄로. 콘솔에서 바로 원인을 본다. */
  function diagnose() {
    const provider = options.provider || target.CarrotDriveLiveStateProvider;
    const stage = options.stage || target.CarrotVisionStageTransform;
    const snap = provider?.snapshot?.();
    const o = snap?.overlayState || {};
    const frameSync = options.frameSync || target.CarrotVisionFrameSync;
    const modelV2 = frameSync?.selectModel?.() || o.modelV2;
    const navi = o.carrotNavi || null;
    const naviStatus = navi?.navigationStatus || null;
    const replayStatus = target.CarrotVisionReplay?.status?.() || null;
    const workerStatus = workerPipe?.status() || null;
    const composition = workerStatus?.composition || null;
    const problems = [];
    const diagnoseNow = target.performance?.now?.() ?? Date.now();
    if (!active) problems.push("AR 런타임 비활성 (주행화면이 활성인지 확인)");
    if (active && diagnoseNow - activatedAt >= 1000 && presentedSignals === 0) {
      problems.push("presented-frame 신호가 들어오지 않음");
    }
    if (!provider) problems.push("CarrotDriveLiveStateProvider 없음");
    if (!stage) problems.push("CarrotVisionStageTransform 없음 (영상이 렌더 중인지 확인)");
    if (!modelV2) problems.push("modelV2 없음");
    else if (!Array.isArray(modelV2?.position?.x)) problems.push("modelV2.position.x 없음");
    if (!o.roadCameraState) problems.push("roadCameraState 없음");
    if (!o.liveCalibration) problems.push("liveCalibration 없음");
    if (lastSync && !lastSync.canDrawPrecise) problems.push("AR gate 차단: " + lastSync.reasons.join(", "));
    const drawn = workerPipe ? (workerStatus?.renderer?.drawn ?? null) : renderer.status().drawn;
    if (workerPipe && workerStatus?.broken) problems.push("AR 워커 오류: " + workerStatus.broken);
    if (workerPipe && !workerStatus?.ready) problems.push("AR 워커 준비 중");
    if (drawn === 0 && problems.length === 0) {
      if (composition?.signCount === 0) {
        problems.push("AR 마커 후보 없음");
      } else if (composition?.anchoredCount === 0) {
        problems.push("AR 마커는 있으나 model/geo 앵커를 만들지 못함");
      } else if (workerStatus?.renderer?.lastReason) {
        problems.push("AR 렌더러가 프레임을 건너뜀: " + workerStatus.renderer.lastReason);
      } else {
        problems.push("AR 마커와 앵커는 있으나 렌더러가 그리지 못함");
      }
    }
    return Object.freeze({
      ok: problems.length === 0,
      problems: Object.freeze(problems),
      calStatusRaw: o.liveCalibration?.calStatus ?? null,
      camFrameId: o.roadCameraState?.frameId ?? null,
      modelFrameId: modelV2?.frameId ?? null,
      pathLengthM: Array.isArray(modelV2?.position?.x) ? modelV2.position.x.at(-1) : null,
      drawn,
      composition,
      frameSignal: Object.freeze({
        mode: frameSignalMode,
        received: presentedSignals,
        lastSource: lastPresentedSource,
      }),
      replay: replayStatus,
      navi: Object.freeze({
        present: Boolean(navi),
        connected: navi?.connected === true,
        guidanceActive: naviStatus?.guidanceActive === true,
        offRoute: naviStatus?.offRoute === true,
        routePresent: naviStatus?.routePresent === true,
        usable: lastSync?.naviUsable ?? null,
        ageMs: lastSync?.naviAgeMs ?? null,
      }),
    });
  }

  function status() {
    const workerStatus = workerPipe?.status() || null;
    return Object.freeze({
      active,
      destroyed,
      frames,
      lastFrameAt,
      frameSignalMode,
      presentedSignals,
      lastPresentedSource,
      targetFps,
      degradedFps,
      currentFps: workerPipe
        ? Math.max(1, finite(workerStatus?.performance?.fps, targetFps))
        : targetFps,
      presentationClock: requestPresentationFrame ? "raf" : "presented-frame",
      presentationLoopActive: presentationFrameId !== null,
      leaseActive: Boolean(lease?.active),
      positionQuality: lastPositionQuality,
      timeline: lastTimeline,
      sync: lastSync
        ? Object.freeze({
          state: lastSync.state,
          canDrawPrecise: lastSync.canDrawPrecise,
          canHoldAnchor: lastSync.canHoldAnchor,
          calibrated: lastSync.calibrated,
          calibrationPlausible: lastSync.calibrationPlausible,
          frameIdGap: lastSync.frameIdGap,
          modelAgeMs: lastSync.modelAgeMs,
          odometryAgeMs: lastSync.odometryAgeMs,
          poseAgeMs: lastSync.poseAgeMs,
          calibrationAgeMs: lastSync.calibrationAgeMs,
          naviAgeMs: lastSync.naviAgeMs,
          naviUsable: lastSync.naviUsable,
          reasons: lastSync.reasons,
        })
        : null,
      // 워커 모드에서는 워커가 보고한 값을, 아니면 메인의 값을 그대로 쓴다.
      mode: workerPipe ? "worker" : "main",
      renderer: workerPipe ? workerStatus?.renderer : renderer.status(),
      worker: workerStatus,
      composition: workerStatus?.composition || null,
      probeEnabled: options.calibrationProbe === true,
      probeDistanceM,
      anchor: lastAnchor,
      hold: workerPipe ? workerStatus?.hold : anchorStore.status(),
    });
  }

  function destroy() {
    if (destroyed) return false;
    deactivate();
    destroyed = true;
    workerPipe?.destroy();
    renderer?.destroy();
    runtimeSingletons.delete(target);
    return true;
  }

  return Object.freeze({
    activate, deactivate, resize, status, diagnose, setProbeDistance,
    destroy, render: drawFrame,
  });
}

export function getOrCreateArRuntime(options = {}) {
  const target = options.target || globalThis;
  const existing = runtimeSingletons.get(target);
  if (existing) return existing;
  const runtime = createArRuntime({ ...options, target });
  if (runtime) runtimeSingletons.set(target, runtime);
  return runtime;
}

export function installArRuntimeFacade(target = globalThis, options = {}) {
  const runtime = getOrCreateArRuntime({ ...options, target });
  if (runtime) target.CarrotVisionAr = runtime;
  return runtime;
}
