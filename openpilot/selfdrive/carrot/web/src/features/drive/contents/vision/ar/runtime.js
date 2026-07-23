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
import { createMarkerIdentityTracker } from "./marker_identity.js";
import { createRouteMatcher } from "./route_matcher.js";
import { AR_POSITION_QUALITY, AR_RENDER } from "./tokens.js";
import { evaluateGeoPositionQuality } from "./position_quality.js";
import { odometryInDeviceFrame } from "./odometry.js";
import { createArTrace } from "./trace.js";
import {
  AR_CLOCK_DOMAIN,
  createArTimelineTracker,
} from "./timeline.js";
import {
  createCameraOdometryTimeline,
  createPresentedFrameClockMapper,
} from "./pose_timeline.js";
import { createArTrackingState } from "./tracking_state.js";
import { createDeviceWorldPose } from "./world_pose.js";
import {
  AR_COORDINATE_FRAME,
  AR_LIVE_POSE_FRAMES,
  deviceOdometryFrdToRouteFlu,
  modelPositionFrdToRouteFlu,
} from "./coordinate_frames.js";

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
function createWorkerPipe({ host, documentRoot, target, options, onBroken, onDrawn }) {
  if (typeof target.Worker !== "function") return null;
  if (typeof documentRoot.createElement !== "function") return null;

  const canvas = documentRoot.createElement("canvas");
  if (typeof canvas.transferControlToOffscreen !== "function") return null;
  canvas.className = "carrot-ar__canvas";
  canvas.style.cssText = "position:absolute;left:0;top:0;width:100%;height:100%;pointer-events:none;z-index:6;display:none";
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
  let lastViewport = { left: 0, top: 0, width: 0, height: 0 };
  const queueStats = {
    posted: 0,
    completed: 0,
    deferred: 0,
    replaced: 0,
    discarded: 0,
  };
  const sourceTransferStats = {
    navi: 0,
    modelPosition: 0,
    reusedNavi: 0,
    reusedModelPosition: 0,
  };
  const sourceState = {
    navi: { initialized: false, reference: null, revision: null },
    modelPosition: { initialized: false, reference: null, revision: null },
  };

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
      queueStats.completed += 1;
      lastStatus = data;
      onDrawn?.(data);
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
      setCanvasDiagnostic("arTracking", data.tracking?.state || null);
      setCanvasDiagnostic("arTrackingAlpha", data.tracking?.alpha ?? null);
      setCanvasDiagnostic("arTrackingUncertaintyM", data.tracking?.uncertainty?.lateralM ?? null);
      setCanvasDiagnostic("arWorldPoseInitialized", data.worldPose?.initialized === true);
      setCanvasDiagnostic("arWorldPoseIntegrations", data.worldPose?.integrations ?? null);
      setCanvasDiagnostic("arWorldPoseEpoch", data.worldPose?.epoch || null);
      // arReason은 기존 원격 점검 도구와의 호환용 요약이고, 위 필드들이
      // 실제 원인별 source of truth다.
      setCanvasDiagnostic("arReason", renderReason || lastSyncReason || holdReason || null);
      setCanvasDiagnostic("arSignCount", data.composition?.signCount ?? 0);
      setCanvasDiagnostic("arComposedAnchorCount", data.composition?.anchoredCount ?? 0);
      setCanvasDiagnostic("arQueuePending", pending ? 1 : 0);
      setCanvasDiagnostic("arQueueReplaced", queueStats.replaced);
      if (pending) { const next = pending; pending = null; post(next); }
    }
  };
  worker.onerror = (event) => { failClosed(event?.message || "worker failed"); };

  function sourceChanged(name, value, revision) {
    const state = sourceState[name];
    const hasRevision = revision !== null && revision !== undefined && revision !== "";
    const normalizedRevision = hasRevision ? String(revision) : null;
    const changed = !state.initialized || (hasRevision
      ? state.revision !== normalizedRevision
      : state.reference !== value);
    if (!changed) return false;
    state.initialized = true;
    state.reference = value;
    state.revision = normalizedRevision;
    return true;
  }

  function compactPayload(payload = {}) {
    const rawComposeInput = payload.composeInput || {};
    const revisions = payload.sourceRevisions || {};
    const navi = rawComposeInput.navi ?? null;
    const modelPosition = rawComposeInput.modelPosition ?? null;
    const sourceUpdate = {};
    if (sourceChanged("navi", navi, revisions.navi)) {
      sourceUpdate.navi = navi;
      sourceTransferStats.navi += 1;
    } else {
      sourceTransferStats.reusedNavi += 1;
    }
    if (sourceChanged("modelPosition", modelPosition, revisions.modelPosition)) {
      sourceUpdate.modelPosition = modelPosition;
      sourceTransferStats.modelPosition += 1;
    } else {
      sourceTransferStats.reusedModelPosition += 1;
    }
    const composeInput = { ...rawComposeInput };
    delete composeInput.navi;
    delete composeInput.modelPosition;
    const { sourceRevisions: _sourceRevisions, ...rest } = payload;
    return {
      ...rest,
      composeInput,
      ...(Object.keys(sourceUpdate).length ? { sourceUpdate } : {}),
    };
  }

  function post(payload) {
    if (!ready || broken) return false;
    if (inFlight) {
      queueStats.deferred += 1;
      if (pending) queueStats.replaced += 1;
      pending = payload;
      setCanvasDiagnostic("arQueuePending", 1);
      setCanvasDiagnostic("arQueueReplaced", queueStats.replaced);
      return true;
    }
    inFlight = true;
    queueStats.posted += 1;
    worker.postMessage({ type: "frame", payload: compactPayload(payload) });
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
  function syncViewport(stage) {
    const width = Math.max(1, finite(stage?.stageWidth, 0));
    const height = Math.max(1, finite(stage?.stageHeight, 0));
    if (!(width > 1 && height > 1)) return false;
    const left = finite(stage?.viewportLeft, 0);
    const top = finite(stage?.viewportTop, 0);
    const next = { left, top, width, height };
    if (
      next.left === lastViewport.left
      && next.top === lastViewport.top
      && next.width === lastViewport.width
      && next.height === lastViewport.height
    ) return false;
    lastViewport = next;
    canvas.style.left = `${left}px`;
    canvas.style.top = `${top}px`;
    canvas.style.width = `${width}px`;
    canvas.style.height = `${height}px`;
    setCanvasDiagnostic("arViewport", `${left},${top},${width}x${height}`);
    return true;
  }

  function syncSize(stage) {
    const rect = canvas.getBoundingClientRect?.() || {};
    const rawDpr = Math.max(1, finite(target.devicePixelRatio, 1));
    const degraded = lastStatus?.performance?.level === "degraded";
    const dprLimit = degraded
      ? AR_RENDER.degradedDevicePixelRatioMax
      : AR_RENDER.devicePixelRatioMax;
    const dpr = Math.min(rawDpr, Math.max(1, finite(dprLimit, 1)));
    const cssWidth = finite(stage?.stageWidth, rect.width || 1);
    const cssHeight = finite(stage?.stageHeight, rect.height || 1);
    const pixelWidth = Math.max(1, Math.round(cssWidth * dpr));
    const pixelHeight = Math.max(1, Math.round(cssHeight * dpr));
    if (pixelWidth !== lastSize.pixelWidth || pixelHeight !== lastSize.pixelHeight) {
      lastSize = { pixelWidth, pixelHeight, dpr };
      worker.postMessage({ type: "resize", pixelWidth, pixelHeight });
    }
    return lastSize;
  }

  return Object.freeze({
    canvas,
    render(payload) {
      syncViewport(payload?.stage);
      const renderSize = syncSize(payload?.stage);
      lastSyncReason = String(payload?.sync?.reasons?.[0] || "");
      setCanvasDiagnostic("arSync", payload?.sync?.state || "missing");
      setCanvasDiagnostic("arCanDraw", payload?.sync?.canDrawPrecise === true);
      setCanvasDiagnostic("arClockDomain", payload?.clockDomain || null);
      setCanvasDiagnostic("arPresentedClock", payload?.presentedClock?.domain || null);
      setCanvasDiagnostic("arClockConfidence", payload?.presentedClock?.confidence || "unmapped");
      setCanvasDiagnostic("arTracking", payload?.tracking?.state || null);
      setCanvasDiagnostic("arTrackingAlpha", payload?.tracking?.alpha ?? null);
      setCanvasDiagnostic("arWorldPoseEpoch", payload?.worldPoseEpoch || null);
      setCanvasDiagnostic("arSyncReason", lastSyncReason || null);
      setCanvasDiagnostic("arReason", lastSyncReason || null);
      const normalizedPayload = payload?.stage
        ? { ...payload, stage: { ...payload.stage, devicePixelRatio: renderSize.dpr } }
        : payload;
      return post(normalizedPayload);
    },
    reset() {
      if (pending) queueStats.discarded += 1;
      pending = null;
      for (const state of Object.values(sourceState)) {
        state.initialized = false;
        state.reference = null;
        state.revision = null;
      }
      worker.postMessage({ type: "reset" });
    },
    status() {
      return Object.freeze({
        mode: "worker", ready, broken: broken || null,
        composition: lastStatus?.composition || null,
        hold: lastStatus?.hold || null,
        renderer: lastStatus?.renderer || null,
        performance: lastStatus?.performance || null,
        canvas: `${lastSize.pixelWidth}x${lastSize.pixelHeight}`,
        dpr: lastSize.dpr || 1,
        queue: Object.freeze({
          ...queueStats,
          inFlight,
          pending: Boolean(pending),
          maxPending: 1,
        }),
        sourceTransfers: Object.freeze({ ...sourceTransferStats }),
        viewport: Object.freeze({ ...lastViewport }),
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
  if (v === null || v === undefined || v === "") return fallback;
  const n = Number(v);
  return Number.isFinite(n) ? n : fallback;
}

function diagnosticGuidanceSnapshot(item) {
  if (!item) return null;
  return Object.freeze({
    present: item.present !== false,
    distanceM: finite(item.distanceM),
    turnType: finite(item.turnType),
    roadName: String(item.roadName || ""),
    latitude: finite(item.latitude),
    longitude: finite(item.longitude),
    pointValid: item.pointValid === true,
  });
}

function diagnosticNaviSnapshot(navi) {
  const status = navi?.navigationStatus || null;
  const vehicle = navi?.vehicle || null;
  const route = navi?.route || null;
  const polyline = Array.isArray(route?.polyline) ? route.polyline : [];
  return Object.freeze({
    present: Boolean(navi),
    sessionId: String(navi?.sessionId || ""),
    connected: navi?.connected === true,
    guidanceActive: status?.guidanceActive === true,
    offRoute: status?.offRoute === true,
    routePresent: status?.routePresent === true || route?.present === true,
    vehicle: Object.freeze({
      present: vehicle?.present !== false && Boolean(vehicle),
      latitude: finite(vehicle?.latitude),
      longitude: finite(vehicle?.longitude),
      headingDeg: finite(vehicle?.headingDeg),
      speedKph: finite(vehicle?.speedKph),
    }),
    route: Object.freeze({
      present: route?.present === true,
      pointCount: polyline.length,
      totalDistanceM: finite(route?.totalDistanceM),
      remainingDistanceM: finite(route?.remainingDistanceM),
      movedDistanceM: finite(route?.movedDistanceM),
    }),
    guidanceCurrent: diagnosticGuidanceSnapshot(navi?.guidanceCurrent),
    guidanceNext: diagnosticGuidanceSnapshot(navi?.guidanceNext),
    publishMonoTimeNanos: finite(navi?.publishMonoTimeNanos),
  });
}

function diagnosticSyncSnapshot(sync) {
  if (!sync) return null;
  return Object.freeze({
    state: sync.state,
    canDrawPrecise: sync.canDrawPrecise === true,
    canHoldAnchor: sync.canHoldAnchor === true,
    frameIdGap: finite(sync.frameIdGap),
    modelAgeMs: finite(sync.modelAgeMs),
    odometryAgeMs: finite(sync.odometryAgeMs),
    poseAgeMs: finite(sync.poseAgeMs),
    calibrationAgeMs: finite(sync.calibrationAgeMs),
    naviAgeMs: finite(sync.naviAgeMs),
    naviUsable: sync.naviUsable === true,
    presentedClockConfidence: String(sync.presentedClockConfidence || "unmapped"),
    presentedTargetTimestampNs: finite(sync.presentedTargetTimestampNs),
    reasons: Object.freeze([...(sync.reasons || [])].map(String)),
  });
}

function diagnosticPositionSnapshot(quality) {
  if (!quality) return null;
  return Object.freeze({
    canUseGeo: quality.canUseGeo === true,
    canUseRoute: quality.canUseRoute === true,
    fallback: String(quality.fallback || ""),
    positionSigmaM: finite(quality.positionSigmaM),
    headingSigmaDeg: finite(quality.headingSigmaDeg),
    separationM: finite(quality.separationM),
    reasons: Object.freeze([...(quality.reasons || [])].map(String)),
    routeReasons: Object.freeze([...(quality.routeReasons || [])].map(String)),
  });
}

function trackingDistanceM(navi, probeEnabled, probeDistanceM) {
  const distances = [];
  if (probeEnabled) distances.push(finite(probeDistanceM, 40));
  for (const item of [navi?.guidanceCurrent, navi?.guidanceNext]) {
    if (item?.present === false) continue;
    const distance = finite(item?.distanceM, null);
    if (distance !== null && distance >= 0) distances.push(distance);
  }
  const speedDistance = finite(navi?.speed?.sdiDistanceM, null);
  if (speedDistance !== null && speedDistance >= 0) distances.push(speedDistance);
  return distances.length ? Math.min(300, Math.max(...distances)) : 40;
}

export function geographicWorldObservation(navi, gps, quality) {
  const vehicle = navi?.vehicle;
  const latitude = finite(vehicle?.latitude);
  const longitude = finite(vehicle?.longitude);
  const headingDeg = finite(vehicle?.headingDeg);
  const publishMonoTimeNanos = finite(navi?.publishMonoTimeNanos, 0);
  const gpsTimestampMs = finite(gps?.unixTimestampMillis, 0);
  const key = publishMonoTimeNanos > 0
    ? `navi:${publishMonoTimeNanos}`
    : `legacy:${latitude}|${longitude}|${headingDeg}|${gpsTimestampMs}`;
  // CarrotNavi's vehicle fix, route and event points are one map-matched
  // coordinate source. Even when Comma GPS cannot approve an absolute geo
  // fix, a fresh Navi route can safely bound local world-pose drift. Use a
  // conservative covariance in that route-only mode; large map jumps still
  // fail the world-pose innovation gate.
  const absoluteGeo = quality?.canUseGeo === true;
  const routeRelative = !absoluteGeo && quality?.canUseRoute === true;
  const positionSigmaM = absoluteGeo
    ? finite(quality?.positionSigmaM)
    : routeRelative ? AR_POSITION_QUALITY.routeWorldPositionSigmaM : null;
  const headingSigmaDeg = absoluteGeo
    ? finite(quality?.headingSigmaDeg)
    : routeRelative ? AR_POSITION_QUALITY.routeWorldHeadingSigmaDeg : null;
  return Object.freeze({
    key,
    valid: absoluteGeo || routeRelative,
    latitude,
    longitude,
    headingDeg,
    positionSigmaM,
    headingSigmaDeg,
    yawUsable: headingDeg !== null && headingSigmaDeg !== null,
    source: absoluteGeo ? "tmap-comma-verified" : routeRelative ? "tmap-route" : "unavailable",
    sourceTimestampNs: publishMonoTimeNanos > 0 ? publishMonoTimeNanos : null,
    reasons: absoluteGeo ? [] : quality?.routeReasons || quality?.reasons || [],
  });
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
    const sourceEpoch = String(
      replay?.videoSourceKey?.()
      || replayStatus?.segment
      || replayStatus?.route
      || "replay",
    );
    return Object.freeze({
      domain: AR_CLOCK_DOMAIN.REPLAY_MEDIA,
      nowMs: replaySeconds * 1000,
      epoch: sourceEpoch,
    });
  }
  return Object.freeze({
    domain: AR_CLOCK_DOMAIN.LIVE_MONOTONIC,
    nowMs: Math.max(0, finite(monotonicNowMs, 0)),
    epoch: "live",
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

  const trace = createArTrace({
    enabled: options.traceEnabled === true,
    capacity: options.traceCapacity,
    now: () => target.performance?.now?.() ?? Date.now(),
  });
  const diagnosticProbeEnabled = options.diagnosticProbe === true;
  // May be a boolean or a live source (function); the store resolves it.
  const bypassWorldAnchor = options.bypassWorldAnchor ?? false;
  let workerFailureHandler = () => {};
  let workerDrawHandler = () => {};
  const diagnosticListeners = new Set();
  /* 제품 renderer는 Worker Three.js만 사용한다. renderer injection은 정적 테스트 seam이다. */
  const workerPipe = options.renderer ? null : createWorkerPipe({
    host,
    documentRoot,
    target,
    options,
    onBroken: (reason) => workerFailureHandler(reason),
    onDrawn: (data) => workerDrawHandler(data),
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
  let lastPresentedDetail = null;
  let presentedSubmissions = 0;
  let lastSubmittedPresentedSequence = null;
  const targetFps = Math.max(1, finite(options.targetFps, AR_RENDER.targetFps));
  const degradedFps = Math.min(
    targetFps,
    Math.max(1, finite(options.degradedFps, AR_RENDER.degradedFps)),
  );
  let probeDistanceM = finite(options.probeDistanceM, 40);
  let lastAnchor = null;
  let lastComposition = null;
  let lastNavigationSession = "";
  let lastTimeline = null;
  let lastPresentedClock = null;
  let lastTracking = null;
  let lastWorldPose = null;
  let lastWorldEpoch = null;
  let lastSpatialNowMs = null;
  let lastModelPositionSource = null;
  let lastModelPosition = null;
  let pendingTimelineDiscontinuity = null;
  const timelineTracker = options.timelineTracker || createArTimelineTracker();
  const presentedClockMapper = options.presentedClockMapper || createPresentedFrameClockMapper();
  const odometryTimeline = options.odometryTimeline || createCameraOdometryTimeline();
  const trackingTracker = options.trackingTracker || createArTrackingState({
    limits: options.trackingLimits,
  });
  const worldPoseTracker = options.worldPoseTracker || createDeviceWorldPose({
    limits: options.worldPoseLimits,
  });
  const anchorStore = options.anchorStore
    || createContinuousAnchorStore({ limits: options.holdLimits, bypassWorldAnchor });
  const identityTracker = options.identityTracker || createMarkerIdentityTracker();
  const routeMatcher = options.routeMatcher || createRouteMatcher();

  function publishDiagnosticFrame(frame) {
    for (const listener of [...diagnosticListeners]) {
      try { listener(frame); } catch (error) {
        target.console?.error?.("[carrot AR debug] frame listener failed", error);
      }
    }
  }

  function subscribeDiagnostics(listener) {
    if (typeof listener !== "function") {
      throw new TypeError("AR diagnostic listener must be a function");
    }
    diagnosticListeners.add(listener);
    let subscribed = true;
    return () => {
      if (!subscribed) return false;
      subscribed = false;
      return diagnosticListeners.delete(listener);
    };
  }

  workerDrawHandler = (data) => {
    if (data?.worldPose) lastWorldPose = data.worldPose;
    const workerStatus = workerPipe?.status?.() || null;
    const transport = Object.freeze({
      dpr: workerStatus?.dpr ?? null,
      queue: workerStatus?.queue || null,
      sourceTransfers: workerStatus?.sourceTransfers || null,
    });
    const diagnosticData = data ? Object.freeze({ ...data, transport }) : data;
    publishDiagnosticFrame(diagnosticData);
    if (trace.isEnabled()) {
      trace.record("worker-drawn", {
        traceFrameId: data?.traceFrameId ?? null,
        ok: data?.ok === true,
        composition: data?.composition || null,
        hold: data?.hold || null,
        worldPose: data?.worldPose || null,
        renderer: data?.renderer || null,
        performance: data?.performance || null,
        transport,
      });
    }
  };

  workerFailureHandler = (reason) => {
    if (!active || destroyed) return;
    trace.record("worker-failure", { reason: String(reason || "worker failed") });
    active = false;
    unsubscribeFrame?.();
    unsubscribeFrame = null;
    frameSignalMode = "none";
    anchorStore.reset();
    identityTracker.reset();
    routeMatcher.reset();
    lease?.release?.();
    lease = null;
  };

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

    const timelineSample = resolveArTimeline(target, frameNowMs);
    const timeline = timelineTracker.update({
      ...timelineSample,
      explicitDiscontinuity: pendingTimelineDiscontinuity !== null,
      discontinuityReason: pendingTimelineDiscontinuity?.reason,
    });
    pendingTimelineDiscontinuity = null;
    if (timeline.discontinuity) {
      presentedClockMapper.reset();
      odometryTimeline.reset();
      lastSpatialNowMs = null;
    }
    lastTimeline = timeline;
    /* 같은 tick의 cereal 카메라 시각을 함께 넘긴다. replay 프레임 metadata에는
     * cereal timestamp가 없으므로 media↔cereal offset을 세울 근거는 이것뿐이다.
     *
     * roadCameraState가 1순위지만 replay는 그 서비스를 frameId/sensor만 실어
     * 보낸다(timestampEof 없음). cameraOdometry.timestampEof가 같은 카메라
     * 프레임의 end-of-frame 시각이고 replay에서도 항상 들어오므로 대체로 쓴다. */
    const presentedClock = presentedClockMapper.map(lastPresentedDetail, timeline, {
      cameraTimestampEof: overlayState.roadCameraState?.timestampEof
        ?? overlayState.cameraOdometry?.timestampEof
        ?? null,
    });
    lastPresentedClock = presentedClock;

    // cameraOdometry: Calibrated FRD -> Device FRD -> route FLU, exactly once.
    // timestampEof itself is not the observation time: locationd assigns the
    // model pose 100ms earlier, and the timeline resolves that delayed sample
    // against the frame actually presented by the browser.
    const deviceOdometry = odometryInDeviceFrame(
      overlayState.cameraOdometry,
      overlayState.liveCalibration?.rpyCalib,
    );
    odometryTimeline.push(deviceOdometry);
    const presentedDeviceOdometry = odometryTimeline.sampleAt(presentedClock.targetTimestampNs);
    const routeOdometry = deviceOdometryFrdToRouteFlu(presentedDeviceOdometry);
    const spatialClockReady = presentedClock.targetTimestampNs !== null
      && presentedClock.targetTimestampNs > 0;
    if (spatialClockReady) lastSpatialNowMs = presentedClock.targetTimeMs;
    const inputSync = evaluateFrameSync({
      roadCameraState: overlayState.roadCameraState,
      modelV2,
      cameraOdometry: overlayState.cameraOdometry,
      livePose: overlayState.livePose,
      liveCalibration: overlayState.liveCalibration,
      carrotNavi: overlayState.carrotNavi,
      nowMs: timeline.nowMs,
      clockDomain: timeline.domain,
      presentedClock,
      receivedAtMonotonic: snapshot?.receivedAtMonotonic,
    });
    const positionQuality = evaluateGeoPositionQuality({
      nowMs: timeline.nowMs,
      clockDomain: timeline.domain,
      receivedAtMonotonic: snapshot?.receivedAtMonotonic,
      naviUsable: inputSync.naviUsable,
      naviVehicle: overlayState.carrotNavi?.vehicle,
      naviRoutePolyline: overlayState.carrotNavi?.route?.polyline,
      gpsLocationExternal: hudState.gpsLocationExternal,
    });
    lastPositionQuality = positionQuality;
    const geographicObservation = geographicWorldObservation(
      overlayState.carrotNavi,
      hudState.gpsLocationExternal,
      positionQuality,
    );
    const sync = inputSync;
    lastSync = sync;

    const egoSpeedMps = finite(hudState.carState?.vEgo, 0);
    const tracking = trackingTracker.update({
      nowMs: timeline.nowMs,
      /* 추적 샘플의 정체성은 "카메라 프레임"이다. presentation sequence는 30Hz UI
       * tick마다 증가하는 카운터라, 같은 영상 프레임을 새 관측으로 오인하게 만든다.
       * frameId → spatial target 시각 → (둘 다 없을 때만) sequence 순으로 본다. */
      sampleId: presentedClock.targetTimestampNs
        ?? presentedClock.sourceFrameId
        ?? lastPresentedDetail?.sequence,
      presentedTimestampNs: presentedClock.targetTimestampNs,
      spatialClockReady,
      sync,
      odometry: routeOdometry,
      egoSpeedMps,
      anchorDistanceM: trackingDistanceM(
        overlayState.carrotNavi,
        diagnosticProbeEnabled,
        probeDistanceM,
      ),
      discontinuity: timeline.discontinuity,
      discontinuityReason: timeline.discontinuityReason,
    });
    lastTracking = tracking;

    /* navi 스냅샷은 provider 특성상 프레임마다 찼다 비었다 한다. 비는 프레임을
     * "새 내비 세션"으로 보면 world epoch가 매 프레임 뒤집혀 world pose 적분과
     * anchor store가 계속 리셋된다(적분 0회, hold 즉시 dropped). 마지막으로
     * 실제 관측한 sessionId를 유지하고, 진짜 다른 세션이 올 때만 교체한다. */
    const observedSession = String(overlayState.carrotNavi?.sessionId || "");
    if (observedSession) lastNavigationSession = observedSession;
    const navigationSession = lastNavigationSession || "no-navigation-session";
    const worldEpoch = `${timeline.domain}|${timeline.epoch}|${navigationSession}`;

    // The renderer/anchor store owns one explicit route-local FLU space.
    // Source messages keep their openpilot FRD definitions until this seam.
    const modelPositionSource = modelV2?.position || null;
    if (modelPositionSource !== lastModelPositionSource) {
      lastModelPositionSource = modelPositionSource;
      lastModelPosition = modelPositionFrdToRouteFlu(modelPositionSource);
    }
    const modelPosition = lastModelPosition;
    const posX = modelPosition?.x;
    const posY = modelPosition?.y;
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

    // Once a cereal/video mapping has created anchors, an unmapped frame must
    // freeze that spatial clock instead of jumping back to performance/media
    // time and looking like a seek.
    const nowMs = lastSpatialNowMs ?? timeline.nowMs;
    /* 순수 입력만 만든다. 워커 모드에서는 마커 선택·위경도 변환·경로 배치까지
     * 모두 워커가 composeFrame()으로 처리한다. 메인 폴백도 같은 함수를 써서
     * 두 경로의 결과가 갈라지지 않는다. */
    const composeInput = {
      navi: overlayState.carrotNavi || null,
      naviUsable: sync.naviUsable,
      laneWidthM: finite(overlayState.lateralPlan?.laneWidth, 3.5),
      egoSpeedMps,
      calibrationProbe: diagnosticProbeEnabled,
      probeDistanceM,
      canDrawPrecise: tracking.canCreateAnchor,
      geoAllowed: positionQuality.canUseGeo,
      routeAllowed: positionQuality.canUseRoute,
      routePositionSigmaM: positionQuality.positionSigmaM,
      modelPosition,
    };
    // 실제 Navi의 off-route 여부는 worker/store의 valid 입력이 맡는다. 여기서
    // canHold까지 끄면 비활성 Navi 객체와 함께 그리는 probe도 매번 재생성된다.
    const canHoldAnchor = tracking.canPropagateAnchor
      && spatialClockReady
      && Boolean(routeOdometry);
    const traceFrameId = frames + 1;
    const diagnosticsEnabled = diagnosticListeners.size > 0;
    const debugFrame = diagnosticsEnabled ? Object.freeze({
      replayTimeMs: timeline.domain === AR_CLOCK_DOMAIN.REPLAY_MEDIA ? timeline.nowMs : null,
      timeline: Object.freeze({
        domain: timeline.domain,
        nowMs: timeline.nowMs,
        epoch: timeline.epoch,
        deltaMs: timeline.deltaMs,
        discontinuity: timeline.discontinuity,
        discontinuityReason: timeline.discontinuityReason,
      }),
      presented: lastPresentedDetail,
      presentedClock,
      sync: diagnosticSyncSnapshot(sync),
      positionQuality: diagnosticPositionSnapshot(positionQuality),
      navi: diagnosticNaviSnapshot(overlayState.carrotNavi),
      sources: Object.freeze({
        cameraFrameId: finite(overlayState.roadCameraState?.frameId),
        modelFrameId: finite(modelV2?.frameId),
        odometryFrameId: finite(overlayState.cameraOdometry?.frameId),
        livePoseTimestamp: finite(overlayState.livePose?.timestamp),
      }),
      worldEpoch,
    }) : null;
    if (trace.isEnabled()) {
      trace.record("frame-submit", {
        traceFrameId,
        force: force === true,
        presented: lastPresentedDetail,
        timeline: {
          domain: timeline.domain,
          nowMs: timeline.nowMs,
          epoch: timeline.epoch,
          deltaMs: timeline.deltaMs,
          discontinuity: timeline.discontinuity,
          discontinuityReason: timeline.discontinuityReason,
        },
        presentedClock,
        stage: stage ? {
          width: stage.stageWidth ?? null,
          height: stage.stageHeight ?? null,
          viewportLeft: stage.viewportLeft ?? null,
          viewportTop: stage.viewportTop ?? null,
        } : null,
        sources: {
          cameraFrameId: overlayState.roadCameraState?.frameId ?? null,
          cameraTimestampEof: overlayState.roadCameraState?.timestampEof ?? null,
          modelFrameId: modelV2?.frameId ?? null,
          modelFrameAge: modelV2?.frameAge ?? null,
          odometryFrameId: overlayState.cameraOdometry?.frameId ?? null,
          odometryTimestampEof: overlayState.cameraOdometry?.timestampEof ?? null,
          odometryObservationTimestampNs: routeOdometry?.observationTimestampNs ?? null,
          odometryAlignment: routeOdometry?.temporalAlignment || null,
          livePoseTimestamp: overlayState.livePose?.timestamp ?? null,
          naviPublishMonoTimeNanos: overlayState.carrotNavi?.publishMonoTimeNanos ?? null,
          geographicPositionSigmaM: positionQuality.positionSigmaM,
          geographicHeadingSigmaDeg: positionQuality.headingSigmaDeg,
          coordinateFrames: {
            modelPosition: modelPosition?.coordinateFrame ?? null,
            cameraOdometry: routeOdometry?.coordinateFrame ?? null,
            cameraOdometryDevice: presentedDeviceOdometry?.coordinateFrame ?? null,
            worldPose: lastWorldPose?.worldCoordinateFrame
              ?? AR_COORDINATE_FRAME.LOCAL_WORLD_DEVICE,
            livePose: AR_LIVE_POSE_FRAMES,
            tmapRoute: AR_COORDINATE_FRAME.ROUTE_FLU,
          },
        },
        sync: {
          state: sync.state,
          canDrawPrecise: sync.canDrawPrecise,
          canHoldAnchor: sync.canHoldAnchor,
          naviUsable: sync.naviUsable,
          reasons: sync.reasons,
        },
        tracking,
        worldPoseEpoch: worldEpoch,
        probe: diagnosticProbeEnabled,
      }, frameNowMs);
    }

    let ok;
    if (workerPipe) {
      /* 유지(hold) 판단도 워커가 한다. 메인은 상태만 넘긴다 — 그래야 계산이
       * 정말로 워커에서 일어난다. stage 에 dpr 을 실어 보내야 워커가 레이아웃을
       * 재지 않고도 논리 크기를 안다. */
      ok = workerPipe.render({
        nowMs,
        // Spatial time freezes when a presented frame cannot be mapped, while
        // visual confidence/fade still follows the live or replay presentation
        // timeline. Keeping both clocks prevents pose jumps and frozen fades.
        presentationNowMs: timeline.nowMs,
        stage: stage ? { ...stage, devicePixelRatio: target.devicePixelRatio || 1 } : null,
        sync,
        tracking,
        deviceOdometry: presentedDeviceOdometry,
        livePose: overlayState.livePose,
        geographicObservation,
        worldPoseEpoch: worldEpoch,
        worldPoseTargetTimestampNs: presentedClock.targetTimestampNs,
        clockDomain: timeline.domain,
        presentedClock,
        timelineDiscontinuity: timeline.discontinuity,
        timelineDiscontinuityReason: timeline.discontinuityReason,
        canHoldAnchor,
        routeOdometry,
        traceFrameId,
        debugFrame,
        diagnosticsEnabled,
        composeInput,
        sourceRevisions: {
          navi: overlayState.carrotNavi?.publishMonoTimeNanos ?? null,
          modelPosition: modelV2?.frameId ?? null,
        },
      });
    } else {
      if (timeline.discontinuity || lastWorldEpoch !== worldEpoch) {
        worldPoseTracker.reset({
          epoch: worldEpoch,
          reason: timeline.discontinuity
            ? timeline.discontinuityReason || "timeline discontinuity"
            : "navigation/session epoch initialized",
        });
        lastWorldEpoch = worldEpoch;
      }
      lastWorldPose = presentedDeviceOdometry
        ? worldPoseTracker.update({
          timestampNs: presentedClock.targetTimestampNs,
          odometry: presentedDeviceOdometry,
          livePose: overlayState.livePose,
          geographicObservation,
          trackingState: tracking.state,
        })
        : worldPoseTracker.status();
      if (timeline.discontinuity) {
        anchorStore.reset();
        identityTracker.reset();
        routeMatcher.reset();
        renderer.reset?.();
      }
      const composition = composeFrame(composeInput, { identityTracker, routeMatcher });
      lastComposition = Object.freeze({
        signCount: composition.signs.length,
        anchoredCount: composition.fresh?.length || 0,
        sources: Object.freeze(composition.signs.map((item) => item.source || "unknown")),
        diag: composition.diag || null,
      });
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
        precise: tracking.canCreateAnchor,
        trackingState: tracking.state,
        trackingRecovered: tracking.recovered,
        retainAnchor: tracking.retainAnchor,
        trackingUncertaintyM: tracking.uncertainty?.lateralM,
        reason: tracking.reasons[0] || "",
        odometry: routeOdometry,
        worldPose: lastWorldPose,
      });
      ok = renderer.render({
        stage,
        sync,
        tracking,
        nowMs: timeline.nowMs,
        clockDomain: timeline.domain,
        held: heldResult.state === AR_HOLD_STATE.HELD,
        modelPosition,
        egoSpeedMps,
        signs: heldResult.anchors || [],
        diagnosticsEnabled,
      });
      publishDiagnosticFrame(Object.freeze({
        type: "drawn",
        traceFrameId,
        ok: ok === true,
        debugFrame,
        composition: lastComposition,
        hold: anchorStore.status(nowMs),
        tracking,
        worldPose: lastWorldPose,
        renderer: renderer.status?.() || null,
        performance: null,
      }));
      if (trace.isEnabled()) {
        trace.record("main-drawn", {
          traceFrameId,
          ok: ok === true,
          composition: {
            signCount: composition.signs.length,
            anchoredCount: composition.fresh?.length || 0,
            sources: composition.signs.map((item) => item.source || "unknown"),
          },
          hold: anchorStore.status(nowMs),
          renderer: renderer.status?.() || null,
        });
      }
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
      const metadata = detail?.metadata || {};
      if (lastPresentedSource === "replay" && metadata.discontinuity === true) {
        pendingTimelineDiscontinuity = Object.freeze({
          sequence: detail?.sequence ?? null,
          reason: String(metadata.discontinuityReason || "replay-seek"),
        });
      }
      lastPresentedDetail = Object.freeze({
        source: lastPresentedSource,
        sequence: detail?.sequence ?? null,
        reason: detail?.reason || null,
        mediaTime: metadata.mediaTime ?? null,
        expectedDisplayTime: metadata.expectedDisplayTime ?? null,
        presentedFrames: metadata.presentedFrames ?? null,
        rtpTimestamp: metadata.rtpTimestamp ?? null,
        captureTime: metadata.captureTime ?? null,
        receiveTime: metadata.receiveTime ?? null,
        frameId: detail?.frameId ?? null,
        cameraTimestampEof: detail?.cameraTimestampEof ?? null,
        clockMappingConfidence: detail?.clockMappingConfidence || "unmapped",
        discontinuity: metadata.discontinuity === true,
        discontinuityReason: metadata.discontinuityReason || null,
      });
      trace.record("presented-frame", lastPresentedDetail);
      // Spatial presentation is locked to the camera frame. A separate rAF
      // loop would move a world marker over a frozen 20 Hz image and make it
      // look vehicle-following. Lifecycle and pose therefore advance exactly
      // once for each actually presented live/replay frame (subject only to
      // the worker performance cadence).
      if (drawFrame(false)) {
        presentedSubmissions += 1;
        lastSubmittedPresentedSequence = lastPresentedDetail.sequence;
      }
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
    lastPresentedDetail = null;
    presentedSubmissions = 0;
    lastSubmittedPresentedSequence = null;
    lastSubmitAt = null;
    lastTimeline = null;
    lastPresentedClock = null;
    lastSpatialNowMs = null;
    lastModelPositionSource = null;
    lastModelPosition = null;
    pendingTimelineDiscontinuity = null;
    timelineTracker.reset();
    presentedClockMapper.reset();
    odometryTimeline.reset();
    trackingTracker.reset("activate");
    routeMatcher.reset();
    lastTracking = null;
    worldPoseTracker.reset({ epoch: "uninitialized", reason: "activate" });
    lastNavigationSession = "";
    lastWorldPose = null;
    lastWorldEpoch = null;
    unsubscribeFrame = subscribeFrames();
    trace.record("activate", { frameSignalMode, diagnosticProbeEnabled });
    drawFrame(true);
    // The activation paint has no camera presentation identity. It must not
    // throttle the first real rVFC/replay frame arriving immediately after it.
    lastSubmitAt = null;
    return true;
  }

  function deactivate() {
    if (destroyed || !active) return false;
    trace.record("deactivate", { frames, presentedSignals });
    active = false;
    unsubscribeFrame?.();
    unsubscribeFrame = null;
    frameSignalMode = "none";
    anchorStore.reset();
    identityTracker.reset();
    routeMatcher.reset();
    workerPipe?.reset();
    lastTimeline = null;
    lastPresentedClock = null;
    lastSpatialNowMs = null;
    lastModelPositionSource = null;
    lastModelPosition = null;
    pendingTimelineDiscontinuity = null;
    timelineTracker.reset();
    presentedClockMapper.reset();
    odometryTimeline.reset();
    trackingTracker.reset("deactivate");
    lastTracking = null;
    worldPoseTracker.reset({ epoch: "uninitialized", reason: "deactivate" });
    lastNavigationSession = "";
    lastWorldPose = null;
    lastWorldEpoch = null;
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
    if (!diagnosticProbeEnabled) return false;
    const next = finite(meters, null);
    if (next === null || next <= 0) return false;
    probeDistanceM = next;
    // 이전 거리의 앵커가 남아 한 프레임 튀는 것을 막는다
    anchorStore.reset();
    identityTracker.reset();
    routeMatcher.reset();
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
    const composition = (workerPipe ? workerStatus?.composition : lastComposition) || null;
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
      presentedSubmissions,
      lastSubmittedPresentedSequence,
      lastPresentedSource,
      targetFps,
      degradedFps,
      currentFps: workerPipe
        ? Math.max(1, finite(workerStatus?.performance?.fps, targetFps))
        : targetFps,
      presentationClock: "presented-frame",
      presentationLoopActive: false,
      leaseActive: Boolean(lease?.active),
      positionQuality: lastPositionQuality,
      timeline: lastTimeline,
      presentedClock: lastPresentedClock,
      odometryTimeline: odometryTimeline.status(),
      tracking: lastTracking || trackingTracker.status(),
      worldPose: lastWorldPose || worldPoseTracker.status(),
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
          presentedClockConfidence: lastSync.presentedClockConfidence,
          presentedTargetTimestampNs: lastSync.presentedTargetTimestampNs,
          odometryPoseDelayMs: lastSync.odometryPoseDelayMs,
          reasons: lastSync.reasons,
        })
        : null,
      // 워커 모드에서는 워커가 보고한 값을, 아니면 메인의 값을 그대로 쓴다.
      mode: workerPipe ? "worker" : "main",
      renderer: workerPipe ? workerStatus?.renderer : renderer.status(),
      worker: workerStatus,
      composition: (workerPipe ? workerStatus?.composition : lastComposition) || null,
      probeEnabled: diagnosticProbeEnabled,
      probeDistanceM,
      trace: trace.status(),
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
    diagnosticListeners.clear();
    runtimeSingletons.delete(target);
    return true;
  }

  const traceControl = Object.freeze({
    enable(settings = {}) { return trace.setEnabled(true, settings); },
    disable(settings = {}) { return trace.setEnabled(false, settings); },
    clear: trace.clear,
    status: trace.status,
    snapshot: trace.snapshot,
  });

  return Object.freeze({
    activate, deactivate, resize, status, diagnose, setProbeDistance, subscribeDiagnostics,
    destroy, render: drawFrame, trace: traceControl,
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
