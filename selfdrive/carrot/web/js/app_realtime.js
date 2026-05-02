/* ---------- Home Runtime State ---------- */
function setHomeServerState(summary, detail = summary, tone = "idle") {
  void summary;
  void detail;
  void tone;
}

function summarizeLiveRuntimeState(payload, errorMessage = "") {
  if (!payload?.ok) {
    const summary = errorMessage || getUIText("reconnecting", "Reconnecting...");
    return {
      summary,
      detail: JSON.stringify({ ok: false, error: errorMessage || "live_runtime unavailable" }, null, 2),
      tone: "error",
    };
  }

  const ageMs = Number.isFinite(Number(payload.snapshotAgeMs)) ? Number(payload.snapshotAgeMs) : null;
  const servicesCount = payload.services && typeof payload.services === "object" ? Object.keys(payload.services).length : 0;
  const paramsCount = payload.runtime?.params && typeof payload.runtime.params === "object" ? Object.keys(payload.runtime.params).length : 0;
  const summary = ageMs != null && ageMs > 1500
    ? `${getUIText("reconnecting", "Reconnecting...")} ${Math.round(ageMs)}ms`
    : getUIText("connected", "Connected");
  const tone = ageMs != null && ageMs > 1500 ? "error" : "connected";
  const detail = JSON.stringify({
    ok: true,
    snapshotAgeMs: ageMs,
    servicesCount,
    paramsCount,
    meta: payload.meta || {},
  }, null, 2);

  return { summary, detail, tone };
}

const CARROT_LIVE_RUNTIME_STATE = {
  ok: false,
  meta: {},
  runtime: { params: {} },
  services: {},
  snapshotAgeMs: null,
  fetchedAtMs: 0,
  dataSignature: "",
};
window.CarrotLiveRuntimeState = CARROT_LIVE_RUNTIME_STATE;

let LIVE_RUNTIME_FETCH_T = null;
let LIVE_RUNTIME_FETCH_IN_FLIGHT = null;
let LIVE_RUNTIME_POLL_ACTIVE = false;
let LAST_HUD_PAYLOAD_SIGNATURE = "";
const RTC_STATS_POLL_MS = 1000;
const RTC_FREEZE_MAX_STALL_SAMPLES = 8;
const RTC_INITIAL_FRAME_MAX_STALL_SAMPLES = 5;
const RTC_FREEZE_CURRENT_TIME_EPSILON = 0.05;
const RTC_FREEZE_RECOVERY_COOLDOWN_MS = 4000;
const RTC_RESUME_PROGRESS_CHECK_MS = 900;
const RTC_RETRY_BASE_MS = 700;
const RTC_ICE_GATHER_TIMEOUT_MS = 1200;
const RTC_INITIAL_TRACK_TIMEOUT_MS = 2800;
const RTC_PERF_STATE = {
  active: false,
  collectedAtMs: 0,
  connectionState: "idle",
  iceConnectionState: "new",
  codec: "",
  inbound: null,
  video: null,
  network: null,
  error: "",
};
window.CarrotRtcPerf = RTC_PERF_STATE;
let RTC_STATS_T = null;
const RTC_RATE_STATE = {
  lastBytesReceived: null,
  lastCollectedAtMs: 0,
};
const RTC_FREEZE_STATE = {
  stallSamples: 0,
  lastFramesDecoded: null,
  lastTotalVideoFrames: null,
  lastCurrentTime: null,
  lastRecoveredAtMs: 0,
  consecutiveRecoveries: 0,
  everDecodedFrame: false,
};
let RTC_FREEZE_RECOVER_T = null;
let RTC_VIDEO_EVENTS_BOUND = false;
let RTC_WAIT_TRACK_PC = null;
let RTC_RESUME_CHECK_T = null;
const RTC_VISIBILITY_STATE = {
  hiddenAtMs: 0,
  currentTimeAtHide: null,
};
const RTC_TRACE_ENABLED = false;
let RTC_PC_SEQ = 0;

function rtcPcLabel(pc) {
  if (!pc) return "none";
  if (!pc.__carrotRtcLabel) {
    RTC_PC_SEQ += 1;
    pc.__carrotRtcLabel = `pc${RTC_PC_SEQ}`;
  }
  return pc.__carrotRtcLabel;
}

function rtcBuildTraceSnapshot(pc = RTC_PC) {
  const video = getRtcVideoElement();
  const track = video?.srcObject?.getVideoTracks?.()?.[0] || null;
  return {
    conn: pc?.connectionState || "none",
    ice: pc?.iceConnectionState || "none",
    framesDecoded: RTC_PERF_STATE.inbound?.framesDecoded ?? null,
    fps: RTC_PERF_STATE.inbound?.framesPerSecond ?? null,
    currentTime: Number.isFinite(Number(video?.currentTime)) ? Number(video.currentTime).toFixed(2) : null,
    readyState: Number.isFinite(Number(video?.readyState)) ? Number(video.readyState) : null,
    trackState: track?.readyState || null,
    trackMuted: typeof track?.muted === "boolean" ? track.muted : null,
    hold: Boolean(getRtcStageElement()?.classList.contains("is-video-held")),
  };
}

function rtcTrace(event, extra = {}, pc = RTC_PC) {
  if (!RTC_TRACE_ENABLED) return;
  console.log("[RTC TRACE]", {
    ts: Date.now(),
    iso: new Date().toISOString(),
    event,
    pc: rtcPcLabel(pc),
    ...rtcBuildTraceSnapshot(pc),
    ...extra,
  });
}

function rtcPcSawTrack(pc) {
  return Boolean(pc && pc.__carrotTrackSeen);
}

function getRtcVideoElement() {
  return document.getElementById("carrotRoadVideo") || document.getElementById("rtcVideo");
}

function getLegacyRtcVideoElement() {
  return document.getElementById("rtcVideo");
}

function rtcExitPictureInPicture() {
  try {
    if (document.pictureInPictureElement && typeof document.exitPictureInPicture === "function") {
      document.exitPictureInPicture().catch(() => {});
    }
  } catch {}
}

function rtcDisablePictureInPicture(video) {
  if (!video) return;
  try { video.disablePictureInPicture = true; } catch {}
  try { video.setAttribute("disablepictureinpicture", ""); } catch {}
  try { video.controlsList?.add?.("nopictureinpicture"); } catch {}
}

function getRtcVideoHoldElement() {
  return document.getElementById("carrotLastFrameCanvas");
}

function getRtcStageElement() {
  return document.getElementById("carrotStage");
}

function rtcShowVideoHold(show) {
  const stage = getRtcStageElement();
  if (!stage) return;
  stage.classList.toggle("is-video-held", Boolean(show));
}

function rtcClearVideoHold() {
  const hold = getRtcVideoHoldElement();
  if (hold) {
    const ctx = hold.getContext("2d");
    if (ctx) {
      ctx.clearRect(0, 0, hold.width || 0, hold.height || 0);
    }
  }
  rtcShowVideoHold(false);
}

function rtcCaptureVideoHoldFrame() {
  const video = getRtcVideoElement();
  const hold = getRtcVideoHoldElement();
  if (!video || !hold || Number(video.readyState || 0) < 2) return false;

  const targetWidth = Math.max(1, Number(hold.width || video.videoWidth || 0));
  const targetHeight = Math.max(1, Number(hold.height || video.videoHeight || 0));
  if (!targetWidth || !targetHeight) return false;

  const ctx = hold.getContext("2d");
  if (!ctx) return false;

  if (hold.width !== targetWidth) hold.width = targetWidth;
  if (hold.height !== targetHeight) hold.height = targetHeight;
  try {
    ctx.clearRect(0, 0, targetWidth, targetHeight);
    ctx.drawImage(video, 0, 0, targetWidth, targetHeight);
    rtcShowVideoHold(true);
    return true;
  } catch {
    return false;
  }
}

function isCarrotPageActive() {
  return document.body?.dataset?.page === "carrot";
}

function isCarrotPageVisible() {
  return isCarrotPageActive() && !document.hidden;
}

function shouldRunCarrotHudRealtime() {
  return isCarrotPageActive();
}

function shouldRunCarrotVisionRealtime() {
  return isCarrotPageActive() && window.CARROT_VISION_ACTIVE;
}

function isLandscapeOrientation() {
  if (typeof window.matchMedia === "function") {
    try {
      return window.matchMedia("(orientation: landscape)").matches;
    } catch {}
  }
  return Number(window.innerWidth || 0) > Number(window.innerHeight || 0);
}

function isFullscreenActive() {
  return Boolean(
    document.fullscreenElement ||
    document.webkitFullscreenElement ||
    document.msFullscreenElement
  );
}

async function requestCarrotFullscreen(options = {}) {
  const quiet = Boolean(options.quiet);
  if (!isLandscapeOrientation()) return false;
  if (isFullscreenActive()) return true;

  const root = document.documentElement || document.body;
  if (!root) return false;

  const request =
    root.requestFullscreen ||
    root.webkitRequestFullscreen ||
    root.msRequestFullscreen;
  if (typeof request !== "function") {
    if (!quiet && typeof showAppToast === "function") {
      showAppToast(getUIText("fullscreen_not_supported", "Fullscreen not supported on this browser."), { tone: "error" });
    }
    return false;
  }

  try {
    const result = request.call(root);
    if (result && typeof result.then === "function") {
      await result;
    }
    return true;
  } catch (error) {
    console.log("[fullscreen] request failed", error);
    return false;
  }
}

async function exitCarrotFullscreen(options = {}) {
  const quiet = Boolean(options.quiet);
  if (!isFullscreenActive()) return true;

  const exit =
    document.exitFullscreen ||
    document.webkitExitFullscreen ||
    document.msExitFullscreen;
  if (typeof exit !== "function") {
    if (!quiet && typeof showAppToast === "function") {
      showAppToast(getUIText("fullscreen_not_supported", "Fullscreen not supported on this browser."), { tone: "error" });
    }
    return false;
  }

  try {
    const result = exit.call(document);
    if (result && typeof result.then === "function") {
      await result;
    }
    return true;
  } catch (error) {
    console.log("[fullscreen] exit failed", error);
    return false;
  }
}

async function toggleCarrotFullscreen(options = {}) {
  if (isFullscreenActive()) {
    return exitCarrotFullscreen(options);
  }
  return requestCarrotFullscreen(options);
}

function shouldKeepCarrotFullscreen() {
  return document.body?.dataset?.page === "carrot" && Boolean(window.CARROT_VISION_ACTIVE);
}

async function syncCarrotFullscreenLifecycle() {
  if (shouldKeepCarrotFullscreen()) return;
  await exitCarrotFullscreen({ quiet: true }).catch(() => {});
}

window.RequestCarrotFullscreen = requestCarrotFullscreen;
window.ExitCarrotFullscreen = exitCarrotFullscreen;
window.ToggleCarrotFullscreen = toggleCarrotFullscreen;

window.addEventListener("carrot:pagechange", () => {
  void syncCarrotFullscreenLifecycle();
});
window.addEventListener("carrot:visionchange", () => {
  void syncCarrotFullscreenLifecycle();
});

function emitCarrotRenderRequest(detail = {}) {
  window.dispatchEvent(new CustomEvent("carrot:render-request", { detail }));
}

function emitCarrotVisionChange(active) {
  window.dispatchEvent(new CustomEvent("carrot:visionchange", { detail: { active: Boolean(active) } }));
}

function maybeRequestCarrotFullscreenOnPageChange(detail = {}) {
  if (String(detail?.page || "") !== "carrot") return;
  if (!window.CARROT_VISION_ACTIVE) return;
  requestCarrotFullscreen({ quiet: true }).catch(() => {});
}

function getLiveRuntimeDataSignature(payload) {
  try {
    return JSON.stringify({
      meta: payload?.meta || {},
      runtime: payload?.runtime || { params: {} },
      services: payload?.services || {},
    });
  } catch {
    return "";
  }
}

async function fetchLiveRuntimeState(force = false) {
  if (LIVE_RUNTIME_FETCH_IN_FLIGHT) return LIVE_RUNTIME_FETCH_IN_FLIGHT;

  LIVE_RUNTIME_FETCH_IN_FLIGHT = (async () => {
    let shouldNotifyRender = Boolean(force);
    try {
      const suffix = force ? "?force=1" : "";
      const response = await fetch(`/api/live_runtime${suffix}`, { cache: "no-store" });
      const payload = await response.json();
      if (!payload?.ok) throw new Error(payload?.error || "live_runtime failed");

      const wasOk = CARROT_LIVE_RUNTIME_STATE.ok;
      const nextDataSignature = getLiveRuntimeDataSignature(payload);
      const dataChanged = nextDataSignature !== CARROT_LIVE_RUNTIME_STATE.dataSignature;

      CARROT_LIVE_RUNTIME_STATE.ok = true;
      if (dataChanged) {
        CARROT_LIVE_RUNTIME_STATE.meta = payload.meta || {};
        CARROT_LIVE_RUNTIME_STATE.runtime = payload.runtime || { params: {} };
        CARROT_LIVE_RUNTIME_STATE.services = payload.services || {};
        CARROT_LIVE_RUNTIME_STATE.dataSignature = nextDataSignature;
      }
      CARROT_LIVE_RUNTIME_STATE.snapshotAgeMs = Number.isFinite(Number(payload.snapshotAgeMs)) ? Number(payload.snapshotAgeMs) : null;
      CARROT_LIVE_RUNTIME_STATE.fetchedAtMs = Date.now();
      window.CarrotLiveRuntimeState = CARROT_LIVE_RUNTIME_STATE;
      const serverState = summarizeLiveRuntimeState(payload);
      setHomeServerState(serverState.summary, serverState.detail, serverState.tone);
      shouldNotifyRender = shouldNotifyRender || dataChanged || !wasOk;
	    } catch (_error) {
      const wasOk = CARROT_LIVE_RUNTIME_STATE.ok;
      CARROT_LIVE_RUNTIME_STATE.ok = false;
      CARROT_LIVE_RUNTIME_STATE.fetchedAtMs = Date.now();
      window.CarrotLiveRuntimeState = CARROT_LIVE_RUNTIME_STATE;
	      const serverState = summarizeLiveRuntimeState(null, _error?.message || "");
	      setHomeServerState(serverState.summary, serverState.detail, serverState.tone);
      shouldNotifyRender = shouldNotifyRender || wasOk;
	    } finally {
      if (shouldNotifyRender) {
        _hudMarkDirty();
        emitCarrotRenderRequest({ force: false, overlayDirty: true, hudDirty: true });
      }
	      LIVE_RUNTIME_FETCH_IN_FLIGHT = null;
	    }
  })();

  return LIVE_RUNTIME_FETCH_IN_FLIGHT;
}

function getLiveRuntimePollMs() {
  return isCarrotPageVisible() ? 1000 : 3000;
}

function scheduleLiveRuntimeStateFetch(ms = getLiveRuntimePollMs()) {
  if (!LIVE_RUNTIME_POLL_ACTIVE) return;
  if (LIVE_RUNTIME_FETCH_T) clearTimeout(LIVE_RUNTIME_FETCH_T);
  LIVE_RUNTIME_FETCH_T = setTimeout(async () => {
    LIVE_RUNTIME_FETCH_T = null;
    if (!LIVE_RUNTIME_POLL_ACTIVE) return;
    await fetchLiveRuntimeState(false).catch(() => {});
    scheduleLiveRuntimeStateFetch(getLiveRuntimePollMs());
  }, ms);
}

function stopLiveRuntimeStateFetch() {
  LIVE_RUNTIME_POLL_ACTIVE = false;
  if (LIVE_RUNTIME_FETCH_T) {
    clearTimeout(LIVE_RUNTIME_FETCH_T);
    LIVE_RUNTIME_FETCH_T = null;
  }
}

function startLiveRuntimeStateFetch(force = false, ms = getLiveRuntimePollMs()) {
  LIVE_RUNTIME_POLL_ACTIVE = true;
  if (force) fetchLiveRuntimeState(true).catch(() => {});
  scheduleLiveRuntimeStateFetch(ms);
}

function resetRtcPerfState() {
  RTC_PERF_STATE.active = false;
  RTC_PERF_STATE.collectedAtMs = Date.now();
  RTC_PERF_STATE.connectionState = "idle";
  RTC_PERF_STATE.iceConnectionState = "new";
  RTC_PERF_STATE.codec = "";
  RTC_PERF_STATE.inbound = null;
  RTC_PERF_STATE.video = null;
  RTC_PERF_STATE.network = null;
  RTC_PERF_STATE.error = "";
  RTC_RATE_STATE.lastBytesReceived = null;
  RTC_RATE_STATE.lastCollectedAtMs = 0;
  window.CarrotRtcPerf = RTC_PERF_STATE;
}

function rtcResetFreezeWatchdog() {
  RTC_FREEZE_STATE.stallSamples = 0;
  RTC_FREEZE_STATE.lastFramesDecoded = null;
  RTC_FREEZE_STATE.lastTotalVideoFrames = null;
  RTC_FREEZE_STATE.lastCurrentTime = null;
  RTC_FREEZE_STATE.everDecodedFrame = false;
}

function rtcCancelResumeCheck() {
  if (RTC_RESUME_CHECK_T) {
    clearTimeout(RTC_RESUME_CHECK_T);
    RTC_RESUME_CHECK_T = null;
  }
}

function rtcCancelFreezeRecovery() {
  if (RTC_FREEZE_RECOVER_T) {
    clearTimeout(RTC_FREEZE_RECOVER_T);
    RTC_FREEZE_RECOVER_T = null;
  }
}

function stopRtcPerfPolling() {
  if (RTC_STATS_T) {
    clearTimeout(RTC_STATS_T);
    RTC_STATS_T = null;
  }
}

function readRtcVideoPlaybackQuality(video) {
  if (!video || typeof video.getVideoPlaybackQuality !== "function") return null;
  const quality = video.getVideoPlaybackQuality();
  if (!quality) return null;
  return {
    totalVideoFrames: Number(quality.totalVideoFrames ?? 0),
    droppedVideoFrames: Number(quality.droppedVideoFrames ?? 0),
    corruptedVideoFrames: Number(quality.corruptedVideoFrames ?? 0),
    creationTime: Number(quality.creationTime ?? 0),
    width: Number(video.videoWidth || 0),
    height: Number(video.videoHeight || 0),
    readyState: Number(video.readyState || 0),
    currentTime: Number(video.currentTime || 0),
  };
}

function extractRtcInboundVideoStats(statsReport, statsMap) {
  if (!statsReport) return { inbound: null, codec: "" };
  const codecReport = statsReport.codecId ? statsMap.get(statsReport.codecId) : null;
  return {
    codec: codecReport?.mimeType || codecReport?.id || "",
    inbound: {
      framesDecoded: Number(statsReport.framesDecoded ?? 0),
      framesDropped: Number(statsReport.framesDropped ?? 0),
      framesPerSecond: Number(statsReport.framesPerSecond ?? 0),
      frameWidth: Number(statsReport.frameWidth ?? 0),
      frameHeight: Number(statsReport.frameHeight ?? 0),
      jitter: Number(statsReport.jitter ?? 0),
      bytesReceived: Number(statsReport.bytesReceived ?? 0),
      packetsLost: Number(statsReport.packetsLost ?? 0),
      decoderImplementation: statsReport.decoderImplementation || "",
      freezeCount: Number(statsReport.freezeCount ?? 0),
      totalFreezesDuration: Number(statsReport.totalFreezesDuration ?? 0),
    },
  };
}

function extractRtcTransportStats(statsMap) {
  let selectedPair = null;
  statsMap.forEach((report) => {
    if (report?.type !== "candidate-pair") return;
    if (!(report.selected === true || report.nominated === true || report.state === "succeeded")) return;
    if (!selectedPair || report.selected === true || report.nominated === true) {
      selectedPair = report;
    }
  });

  return {
    rttMs: Number.isFinite(Number(selectedPair?.currentRoundTripTime))
      ? Number(selectedPair.currentRoundTripTime) * 1000
      : null,
  };
}

function computeRtcBitrateMbps(bytesReceived, collectedAtMs) {
  const nextBytes = Number(bytesReceived);
  const nextAtMs = Number(collectedAtMs);
  if (!Number.isFinite(nextBytes) || !Number.isFinite(nextAtMs)) {
    return null;
  }

  const prevBytes = RTC_RATE_STATE.lastBytesReceived;
  const prevAtMs = RTC_RATE_STATE.lastCollectedAtMs;
  RTC_RATE_STATE.lastBytesReceived = nextBytes;
  RTC_RATE_STATE.lastCollectedAtMs = nextAtMs;

  if (!Number.isFinite(prevBytes) || !Number.isFinite(prevAtMs)) {
    return null;
  }

  const deltaBytes = nextBytes - prevBytes;
  const deltaMs = nextAtMs - prevAtMs;
  if (deltaBytes < 0 || deltaMs < 250) {
    return null;
  }
  return (deltaBytes * 8) / (deltaMs / 1000) / 1_000_000;
}

function buildRtcNetworkStats(inboundStats, videoStats, statsMap, collectedAtMs) {
  const width = Number.isFinite(Number(inboundStats?.frameWidth))
    ? Number(inboundStats.frameWidth)
    : Number.isFinite(Number(videoStats?.width))
      ? Number(videoStats.width)
      : null;
  const height = Number.isFinite(Number(inboundStats?.frameHeight))
    ? Number(inboundStats.frameHeight)
    : Number.isFinite(Number(videoStats?.height))
      ? Number(videoStats.height)
      : null;
  const resolutionLabel =
    Number.isFinite(width) && width > 0 && Number.isFinite(height) && height > 0
      ? `${Math.round(width)}x${Math.round(height)}`
      : "";
  const bitrateMbps = computeRtcBitrateMbps(inboundStats?.bytesReceived, collectedAtMs);
  const transport = extractRtcTransportStats(statsMap);
  return {
    resolutionLabel,
    bitrateMbps: Number.isFinite(Number(bitrateMbps)) ? Number(bitrateMbps) : null,
    rttMs: Number.isFinite(Number(transport.rttMs)) ? Number(transport.rttMs) : null,
  };
}

async function collectRtcPerfStats() {
  const pc = RTC_PC;
  if (!pc) return;

  try {
    const collectedAtMs = Date.now();
    const stats = await pc.getStats(null);
    let inboundVideoReport = null;
    stats.forEach((report) => {
      if (inboundVideoReport) return;
      if (report?.type === "inbound-rtp" && report.kind === "video" && !report.isRemote) {
        inboundVideoReport = report;
      }
    });

    const video = getRtcVideoElement();
    const inbound = extractRtcInboundVideoStats(inboundVideoReport, stats);
    RTC_PERF_STATE.active = shouldRunCarrotVisionRealtime();
    RTC_PERF_STATE.collectedAtMs = collectedAtMs;
    RTC_PERF_STATE.connectionState = pc.connectionState || "unknown";
    RTC_PERF_STATE.iceConnectionState = pc.iceConnectionState || "unknown";
    RTC_PERF_STATE.codec = inbound.codec;
    RTC_PERF_STATE.inbound = inbound.inbound;
    RTC_PERF_STATE.video = readRtcVideoPlaybackQuality(video);
    RTC_PERF_STATE.network = buildRtcNetworkStats(inbound.inbound, RTC_PERF_STATE.video, stats, collectedAtMs);
    RTC_PERF_STATE.error = "";
    window.CarrotRtcPerf = RTC_PERF_STATE;
    rtcUpdateFreezeWatchdog(pc, video);
    _hudMarkDirty();
    emitCarrotRenderRequest({ force: false, overlayDirty: false, hudDirty: true });
  } catch (error) {
    RTC_PERF_STATE.active = shouldRunCarrotVisionRealtime();
    RTC_PERF_STATE.collectedAtMs = Date.now();
    RTC_PERF_STATE.connectionState = pc.connectionState || "unknown";
    RTC_PERF_STATE.iceConnectionState = pc.iceConnectionState || "unknown";
    RTC_PERF_STATE.network = null;
    RTC_PERF_STATE.error = error?.message || String(error);
    window.CarrotRtcPerf = RTC_PERF_STATE;
    rtcResetFreezeWatchdog();
    _hudMarkDirty();
    emitCarrotRenderRequest({ force: false, overlayDirty: false, hudDirty: true });
  }
}

function scheduleRtcPerfPolling(ms = RTC_STATS_POLL_MS) {
  if (RTC_STATS_T) return;
  RTC_STATS_T = setTimeout(async () => {
    RTC_STATS_T = null;
    if (!shouldRunCarrotVisionRealtime()) return;
    await collectRtcPerfStats().catch(() => {});
    scheduleRtcPerfPolling(isCarrotPageVisible() ? RTC_STATS_POLL_MS : 2500);
  }, ms);
}

function startRtcPerfPolling(force = false) {
  if (force && isCarrotPageVisible()) collectRtcPerfStats().catch(() => {});
  scheduleRtcPerfPolling(force ? (isCarrotPageVisible() ? 500 : 2500) : (isCarrotPageVisible() ? RTC_STATS_POLL_MS : 2500));
}


// ===== WebRTC (auto) =====
let RTC_PC = null;
let RTC_PENDING_PC = null;
let RTC_STANDBY_PC = null;
let RTC_RETRY_T = null;
let RTC_WAIT_TRACK_T = null;
let RTC_FAIL_COUNT = 0;
function rtcHasLiveTrack() {
  const video = getRtcVideoElement();
  const stream = video?.srcObject;
  if (!stream) return false;
  if (stream.active === false) return false;
  if (typeof stream.getVideoTracks !== "function") return true;
  const tracks = stream.getVideoTracks();
  if (!tracks.length) return true;
  return tracks.some((track) => track && track.readyState !== "ended");
}

function rtcClosePeer(pc) {
  if (!pc) return;
  try { pc.ontrack = null; } catch {}
  try { pc.onconnectionstatechange = null; } catch {}
  try { pc.oniceconnectionstatechange = null; } catch {}
  try { pc.close(); } catch {}
  if (RTC_PC === pc) RTC_PC = null;
  if (RTC_PENDING_PC === pc) RTC_PENDING_PC = null;
  if (RTC_STANDBY_PC === pc) RTC_STANDBY_PC = null;
}

function rtcStatusSet(s) {
  const el = document.getElementById("rtcStatus");
  if (el) el.textContent = String(s);
}

function rtcCancelRetry() {
  if (RTC_RETRY_T) {
    clearTimeout(RTC_RETRY_T);
    RTC_RETRY_T = null;
  }
}

async function rtcDisconnect(options = {}) {
  const keepVideo = Boolean(options.keepVideo);
  rtcCancelRetry();
  rtcDisarmTrackTimeout();
  rtcCancelResumeCheck();
  rtcCancelFreezeRecovery();
  stopRtcPerfPolling();
  const activePc = RTC_PC;
  const pendingPc = RTC_PENDING_PC;
  const standbyPc = RTC_STANDBY_PC;
  RTC_PC = null;
  RTC_PENDING_PC = null;
  RTC_STANDBY_PC = null;
  rtcClosePeer(pendingPc);
  rtcClosePeer(activePc);
  rtcClosePeer(standbyPc);
  resetRtcPerfState();
  rtcResetFreezeWatchdog();

  if (!keepVideo) {
    rtcClearVideoHold();
    const video = getRtcVideoElement();
    if (video) {
      video.srcObject = null;
    }
    const legacyVideo = getLegacyRtcVideoElement();
    if (legacyVideo && legacyVideo !== video) {
      legacyVideo.srcObject = null;
    }
  }
}

function rtcConnectionLooksLive(pc = RTC_PC) {
  if (!pc) return false;
  return pc.connectionState === "connected" || pc.iceConnectionState === "connected" || pc.iceConnectionState === "completed";
}

function rtcIsWaitingForInitialTrack(pc = RTC_PC) {
  return Boolean(RTC_WAIT_TRACK_T && RTC_WAIT_TRACK_PC && RTC_WAIT_TRACK_PC === pc);
}

function rtcUpdateFreezeSnapshot(snapshot) {
  RTC_FREEZE_STATE.lastFramesDecoded = snapshot.framesDecoded;
  RTC_FREEZE_STATE.lastTotalVideoFrames = snapshot.totalVideoFrames;
  RTC_FREEZE_STATE.lastCurrentTime = snapshot.currentTime;
}

function rtcScheduleFreezeRecovery(reason, options = {}) {
  const force = Boolean(options.force);
  if (!shouldRunCarrotVisionRealtime() || _rtcConnecting || RTC_PENDING_PC || RTC_FREEZE_RECOVER_T) return;
  const now = Date.now();
  if (!force && (now - RTC_FREEZE_STATE.lastRecoveredAtMs < RTC_FREEZE_RECOVERY_COOLDOWN_MS)) return;

  RTC_FREEZE_STATE.consecutiveRecoveries++;
  RTC_FREEZE_STATE.lastRecoveredAtMs = now;
  RTC_FREEZE_STATE.stallSamples = 0;
  rtcStatusSet(reason);
  rtcTrace("freeze_recovery_scheduled", {
    reason,
    force,
    attempt: RTC_FREEZE_STATE.consecutiveRecoveries,
  }, RTC_PC || RTC_PENDING_PC);
  console.warn("[RTC] road video stalled, reconnecting", {
    reason,
    attempt: RTC_FREEZE_STATE.consecutiveRecoveries,
    connectionState: RTC_PERF_STATE.connectionState,
    iceConnectionState: RTC_PERF_STATE.iceConnectionState,
    inbound: RTC_PERF_STATE.inbound,
    video: RTC_PERF_STATE.video,
  });

  RTC_FREEZE_RECOVER_T = setTimeout(async () => {
    RTC_FREEZE_RECOVER_T = null;
    if (!shouldRunCarrotVisionRealtime()) return;
    rtcCaptureVideoHoldFrame();
    RTC_FAIL_COUNT = 0;
    await rtcConnectOnce({ force: true }).catch(() => {});
  }, 0);
}

function rtcUpdateFreezeWatchdog(pc, video) {
  if (!shouldRunCarrotVisionRealtime() || !video) {
    rtcResetFreezeWatchdog();
    return;
  }

  if (rtcIsWaitingForInitialTrack(pc)) {
    rtcResetFreezeWatchdog();
    return;
  }

  // PC connected but track dead/muted/inactive → force reconnect
  if (rtcConnectionLooksLive(pc) && !rtcHasLiveTrack() && video.srcObject) {
    rtcResetFreezeWatchdog();
    rtcScheduleFreezeRecovery(getUIText("video_track_lost_reconnecting", "Video track lost, reconnecting..."), { force: true });
    return;
  }

  if (!rtcConnectionLooksLive(pc) || !rtcHasLiveTrack()) {
    rtcResetFreezeWatchdog();
    return;
  }

  const snapshot = {
    framesDecoded: Number.isFinite(Number(RTC_PERF_STATE.inbound?.framesDecoded)) ? Number(RTC_PERF_STATE.inbound.framesDecoded) : null,
    totalVideoFrames: Number.isFinite(Number(RTC_PERF_STATE.video?.totalVideoFrames)) ? Number(RTC_PERF_STATE.video.totalVideoFrames) : null,
    currentTime: Number.isFinite(Number(RTC_PERF_STATE.video?.currentTime)) ? Number(RTC_PERF_STATE.video.currentTime) : null,
  };
  const readyState = Number.isFinite(Number(RTC_PERF_STATE.video?.readyState)) ? Number(RTC_PERF_STATE.video.readyState) : Number(video.readyState || 0);

  if (readyState < 2 || (snapshot.framesDecoded == null && snapshot.totalVideoFrames == null && snapshot.currentTime == null)) {
    rtcResetFreezeWatchdog();
    rtcUpdateFreezeSnapshot(snapshot);
    return;
  }

  if (RTC_FREEZE_STATE.lastFramesDecoded == null && RTC_FREEZE_STATE.lastTotalVideoFrames == null && RTC_FREEZE_STATE.lastCurrentTime == null) {
    rtcUpdateFreezeSnapshot(snapshot);
    RTC_FREEZE_STATE.stallSamples = 0;
    return;
  }

  const hasProgress =
    (snapshot.framesDecoded != null && RTC_FREEZE_STATE.lastFramesDecoded != null && snapshot.framesDecoded > RTC_FREEZE_STATE.lastFramesDecoded) ||
    (snapshot.totalVideoFrames != null && RTC_FREEZE_STATE.lastTotalVideoFrames != null && snapshot.totalVideoFrames > RTC_FREEZE_STATE.lastTotalVideoFrames) ||
    (snapshot.currentTime != null && RTC_FREEZE_STATE.lastCurrentTime != null && snapshot.currentTime > RTC_FREEZE_STATE.lastCurrentTime + RTC_FREEZE_CURRENT_TIME_EPSILON);

  if (hasProgress) {
    RTC_FREEZE_STATE.stallSamples = 0;
    RTC_FREEZE_STATE.consecutiveRecoveries = 0;
    if (!RTC_FREEZE_STATE.everDecodedFrame) {
      RTC_FREEZE_STATE.everDecodedFrame = true;
    }
  } else {
    RTC_FREEZE_STATE.stallSamples++;
  }
  rtcUpdateFreezeSnapshot(snapshot);

  const stallLimit = RTC_FREEZE_STATE.everDecodedFrame ? RTC_FREEZE_MAX_STALL_SAMPLES : RTC_INITIAL_FRAME_MAX_STALL_SAMPLES;
  if (RTC_FREEZE_STATE.stallSamples >= stallLimit) {
    rtcScheduleFreezeRecovery(RTC_FREEZE_STATE.everDecodedFrame ? getUIText("video_stalled_reconnecting", "Video stalled, reconnecting...") : getUIText("no_initial_frame_reconnecting", "No initial frame, reconnecting..."));
  }
}

function rtcBindVideoEvents() {
  if (RTC_VIDEO_EVENTS_BOUND) return;
  const video = getRtcVideoElement();
  if (!video) return;

  RTC_VIDEO_EVENTS_BOUND = true;
  const legacyVideo = getLegacyRtcVideoElement();
  [video, legacyVideo].forEach((videoEl, index, list) => {
    if (!videoEl || list.indexOf(videoEl) !== index) return;
    rtcDisablePictureInPicture(videoEl);
    videoEl.addEventListener("enterpictureinpicture", rtcExitPictureInPicture);
  });

  const nudgePlayback = () => {
    if (!shouldRunCarrotVisionRealtime() || !video.srcObject) return;
    video.play().catch(() => {});
    collectRtcPerfStats().catch(() => {});
  };

  video.addEventListener("playing", () => {
    RTC_FREEZE_STATE.stallSamples = 0;
    RTC_FREEZE_STATE.everDecodedFrame = true;
    rtcClearVideoHold();
    collectRtcPerfStats().catch(() => {});
  });
  ["waiting", "stalled", "suspend", "pause", "ended"].forEach((eventName) => {
    video.addEventListener(eventName, nudgePlayback);
  });
}

function rtcScheduleRetry(ms = RTC_RETRY_BASE_MS) {
  if (!shouldRunCarrotVisionRealtime()) return;
  rtcCancelRetry();
  const backoff = Math.min(ms * Math.pow(1.5, RTC_FAIL_COUNT), 30000);
  RTC_FAIL_COUNT = Math.min(RTC_FAIL_COUNT + 1, 20);
  RTC_RETRY_T = setTimeout(async () => {
    RTC_RETRY_T = null;
    if (!shouldRunCarrotVisionRealtime()) return;
    await rtcConnectOnce().catch(() => {});
  }, backoff);
}

function rtcArmTrackTimeout(ms = 5000, expectedPc = RTC_PC) {
  if (rtcPcSawTrack(expectedPc)) {
    rtcTrace("track_timeout_arm_skipped", { timeoutMs: ms, reason: "track already seen" }, expectedPc);
    return;
  }
  if (RTC_WAIT_TRACK_T) clearTimeout(RTC_WAIT_TRACK_T);
  RTC_WAIT_TRACK_PC = expectedPc;
  RTC_WAIT_TRACK_T = setTimeout(async () => {
    RTC_WAIT_TRACK_T = null;
    if (RTC_WAIT_TRACK_PC !== expectedPc || (RTC_PC !== expectedPc && RTC_PENDING_PC !== expectedPc)) return;
    if (rtcPcSawTrack(expectedPc)) {
      RTC_WAIT_TRACK_PC = null;
      rtcTrace("track_timeout_ignored", { timeoutMs: ms, reason: "track arrived before timeout fired" }, expectedPc);
      return;
    }
    RTC_WAIT_TRACK_PC = null;
    rtcTrace("track_timeout", { timeoutMs: ms }, expectedPc);
    rtcStatusSet(getUIText("no_track_retry", "No track, retry..."));
    rtcCaptureVideoHoldFrame();
    if (RTC_PENDING_PC === expectedPc) {
      rtcClosePeer(expectedPc);
      rtcScheduleRetry(RTC_RETRY_BASE_MS);
      return;
    }
    await rtcDisconnect({ keepVideo: true });
    rtcScheduleRetry(RTC_RETRY_BASE_MS);
  }, ms);
}

function rtcDisarmTrackTimeout(expectedPc = null) {
  if (expectedPc && RTC_WAIT_TRACK_PC && RTC_WAIT_TRACK_PC !== expectedPc) return;
  if (RTC_WAIT_TRACK_T) {
    clearTimeout(RTC_WAIT_TRACK_T);
    RTC_WAIT_TRACK_T = null;
  }
  RTC_WAIT_TRACK_PC = null;
}

function rtcScheduleResumeHealthCheck(reason = "returned visible") {
  rtcCancelResumeCheck();
  RTC_RESUME_CHECK_T = setTimeout(async () => {
    RTC_RESUME_CHECK_T = null;
    if (!shouldRunCarrotVisionRealtime() || _rtcConnecting || RTC_PENDING_PC || !RTC_PC) return;
    if (!rtcConnectionLooksLive(RTC_PC) || !rtcHasLiveTrack()) {
      rtcScheduleFreezeRecovery(`${reason}, reconnecting...`, { force: true });
    }
  }, RTC_RESUME_PROGRESS_CHECK_MS);
}

async function waitIceComplete(pc, timeoutMs = RTC_ICE_GATHER_TIMEOUT_MS) {
  if (pc.iceGatheringState === "complete") return;
  await new Promise((resolve) => {
    const t = setTimeout(() => {
      pc.removeEventListener("icegatheringstatechange", onchg);
      resolve();
    }, timeoutMs);
    function onchg() {
      if (pc.iceGatheringState === "complete") {
        pc.removeEventListener("icegatheringstatechange", onchg);
        clearTimeout(t);
        resolve();
      }
    }
    pc.addEventListener("icegatheringstatechange", onchg);
  });
}

let _rtcConnecting = false;

async function rtcConnectOnce(options = {}) {
  const force = Boolean(options.force);
  if (!shouldRunCarrotVisionRealtime()) return;
  if (_rtcConnecting || RTC_PENDING_PC) return;
  if (!force && RTC_PC && (RTC_PC.connectionState === "connected" || RTC_PC.connectionState === "connecting") && rtcHasLiveTrack()) return;

  _rtcConnecting = true;
  let previousPc = RTC_PC || RTC_STANDBY_PC;
  try {
    rtcCancelRetry();
    rtcDisarmTrackTimeout();
    rtcCancelResumeCheck();
    rtcCancelFreezeRecovery();
    rtcTrace("connect_start", {
      force,
      hasPreviousPc: Boolean(previousPc),
      hasLiveTrack: rtcHasLiveTrack(),
    }, previousPc || RTC_PC);

    const keepPreviousVisible = Boolean(previousPc && rtcHasLiveTrack());
    if (keepPreviousVisible) {
      if (RTC_STANDBY_PC && RTC_STANDBY_PC !== previousPc) {
        rtcClosePeer(RTC_STANDBY_PC);
      }
      RTC_STANDBY_PC = previousPc;
      if (RTC_PC === previousPc) {
        RTC_PC = null;
      }
      stopRtcPerfPolling();
      resetRtcPerfState();
      rtcResetFreezeWatchdog();
      rtcStatusSet(getUIText("reconnecting", "Reconnecting..."));
    } else {
      await rtcDisconnect({ keepVideo: true });
      previousPc = null;
      rtcStatusSet(getUIText("connecting", "Connecting..."));
    }

    const pc = new RTCPeerConnection({
      iceServers: [],
      sdpSemantics: "unified-plan",
      iceCandidatePoolSize: 1,
    });
    rtcPcLabel(pc);
    pc.__carrotTrackSeen = false;
    RTC_PENDING_PC = pc;
    rtcTrace("pc_created", {
      keepPreviousVisible,
      hasPreviousPc: Boolean(previousPc),
    }, pc);

    const video = getRtcVideoElement();
    if (video) {
      video.muted = true;
      video.playsInline = true;
    }

    pc.addTransceiver("video", { direction: "recvonly" });

    pc.ontrack = async (ev) => {
      if (RTC_PENDING_PC !== pc) return;
      const videoEl = getRtcVideoElement();
      if (!videoEl) return;
      rtcTrace("track_received", {
        kind: ev.track?.kind || null,
        streamCount: Array.isArray(ev.streams) ? ev.streams.length : 0,
      }, pc);
      pc.__carrotTrackSeen = true;

      let stream = ev.streams && ev.streams[0];
      if (!stream) {
        stream = new MediaStream([ev.track]);
      }

      videoEl.srcObject = stream;
      RTC_PENDING_PC = null;
      RTC_PC = pc;
      const retiringPc = RTC_STANDBY_PC && RTC_STANDBY_PC !== pc ? RTC_STANDBY_PC : null;
      RTC_STANDBY_PC = null;
      try { await videoEl.play(); } catch (e) { console.log("[RTC] play() failed", e); }
      rtcStatusSet("track: " + ev.track.kind);
      rtcDisarmTrackTimeout(pc);
      RTC_FAIL_COUNT = 0;
      rtcResetFreezeWatchdog();
      rtcClearVideoHold();
      startRtcPerfPolling(true);
      collectRtcPerfStats().catch(() => {});
      if (retiringPc) {
        rtcClosePeer(retiringPc);
      }

      // Detect server-side track close → immediate recovery (guarded by PC identity)
      ev.track.addEventListener("ended", () => {
        rtcTrace("track_ended", {
          kind: ev.track?.kind || null,
          trackReadyState: ev.track?.readyState || null,
        }, pc);
        console.warn("[RTC] remote track ended");
        if ((RTC_PC === pc || RTC_PENDING_PC === pc) && shouldRunCarrotVisionRealtime() && !_rtcConnecting) {
          rtcCaptureVideoHoldFrame();
          rtcScheduleFreezeRecovery("remote track ended", { force: true });
        }
      });
    };

    pc.onconnectionstatechange = () => {
      const isPending = RTC_PENDING_PC === pc;
      const isActive = RTC_PC === pc;
      if (!isPending && !isActive) return;
      const state = pc.connectionState;
      rtcTrace("connection_state_change", {
        isPending,
        isActive,
        state,
      }, pc);
      rtcStatusSet("conn: " + state);
      if (state === "connected") RTC_FAIL_COUNT = 0;
      if (isActive) {
        collectRtcPerfStats().catch(() => {});
      }
      if (state === "failed" || state === "closed") {
        rtcCaptureVideoHoldFrame();
        if (isPending) {
          rtcClosePeer(pc);
        } else {
          rtcDisconnect({ keepVideo: true }).catch(() => {});
        }
        rtcScheduleRetry(RTC_RETRY_BASE_MS);
      }
    };

    pc.oniceconnectionstatechange = () => {
      const isPending = RTC_PENDING_PC === pc;
      const isActive = RTC_PC === pc;
      if (!isPending && !isActive) return;
      const state = pc.iceConnectionState;
      rtcTrace("ice_state_change", {
        isPending,
        isActive,
        state,
      }, pc);
      rtcStatusSet("ice: " + state);
      if (isActive) {
        collectRtcPerfStats().catch(() => {});
      }
      if (state === "failed" || state === "closed") {
        rtcCaptureVideoHoldFrame();
        if (isPending) {
          rtcClosePeer(pc);
        } else {
          rtcDisconnect({ keepVideo: true }).catch(() => {});
        }
        rtcScheduleRetry(RTC_RETRY_BASE_MS);
      }
    };

    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);
    await waitIceComplete(pc, RTC_ICE_GATHER_TIMEOUT_MS);
    rtcTrace("offer_ready", {
      localSdpBytes: pc.localDescription?.sdp?.length || 0,
    }, pc);

    const body = {
      sdp: pc.localDescription.sdp,
      cameras: ["road"],
      bridge_services_in: [],
      bridge_services_out: [],
    };

    const response = await fetch("/stream", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });

    if (!response.ok) {
      const text = await response.text().catch(() => "");
      throw new Error("stream http " + response.status + " " + text);
    }

    const answer = await response.json();
    if (!answer || !answer.sdp) throw new Error("bad answer");
    rtcTrace("answer_received", {
      remoteSdpBytes: answer.sdp?.length || 0,
      answerType: answer.type || "answer",
    }, pc);

    await pc.setRemoteDescription({ type: answer.type || "answer", sdp: answer.sdp });
    rtcTrace("answer_applied", {}, pc);
    rtcStatusSet(getUIText("connected_waiting_track", "Connected, waiting track..."));
    rtcArmTrackTimeout(RTC_INITIAL_TRACK_TIMEOUT_MS, pc);
  } catch (e) {
    rtcTrace("connect_error", {
      message: e?.message || String(e),
    }, RTC_PENDING_PC || previousPc || RTC_PC);
    rtcStatusSet("error: " + e.message);
    rtcCaptureVideoHoldFrame();
    if (RTC_PENDING_PC) {
      rtcClosePeer(RTC_PENDING_PC);
    }
    if (!RTC_PC && !RTC_STANDBY_PC) {
      await rtcDisconnect({ keepVideo: true });
    }
    rtcScheduleRetry(RTC_RETRY_BASE_MS);
  } finally {
    _rtcConnecting = false;
  }
}

window.CARROT_VISION_ACTIVE = false;
window.CARROT_VISION_AVAILABLE = false;
window.CARROT_VISION_DISABLED_MESSAGE = getUIText("vision_unavailable_hint", "DisableDM 2에서 사용 가능");
let carrotVisionBadgeHintTimer = null;

function showCarrotVisionBadgeHint(el) {
  if (!el) return;
  el.classList.add("is-tooltip-visible");
  if (carrotVisionBadgeHintTimer) clearTimeout(carrotVisionBadgeHintTimer);
  carrotVisionBadgeHintTimer = window.setTimeout(() => {
    el.classList.remove("is-tooltip-visible");
    if (document.activeElement === el) el.blur();
    carrotVisionBadgeHintTimer = null;
  }, 1400);
}

async function fetchDisableDmValue() {
  const r = await fetch("/api/params_bulk?names=DisableDM", { cache: "no-store" });
  const j = await r.json().catch(() => ({}));
  if (!r.ok || !j.ok) throw new Error(j.error || `HTTP ${r.status}`);
  const raw = j.values?.DisableDM;
  const value = Number.parseInt(String(raw ?? "0"), 10);
  return Number.isFinite(value) ? value : 0;
}

function updateCarrotVisionAvailabilityUi(available, message = window.CARROT_VISION_DISABLED_MESSAGE) {
  window.CARROT_VISION_AVAILABLE = Boolean(available);
  const button = document.getElementById("btnStartVision");
  const messageEl = document.getElementById("visionDisabledMessage");
  const unavailableHint = getUIText("vision_unavailable_hint", "DisableDM 2에서 사용 가능");
  const disabledMessage = available ? "" : (message || unavailableHint);
  if (button) {
    button.disabled = !available;
    button.setAttribute("aria-disabled", available ? "false" : "true");
    button.title = available ? "" : unavailableHint;
  }
  if (messageEl) {
    messageEl.hidden = Boolean(available);
    messageEl.replaceChildren();
    if (!available) {
      const hint = unavailableHint;
      messageEl.textContent = "!";
      messageEl.dataset.tooltip = hint;
      messageEl.title = hint;
      messageEl.setAttribute("aria-label", hint);
      messageEl.setAttribute("role", "button");
      messageEl.tabIndex = 0;
      messageEl.onclick = () => showCarrotVisionBadgeHint(messageEl);
      messageEl.ontouchstart = () => showCarrotVisionBadgeHint(messageEl);
      messageEl.onkeydown = (event) => {
        if (event.key === "Enter" || event.key === " ") {
          event.preventDefault();
          showCarrotVisionBadgeHint(messageEl);
        }
      };
    } else {
      messageEl.classList.remove("is-tooltip-visible");
      messageEl.removeAttribute("data-tooltip");
      messageEl.removeAttribute("title");
      messageEl.removeAttribute("aria-label");
      messageEl.removeAttribute("role");
      messageEl.removeAttribute("tabindex");
      messageEl.onclick = null;
      messageEl.ontouchstart = null;
      messageEl.onkeydown = null;
      if (carrotVisionBadgeHintTimer) {
        clearTimeout(carrotVisionBadgeHintTimer);
        carrotVisionBadgeHintTimer = null;
      }
    }
  }
  if (available) {
    if (!window.CARROT_VISION_ACTIVE) rtcStatusSet(getUIText("start_vision_hint", "Tap the start button to enable drive vision."));
  } else {
    if (window.CARROT_VISION_ACTIVE) {
      window.CARROT_VISION_ACTIVE = false;
      emitCarrotVisionChange(false);
      syncCarrotRealtimeLifecycle(true);
    }
    window.CARROT_VISION_DISABLED_MESSAGE = disabledMessage;
    rtcStatusSet(disabledMessage);
  }
}

async function syncCarrotVisionAvailability() {
  try {
    const disableDm = await fetchDisableDmValue();
    const available = disableDm === 2;
    updateCarrotVisionAvailabilityUi(available);
    return available;
  } catch (e) {
    updateCarrotVisionAvailabilityUi(false, getUIText("disable_dm_check_failed", "Could not check DisableDM status."));
    return false;
  }
}

window.CarrotVisionStart = async function() {
  if (window.CARROT_VISION_ACTIVE) return;
  if (!window.CARROT_VISION_AVAILABLE && !(await syncCarrotVisionAvailability())) {
    if (typeof showAppToast === "function") showAppToast(window.CARROT_VISION_DISABLED_MESSAGE, { tone: "error" });
    return;
  }
  requestCarrotFullscreen({ quiet: false }).catch(() => {});
  window.CARROT_VISION_ACTIVE = true;
  emitCarrotVisionChange(true);
  
  const btn = document.getElementById("visionStartOverlay");
  if (btn) btn.style.display = "none";

  rtcStatusSet(getUIText("connecting", "Connecting..."));
  syncCarrotRealtimeLifecycle(true);
};

function rtcInitAuto() {
  const btn = document.getElementById("btnStartVision");
  if (btn) btn.onclick = window.CarrotVisionStart;
  syncCarrotVisionAvailability().catch(() => {});
  rtcBindVideoEvents();
}

window.addEventListener("carrot:paramchange", (ev) => {
  if (ev?.detail?.name !== "DisableDM") return;
  syncCarrotVisionAvailability().catch(() => {});
});

window.addEventListener("carrot:pagechange", (ev) => {
  if (ev?.detail?.page !== "carrot") return;
  syncCarrotVisionAvailability().catch(() => {});
});

const RAW_HUD_LOG_PREFIX = "[raw hud]";
const RAW_HUD_MUX_LOG_PREFIX = "[raw hud mux]";
const RAW_HUD_SERVICES = window.CarrotRawCapnp?.HUD_SERVICES || [];
const RAW_HUD_WS = Object.create(null);
const RAW_HUD_RETRY_T = Object.create(null);
const RAW_HUD_STATE = Object.create(null);
window.CarrotHudState = RAW_HUD_STATE;
const RAW_HUD_OVERLAY_DIRTY_SERVICES = new Set(["carState", "controlsState"]);
const RAW_OVERLAY_LOG_PREFIX = "[raw overlay]";
const RAW_OVERLAY_MUX_LOG_PREFIX = "[raw overlay mux]";
const RAW_OVERLAY_SERVICES = window.CarrotRawCapnp?.OVERLAY_SERVICES || [];
const RAW_OVERLAY_WS = Object.create(null);
const RAW_OVERLAY_RETRY_T = Object.create(null);
const RAW_OVERLAY_STATE = Object.create(null);
window.CarrotOverlayState = RAW_OVERLAY_STATE;
const RAW_OVERLAY_HUD_ONLY_SERVICES = new Set(["roadCameraState", "liveDelay", "liveTorqueParameters", "liveParameters"]);
const RAW_MULTIPLEX_MODE = "raw-capnp-multiplex-relay";
const RAW_TEXT_DECODER = typeof TextDecoder !== "undefined" ? new TextDecoder() : null;
let RAW_HUD_MUX_WS = null;
let RAW_HUD_MUX_RETRY_T = null;
let RAW_HUD_MUX_DISABLED = false;
let RAW_OVERLAY_MUX_WS = null;
let RAW_OVERLAY_MUX_RETRY_T = null;
let RAW_OVERLAY_MUX_DISABLED = false;
const RAW_DECODE_WORKER_URL = "/js/raw_capnp_worker.js";
let RAW_DECODE_WORKER = null;
let RAW_DECODE_WORKER_FAILED = false;
let RAW_DECODE_REQ_ID = 0;
const RAW_DECODE_PENDING = new Map();
const RAW_DECODE_BACKPRESSURE = {
  hud: Object.create(null),
  overlay: Object.create(null),
};
const RAW_DECODE_EPOCH = {
  hud: 0,
  overlay: 0,
};

function rejectPendingRawDecodeRequests(message) {
  for (const [requestId, pending] of RAW_DECODE_PENDING.entries()) {
    RAW_DECODE_PENDING.delete(requestId);
    pending.reject(new Error(message));
  }
}

function handleRawDecodeWorkerMessage(event) {
  const payload = event?.data || {};
  const requestId = Number(payload.requestId);
  const pending = RAW_DECODE_PENDING.get(requestId);
  if (!pending) return;
  RAW_DECODE_PENDING.delete(requestId);
  if (!payload.ok) {
    pending.reject(new Error(payload.error || "raw decode failed"));
    return;
  }
  pending.resolve(payload.decoded);
}

function handleRawDecodeWorkerFailure(error) {
  console.log("[raw decode] worker disabled", error);
  RAW_DECODE_WORKER_FAILED = true;
  if (RAW_DECODE_WORKER) {
    try { RAW_DECODE_WORKER.terminate(); } catch {}
    RAW_DECODE_WORKER = null;
  }
  rejectPendingRawDecodeRequests(error?.message || "raw decode worker failed");
}

function ensureRawDecodeWorker() {
  if (RAW_DECODE_WORKER || RAW_DECODE_WORKER_FAILED || typeof Worker === "undefined") return RAW_DECODE_WORKER;
  try {
    const worker = new Worker(RAW_DECODE_WORKER_URL);
    worker.onmessage = handleRawDecodeWorkerMessage;
    worker.onerror = handleRawDecodeWorkerFailure;
    worker.onmessageerror = handleRawDecodeWorkerFailure;
    RAW_DECODE_WORKER = worker;
  } catch (error) {
    RAW_DECODE_WORKER_FAILED = true;
    console.log("[raw decode] worker unavailable", error);
  }
  return RAW_DECODE_WORKER;
}

async function normalizeRawMessageBuffer(data) {
  if (data instanceof ArrayBuffer) return data;
  if (ArrayBuffer.isView(data)) {
    return data.buffer.slice(data.byteOffset, data.byteOffset + data.byteLength);
  }
  if (data instanceof Blob) return data.arrayBuffer();
  throw new Error(`unsupported raw payload: ${Object.prototype.toString.call(data)}`);
}

function decodeRawMessageSync(streamKind, service, buffer) {
  if (streamKind === "hud") {
    return window.CarrotRawCapnp?.decodeHudEvent?.(service, buffer);
  }
  if (streamKind === "overlay") {
    return window.CarrotRawCapnp?.decodeOverlayEvent?.(service, buffer);
  }
  return null;
}

async function decodeRawMessage(streamKind, service, data) {
  const buffer = await normalizeRawMessageBuffer(data);
  const worker = ensureRawDecodeWorker();
  if (!worker) {
    return decodeRawMessageSync(streamKind, service, buffer);
  }

  return new Promise((resolve, reject) => {
    const requestId = ++RAW_DECODE_REQ_ID;
    RAW_DECODE_PENDING.set(requestId, { resolve, reject });
    try {
      worker.postMessage({ requestId, streamKind, service, buffer }, [buffer]);
    } catch (error) {
      RAW_DECODE_PENDING.delete(requestId);
      reject(error);
    }
  });
}

function getRawDecodeEpoch(streamKind) {
  return Number(RAW_DECODE_EPOCH[streamKind] || 0);
}

function invalidateRawDecodeState(streamKind) {
  RAW_DECODE_EPOCH[streamKind] = getRawDecodeEpoch(streamKind) + 1;
  const nextEpoch = RAW_DECODE_EPOCH[streamKind];
  const bucket = RAW_DECODE_BACKPRESSURE[streamKind];
  Object.values(bucket).forEach((state) => {
    state.pendingData = null;
    state.pendingEpoch = nextEpoch;
    state.pendingVersion = Number(state.pendingVersion || 0) + 1;
  });
}

function getRawDecodeServiceState(streamKind, service) {
  const bucket = RAW_DECODE_BACKPRESSURE[streamKind];
  if (!bucket[service]) {
    bucket[service] = {
      inFlight: false,
      pendingData: null,
      pendingEpoch: getRawDecodeEpoch(streamKind),
      pendingVersion: 0,
    };
  }
  return bucket[service];
}

function shouldApplyDecodedStream(streamKind) {
  return streamKind === "hud" ? shouldRunCarrotHudRealtime() : shouldRunCarrotVisionRealtime();
}

async function flushRawDecodeQueue(streamKind, service, applyDecoded) {
  const state = getRawDecodeServiceState(streamKind, service);
  if (state.inFlight) return;
  state.inFlight = true;
  try {
    while (state.pendingData != null) {
      const nextData = state.pendingData;
      const pendingEpoch = state.pendingEpoch;
      const pendingVersion = state.pendingVersion;
      state.pendingData = null;

      let decoded = null;
      try {
        decoded = await decodeRawMessage(streamKind, service, nextData);
      } catch (error) {
        console.log(`[raw decode] ${streamKind}/${service} failed`, error);
        continue;
      }

      if (!decoded) continue;
      if (!shouldApplyDecodedStream(streamKind)) continue;
      if (pendingEpoch !== getRawDecodeEpoch(streamKind)) continue;
      if (state.pendingData != null || state.pendingEpoch !== pendingEpoch || state.pendingVersion !== pendingVersion) continue;
      applyDecoded(decoded);
    }
  } finally {
    state.inFlight = false;
    if (state.pendingData != null) {
      flushRawDecodeQueue(streamKind, service, applyDecoded).catch((error) => {
        console.log(`[raw decode] ${streamKind}/${service} queue restart failed`, error);
      });
    }
  }
}

function queueRawDecodedMessage(streamKind, service, data, applyDecoded) {
  const state = getRawDecodeServiceState(streamKind, service);
  state.pendingData = data;
  state.pendingEpoch = getRawDecodeEpoch(streamKind);
  state.pendingVersion = Number(state.pendingVersion || 0) + 1;
  flushRawDecodeQueue(streamKind, service, applyDecoded).catch((error) => {
    console.log(`[raw decode] ${streamKind}/${service} queue failed`, error);
  });
}

function buildRawMultiplexUrl(services) {
  const wsProto = (location.protocol === "https:") ? "wss" : "ws";
  const params = new URLSearchParams();
  params.set("services", services.join(","));
  return `${wsProto}://${location.host}/ws/raw_multiplex?${params.toString()}`;
}

async function parseRawMultiplexFrame(data) {
  const buffer = await normalizeRawMessageBuffer(data);
  const bytes = new Uint8Array(buffer);
  if (bytes.length < 2) {
    throw new Error("bad raw multiplex frame");
  }
  const serviceNameLength = bytes[0];
  const serviceNameEnd = 1 + serviceNameLength;
  if (bytes.length <= serviceNameEnd) {
    throw new Error("truncated raw multiplex frame");
  }
  const service = RAW_TEXT_DECODER
    ? RAW_TEXT_DECODER.decode(bytes.subarray(1, serviceNameEnd))
    : String.fromCharCode(...bytes.subarray(1, serviceNameEnd));
  return {
    service,
    payload: buffer.slice(serviceNameEnd),
  };
}

function handleHudDecodedMessage(service, data) {
  queueRawDecodedMessage("hud", service, data, (decoded) => {
    RAW_HUD_STATE[service] = decoded;
    window.CarrotHudState = RAW_HUD_STATE;
    _hudMarkDirty();
    emitCarrotRenderRequest({
      hudDirty: true,
      overlayDirty: RAW_HUD_OVERLAY_DIRTY_SERVICES.has(service),
    });
  });
}

function handleOverlayDecodedMessage(service, data) {
  queueRawDecodedMessage("overlay", service, data, (decoded) => {
    RAW_OVERLAY_STATE[service] = decoded;
    window.CarrotOverlayState = RAW_OVERLAY_STATE;
    emitCarrotRenderRequest({
      hudDirty: true,
      overlayDirty: !RAW_OVERLAY_HUD_ONLY_SERVICES.has(service),
    });
  });
}

function drivingHudUpdateFromCarPayload(j) {
  if (!window.DrivingHud || !j) return;

  const runtimeIsMetric = window.CarrotLiveRuntimeState?.runtime?.params?.IsMetric;
  const isMetric = j.isMetric != null
    ? Boolean(Number(j.isMetric))
    : (runtimeIsMetric == null ? true : Boolean(Number(runtimeIsMetric)));
  const vEgoKph = (typeof j.vEgo === "number" && isFinite(j.vEgo)) ? j.vEgo * 3.6 : null;
  const payload = {
    isMetric,
    cpuTempC: j.cpuTempC,
    memPct: j.memPct,
    diskPct: j.diskPct,
    voltageV: j.voltageV,
    vEgoKph,
    vSetKph: j.vSetKph,
    temp: j.temp,
    redDot: j.redDot,
    tlight: j.tlight,
    tfGap: j.tfGap,
    tfBars: j.tfBars,
    gear: j.gear,
    gearStep: j.gearStep,
    gpsOk: j.gpsOk,
    driveMode: j.driveMode,
    speedLimitKph: j.speedLimitKph,
    speedLimitOver: j.speedLimitOver,
    speedLimitBlink: j.speedLimitBlink,
    apm: j.apm,
  };

  const payloadSignature = [
    typeof LANG === "string" ? LANG : "",
    payload.isMetric ? 1 : 0,
    payload.cpuTempC ?? "-",
    payload.memPct ?? "-",
    payload.diskPct ?? "-",
    payload.voltageV ?? "-",
    payload.vEgoKph ?? "-",
    payload.vSetKph ?? "-",
    payload.temp?.source ?? "-",
    payload.temp?.speed ?? "-",
    payload.temp?.is_decel ? 1 : 0,
    payload.redDot ?? "-",
    payload.tlight ?? "-",
    payload.tfGap ?? "-",
    payload.tfBars ?? "-",
    payload.gear ?? "-",
    payload.gearStep ?? "-",
    payload.gpsOk ?? "-",
    payload.driveMode?.name ?? "-",
    payload.driveMode?.kind ?? "-",
    payload.speedLimitKph ?? "-",
    payload.speedLimitOver ? 1 : 0,
    payload.speedLimitBlink ? 1 : 0,
    payload.apm ?? "-",
  ].join("|");
  if (payloadSignature === LAST_HUD_PAYLOAD_SIGNATURE) return;
  LAST_HUD_PAYLOAD_SIGNATURE = payloadSignature;
  window.DrivingHud.update(payload);
}

function averageFiniteMetric(values) {
  const samples = Array.isArray(values)
    ? values.map((value) => Number(value)).filter((value) => Number.isFinite(value))
    : [];
  if (!samples.length) return null;
  const total = samples.reduce((sum, value) => sum + value, 0);
  return total / samples.length;
}

function pickFiniteMetric(...values) {
  for (const value of values) {
    if (value === null || value === undefined || value === "") continue;
    const numeric = Number(value);
    if (Number.isFinite(numeric)) return numeric;
  }
  return null;
}

function deriveNormalizedHudDeviceMetrics(rawHudState) {
  const liveServices = window.CarrotLiveRuntimeState?.services;
  const liveDeviceState = liveServices?.deviceState && typeof liveServices.deviceState === "object"
    ? liveServices.deviceState
    : {};
  const livePeripheralState = liveServices?.peripheralState && typeof liveServices.peripheralState === "object"
    ? liveServices.peripheralState
    : {};
  const rawDeviceState = rawHudState?.deviceState && typeof rawHudState.deviceState === "object"
    ? rawHudState.deviceState
    : {};
  const rawPeripheralState = rawHudState?.peripheralState && typeof rawHudState.peripheralState === "object"
    ? rawHudState.peripheralState
    : {};

  const cpuTempC = pickFiniteMetric(
    averageFiniteMetric(liveDeviceState.cpuTempC),
    averageFiniteMetric(rawDeviceState.cpuTempC),
  );
  const memPct = pickFiniteMetric(
    liveDeviceState.memoryUsagePercent,
    rawDeviceState.memoryUsagePercent,
  );
  const freeSpacePct = pickFiniteMetric(liveDeviceState.freeSpacePercent);
  const diskPct = Number.isFinite(freeSpacePct)
    ? Math.min(100, Math.max(0, 100 - freeSpacePct))
    : null;
  const voltageMv = pickFiniteMetric(
    livePeripheralState.voltage,
    rawPeripheralState.voltage,
  );
  const voltageV = Number.isFinite(voltageMv) ? (voltageMv / 1000.0) : null;

  return { cpuTempC, memPct, diskPct, voltageV };
}

function deriveNormalizedHudVehicleMetrics(rawHudState) {
  const liveServices = window.CarrotLiveRuntimeState?.services;
  const liveCarState = liveServices?.carState && typeof liveServices.carState === "object"
    ? liveServices.carState
    : {};
  const rawCarState = rawHudState?.carState && typeof rawHudState.carState === "object"
    ? rawHudState.carState
    : {};

  const gearStep = pickFiniteMetric(
    rawCarState.gearStep,
    liveCarState.gearStep,
  );

  return {
    gearStep: Number.isFinite(gearStep) && gearStep > 0 ? Math.round(gearStep) : null,
  };
}

function drivingHudUpdateFromRawState() {
  const payload = window.CarrotRawCapnp?.deriveHudPayload?.(RAW_HUD_STATE);
  if (!payload) return;
  Object.assign(payload, deriveNormalizedHudDeviceMetrics(RAW_HUD_STATE));
  Object.assign(payload, deriveNormalizedHudVehicleMetrics(RAW_HUD_STATE));
  drivingHudUpdateFromCarPayload(payload);
}

let _hudRenderDirty = false;
let _hudRafId = null;

function _hudMarkDirty() {
  _hudRenderDirty = true;
  _hudScheduleRender();
}

function _hudScheduleRender() {
  if (_hudRafId != null || !_hudRenderDirty) return;
  if (!shouldRunCarrotHudRealtime()) return;
  _hudRafId = requestAnimationFrame(() => {
    _hudRafId = null;
    if (!_hudRenderDirty || !shouldRunCarrotHudRealtime()) return;
    _hudRenderDirty = false;
    drivingHudUpdateFromRawState();
  });
}

function _hudStopRenderLoop() {
  if (_hudRafId != null) {
    cancelAnimationFrame(_hudRafId);
    _hudRafId = null;
  }
  _hudRenderDirty = false;
}

function rawHudScheduleReconnect(service, ms = 1000) {
  if (!shouldRunCarrotHudRealtime()) return;
  if (RAW_HUD_RETRY_T[service]) return;
  RAW_HUD_RETRY_T[service] = setTimeout(() => {
    RAW_HUD_RETRY_T[service] = null;
    if (!shouldRunCarrotHudRealtime()) return;
    rawHudConnectService(service);
  }, ms);
}

function rawHudScheduleMultiplexReconnect(ms = 1000) {
  if (!shouldRunCarrotHudRealtime() || RAW_HUD_MUX_DISABLED) return;
  if (RAW_HUD_MUX_RETRY_T) return;
  RAW_HUD_MUX_RETRY_T = setTimeout(() => {
    RAW_HUD_MUX_RETRY_T = null;
    if (!shouldRunCarrotHudRealtime() || RAW_HUD_MUX_DISABLED) return;
    rawHudConnectMultiplex();
  }, ms);
}

function rawHudConnectService(service) {
  if (!shouldRunCarrotHudRealtime()) return;
  const current = RAW_HUD_WS[service];
  if (current && (current.readyState === WebSocket.OPEN || current.readyState === WebSocket.CONNECTING)) return;

  const wsProto = (location.protocol === "https:") ? "wss" : "ws";
  const ws = new WebSocket(`${wsProto}://${location.host}/ws/raw/${service}`);
  ws.binaryType = "arraybuffer";
  RAW_HUD_WS[service] = ws;

  ws.onopen = () => {
    console.log(RAW_HUD_LOG_PREFIX, service, "open");
  };

  ws.onmessage = async (ev) => {
    try {
      if (typeof ev.data === "string") {
        const hello = JSON.parse(ev.data);
        console.log(RAW_HUD_LOG_PREFIX, service, "hello", hello);
        return;
      }
      handleHudDecodedMessage(service, ev.data);
    } catch (e) {
      console.log(RAW_HUD_LOG_PREFIX, service, "bad msg", e);
    }
  };

  ws.onerror = (e) => {
    console.log(RAW_HUD_LOG_PREFIX, service, "error", e);
  };

  ws.onclose = () => {
    console.log(RAW_HUD_LOG_PREFIX, service, "close -> reconnect");
    RAW_HUD_WS[service] = null;
    rawHudScheduleReconnect(service, 1000);
  };
}

function rawHudConnectAllLegacy() {
  for (const service of RAW_HUD_SERVICES) {
    rawHudConnectService(service);
  }
  _hudMarkDirty();
}

function rawHudConnectMultiplex() {
  if (!shouldRunCarrotHudRealtime() || RAW_HUD_MUX_DISABLED) return;
  const current = RAW_HUD_MUX_WS;
  if (current && (current.readyState === WebSocket.OPEN || current.readyState === WebSocket.CONNECTING)) return;

  let helloReceived = false;
  const ws = new WebSocket(buildRawMultiplexUrl(RAW_HUD_SERVICES));
  ws.binaryType = "arraybuffer";
  RAW_HUD_MUX_WS = ws;

  ws.onopen = () => {
    console.log(RAW_HUD_MUX_LOG_PREFIX, "open");
  };

  ws.onmessage = async (ev) => {
    try {
      if (typeof ev.data === "string") {
        const hello = JSON.parse(ev.data);
        console.log(RAW_HUD_MUX_LOG_PREFIX, "hello", hello);
        helloReceived = hello?.mode === RAW_MULTIPLEX_MODE;
        return;
      }
      const frame = await parseRawMultiplexFrame(ev.data);
      if (!RAW_HUD_SERVICES.includes(frame.service)) return;
      handleHudDecodedMessage(frame.service, frame.payload);
    } catch (e) {
      console.log(RAW_HUD_MUX_LOG_PREFIX, "bad msg", e);
    }
  };

  ws.onerror = (e) => {
    console.log(RAW_HUD_MUX_LOG_PREFIX, "error", e);
  };

  ws.onclose = () => {
    console.log(RAW_HUD_MUX_LOG_PREFIX, "close");
    RAW_HUD_MUX_WS = null;
    if (!shouldRunCarrotHudRealtime()) return;
    if (!helloReceived) {
      RAW_HUD_MUX_DISABLED = true;
      rawHudConnectAllLegacy();
      return;
    }
    rawHudScheduleMultiplexReconnect(1000);
  };
}

function rawHudConnectAll() {
  invalidateRawDecodeState("hud");
  if (!RAW_HUD_MUX_DISABLED) {
    rawHudConnectMultiplex();
    if (RAW_HUD_MUX_WS) {
      _hudMarkDirty();
      return;
    }
  }
  rawHudConnectAllLegacy();
}

function rawOverlayScheduleReconnect(service, ms = 1000) {
  if (!shouldRunCarrotVisionRealtime()) return;
  if (RAW_OVERLAY_RETRY_T[service]) return;
  RAW_OVERLAY_RETRY_T[service] = setTimeout(() => {
    RAW_OVERLAY_RETRY_T[service] = null;
    if (!shouldRunCarrotVisionRealtime()) return;
    rawOverlayConnectService(service);
  }, ms);
}

function rawOverlayScheduleMultiplexReconnect(ms = 1000) {
  if (!shouldRunCarrotVisionRealtime() || RAW_OVERLAY_MUX_DISABLED) return;
  if (RAW_OVERLAY_MUX_RETRY_T) return;
  RAW_OVERLAY_MUX_RETRY_T = setTimeout(() => {
    RAW_OVERLAY_MUX_RETRY_T = null;
    if (!shouldRunCarrotVisionRealtime() || RAW_OVERLAY_MUX_DISABLED) return;
    rawOverlayConnectMultiplex();
  }, ms);
}

function rawOverlayConnectService(service) {
  if (!shouldRunCarrotVisionRealtime()) return;
  const current = RAW_OVERLAY_WS[service];
  if (current && (current.readyState === WebSocket.OPEN || current.readyState === WebSocket.CONNECTING)) return;

  const wsProto = (location.protocol === "https:") ? "wss" : "ws";
  const ws = new WebSocket(`${wsProto}://${location.host}/ws/raw/${service}`);
  ws.binaryType = "arraybuffer";
  RAW_OVERLAY_WS[service] = ws;

  ws.onopen = () => {
    console.log(RAW_OVERLAY_LOG_PREFIX, service, "open");
  };

  ws.onmessage = async (ev) => {
    try {
      if (typeof ev.data === "string") {
        const hello = JSON.parse(ev.data);
        console.log(RAW_OVERLAY_LOG_PREFIX, service, "hello", hello);
        return;
      }
      handleOverlayDecodedMessage(service, ev.data);
    } catch (e) {
      console.log(RAW_OVERLAY_LOG_PREFIX, service, "bad msg", e);
    }
  };

  ws.onerror = (e) => {
    console.log(RAW_OVERLAY_LOG_PREFIX, service, "error", e);
  };

  ws.onclose = () => {
    console.log(RAW_OVERLAY_LOG_PREFIX, service, "close -> reconnect");
    RAW_OVERLAY_WS[service] = null;
    rawOverlayScheduleReconnect(service, 1000);
  };
}

function rawOverlayConnectAllLegacy() {
  for (const service of RAW_OVERLAY_SERVICES) {
    rawOverlayConnectService(service);
  }
}

function rawOverlayConnectMultiplex() {
  if (!shouldRunCarrotVisionRealtime() || RAW_OVERLAY_MUX_DISABLED) return;
  const current = RAW_OVERLAY_MUX_WS;
  if (current && (current.readyState === WebSocket.OPEN || current.readyState === WebSocket.CONNECTING)) return;

  let helloReceived = false;
  const ws = new WebSocket(buildRawMultiplexUrl(RAW_OVERLAY_SERVICES));
  ws.binaryType = "arraybuffer";
  RAW_OVERLAY_MUX_WS = ws;

  ws.onopen = () => {
    console.log(RAW_OVERLAY_MUX_LOG_PREFIX, "open");
  };

  ws.onmessage = async (ev) => {
    try {
      if (typeof ev.data === "string") {
        const hello = JSON.parse(ev.data);
        console.log(RAW_OVERLAY_MUX_LOG_PREFIX, "hello", hello);
        helloReceived = hello?.mode === RAW_MULTIPLEX_MODE;
        return;
      }
      const frame = await parseRawMultiplexFrame(ev.data);
      if (!RAW_OVERLAY_SERVICES.includes(frame.service)) return;
      handleOverlayDecodedMessage(frame.service, frame.payload);
    } catch (e) {
      console.log(RAW_OVERLAY_MUX_LOG_PREFIX, "bad msg", e);
    }
  };

  ws.onerror = (e) => {
    console.log(RAW_OVERLAY_MUX_LOG_PREFIX, "error", e);
  };

  ws.onclose = () => {
    console.log(RAW_OVERLAY_MUX_LOG_PREFIX, "close");
    RAW_OVERLAY_MUX_WS = null;
    if (!shouldRunCarrotVisionRealtime()) return;
    if (!helloReceived) {
      RAW_OVERLAY_MUX_DISABLED = true;
      rawOverlayConnectAllLegacy();
      return;
    }
    rawOverlayScheduleMultiplexReconnect(1000);
  };
}

function rawOverlayConnectAll() {
  invalidateRawDecodeState("overlay");
  if (!RAW_OVERLAY_MUX_DISABLED) {
    rawOverlayConnectMultiplex();
    if (RAW_OVERLAY_MUX_WS) return;
  }
  rawOverlayConnectAllLegacy();
}

async function rawHudDisconnectAll() {
  invalidateRawDecodeState("hud");
  _hudStopRenderLoop();
  if (RAW_HUD_MUX_RETRY_T) {
    clearTimeout(RAW_HUD_MUX_RETRY_T);
    RAW_HUD_MUX_RETRY_T = null;
  }
  try { if (RAW_HUD_MUX_WS) RAW_HUD_MUX_WS.close(); } catch {}
  RAW_HUD_MUX_WS = null;
  for (const service of RAW_HUD_SERVICES) {
    if (RAW_HUD_RETRY_T[service]) {
      clearTimeout(RAW_HUD_RETRY_T[service]);
      RAW_HUD_RETRY_T[service] = null;
    }
    try { if (RAW_HUD_WS[service]) RAW_HUD_WS[service].close(); } catch {}
    RAW_HUD_WS[service] = null;
  }
}

async function rawOverlayDisconnectAll() {
  invalidateRawDecodeState("overlay");
  if (RAW_OVERLAY_MUX_RETRY_T) {
    clearTimeout(RAW_OVERLAY_MUX_RETRY_T);
    RAW_OVERLAY_MUX_RETRY_T = null;
  }
  try { if (RAW_OVERLAY_MUX_WS) RAW_OVERLAY_MUX_WS.close(); } catch {}
  RAW_OVERLAY_MUX_WS = null;
  for (const service of RAW_OVERLAY_SERVICES) {
    if (RAW_OVERLAY_RETRY_T[service]) {
      clearTimeout(RAW_OVERLAY_RETRY_T[service]);
      RAW_OVERLAY_RETRY_T[service] = null;
    }
    try { if (RAW_OVERLAY_WS[service]) RAW_OVERLAY_WS[service].close(); } catch {}
    RAW_OVERLAY_WS[service] = null;
  }
}

async function syncServerTimeOnConnect() {
  try {
    const timezone = Intl.DateTimeFormat().resolvedOptions().timeZone || "UTC";
    const body = {
      epoch_ms: Date.now(),
      client_iso: new Date().toISOString(),
      timezone,
      debug: true, // server side debug print enabled
    };

    console.log("[time_sync] request", body);
    const r = await fetch("/api/time_sync", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    const j = await r.json().catch(() => ({}));
    console.log("[time_sync] response", j);
    return j;
  } catch (e) {
    console.log("[time_sync] failed", e);
    return { ok: false, error: String(e) };
  }
}

async function startAll() {
  if (typeof syncWebLanguageFromDeviceDefault === "function") {
    await syncWebLanguageFromDeviceDefault();
  }
  renderUIText();
  if (typeof window.bootstrapWebStartPage === "function") {
    window.bootstrapWebStartPage("realtime");
  }
  console.log("[time_sync] syncing server time on page load");
  syncServerTimeOnConnect().catch(() => {});
  rtcInitAuto();
  ensureRawDecodeWorker();

  if (window.DrivingHud) {
    window.DrivingHud.init();
    _hudMarkDirty();
  }
  syncCarrotRealtimeLifecycle(false);
}

let _carrotHudRealtimeActive = false;
let _carrotVisionRealtimeActive = false;

function syncCarrotRealtimeLifecycle(forceFetch = false) {
  const nextHudActive = shouldRunCarrotHudRealtime();
  const nextVisionActive = shouldRunCarrotVisionRealtime();

  if (nextHudActive === _carrotHudRealtimeActive && nextVisionActive === _carrotVisionRealtimeActive && !forceFetch) {
    if (nextVisionActive && !_rtcConnecting && (!RTC_PC || !rtcHasLiveTrack())) {
      rtcConnectOnce().catch(() => {});
    }
    if (nextVisionActive) scheduleRtcPerfPolling();
    if (nextHudActive) _hudScheduleRender();
    return;
  }

  _carrotHudRealtimeActive = nextHudActive;
  _carrotVisionRealtimeActive = nextVisionActive;

  if (nextHudActive) {
    console.log("[perf] carrot hud realtime -> active");
    ensureRawDecodeWorker();
    rawHudConnectAll();
    startLiveRuntimeStateFetch(forceFetch, 100);
    _hudMarkDirty();
  } else {
    console.log("[perf] carrot hud realtime -> idle");
    _hudStopRenderLoop();
    stopLiveRuntimeStateFetch();
    rawHudDisconnectAll();
  }

  if (nextVisionActive) {
    console.log("[perf] carrot vision realtime -> active");
    requestCarrotFullscreen({ quiet: true }).catch(() => {});
    ensureRawDecodeWorker();
    rawOverlayConnectAll();
    startRtcPerfPolling(true);
    if (!_rtcConnecting && (!RTC_PC || !rtcHasLiveTrack())) {
      rtcCancelRetry();
      RTC_FAIL_COUNT = 0;
      rtcConnectOnce().catch(() => {});
    }
  } else {
    console.log("[perf] carrot vision realtime -> idle");
    stopRtcPerfPolling();
    rawOverlayDisconnectAll();
    rtcDisconnect().catch(() => {});
  }

  emitCarrotRenderRequest({ force: true, overlayDirty: true, hudDirty: true });
}

document.addEventListener("visibilitychange", () => {
  if (document.hidden) {
    const video = getRtcVideoElement();
    RTC_VISIBILITY_STATE.hiddenAtMs = Date.now();
    RTC_VISIBILITY_STATE.currentTimeAtHide = Number(video?.currentTime || 0);
    rtcExitPictureInPicture();
  }
  syncCarrotRealtimeLifecycle(false);
  if (!document.hidden && shouldRunCarrotVisionRealtime() && RTC_PC && !_rtcConnecting) {
    collectRtcPerfStats().catch(() => {});
    rtcScheduleResumeHealthCheck("returned visible");
  }
});

window.addEventListener("offline", () => {
  rtcStatusSet("offline");
});

window.addEventListener("online", () => {
  syncCarrotRealtimeLifecycle(false);
  if (shouldRunCarrotVisionRealtime() && RTC_PC && !_rtcConnecting) {
    rtcScheduleResumeHealthCheck("network resumed");
  }
});
window.addEventListener("pagehide", rtcExitPictureInPicture);
window.addEventListener("carrot:pagechange", (event) => {
  maybeRequestCarrotFullscreenOnPageChange(event?.detail || {});
  syncCarrotRealtimeLifecycle(false);
});



if (document.readyState === "loading") {
  window.addEventListener("DOMContentLoaded", startAll);
} else {
  startAll();
}
