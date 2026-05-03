/* Carrot Vision WebRTC runtime.
 * Owns only the road-camera WebRTC connection, RTC health, hold-frame, and RTC stats.
 */
var CARROT_VISION_PHASE = window.CarrotVisionPhase;
var CARROT_VISION_STATE = window.CarrotVisionState;
var setCarrotVisionPhase = window.CarrotVisionSetPhase;
var setCarrotVisionState = window.CarrotVisionSetState;

const RTC_STATS_POLL_MS = 1000;
const RTC_FREEZE_MAX_STALL_SAMPLES = 8;
const RTC_INITIAL_FRAME_MAX_STALL_SAMPLES = 5;
const RTC_FREEZE_CURRENT_TIME_EPSILON = 0.05;
const RTC_FREEZE_RECOVERY_COOLDOWN_MS = 4000;
const RTC_RESUME_PROGRESS_CHECK_MS = 900;
const RTC_RETRY_BASE_MS = 700;
const RTC_ICE_GATHER_TIMEOUT_MS = 1200;
const RTC_INITIAL_TRACK_TIMEOUT_MS = 2800;
const RTC_STREAM_FETCH_TIMEOUT_MS = 6500;
const RTC_PENDING_STALE_MS = 9000;
const CARROT_VISION_HEALTH_POLL_MS = 2000;
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
let RTC_RECOVERY_T = null;
let RTC_VIDEO_EVENTS_BOUND = false;
let RTC_WAIT_TRACK_PC = null;
let RTC_RESUME_CHECK_T = null;
const RTC_VISIBILITY_STATE = {
  hiddenAtMs: 0,
  currentTimeAtHide: null,
};
const RTC_TRACE_ENABLED = false;
let RTC_PC_SEQ = 0;
let CARROT_VISION_HEALTH_T = null;
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

function rtcCancelRecovery() {
  if (RTC_RECOVERY_T) {
    clearTimeout(RTC_RECOVERY_T);
    RTC_RECOVERY_T = null;
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
window.rtcStatusSet = rtcStatusSet;

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
  rtcCancelRecovery();
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

function requestCarrotVisionRecovery(reason, options = {}) {
  const force = Boolean(options.force);
  const allowConnecting = Boolean(options.allowConnecting);
  const allowPending = Boolean(options.allowPending);
  if (!shouldRunCarrotVisionRealtime() || (!allowConnecting && _rtcConnecting) || (!allowPending && RTC_PENDING_PC) || RTC_RECOVERY_T) return false;
  const now = Date.now();
  if (options.cooldown !== false && !force && (now - RTC_FREEZE_STATE.lastRecoveredAtMs < RTC_FREEZE_RECOVERY_COOLDOWN_MS)) return false;

  RTC_FREEZE_STATE.consecutiveRecoveries++;
  RTC_FREEZE_STATE.lastRecoveredAtMs = now;
  RTC_FREEZE_STATE.stallSamples = 0;
  const action = options.action || "force-connect";
  const retryMs = Number.isFinite(Number(options.retryMs)) ? Number(options.retryMs) : RTC_RETRY_BASE_MS;
  const targetPc = options.pc || RTC_PENDING_PC || RTC_PC;
  const statusText = options.statusText || reason;
  rtcStatusSet(statusText);
  setCarrotVisionPhase(CARROT_VISION_PHASE.RECOVERING, {
    reason,
    statusText,
    rtc: {
      state: options.rtcState || "recovering",
      pending: Boolean(RTC_PENDING_PC),
      pcLabel: rtcPcLabel(targetPc),
      liveTrack: rtcHasLiveTrack(),
    },
    updateRtcStatus: false,
  });
  rtcTrace("recovery_scheduled", {
    reason,
    action,
    force,
    attempt: RTC_FREEZE_STATE.consecutiveRecoveries,
  }, targetPc);
  console.warn("[RTC] recovery scheduled", {
    reason,
    action,
    attempt: RTC_FREEZE_STATE.consecutiveRecoveries,
    connectionState: RTC_PERF_STATE.connectionState,
    iceConnectionState: RTC_PERF_STATE.iceConnectionState,
    inbound: RTC_PERF_STATE.inbound,
    video: RTC_PERF_STATE.video,
  });

  RTC_RECOVERY_T = setTimeout(async () => {
    RTC_RECOVERY_T = null;
    if (!shouldRunCarrotVisionRealtime()) return;
    if (options.capture !== false) rtcCaptureVideoHoldFrame();

    if (action === "retry-pending") {
      if (targetPc && RTC_PENDING_PC === targetPc) rtcClosePeer(targetPc);
      _rtcConnecting = false;
      rtcScheduleRetry(retryMs);
      return;
    }

    if (action === "retry-after-disconnect") {
      if (targetPc && RTC_PENDING_PC === targetPc) rtcClosePeer(targetPc);
      else await rtcDisconnect({ keepVideo: true }).catch(() => {});
      _rtcConnecting = false;
      rtcScheduleRetry(retryMs);
      return;
    }

    RTC_FAIL_COUNT = 0;
    await rtcConnectOnce({ force: true }).catch(() => {});
  }, Number.isFinite(Number(options.delayMs)) ? Number(options.delayMs) : 0);
  return true;
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
    requestCarrotVisionRecovery(getUIText("video_track_lost_reconnecting", "Video track lost, reconnecting..."), {
      action: "force-connect",
      force: true,
    });
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
    requestCarrotVisionRecovery(
      RTC_FREEZE_STATE.everDecodedFrame
        ? getUIText("video_stalled_reconnecting", "Video stalled, reconnecting...")
        : getUIText("no_initial_frame_reconnecting", "No initial frame, reconnecting..."),
      { action: "force-connect" },
    );
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
    requestCarrotVisionRender();
  };

  video.addEventListener("playing", () => {
    RTC_FREEZE_STATE.stallSamples = 0;
    RTC_FREEZE_STATE.everDecodedFrame = true;
    rtcClearVideoHold();
    collectRtcPerfStats().catch(() => {});
    requestCarrotVisionRender();
  });
  ["loadedmetadata", "loadeddata", "canplay", "resize"].forEach((eventName) => {
    video.addEventListener(eventName, requestCarrotVisionRender);
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
    requestCarrotVisionRecovery("rtc track timeout", {
      action: RTC_PENDING_PC === expectedPc ? "retry-pending" : "retry-after-disconnect",
      pc: expectedPc,
      force: true,
      allowConnecting: true,
      allowPending: true,
      statusText: getUIText("no_track_retry", "No track, retry..."),
      rtcState: "track-timeout",
      retryMs: RTC_RETRY_BASE_MS,
      cooldown: false,
    });
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
      requestCarrotVisionRecovery(`${reason}, reconnecting...`, {
        action: "force-connect",
        force: true,
        statusText: getUIText("reconnecting", "Reconnecting..."),
        rtcState: "resume-health-failed",
      });
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

async function fetchWithTimeout(url, options = {}, timeoutMs = RTC_STREAM_FETCH_TIMEOUT_MS) {
  if (typeof AbortController === "undefined") {
    return fetch(url, options);
  }
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), timeoutMs);
  try {
    return await fetch(url, { ...options, signal: controller.signal });
  } finally {
    clearTimeout(timer);
  }
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
    rtcCancelRecovery();
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
      setCarrotVisionPhase(CARROT_VISION_PHASE.RECOVERING, {
        reason: "rtc reconnect keep previous",
        rtc: { state: "reconnecting", pending: false, liveTrack: rtcHasLiveTrack(), pcLabel: rtcPcLabel(previousPc) },
        updateRtcStatus: false,
      });
    } else {
      await rtcDisconnect({ keepVideo: true });
      previousPc = null;
      rtcStatusSet(getUIText("connecting", "Connecting..."));
      setCarrotVisionPhase(CARROT_VISION_PHASE.RTC_CONNECTING, {
        reason: "rtc connect",
        rtc: { state: "connecting", pending: false, liveTrack: false, pcLabel: "none", trackSeen: false },
        updateRtcStatus: false,
      });
    }

    const pc = new RTCPeerConnection({
      iceServers: [],
      sdpSemantics: "unified-plan",
      iceCandidatePoolSize: 1,
    });
    rtcPcLabel(pc);
    pc.__carrotTrackSeen = false;
    pc.__carrotCreatedAtMs = Date.now();
    RTC_PENDING_PC = pc;
    setCarrotVisionPhase(CARROT_VISION_PHASE.RTC_CONNECTING, {
      reason: "rtc peer created",
      rtc: { state: "connecting", pending: true, pcLabel: rtcPcLabel(pc), trackSeen: false, liveTrack: false },
      updateRtcStatus: false,
    });
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
      setCarrotVisionPhase(CARROT_VISION_PHASE.FIRST_FRAME_WAITING, {
        reason: "rtc track received",
        rtc: { state: "track-received", pending: false, pcLabel: rtcPcLabel(pc), trackSeen: true, liveTrack: rtcHasLiveTrack() },
        updateRtcStatus: false,
      });

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
      requestCarrotVisionRender();
      if (retiringPc) {
        rtcClosePeer(retiringPc);
      }

      ev.track.addEventListener("unmute", () => {
        videoEl.play().catch(() => {});
        collectRtcPerfStats().catch(() => {});
        setCarrotVisionPhase(CARROT_VISION_PHASE.FIRST_FRAME_WAITING, {
          reason: "rtc track unmuted",
          rtc: { state: "track-unmuted", pending: false, pcLabel: rtcPcLabel(pc), trackSeen: true, liveTrack: true },
          updateRtcStatus: false,
        });
        requestCarrotVisionRender();
      });

      // Detect server-side track close → immediate recovery (guarded by PC identity)
      ev.track.addEventListener("ended", () => {
        rtcTrace("track_ended", {
          kind: ev.track?.kind || null,
          trackReadyState: ev.track?.readyState || null,
        }, pc);
        console.warn("[RTC] remote track ended");
        if ((RTC_PC === pc || RTC_PENDING_PC === pc) && shouldRunCarrotVisionRealtime() && !_rtcConnecting) {
          requestCarrotVisionRecovery("remote track ended", {
            action: "force-connect",
            force: true,
            statusText: getUIText("video_track_lost_reconnecting", "Video track lost, reconnecting..."),
          });
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
      if (state === "connected") {
        setCarrotVisionState({
          rtc: { state, pending: isPending, pcLabel: rtcPcLabel(pc), liveTrack: rtcHasLiveTrack() },
        }, { reason: "rtc connected" });
      }
      if (isActive) {
        collectRtcPerfStats().catch(() => {});
      }
      if (state === "failed" || state === "closed") {
        requestCarrotVisionRecovery(`rtc connection ${state}`, {
          action: isPending ? "retry-pending" : "retry-after-disconnect",
          pc,
          force: true,
          allowConnecting: true,
          allowPending: isPending,
          statusText: getUIText("reconnecting", "Reconnecting..."),
          rtcState: state,
          retryMs: RTC_RETRY_BASE_MS,
          cooldown: false,
        });
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
        requestCarrotVisionRecovery(`rtc ice ${state}`, {
          action: isPending ? "retry-pending" : "retry-after-disconnect",
          pc,
          force: true,
          allowConnecting: true,
          allowPending: isPending,
          statusText: getUIText("reconnecting", "Reconnecting..."),
          rtcState: `ice-${state}`,
          retryMs: RTC_RETRY_BASE_MS,
          cooldown: false,
        });
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

    const response = await fetchWithTimeout("/stream", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    }, RTC_STREAM_FETCH_TIMEOUT_MS);

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
    setCarrotVisionPhase(CARROT_VISION_PHASE.TRACK_WAITING, {
      reason: "rtc answer applied",
      rtc: { state: "track-waiting", pending: true, pcLabel: rtcPcLabel(pc), trackSeen: false, liveTrack: false },
      updateRtcStatus: false,
    });
    rtcArmTrackTimeout(RTC_INITIAL_TRACK_TIMEOUT_MS, pc);
  } catch (e) {
    rtcTrace("connect_error", {
      message: e?.message || String(e),
    }, RTC_PENDING_PC || previousPc || RTC_PC);
    rtcStatusSet("error: " + e.message);
    requestCarrotVisionRecovery(e?.message || "rtc connect error", {
      action: "retry-after-disconnect",
      pc: RTC_PENDING_PC || RTC_PC,
      force: true,
      allowConnecting: true,
      allowPending: true,
      statusText: getUIText("reconnecting", "Reconnecting..."),
      rtcState: "error",
      retryMs: RTC_RETRY_BASE_MS,
      cooldown: false,
    });
  } finally {
    _rtcConnecting = false;
  }
}

function startCarrotVisionHealthWatch() {
  if (CARROT_VISION_HEALTH_T) return;
  CARROT_VISION_HEALTH_T = setInterval(checkCarrotVisionHealth, CARROT_VISION_HEALTH_POLL_MS);
}

function stopCarrotVisionHealthWatch() {
  if (!CARROT_VISION_HEALTH_T) return;
  clearInterval(CARROT_VISION_HEALTH_T);
  CARROT_VISION_HEALTH_T = null;
}

function checkCarrotVisionHealth() {
  if (!shouldRunCarrotVisionRealtime()) {
    stopCarrotVisionHealthWatch();
    return;
  }

  const pendingPc = RTC_PENDING_PC;
  if (pendingPc) {
    const createdAt = Number(pendingPc.__carrotCreatedAtMs || 0);
    if (createdAt > 0 && Date.now() - createdAt > RTC_PENDING_STALE_MS) {
      console.warn("[RTC] pending peer stale, forcing retry", rtcBuildTraceSnapshot(pendingPc));
      requestCarrotVisionRecovery("rtc pending stale", {
        action: "retry-pending",
        pc: pendingPc,
        force: true,
        allowConnecting: true,
        allowPending: true,
        statusText: getUIText("reconnecting", "Reconnecting..."),
        rtcState: "pending-stale",
        retryMs: 0,
        cooldown: false,
      });
      return;
    }
  }

  if (_rtcConnecting || RTC_PENDING_PC) return;
  if (!RTC_PC || !rtcHasLiveTrack()) {
    requestCarrotVisionRecovery("health missing rtc track", {
      action: "force-connect",
      force: true,
      statusText: getUIText("connecting", "Connecting..."),
      rtcState: "missing-track",
      cooldown: false,
    });
    return;
  }
}


function rtcShouldConnect() {
  return shouldRunCarrotVisionRealtime() && !_rtcConnecting && (!RTC_PC || !rtcHasLiveTrack());
}

function rtcResetFailCount() {
  RTC_FAIL_COUNT = 0;
}

function rtcScheduleResumeIfConnected(reason = "network resumed") {
  if (shouldRunCarrotVisionRealtime() && RTC_PC && !_rtcConnecting) {
    rtcScheduleResumeHealthCheck(reason);
  }
}

function rtcHandleVisibilityChange() {
  if (document.hidden) {
    const video = getRtcVideoElement();
    RTC_VISIBILITY_STATE.hiddenAtMs = Date.now();
    RTC_VISIBILITY_STATE.currentTimeAtHide = Number(video?.currentTime || 0);
    rtcExitPictureInPicture();
    return;
  }
  rtcScheduleResumeIfConnected("returned visible");
  if (shouldRunCarrotVisionRealtime() && RTC_PC && !_rtcConnecting) {
    collectRtcPerfStats().catch(() => {});
  }
}

window.CarrotVisionRtc = {
  bindVideoEvents: rtcBindVideoEvents,
  cancelRecovery: rtcCancelRecovery,
  cancelResumeCheck: rtcCancelResumeCheck,
  cancelRetry: rtcCancelRetry,
  captureVideoHoldFrame: rtcCaptureVideoHoldFrame,
  collectPerfStats: collectRtcPerfStats,
  connectOnce: rtcConnectOnce,
  disconnect: rtcDisconnect,
  disarmTrackTimeout: rtcDisarmTrackTimeout,
  exitPictureInPicture: rtcExitPictureInPicture,
  getVideoElement: getRtcVideoElement,
  hasLiveTrack: rtcHasLiveTrack,
  handleVisibilityChange: rtcHandleVisibilityChange,
  resetFailCount: rtcResetFailCount,
  scheduleResumeIfConnected: rtcScheduleResumeIfConnected,
  shouldConnect: rtcShouldConnect,
  startHealthWatch: startCarrotVisionHealthWatch,
  startPerfPolling: startRtcPerfPolling,
  statusSet: rtcStatusSet,
  stopHealthWatch: stopCarrotVisionHealthWatch,
  stopPerfPolling: stopRtcPerfPolling,
};

Object.assign(window, {
  collectRtcPerfStats,
  getRtcVideoElement,
  requestCarrotVisionRecovery,
  rtcBindVideoEvents,
  rtcCancelRecovery,
  rtcCancelResumeCheck,
  rtcCancelRetry,
  rtcCaptureVideoHoldFrame,
  rtcConnectOnce,
  rtcDisconnect,
  rtcDisarmTrackTimeout,
  rtcExitPictureInPicture,
  rtcHandleVisibilityChange,
  rtcHasLiveTrack,
  rtcResetFailCount,
  rtcScheduleResumeIfConnected,
  rtcShouldConnect,
  rtcStatusSet,
  startCarrotVisionHealthWatch,
  startRtcPerfPolling,
  stopCarrotVisionHealthWatch,
  stopRtcPerfPolling,
});
