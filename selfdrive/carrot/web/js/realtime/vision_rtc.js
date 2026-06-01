/* Carrot Vision WebRTC runtime.
 * Owns only the road-camera WebRTC connection, RTC health, hold-frame, and RTC stats.
 */
var CARROT_VISION_PHASE = window.CarrotVisionPhase;
var CARROT_VISION_STATE = window.CarrotVisionState;
var setCarrotVisionPhase = window.CarrotVisionSetPhase;
var setCarrotVisionState = window.CarrotVisionSetState;

const RTC_STATS_POLL_MS = 1000;
// Tolerance philosophy: a frame stall while the PC/ICE is still connected and
// the track has not ended is almost always TRANSIENT (a brief source-side
// encoder/CPU hiccup on the comma device, or a momentary viewer main-thread
// jank) and self-heals the instant frames resume — no reconnect needed.
// A full reconnect re-runs ICE + /stream + spawns a fresh webrtcd session,
// and webrtcd keeps the old session alive up to ~4s during handover, doubling
// the encode/RTP load on the device and STARVING the very pipeline that
// stalled. That turns one transient hiccup into a self-sustaining
// stall->reconnect->handover->stall loop ("once it starts it keeps dropping",
// even on a healthy network).
// So these stall windows are deliberately generous: hold the last frame and
// wait for the source to resume; only fall back to a full reconnect as a
// last resort. Genuine permanent failures (ICE/connection failed|closed,
// remote track ended) still reconnect immediately via their own handlers.
const RTC_FREEZE_MAX_STALL_SAMPLES = 20;          // ~20s live-connection stall before last-resort reconnect (was 8s)
const RTC_INITIAL_FRAME_MAX_STALL_SAMPLES = 12;   // ~12s for first frame before reconnect (was 5s)
const RTC_FREEZE_CURRENT_TIME_EPSILON = 0.05;
const RTC_FREEZE_RECOVERY_COOLDOWN_MS = 4000;
const RTC_RESUME_PROGRESS_CHECK_MS = 900;
// These speed up a reconnect that is ALREADY happening (genuine failure), so
// the recovered video comes back sooner. They do NOT shorten stall *detection*
// (that's the FREEZE/FRAME thresholds above, kept patient to avoid the hotspot
// reconnect loop). Safe to keep snappy because the retry backoff still grows on
// persistent failure.
const RTC_RETRY_BASE_MS = 350;                    // first retry delay after a failed/closed peer (was 700)
const RTC_ICE_GATHER_TIMEOUT_MS = 700;            // host-only candidates gather near-instantly; tighter cap (was 1200)
const RTC_INITIAL_TRACK_TIMEOUT_MS = 4000;        // track should arrive fast; give a loaded device a little more slack (was 2800)
const RTC_INITIAL_FRAME_TIMEOUT_MS = 12000;       // align with the first-frame stall window above (was 6500)
const RTC_STREAM_FETCH_TIMEOUT_MS = 6500;
const RTC_PENDING_STALE_MS = 12000;               // align pending-peer stale with the first-frame window (was 9000)
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
  lastPacketsLost: null,
  lastPacketsReceived: null,
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
    keyFramesDecoded: RTC_PERF_STATE.inbound?.keyFramesDecoded ?? null,
    fps: RTC_PERF_STATE.inbound?.framesPerSecond ?? null,
    packetsLost: RTC_PERF_STATE.inbound?.packetsLost ?? null,
    lossPct: RTC_PERF_STATE.network?.lossPct ?? null,
    jitterMs: RTC_PERF_STATE.network?.jitterMs ?? null,
    bitrateMbps: RTC_PERF_STATE.network?.bitrateMbps ?? null,
    rttMs: RTC_PERF_STATE.network?.rttMs ?? null,
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
  RTC_RATE_STATE.lastPacketsLost = null;
  RTC_RATE_STATE.lastPacketsReceived = null;
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
  const keyFramesDecoded = "keyFramesDecoded" in statsReport ? Number(statsReport.keyFramesDecoded ?? 0) : null;
  return {
    codec: codecReport?.mimeType || codecReport?.id || "",
    inbound: {
      framesDecoded: Number(statsReport.framesDecoded ?? 0),
      keyFramesDecoded: Number.isFinite(Number(keyFramesDecoded)) ? Number(keyFramesDecoded) : null,
      framesDropped: Number(statsReport.framesDropped ?? 0),
      framesPerSecond: Number(statsReport.framesPerSecond ?? 0),
      frameWidth: Number(statsReport.frameWidth ?? 0),
      frameHeight: Number(statsReport.frameHeight ?? 0),
      jitter: Number(statsReport.jitter ?? 0),
      bytesReceived: Number(statsReport.bytesReceived ?? 0),
      packetsReceived: Number(statsReport.packetsReceived ?? 0),
      packetsLost: Number(statsReport.packetsLost ?? 0),
      nackCount: Number(statsReport.nackCount ?? 0),
      pliCount: Number(statsReport.pliCount ?? 0),
      firCount: Number(statsReport.firCount ?? 0),
      decoderImplementation: statsReport.decoderImplementation || "",
      freezeCount: Number(statsReport.freezeCount ?? 0),
      totalFreezesDuration: Number(statsReport.totalFreezesDuration ?? 0),
    },
  };
}

function extractRtcTransportStats(statsMap) {
  let selectedPair = null;
  statsMap.forEach((report) => {
    if (report?.type !== "transport" || !report.selectedCandidatePairId) return;
    selectedPair = statsMap.get(report.selectedCandidatePairId) || selectedPair;
  });
  statsMap.forEach((report) => {
    if (selectedPair) return;
    if (report?.type !== "candidate-pair") return;
    if (!(report.selected === true || report.nominated === true || report.state === "succeeded")) return;
    if (!selectedPair || report.selected === true || report.nominated === true) {
      selectedPair = report;
    }
  });
  const localCandidate = selectedPair?.localCandidateId ? statsMap.get(selectedPair.localCandidateId) : null;
  const remoteCandidate = selectedPair?.remoteCandidateId ? statsMap.get(selectedPair.remoteCandidateId) : null;

  return {
    rttMs: Number.isFinite(Number(selectedPair?.currentRoundTripTime))
      ? Number(selectedPair.currentRoundTripTime) * 1000
      : null,
    availableIncomingBitrate: Number.isFinite(Number(selectedPair?.availableIncomingBitrate))
      ? Number(selectedPair.availableIncomingBitrate)
      : null,
    protocol: localCandidate?.protocol || remoteCandidate?.protocol || selectedPair?.protocol || "",
    localCandidateType: localCandidate?.candidateType || "",
    remoteCandidateType: remoteCandidate?.candidateType || "",
    localAddress: localCandidate?.address || localCandidate?.ip || "",
    remoteAddress: remoteCandidate?.address || remoteCandidate?.ip || "",
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

function computeRtcPacketLoss(inboundStats) {
  const nextLost = Number(inboundStats?.packetsLost);
  const nextReceived = Number(inboundStats?.packetsReceived);
  if (!Number.isFinite(nextLost) || !Number.isFinite(nextReceived)) {
    return { lossPct: null, lostDelta: null, receivedDelta: null };
  }

  const prevLost = RTC_RATE_STATE.lastPacketsLost;
  const prevReceived = RTC_RATE_STATE.lastPacketsReceived;
  RTC_RATE_STATE.lastPacketsLost = nextLost;
  RTC_RATE_STATE.lastPacketsReceived = nextReceived;

  if (!Number.isFinite(prevLost) || !Number.isFinite(prevReceived)) {
    return { lossPct: null, lostDelta: null, receivedDelta: null };
  }

  const lostDelta = nextLost - prevLost;
  const receivedDelta = nextReceived - prevReceived;
  const totalDelta = lostDelta + receivedDelta;
  const lossPct = totalDelta > 0 ? Math.max(0, lostDelta) / totalDelta * 100 : null;
  return {
    lossPct: Number.isFinite(Number(lossPct)) ? lossPct : null,
    lostDelta: Number.isFinite(Number(lostDelta)) ? lostDelta : null,
    receivedDelta: Number.isFinite(Number(receivedDelta)) ? receivedDelta : null,
  };
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
  const packetLoss = computeRtcPacketLoss(inboundStats);
  const transport = extractRtcTransportStats(statsMap);
  const jitterMs = Number.isFinite(Number(inboundStats?.jitter)) ? Number(inboundStats.jitter) * 1000 : null;
  return {
    resolutionLabel,
    bitrateMbps: Number.isFinite(Number(bitrateMbps)) ? Number(bitrateMbps) : null,
    rttMs: Number.isFinite(Number(transport.rttMs)) ? Number(transport.rttMs) : null,
    jitterMs: Number.isFinite(Number(jitterMs)) ? Number(jitterMs) : null,
    lossPct: Number.isFinite(Number(packetLoss.lossPct)) ? Number(packetLoss.lossPct) : null,
    lostDelta: Number.isFinite(Number(packetLoss.lostDelta)) ? Number(packetLoss.lostDelta) : null,
    receivedDelta: Number.isFinite(Number(packetLoss.receivedDelta)) ? Number(packetLoss.receivedDelta) : null,
    availableIncomingMbps: Number.isFinite(Number(transport.availableIncomingBitrate))
      ? Number(transport.availableIncomingBitrate) / 1_000_000
      : null,
    protocol: transport.protocol || "",
    localCandidateType: transport.localCandidateType || "",
    remoteCandidateType: transport.remoteCandidateType || "",
    localAddress: transport.localAddress || "",
    remoteAddress: transport.remoteAddress || "",
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
let RTC_RETRY_T = null;
let RTC_WAIT_TRACK_T = null;
let RTC_WAIT_FIRST_FRAME_T = null;
let RTC_WAIT_FIRST_FRAME_PC = null;
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

function rtcVideoHasRenderableFrame(video = getRtcVideoElement()) {
  if (!video) return false;
  const width = Number(video.videoWidth || 0);
  const height = Number(video.videoHeight || 0);
  if (!Number.isFinite(width) || !Number.isFinite(height) || width <= 0 || height <= 0) return false;
  return Number(video.readyState || 0) >= 2;
}

// Single owner of the live / first-frame phase transition.
// home_drive (the renderer) is the authority on whether a real camera frame is
// actually on screen, so it REPORTS renderability here instead of writing the
// phase itself. This collapses the old home_drive/vision_rtc dual-ownership of
// "ready"/"first-frame-waiting" into one writer, and conveniently ties the
// freeze watchdog's progress reset to actual on-screen frames.
function rtcReportCameraRenderable(renderable) {
  if (!shouldRunCarrotVisionRealtime()) return;
  if (renderable) {
    // A renderable frame during a reconnect is the STALE last frame still held
    // in the <video> element, not live video. Promoting it to READY would hide
    // the "reconnecting" status (frozen screen with no message — the reported
    // symptom). Only go live when the peer connection is genuinely up and not
    // mid-reconnect. During a brief Phase-1 hold the connection is still live,
    // so we intentionally stay READY and keep showing the last frame without an
    // alarming message.
    if (!RTC_PC || _rtcConnecting || RTC_PENDING_PC || !rtcConnectionLooksLive(RTC_PC) || !rtcHasLiveTrack()) {
      return;
    }
    // NOTE: do not reset stallSamples here — a frozen frame is still
    // "renderable", so resetting on every render would defeat the freeze
    // watchdog. Progress detection (framesDecoded/currentTime) owns that reset.
    RTC_FREEZE_STATE.everDecodedFrame = true;
    rtcDisarmFirstFrameTimeout(RTC_PC);
    setCarrotVisionPhase(CARROT_VISION_PHASE.READY, {
      reason: "camera frame renderable",
      updateRtcStatus: false,
      render: false,
    });
  } else if (rtcHasLiveTrack()) {
    // Track present but no paintable frame yet — keep this as a first-frame
    // wait. Guarded by rtcHasLiveTrack() so it does not override an active
    // reconnect's "recovering" status.
    setCarrotVisionPhase(CARROT_VISION_PHASE.FIRST_FRAME_WAITING, {
      reason: "camera stream waiting first frame",
      updateRtcStatus: false,
      render: false,
    });
  }
}

function rtcClosePeer(pc) {
  if (!pc) return;
  try { pc.ontrack = null; } catch {}
  try { pc.onconnectionstatechange = null; } catch {}
  try { pc.oniceconnectionstatechange = null; } catch {}
  try { pc.close(); } catch {}
  if (RTC_PC === pc) RTC_PC = null;
  if (RTC_PENDING_PC === pc) RTC_PENDING_PC = null;
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
  rtcDisarmFirstFrameTimeout();
  rtcCancelResumeCheck();
  rtcCancelRecovery();
  stopRtcPerfPolling();
  const activePc = RTC_PC;
  const pendingPc = RTC_PENDING_PC;
  RTC_PC = null;
  RTC_PENDING_PC = null;
  rtcClosePeer(pendingPc);
  rtcClosePeer(activePc);
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
  rtcDisarmFirstFrameTimeout(targetPc);
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

// ── Single recovery policy ──────────────────────────────────────────────
// Every failure detector funnels through here. Instead of each call site
// picking action/force/cooldown/retryMs ad hoc (which is how the recovery
// logic drifted into a dozen subtly-different policies), the *classification*
// decides how we recover. requestCarrotVisionRecovery() below stays the proven
// mechanic (timers, cooldown, standby, phase); rtcRecover() is the one place
// the policy lives.
//
//   dead          connection genuinely gone (ICE/connection failed|closed,
//                 track missing, resume health failed, connect error)
//                 → tear down + reconnect now, no cooldown
//   track-lost    track ended / died while the PC still looks connected
//                 → fresh peer, keep the last frame visible via standby
//   establishing  never reached "live": no track / no first frame / pending
//                 peer stale → discard the half-open peer and retry
//   stall         live connection but decoded frames stopped advancing.
//                 LAST RESORT only — the freeze watchdog reaches here only
//                 after the long hold window, since transient stalls
//                 self-heal without a reconnect. Keep the default cooldown.
function rtcRecover(kind, reason, extra = {}) {
  const targetPc = extra.pc || RTC_PENDING_PC || RTC_PC;
  const isPending = Boolean(targetPc && RTC_PENDING_PC === targetPc);
  const reconnecting = getUIText("reconnecting", "Reconnecting...");
  switch (kind) {
    case "dead":
      return requestCarrotVisionRecovery(reason, {
        action: isPending ? "retry-pending" : "retry-after-disconnect",
        pc: targetPc, force: true, allowConnecting: true, allowPending: true,
        statusText: extra.statusText || reconnecting,
        rtcState: extra.rtcState, retryMs: RTC_RETRY_BASE_MS, cooldown: false,
      });
    case "track-lost":
      return requestCarrotVisionRecovery(reason, {
        action: "force-connect", pc: targetPc, force: true,
        statusText: extra.statusText || getUIText("video_track_lost_reconnecting", "Video track lost, reconnecting..."),
        rtcState: extra.rtcState,
      });
    case "establishing":
      return requestCarrotVisionRecovery(reason, {
        action: isPending ? "retry-pending" : "retry-after-disconnect",
        pc: targetPc, force: true, allowConnecting: true, allowPending: true,
        statusText: extra.statusText || reconnecting,
        rtcState: extra.rtcState,
        retryMs: Number.isFinite(Number(extra.retryMs)) ? Number(extra.retryMs) : RTC_RETRY_BASE_MS,
        cooldown: false,
      });
    case "stall":
      return requestCarrotVisionRecovery(reason, {
        action: "force-connect", pc: targetPc, force: Boolean(extra.force),
        statusText: extra.statusText, rtcState: extra.rtcState,
      });
    default:
      return false;
  }
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
    rtcRecover("track-lost", getUIText("video_track_lost_reconnecting", "Video track lost, reconnecting..."), { pc });
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
  if (rtcVideoHasRenderableFrame(video)) {
    rtcDisarmFirstFrameTimeout(pc);
  }

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
    rtcRecover(
      "stall",
      RTC_FREEZE_STATE.everDecodedFrame
        ? getUIText("video_stalled_reconnecting", "Video stalled, reconnecting...")
        : getUIText("no_initial_frame_reconnecting", "No initial frame, reconnecting..."),
      { pc },
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
    rtcDisarmFirstFrameTimeout(RTC_PC);
    rtcClearVideoHold();
    collectRtcPerfStats().catch(() => {});
    requestCarrotVisionRender();
  });
  const handleRenderableVideoEvent = () => {
    if (rtcVideoHasRenderableFrame(video)) {
      rtcDisarmFirstFrameTimeout(RTC_PC);
    }
    requestCarrotVisionRender();
  };
  ["loadedmetadata", "loadeddata", "canplay", "resize"].forEach((eventName) => {
    video.addEventListener(eventName, handleRenderableVideoEvent);
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
    rtcRecover("establishing", "rtc track timeout", {
      pc: expectedPc,
      statusText: getUIText("no_track_retry", "No track, retry..."),
      rtcState: "track-timeout",
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

function rtcArmFirstFrameTimeout(ms = RTC_INITIAL_FRAME_TIMEOUT_MS, expectedPc = RTC_PC) {
  if (!expectedPc) return;
  if (RTC_WAIT_FIRST_FRAME_T) clearTimeout(RTC_WAIT_FIRST_FRAME_T);
  const timeoutMs = Number.isFinite(Number(ms)) ? Number(ms) : RTC_INITIAL_FRAME_TIMEOUT_MS;
  RTC_WAIT_FIRST_FRAME_PC = expectedPc;
  const timer = setTimeout(() => {
    if (RTC_WAIT_FIRST_FRAME_T !== timer || RTC_WAIT_FIRST_FRAME_PC !== expectedPc || RTC_PC !== expectedPc) return;
    RTC_WAIT_FIRST_FRAME_T = null;
    RTC_WAIT_FIRST_FRAME_PC = null;
    if (!shouldRunCarrotVisionRealtime() || _rtcConnecting || RTC_PENDING_PC) return;
    if (!rtcHasLiveTrack() || rtcVideoHasRenderableFrame()) return;
    rtcRecover("establishing", getUIText("no_initial_frame_reconnecting", "No initial frame, reconnecting..."), {
      pc: expectedPc,
      statusText: getUIText("no_initial_frame_reconnecting", "No initial frame, reconnecting..."),
      rtcState: "first-frame-timeout",
    });
  }, timeoutMs);
  RTC_WAIT_FIRST_FRAME_T = timer;
}

function rtcDisarmFirstFrameTimeout(expectedPc = null) {
  if (expectedPc && RTC_WAIT_FIRST_FRAME_PC && RTC_WAIT_FIRST_FRAME_PC !== expectedPc) return;
  if (RTC_WAIT_FIRST_FRAME_T) {
    clearTimeout(RTC_WAIT_FIRST_FRAME_T);
    RTC_WAIT_FIRST_FRAME_T = null;
  }
  RTC_WAIT_FIRST_FRAME_PC = null;
}

function rtcScheduleResumeHealthCheck(reason = "returned visible") {
  rtcCancelResumeCheck();
  RTC_RESUME_CHECK_T = setTimeout(async () => {
    RTC_RESUME_CHECK_T = null;
    if (!shouldRunCarrotVisionRealtime() || _rtcConnecting || RTC_PENDING_PC || !RTC_PC) return;
    if (!rtcConnectionLooksLive(RTC_PC) || !rtcHasLiveTrack()) {
      rtcRecover("dead", `${reason}, reconnecting...`, {
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
  let previousPc = RTC_PC;
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

    // No standby PC. If a working stream exists, freeze its last frame on the
    // hold canvas so the view doesn't go black, then tear the old peer down.
    // webrtcd already runs its own old-session handover (retires the previous
    // session ~4s after the new one connects), so a client-side standby peer
    // was redundant double-handover.
    const hadLiveStream = Boolean(previousPc && rtcHasLiveTrack());
    if (hadLiveStream) rtcCaptureVideoHoldFrame();
    await rtcDisconnect({ keepVideo: true });
    previousPc = null;
    if (hadLiveStream) {
      rtcStatusSet(getUIText("reconnecting", "Reconnecting..."));
      setCarrotVisionPhase(CARROT_VISION_PHASE.RECOVERING, {
        reason: "rtc reconnect",
        rtc: { state: "reconnecting", pending: false, liveTrack: false, pcLabel: "none", trackSeen: false },
        updateRtcStatus: false,
      });
    } else {
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
      hadLiveStream,
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
      try { await videoEl.play(); } catch (e) { console.log("[RTC] play() failed", e); }
      rtcStatusSet("track: " + ev.track.kind);
      rtcDisarmTrackTimeout(pc);
      rtcArmFirstFrameTimeout(RTC_INITIAL_FRAME_TIMEOUT_MS, pc);
      RTC_FAIL_COUNT = 0;
      rtcResetFreezeWatchdog();
      rtcClearVideoHold();
      startRtcPerfPolling(true);
      collectRtcPerfStats().catch(() => {});
      requestCarrotVisionRender();

      ev.track.addEventListener("unmute", () => {
        videoEl.play().catch(() => {});
        collectRtcPerfStats().catch(() => {});
        rtcArmFirstFrameTimeout(RTC_INITIAL_FRAME_TIMEOUT_MS, pc);
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
          rtcRecover("track-lost", "remote track ended", { pc });
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
        rtcRecover("dead", `rtc connection ${state}`, { pc, rtcState: state });
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
        rtcRecover("dead", `rtc ice ${state}`, { pc, rtcState: `ice-${state}` });
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
    rtcRecover("dead", e?.message || "rtc connect error", { pc: RTC_PENDING_PC || RTC_PC, rtcState: "error" });
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
      rtcRecover("establishing", "rtc pending stale", { pc: pendingPc, rtcState: "pending-stale", retryMs: 0 });
      return;
    }
  }

  if (_rtcConnecting || RTC_PENDING_PC) return;
  if (!RTC_PC || !rtcHasLiveTrack()) {
    rtcRecover("dead", "health missing rtc track", {
      statusText: getUIText("connecting", "Connecting..."),
      rtcState: "missing-track",
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
  disarmFirstFrameTimeout: rtcDisarmFirstFrameTimeout,
  exitPictureInPicture: rtcExitPictureInPicture,
  getVideoElement: getRtcVideoElement,
  hasLiveTrack: rtcHasLiveTrack,
  handleVisibilityChange: rtcHandleVisibilityChange,
  reportCameraRenderable: rtcReportCameraRenderable,
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
  rtcDisarmFirstFrameTimeout,
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
