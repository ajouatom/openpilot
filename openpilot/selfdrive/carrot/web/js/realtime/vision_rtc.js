/* Carrot Vision WebRTC runtime.
 * Owns only the road-camera WebRTC connection, RTC health, hold-frame, and RTC stats.
 */
var CARROT_VISION_PHASE = window.CarrotVisionPhase;
var CARROT_VISION_STATE = window.CarrotVisionState;
var setCarrotVisionPhase = window.CarrotVisionSetPhase;
var setCarrotVisionState = window.CarrotVisionSetState;

const RTC_STATS_POLL_MS = 5000;
const RTC_STREAM_BUSY_CODE = "carrot_vision_busy";
const RTC_DEVICE_ID = String(window.CarrotStreamIdentity?.deviceId
  || window.CarrotStreamIdentity?.clientId
  || `carrot-${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`);
const RTC_TAB_ID = String(window.CarrotStreamIdentity?.tabId || "");
const RTC_CLIENT_ID = RTC_DEVICE_ID;
const RTC_CONNECTION_TRANSACTION_API = window.DriveVisionConnectionTransactions;
if (!RTC_CONNECTION_TRANSACTION_API?.create) {
  throw new Error("Carrot Vision connection transaction manager is unavailable");
}
const RTC_CONNECTION_TRANSACTIONS = RTC_CONNECTION_TRANSACTION_API.create();

function rtcCreateAttemptId() {
  try {
    const generated = window.CarrotStreamIdentity?.createAttemptId?.();
    if (generated) return String(generated).slice(0, 128);
  } catch (_) {}
  return `carrot-attempt-${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`.slice(0, 128);
}

function rtcIsConnectionAbort(error) {
  return error?.name === "AbortError"
    || error?.code === RTC_CONNECTION_TRANSACTION_API.abortCode;
}

function rtcAssertCurrentTransaction(transaction, pc = null, boundary = "") {
  RTC_CONNECTION_TRANSACTIONS.assertCurrent(
    transaction,
    boundary ? `Carrot Vision connection superseded after ${boundary}` : "",
  );
  if (pc && (
    pc.__carrotRtcGeneration !== transaction.generation
    || pc.__carrotAttemptId !== transaction.attemptId
    || (RTC_PENDING_PC !== pc && RTC_PC !== pc)
  )) {
    throw new RTC_CONNECTION_TRANSACTION_API.AbortError(
      boundary ? `Carrot Vision peer superseded after ${boundary}` : "Carrot Vision peer is stale",
    );
  }
  return transaction;
}

function rtcPeerTransactionIsCurrent(pc) {
  const transaction = pc?.__carrotTransaction || null;
  return Boolean(
    transaction
    && RTC_CONNECTION_TRANSACTIONS.matches(transaction, {
      generation: pc.__carrotRtcGeneration,
      attemptId: pc.__carrotAttemptId,
    })
    && (RTC_PENDING_PC === pc || RTC_PC === pc)
  );
}

class CarrotVisionStreamBusyError extends Error {
  constructor(message = "Carrot Vision is active on another device.") {
    super(message);
    this.name = "CarrotVisionStreamBusyError";
    this.code = RTC_STREAM_BUSY_CODE;
  }
}
const RTC_RAW_STATS_HISTORY_MAX = 60;
const RTC_RAW_STATS_KEEP_TYPES = new Set([
  "candidate-pair",
  "codec",
  "inbound-rtp",
  "local-candidate",
  "media-source",
  "remote-candidate",
  "remote-inbound-rtp",
  "track",
  "transport",
]);
// Tolerance philosophy: a frame stall while the PC/ICE is still connected and
// the track has not ended is almost always TRANSIENT (a brief source-side
// encoder/CPU hiccup on the comma device, or a momentary viewer main-thread
// jank) and self-heals the instant frames resume — no reconnect needed.
// A full reconnect re-runs ICE + /stream, so actual compositor frame progress
// is the single stall signal and reconnect remains the last resort.
// So these stall windows are deliberately generous: hold the last frame and
// wait for the source to resume; only fall back to a full reconnect as a
// last resort. Genuine permanent failures (ICE/connection failed|closed,
// remote track ended) still reconnect immediately via their own handlers.
const RTC_FRAME_STALL_NUDGE_MS = 8000;
const RTC_FRAME_STALL_RECONNECT_MS = 20000;
const RTC_FREEZE_RECOVERY_COOLDOWN_MS = 4000;
// Reconnects that are already necessary stay quick, while persistent failures
// still receive exponential retry backoff.
const RTC_RETRY_BASE_MS = 350;                    // first retry delay after a failed/closed peer (was 700)
const RTC_ICE_GATHER_TIMEOUT_MS = 700;            // host-only candidates gather near-instantly; tighter cap (was 1200)
const RTC_INITIAL_TRACK_TIMEOUT_MS = 3000;        // track should arrive quickly on host-only WebRTC
const RTC_INITIAL_FRAME_TIMEOUT_MS = 6000;        // ICE/track connected but no first frame -> recreate /stream session
const RTC_STREAM_FETCH_TIMEOUT_MS = 10000;
const CARROT_VISION_HEALTH_POLL_MS = 2000;
const RTC_PERF_STATE = {
  active: false,
  collectedAtMs: 0,
  connectionState: "idle",
  iceConnectionState: "new",
  codec: "",
  codecParams: "",
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
  lastPresentedFrameMs: 0,
  lastRecoveredAtMs: 0,
  lastPlaybackNudgeAtMs: 0,
  consecutiveRecoveries: 0,
  everDecodedFrame: false,
};
const RTC_RAW_STATS_HISTORY = [];
let RTC_RECOVERY_T = null;
let RTC_VIDEO_EVENTS_BOUND = false;
let RTC_WAIT_TRACK_PC = null;
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
  const detail = {
    ts: Date.now(),
    iso: new Date().toISOString(),
    event,
    pc: rtcPcLabel(pc),
    ...rtcBuildTraceSnapshot(pc),
    ...extra,
  };
  window.dispatchEvent(new CustomEvent("carrot:rtctrace", { detail }));
  if (RTC_TRACE_ENABLED) console.log("[RTC TRACE]", detail);
}

function rtcDescriptionSnapshot(description) {
  if (!description) return null;
  return {
    type: description.type || "",
    sdp: description.sdp || "",
    sdpBytes: String(description.sdp || "").length,
  };
}

function rtcPlainStatsReport(report) {
  const snapshot = {};
  try {
    Object.entries(report || {}).forEach(([key, value]) => {
      snapshot[key] = value;
    });
  } catch {}
  return snapshot;
}

function rtcImportantStatsReports(stats) {
  if (!stats) return [];
  const reports = [];
  try {
    stats.forEach((report) => {
      if (!report || !RTC_RAW_STATS_KEEP_TYPES.has(report.type)) return;
      if (report.type === "inbound-rtp" && report.kind && report.kind !== "video") return;
      if (report.type === "media-source" && report.kind && report.kind !== "video") return;
      if (report.type === "track" && report.kind && report.kind !== "video") return;
      reports.push(rtcPlainStatsReport(report));
    });
  } catch {}
  return reports;
}

function rtcTrackSnapshot(track) {
  if (!track) return null;
  let settings = null;
  let constraints = null;
  let capabilities = null;
  try { settings = track.getSettings?.() || null; } catch {}
  try { constraints = track.getConstraints?.() || null; } catch {}
  try { capabilities = track.getCapabilities?.() || null; } catch {}
  return {
    id: track.id || "",
    kind: track.kind || "",
    label: track.label || "",
    enabled: typeof track.enabled === "boolean" ? track.enabled : null,
    muted: typeof track.muted === "boolean" ? track.muted : null,
    readyState: track.readyState || "",
    contentHint: track.contentHint || "",
    settings,
    constraints,
    capabilities,
  };
}

function rtcStatsReportSnapshot(report) {
  return rtcPlainStatsReport(report);
}

function rtcMediaSnapshot(video = getRtcVideoElement()) {
  const stream = video?.srcObject || null;
  const tracks = typeof stream?.getTracks === "function" ? stream.getTracks().map(rtcTrackSnapshot) : [];
  return {
    video: {
      id: video?.id || "",
      width: Number(video?.videoWidth || 0),
      height: Number(video?.videoHeight || 0),
      readyState: Number(video?.readyState || 0),
      networkState: Number(video?.networkState || 0),
      currentTime: Number.isFinite(Number(video?.currentTime)) ? Number(video.currentTime) : null,
      paused: typeof video?.paused === "boolean" ? video.paused : null,
      muted: typeof video?.muted === "boolean" ? video.muted : null,
      srcObjectActive: typeof stream?.active === "boolean" ? stream.active : null,
      tracks,
    },
  };
}

function rtcPushRawStatsHistory(pc, stats, reason = "poll") {
  try {
    const video = getRtcVideoElement();
    RTC_RAW_STATS_HISTORY.push({
      capturedAtMs: Date.now(),
      capturedAtIso: new Date().toISOString(),
      reason,
      pcLabel: pc ? rtcPcLabel(pc) : null,
      pc: pc ? {
        connectionState: pc.connectionState || "",
        iceConnectionState: pc.iceConnectionState || "",
        iceGatheringState: pc.iceGatheringState || "",
        signalingState: pc.signalingState || "",
      } : null,
      trace: rtcBuildTraceSnapshot(pc),
      media: rtcMediaSnapshot(video),
      perf: {
        inbound: RTC_PERF_STATE.inbound,
        video: RTC_PERF_STATE.video,
        network: RTC_PERF_STATE.network,
        codec: RTC_PERF_STATE.codec,
        codecParams: RTC_PERF_STATE.codecParams,
      },
      stats: rtcImportantStatsReports(stats),
    });
    if (RTC_RAW_STATS_HISTORY.length > RTC_RAW_STATS_HISTORY_MAX) {
      RTC_RAW_STATS_HISTORY.splice(0, RTC_RAW_STATS_HISTORY.length - RTC_RAW_STATS_HISTORY_MAX);
    }
  } catch {}
}

async function rtcDiagnosticSnapshot() {
  const pc = RTC_PC || RTC_PENDING_PC || null;
  const video = getRtcVideoElement();
  let stats = [];
  let statsError = "";
  if (pc) {
    try {
      const report = await pc.getStats(null);
      stats = Array.from(report.values()).map(rtcStatsReportSnapshot);
    } catch (error) {
      statsError = error?.message || String(error);
    }
  }

  return {
    capturedAtMs: Date.now(),
    capturedAtIso: new Date().toISOString(),
    active: shouldRunCarrotVisionRealtime(),
    connecting: _rtcConnecting,
    failCount: RTC_FAIL_COUNT,
    retryArmed: Boolean(RTC_RETRY_T),
    trackTimeoutArmed: Boolean(RTC_WAIT_TRACK_T),
    firstFrameTimeoutArmed: Boolean(RTC_WAIT_FIRST_FRAME_T),
    pendingPc: RTC_PENDING_PC ? rtcPcLabel(RTC_PENDING_PC) : null,
    activePc: RTC_PC ? rtcPcLabel(RTC_PC) : null,
    transaction: RTC_CONNECTION_TRANSACTIONS.snapshot(),
    compactState: {
      active: Boolean(window.CarrotVisionRaw?.hasCompactState?.()),
    },
    trace: rtcBuildTraceSnapshot(pc),
    freezeState: { ...RTC_FREEZE_STATE },
    perfState: window.CarrotRtcPerf || null,
    visionState: window.CarrotVisionState || null,
    testState: window.CarrotVisionTestState || null,
    video: rtcMediaSnapshot(video).video,
    pc: pc ? {
      label: rtcPcLabel(pc),
      connectionState: pc.connectionState || "",
      iceConnectionState: pc.iceConnectionState || "",
      iceGatheringState: pc.iceGatheringState || "",
      signalingState: pc.signalingState || "",
      localDescription: rtcDescriptionSnapshot(pc.localDescription),
      remoteDescription: rtcDescriptionSnapshot(pc.remoteDescription),
    } : null,
    statsError,
    stats,
    rawStatsHistory: RTC_RAW_STATS_HISTORY.slice(-RTC_RAW_STATS_HISTORY_MAX),
  };
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
  RTC_FREEZE_STATE.lastPresentedFrameMs = 0;
  RTC_FREEZE_STATE.lastPlaybackNudgeAtMs = 0;
  RTC_FREEZE_STATE.everDecodedFrame = false;
}

function rtcCancelResumeCheck() {}

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
    networkState: Number(video.networkState || 0),
    currentTime: Number(video.currentTime || 0),
    paused: Boolean(video.paused),
    errorCode: video.error ? Number(video.error.code || 0) : 0,
    errorMessage: video.error ? String(video.error.message || "") : "",
  };
}

function extractRtcInboundVideoStats(statsReport, statsMap) {
  if (!statsReport) return { inbound: null, codec: "" };
  const codecReport = statsReport.codecId ? statsMap.get(statsReport.codecId) : null;
  const keyFramesDecoded = "keyFramesDecoded" in statsReport ? Number(statsReport.keyFramesDecoded ?? 0) : null;
  const framesReceived = "framesReceived" in statsReport ? Number(statsReport.framesReceived ?? 0) : null;
  return {
    codec: codecReport?.mimeType || codecReport?.id || "",
    codecFmtp: codecReport?.sdpFmtpLine || "",
    inbound: {
      framesDecoded: Number(statsReport.framesDecoded ?? 0),
      keyFramesDecoded: Number.isFinite(Number(keyFramesDecoded)) ? Number(keyFramesDecoded) : null,
      framesReceived: Number.isFinite(Number(framesReceived)) ? Number(framesReceived) : null,
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
  if (!pc || !rtcPeerTransactionIsCurrent(pc)) return;

  try {
    const collectedAtMs = Date.now();
    const stats = await pc.getStats(null);
    if (RTC_PC !== pc || !rtcPeerTransactionIsCurrent(pc)) return;
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
    RTC_PERF_STATE.codecParams = inbound.codecFmtp || "";
    RTC_PERF_STATE.inbound = inbound.inbound;
    RTC_PERF_STATE.video = readRtcVideoPlaybackQuality(video);
    RTC_PERF_STATE.network = buildRtcNetworkStats(inbound.inbound, RTC_PERF_STATE.video, stats, collectedAtMs);
    RTC_PERF_STATE.error = "";
    window.CarrotRtcPerf = RTC_PERF_STATE;
    rtcPushRawStatsHistory(pc, stats, "poll");
    _hudMarkDirty();
    emitCarrotRenderRequest({ force: false, overlayDirty: false, hudDirty: true });
  } catch (error) {
    if (RTC_PC !== pc || !rtcPeerTransactionIsCurrent(pc)) return;
    RTC_PERF_STATE.active = shouldRunCarrotVisionRealtime();
    RTC_PERF_STATE.collectedAtMs = Date.now();
    RTC_PERF_STATE.connectionState = pc.connectionState || "unknown";
    RTC_PERF_STATE.iceConnectionState = pc.iceConnectionState || "unknown";
    RTC_PERF_STATE.network = null;
    RTC_PERF_STATE.error = error?.message || String(error);
    window.CarrotRtcPerf = RTC_PERF_STATE;
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

function rtcOwnershipBlocked() {
  return Boolean(CARROT_VISION_STATE?.ownership?.blocked);
}

function rtcSetOwnershipBlocked(blocked, code = "", reason = "vision ownership") {
  setCarrotVisionState({
    ownership: {
      blocked: Boolean(blocked),
      code: blocked ? String(code || RTC_STREAM_BUSY_CODE) : "",
    },
  }, { reason, silent: true });
}

function rtcPrepareOwnershipRetry(reason = "passive ownership retry") {
  if (!rtcOwnershipBlocked()) return false;
  rtcSetOwnershipBlocked(false, "", reason);
  rtcCancelRetry();
  rtcCancelRecovery();
  return true;
}

async function rtcTakeOwnership(reason = "user takeover action") {
  rtcPrepareOwnershipRetry(reason);
  rtcCancelRetry();
  rtcCancelRecovery();
  if (!shouldRunCarrotVisionRealtime()) return false;
  await rtcDisconnect({ keepVideo: true });
  startCarrotVisionHealthWatch();
  await rtcConnectOnce({ force: true, takeover: true });
  return !rtcOwnershipBlocked();
}

function rtcMarkOwnershipBusy(message = "") {
  const transaction = RTC_CONNECTION_TRANSACTIONS.current();
  RTC_CONNECTION_TRANSACTIONS.cancel("vision stream busy", transaction);
  if (RTC_CONNECTING_TRANSACTION === transaction) RTC_CONNECTING_TRANSACTION = null;
  _rtcConnecting = false;
  rtcSetOwnershipBlocked(true, RTC_STREAM_BUSY_CODE, "vision stream busy");
  rtcCancelRetry();
  rtcCancelRecovery();
  rtcDisarmTrackTimeout();
  rtcDisarmFirstFrameTimeout();
  const pendingPc = RTC_PENDING_PC;
  if (pendingPc) rtcClosePeer(pendingPc);
  stopCarrotVisionHealthWatch();
  stopRtcPerfPolling();
  setCarrotVisionPhase(CARROT_VISION_PHASE.BUSY, {
    reason: "vision stream busy",
    statusText: getUIText("vision_stream_busy", "Carrot Vision is active on another device."),
    detailText: "",
    rtc: { state: "busy", pending: false, liveTrack: false, pcLabel: "none", trackSeen: false },
    updateRtcStatus: true,
  });
  rtcTrace("ownership_busy", { message: String(message || "") });
}

function rtcHasLiveTrack() {
  const video = getRtcVideoElement();
  const stream = video?.srcObject;
  if (!stream) return false;
  // MediaStream.active can remain false while ICE/DTLS is still completing and
  // the newly delivered remote track is muted. Treat the attached, non-ended
  // track as present so the first-frame timeout owns this handshake window;
  // otherwise the health poll tears the peer down every two seconds.
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

function rtcReportCameraPresentedFrame() {
  if (!shouldRunCarrotVisionRealtime() || !RTC_PC || !rtcConnectionLooksLive(RTC_PC)) return;
  RTC_FREEZE_STATE.lastPresentedFrameMs = Date.now();
  RTC_FREEZE_STATE.consecutiveRecoveries = 0;
  RTC_FREEZE_STATE.everDecodedFrame = true;
  rtcDisarmFirstFrameTimeout(RTC_PC);
}

function rtcClosePeer(pc) {
  if (!pc) return;
  try { pc.__carrotFrameSyncChannel.onmessage = null; } catch {}
  try { pc.__carrotFrameSyncChannel.close(); } catch {}
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

function rtcApplyFrameSyncPacket(data) {
  const buffer = data instanceof ArrayBuffer
    ? data
    : (ArrayBuffer.isView(data) ? data.buffer.slice(data.byteOffset, data.byteOffset + data.byteLength) : null);
  if (!buffer || buffer.byteLength !== 12) return;
  const view = new DataView(buffer);
  if (view.getUint8(0) !== 0x43 || view.getUint8(1) !== 0x56
      || view.getUint8(2) !== 0x46 || view.getUint8(3) !== 0x31) return;
  const frameId = view.getUint32(4, false);
  const rtpTimestamp = view.getUint32(8, false);
  window.CarrotVisionFrameSync?.noteRtpFrameMapping?.(rtpTimestamp, frameId);
}

function rtcHandleFrameSyncMessage(event, pc = null) {
  const applyPacket = (data) => {
    if (pc && !rtcPeerTransactionIsCurrent(pc)) return;
    rtcApplyFrameSyncPacket(data);
  };
  if (event?.data instanceof Blob) {
    event.data.arrayBuffer().then(applyPacket).catch(() => {});
    return;
  }
  applyPacket(event?.data);
}

async function rtcDisconnect(options = {}) {
  const keepVideo = Boolean(options.keepVideo);
  const preserveTransaction = options.preserveTransaction || null;
  if (preserveTransaction && !RTC_CONNECTION_TRANSACTIONS.isCurrent(preserveTransaction)) {
    return false;
  }
  if (!preserveTransaction) {
    RTC_CONNECTION_TRANSACTIONS.cancel(options.reason || "rtc disconnect");
    RTC_CONNECTING_TRANSACTION = null;
    _rtcConnecting = false;
  }
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
  window.CarrotVisionFrameSync?.reset?.();
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
  return true;
}

function rtcConnectionLooksLive(pc = RTC_PC) {
  if (!pc) return false;
  return pc.connectionState === "connected" || pc.iceConnectionState === "connected" || pc.iceConnectionState === "completed";
}

function rtcCanResumeWithoutReconnect() {
  return Boolean(
    shouldRunCarrotVisionRealtime() &&
    !rtcOwnershipBlocked() &&
    RTC_PC &&
    !RTC_PENDING_PC &&
    !_rtcConnecting &&
    rtcConnectionLooksLive(RTC_PC) &&
    rtcHasLiveTrack() &&
    (rtcVideoHasRenderableFrame() || RTC_FREEZE_STATE.everDecodedFrame)
  );
}

function rtcNudgePlayback(pc, video, reason = "decode stalled while RTP advances") {
  if (!video?.srcObject || !rtcConnectionLooksLive(pc) || !rtcHasLiveTrack()) return;
  const now = Date.now();
  if (now - RTC_FREEZE_STATE.lastPlaybackNudgeAtMs < 2500) return;
  RTC_FREEZE_STATE.lastPlaybackNudgeAtMs = now;
  try {
    video.muted = true;
    video.playsInline = true;
    video.play().catch(() => {});
  } catch {}
  rtcTrace("playback_nudge", { reason }, pc);
  requestCarrotVisionRender();
}

function requestCarrotVisionRecovery(reason, options = {}) {
  if (rtcOwnershipBlocked()) return false;
  const force = Boolean(options.force);
  const allowConnecting = Boolean(options.allowConnecting);
  const allowPending = Boolean(options.allowPending);
  if (!shouldRunCarrotVisionRealtime() || (!allowConnecting && _rtcConnecting) || (!allowPending && RTC_PENDING_PC) || RTC_RECOVERY_T) return false;
  const now = Date.now();
  if (options.cooldown !== false && !force && (now - RTC_FREEZE_STATE.lastRecoveredAtMs < RTC_FREEZE_RECOVERY_COOLDOWN_MS)) return false;

  RTC_FREEZE_STATE.consecutiveRecoveries++;
  RTC_FREEZE_STATE.lastRecoveredAtMs = now;
  const action = options.action || "force-connect";
  const retryMs = Number.isFinite(Number(options.retryMs)) ? Number(options.retryMs) : RTC_RETRY_BASE_MS;
  const targetPc = options.pc || RTC_PENDING_PC || RTC_PC;
  const targetTransaction = targetPc?.__carrotTransaction || RTC_CONNECTION_TRANSACTIONS.current();
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
    if (targetTransaction && !RTC_CONNECTION_TRANSACTIONS.isCurrent(targetTransaction)) return;
    if (targetPc && !rtcPeerTransactionIsCurrent(targetPc)) return;
    if (options.capture !== false) rtcCaptureVideoHoldFrame();

    if (action === "retry-pending") {
      if (targetPc && RTC_PENDING_PC === targetPc) rtcClosePeer(targetPc);
      RTC_CONNECTION_TRANSACTIONS.cancel("retry pending connection", targetTransaction);
      if (RTC_CONNECTING_TRANSACTION === targetTransaction) RTC_CONNECTING_TRANSACTION = null;
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
  if (!shouldRunCarrotVisionRealtime() || rtcOwnershipBlocked()) return;
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
  if (!rtcPeerTransactionIsCurrent(expectedPc)) return;
  if (rtcPcSawTrack(expectedPc)) {
    rtcTrace("track_timeout_arm_skipped", { timeoutMs: ms, reason: "track already seen" }, expectedPc);
    return;
  }
  if (RTC_WAIT_TRACK_T) clearTimeout(RTC_WAIT_TRACK_T);
  RTC_WAIT_TRACK_PC = expectedPc;
  RTC_WAIT_TRACK_T = setTimeout(async () => {
    RTC_WAIT_TRACK_T = null;
    if (
      RTC_WAIT_TRACK_PC !== expectedPc
      || !rtcPeerTransactionIsCurrent(expectedPc)
      || (RTC_PC !== expectedPc && RTC_PENDING_PC !== expectedPc)
    ) return;
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
  if (!expectedPc || !rtcPeerTransactionIsCurrent(expectedPc)) return;
  if (RTC_WAIT_FIRST_FRAME_T) clearTimeout(RTC_WAIT_FIRST_FRAME_T);
  const timeoutMs = Number.isFinite(Number(ms)) ? Number(ms) : RTC_INITIAL_FRAME_TIMEOUT_MS;
  RTC_WAIT_FIRST_FRAME_PC = expectedPc;
  const timer = setTimeout(() => {
    if (
      RTC_WAIT_FIRST_FRAME_T !== timer
      || RTC_WAIT_FIRST_FRAME_PC !== expectedPc
      || RTC_PC !== expectedPc
      || !rtcPeerTransactionIsCurrent(expectedPc)
    ) return;
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
  if (
    !shouldRunCarrotVisionRealtime()
    || _rtcConnecting
    || RTC_PENDING_PC
    || !RTC_PC
    || !rtcPeerTransactionIsCurrent(RTC_PC)
  ) return;
  if (!rtcConnectionLooksLive(RTC_PC) || !rtcHasLiveTrack()) {
    rtcRecover("dead", `${reason}, reconnecting...`, {
      statusText: getUIText("reconnecting", "Reconnecting..."),
      rtcState: "resume-health-failed",
    });
    return;
  }
  const video = getRtcVideoElement();
  try { video?.play?.().catch(() => {}); } catch {}
  requestCarrotVisionRender();
}

async function waitIceComplete(pc, timeoutMs = RTC_ICE_GATHER_TIMEOUT_MS, signal = null) {
  if (signal?.aborted) {
    throw new RTC_CONNECTION_TRANSACTION_API.AbortError("Carrot Vision ICE gathering was cancelled");
  }
  if (pc.iceGatheringState === "complete") return;
  await new Promise((resolve, reject) => {
    let settled = false;
    const cleanup = () => {
      pc.removeEventListener("icegatheringstatechange", onchg);
      signal?.removeEventListener?.("abort", onabort);
      clearTimeout(t);
    };
    const finish = (callback) => {
      if (settled) return;
      settled = true;
      cleanup();
      callback();
    };
    const t = setTimeout(() => finish(resolve), timeoutMs);
    function onchg() {
      if (pc.iceGatheringState === "complete") {
        finish(resolve);
      }
    }
    function onabort() {
      finish(() => reject(new RTC_CONNECTION_TRANSACTION_API.AbortError(
        "Carrot Vision ICE gathering was cancelled",
      )));
    }
    pc.addEventListener("icegatheringstatechange", onchg);
    signal?.addEventListener?.("abort", onabort, { once: true });
  });
}

async function fetchWithTimeout(url, options = {}, timeoutMs = RTC_STREAM_FETCH_TIMEOUT_MS, externalSignal = null) {
  const upstreamSignal = externalSignal || options.signal || null;
  if (upstreamSignal?.aborted) {
    throw new RTC_CONNECTION_TRANSACTION_API.AbortError("Carrot Vision stream request was cancelled");
  }
  if (typeof AbortController === "undefined") {
    return fetch(url, options);
  }
  const controller = new AbortController();
  let timedOut = false;
  const abortFromUpstream = () => {
    try { controller.abort(upstreamSignal?.reason); } catch (_) { controller.abort(); }
  };
  upstreamSignal?.addEventListener?.("abort", abortFromUpstream, { once: true });
  const timer = setTimeout(() => {
    timedOut = true;
    try { controller.abort(); } catch (_) {}
  }, timeoutMs);
  try {
    return await fetch(url, { ...options, signal: controller.signal });
  } catch (error) {
    if (upstreamSignal?.aborted) {
      throw new RTC_CONNECTION_TRANSACTION_API.AbortError("Carrot Vision stream request was cancelled");
    }
    if (timedOut) {
      const timeoutError = new Error(`Carrot Vision stream request timed out after ${timeoutMs}ms`);
      timeoutError.name = "TimeoutError";
      throw timeoutError;
    }
    throw error;
  } finally {
    clearTimeout(timer);
    upstreamSignal?.removeEventListener?.("abort", abortFromUpstream);
  }
}

let _rtcConnecting = false;
let RTC_CONNECTING_TRANSACTION = null;

async function rtcConnectOnce(options = {}) {
  const force = Boolean(options.force);
  const takeover = options.takeover === true;
  if (!shouldRunCarrotVisionRealtime()) return;
  if (rtcOwnershipBlocked() && !takeover) return;
  if ((_rtcConnecting || RTC_PENDING_PC) && !force) return;
  if (!force && RTC_PC && (RTC_PC.connectionState === "connected" || RTC_PC.connectionState === "connecting") && rtcHasLiveTrack()) return;

  if (takeover) rtcSetOwnershipBlocked(false, "", "vision takeover requested");

  const attemptId = rtcCreateAttemptId();
  const transaction = RTC_CONNECTION_TRANSACTIONS.begin({
    attemptId,
    supersedeReason: force ? "forced connection replacement" : "new connection attempt",
  });
  RTC_CONNECTING_TRANSACTION = transaction;
  _rtcConnecting = true;
  let previousPc = RTC_PC;
  let pc = null;
  try {
    rtcCancelRetry();
    rtcDisarmTrackTimeout();
    rtcCancelResumeCheck();
    rtcCancelRecovery();
    rtcTrace("connect_start", {
      force,
      takeover,
      deviceId: RTC_DEVICE_ID,
      tabId: RTC_TAB_ID,
      attemptId,
      hasPreviousPc: Boolean(previousPc),
      hasLiveTrack: rtcHasLiveTrack(),
    }, previousPc || RTC_PC);

    // No standby PC. Freeze the last frame on the hold canvas, then tear the old
    // peer down before creating its replacement so recovery never doubles RTP.
    const hadLiveStream = Boolean(previousPc && rtcHasLiveTrack());
    if (hadLiveStream) rtcCaptureVideoHoldFrame();
    await rtcDisconnect({
      keepVideo: true,
      preserveTransaction: transaction,
      reason: "replace peer for current connection attempt",
    });
    rtcAssertCurrentTransaction(transaction, null, "previous peer disconnect");
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

    pc = new RTCPeerConnection({
      iceServers: [],
      sdpSemantics: "unified-plan",
      iceCandidatePoolSize: 1,
    });
    rtcPcLabel(pc);
    pc.__carrotTrackSeen = false;
    pc.__carrotCreatedAtMs = Date.now();
    pc.__carrotAttemptId = attemptId;
    pc.__carrotRtcGeneration = transaction.generation;
    pc.__carrotTransaction = transaction;
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

    // teleoprtc uses the "data" label for the offer-side channel. Carrot Vision
    // carries only a 12-byte RTP timestamp/source-frame mapping on it; state
    // remains on the single Compact WebSocket.
    const frameSyncChannel = pc.createDataChannel("data", {
      ordered: false,
      maxRetransmits: 0,
    });
    frameSyncChannel.binaryType = "arraybuffer";
    frameSyncChannel.onmessage = (event) => rtcHandleFrameSyncMessage(event, pc);
    pc.__carrotFrameSyncChannel = frameSyncChannel;

    pc.addTransceiver("video", { direction: "recvonly" });

    pc.ontrack = async (ev) => {
      if (RTC_PENDING_PC !== pc || !rtcPeerTransactionIsCurrent(pc)) return;
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
      RTC_CONNECTION_TRANSACTIONS.markActive(transaction);
      rtcStatusSet("track: " + ev.track.kind);
      rtcDisarmTrackTimeout(pc);
      rtcArmFirstFrameTimeout(RTC_INITIAL_FRAME_TIMEOUT_MS, pc);
      RTC_FAIL_COUNT = 0;
      rtcResetFreezeWatchdog();
      rtcClearVideoHold();
      startRtcPerfPolling(true);
      collectRtcPerfStats().catch(() => {});
      requestCarrotVisionRender();
      videoEl.play().catch((e) => console.log("[RTC] play() failed", e));

      ev.track.addEventListener("unmute", () => {
        if (RTC_PC !== pc || !rtcPeerTransactionIsCurrent(pc)) return;
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
        if (!rtcPeerTransactionIsCurrent(pc)) return;
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
      if (!rtcPeerTransactionIsCurrent(pc)) return;
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
        window.CarrotVisionNetworkRecovery?.reportTransportFailure?.(`rtc connection ${state}`);
        rtcRecover("dead", `rtc connection ${state}`, { pc, rtcState: state });
      }
    };

    pc.oniceconnectionstatechange = () => {
      if (!rtcPeerTransactionIsCurrent(pc)) return;
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
        window.CarrotVisionNetworkRecovery?.reportTransportFailure?.(`rtc ice ${state}`);
        rtcRecover("dead", `rtc ice ${state}`, { pc, rtcState: `ice-${state}` });
      }
    };

    const offer = await pc.createOffer();
    rtcAssertCurrentTransaction(transaction, pc, "offer creation");
    await pc.setLocalDescription(offer);
    rtcAssertCurrentTransaction(transaction, pc, "local description");
    await waitIceComplete(pc, RTC_ICE_GATHER_TIMEOUT_MS, transaction.signal);
    rtcAssertCurrentTransaction(transaction, pc, "ICE gathering");
    rtcTrace("offer_ready", {
      localSdpBytes: pc.localDescription?.sdp?.length || 0,
    }, pc);

    const body = {
      sdp: pc.localDescription.sdp,
      cameras: ["road"],
      bridge_services_in: [],
      bridge_services_out: [],
      client_id: RTC_CLIENT_ID,
      device_id: RTC_DEVICE_ID,
      tab_id: RTC_TAB_ID,
      attempt_id: attemptId,
      takeover,
      carrot_state: true,
    };

    const response = await fetchWithTimeout("/stream", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    }, RTC_STREAM_FETCH_TIMEOUT_MS, transaction.signal);
    rtcAssertCurrentTransaction(transaction, pc, "stream response");

    const responseText = await response.text().catch(() => "");
    rtcAssertCurrentTransaction(transaction, pc, "stream response body");
    let responsePayload = null;
    try {
      responsePayload = responseText ? JSON.parse(responseText) : null;
    } catch (_) {}
    if (!response.ok) {
      if (response.status === 409 && responsePayload?.code === RTC_STREAM_BUSY_CODE) {
        throw new CarrotVisionStreamBusyError(responsePayload?.error || responseText);
      }
      throw new Error("stream http " + response.status + " " + responseText);
    }

    rtcSetOwnershipBlocked(false, "", "vision stream accepted");
    const answer = responsePayload;
    if (!answer || !answer.sdp) throw new Error("bad answer");
    if (answer.attempt_id && String(answer.attempt_id) !== transaction.attemptId) {
      const mismatchError = new Error("Carrot Vision stream answer belongs to an older attempt");
      mismatchError.code = "vision-answer-attempt-mismatch";
      throw mismatchError;
    }
    rtcTrace("answer_received", {
      remoteSdpBytes: answer.sdp?.length || 0,
      answerType: answer.type || "answer",
    }, pc);

    await pc.setRemoteDescription({ type: answer.type || "answer", sdp: answer.sdp });
    rtcAssertCurrentTransaction(transaction, pc, "remote description");
    rtcTrace("answer_applied", {}, pc);
    if (!rtcPcSawTrack(pc)) {
      rtcStatusSet(getUIText("connected_waiting_track", "Connected, waiting track..."));
      setCarrotVisionPhase(CARROT_VISION_PHASE.TRACK_WAITING, {
        reason: "rtc answer applied",
        rtc: { state: "track-waiting", pending: true, pcLabel: rtcPcLabel(pc), trackSeen: false, liveTrack: false },
        updateRtcStatus: false,
      });
      rtcArmTrackTimeout(RTC_INITIAL_TRACK_TIMEOUT_MS, pc);
    }
  } catch (e) {
    if (!RTC_CONNECTION_TRANSACTIONS.isCurrent(transaction) || rtcIsConnectionAbort(e)) {
      RTC_CONNECTION_TRANSACTIONS.cancel("connection attempt aborted", transaction);
      rtcTrace("connect_aborted", {
        message: e?.message || String(e),
        generation: transaction.generation,
        attemptId: transaction.attemptId,
      }, pc || previousPc);
      rtcClosePeer(pc);
      return;
    }
    if (e instanceof CarrotVisionStreamBusyError || e?.code === RTC_STREAM_BUSY_CODE) {
      rtcMarkOwnershipBusy(e?.message || "");
      return;
    }
    rtcTrace("connect_error", {
      message: e?.message || String(e),
    }, pc || RTC_PENDING_PC || previousPc || RTC_PC);
    window.CarrotVisionNetworkRecovery?.reportTransportFailure?.(
      e?.message || "rtc connection error",
    );
    rtcStatusSet("error: " + e.message);
    rtcRecover("dead", e?.message || "rtc connect error", { pc: pc || RTC_PENDING_PC || RTC_PC, rtcState: "error" });
  } finally {
    if (RTC_CONNECTING_TRANSACTION === transaction) {
      RTC_CONNECTING_TRANSACTION = null;
      _rtcConnecting = false;
    }
    if (!RTC_CONNECTION_TRANSACTIONS.isCurrent(transaction)) rtcClosePeer(pc);
  }
}

function startCarrotVisionHealthWatch() {
  if (CARROT_VISION_HEALTH_T || rtcOwnershipBlocked()) return;
  CARROT_VISION_HEALTH_T = setInterval(checkCarrotVisionHealth, CARROT_VISION_HEALTH_POLL_MS);
}

function stopCarrotVisionHealthWatch() {
  if (!CARROT_VISION_HEALTH_T) return;
  clearInterval(CARROT_VISION_HEALTH_T);
  CARROT_VISION_HEALTH_T = null;
}

function checkCarrotVisionHealth() {
  if (!shouldRunCarrotVisionRealtime() || rtcOwnershipBlocked()) {
    stopCarrotVisionHealthWatch();
    return;
  }

  if (_rtcConnecting || RTC_PENDING_PC) return;
  if (!RTC_PC || !rtcHasLiveTrack()) {
    rtcRecover("dead", "health missing rtc track", {
      statusText: getUIText("connecting", "Connecting..."),
      rtcState: "missing-track",
    });
    return;
  }
  const lastFrameAt = Number(RTC_FREEZE_STATE.lastPresentedFrameMs || 0);
  if (lastFrameAt <= 0) return;
  const frameAgeMs = Date.now() - lastFrameAt;
  if (frameAgeMs >= RTC_FRAME_STALL_RECONNECT_MS) {
    rtcRecover("stall", getUIText("video_stalled_reconnecting", "Video stalled, reconnecting..."), { pc: RTC_PC });
  } else if (frameAgeMs >= RTC_FRAME_STALL_NUDGE_MS) {
    rtcNudgePlayback(RTC_PC, getRtcVideoElement(), "no presented frame");
  }
}


function rtcShouldConnect() {
  return shouldRunCarrotVisionRealtime()
    && !rtcOwnershipBlocked()
    && !_rtcConnecting
    && (!RTC_PC || !rtcHasLiveTrack());
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
  canResumeWithoutReconnect: rtcCanResumeWithoutReconnect,
  cancelRecovery: rtcCancelRecovery,
  cancelResumeCheck: rtcCancelResumeCheck,
  cancelRetry: rtcCancelRetry,
  captureVideoHoldFrame: rtcCaptureVideoHoldFrame,
  collectPerfStats: collectRtcPerfStats,
  connectOnce: rtcConnectOnce,
  diagnosticSnapshot: rtcDiagnosticSnapshot,
  disconnect: rtcDisconnect,
  disarmTrackTimeout: rtcDisarmTrackTimeout,
  disarmFirstFrameTimeout: rtcDisarmFirstFrameTimeout,
  exitPictureInPicture: rtcExitPictureInPicture,
  getVideoElement: getRtcVideoElement,
  hasCompactState: () => Boolean(window.CarrotVisionRaw?.hasCompactState?.()),
  hasLiveTrack: rtcHasLiveTrack,
  handleVisibilityChange: rtcHandleVisibilityChange,
  reportCameraRenderable: rtcReportCameraRenderable,
  reportPresentedFrame: rtcReportCameraPresentedFrame,
  prepareOwnershipRetry: rtcPrepareOwnershipRetry,
  takeOwnership: rtcTakeOwnership,
  ownershipBlocked: rtcOwnershipBlocked,
  rawStatsHistory: () => RTC_RAW_STATS_HISTORY.slice(-RTC_RAW_STATS_HISTORY_MAX),
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
  rtcCanResumeWithoutReconnect,
  rtcCancelRecovery,
  rtcCancelResumeCheck,
  rtcCancelRetry,
  rtcCaptureVideoHoldFrame,
  rtcConnectOnce,
  rtcDiagnosticSnapshot,
  rtcDisconnect,
  rtcDisarmFirstFrameTimeout,
  rtcDisarmTrackTimeout,
  rtcExitPictureInPicture,
  rtcHandleVisibilityChange,
  rtcHasLiveTrack,
  rtcOwnershipBlocked,
  rtcRawStatsHistory: () => RTC_RAW_STATS_HISTORY.slice(-RTC_RAW_STATS_HISTORY_MAX),
  rtcPrepareOwnershipRetry,
  rtcTakeOwnership,
  rtcResetFailCount,
  rtcScheduleResumeIfConnected,
  rtcShouldConnect,
  rtcStatusSet,
  startCarrotVisionHealthWatch,
  startRtcPerfPolling,
  stopCarrotVisionHealthWatch,
  stopRtcPerfPolling,
});
