export const DEFAULT_MIME = 'video/mp4; codecs="avc1.42E01E"';
export const MAX_QUEUE = 12;
export const KEEP_SECONDS = 5;
export const START_SEGMENTS = 2;
export const TARGET_LAG_SECONDS = 0.55;
export const HARD_RECOVERY_LAG_SECONDS = 4;

function constructorFor(mime = DEFAULT_MIME, target = globalThis) {
  for (const constructor of [target.ManagedMediaSource, target.MediaSource]) {
    if (typeof constructor !== "function") continue;
    const checker = constructor.isTypeSupported || target.MediaSource?.isTypeSupported;
    try {
      if (typeof checker === "function" && checker.call(constructor, mime)) return constructor;
    } catch (_) {}
  }
  return null;
}

export function isMseSupported(mime = DEFAULT_MIME, target = globalThis) {
  return Boolean(constructorFor(mime, target));
}

export function createMse(video, options = {}) {
  if (!video) return null;
  const target = options.target || globalThis;
  let mediaSource = null;
  let sourceBuffer = null;
  let mediaSourceUrl = "";
  let queue = [];
  let trimming = false;
  let frameCallbackId = 0;
  let segmentCount = 0;
  let appendedSegmentCount = 0;
  let playbackStarted = false;
  let waitingForKeyframe = false;
  let errorMessage = "";
  const diagnostics = {
    mime: "",
    initCount: 0,
    segmentCount: 0,
    frameCount: 0,
    keyframeCount: 0,
    lastSequence: 0,
    lastSourceTimestampMillis: 0,
    lastDuration: 0,
    lastSegmentAt: 0,
    clearCount: 0,
    lastClearReason: "",
    lastClearAt: 0,
  };

  function supported(mime = DEFAULT_MIME) {
    return isMseSupported(mime, target);
  }

  function reportError(error) {
    const videoError = video.error;
    errorMessage = String(error?.message
      || videoError?.message
      || (videoError?.code ? `HTMLMediaElement error ${videoError.code}` : error || "Carrot Navi MSE playback error"));
    options.onError?.(errorMessage);
  }

  function scheduleFrameCallback() {
    if (typeof video.requestVideoFrameCallback !== "function" || frameCallbackId) return;
    frameCallbackId = video.requestVideoFrameCallback(() => {
      frameCallbackId = 0;
      options.onFrame?.();
      scheduleFrameCallback();
    });
  }

  function syncPlayback() {
    if (!video.buffered?.length) return;
    const range = video.buffered.length - 1;
    const start = video.buffered.start(range);
    const end = video.buffered.end(range);
    const currentTime = Number(video.currentTime);
    const lag = Number.isFinite(currentTime) ? end - currentTime : Number.POSITIVE_INFINITY;
    if (!playbackStarted) {
      if (appendedSegmentCount < START_SEGMENTS) return;
      try { video.currentTime = Math.max(start, end - TARGET_LAG_SECONDS); } catch (_) {}
      playbackStarted = true;
      video.playbackRate = 1;
    } else if (!Number.isFinite(currentTime) || currentTime < start || lag > HARD_RECOVERY_LAG_SECONDS) {
      try { video.currentTime = Math.max(start, end - TARGET_LAG_SECONDS); } catch (_) {}
      video.playbackRate = 1;
    } else if (lag > 1.2) {
      video.playbackRate = 1.05;
    } else if (lag > 0.85) {
      video.playbackRate = 1.02;
    } else if (lag < 0.7 && video.playbackRate !== 1) {
      video.playbackRate = 1;
    }
    if (video.paused) video.play().catch(() => {});
    scheduleFrameCallback();
  }

  function pump() {
    if (!sourceBuffer || sourceBuffer.updating || mediaSource?.readyState !== "open") return;
    if (sourceBuffer.buffered.length && queue.length <= 2) {
      const range = sourceBuffer.buffered.length - 1;
      const start = sourceBuffer.buffered.start(0);
      const end = sourceBuffer.buffered.end(range);
      const currentTime = Number(video.currentTime);
      const removeBefore = Math.min(end - KEEP_SECONDS, Number.isFinite(currentTime) ? currentTime - 3 : 0);
      if (!trimming && removeBefore > start + 0.5) {
        trimming = true;
        try {
          sourceBuffer.remove(0, removeBefore);
          return;
        } catch (_) {
          trimming = false;
        }
      }
    }
    const next = queue.shift();
    if (!next) return;
    try {
      sourceBuffer.appendBuffer(next.bytes);
      if (next.media) appendedSegmentCount += 1;
    } catch (error) {
      reportError(error);
    }
  }

  function onUpdateEnd() {
    trimming = false;
    syncPlayback();
    pump();
  }

  function destroy() {
    if (frameCallbackId && typeof video.cancelVideoFrameCallback === "function") {
      try { video.cancelVideoFrameCallback(frameCallbackId); } catch (_) {}
    }
    frameCallbackId = 0;
    try { video.pause(); } catch (_) {}
    video.playbackRate = 1;
    if (sourceBuffer) {
      sourceBuffer.removeEventListener("updateend", onUpdateEnd);
      sourceBuffer.removeEventListener("error", reportError);
      try { if (sourceBuffer.updating) sourceBuffer.abort(); } catch (_) {}
    }
    sourceBuffer = null;
    mediaSource = null;
    queue = [];
    trimming = false;
    segmentCount = 0;
    appendedSegmentCount = 0;
    playbackStarted = false;
    waitingForKeyframe = false;
    errorMessage = "";
    diagnostics.lastSegmentAt = 0;
    try { video.srcObject = null; } catch (_) {}
    video.removeAttribute("src");
    try { video.load(); } catch (_) {}
    if (mediaSourceUrl) target.URL.revokeObjectURL(mediaSourceUrl);
    mediaSourceUrl = "";
  }

  function start(initializationBuffer, mimeValue = DEFAULT_MIME) {
    destroy();
    const mime = String(mimeValue || DEFAULT_MIME);
    const Constructor = constructorFor(mime, target);
    if (!Constructor) {
      reportError(new Error(`Unsupported Carrot Navi media type: ${mime || "unknown"}`));
      return false;
    }
    const source = new Constructor();
    mediaSource = source;
    errorMessage = "";
    diagnostics.mime = mime;
    diagnostics.initCount += 1;
    queue.push({ bytes: new Uint8Array(initializationBuffer), media: false });
    source.addEventListener("sourceopen", () => {
      if (mediaSource !== source || source.readyState !== "open" || sourceBuffer) return;
      try {
        sourceBuffer = source.addSourceBuffer(mime);
        sourceBuffer.addEventListener("updateend", onUpdateEnd);
        sourceBuffer.addEventListener("error", reportError);
        pump();
      } catch (error) {
        reportError(error);
      }
    });
    video.muted = true;
    video.autoplay = true;
    video.playsInline = true;
    video.disableRemotePlayback = true;
    mediaSourceUrl = target.URL.createObjectURL(source);
    video.src = mediaSourceUrl;
    return true;
  }

  function append(buffer, metadata = {}) {
    if (!mediaSource) return false;
    const keyframe = Boolean(metadata.keyframe);
    if (waitingForKeyframe) {
      if (!keyframe) return false;
      waitingForKeyframe = false;
    }
    if (queue.length >= MAX_QUEUE) {
      errorMessage = "Carrot Navi MSE queue overflow";
      queue = queue.filter((item) => !item.media);
      waitingForKeyframe = !keyframe;
      if (waitingForKeyframe) return false;
    }
    segmentCount += 1;
    diagnostics.segmentCount += 1;
    diagnostics.frameCount += Math.max(1, Number(metadata.frameCount) || 1);
    diagnostics.keyframeCount += Math.max(0, Number(metadata.keyframeCount) || (keyframe ? 1 : 0));
    diagnostics.lastSequence = Math.max(0, Number(metadata.sourceSequence ?? metadata.sequence) || 0);
    diagnostics.lastSourceTimestampMillis = Math.max(0, Number(metadata.sourceTimestampMillis) || 0);
    diagnostics.lastDuration = Math.max(0, Number(metadata.durationMs ?? metadata.duration) || 0);
    diagnostics.lastSegmentAt = target.performance.now();
    queue.push({ bytes: new Uint8Array(buffer), media: true, keyframe });
    pump();
    return true;
  }

  function markClear(reason = "cleared") {
    diagnostics.clearCount += 1;
    diagnostics.lastClearReason = String(reason).slice(0, 64);
    diagnostics.lastClearAt = target.performance.now();
    diagnostics.lastSegmentAt = 0;
  }

  function bufferedRanges() {
    const ranges = [];
    const buffered = video.buffered;
    if (!buffered) return ranges;
    for (let index = Math.max(0, buffered.length - 2); index < buffered.length; index += 1) {
      try { ranges.push([Number(buffered.start(index).toFixed(3)), Number(buffered.end(index).toFixed(3))]); } catch (_) {}
    }
    return ranges;
  }

  function snapshot() {
    const quality = typeof video.getVideoPlaybackQuality === "function" ? video.getVideoPlaybackQuality() : null;
    return {
      error: errorMessage,
      media: { ...diagnostics },
      mse: {
        constructor: mediaSource?.constructor?.name || "",
        readyState: mediaSource?.readyState || "",
        sourceBufferUpdating: Boolean(sourceBuffer?.updating),
        queueLength: queue.length,
        segmentCount,
        appendedSegmentCount,
        playbackStarted,
        buffered: bufferedRanges(),
        currentTime: Number.isFinite(video.currentTime) ? Number(video.currentTime.toFixed(3)) : null,
        readyStateCode: video.readyState,
        networkStateCode: video.networkState,
        paused: video.paused,
        playbackRate: video.playbackRate,
        videoWidth: video.videoWidth,
        videoHeight: video.videoHeight,
        errorCode: video.error?.code || 0,
        errorMessage: String(video.error?.message || "").slice(0, 256),
        totalVideoFrames: quality?.totalVideoFrames ?? null,
        droppedVideoFrames: quality?.droppedVideoFrames ?? null,
        corruptedVideoFrames: quality?.corruptedVideoFrames ?? null,
      },
    };
  }

  video.addEventListener("error", reportError);
  return Object.freeze({ supported, start, append, markClear, destroy, snapshot });
}

export const CarrotNaviMse = Object.freeze({
  DEFAULT_MIME,
  create: createMse,
  supported: (mime) => isMseSupported(mime),
});

export function installCarrotNaviMseGlobal(target = globalThis) {
  target.CarrotNaviMse = CarrotNaviMse;
  return CarrotNaviMse;
}
