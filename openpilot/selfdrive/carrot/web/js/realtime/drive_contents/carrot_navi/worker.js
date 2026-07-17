"use strict";

const MEDIA_MAGIC = [0x43, 0x4e, 0x57, 0x42]; // CNWB
const MEDIA_HEADER_BYTES = 9;
const MSE_TIMESCALE = 90000;
const MSE_BATCH_TARGET_DURATION = 22500; // 250ms
const MSE_BATCH_MAX_WAIT_MS = 280;

let sessionId = "";
let mseBatch = [];
let mseBatchDuration = 0;
let mseBatchTimer = 0;
let imageGeneration = 0;
let lastMapSequence = -1;
let pipelineMode = "";
const imageSequences = new Map();
const pendingImageDecodes = new Map();
const activeImageDecodes = new Set();

function clearMseBatch() {
  if (mseBatchTimer) clearTimeout(mseBatchTimer);
  mseBatchTimer = 0;
  mseBatch = [];
  mseBatchDuration = 0;
}

function concatSegments(entries) {
  const byteLength = entries.reduce((total, entry) => total + entry.segment.byteLength, 0);
  const result = new Uint8Array(byteLength);
  let offset = 0;
  for (const entry of entries) {
    result.set(entry.segment, offset);
    offset += entry.segment.byteLength;
  }
  return result;
}

function flushMseBatch() {
  if (!mseBatch.length) return;
  if (mseBatchTimer) clearTimeout(mseBatchTimer);
  mseBatchTimer = 0;
  const entries = mseBatch;
  const duration = mseBatchDuration;
  mseBatch = [];
  mseBatchDuration = 0;

  const segment = concatSegments(entries);
  const first = entries[0];
  const last = entries[entries.length - 1];
  self.postMessage({
    type: "mse-segment",
    segment: segment.buffer,
    keyframe: Boolean(first.keyframe),
    keyframeCount: entries.reduce((count, entry) => count + (entry.keyframe ? 1 : 0), 0),
    frameCount: entries.length,
    sequence: first.sequence,
    sourceSequence: last.sourceSequence,
    sourceTimestampMillis: last.sourceTimestampMillis,
    duration,
    durationMs: Math.round(duration * 1000 / MSE_TIMESCALE),
  }, [segment.buffer]);
}

function queueMseFragment(fragment, metadata, sourceSequence) {
  // Keep random-access frames at the beginning of an append batch. This lets
  // the page discard queued delta batches safely if it ever falls behind.
  if (fragment.keyframe && mseBatch.length) flushMseBatch();
  mseBatch.push({
    segment: fragment.segment,
    keyframe: fragment.keyframe,
    sequence: fragment.sequence,
    sourceSequence,
    sourceTimestampMillis: Math.max(0, Number(metadata.sourceTimestampMillis) || 0),
  });
  mseBatchDuration += fragment.duration;
  if (mseBatchDuration >= MSE_BATCH_TARGET_DURATION) {
    flushMseBatch();
  } else if (!mseBatchTimer) {
    mseBatchTimer = setTimeout(flushMseBatch, MSE_BATCH_MAX_WAIT_MS);
  }
}

function resetRuntime() {
  clearMseBatch();
  pipelineMode = "";
}

function selectPipeline(mode) {
  const normalized = String(mode || "");
  if (pipelineMode === normalized) return;
  pipelineMode = normalized;
  self.postMessage({ type: "pipeline", mode: normalized });
}

function resetImages(notify = true) {
  imageGeneration += 1;
  imageSequences.clear();
  pendingImageDecodes.clear();
  if (notify) self.postMessage({ type: "overlay-reset" });
}

function reportError(error) {
  self.postMessage({ type: "error", message: error?.message || String(error || "Carrot Navi decode error") });
}

function parseWire(buffer) {
  const bytes = new Uint8Array(buffer);
  if (bytes.byteLength < MEDIA_HEADER_BYTES || MEDIA_MAGIC.some((value, index) => bytes[index] !== value)) {
    throw new Error("Invalid Carrot Navi media frame");
  }
  const view = new DataView(buffer);
  if (view.getUint8(4) !== 1) throw new Error("Unsupported Carrot Navi media version");
  const headerLength = view.getUint32(5, false);
  const payloadOffset = MEDIA_HEADER_BYTES + headerLength;
  if (headerLength <= 0 || payloadOffset > bytes.byteLength) throw new Error("Invalid Carrot Navi media header");
  const headerBytes = bytes.subarray(MEDIA_HEADER_BYTES, payloadOffset);
  const metadata = JSON.parse(new TextDecoder().decode(headerBytes));
  return { metadata, payload: bytes.subarray(payloadOffset) };
}

async function decodeOverlayImage(metadata, payload, generation) {
  const name = String(metadata.name || "");
  const sequence = Math.max(0, Number(metadata.sequence) || 0);
  const sourceTimestampMillis = Math.max(0, Number(metadata.sourceTimestampMillis) || 0);
  const mime = Number(metadata.formatOrReason) === 1 ? "image/png" : "image/jpeg";
  const bytes = new Uint8Array(payload);
  if (typeof createImageBitmap !== "function") {
    if (generation !== imageGeneration || imageSequences.get(name) !== sequence) return;
    const buffer = bytes.slice().buffer;
    self.postMessage({ type: "overlay-bytes", name, sequence, sourceTimestampMillis, mime, buffer }, [buffer]);
    return;
  }
  try {
    const bitmap = await createImageBitmap(new Blob([bytes], { type: mime }));
    if (generation !== imageGeneration || imageSequences.get(name) !== sequence) {
      bitmap.close?.();
      return;
    }
    self.postMessage({ type: "overlay-image", name, sequence, sourceTimestampMillis, bitmap }, [bitmap]);
  } catch (error) {
    self.postMessage({ type: "overlay-error", name, message: error?.message || String(error || "image decode error") });
  }
}

async function drainOverlayImage(name) {
  if (activeImageDecodes.has(name)) return;
  activeImageDecodes.add(name);
  try {
    while (pendingImageDecodes.has(name)) {
      const pending = pendingImageDecodes.get(name);
      pendingImageDecodes.delete(name);
      await decodeOverlayImage(pending.metadata, pending.payload, pending.generation);
    }
  } finally {
    activeImageDecodes.delete(name);
  }
}

function queueOverlayImage(metadata, payload) {
  const name = String(metadata.name || "");
  pendingImageDecodes.set(name, {
    metadata: { ...metadata },
    payload: new Uint8Array(payload),
    generation: imageGeneration,
  });
  void drainOverlayImage(name);
}

function handleOverlayMedia(metadata, payload) {
  const name = String(metadata.name || "");
  if (!name) return;
  const sequence = Math.max(0, Number(metadata.sequence) || 0);
  const previous = imageSequences.get(name);
  if (previous !== undefined && sequence <= previous) return;
  imageSequences.set(name, sequence);
  const messageType = Number(metadata.messageType) || 0;
  if (!metadata.present || messageType === 4) {
    self.postMessage({
      type: "overlay-clear",
      name,
      sequence,
      sourceTimestampMillis: Math.max(0, Number(metadata.sourceTimestampMillis) || 0),
    });
    return;
  }
  if (messageType !== 1 || !payload.byteLength) return;
  queueOverlayImage(metadata, payload);
}

function handleMedia(buffer) {
  const { metadata, payload } = parseWire(buffer);

  const nextSessionId = String(metadata.sessionId || "");
  if (nextSessionId !== sessionId) {
    resetRuntime();
    resetImages();
    lastMapSequence = -1;
    sessionId = nextSessionId;
  }

  if (metadata.kind === "image") {
    handleOverlayMedia(metadata, payload);
    return;
  }
  if (metadata.kind === "fmp4" && metadata.name === "map_main") {
    const messageType = Number(metadata.messageType) || 0;
    if (!metadata.present || messageType === 4) {
      resetRuntime();
      self.postMessage({ type: "clear", reason: String(metadata.reason || "cleared") });
      return;
    }
    if (messageType === 2 && payload.byteLength) {
      clearMseBatch();
      lastMapSequence = -1;
      selectPipeline("server-fmp4");
      const initialization = payload.slice();
      self.postMessage({
        type: "mse-init",
        segment: initialization.buffer,
        mime: String(metadata.mime || 'video/mp4; codecs="avc1.42E01E"'),
        width: Math.max(1, Number(metadata.width) || 960),
        height: Math.max(1, Number(metadata.height) || 540),
      }, [initialization.buffer]);
      return;
    }
    if (messageType !== 3 || !payload.byteLength || pipelineMode !== "server-fmp4") return;
    const sequence = Math.max(0, Number(metadata.sequence) || 0);
    if (sequence <= lastMapSequence) return;
    lastMapSequence = sequence;
    const segment = payload.slice();
    const durationMs = Math.max(1, Number(metadata.durationMs) || 200);
    queueMseFragment({
      segment,
      keyframe: Boolean((Number(metadata.flags) || 0) & 1),
      sequence,
      duration: Math.max(1, Math.round(durationMs * MSE_TIMESCALE / 1000)),
    }, metadata, sequence);
    return;
  }
}

self.addEventListener("message", (event) => {
  const message = event.data || {};
  if (message.type === "configure") {
    resetRuntime();
    resetImages();
    lastMapSequence = -1;
    sessionId = "";
    return;
  }
  if (message.type === "reset-media") {
    resetRuntime();
    lastMapSequence = -1;
    return;
  }
  if (message.type === "reset") {
    resetRuntime();
    resetImages();
    lastMapSequence = -1;
    sessionId = "";
    return;
  }
  if (message.type !== "media" || !(message.buffer instanceof ArrayBuffer)) return;
  try {
    handleMedia(message.buffer);
  } catch (error) {
    reportError(error);
  }
});
