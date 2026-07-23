import { createRouteSummaryAccumulator, ROUTE_SUMMARY_POLICY } from "./policy.js";
import { createRouteSummaryProgressEmitter } from "./progress.js";

self.importScripts("/js/vendor/fzstd.js?v=0.1.1", "/js/realtime/raw_capnp.js?v=2607-08");

const rawCapnp = self.CarrotRawCapnp;
const SUMMARY_SERVICES = new Set([
  "initData",
  "clocks",
  "carParams",
  "selfdriveState",
  "onroadEvents",
  "carState",
]);

let activeAbortController = null;

function appendBytes(left, right) {
  if (!left.byteLength) return right.slice();
  if (!right.byteLength) return left;
  const joined = new Uint8Array(left.byteLength + right.byteLength);
  joined.set(left, 0);
  joined.set(right, left.byteLength);
  return joined;
}

class SummaryLogParser {
  constructor(accumulator) {
    this.accumulator = accumulator;
    this.pending = new Uint8Array(0);
    this.decodeErrors = 0;
    this.eventCount = 0;
  }

  push(chunk) {
    this.pending = appendBytes(this.pending, chunk);
    let offset = 0;
    while (offset < this.pending.byteLength) {
      const length = rawCapnp.framedMessageByteLength(this.pending.subarray(offset));
      if (!length) break;
      this.ingest(this.pending.subarray(offset, offset + length));
      offset += length;
    }
    this.pending = offset ? this.pending.slice(offset) : this.pending;
  }

  ingest(message) {
    this.eventCount += 1;
    try {
      const event = rawCapnp.decodeReplayEvent(message, "", "", SUMMARY_SERVICES);
      if (event) this.accumulator.ingest(event);
    } catch {
      this.decodeErrors += 1;
    }
  }

  finish() {
    return {
      eventCount: this.eventCount,
      decodeErrors: this.decodeErrors,
      tailBytes: this.pending.byteLength,
    };
  }
}

async function streamResponse(url, onChunk, signal, onProgress) {
  const response = await fetch(url, { cache: "no-store", signal });
  if (!response.ok) throw new Error(`log-fetch-${response.status}`);
  const total = Math.max(0, Number(response.headers.get("content-length")) || 0);
  if (response.body?.getReader) {
    const reader = response.body.getReader();
    let loaded = 0;
    while (true) {
      const result = await reader.read();
      if (result.done) break;
      loaded += result.value.byteLength;
      onChunk(result.value, false);
      onProgress(loaded, total);
    }
    onChunk(new Uint8Array(0), true);
    return;
  }
  const bytes = new Uint8Array(await response.arrayBuffer());
  onChunk(bytes, true);
  onProgress(bytes.byteLength, total || bytes.byteLength);
}

async function bufferedResponse(url, signal, onProgress) {
  const response = await fetch(url, { cache: "no-store", signal });
  if (!response.ok) throw new Error(`log-fetch-${response.status}`);
  const bytes = new Uint8Array(await response.arrayBuffer());
  onProgress(bytes.byteLength, Number(response.headers.get("content-length")) || bytes.byteLength);
  return bytes;
}

function decodeBzip(compressed, parser) {
  if (!self.CarrotBzip?.decode) self.importScripts("/js/vendor/seek-bzip.js");
  if (!self.CarrotBzip?.decode) throw new Error("bzip-decoder-unavailable");
  const outputSize = 256 * 1024;
  let output = new Uint8Array(outputSize);
  let used = 0;
  self.CarrotBzip.decode(compressed, {
    writeByte(value) {
      output[used++] = value;
      if (used !== output.byteLength) return;
      parser.push(output);
      output = new Uint8Array(outputSize);
      used = 0;
    },
  }, true);
  if (used) parser.push(output.subarray(0, used));
}

async function processSegment(source, accumulator, signal, onProgress) {
  accumulator.startSegment();
  const parser = new SummaryLogParser(accumulator);
  const compression = String(source.compression || "");
  if (compression === "zstd") {
    if (!self.fzstd?.Decompress) throw new Error("zstd-decoder-unavailable");
    const decoder = new self.fzstd.Decompress((chunk) => parser.push(chunk));
    await streamResponse(source.url, (chunk, final) => decoder.push(chunk, final), signal, onProgress);
  } else if (compression === "bzip2") {
    decodeBzip(await bufferedResponse(source.url, signal, onProgress), parser);
  } else if (compression === "none") {
    await streamResponse(source.url, (chunk) => parser.push(chunk), signal, onProgress);
  } else {
    throw new Error("unsupported-log-compression");
  }
  return parser.finish();
}

async function analyze(payload) {
  if (!rawCapnp) throw new Error("capnp-decoder-unavailable");
  const segments = Array.isArray(payload.segments) ? payload.segments : [];
  const sourceKinds = [...new Set(segments.map((item) => String(item.kind || "")).filter(Boolean))];
  const accumulator = createRouteSummaryAccumulator({
    route: payload.route,
    source: sourceKinds.join("+"),
    policy: ROUTE_SUMMARY_POLICY,
  });
  const controller = new AbortController();
  activeAbortController = controller;
  const totalBytes = segments.reduce((sum, item) => sum + Math.max(0, Number(item.size) || 0), 0);
  let completedBytes = 0;
  let processedSegments = 0;
  let partialSegments = Math.max(0, Number(payload.skippedSegments) || 0);
  let decodedEvents = 0;
  let decodeErrors = 0;
  const startedAt = performance.now();
  const progress = createRouteSummaryProgressEmitter((detail) => {
    self.postMessage({ type: "progress", ...detail });
  });

  for (let index = 0; index < segments.length; index += 1) {
    const source = segments[index];
    const segmentBytes = Math.max(0, Number(source.size) || 0);
    try {
      const parsed = await processSegment(source, accumulator, controller.signal, (loaded) => {
        progress.update({
          completed: index,
          total: segments.length,
          loadedBytes: Math.min(totalBytes, completedBytes + Math.max(0, Number(loaded) || 0)),
          totalBytes,
        });
      });
      processedSegments += 1;
      decodedEvents += parsed.eventCount;
      decodeErrors += parsed.decodeErrors;
      if (parsed.tailBytes || parsed.decodeErrors) partialSegments += 1;
    } catch (error) {
      if (error?.name === "AbortError") throw error;
      partialSegments += 1;
      self.postMessage({ type: "warning", segment: source.segment, code: error?.message || "segment-failed" });
    }
    completedBytes += segmentBytes;
    progress.flush({
      completed: index + 1,
      total: segments.length,
      loadedBytes: Math.min(totalBytes, completedBytes),
      totalBytes,
    });
  }

  return accumulator.finish({
    requestedSegments: Math.max(segments.length, Number(payload.segmentCount) || 0),
    processedSegments,
    partialSegments,
    decodedEvents,
    decodeErrors,
    elapsedMs: Math.round(performance.now() - startedAt),
  });
}

self.onmessage = async (event) => {
  const payload = event?.data || {};
  if (payload.type === "abort") {
    activeAbortController?.abort?.();
    return;
  }
  if (payload.type !== "analyze-route") return;
  try {
    const result = await analyze(payload);
    self.postMessage({ type: "complete", result });
  } catch (error) {
    if (error?.name === "AbortError") return;
    self.postMessage({ type: "error", code: error?.message || "summary-worker-failed" });
  } finally {
    activeAbortController = null;
  }
};
