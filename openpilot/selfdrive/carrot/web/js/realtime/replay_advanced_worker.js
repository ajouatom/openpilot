"use strict";

self.importScripts("/js/vendor/fzstd.js?v=0.1.1", "/js/realtime/raw_capnp.js?v=2607-06");

const rawCapnp = self.CarrotRawCapnp;
const MAX_DURATION_MS = 3 * 60 * 1000;
const MAX_NUMERIC_VALUES = 50_000;
const MAX_NUMERIC_SAMPLES = 100_000;
const MAX_STATE_SAMPLES = 6_000;
const MAX_SNAPSHOT_CANDIDATES = 5_000;
const DEFAULT_MAX_POINTS = 1_200;

function appendBytes(left, right) {
  if (!left?.byteLength) return right;
  if (!right?.byteLength) return left;
  const joined = new Uint8Array(left.byteLength + right.byteLength);
  joined.set(left, 0);
  joined.set(right, left.byteLength);
  return joined;
}

function valueAtPath(object, path, service) {
  const prefix = `${service}.`;
  const normalizedPath = String(path || "").startsWith(prefix)
    ? String(path).slice(prefix.length)
    : String(path || "");
  let value = object;
  for (const key of normalizedPath.split(".")) {
    if (!key || value == null || typeof value !== "object") return undefined;
    value = value[key];
  }
  return value;
}

function normalizedValue(value, field) {
  const type = String(field?.type || "");
  if (type === "enumname" && typeof value === "number" && Number.isFinite(value)) {
    return { kind: "state", value: field?.values?.[value] ?? String(value) };
  }
  if (typeof value === "number" && Number.isFinite(value)) return { kind: "number", value };
  if (typeof value === "boolean" || typeof value === "string") return { kind: "state", value };
  if (Array.isArray(value)) {
    const numeric = value.filter((item) => typeof item === "number" && Number.isFinite(item));
    return {
      kind: "summary",
      value: {
        type,
        length: value.length,
        average: numeric.length ? numeric.reduce((sum, item) => sum + item, 0) / numeric.length : null,
      },
    };
  }
  if (value && typeof value === "object") {
    return { kind: "summary", value: { type, keys: Object.keys(value).length } };
  }
  return null;
}

function valueSignature(value) {
  if (value == null) return "null";
  if (typeof value === "object") return JSON.stringify(value);
  return `${typeof value}:${String(value)}`;
}

function quantile(sorted, ratio) {
  if (!sorted.length) return null;
  const index = Math.max(0, Math.min(sorted.length - 1, (sorted.length - 1) * ratio));
  const lower = Math.floor(index);
  const upper = Math.ceil(index);
  if (lower === upper) return sorted[lower];
  const weight = index - lower;
  return sorted[lower] * (1 - weight) + sorted[upper] * weight;
}

function minMaxEnvelope(samples, maxPoints) {
  const limit = Math.max(50, Math.min(3_000, Number(maxPoints) || DEFAULT_MAX_POINTS));
  if (samples.length <= limit) return samples;
  const interior = samples.slice(1, -1);
  const bucketCount = Math.max(1, Math.floor((limit - 2) / 2));
  const bucketSize = interior.length / bucketCount;
  const selected = [samples[0]];
  for (let bucket = 0; bucket < bucketCount; bucket += 1) {
    const start = Math.floor(bucket * bucketSize);
    const end = Math.min(interior.length, Math.max(start + 1, Math.floor((bucket + 1) * bucketSize)));
    let minimum = null;
    let maximum = null;
    for (let index = start; index < end; index += 1) {
      const sample = interior[index];
      if (!minimum || sample.value < minimum.value) minimum = sample;
      if (!maximum || sample.value > maximum.value) maximum = sample;
    }
    if (minimum && maximum && minimum !== maximum) {
      if (minimum.monoTime <= maximum.monoTime) selected.push(minimum, maximum);
      else selected.push(maximum, minimum);
    } else if (minimum) selected.push(minimum);
  }
  selected.push(samples[samples.length - 1]);
  return selected.slice(0, limit);
}

function isRecoverableTailError(error) {
  const message = String(error?.message || error || "");
  return Number(error?.code) === 5 || /unexpected\s+eof|unexpected\s+end|incomplete\s+(?:frame|stream)/i.test(message);
}

class AdvancedLogQuery {
  constructor(options) {
    this.options = options;
    this.pending = new Uint8Array(0);
    this.partialReason = "";
    this.tailBytes = 0;
    this.decodeErrorCount = 0;
    this.rawEventCount = 0;
    this.matchingEventCount = 0;
    this.invalidEventCount = 0;
    this.missingValueCount = 0;
    this.nonfiniteCount = 0;
    this.firstMonoTime = 0;
    this.firstCameraMonoTime = 0;
    this.firstMatchingCameraMonoTime = 0;
    this.samples = [];
    this.numericValues = [];
    this.numericCount = 0;
    this.numericSum = 0;
    this.numericSumSquares = 0;
    this.numericMin = Infinity;
    this.numericMax = -Infinity;
    this.kind = "state";
    this.lastSignature = "";
    this.distinctValues = new Map();
    this.transitions = new Map();
    this.snapshotCandidates = [];
  }

  markPartial(reason, tailBytes = 0) {
    if (!this.partialReason) this.partialReason = String(reason || "Recorded log ended early").slice(0, 320);
    this.tailBytes = Math.max(this.tailBytes, Number(tailBytes) || 0);
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
    this.rawEventCount += 1;
    let event = null;
    try {
      // The sensor view owns raw liveTracks processing. Other analysis
      // queries skip that high-cardinality payload before it is decoded.
      event = rawCapnp.decodeReplayEvent(
        message,
        "",
        this.options.service === "liveTracks" ? "" : "liveTracks",
      );
    } catch {
      this.decodeErrorCount += 1;
      return;
    }
    if (!event?.service || !Number.isFinite(event.logMonoTime) || event.logMonoTime <= 0) return;
    if (!this.firstMonoTime) this.firstMonoTime = event.logMonoTime;
    if (event.service === "qRoadEncodeIdx") {
      if (!this.firstCameraMonoTime) this.firstCameraMonoTime = event.logMonoTime;
      if (Number(event.decoded?.segmentNum) === Number(this.options.expectedSegment)
          && !this.firstMatchingCameraMonoTime) {
        this.firstMatchingCameraMonoTime = event.logMonoTime;
      }
      return;
    }
    if (event.service !== this.options.service) return;
    this.matchingEventCount += 1;
    if (!event.valid) this.invalidEventCount += 1;

    if (this.options.mode === "snapshot") {
      if (this.snapshotCandidates.length < MAX_SNAPSHOT_CANDIDATES) {
        this.snapshotCandidates.push({ monoTime: event.logMonoTime, valid: event.valid, decoded: event.decoded });
      }
      return;
    }

    const rawValue = valueAtPath(event.decoded, this.options.field?.path, this.options.service);
    if (rawValue === undefined || rawValue === null) {
      this.missingValueCount += 1;
      return;
    }
    const normalized = normalizedValue(rawValue, this.options.field);
    if (!normalized) {
      this.nonfiniteCount += 1;
      return;
    }
    this.kind = normalized.kind;
    if (normalized.kind === "number") {
      const value = normalized.value;
      this.numericCount += 1;
      this.numericSum += value;
      this.numericSumSquares += value * value;
      this.numericMin = Math.min(this.numericMin, value);
      this.numericMax = Math.max(this.numericMax, value);
      if (this.numericValues.length < MAX_NUMERIC_VALUES) {
        this.numericValues.push(value);
      } else {
        // Deterministic reservoir sampling keeps quantiles representative
        // without retaining an unbounded copy of the selected field.
        const replacement = (Math.imul(this.numericCount, 2654435761) >>> 0) % this.numericCount;
        if (replacement < MAX_NUMERIC_VALUES) this.numericValues[replacement] = value;
      }
      if (this.samples.length < MAX_NUMERIC_SAMPLES) {
        this.samples.push({ monoTime: event.logMonoTime, value });
      }
      return;
    }

    const signature = valueSignature(normalized.value);
    this.distinctValues.set(signature, (this.distinctValues.get(signature) || 0) + 1);
    if (signature === this.lastSignature) return;
    if (this.lastSignature) {
      const transition = `${this.lastSignature}→${signature}`;
      this.transitions.set(transition, (this.transitions.get(transition) || 0) + 1);
    }
    this.lastSignature = signature;
    if (this.samples.length < MAX_STATE_SAMPLES) {
      this.samples.push({ monoTime: event.logMonoTime, value: normalized.value });
    }
  }

  baseMonoTime() {
    const provided = Number(this.options.baseMonoTime);
    if (Number.isFinite(provided) && provided > 0) return provided;
    return this.firstMatchingCameraMonoTime || this.firstCameraMonoTime || this.firstMonoTime;
  }

  finish() {
    if (this.pending.byteLength) {
      this.markPartial("Recorded log ended with an incomplete Cap'n Proto message", this.pending.byteLength);
      this.pending = new Uint8Array(0);
    }
    const baseMonoTime = this.baseMonoTime();
    if (!baseMonoTime) throw new Error("Recorded log has no readable events");
    if (this.options.mode === "snapshot") return this.finishSnapshot(baseMonoTime);
    return this.finishTrack(baseMonoTime);
  }

  finishSnapshot(baseMonoTime) {
    const targetMonoTime = baseMonoTime + Math.max(0, Number(this.options.timeMs) || 0) * 1_000_000;
    let nearest = null;
    let distance = Infinity;
    for (const candidate of this.snapshotCandidates) {
      const candidateDistance = Math.abs(candidate.monoTime - targetMonoTime);
      if (candidateDistance >= distance) continue;
      nearest = candidate;
      distance = candidateDistance;
    }
    return {
      mode: "snapshot",
      found: Boolean(nearest),
      service: this.options.service,
      timeMs: nearest ? Math.max(0, (nearest.monoTime - baseMonoTime) / 1_000_000) : 0,
      distanceMs: nearest ? distance / 1_000_000 : 0,
      valid: Boolean(nearest?.valid),
      snapshot: nearest?.decoded || null,
      parseStatus: this.partialReason || this.decodeErrorCount ? "partial" : "complete",
      parseError: this.partialReason,
    };
  }

  finishTrack(baseMonoTime) {
    let samples = this.samples
      .map((sample) => ({
        monoTime: sample.monoTime,
        timeMs: Math.max(0, Math.min(MAX_DURATION_MS, (sample.monoTime - baseMonoTime) / 1_000_000)),
        value: sample.value,
      }))
      .filter((sample) => sample.monoTime >= baseMonoTime)
      .sort((left, right) => left.monoTime - right.monoTime);
    if (this.kind === "number") samples = minMaxEnvelope(samples, this.options.maxPoints);
    samples = samples.map(({ timeMs, value }) => ({ timeMs, value }));

    const sorted = [...this.numericValues].sort((left, right) => left - right);
    const average = this.numericCount ? this.numericSum / this.numericCount : null;
    const variance = this.numericCount
      ? Math.max(0, this.numericSumSquares / this.numericCount - average * average)
      : null;
    const numeric = this.numericCount ? {
      count: this.numericCount,
      min: this.numericMin,
      max: this.numericMax,
      mean: average,
      stddev: Math.sqrt(variance),
      rms: Math.sqrt(this.numericSumSquares / this.numericCount),
      p01: quantile(sorted, 0.01),
      p50: quantile(sorted, 0.5),
      p99: quantile(sorted, 0.99),
    } : {};
    const stateTransitions = Array.from(this.transitions, ([value, count]) => ({ value, count }))
      .sort((left, right) => right.count - left.count)
      .slice(0, 24);
    const parseStatus = this.partialReason || this.decodeErrorCount ? "partial" : "complete";
    return {
      mode: "track",
      service: this.options.service,
      field: this.options.field,
      kind: this.kind,
      renderer: String(this.options.field?.recommendedView || (this.kind === "number" ? "graph" : "state-band")),
      statisticsPolicy: String(this.options.field?.statisticsPolicy || ""),
      source: "client-worker-rlog",
      samples,
      stats: this.kind === "number" ? {
        count: this.numericCount,
        min: this.numericMin,
        max: this.numericMax,
        average,
        stddev: numeric.stddev,
        p01: numeric.p01,
        p50: numeric.p50,
        p99: numeric.p99,
      } : { count: this.matchingEventCount - this.missingValueCount },
      rawStats: {
        matchingEventCount: this.matchingEventCount,
        extractedValueCount: this.kind === "number" ? this.numericCount : this.samples.length,
        invalidEventCount: this.invalidEventCount,
        missingValueCount: this.missingValueCount,
        nonfiniteCount: this.nonfiniteCount,
        distinctValueCount: this.distinctValues.size,
        stateTransitions,
        numeric,
        decodeErrorCount: this.decodeErrorCount,
        rawEventCount: this.rawEventCount,
        tailBytes: this.tailBytes,
      },
      redacted: false,
      parseStatus,
      parseError: this.partialReason,
      cacheHit: false,
    };
  }
}

async function streamResponse(url, onChunk, signal) {
  const response = await fetch(url, { cache: "no-store", signal });
  if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
  const total = Number(response.headers.get("Content-Length")) || 0;
  let loaded = 0;
  if (response.body?.getReader) {
    const reader = response.body.getReader();
    while (true) {
      const result = await reader.read();
      if (result.done) break;
      loaded += result.value.byteLength;
      onChunk(result.value, false);
      self.postMessage({ type: "progress", loaded, total });
    }
  } else {
    const bytes = new Uint8Array(await response.arrayBuffer());
    loaded = bytes.byteLength;
    onChunk(bytes, false);
    self.postMessage({ type: "progress", loaded, total: total || loaded });
  }
  onChunk(new Uint8Array(0), true);
}

async function bufferedResponse(url, signal) {
  const response = await fetch(url, { cache: "no-store", signal });
  if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
  const bytes = new Uint8Array(await response.arrayBuffer());
  self.postMessage({ type: "progress", loaded: bytes.byteLength, total: bytes.byteLength });
  return bytes;
}

function decodeBzip(compressed, query) {
  if (!self.CarrotBzip?.decode) self.importScripts("/js/vendor/seek-bzip.js");
  if (!self.CarrotBzip?.decode) throw new Error("Client Bzip2 decoder is unavailable");
  const outputSize = 256 * 1024;
  let output = new Uint8Array(outputSize);
  let used = 0;
  self.CarrotBzip.decode(compressed, {
    writeByte(value) {
      output[used++] = value;
      if (used !== output.byteLength) return;
      query.push(output);
      output = new Uint8Array(outputSize);
      used = 0;
    },
  }, true);
  if (used) query.push(output.subarray(0, used));
}

async function run(options) {
  if (!rawCapnp) throw new Error("Client Cap'n Proto decoder is unavailable");
  const query = new AdvancedLogQuery(options);
  const compression = String(options.compression || "");
  const controller = new AbortController();
  self.advancedAbortController = controller;
  try {
    if (compression === "zstd") {
      if (!self.fzstd?.Decompress) throw new Error("Client Zstandard decoder is unavailable");
      const decoder = new self.fzstd.Decompress((chunk) => query.push(chunk));
      await streamResponse(options.url, (chunk, final) => decoder.push(chunk, final), controller.signal);
    } else if (compression === "bzip2") {
      decodeBzip(await bufferedResponse(options.url, controller.signal), query);
    } else if (compression === "none") {
      await streamResponse(options.url, (chunk) => query.push(chunk), controller.signal);
    } else {
      throw new Error(`unsupported-client-compression:${compression || "unknown"}`);
    }
  } catch (error) {
    if (!isRecoverableTailError(error)) throw error;
    query.markPartial(error?.message || error);
  }
  return query.finish();
}

self.onmessage = async (event) => {
  const payload = event?.data || {};
  if (payload.type === "abort") {
    self.advancedAbortController?.abort?.();
    return;
  }
  if (payload.type !== "query" && payload.type !== "snapshot") return;
  try {
    const result = await run({ ...payload, mode: payload.type === "snapshot" ? "snapshot" : "track" });
    self.postMessage({ type: "complete", result });
  } catch (error) {
    if (error?.name === "AbortError") return;
    self.postMessage({ type: "error", error: error?.message || String(error) });
  }
};
