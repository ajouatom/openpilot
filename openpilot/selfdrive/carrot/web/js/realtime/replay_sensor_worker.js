"use strict";

self.importScripts("/js/vendor/fzstd.js?v=0.1.1", "/js/realtime/raw_capnp.js?v=2607-06");

const rawCapnp = self.CarrotRawCapnp;
const TRACK_STRIDE = 8;
const SAMPLE_INTERVAL_MS = 50;
const MAX_DURATION_MS = 3 * 60 * 1000;
const radarSourceCode = Object.freeze({ frontRadar: 0, scc: 1, corner235: 2, corner180: 3, corner430: 4 });

function appendBytes(left, right) {
  if (!left?.byteLength) return right;
  if (!right?.byteLength) return left;
  const joined = new Uint8Array(left.byteLength + right.byteLength);
  joined.set(left, 0);
  joined.set(right, left.byteLength);
  return joined;
}

function compactLiveTracks(decoded, availability) {
  const tracks = [];
  for (const point of Array.isArray(decoded?.points) ? decoded.points : []) {
    const dRel = Number(point?.dRel);
    const yRel = Number(point?.yRel);
    const vRel = Number(point?.vRel);
    if (!Number.isFinite(dRel) || !Number.isFinite(yRel) || !Number.isFinite(vRel)) continue;
    const rawSource = point?.radarSource;
    const source = Number.isFinite(Number(rawSource))
      ? Math.max(0, Math.min(3, Number(rawSource)))
      : (radarSourceCode[String(rawSource)] ?? 0);
    if (source === 0) availability.front = true;
    else if (source === 1) availability.scc = true;
    else availability.corner = true;
    tracks.push(
      Number(point?.trackId) || 0,
      dRel,
      yRel,
      vRel,
      Number(point?.aRel),
      Number(point?.yvRel),
      point?.measured ? 1 : 0,
      source,
    );
  }
  return { stride: TRACK_STRIDE, tracks };
}

class SensorTrackBuilder {
  constructor(baseMonoTime, durationMs) {
    this.baseMonoTime = Number(baseMonoTime);
    this.durationMs = Math.min(MAX_DURATION_MS, Math.max(0, Number(durationMs) || MAX_DURATION_MS));
    this.pending = new Uint8Array(0);
    this.buckets = new Map();
    this.availability = { front: false, scc: false, corner: false };
  }

  push(chunk) {
    this.pending = appendBytes(this.pending, chunk);
    let offset = 0;
    while (offset < this.pending.byteLength) {
      const messageLength = rawCapnp.framedMessageByteLength(this.pending.subarray(offset));
      if (!messageLength) break;
      this.ingest(this.pending.subarray(offset, offset + messageLength));
      offset += messageLength;
    }
    this.pending = offset ? this.pending.slice(offset) : this.pending;
  }

  ingest(message) {
    let event = null;
    try { event = rawCapnp.decodeReplayEvent(message, "liveTracks"); } catch { return; }
    if (!event?.valid || event.service !== "liveTracks") return;
    const timeMs = Math.round((Number(event.logMonoTime) - this.baseMonoTime) / 1_000_000);
    if (!Number.isFinite(timeMs) || timeMs < 0 || timeMs > this.durationMs) return;
    const bucket = Math.floor(timeMs / SAMPLE_INTERVAL_MS);
    const previous = this.buckets.get(bucket);
    if (!previous || Number(event.logMonoTime) >= previous.logMonoTime) {
      this.buckets.set(bucket, {
        logMonoTime: Number(event.logMonoTime),
        timeMs,
        decoded: compactLiveTracks(event.decoded, this.availability),
      });
    }
  }

  finish() {
    const frames = Array.from(this.buckets.values())
      .sort((left, right) => left.timeMs - right.timeMs)
      .map(({ timeMs, decoded }) => ({ timeMs, decoded }));
    return { frames, availability: this.availability };
  }
}

async function streamResponse(url, onChunk, signal) {
  const response = await fetch(url, { cache: "no-store", signal });
  if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
  if (response.body?.getReader) {
    const reader = response.body.getReader();
    while (true) {
      const result = await reader.read();
      if (result.done) break;
      onChunk(result.value, false);
    }
  } else {
    onChunk(new Uint8Array(await response.arrayBuffer()), false);
  }
  onChunk(new Uint8Array(0), true);
}

async function bufferedResponse(url, signal) {
  const response = await fetch(url, { cache: "no-store", signal });
  if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
  return new Uint8Array(await response.arrayBuffer());
}

function decodeBzipToBuilder(compressed, builder) {
  if (!self.CarrotBzip?.decode) self.importScripts("/js/vendor/seek-bzip.js");
  if (!self.CarrotBzip?.decode) throw new Error("Client Bzip2 decoder is unavailable");
  const outputSize = 256 * 1024;
  let output = new Uint8Array(outputSize);
  let used = 0;
  const sink = {
    writeByte(value) {
      output[used++] = value;
      if (used !== output.byteLength) return;
      builder.push(output);
      output = new Uint8Array(outputSize);
      used = 0;
    },
  };
  self.CarrotBzip.decode(compressed, sink, true);
  if (used) builder.push(output.subarray(0, used));
}

function isRecoverableTailError(error) {
  return /unexpected\s+eof|unexpected\s+end|incomplete\s+(?:frame|stream)/i.test(String(error?.message || error || ""));
}

async function buildTracks(options) {
  if (!rawCapnp) throw new Error("Client Cap'n Proto decoder is unavailable");
  const baseMonoTime = Number(options.baseMonoTime);
  if (!Number.isFinite(baseMonoTime) || baseMonoTime <= 0) throw new Error("Replay time base is unavailable");
  const builder = new SensorTrackBuilder(baseMonoTime, options.durationMs);
  const controller = new AbortController();
  self.sensorAbortController = controller;
  try {
    const compression = String(options.compression || "");
    if (compression === "zstd") {
      if (!self.fzstd?.Decompress) throw new Error("Client Zstandard decoder is unavailable");
      const decoder = new self.fzstd.Decompress((chunk) => builder.push(chunk));
      await streamResponse(options.url, (chunk, final) => decoder.push(chunk, final), controller.signal);
    } else if (compression === "none") {
      await streamResponse(options.url, (chunk) => builder.push(chunk), controller.signal);
    } else if (compression === "bzip2") {
      decodeBzipToBuilder(await bufferedResponse(options.url, controller.signal), builder);
    } else {
      throw new Error(`unsupported-client-compression:${compression || "unknown"}`);
    }
  } catch (error) {
    if (!isRecoverableTailError(error)) throw error;
  }
  return builder.finish();
}

self.onmessage = async (event) => {
  const payload = event?.data || {};
  if (payload.type === "abort") {
    self.sensorAbortController?.abort?.();
    return;
  }
  if (payload.type !== "start") return;
  try {
    self.postMessage({ type: "complete", ...(await buildTracks(payload)) });
  } catch (error) {
    if (error?.name === "AbortError") return;
    self.postMessage({ type: "error", error: error?.message || String(error) });
  }
};
