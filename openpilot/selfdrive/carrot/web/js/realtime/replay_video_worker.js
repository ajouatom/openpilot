"use strict";

self.window = self;
self.importScripts("/js/vendor/mux-mp4.js");

const TS_PACKET_BYTES = 188;
const H264_STREAM_TYPE = 0x1b;
const PROGRAM_MAP_PROBE_BYTES = 1024 * 1024;

function packetPayloadOffset(data, packetStart) {
  if (data[packetStart] !== 0x47 || (data[packetStart + 1] & 0x80)) return -1;
  const adaptationControl = (data[packetStart + 3] >> 4) & 0x03;
  if (adaptationControl === 0 || adaptationControl === 2) return -1;
  let offset = packetStart + 4;
  if (adaptationControl === 3) offset += 1 + data[offset];
  return offset < packetStart + TS_PACKET_BYTES ? offset : -1;
}

function findTsSyncOffset(data) {
  const limit = Math.min(TS_PACKET_BYTES, Math.max(0, data.byteLength - TS_PACKET_BYTES * 2));
  for (let offset = 0; offset < limit; offset += 1) {
    if (data[offset] === 0x47 && data[offset + TS_PACKET_BYTES] === 0x47
        && data[offset + TS_PACKET_BYTES * 2] === 0x47) return offset;
  }
  return -1;
}

function collectPsiSection(data, packetStart, payloadOffset, pid) {
  const packetEnd = packetStart + TS_PACKET_BYTES;
  const pointer = data[payloadOffset];
  const sectionStart = payloadOffset + 1 + pointer;
  if (sectionStart + 3 > packetEnd || data[sectionStart] !== 0x02) return null;
  const sectionBytes = 3 + (((data[sectionStart + 1] & 0x0f) << 8) | data[sectionStart + 2]);
  if (sectionBytes < 16 || sectionBytes > 4096) return null;
  const section = new Uint8Array(sectionBytes);
  let written = Math.min(sectionBytes, packetEnd - sectionStart);
  section.set(data.subarray(sectionStart, sectionStart + written));
  for (let next = packetStart + TS_PACKET_BYTES;
    written < sectionBytes && next + TS_PACKET_BYTES <= data.byteLength;
    next += TS_PACKET_BYTES) {
    const nextPid = ((data[next + 1] & 0x1f) << 8) | data[next + 2];
    if (nextPid !== pid) continue;
    const nextPayload = packetPayloadOffset(data, next);
    if (nextPayload < 0) continue;
    const available = Math.min(sectionBytes - written, next + TS_PACKET_BYTES - nextPayload);
    section.set(data.subarray(nextPayload, nextPayload + available), written);
    written += available;
  }
  return written === sectionBytes ? section : null;
}

function inspectProgramMap(data) {
  const syncOffset = findTsSyncOffset(data);
  if (syncOffset < 0) return null;
  for (let packetStart = syncOffset;
    packetStart + TS_PACKET_BYTES <= data.byteLength;
    packetStart += TS_PACKET_BYTES) {
    if (!(data[packetStart + 1] & 0x40)) continue;
    const payloadOffset = packetPayloadOffset(data, packetStart);
    if (payloadOffset < 0) continue;
    const pid = ((data[packetStart + 1] & 0x1f) << 8) | data[packetStart + 2];
    const section = collectPsiSection(data, packetStart, payloadOffset, pid);
    if (!section) continue;
    const programInfoBytes = ((section[10] & 0x0f) << 8) | section[11];
    const streamEnd = section.byteLength - 4;
    const streams = [];
    for (let offset = 12 + programInfoBytes; offset + 5 <= streamEnd;) {
      const streamType = section[offset];
      const streamPid = ((section[offset + 1] & 0x1f) << 8) | section[offset + 2];
      const infoBytes = ((section[offset + 3] & 0x0f) << 8) | section[offset + 4];
      streams.push({ streamType, streamPid });
      offset += 5 + infoBytes;
    }
    const videoStreams = streams.filter((stream) => stream.streamType === H264_STREAM_TYPE);
    return {
      videoPid: videoStreams.length === 1 ? videoStreams[0].streamPid : -1,
      // qcamera is video-only. If another elementary stream exists, retain the
      // whole TS until EOF rather than risk cutting an audio/metadata PES.
      canSegment: videoStreams.length === 1 && streams.length === 1,
    };
  }
  return null;
}

function findSecondIdrBoundary(data, videoPid) {
  const syncOffset = findTsSyncOffset(data);
  if (syncOffset < 0 || videoPid < 0) return -1;
  let pesStart = -1;
  let pesHasIdr = false;
  let zeroBytes = 0;
  let expectNalHeader = false;
  const idrStarts = [];
  for (let packetStart = syncOffset;
    packetStart + TS_PACKET_BYTES <= data.byteLength;
    packetStart += TS_PACKET_BYTES) {
    const pid = ((data[packetStart + 1] & 0x1f) << 8) | data[packetStart + 2];
    if (pid !== videoPid) continue;
    const payloadOffset = packetPayloadOffset(data, packetStart);
    if (payloadOffset < 0) continue;
    if (data[packetStart + 1] & 0x40) {
      const isVideoPes = payloadOffset + 4 <= packetStart + TS_PACKET_BYTES
        && data[payloadOffset] === 0x00 && data[payloadOffset + 1] === 0x00
        && data[payloadOffset + 2] === 0x01 && (data[payloadOffset + 3] & 0xf0) === 0xe0;
      pesStart = isVideoPes ? packetStart : -1;
      pesHasIdr = false;
      zeroBytes = 0;
      expectNalHeader = false;
    }
    if (pesStart < 0 || pesHasIdr) continue;
    const packetEnd = packetStart + TS_PACKET_BYTES;
    for (let offset = payloadOffset; offset < packetEnd; offset += 1) {
      const value = data[offset];
      if (expectNalHeader) {
        expectNalHeader = false;
        if ((value & 0x1f) === 5) {
          pesHasIdr = true;
          idrStarts.push(pesStart);
          if (idrStarts.length >= 2) return idrStarts[1];
          break;
        }
        zeroBytes = value === 0 ? 1 : 0;
      } else if (value === 0) {
        zeroBytes += 1;
      } else if (value === 1 && zeroBytes >= 2) {
        expectNalHeader = true;
        zeroBytes = 0;
      } else {
        zeroBytes = 0;
      }
    }
  }
  return -1;
}

function appendBytes(left, right) {
  if (!left.byteLength) return new Uint8Array(right);
  const joined = new Uint8Array(left.byteLength + right.byteLength);
  joined.set(left);
  joined.set(right, left.byteLength);
  return joined;
}

class TsKeyframeSegmenter {
  constructor() {
    this.buffer = new Uint8Array(0);
    this.programMap = null;
    this.passthrough = false;
  }

  push(chunk) {
    if (this.passthrough) return { segments: [], passthrough: [chunk] };
    this.buffer = appendBytes(this.buffer, chunk);
    if (!this.programMap) this.programMap = inspectProgramMap(this.buffer);
    const segments = [];
    if (this.programMap && !this.programMap.canSegment) {
      this.passthrough = true;
      const pending = this.finish();
      return { segments, passthrough: pending.byteLength ? [pending] : [] };
    }
    if (!this.programMap) {
      if (this.buffer.byteLength >= PROGRAM_MAP_PROBE_BYTES) {
        this.passthrough = true;
        const pending = this.finish();
        return { segments, passthrough: [pending] };
      }
      return { segments, passthrough: [] };
    }
    while (true) {
      const boundary = findSecondIdrBoundary(this.buffer, this.programMap.videoPid);
      if (boundary <= 0) break;
      segments.push(this.buffer.slice(0, boundary));
      this.buffer = this.buffer.slice(boundary);
    }
    return { segments, passthrough: [] };
  }

  finish() {
    const remainder = this.buffer;
    this.buffer = new Uint8Array(0);
    return remainder;
  }
}

function boxCodec(initSegment) {
  let avcCodec = "avc1.64001f";
  let hasAudio = false;
  for (let index = 4; index + 8 < initSegment.byteLength; index += 1) {
    const type = String.fromCharCode(
      initSegment[index], initSegment[index + 1], initSegment[index + 2], initSegment[index + 3],
    );
    if (type === "avcC" && index + 8 < initSegment.byteLength) {
      avcCodec = `avc1.${[initSegment[index + 5], initSegment[index + 6], initSegment[index + 7]]
        .map((value) => {
          const hex = value.toString(16);
          return hex.length < 2 ? `0${hex}` : hex;
        }).join("")}`;
    } else if (type === "mp4a") {
      hasAudio = true;
    }
  }
  return `video/mp4; codecs="${avcCodec}${hasAudio ? ", mp4a.40.2" : ""}"`;
}

async function transmux(options) {
  const Transmuxer = self.muxjs?.mp4?.Transmuxer || self.muxjs?.Transmuxer;
  if (!Transmuxer) throw new Error("Client MPEG-TS transmuxer is unavailable");
  const controller = new AbortController();
  self.replayAbortController = controller;
  const response = await fetch(options.url, { cache: "no-store", signal: controller.signal });
  if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
  const total = Number(response.headers.get("Content-Length")) || 0;
  const transmuxer = new Transmuxer({ remux: true, keepOriginalTimestamps: false });
  let initialized = false;
  let outputCount = 0;
  let loaded = 0;
  let mime = "";
  const segmenter = new TsKeyframeSegmenter();

  transmuxer.on("data", (segment) => {
    const init = initialized ? null : new Uint8Array(segment.initSegment);
    const media = new Uint8Array(segment.data);
    if (!initialized) {
      mime = boxCodec(init);
      initialized = true;
    }
    const joined = new Uint8Array((init?.byteLength || 0) + media.byteLength);
    if (init) joined.set(init, 0);
    joined.set(media, init?.byteLength || 0);
    outputCount += 1;
    self.postMessage({ type: "chunk", mime, data: joined.buffer, index: outputCount - 1 }, [joined.buffer]);
  });

  const push = (chunk) => {
    if (!chunk?.byteLength) return;
    loaded += chunk.byteLength;
    const output = segmenter.push(chunk);
    for (const pending of output.passthrough) transmuxer.push(pending);
    for (const segment of output.segments) {
      transmuxer.push(segment);
      transmuxer.flush();
    }
    self.postMessage({ type: "progress", loaded, total });
  };

  if (response.body?.getReader) {
    const reader = response.body.getReader();
    while (true) {
      const result = await reader.read();
      if (result.done) break;
      push(result.value);
    }
  } else {
    push(new Uint8Array(await response.arrayBuffer()));
  }
  const remainder = segmenter.finish();
  if (remainder.byteLength) transmuxer.push(remainder);
  transmuxer.flush();
  if (!initialized || outputCount <= 0) throw new Error("Recorded MPEG-TS video could not be repackaged");
  self.postMessage({ type: "complete", mime, outputCount, loaded, total: total || loaded });
}

self.onmessage = async (event) => {
  const payload = event?.data || {};
  if (payload.type === "abort") {
    self.replayAbortController?.abort?.();
    return;
  }
  if (payload.type !== "start") return;
  try {
    await transmux(payload);
  } catch (error) {
    if (error?.name === "AbortError") return;
    self.postMessage({ type: "error", error: error?.message || String(error) });
  }
};
