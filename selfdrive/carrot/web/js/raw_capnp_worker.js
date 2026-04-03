"use strict";

self.importScripts("/js/raw_capnp.js");

const rawCapnp = self.CarrotRawCapnp || null;

function decodeMessage(streamKind, service, buffer) {
  if (!rawCapnp) {
    throw new Error("CarrotRawCapnp unavailable in worker");
  }
  if (streamKind === "hud") {
    return rawCapnp.decodeHudEvent(service, buffer);
  }
  if (streamKind === "overlay") {
    return rawCapnp.decodeOverlayEvent(service, buffer);
  }
  throw new Error(`unsupported stream kind: ${streamKind}`);
}

self.onmessage = (event) => {
  const payload = event?.data || {};
  const requestId = Number(payload.requestId);
  const streamKind = String(payload.streamKind || "");
  const service = String(payload.service || "");
  const buffer = payload.buffer;

  try {
    const decoded = decodeMessage(streamKind, service, buffer);
    self.postMessage({
      requestId,
      streamKind,
      service,
      decoded,
      ok: true,
    });
  } catch (error) {
    self.postMessage({
      requestId,
      streamKind,
      service,
      ok: false,
      error: error?.message || String(error),
    });
  }
};
