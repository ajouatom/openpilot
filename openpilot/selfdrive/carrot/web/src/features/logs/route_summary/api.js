function summaryError(code) {
  const error = new Error(code);
  error.code = code;
  return error;
}

export async function fetchRouteSummarySource(route, options = {}) {
  const response = await fetch(`/api/dashcam/summary-source/${encodeURIComponent(route)}`, {
    cache: "no-store",
    signal: options.signal,
  });
  let payload = null;
  try {
    payload = await response.json();
  } catch {}
  if (!response.ok || payload?.ok === false) throw summaryError("summary-source-unavailable");
  if (payload?.mode !== "client-worker" || !Array.isArray(payload.segments)) {
    throw summaryError("summary-source-invalid");
  }
  const segments = payload.segments.filter((item) => (
    item
    && typeof item.url === "string"
    && ["zstd", "bzip2", "none"].includes(item.compression)
  ));
  if (!segments.length) throw summaryError("summary-source-empty");
  return Object.freeze({
    route: String(payload.route || route),
    schemaVersion: Number(payload.schemaVersion) || 1,
    segmentCount: Math.max(segments.length, Number(payload.segmentCount) || 0),
    skippedSegments: Math.max(0, Number(payload.skippedSegments) || 0),
    segments: Object.freeze(segments.map((item) => Object.freeze({
      segment: String(item.segment || ""),
      index: Number(item.index) || 0,
      kind: item.kind === "qlog" ? "qlog" : "rlog",
      name: String(item.name || ""),
      url: item.url,
      size: Math.max(0, Number(item.size) || 0),
      modifiedMs: Math.max(0, Number(item.modifiedMs) || 0),
      compression: item.compression,
    }))),
  });
}
