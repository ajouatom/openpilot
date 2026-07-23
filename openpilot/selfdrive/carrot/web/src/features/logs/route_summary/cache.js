const ROUTE_SUMMARY_CACHE_LIMIT = 6;
const resultCache = new Map();

export function routeSummarySourceFingerprint(source) {
  if (!source || !Array.isArray(source.segments) || !source.segments.length) return "";
  return JSON.stringify([
    String(source.route || ""),
    Number(source.schemaVersion) || 0,
    Number(source.segmentCount) || 0,
    Number(source.skippedSegments) || 0,
    source.segments.map((item) => [
      String(item.segment || ""),
      Number(item.index) || 0,
      String(item.kind || ""),
      String(item.name || ""),
      Number(item.size) || 0,
      Number(item.modifiedMs) || 0,
      String(item.compression || ""),
    ]),
  ]);
}

export function getCachedRouteSummary(source) {
  const fingerprint = routeSummarySourceFingerprint(source);
  if (!fingerprint || !resultCache.has(fingerprint)) return null;
  const result = resultCache.get(fingerprint);
  resultCache.delete(fingerprint);
  resultCache.set(fingerprint, result);
  return result;
}

export function setCachedRouteSummary(source, result) {
  const fingerprint = routeSummarySourceFingerprint(source);
  if (!fingerprint || !result?.ok) return false;
  resultCache.delete(fingerprint);
  resultCache.set(fingerprint, result);
  while (resultCache.size > ROUTE_SUMMARY_CACHE_LIMIT) {
    resultCache.delete(resultCache.keys().next().value);
  }
  return true;
}

export function clearRouteSummaryCache() {
  resultCache.clear();
}
