const ROUTE_SUMMARY_WORKER_URL = "/js/realtime/route_summary_worker.js?v=2607-02";

export function analyzeRouteSummary(source, options = {}) {
  if (typeof Worker !== "function") {
    return Promise.reject(Object.assign(new Error("summary-worker-unsupported"), { code: "summary-worker-unsupported" }));
  }
  return new Promise((resolve, reject) => {
    const worker = new Worker(ROUTE_SUMMARY_WORKER_URL);
    const signal = options.signal;
    let settled = false;
    const finish = (callback, value) => {
      if (settled) return;
      settled = true;
      signal?.removeEventListener?.("abort", onAbort);
      worker.terminate();
      callback(value);
    };
    const onAbort = () => {
      try { worker.postMessage({ type: "abort" }); } catch {}
      finish(reject, new DOMException("Aborted", "AbortError"));
    };
    worker.onmessage = (event) => {
      const payload = event?.data || {};
      if (payload.type === "progress") {
        options.onProgress?.(payload);
      } else if (payload.type === "warning") {
        options.onWarning?.(payload);
      } else if (payload.type === "complete") {
        finish(resolve, payload.result);
      } else if (payload.type === "error") {
        const error = Object.assign(new Error(String(payload.code || "summary-worker-failed")), {
          code: String(payload.code || "summary-worker-failed"),
        });
        finish(reject, error);
      }
    };
    worker.onerror = () => {
      finish(reject, Object.assign(new Error("summary-worker-failed"), { code: "summary-worker-failed" }));
    };
    if (signal?.aborted) {
      onAbort();
      return;
    }
    signal?.addEventListener?.("abort", onAbort, { once: true });
    worker.postMessage({
      type: "analyze-route",
      route: source.route,
      schemaVersion: source.schemaVersion,
      segmentCount: source.segmentCount,
      skippedSegments: source.skippedSegments,
      segments: source.segments,
    });
  });
}
