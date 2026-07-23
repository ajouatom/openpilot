export const ROUTE_SUMMARY_PROGRESS_INTERVAL_MS = 150;

export function createRouteSummaryProgressEmitter(send, options = {}) {
  if (typeof send !== "function") throw new TypeError("progress-send-required");

  const requestedInterval = Number(options.intervalMs);
  const intervalMs = Number.isFinite(requestedInterval)
    ? Math.min(200, Math.max(100, requestedInterval))
    : ROUTE_SUMMARY_PROGRESS_INTERVAL_MS;
  const now = typeof options.now === "function" ? options.now : () => performance.now();
  let lastSentAt = null;
  let pending = null;

  const emit = (timestamp) => {
    if (!pending) return false;
    const payload = pending;
    pending = null;
    lastSentAt = timestamp;
    send(payload);
    return true;
  };

  return Object.freeze({
    update(payload) {
      pending = payload;
      const timestamp = now();
      if (lastSentAt !== null && timestamp - lastSentAt < intervalMs) return false;
      return emit(timestamp);
    },
    flush(payload) {
      if (payload) pending = payload;
      return emit(now());
    },
  });
}
