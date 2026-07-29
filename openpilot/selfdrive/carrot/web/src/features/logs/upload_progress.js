const TERMINAL_UPLOAD_PHASES = new Set(["complete", "failed", "canceled"]);
const PHASE_PROGRESS_FALLBACK = Object.freeze({
  queued: 0,
  preparing: 1,
  uploading: 8,
  notifying: 98,
  canceling: 98,
  complete: 100,
  canceled: 0,
  failed: 0,
});

function finiteNonNegative(value) {
  if (value == null || value === "") return Number.NaN;
  const numeric = Number(value);
  return Number.isFinite(numeric) && numeric >= 0 ? numeric : Number.NaN;
}

function boundedPercent(value) {
  return Math.max(0, Math.min(100, value));
}

function normalizedPhase(snapshot) {
  const phase = String(snapshot?.phase || "").trim().toLowerCase();
  if (phase) return phase;
  const status = String(snapshot?.status || "").trim().toLowerCase();
  if (status === "done") return "complete";
  if (status === "failed" || status === "canceled") return status;
  return "queued";
}

export function dashcamUploadProgressState(snapshot = {}) {
  const phase = normalizedPhase(snapshot);
  const status = String(snapshot.status || "").trim().toLowerCase();
  const terminal = snapshot.done === true
    || status === "done"
    || status === "failed"
    || status === "canceled"
    || TERMINAL_UPLOAD_PHASES.has(phase);
  const completed = status === "done" || phase === "complete";

  if (completed) return { mode: "determinate", value: 100 };

  const reported = finiteNonNegative(snapshot.progress);
  if (Number.isFinite(reported)) {
    return { mode: "determinate", value: boundedPercent(reported) };
  }

  const bytesCurrent = finiteNonNegative(snapshot.bytes_current);
  const bytesTotal = finiteNonNegative(snapshot.bytes_total);
  if (Number.isFinite(bytesCurrent) && Number.isFinite(bytesTotal) && bytesTotal > 0) {
    const transferPercent = 8 + ((Math.min(bytesCurrent, bytesTotal) / bytesTotal) * 89);
    return {
      mode: "determinate",
      value: boundedPercent(Math.min(97, transferPercent)),
    };
  }

  const fallback = PHASE_PROGRESS_FALLBACK[phase];
  if (Number.isFinite(fallback)) {
    return {
      mode: "determinate",
      value: terminal ? Math.max(0, fallback) : fallback,
    };
  }

  return { mode: "indeterminate", value: null };
}

export function createDashcamUploadProgressModel() {
  let jobId = "";
  let revision = -1;
  let value = 0;
  let last = Object.freeze({
    accepted: true,
    phase: "queued",
    progressState: Object.freeze({ mode: "determinate", value: 0 }),
  });

  function reset(nextJobId = "") {
    jobId = String(nextJobId || "");
    revision = -1;
    value = 0;
  }

  function update(snapshot = {}) {
    const nextJobId = String(snapshot.id || "");
    if (nextJobId && nextJobId !== jobId) reset(nextJobId);

    const nextRevision = finiteNonNegative(snapshot.revision);
    if (Number.isFinite(nextRevision) && revision >= 0 && nextRevision < revision) {
      return Object.freeze({ ...last, accepted: false });
    }
    if (Number.isFinite(nextRevision)) revision = nextRevision;

    const phase = normalizedPhase(snapshot);
    const nextState = dashcamUploadProgressState(snapshot);
    const completed = phase === "complete" || String(snapshot.status || "").toLowerCase() === "done";
    if (nextState.mode === "determinate") {
      value = completed ? 100 : Math.max(value, nextState.value);
    }
    last = Object.freeze({
      accepted: true,
      phase,
      revision,
      progressState: Object.freeze(
        nextState.mode === "determinate"
          ? { mode: "determinate", value }
          : nextState,
      ),
    });
    return last;
  }

  return Object.freeze({
    update,
    reset,
    state() {
      return last;
    },
  });
}
