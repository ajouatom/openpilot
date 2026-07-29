export function normalizeAppProgressState(input) {
  if (input && typeof input === "object") {
    const mode = String(input.mode || "").trim().toLowerCase();
    if (mode === "indeterminate") return { mode, value: null };
    if (mode === "determinate") {
      const numeric = input.value == null ? Number.NaN : Number(input.value);
      if (Number.isFinite(numeric)) {
        return {
          mode,
          value: Math.max(0, Math.min(100, numeric)),
        };
      }
      return { mode: "indeterminate", value: null };
    }
  }

  const numeric = input == null ? Number.NaN : Number(input);
  return Number.isFinite(numeric)
    ? { mode: "determinate", value: Math.max(0, Math.min(100, numeric)) }
    : { mode: "indeterminate", value: null };
}

export function applyAppProgressState(progress, output, input) {
  const state = normalizeAppProgressState(input);
  if (!progress) return state;

  const determinate = state.mode === "determinate";
  progress.dataset.progressMode = state.mode;
  progress.setAttribute("aria-busy", determinate ? "false" : "true");
  if (!determinate) {
    progress.removeAttribute("value");
    progress.removeAttribute("aria-valuenow");
    progress.removeAttribute("data-progress-value");
    if (output) {
      output.textContent = "…";
      output.removeAttribute("data-progress-value");
    }
    return state;
  }

  const rounded = Math.round(state.value);
  progress.max = 100;
  progress.value = state.value;
  progress.setAttribute("value", String(state.value));
  progress.setAttribute("aria-valuenow", String(rounded));
  progress.dataset.progressValue = String(state.value);
  if (output) {
    output.textContent = `${rounded}%`;
    output.dataset.progressValue = String(state.value);
  }
  return state;
}

function element(documentRoot, tag, className) {
  const node = documentRoot.createElement(tag);
  node.className = className;
  return node;
}

export function createAppProgressView(documentRoot, options = {}) {
  if (!documentRoot?.createElement) return null;

  const root = element(documentRoot, "div", "app-progress");
  const header = element(documentRoot, "div", "app-progress__header");
  const message = element(documentRoot, "div", "app-progress__message");
  const output = element(documentRoot, "output", "app-progress__value");
  const progress = element(documentRoot, "progress", "app-progress__track");
  const summary = element(documentRoot, "div", "app-progress__summary");

  message.setAttribute("aria-live", "polite");
  output.setAttribute("aria-live", "polite");
  progress.max = 100;
  progress.setAttribute("max", "100");
  if (options.progressLabel) progress.setAttribute("aria-label", String(options.progressLabel));

  header.append(message, output);
  root.append(header, progress, summary);

  const controller = {
    element: root,
    progressElement: progress,
    valueElement: output,
    setMessage(value) {
      message.textContent = String(value || "");
    },
    setProgressState(value) {
      return applyAppProgressState(progress, output, value);
    },
    setSummary(value) {
      summary.textContent = String(value || "");
    },
  };
  controller.setMessage(options.message);
  controller.setProgressState(options.progressState ?? options.progress);
  controller.setSummary(options.summary);
  return Object.freeze(controller);
}
