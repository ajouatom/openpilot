import { loadOnnxVisionStatus } from "./api.js";
import { createConfidenceMeter } from "./components/confidence_meter.js";
import { mountOnnxPolygonEditor } from "./components/polygon_editor.js";
import { ONNX_VISION_STATE, normalizeOnnxVisionStatus } from "./model.js";

const POLL_INTERVAL_MS = 1000;
const LANE_SETTING_NAMES = Object.freeze(["OnnxLaneThreshold", "OnnxLaneIntervalMs"]);
const BSD_SETTING_NAMES = Object.freeze(["OnnxBsdThreshold", "OnnxBsdSmoothingMs", "OnnxBsdIntervalMs"]);

function text(key, fallback, vars) {
  if (typeof globalThis.getUIText === "function") return globalThis.getUIText(key, fallback, vars);
  let value = fallback || key;
  Object.entries(vars || {}).forEach(([name, replacement]) => {
    value = value.replaceAll(`{${name}}`, String(replacement));
  });
  return value;
}

function escapeHtml(value) {
  return String(value ?? "").replace(/[&<>'"]/g, (character) => ({
    "&": "&amp;", "<": "&lt;", ">": "&gt;", "'": "&#039;", '"': "&quot;",
  }[character]));
}

function enabled(setting) {
  return setting.querySelector('input[type="checkbox"]')?.checked === true;
}

function stateText(state) {
  const labels = {
    [ONNX_VISION_STATE.DISABLED]: ["onnx_vision_state_disabled", "Off"],
    [ONNX_VISION_STATE.STARTING]: ["onnx_vision_state_starting", "Starting"],
    [ONNX_VISION_STATE.UNAVAILABLE]: ["onnx_vision_state_unavailable", "Unavailable"],
    [ONNX_VISION_STATE.WAITING]: ["onnx_vision_state_waiting", "Waiting for camera"],
    [ONNX_VISION_STATE.READY]: ["onnx_vision_state_ready", "Ready"],
    [ONNX_VISION_STATE.ATTENTION]: ["onnx_vision_state_attention", "Needs attention"],
  };
  const [key, fallback] = labels[state] || labels[ONNX_VISION_STATE.ATTENTION];
  return text(key, fallback);
}

function summaryText(status) {
  const summaries = {
    [ONNX_VISION_STATE.DISABLED]: ["onnx_vision_summary_disabled", "Turn on ONNX lane and BSD detection to see live status."],
    [ONNX_VISION_STATE.STARTING]: ["onnx_vision_summary_starting", "Waiting for the ONNX service to start."],
    [ONNX_VISION_STATE.UNAVAILABLE]: ["onnx_vision_summary_unavailable", "The ONNX service is not responding."],
    [ONNX_VISION_STATE.WAITING]: ["onnx_vision_summary_waiting", "Models are ready; waiting for road cameras."],
    [ONNX_VISION_STATE.READY]: ["onnx_vision_summary_ready", "Lane and blindspot detection are ready."],
    [ONNX_VISION_STATE.ATTENTION]: ["onnx_vision_summary_attention", "Check the ONNX model or its configuration."],
  };
  const [key, fallback] = summaries[status.state] || summaries[ONNX_VISION_STATE.ATTENTION];
  return text(key, fallback);
}

function laneText(value) {
  if (value === "solid") return text("onnx_vision_lane_solid", "Solid");
  if (value === "dashed") return text("onnx_vision_lane_dashed", "Dashed");
  return text("onnx_vision_lane_unknown", "Unknown");
}

function sideText(side) {
  return side === "left"
    ? text("onnx_vision_side_left", "Left")
    : text("onnx_vision_side_right", "Right");
}

function formatNumber(value, digits = 1, suffix = "") {
  return Number.isFinite(value) ? `${Number(value).toFixed(digits)}${suffix}` : "—";
}

function performanceText(fps, latencyMs) {
  const parts = [];
  if (Number.isFinite(fps)) parts.push(`${fps.toFixed(1)} Hz`);
  if (Number.isFinite(latencyMs)) parts.push(`${latencyMs.toFixed(0)} ms`);
  return parts.join(" · ") || "—";
}

function setChip(chip, label, tone = "") {
  chip.className = "chip chip--compact onnx-vision-card__state";
  if (tone) chip.classList.add(`chip--${tone}`);
  chip.textContent = label;
}

function metric(documentRoot, label, role) {
  const item = documentRoot.createElement("div");
  item.className = "setting-profile-row onnx-vision-card__metric";
  const name = documentRoot.createElement("span");
  name.className = "setting-profile-row__label";
  name.textContent = label;
  const detail = documentRoot.createElement("strong");
  detail.className = "setting-profile-row__value";
  detail.dataset.role = role;
  detail.textContent = "—";
  item.append(name, detail);
  return item;
}

function createSideStatus(documentRoot, side, kind) {
  const card = documentRoot.createElement("div");
  card.className = "onnx-side-status";
  const head = documentRoot.createElement("div");
  head.className = "onnx-side-status__head";
  const title = documentRoot.createElement("strong");
  title.textContent = sideText(side);
  const state = documentRoot.createElement("span");
  state.className = "chip chip--compact";
  state.dataset.role = `${kind}-${side}-state`;
  head.append(title, state);
  const meter = createConfidenceMeter(documentRoot, {
    label: text("onnx_vision_confidence", "Confidence"),
  });
  card.append(head, meter.root);
  return Object.freeze({ card, state, meter });
}

function createSection(documentRoot, titleText, kind, settingRows) {
  const section = documentRoot.createElement("section");
  section.className = "setting-section-block onnx-vision-section ui-stagger-item";
  section.dataset.onnxSection = kind;

  const card = documentRoot.createElement("div");
  card.className = "setting-group-card onnx-vision-feature-card";
  const header = documentRoot.createElement("div");
  header.className = "setting-profile-row setting-profile-row--name onnx-vision-feature-card__head";
  const copy = documentRoot.createElement("div");
  copy.className = "onnx-vision-card__copy";
  const heading = documentRoot.createElement("strong");
  heading.className = "setting-profile-row__label";
  heading.textContent = titleText;
  const summary = documentRoot.createElement("p");
  summary.className = "onnx-vision-card__summary";
  summary.dataset.role = `${kind}-summary`;
  copy.append(heading, summary);
  const state = documentRoot.createElement("span");
  state.dataset.role = `${kind}-state`;
  header.append(copy, state);

  const sides = documentRoot.createElement("div");
  sides.className = "onnx-side-grid";
  const left = createSideStatus(documentRoot, "left", kind);
  const right = createSideStatus(documentRoot, "right", kind);
  sides.append(left.card, right.card);

  const metrics = documentRoot.createElement("div");
  metrics.className = "setting-profile-card__rows onnx-vision-feature-card__metrics";
  if (kind === "lane") {
    metrics.append(
      metric(documentRoot, text("onnx_vision_candidates", "Candidates"), "lane-candidates"),
      metric(documentRoot, text("onnx_vision_metric_performance", "Performance"), "lane-performance"),
    );
  } else {
    metrics.append(
      metric(documentRoot, text("onnx_vision_gate", "Inference condition"), "bsd-gate"),
      metric(documentRoot, text("onnx_vision_metric_performance", "Performance"), "bsd-performance"),
    );
  }

  const settingsBody = documentRoot.createElement("div");
  settingsBody.className = "setting-group-card__body onnx-vision-feature-card__settings";
  settingRows.forEach((row) => settingsBody.appendChild(row));

  card.append(header, sides, metrics, settingsBody);
  section.append(card);
  return Object.freeze({ section, card, state, summary, left, right });
}

function createRuntimeSection(documentRoot) {
  const section = documentRoot.createElement("section");
  section.className = "setting-section-block onnx-vision-section ui-stagger-item";
  section.dataset.settingsExtensionPanel = "onnx-vision";
  const card = documentRoot.createElement("div");
  card.className = "setting-group-card onnx-vision-card";
  card.innerHTML = `
    <div class="setting-profile-card__rows">
      <div class="setting-profile-row setting-profile-row--name onnx-vision-card__head">
        <div class="onnx-vision-card__copy">
          <div class="setting-profile-row__label" data-role="title"></div>
          <p class="onnx-vision-card__summary" data-role="summary"></p>
        </div>
        <span data-role="state" role="status"></span>
      </div>
      <div data-role="runtime-metrics"></div>
    </div>
    <div class="ui-action-grid onnx-vision-card__actions">
      <button type="button" class="smallBtn" data-role="diagnostics"></button>
    </div>
  `;
  const metrics = card.querySelector('[data-role="runtime-metrics"]');
  metrics.append(
    metric(documentRoot, text("onnx_vision_metric_models", "Models"), "models"),
    metric(documentRoot, text("onnx_vision_cameras", "Cameras"), "cameras"),
  );
  section.appendChild(card);
  return Object.freeze({ section, card });
}

function createEditorSection(documentRoot) {
  const section = documentRoot.createElement("section");
  section.className = "setting-section-block onnx-vision-section onnx-vision-editor-section ui-stagger-item";
  section.dataset.settingsExtensionPanel = "onnx-vision";
  const card = documentRoot.createElement("div");
  card.className = "setting-group-card onnx-vision-editor-card";
  const host = documentRoot.createElement("div");
  host.className = "onnx-vision-editor-host";
  card.appendChild(host);
  section.appendChild(card);
  return Object.freeze({ section, card, host });
}

function diagnosticsRows(status) {
  const service = status.service || {};
  return [
    [text("onnx_vision_metric_service", "Service"), stateText(status.state)],
    [text("onnx_vision_bsd_model", "BSD model"), service.bsdModelReady ? text("ready", "Ready") : (service.bsdModelError || text("onnx_vision_models_check", "Check needed"))],
    [text("onnx_vision_lane_model", "Lane model"), service.laneModelReady ? text("ready", "Ready") : (service.laneModelError || text("onnx_vision_models_check", "Check needed"))],
    [text("onnx_vision_road_camera", "Road camera"), status.lane.cameraReady ? text("ready", "Ready") : (status.lane.cameraError || "—")],
    [text("onnx_vision_wide_camera", "Wide camera"), status.bsd.cameraReady ? text("ready", "Ready") : (status.bsd.cameraError || "—")],
    [text("onnx_vision_lane_count", "Lane inferences"), String(status.lane.count)],
    [text("onnx_vision_bsd_count", "BSD inferences"), String(status.bsd.count)],
    [text("onnx_vision_gate", "Inference condition"), status.bsd.gateReason || text("onnx_vision_gate_active", "Active")],
  ];
}

function openDiagnostics(status) {
  const raw = JSON.stringify(status.raw || {}, null, 2);
  const rows = diagnosticsRows(status);
  const html = `
    <div class="onnx-vision-dialog">
      <p class="onnx-vision-dialog__summary">${escapeHtml(status.error || summaryText(status))}</p>
      <dl class="onnx-vision-dialog__list">${rows.map(([label, value]) => `<div><dt>${escapeHtml(label)}</dt><dd>${escapeHtml(value)}</dd></div>`).join("")}</dl>
      <details class="onnx-vision-dialog__raw"><summary>${escapeHtml(text("onnx_vision_raw_status", "Raw status"))}</summary><pre>${escapeHtml(raw || "{}")}</pre></details>
    </div>
  `;
  globalThis.appAlert?.("", {
    title: text("onnx_vision_diagnostics", "ONNX diagnostics"),
    html: true,
    messageHtml: html,
    copyText: raw,
    copyLabel: text("copy", "Copy"),
  });
}

function renderRuntime(view, status) {
  const card = view.card;
  card.dataset.state = status.state;
  card.querySelector('[data-role="title"]').textContent = text("onnx_vision_runtime", "ONNX runtime");
  card.querySelector('[data-role="summary"]').textContent = status.error || summaryText(status);
  const tone = status.state === ONNX_VISION_STATE.READY
    ? "success"
    : status.state === ONNX_VISION_STATE.ATTENTION || status.state === ONNX_VISION_STATE.UNAVAILABLE
      ? "warning"
      : status.state === ONNX_VISION_STATE.DISABLED ? "" : "info";
  setChip(card.querySelector('[data-role="state"]'), stateText(status.state), tone);
  const modelsReady = status.service?.bsdModelReady && status.service?.laneModelReady;
  card.querySelector('[data-role="models"]').textContent = status.state === ONNX_VISION_STATE.DISABLED
    ? "—"
    : modelsReady ? text("onnx_vision_models_ready", "Ready") : text("onnx_vision_models_check", "Check needed");
  const cameras = status.state === ONNX_VISION_STATE.DISABLED
    ? "—"
    : `${text("onnx_vision_camera_road_short", "Road")} ${status.lane.cameraReady ? text("ready", "Ready") : text("onnx_vision_camera_wait", "Waiting")} · ${text("onnx_vision_camera_wide_short", "Wide")} ${status.bsd.cameraReady ? text("ready", "Ready") : text("onnx_vision_camera_wait", "Waiting")}`;
  card.querySelector('[data-role="cameras"]').textContent = cameras;
  card.querySelector('[data-role="diagnostics"]').textContent = text("onnx_vision_open_diagnostics", "Open diagnostics");
}

function renderLane(view, status) {
  if ([ONNX_VISION_STATE.DISABLED, ONNX_VISION_STATE.STARTING, ONNX_VISION_STATE.UNAVAILABLE].includes(status.state)) {
    setChip(view.state, stateText(status.state), status.state === ONNX_VISION_STATE.UNAVAILABLE ? "warning" : "info");
    view.summary.textContent = status.error || summaryText(status);
    for (const sideView of [view.left, view.right]) {
      setChip(sideView.state, laneText("unknown"), "");
      sideView.meter.update({
        label: text("onnx_vision_confidence", "Confidence"),
        confidence: 0,
        valid: false,
      });
    }
    view.card.querySelector('[data-role="lane-candidates"]').textContent = "—";
    view.card.querySelector('[data-role="lane-performance"]').textContent = "—";
    return;
  }
  const fresh = status.lane.fresh;
  setChip(view.state, fresh ? text("onnx_vision_live", "Live") : text("onnx_vision_waiting_result", "Waiting"), fresh ? "success" : "info");
  view.summary.textContent = status.lane.cameraError || text("onnx_vision_lane_summary", "Road-camera solid and dashed lane classification");
  for (const [side, sideView] of [["left", view.left], ["right", view.right]]) {
    const result = status.lane[side];
    setChip(sideView.state, laneText(result.line), result.line === "unknown" ? "" : "info");
    sideView.meter.update({
      label: text("onnx_vision_confidence", "Confidence"),
      confidence: result.confidence,
      valid: fresh,
    });
  }
  view.card.querySelector('[data-role="lane-candidates"]').textContent = String(status.lane.candidates);
  view.card.querySelector('[data-role="lane-performance"]').textContent = performanceText(status.lane.fps, status.lane.latencyMs);
}

function bsdSideLabel(result, configured) {
  if (!configured) return text("onnx_vision_bsd_unconfigured", "Not configured");
  if (!result.valid) return text("onnx_vision_bsd_standby", "Standby");
  return result.active
    ? text("onnx_vision_bsd_detected", "Vehicle detected")
    : text("onnx_vision_bsd_clear", "Clear");
}

function renderBsd(view, status) {
  if ([ONNX_VISION_STATE.DISABLED, ONNX_VISION_STATE.STARTING, ONNX_VISION_STATE.UNAVAILABLE].includes(status.state)) {
    setChip(view.state, stateText(status.state), status.state === ONNX_VISION_STATE.UNAVAILABLE ? "warning" : "info");
    view.summary.textContent = status.error || summaryText(status);
    for (const sideView of [view.left, view.right]) {
      setChip(sideView.state, text("onnx_vision_bsd_standby", "Standby"), "");
      sideView.meter.update({
        label: text("onnx_vision_confidence", "Confidence"),
        confidence: 0,
        valid: false,
      });
    }
    view.card.querySelector('[data-role="bsd-gate"]').textContent = "—";
    view.card.querySelector('[data-role="bsd-performance"]').textContent = "—";
    return;
  }
  const monitoring = status.bsd.active && status.bsd.side;
  const stateLabel = monitoring
    ? text("onnx_vision_bsd_monitoring", "Monitoring {side}", { side: sideText(status.bsd.side) })
    : status.bsd.configured ? text("onnx_vision_bsd_standby", "Standby") : text("onnx_vision_bsd_unconfigured", "Not configured");
  setChip(view.state, stateLabel, monitoring ? "info" : "");
  view.summary.textContent = text("onnx_vision_bsd_summary", "Wide-camera blindspot detection runs only when lane-change conditions are met.");
  for (const [side, sideView] of [["left", view.left], ["right", view.right]]) {
    const result = status.bsd[side];
    const configured = status.bsd.configuredSides.includes(side);
    const label = bsdSideLabel(result, configured);
    setChip(sideView.state, label, result.active ? "warning" : (result.valid ? "success" : ""));
    sideView.meter.update({
      label: text("onnx_vision_confidence", "Confidence"),
      confidence: result.confidence,
      active: result.active,
      valid: result.valid,
    });
  }
  const gateText = monitoring
    ? `${sideText(status.bsd.side)} · ${formatNumber(status.bsd.laneWidth, 1, " m")}`
    : status.bsd.gateReason || "—";
  view.card.querySelector('[data-role="bsd-gate"]').textContent = gateText;
  view.card.querySelector('[data-role="bsd-performance"]').textContent = performanceText(status.bsd.fps, status.bsd.latencyMs);
}

function childRows(root, names) {
  return names.map((name) => root.querySelector(`[data-setting-name="${name}"]`)).filter(Boolean);
}

function createPanel(root) {
  const documentRoot = root.ownerDocument;
  const laneRows = childRows(root, LANE_SETTING_NAMES);
  const bsdRows = childRows(root, BSD_SETTING_NAMES);
  const rowsToRestore = [...new Set([...laneRows, ...bsdRows].filter(Boolean))];
  const placeholders = rowsToRestore.map((row) => {
    const placeholder = documentRoot.createComment(`onnx-vision:${row.dataset.settingName || "setting"}`);
    row.before(placeholder);
    return { row, placeholder };
  });
  const editor = createEditorSection(documentRoot);
  const runtime = createRuntimeSection(documentRoot);
  const lane = createSection(
    documentRoot,
    text("onnx_vision_lane_section", "Lane detection"),
    "lane",
    laneRows,
  );
  const bsd = createSection(
    documentRoot,
    text("onnx_vision_bsd_section", "BSD detection"),
    "bsd",
    bsdRows,
  );
  const advanced = documentRoot.createElement("details");
  advanced.className = "setting-section-block onnx-vision-advanced-settings ui-stagger-item";
  const advancedSummary = documentRoot.createElement("summary");
  advancedSummary.className = "onnx-vision-advanced-settings__summary";
  const advancedTitle = documentRoot.createElement("strong");
  const updateAdvancedTitle = () => {
    advancedTitle.textContent = advanced.open
      ? text("onnx_vision_advanced_collapse", "Collapse advanced settings")
      : text("onnx_vision_advanced_expand", "Expand advanced settings");
  };
  const advancedState = documentRoot.createElement("span");
  advancedState.setAttribute("role", "status");
  advancedSummary.append(advancedTitle, advancedState);
  const advancedBody = documentRoot.createElement("div");
  advancedBody.className = "onnx-vision-advanced-settings__body";
  advancedBody.append(runtime.section, lane.section, bsd.section);
  advanced.append(advancedSummary, advancedBody);
  advanced.addEventListener("toggle", updateAdvancedTitle);
  updateAdvancedTitle();
  editor.section.style.setProperty("--i", "2");
  advanced.style.setProperty("--i", "3");
  const contextPanel = root.querySelector(":scope > .setting-context");
  const fragment = documentRoot.createDocumentFragment();
  fragment.append(editor.section, advanced);
  root.insertBefore(fragment, contextPanel || null);
  return Object.freeze({
    editor,
    advanced,
    advancedState,
    runtime,
    lane,
    bsd,
    roots: [editor.section, advanced],
    restoreRows() {
      placeholders.forEach(({ row, placeholder }) => {
        if (placeholder.isConnected) placeholder.replaceWith(row);
      });
    },
  });
}

/** Mount the migrated 8082 controls only inside the ShareData detail screen. */
export function mountOnnxVisionPanel({ root, setting, lifecycle }) {
  const view = createPanel(root);
  const editor = mountOnnxPolygonEditor(view.editor.host, { embedded: true });
  let loading = false;
  let current = normalizeOnnxVisionStatus({}, {
    enabled: enabled(setting),
    loading: enabled(setting),
  });

  function render() {
    renderRuntime(view.runtime, current);
    renderLane(view.lane, current);
    renderBsd(view.bsd, current);
    const showAdvancedWarning = [ONNX_VISION_STATE.ATTENTION, ONNX_VISION_STATE.UNAVAILABLE].includes(current.state);
    view.advancedState.hidden = !showAdvancedWarning;
    if (showAdvancedWarning) setChip(view.advancedState, stateText(current.state), "warning");
    const serviceAvailable = ![ONNX_VISION_STATE.DISABLED, ONNX_VISION_STATE.UNAVAILABLE, ONNX_VISION_STATE.STARTING].includes(current.state);
    editor?.setCameraAvailable(serviceAvailable);
  }

  view.runtime.card.querySelector('[data-role="diagnostics"]').addEventListener("click", () => openDiagnostics(current));

  async function refresh() {
    if (loading || lifecycle.destroyed || globalThis.document?.hidden) return;
    const onnxEnabled = enabled(setting);
    if (!onnxEnabled) {
      current = normalizeOnnxVisionStatus({}, { enabled: false });
      render();
      return;
    }
    loading = true;
    try {
      const payload = await loadOnnxVisionStatus(globalThis, { signal: lifecycle.signal });
      if (!lifecycle.destroyed) {
        current = normalizeOnnxVisionStatus(payload, { enabled: true });
        render();
      }
    } catch (error) {
      if (!lifecycle.destroyed && error?.name !== "AbortError") {
        current = normalizeOnnxVisionStatus({}, { enabled: true, error });
        render();
      }
    } finally {
      loading = false;
    }
  }

  const toggle = setting.querySelector('input[type="checkbox"]');
  const onToggle = () => {
    current = normalizeOnnxVisionStatus({}, {
      enabled: enabled(setting),
      loading: enabled(setting),
    });
    render();
    globalThis.setTimeout(refresh, 250);
  };
  toggle?.addEventListener("change", onToggle);
  lifecycle.addCleanup(() => toggle?.removeEventListener("change", onToggle));

  const onVisibilityChange = () => {
    if (!globalThis.document?.hidden) refresh();
  };
  globalThis.document?.addEventListener("visibilitychange", onVisibilityChange);
  lifecycle.addCleanup(() => globalThis.document?.removeEventListener("visibilitychange", onVisibilityChange));
  lifecycle.setIntervalWhileMounted(refresh, POLL_INTERVAL_MS);
  render();
  refresh();

  return {
    root: view.editor.section,
    sync() { render(); },
    destroy() {
      editor?.destroy();
      view.restoreRows();
      view.roots.forEach((element) => element.remove());
    },
  };
}
