"use strict";

(function () {
  const YOUTUBE_GROUP = "SYS_YOUTUBE";
  const CARD_ID = "youtubeLiveStatusCard";
  const POLL_MS = 2500;
  const state = {
    timer: null,
    loading: false,
    status: null,
    observer: null,
  };

  function text(key, fallback, vars) {
    if (typeof getUIText === "function") return getUIText(key, fallback, vars);
    let value = fallback || key;
    if (vars) {
      Object.keys(vars).forEach((name) => {
        value = value.replace(new RegExp(`\\{${name}\\}`, "g"), String(vars[name]));
      });
    }
    return value;
  }

  function toast(message, options) {
    if (typeof showAppToast === "function") showAppToast(message, options || undefined);
  }

  function itemsBox() {
    return document.getElementById("items");
  }

  function isVisible() {
    const box = itemsBox();
    return Boolean(box && box.dataset.renderedGroup === YOUTUBE_GROUP && !box.dataset.renderedDetail);
  }

  function stateLabel(status) {
    const key = String(status?.state || "disabled");
    return text(`youtube_live_state_${key}`, key);
  }

  function configuredLabel(status) {
    if (status?.configured) {
      return text("youtube_live_key_configured", "Key saved: {key}", { key: status.masked_key || "****" });
    }
    return text("youtube_live_key_missing", "Stream key is not saved");
  }

  function metric(label, value) {
    const item = document.createElement("div");
    item.className = "youtube-live-metric";
    const key = document.createElement("div");
    key.className = "youtube-live-metric__label";
    key.textContent = label;
    const val = document.createElement("div");
    val.className = "youtube-live-metric__value";
    val.textContent = value;
    item.appendChild(key);
    item.appendChild(val);
    return item;
  }

  function makeButton(label, action, tone) {
    const button = document.createElement("button");
    button.type = "button";
    button.className = `smallBtn youtube-live-action${tone ? ` youtube-live-action--${tone}` : ""}`;
    button.textContent = label;
    button.addEventListener("click", action);
    return button;
  }

  function ensureCard() {
    const box = itemsBox();
    if (!box) return null;
    const existing = document.getElementById(CARD_ID);
    if (existing) return existing;

    const section = document.createElement("section");
    section.id = CARD_ID;
    section.className = "setting-section-block youtube-live-section";

    const title = document.createElement("div");
    title.className = "setting-group-card__title youtube-live-section__title";
    title.textContent = text("youtube_live_title", "YouTube Live");
    section.appendChild(title);

    const card = document.createElement("div");
    card.className = "setting-group-card youtube-live-card";
    section.appendChild(card);

    const head = document.createElement("div");
    head.className = "youtube-live-card__head";
    const summary = document.createElement("div");
    summary.className = "youtube-live-card__summary";
    const summaryTitle = document.createElement("div");
    summaryTitle.className = "youtube-live-card__title";
    summaryTitle.dataset.role = "title";
    summaryTitle.textContent = text("youtube_live_phase1_source", "Phase 1 - qRoadEncodeData");
    const summaryDesc = document.createElement("div");
    summaryDesc.className = "youtube-live-card__desc";
    summaryDesc.dataset.role = "configured";
    summaryDesc.textContent = text("loading", "Loading...");
    summary.appendChild(summaryTitle);
    summary.appendChild(summaryDesc);
    const pill = document.createElement("span");
    pill.className = "youtube-live-pill";
    pill.dataset.role = "state";
    pill.textContent = "-";
    head.appendChild(summary);
    head.appendChild(pill);
    card.appendChild(head);

    const metrics = document.createElement("div");
    metrics.className = "youtube-live-metrics";
    metrics.dataset.role = "metrics";
    card.appendChild(metrics);

    const error = document.createElement("div");
    error.className = "youtube-live-error";
    error.dataset.role = "error";
    error.hidden = true;
    card.appendChild(error);

    const warnings = document.createElement("div");
    warnings.className = "youtube-live-warnings";
    warnings.dataset.role = "warnings";
    warnings.hidden = true;
    card.appendChild(warnings);

    const actions = document.createElement("div");
    actions.className = "youtube-live-actions";
    actions.appendChild(makeButton(text("youtube_live_set_key", "Set key"), promptSetKey));
    actions.appendChild(makeButton(text("youtube_live_test", "Test"), testConfig));
    actions.appendChild(makeButton(text("youtube_live_diagnostics", "Diagnostics"), showDiagnostics));
    actions.appendChild(makeButton(text("youtube_live_download_logs", "Download logs"), downloadDiagnostics));
    actions.appendChild(makeButton(text("youtube_live_stop", "Stop"), stopStream));
    actions.appendChild(makeButton(text("youtube_live_clear_key", "Clear key"), clearKey, "danger"));
    card.appendChild(actions);

    box.appendChild(section);
    return section;
  }

  function renderStatus(status) {
    const card = ensureCard();
    if (!card) return;
    const stateEl = card.querySelector('[data-role="state"]');
    const configuredEl = card.querySelector('[data-role="configured"]');
    const metricsEl = card.querySelector('[data-role="metrics"]');
    const errorEl = card.querySelector('[data-role="error"]');
    const warningsEl = card.querySelector('[data-role="warnings"]');
    card.dataset.liveState = String(status?.state || "disabled");
    if (stateEl) stateEl.textContent = stateLabel(status);
    if (configuredEl) configuredEl.textContent = configuredLabel(status);
    if (metricsEl) {
      metricsEl.innerHTML = "";
      metricsEl.appendChild(metric(text("youtube_live_metric_source", "Source"), status?.source || "qRoadEncodeData"));
      metricsEl.appendChild(metric(text("youtube_live_metric_runtime", "Runtime"), formatSeconds(status?.uptime_sec || 0)));
      metricsEl.appendChild(metric(text("youtube_live_metric_bitrate", "Bitrate"), `${status?.estimated_kbps || 0} kbps`));
      metricsEl.appendChild(metric(text("youtube_live_metric_session", "Session"), `${status?.session_mb || 0} MB`));
      metricsEl.appendChild(metric(text("youtube_live_metric_total", "Total"), `${status?.total_mb || 0} MB`));
      metricsEl.appendChild(metric(text("youtube_live_metric_retry", "Retry"), retryLabel(status)));
    }
    const error = String(status?.last_error || "").trim();
    if (errorEl) {
      errorEl.hidden = !error;
      errorEl.textContent = error;
    }
    renderWarnings(warningsEl, status);
  }

  function formatSeconds(seconds) {
    const total = Math.max(0, Math.floor(Number(seconds) || 0));
    const h = Math.floor(total / 3600);
    const m = Math.floor((total % 3600) / 60);
    const s = total % 60;
    return [h, m, s].map((v) => String(v).padStart(2, "0")).join(":");
  }

  function retryLabel(status) {
    const retry = Number(status?.retry_in_sec || 0);
    if (String(status?.state || "") === "backoff" && retry > 0) {
      return text("youtube_live_retry_in", "{seconds}s", { seconds: retry });
    }
    return "-";
  }

  function renderWarnings(container, status) {
    if (!container) return;
    const warnings = Array.isArray(status?.warnings) ? status.warnings.filter(Boolean) : [];
    if (!warnings.length) {
      container.hidden = true;
      container.innerHTML = "";
      return;
    }
    container.hidden = false;
    container.innerHTML = "";
    warnings.slice(0, 3).forEach((warning) => {
      const item = document.createElement("div");
      item.className = "youtube-live-warning";
      item.textContent = warning;
      container.appendChild(item);
    });
  }

  async function loadStatus() {
    if (state.loading || !isVisible()) return;
    state.loading = true;
    try {
      const status = await getJson("/api/youtube_live/status");
      state.status = status;
      renderStatus(status);
    } catch (err) {
      renderStatus({ state: "error", last_error: err?.message || String(err) });
    } finally {
      state.loading = false;
    }
  }

  async function promptSetKey() {
    const current = state.status?.configured ? state.status?.masked_key : "";
    const prompt = text("youtube_live_key_prompt", "Enter YouTube stream key.");
    const value = await appPrompt(prompt, {
      title: text("youtube_live_set_key", "Set key"),
      defaultValue: "",
      placeholder: current || "xxxx-xxxx-xxxx-xxxx",
      confirmLabel: text("save", "Save"),
    });
    if (value === null || value === false || String(value).trim() === "") return;
    try {
      const status = await postJson("/api/youtube_live/stream_key", { stream_key: String(value).trim() });
      state.status = status;
      renderStatus(status);
      toast(text("youtube_live_key_saved", "Stream key saved"));
    } catch (err) {
      toast(err?.message || String(err), { tone: "error" });
    }
  }

  async function clearKey() {
    const ok = await appConfirm(text("youtube_live_clear_confirm", "Clear saved stream key?"), {
      title: text("youtube_live_clear_key", "Clear key"),
    });
    if (!ok) return;
    try {
      const status = await requestJson("/api/youtube_live/stream_key", { method: "DELETE" });
      state.status = status;
      renderStatus(status);
      toast(text("youtube_live_key_cleared", "Stream key cleared"));
    } catch (err) {
      toast(err?.message || String(err), { tone: "error" });
    }
  }

  async function stopStream() {
    try {
      const status = await postJson("/api/youtube_live/stop", {});
      state.status = status;
      renderStatus(status);
      toast(text("youtube_live_stopped", "Stream stopped"));
    } catch (err) {
      toast(err?.message || String(err), { tone: "error" });
    }
  }

  async function testConfig() {
    try {
      const result = await postJson("/api/youtube_live/test", {});
      toast(result.message || text("youtube_live_test_ready", "Ready"));
    } catch (err) {
      toast(err?.message || text("youtube_live_test_failed", "Test failed"), { tone: "error" });
    }
  }

  function diagnosticText(payload) {
    const diag = payload?.diagnostics || payload || {};
    const status = diag.status || {};
    const resources = status.resource_status || {};
    const cluster = resources.cluster || {};
    const vision = resources.carrot_vision || {};
    const lines = [
      `${text("youtube_live_state_label", "State")}: ${stateLabel(status)}`,
      `${text("youtube_live_metric_source", "Source")}: ${status.source || "-"}`,
      `${text("youtube_live_metric_bitrate", "Bitrate")}: ${status.estimated_kbps || 0} kbps`,
      `${text("youtube_live_metric_session", "Session")}: ${status.session_mb || 0} MB`,
      `${text("youtube_live_metric_total", "Total")}: ${status.total_mb || 0} MB`,
      `ClusterHud: ${cluster.enabled ? "on" : "off"} (${cluster.param ?? "-"})`,
      `CarrotVision: ${vision.enabled ? "on" : "off"} (DisableDM=${vision.disable_dm ?? "-"})`,
    ];
    const warnings = Array.isArray(status.warnings) ? status.warnings : [];
    if (warnings.length) {
      lines.push("");
      lines.push(text("youtube_live_warnings", "Warnings"));
      warnings.forEach((warning) => lines.push(`- ${warning}`));
    }
    const stderr = diag.ffmpeg?.stderr_tail || status.stderr_tail || [];
    if (stderr.length) {
      lines.push("");
      lines.push("ffmpeg stderr");
      stderr.slice(-8).forEach((line) => lines.push(`- ${line}`));
    }
    return lines.join("\n");
  }

  async function showDiagnostics() {
    try {
      const payload = await getJson("/api/youtube_live/diagnostics");
      const raw = JSON.stringify(payload.diagnostics || payload, null, 2);
      await appAlert(diagnosticText(payload), {
        title: text("youtube_live_diagnostics", "Diagnostics"),
        copyText: raw,
      });
    } catch (err) {
      toast(err?.message || String(err), { tone: "error" });
    }
  }

  function downloadDiagnostics() {
    window.open("/api/youtube_live/diagnostics/download", "_blank", "noopener");
  }

  function startPolling() {
    if (!state.timer) state.timer = window.setInterval(loadStatus, POLL_MS);
    loadStatus();
  }

  function stopPolling() {
    if (state.timer) window.clearInterval(state.timer);
    state.timer = null;
  }

  function sync() {
    if (isVisible()) {
      ensureCard();
      startPolling();
      return;
    }
    stopPolling();
    const card = document.getElementById(CARD_ID);
    if (card) card.remove();
  }

  function init() {
    const box = itemsBox();
    if (!box || state.observer) return;
    state.observer = new MutationObserver(sync);
    state.observer.observe(box, {
      childList: true,
      attributes: true,
      attributeFilter: ["data-rendered-group", "data-rendered-detail"],
    });
    window.addEventListener("carrot:paramchange", (event) => {
      const name = event?.detail?.name;
      if (name === "CarrotYouTubeLive" || name === "CarrotYouTubeQuality") loadStatus();
    });
    sync();
  }

  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", init, { once: true });
  else init();
})();
