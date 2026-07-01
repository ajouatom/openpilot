"use strict";

(function () {
  const YOUTUBE_GROUP = "SYS_YOUTUBE";
  const KEY_SECTION_ID = "youtubeLiveKeySection";
  const CARD_ID = "youtubeLiveStatusCard";
  const LIVE_CONTROL_ROOM_URL = "https://www.youtube.com/livestreaming";
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
    button.className = `smallBtn${tone ? ` btn--${tone}` : ""}`;
    button.textContent = label;
    button.addEventListener("click", action);
    return button;
  }

  function ensureCard() {
    const box = itemsBox();
    if (!box) return null;
    const existing = document.getElementById(CARD_ID);
    const existingKey = document.getElementById(KEY_SECTION_ID);
    if (existing && existingKey) {
      if (existing.nextSibling) box.appendChild(existing);
      return existing;
    }

    if (!existingKey) {
      const keySection = document.createElement("section");
      keySection.id = KEY_SECTION_ID;
      keySection.className = "setting-section-block youtube-live-section youtube-live-section--key";
    const keyCard = document.createElement("div");
    keyCard.className = "setting-group-card youtube-live-key-card";
    keyCard.innerHTML = `
      <div class="youtube-live-card__title">${escapeHtmlLocal(text("youtube_live_key_panel_title", "Stream key"))}</div>
      <div class="youtube-live-card__desc" data-role="key-status">${escapeHtmlLocal(text("loading", "Loading..."))}</div>
      <div class="youtube-live-key-row">
        <input class="youtube-live-key-input" data-role="key-input" type="password" autocomplete="off" spellcheck="false" placeholder="${escapeHtmlLocal(text("youtube_live_key_placeholder", "Paste stream key"))}" />
        <button class="smallBtn btn--filled youtube-live-action" type="button" data-action="save-key">${escapeHtmlLocal(text("save", "Save"))}</button>
        <button class="smallBtn youtube-live-action btn--danger" type="button" data-action="clear-key">${escapeHtmlLocal(text("youtube_live_clear_key", "Clear key"))}</button>
        <button class="smallBtn youtube-live-action" type="button" data-action="validate-key">${escapeHtmlLocal(text("youtube_live_validate_key", "Validate key"))}</button>
      </div>
    `;
      keySection.appendChild(keyCard);
      keyCard.querySelector('[data-action="save-key"]')?.addEventListener("click", saveKeyFromInput);
      keyCard.querySelector('[data-action="clear-key"]')?.addEventListener("click", clearKey);
      keyCard.querySelector('[data-action="validate-key"]')?.addEventListener("click", validateStreamKey);
      keyCard.querySelector('[data-role="key-input"]')?.addEventListener("keydown", (event) => {
        if (event.key === "Enter") {
          event.preventDefault();
          saveKeyFromInput();
        }
      });
      box.insertBefore(keySection, box.firstChild);
    }

    const section = existing || document.createElement("section");
    if (!existing) {
      section.id = CARD_ID;
      section.className = "setting-section-block youtube-live-section youtube-live-section--info";

      const helpCard = document.createElement("div");
      helpCard.className = "setting-group-card youtube-live-help-card";
      helpCard.innerHTML = `
        <div class="youtube-live-card__title">${escapeHtmlLocal(text("youtube_live_help", "Help"))}</div>
        <ol class="youtube-live-help-list">
          <li>${escapeHtmlLocal(text("youtube_live_help_step_1", "Open YouTube Studio and click Create > Go live."))}</li>
          <li>${escapeHtmlLocal(text("youtube_live_help_step_2", "In Live Control Room, open Stream and copy Stream key."))}</li>
          <li>${escapeHtmlLocal(text("youtube_live_help_step_3", "Paste the key here, save it, then run Validate key."))}</li>
        </ol>
        <div class="youtube-live-help-note">${escapeHtmlLocal(text("youtube_live_help_step_4", "Standard mode uses qRoadEncodeData, about 526x330 at 20fps. This fits YouTube's 240p-720p encoder range; stereo audio uses AAC 128Kbps."))}</div>
        <div class="youtube-live-help-note">${escapeHtmlLocal(text("youtube_live_help_note", "Validation checks local format, FFmpeg, and RTMPS reachability. YouTube confirms the key only after streaming starts."))}</div>
        <div class="ui-action-grid ui-action-grid--quick youtube-live-actions" data-role="action-menu"></div>
      `;
      section.appendChild(helpCard);

      const menuActions = helpCard.querySelector('[data-role="action-menu"]');
      if (menuActions) {
        menuActions.appendChild(makeButton(text("youtube_live_open_studio", "Open Studio"), openLiveControlRoom));
        menuActions.appendChild(makeButton(text("youtube_live_check", "Check"), runCheck));
      }

      const statusCard = document.createElement("div");
      statusCard.className = "setting-group-card youtube-live-card youtube-live-status-card";
      section.appendChild(statusCard);

      const head = document.createElement("div");
      head.className = "youtube-live-card__head";
      const summary = document.createElement("div");
      summary.className = "youtube-live-card__summary";
      const summaryTitle = document.createElement("div");
      summaryTitle.className = "youtube-live-card__title";
      summaryTitle.dataset.role = "title";
      summaryTitle.textContent = text("youtube_live_phase_status_title", "Phase 1 / status");
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
      statusCard.appendChild(head);

      const metrics = document.createElement("div");
      metrics.className = "youtube-live-metrics";
      metrics.dataset.role = "metrics";
      statusCard.appendChild(metrics);

      const error = document.createElement("div");
      error.className = "youtube-live-error";
      error.dataset.role = "error";
      error.hidden = true;
      statusCard.appendChild(error);

      const warnings = document.createElement("div");
      warnings.className = "youtube-live-warnings";
      warnings.dataset.role = "warnings";
      warnings.hidden = true;
      statusCard.appendChild(warnings);

    }

    box.appendChild(section);
    return section;
  }

  function escapeHtmlLocal(value) {
    return String(value ?? "").replace(/[&<>"']/g, (ch) => ({
      "&": "&amp;",
      "<": "&lt;",
      ">": "&gt;",
      '"': "&quot;",
      "'": "&#39;",
    }[ch]));
  }

  function renderStatus(status) {
    const card = ensureCard();
    if (!card) return;
    const stateEl = card.querySelector('[data-role="state"]');
    const keyStatusEl = document.querySelector(`#${KEY_SECTION_ID} [data-role="key-status"]`);
    const configuredEl = card.querySelector('[data-role="configured"]');
    const metricsEl = card.querySelector('[data-role="metrics"]');
    const errorEl = card.querySelector('[data-role="error"]');
    const warningsEl = card.querySelector('[data-role="warnings"]');
    card.dataset.liveState = String(status?.state || "disabled");
    if (stateEl) stateEl.textContent = stateLabel(status);
    if (keyStatusEl) keyStatusEl.textContent = configuredLabel(status);
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

  function keyInputValue() {
    return String(document.querySelector(`#${KEY_SECTION_ID} [data-role="key-input"]`)?.value || "").trim();
  }

  async function saveKeyFromInput() {
    const value = keyInputValue();
    if (!value) {
      toast(text("youtube_live_key_empty", "Enter a stream key first."), { tone: "error" });
      return;
    }
    try {
      const status = await postJson("/api/youtube_live/stream_key", { stream_key: value });
      state.status = status;
      renderStatus(status);
      const input = document.querySelector(`#${KEY_SECTION_ID} [data-role="key-input"]`);
      if (input) input.value = "";
      toast(text("youtube_live_key_saved", "Stream key saved"));
    } catch (err) {
      toast(err?.message || String(err), { tone: "error" });
    }
  }

  async function validateStreamKey() {
    const value = keyInputValue();
    try {
      const payload = value
        ? { stream_key: value }
        : {};
      const result = await postJson("/api/youtube_live/stream_key/validate", payload);
      await appAlert(validationText(result), {
        title: text("youtube_live_validate_key", "Validate key"),
      });
    } catch (err) {
      const payload = err?.payload || null;
      if (payload) {
        await appAlert(validationText(payload), {
          title: text("youtube_live_validate_key", "Validate key"),
        });
        return;
      }
      toast(err?.message || String(err), { tone: "error" });
    }
  }

  function validationText(result) {
    const pass = text("youtube_live_validation_pass", "OK");
    const fail = text("youtube_live_validation_fail", "FAIL");
    const lines = [
      `${text("youtube_live_validation_format", "Key format")}: ${result?.format_ok ? pass : fail} - ${localizedValidationMessage(result?.format_message)}`,
      `${text("youtube_live_validation_ffmpeg", "FFmpeg")}: ${result?.ffmpeg_available ? pass : fail}`,
      `${text("youtube_live_validation_rtmps", "YouTube RTMPS")}: ${result?.rtmps_reachable ? pass : fail} - ${localizedValidationMessage(result?.rtmps_message)}`,
    ];
    if (result?.masked_key) lines.push(`${text("youtube_live_key", "Stream key")}: ${result.masked_key}`);
    lines.push("");
    lines.push(result?.note || text("youtube_live_validation_note", "YouTube confirms the key only when streaming starts."));
    return lines.join("\n");
  }

  function localizedValidationMessage(message) {
    const raw = String(message || "").trim();
    if (!raw) return "-";
    const map = {
      "stream key is required": text("youtube_live_validation_required", "Stream key is required."),
      "stream key is too short": text("youtube_live_validation_too_short", "Stream key is too short."),
      "stream key is too long": text("youtube_live_validation_too_long", "Stream key is too long."),
      "stream key must not contain spaces": text("youtube_live_validation_no_spaces", "Stream key must not contain spaces."),
      "stream key contains unsupported characters": text("youtube_live_validation_bad_chars", "Stream key contains unsupported characters."),
      "format looks valid": text("youtube_live_validation_format_ok", "Format looks valid."),
      "YouTube RTMPS ingest is reachable": text("youtube_live_validation_rtmps_ok", "YouTube RTMPS ingest is reachable."),
      "missing stream key or ffmpeg": text("youtube_live_test_missing", "Missing stream key or FFmpeg."),
      "ready": text("youtube_live_check_ready", "Ready"),
    };
    return map[raw] || raw;
  }

  function openLiveControlRoom() {
    window.open(LIVE_CONTROL_ROOM_URL, "_blank", "noopener");
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

  async function fetchTestResult() {
    try {
      const result = await postJson("/api/youtube_live/test", {});
      return { ok: Boolean(result?.ok), payload: result };
    } catch (err) {
      const payload = err?.payload || null;
      return {
        ok: false,
        payload,
        error: payload?.message ? localizedValidationMessage(payload.message) : (err?.message || text("youtube_live_test_failed", "Test failed")),
      };
    }
  }

  async function fetchDiagnosticsResult() {
    try {
      const payload = await getJson("/api/youtube_live/diagnostics");
      return { ok: true, payload };
    } catch (err) {
      return { ok: false, error: err?.message || String(err) };
    }
  }

  function checkText(testResult, diagnosticsResult) {
    const pass = text("youtube_live_validation_pass", "OK");
    const fail = text("youtube_live_validation_fail", "FAIL");
    const test = testResult?.payload || {};
    const lines = [
      `${text("youtube_live_test", "Test")}: ${testResult?.ok ? pass : fail} - ${localizedValidationMessage(test.message || testResult?.error)}`,
      `${text("youtube_live_key", "Stream key")}: ${test.configured ? pass : fail}`,
      `${text("youtube_live_validation_ffmpeg", "FFmpeg")}: ${test.ffmpeg_available ? pass : fail}`,
      `${text("youtube_live_metric_source", "Source")}: ${test.source || "-"}`,
    ];
    if (Array.isArray(test.warnings) && test.warnings.length) {
      lines.push("");
      lines.push(text("youtube_live_warnings", "Warnings"));
      test.warnings.forEach((warning) => lines.push(`- ${warning}`));
    }
    lines.push("");
    if (diagnosticsResult?.ok) {
      lines.push(diagnosticText(diagnosticsResult.payload));
    } else {
      lines.push(`${text("youtube_live_diagnostics", "Diagnostics")}: ${fail} - ${diagnosticsResult?.error || "-"}`);
    }
    return lines.join("\n");
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

  async function runCheck() {
    const [testResult, diagnosticsResult] = await Promise.all([
      fetchTestResult(),
      fetchDiagnosticsResult(),
    ]);
    const status = diagnosticsResult?.payload?.diagnostics?.status;
    if (status) {
      state.status = status;
      renderStatus(status);
    }
    await appAlert(checkText(testResult, diagnosticsResult), {
      title: text("youtube_live_check_result", "Check result"),
      copyText: JSON.stringify({
        test: testResult.payload || { ok: false, error: testResult.error },
        diagnostics: diagnosticsResult.payload?.diagnostics || { ok: false, error: diagnosticsResult.error },
      }, null, 2),
    });
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
    const keySection = document.getElementById(KEY_SECTION_ID);
    if (keySection) keySection.remove();
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
