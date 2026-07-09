"use strict";

(function () {
  const YOUTUBE_SECTION = "SYS_YOUTUBE";
  const KEY_SECTION_ID = "youtubeLiveKeySection";
  const CARD_ID = "youtubeLiveStatusCard";
  const LIVE_CONTROL_ROOM_URL = "https://www.youtube.com/livestreaming";
  const POLL_MS = 1000;
  const state = {
    timer: null,
    loading: false,
    keyLoading: false,
    keyLoaded: false,
    streamKey: "",
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
    return Boolean(box?.querySelector(`[data-setting-section-id="${YOUTUBE_SECTION}"]`));
  }

  function stateLabel(status) {
    const key = String(status?.state || "disabled");
    return text(`youtube_live_state_${key}`, key);
  }

  function configuredLabel(status) {
    if (status?.configured) {
      if (!state.keyLoaded) return text("loading", "Loading...");
      return text("youtube_live_key_configured", "Key saved: {key}", { key: maskedKey(state.streamKey) || "-" });
    }
    return text("youtube_live_key_missing", "Stream key is not saved");
  }

  function maskedKey(value) {
    const key = String(value || "").trim();
    if (!key) return "";
    if (key.length <= 12) return `${key.slice(0, 3)}•••`;
    return `${key.slice(0, 8)}•••${key.slice(-4)}`;
  }

  function keyControlMarkup() {
    const configured = Boolean(state.status?.configured || state.streamKey);
    return renderSettingFormControl({
      label: text("youtube_live_key_panel_title", "Stream key"),
      value: maskedKey(state.streamKey) || text("youtube_live_key_missing", "Stream key is not saved"),
      placeholder: text("youtube_live_key_placeholder", "Paste stream key"),
      configured,
      editAction: "edit-key",
      actions: [
        { action: "clear-key", label: text("youtube_live_clear_key", "Clear key"), disabled: !configured },
      ],
    });
  }

  function bindKeyControl(keyCard) {
    keyCard?.querySelector('[data-action="edit-key"]')?.addEventListener("click", editStreamKey);
    keyCard?.querySelector('[data-action="clear-key"]')?.addEventListener("click", clearKey);
  }

  function syncKeyControl() {
    const keyCard = document.querySelector(`#${KEY_SECTION_ID} .youtube-live-key-card`);
    const host = keyCard?.querySelector('[data-role="key-control"]');
    if (!host) return;
    host.innerHTML = keyControlMarkup();
    bindKeyControl(keyCard);
  }

  async function editStreamKey() {
    await openSettingFormDialog({
      title: text("youtube_live_key_panel_title", "Stream key"),
      placeholder: text("youtube_live_key_placeholder", "Paste stream key"),
      inputType: "password",
      onSave: saveStreamKey,
    });
  }

  function streamTitle(status) {
    return stateLabel(status);
  }

  function streamSummary(status) {
    return videoFormatLabel(status);
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

  function placeYoutubeSections(box, keySection, infoSection) {
    const target = box.querySelector(`[data-setting-section-id="${YOUTUBE_SECTION}"]`);
    if (!target) return false;
    const settingsCard = Array.from(target.children).find((child) => child.classList.contains("setting-group-card"));
    if (keySection.parentElement !== target || keySection.nextSibling !== settingsCard) {
      target.insertBefore(keySection, settingsCard || null);
    }
    if (infoSection.parentElement !== target || target.lastElementChild !== infoSection) {
      target.appendChild(infoSection);
    }
    return true;
  }

  function ensureCard() {
    const box = itemsBox();
    if (!box) return null;
    if (!box.querySelector(`[data-setting-section-id="${YOUTUBE_SECTION}"]`)) return null;
    const existing = document.getElementById(CARD_ID);
    const existingKey = document.getElementById(KEY_SECTION_ID);
    if (existing && existingKey) {
      placeYoutubeSections(box, existingKey, existing);
      return existing;
    }

    let keySection = existingKey;
    if (!keySection) {
      keySection = document.createElement("section");
      keySection.id = KEY_SECTION_ID;
      keySection.className = "youtube-live-section youtube-live-section--key";
    const keyCard = document.createElement("div");
    keyCard.className = "setting-group-card youtube-live-key-card";
    keyCard.innerHTML = `
      <div class="youtube-live-card__title">${escapeHtmlLocal(text("youtube_live_key_panel_title", "Stream key"))}</div>
      <div class="youtube-live-card__desc" data-role="key-status">${escapeHtmlLocal(text("loading", "Loading..."))}</div>
      <div data-role="key-control">${keyControlMarkup()}</div>
    `;
      keySection.appendChild(keyCard);
      bindKeyControl(keyCard);
    }

    const section = existing || document.createElement("section");
    if (!existing) {
      section.id = CARD_ID;
      section.className = "youtube-live-section youtube-live-section--info";

      const helpCard = document.createElement("div");
      helpCard.className = "setting-group-card youtube-live-help-card";
      helpCard.innerHTML = `
        <div class="youtube-live-card__title">${escapeHtmlLocal(text("youtube_live_help", "Help"))}</div>
        <ol class="youtube-live-help-list">
          <li>${escapeHtmlLocal(text("youtube_live_help_step_1", "Open YouTube Studio and click Create > Go live."))}</li>
          <li>${escapeHtmlLocal(text("youtube_live_help_step_2", "In Live Control Room, open Stream and copy Stream key."))}</li>
          <li>${escapeHtmlLocal(text("youtube_live_help_step_3", "Paste the key here, save it, then run Validate key."))}</li>
        </ol>
        <div class="ui-action-grid ui-action-grid--quick youtube-live-actions" data-role="action-menu"></div>
      `;

      const menuActions = helpCard.querySelector('[data-role="action-menu"]');
      if (menuActions) {
        menuActions.appendChild(makeButton(text("youtube_live_open_studio", "Open Studio"), openLiveControlRoom));
        menuActions.appendChild(makeButton(text("youtube_live_check", "Check"), runCheck));
      }

      const statusCard = document.createElement("div");
      statusCard.className = "setting-group-card youtube-live-card youtube-live-status-card";
      section.appendChild(statusCard);

      const summary = document.createElement("div");
      summary.className = "youtube-live-card__summary";
      const summaryTitle = document.createElement("div");
      summaryTitle.className = "youtube-live-card__title";
      summaryTitle.dataset.role = "title";
      summaryTitle.textContent = stateLabel({ state: "disabled" });
      const summaryDesc = document.createElement("div");
      summaryDesc.className = "youtube-live-card__desc";
      summaryDesc.dataset.role = "summary";
      summaryDesc.textContent = text("loading", "Loading...");
      summary.appendChild(summaryTitle);
      summary.appendChild(summaryDesc);
      statusCard.appendChild(summary);

      const metrics = document.createElement("div");
      metrics.className = "youtube-live-metrics";
      metrics.dataset.role = "metrics";
      statusCard.appendChild(metrics);

      const log = document.createElement("pre");
      log.className = "muted text-mono";
      log.dataset.role = "runtime-log";
      log.hidden = true;
      statusCard.appendChild(log);

      section.appendChild(helpCard);

    }

    placeYoutubeSections(box, keySection, section);
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
    const titleEl = card.querySelector('[data-role="title"]');
    const keyStatusEl = document.querySelector(`#${KEY_SECTION_ID} [data-role="key-status"]`);
    const summaryEl = card.querySelector('[data-role="summary"]');
    const metricsEl = card.querySelector('[data-role="metrics"]');
    const logEl = card.querySelector('[data-role="runtime-log"]');
    if (titleEl) titleEl.textContent = streamTitle(status);
    if (keyStatusEl) keyStatusEl.textContent = configuredLabel(status);
    if (summaryEl) summaryEl.textContent = streamSummary(status);
    if (metricsEl) {
      metricsEl.innerHTML = "";
      metricsEl.appendChild(metric(text("youtube_live_metric_source", "Video"), sourceLabel(status)));
      metricsEl.appendChild(metric(text("youtube_live_metric_format", "Format"), videoFormatLabel(status)));
      metricsEl.appendChild(metric(text("youtube_live_metric_target", "Target"), targetFormatLabel(status)));
      metricsEl.appendChild(metric(
        text("youtube_live_metric_timestamp", "Time caption"),
        status?.timestamp_caption_enabled
          ? text("youtube_live_resource_enabled", "Enabled")
          : text("youtube_live_resource_disabled", "Disabled"),
      ));
      metricsEl.appendChild(metric(text("youtube_live_metric_runtime", "Runtime"), formatSeconds(status?.uptime_sec || 0)));
      metricsEl.appendChild(metric(text("youtube_live_metric_bitrate", "Bitrate"), `${status?.estimated_kbps || 0} kbps`));
      metricsEl.appendChild(metric(text("youtube_live_metric_session", "Session"), `${status?.session_mb || 0} MB`));
      metricsEl.appendChild(metric(text("youtube_live_metric_total", "Total"), `${status?.total_mb || 0} MB`));
      metricsEl.appendChild(metric(text("youtube_live_metric_retry", "Retry"), retryLabel(status)));
    }
    renderRuntimeLog(logEl, status);
  }

  function formatSeconds(seconds) {
    const total = Math.max(0, Math.floor(Number(seconds) || 0));
    const h = Math.floor(total / 3600);
    const m = Math.floor((total % 3600) / 60);
    const s = total % 60;
    return [h, m, s].map((v) => String(v).padStart(2, "0")).join(":");
  }

  function videoFormatLabel(status) {
    const width = Number(status?.frame_width || 526);
    const height = Number(status?.frame_height || 330);
    const fps = Number(status?.frame_fps || 20);
    return `${width}*${height} * ${fps}fps`;
  }

  function targetFormatLabel(status) {
    const width = Number(status?.target_width || status?.target?.width || 0);
    const height = Number(status?.target_height || status?.target?.height || 0);
    const kbps = Number(status?.target_video_kbps || status?.target?.video_kbps || 0);
    if (!width || !height) return "-";
    return `${width}*${height}${kbps ? ` / ${kbps}kbps` : ""}`;
  }

  function sourceLabel(status) {
    const requestedQuality = Number(status?.requested_quality);
    if (Number.isFinite(requestedQuality)) {
      if (requestedQuality <= 0) return text("youtube_live_source_low", "Low quality");
      if (requestedQuality === 1) return text("youtube_live_source_medium", "Medium quality");
      if (requestedQuality === 2) return text("youtube_live_source_high", "High quality");
      if (requestedQuality === 3) return text("youtube_live_source_wide", "Wide angle");
    }
    const quality = String(status?.quality || "").toLowerCase();
    if (quality === "low" || quality === "standard") return text("youtube_live_source_low", "Low quality");
    if (quality === "medium") return text("youtube_live_source_medium", "Medium quality");
    if (quality === "high") return text("youtube_live_source_high", "High quality");
    if (quality === "wide") return text("youtube_live_source_wide", "Wide angle");
    const source = String(status?.source || "");
    if (source === "livestreamRoadEncodeData") return text("youtube_live_source_medium", "Medium quality");
    if (source === "youtubeRoadEncodeData") return text("youtube_live_source_high", "High quality");
    if (source === "livestreamWideRoadEncodeData") return text("youtube_live_source_wide", "Wide angle");
    return text("youtube_live_source_low", "Low quality");
  }

  function retryLabel(status) {
    const retry = Number(status?.retry_in_sec || 0);
    if (String(status?.state || "") === "backoff" && retry > 0) {
      return text("youtube_live_retry_in", "{seconds}s", { seconds: retry });
    }
    return "-";
  }

  function renderRuntimeLog(container, status) {
    if (!container) return;
    const error = String(status?.last_error || "").trim();
    const warnings = uniqueLocalizedMessages(Array.isArray(status?.warnings) ? status.warnings : []);
    const lines = [];
    if (error) lines.push(`${text("youtube_live_error_label", "Error")}: ${localizedRuntimeMessage(error)}`);
    warnings.forEach((warning) => lines.push(`${text("youtube_live_warning_label", "Notice")}: ${warning}`));
    container.hidden = lines.length === 0;
    container.textContent = lines.join("\n");
  }

  async function loadStatus() {
    if (state.loading || !isVisible()) return;
    loadStreamKey();
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

  async function loadStreamKey() {
    if (state.keyLoading || state.keyLoaded || !isVisible()) return;
    state.keyLoading = true;
    try {
      const result = await getJson("/api/youtube_live/stream_key");
      state.streamKey = String(result?.stream_key || "");
      state.keyLoaded = true;
      syncKeyControl();
      if (state.status) renderStatus(state.status);
    } catch (err) {
      state.keyLoaded = false;
    } finally {
      state.keyLoading = false;
    }
  }

  async function saveStreamKey(rawValue) {
    const value = String(rawValue || "").trim();
    const validation = await fetchStreamKeyValidation(value, true);
    if (!validation?.format_ok) {
      const validationMessage = localizedValidationMessage(validation?.format_message);
      throw new Error(validationMessage !== "-"
        ? validationMessage
        : text("youtube_live_validation_required", "Stream key is required."));
    }
    try {
      const status = await postJson("/api/youtube_live/stream_key", { stream_key: value });
      state.streamKey = value;
      state.keyLoaded = true;
      state.status = status;
      renderStatus(status);
      syncKeyControl();
      toast(text("youtube_live_key_saved", "Stream key saved"));
    } catch (err) {
      throw new Error(err?.message || String(err));
    }
  }

  async function fetchStreamKeyValidation(value, explicitValue) {
    const payload = explicitValue || value ? { stream_key: value } : {};
    try {
      return await postJson("/api/youtube_live/stream_key/validate", payload);
    } catch (err) {
      if (err?.payload) return err.payload;
      return {
        ok: false,
        format_ok: false,
        format_message: err?.message || String(err),
        transport_available: false,
        muxer_available: false,
        rtmps_reachable: false,
        rtmps_message: "-",
      };
    }
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
      "missing stream key, librtmp, or PyAV FLV/AAC support": text("youtube_live_test_muxer_missing", "Stream key, librtmp, or PyAV FLV/AAC support is missing."),
      "stream key, RTMPS network, librtmp, or PyAV FLV/AAC is unavailable": text("youtube_live_test_muxer_missing", "Stream key, RTMPS network, librtmp, or PyAV FLV/AAC is unavailable."),
      "stream key, RTMPS network, librtmp, or FLV/AAC muxer is unavailable": text("youtube_live_test_muxer_missing", "Stream key, RTMPS network, librtmp, or FLV/AAC muxer is unavailable."),
      "ready": text("youtube_live_check_ready", "Ready"),
    };
    if (raw.startsWith("YouTube RTMPS ingest unreachable:")) {
      return text("youtube_live_validation_rtmps_failed", "Cannot reach YouTube RTMPS: {error}", {
        error: raw.slice(raw.indexOf(":") + 1).trim() || "-",
      });
    }
    return map[raw] || raw;
  }

  function localizedRuntimeMessage(message) {
    const raw = String(message || "").trim();
    if (!raw) return "-";
    const exact = {
      "Cluster HUD is enabled; monitor encoder load and thermal headroom while streaming.": text("youtube_live_warning_cluster", "Cluster HUD is enabled. Monitor encoder load and temperature while streaming."),
      "Cluster HUD is enabled; monitor overall load and temperature during simultaneous use.": text("youtube_live_warning_cluster", "Cluster HUD is enabled. Monitor overall load and temperature during simultaneous use."),
      "Carrot Vision is enabled; YouTube Live shares camera/encoder/network resources.": text("youtube_live_warning_vision", "Carrot Vision is enabled. Camera, encoder, and network resources are shared."),
      "Carrot Vision is enabled; simultaneous streaming increases network and memory bandwidth use.": text("youtube_live_warning_vision", "Carrot Vision is enabled. Simultaneous streaming increases network and memory bandwidth use."),
      "High quality mode is planned for Phase 3; Phase 2 keeps qRoadEncodeData only.": text("youtube_live_warning_quality", "The selected video mode is unavailable."),
      "The selected video mode is waiting for the shared livestream encoder to start onroad.": text("youtube_live_warning_source_wait", "The selected video mode is waiting for its encoder."),
      "High quality is waiting for the dedicated YouTube encoder to start onroad.": text("youtube_live_warning_high_wait", "High quality is waiting for its encoder."),
      "The selected road mode is waiting for the dedicated YouTube encoder to start onroad.": text("youtube_live_warning_high_wait", "Selected road mode is waiting for its encoder."),
      "The selected YouTube encoder is waiting to start onroad.": text("youtube_live_warning_high_wait", "Selected video mode is waiting for its encoder."),
      "librtmp is unavailable": text("youtube_live_error_transport_missing", "librtmp is unavailable."),
      "librtmp not found": text("youtube_live_error_transport_missing", "librtmp is unavailable."),
      "PyAV FLV/AAC muxer is unavailable": text("youtube_live_error_muxer_missing", "PyAV FLV/AAC muxing is unavailable."),
      "YouTube RTMPS connection closed": text("youtube_live_error_transport_closed", "The YouTube RTMPS connection was closed."),
      "no qRoadEncodeData frames": text("youtube_live_error_no_frames", "No camera frames were received."),
    };
    if (exact[raw]) return exact[raw];
    if (/^no .* frames$/i.test(raw)) return text("youtube_live_error_no_frames", "No camera frames were received.");
    if (/^YouTube ingest may be starved:/i.test(raw)) return raw;
    if (/^YouTube RTMPS publish failed:/i.test(raw)) {
      return text("youtube_live_error_publish_failed", "YouTube RTMPS publishing failed: {error}", { error: raw.split(":", 2)[1]?.trim() || "-" });
    }
    if (/^YouTube RTMPS start failed:/i.test(raw)) {
      return text("youtube_live_error_start_failed", "YouTube RTMPS connection failed: {error}", { error: raw.split(":", 2)[1]?.trim() || "-" });
    }
    return raw;
  }

  function uniqueLocalizedMessages(values) {
    return [...new Set((values || []).filter(Boolean).map(localizedRuntimeMessage))];
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
      state.streamKey = "";
      state.keyLoaded = true;
      state.status = status;
      renderStatus(status);
      syncKeyControl();
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
    const diag = diagnosticsResult?.payload?.diagnostics || diagnosticsResult?.payload || {};
    const status = diag.status || {};
    const resources = status.resource_status || test.resource_status || {};
    const cluster = resources.cluster || {};
    const vision = resources.carrot_vision || {};
    const enabled = text("youtube_live_resource_enabled", "Enabled");
    const disabled = text("youtube_live_resource_disabled", "Disabled");
    const lines = [
      `[${text("youtube_live_check_ready_section", "Readiness")}]`,
      `${text("youtube_live_key", "Stream key")}: ${test.configured ? pass : fail}`,
      `${text("youtube_live_muxer", "FLV/AAC muxer")}: ${test.muxer_available ? pass : fail}`,
      `${text("youtube_live_validation_transport", "librtmp")}: ${test.transport_available ? pass : fail}`,
      `${text("youtube_live_validation_rtmps", "YouTube RTMPS")}: ${test.rtmps_reachable ? pass : fail}`,
      `${text("youtube_live_metric_source", "Source")}: ${sourceLabel(test)}`,
      "",
      `[${text("youtube_live_check_stream_section", "Stream status")}]`,
      `${text("youtube_live_state_label", "State")}: ${stateLabel(status)}`,
      `${text("youtube_live_metric_format", "Format")}: ${videoFormatLabel(status)}`,
      `${text("youtube_live_metric_target", "Target")}: ${targetFormatLabel(status)}`,
      `${text("youtube_live_metric_bitrate", "Bitrate")}: ${status.estimated_kbps || 0} kbps`,
      `${text("youtube_live_metric_session", "Session")}: ${status.session_mb || 0} MB`,
      `${text("youtube_live_metric_total", "Total")}: ${status.total_mb || 0} MB`,
      "",
      `[${text("youtube_live_check_resource_section", "Resources")}]`,
      `${text("youtube_live_resource_cluster", "Cluster HUD")}: ${cluster.enabled ? enabled : disabled}`,
      `${text("youtube_live_resource_vision", "Carrot Vision")}: ${vision.enabled ? enabled : disabled}`,
    ];
    const warnings = uniqueLocalizedMessages([
      ...(Array.isArray(test.warnings) ? test.warnings : []),
      ...(Array.isArray(status.warnings) ? status.warnings : []),
    ]);
    if (warnings.length) {
      lines.push("");
      lines.push(`[${text("youtube_live_warnings", "Warnings")}]`);
      warnings.forEach((warning) => lines.push(`- ${warning}`));
    }
    const streamLogs = diag.transport?.log_tail || status.log_tail || [];
    const errors = uniqueLocalizedMessages([
      status.last_error,
      ...(Array.isArray(streamLogs) ? streamLogs.slice(-8) : []),
      test.transport?.error,
      test.muxer?.error,
      test.rtmps_reachable ? "" : test.rtmps_message,
      diagnosticsResult?.ok ? "" : diagnosticsResult?.error,
      testResult?.ok || test.message === "ready" ? "" : (testResult?.error || test.message),
    ]);
    if (errors.length) {
      lines.push("");
      lines.push(`[${text("youtube_live_check_error_section", "Errors and stream logs")}]`);
      errors.forEach((error) => lines.push(`- ${localizedValidationMessage(error)}`));
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
    loadStreamKey();
    loadStatus();
  }

  function stopPolling() {
    if (state.timer) window.clearInterval(state.timer);
    state.timer = null;
  }

  function lockTimestampToggle() {
    const row = document.querySelector('.setting[data-setting-name="CarrotYouTubeTimestamp"]');
    if (!row) return;
    const toggle = row.querySelector(".c-switch__input");
    if (toggle) {
      toggle.checked = false;
      toggle.disabled = true;
      toggle.setAttribute("aria-disabled", "true");
    }
    row.classList.add("youtube-timestamp-disabled");
    if (state.timestampDisabledSynced) return;
    state.timestampDisabledSynced = true;
    setParam("CarrotYouTubeTimestamp", 0).catch(() => {
      state.timestampDisabledSynced = false;
    });
  }

  function sync() {
    if (isVisible()) {
      lockTimestampToggle();
      ensureCard();
      startPolling();
      return;
    }
    stopPolling();
    state.keyLoaded = false;
    state.streamKey = "";
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
      if (name === "CarrotYouTubeLive" || name === "CarrotYouTubeQuality" || name === "CarrotYouTubeTimestamp") loadStatus();
    });
    sync();
  }

  window.CarrotYouTubeLiveSettings = Object.freeze({ sync });

  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", init, { once: true });
  else init();
})();
