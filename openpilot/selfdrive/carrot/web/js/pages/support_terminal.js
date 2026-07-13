"use strict";

(function () {
  const state = {
    active: false,
    busy: false,
    dialogOpen: false,
    ownerWs: null,
    reconnectTimer: 0,
    startPollTimer: 0,
    busyAction: "",
    pending: new Map(),
    snapshot: { active: false, state: "idle" },
    typingActive: false,
    typingText: "",
    typingTimer: 0,
    typingRenderRaf: 0,
    rawTypingBuffer: "",
    rawTypingTimer: 0,
    remainingTimer: 0,
    remainingDeadlineMs: 0,
    initialized: false,
    pageActive: false,
    issueDraft: "",
    settings: {
      ttlSeconds: 1800,
      // Default to confirm-each-command; the persisted web setting (seeded in
      // init) overrides this, and the server default matches.
      permissionMode: "approve_each",
      commandTimeoutSeconds: 30,
    },
  };

  const TTL_OPTIONS = [900, 1800, 3600];
  const COMMAND_TIMEOUT_OPTIONS = [15, 30, 60, 120];

  const els = {};
  let typingIndicator = null;

  function $(id) {
    return document.getElementById(id);
  }

  function bindElements() {
    if (state.initialized) return;
    state.initialized = true;
    els.open = $("btnSupportTerminalOpen");
    els.approvalHost = $("supportTerminalApprovalHost");
    els.typingHost = $("supportTerminalTypingHost");
    if (els.open) els.open.addEventListener("click", openSupportDialog);
    ensureApprovalHost();
  }

  function text(value, fallback = "") {
    return String(value == null || value === "" ? fallback : value);
  }

  function t(key, fallback, vars = null) {
    if (typeof getUIText === "function") return getUIText(key, fallback, vars);
    if (!vars) return fallback;
    return Object.entries(vars).reduce(
      (out, [name, value]) => out.replaceAll(`{${name}}`, String(value)),
      fallback,
    );
  }

  function toast(message, options) {
    if (state.dialogOpen && options?.tone !== "error") return;
    if (typeof showAppToast === "function") showAppToast(message, options || undefined);
  }

  function clearReconnectTimer() {
    if (!state.reconnectTimer) return;
    clearTimeout(state.reconnectTimer);
    state.reconnectTimer = 0;
  }

  function clearStartPollTimer() {
    if (!state.startPollTimer) return;
    clearInterval(state.startPollTimer);
    state.startPollTimer = 0;
  }

  function clearRemainingTimer() {
    if (!state.remainingTimer) return;
    clearInterval(state.remainingTimer);
    state.remainingTimer = 0;
  }

  function startRemainingTimer() {
    clearRemainingTimer();
    if (!state.active) return;
    state.remainingTimer = window.setInterval(() => {
      renderToolbarButton();
      if (state.dialogOpen) renderDialog();
    }, 1000);
  }

  function startStatusPolling() {
    clearStartPollTimer();
    state.startPollTimer = window.setInterval(() => {
      loadStatus({ quiet: true });
    }, 1000);
  }

  function closeOwnerSocket() {
    clearReconnectTimer();
    const ws = state.ownerWs;
    state.ownerWs = null;
    if (!ws) return;
    try {
      ws.onopen = null;
      ws.onmessage = null;
      ws.onclose = null;
      ws.onerror = null;
      if (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING) ws.close();
    } catch (err) {
      console.log("[SupportTerminal] ws close failed:", err);
    }
  }

  function ownerWsUrl() {
    const proto = location.protocol === "https:" ? "wss" : "ws";
    return `${proto}://${location.host}/ws/support_terminal/owner`;
  }

  function connectOwnerSocket(force = false) {
    if (!state.pageActive || !state.active) return;
    if (state.ownerWs && (state.ownerWs.readyState === WebSocket.OPEN || state.ownerWs.readyState === WebSocket.CONNECTING)) {
      if (!force) return;
      closeOwnerSocket();
    }

    let ws;
    try {
      ws = new WebSocket(ownerWsUrl());
    } catch (err) {
      scheduleReconnect();
      return;
    }
    state.ownerWs = ws;

    ws.onopen = () => {
      if (state.ownerWs !== ws) return;
      try {
        ws.send(JSON.stringify({ type: "refresh" }));
      } catch (err) {}
    };

    ws.onmessage = (event) => {
      if (state.ownerWs !== ws) return;
      let data;
      try {
        data = JSON.parse(event.data);
      } catch (err) {
        return;
      }
      handleEvent(data);
    };

    ws.onclose = () => {
      if (state.ownerWs !== ws) return;
      state.ownerWs = null;
      if (state.pageActive && state.active) scheduleReconnect();
    };

    ws.onerror = () => {
      if (state.ownerWs !== ws) return;
      scheduleReconnect();
    };
  }

  function scheduleReconnect() {
    clearReconnectTimer();
    if (!state.pageActive || !state.active) return;
    state.reconnectTimer = window.setTimeout(() => {
      state.reconnectTimer = 0;
      connectOwnerSocket(true);
    }, 1400);
  }

  async function loadStatus(options = {}) {
    bindElements();
    try {
      const snapshot = await getJson("/api/support_terminal/status");
      applySnapshot(snapshot);
    } catch (err) {
      if (!options.quiet) {
        state.snapshot = { active: false, state: err?.message || "status unavailable", error: err?.message || "" };
        renderToolbarButton();
        renderApprovalHost();
        renderDialog();
      }
    }
  }

  async function startSession() {
    if (state.busy) return;
    const issueInput = $("supportTerminalIssue");
    state.issueDraft = issueInput?.value || state.issueDraft || "";
    state.busy = true;
    state.busyAction = "start";
    applySnapshot({
      ok: true,
      active: true,
      state: "starting",
      ttl_seconds: state.settings.ttlSeconds,
      permission_mode: state.settings.permissionMode,
      command_timeout_seconds: state.settings.commandTimeoutSeconds,
      guest_count: 0,
      pending_commands: [],
    });
    startStatusPolling();
    try {
      const snapshot = await postJson("/api/support_terminal/start", {
        note: state.issueDraft,
        ttl_seconds: state.settings.ttlSeconds,
        permission_mode: state.settings.permissionMode,
        command_timeout_seconds: state.settings.commandTimeoutSeconds,
      });
      if (state.busyAction === "start") applySnapshot(snapshot);
      scheduleStatusRefresh(350);
      scheduleStatusRefresh(1200);
      if (state.busyAction === "start" && snapshot?.active) toast(t("support_terminal_started", "Remote support started"));
    } catch (err) {
      if (state.busyAction === "start") {
        state.snapshot = { ...(state.snapshot || {}), error: err?.message || "start failed" };
        renderDialog();
        toast(err?.message || t("support_terminal_start_failed", "Remote support start failed"), { tone: "error", duration: 4200 });
      } else {
        scheduleStatusRefresh(350);
      }
    } finally {
      if (state.busyAction === "start") {
        clearStartPollTimer();
        state.busy = false;
        state.busyAction = "";
        renderToolbarButton();
        renderDialog();
      }
    }
  }

  async function stopSession() {
    if (state.busy && state.busyAction !== "start") return;
    state.busy = true;
    state.busyAction = "stop";
    applySnapshot({ ok: true, active: false, state: "stopping" });
    state.issueDraft = "";
    try {
      const snapshot = await postJson("/api/support_terminal/stop", {});
      applySnapshot(snapshot);
      scheduleStatusRefresh(350);
      toast(t("support_terminal_stopped", "Remote support stopped"));
    } catch (err) {
      await loadStatus({ quiet: true });
      toast(err?.message || t("support_terminal_stop_failed", "Stop failed"), { tone: "error" });
    } finally {
      if (state.busyAction === "stop") {
        clearStartPollTimer();
        state.busy = false;
        state.busyAction = "";
        renderToolbarButton();
        renderDialog();
      }
    }
  }

  async function resolveCommand(id, action) {
    const command = state.pending.get(id);
    if (!command || command.busy) return;
    command.busy = true;
    renderDialog();
    try {
      await postJson(`/api/support_terminal/commands/${encodeURIComponent(id)}/${action}`, {});
      state.pending.delete(id);
      renderToolbarButton();
      renderApprovalHost();
      renderDialog();
    } catch (err) {
      command.busy = false;
      renderApprovalHost();
      renderDialog();
      toast(err?.message || t("support_terminal_command_action_failed", "Command action failed"), { tone: "error" });
    }
  }

  function handleEvent(data) {
    const type = data?.type;
    if (type === "session_status") {
      applySnapshot(data);
      return;
    }
    if (type === "guest_presence") {
      state.snapshot = { ...(state.snapshot || {}), guest_count: Number(data.count || 0) };
      renderToolbarButton();
      renderApprovalHost();
      renderDialog();
      return;
    }
    if (type === "guest_typing") {
      showTyping(Boolean(data.active), String(data.text || ""));
      return;
    }
    if (type === "command_request") {
      const id = String(data.id || "");
      if (!id) return;
      state.pending.set(id, {
        id,
        line: String(data.line || ""),
        created_at: Number(data.created_at || 0),
        busy: false,
      });
      renderToolbarButton();
      renderApprovalHost();
      renderDialog();
      return;
    }
    if (type === "command_expired" || type === "command_approved" || type === "command_rejected") {
      state.pending.delete(String(data.id || ""));
      renderToolbarButton();
      renderApprovalHost();
      renderDialog();
      return;
    }
    if (type === "command_auto_run") {
      renderToolbarButton();
      renderApprovalHost();
      renderDialog();
      return;
    }
    if (type === "session_closed") {
      applySnapshot({ ok: true, active: false, state: data.reason || "closed" });
      renderToolbarButton();
      renderApprovalHost();
      renderDialog();
      return;
    }
    if (type === "error") {
      state.snapshot = { ...(state.snapshot || {}), error: data.message || "support error" };
      renderToolbarButton();
      renderApprovalHost();
      renderDialog();
      toast(data.message || t("support_terminal_error", "Remote support error"), { tone: "error" });
    }
  }

  function scheduleStatusRefresh(delay = 350) {
    window.setTimeout(() => loadStatus({ quiet: true }), delay);
  }

  function applySnapshot(snapshot) {
    bindElements();
    const wasActive = state.active;
    state.snapshot = snapshot || { active: false, state: "idle" };
    state.active = Boolean(snapshot?.active);
    try {
      window.dispatchEvent(new CustomEvent("carrot:support-terminal-status", { detail: state.snapshot }));
    } catch (err) {}
    if (snapshot?.ttl_seconds != null) state.settings.ttlSeconds = Number(snapshot.ttl_seconds);
    if (snapshot?.permission_mode) state.settings.permissionMode = String(snapshot.permission_mode);
    if (snapshot?.command_timeout_seconds != null) state.settings.commandTimeoutSeconds = Number(snapshot.command_timeout_seconds);
    if (state.active) {
      state.remainingDeadlineMs = snapshot?.expires_in == null ? 0 : Date.now() + Math.max(0, Number(snapshot.expires_in || 0)) * 1000;
      startRemainingTimer();
    } else {
      state.remainingDeadlineMs = 0;
      clearRemainingTimer();
    }

    if (Array.isArray(snapshot?.pending_commands)) {
      state.pending.clear();
      snapshot.pending_commands.forEach((command) => {
        const id = String(command?.id || "");
        if (!id) return;
        state.pending.set(id, {
          id,
          line: String(command?.line || ""),
          created_at: Number(command?.created_at || 0),
          busy: false,
        });
      });
    } else if (!state.active) {
      state.pending.clear();
    }

    if (state.active) {
      connectOwnerSocket(!wasActive);
    } else {
      closeOwnerSocket();
      showTyping(false);
    }
    renderToolbarButton();
    renderApprovalHost();
    renderDialog();
  }

  function ensureApprovalHost() {
    if (els.approvalHost && document.body.contains(els.approvalHost)) return els.approvalHost;
    const host = $("supportTerminalApprovalHost");
    if (!host) return null;
    els.approvalHost = host;
    return host;
  }

  function ensureTypingHost() {
    const host = els.typingHost || $("supportTerminalTypingHost");
    const terminal = $("terminalXterm");
    if (!host || !terminal) return null;
    els.typingHost = host;
    // xterm is opened before support input can arrive. Mount inside it so this
    // visual-only presence indicator is positioned over the grid, not in the
    // terminal shell's flex layout.
    if (terminal.querySelector(".xterm") && host.parentElement !== terminal) terminal.append(host);
    return host;
  }

  function renderApprovalHost() {
    const host = ensureApprovalHost();
    if (!host) return;
    const commands = Array.from(state.pending.values());
    if (!commands.length) {
      host.hidden = true;
      host.innerHTML = "";
      return;
    }
    host.hidden = false;
    host.innerHTML = commands.map((command) => `
      <div class="terminal-approval" data-support-command-id="${escapeHtml(command.id)}">
        <div class="terminal-approval__meta">${escapeHtml(t("support_terminal_command_approval", "Command approval"))}</div>
        <code class="terminal-approval__line" title="${escapeHtml(command.line)}">${escapeHtml(command.line)}</code>
        <div class="terminal-approval__actions">
          <button class="smallBtn btn--filled" type="button" data-support-command-action="approve" data-support-command-id="${escapeHtml(command.id)}" ${command.busy ? "disabled" : ""}>${escapeHtml(t("support_terminal_run", "Run"))}</button>
          <button class="smallBtn" type="button" data-support-command-action="reject" data-support-command-id="${escapeHtml(command.id)}" ${command.busy ? "disabled" : ""}>${escapeHtml(t("support_terminal_reject", "Reject"))}</button>
        </div>
      </div>
    `).join("");
    bindCommandButtons(host);
  }

  function renderToolbarButton() {
    if (!els.open) return;
    const pendingCount = state.pending.size;
    els.open.classList.toggle("is-active", state.active);
    els.open.classList.toggle("has-pending", pendingCount > 0);
    els.open.disabled = state.busy && !state.dialogOpen;
    if (pendingCount > 0) {
      els.open.textContent = t("support_terminal_button_pending", "Support {count}", { count: pendingCount });
    } else if (state.active) {
      els.open.textContent = supportButtonActiveLabel();
    } else {
      els.open.textContent = t("support_terminal_button", "Support");
    }
    const label = pendingCount > 0
      ? t("support_terminal_pending_label", "{count} command approval pending", { count: pendingCount })
      : state.active
        ? t("support_terminal_active_label", "Remote support active")
        : t("support_terminal_title", "Remote Support");
    els.open.title = label;
    els.open.setAttribute("aria-label", label);
  }

  function supportButtonActiveLabel() {
    const remaining = activeRemainingSeconds();
    const time = remaining == null ? t("support_terminal_time_unlimited", "Unlimited") : formatRemaining(remaining);
    const guestCount = Number(state.snapshot?.guest_count || 0);
    if (state.settings.permissionMode === "allow_all" && guestCount > 0) {
      return t("support_terminal_button_allow_all", "Allow all {time}", { time });
    }
    if (guestCount > 0) {
      return t("support_terminal_button_connected", "Connected {time}", { time });
    }
    return t("support_terminal_button_waiting", "Waiting {time}", { time });
  }

  function supportStatusLabel() {
    const snapshot = state.snapshot || {};
    const active = Boolean(snapshot.active);
    const stateText = String(snapshot.state || "idle");
    const error = String(snapshot.error || "");
    const detail = String(snapshot.status_detail || "");
    const remaining = activeRemainingSeconds();
    const guestCount = Number(snapshot.guest_count || 0);
    const pendingCount = state.pending.size;
    if (error) return error;
    if (!active) return state.busy ? busyLabel() : localizeState(stateText);
    if (stateText === "starting") return detail ? localizeStatusDetail(detail) : t("support_terminal_starting", "Starting...");
    if ([
      "Secure tunnel ready",
      "Sending Carrot server notification",
    ].includes(detail)) {
      return localizeStatusDetail(detail);
    }
    const time = remaining == null ? t("support_terminal_time_unlimited", "Unlimited") : formatRemaining(remaining);
    if (pendingCount > 0) {
      return t("support_terminal_status_command_pending", "Command pending {time} - {count} approval", { time, count: pendingCount });
    }
    if (state.settings.permissionMode === "allow_all" && guestCount > 0) {
      return t("support_terminal_status_allow_all", "Allow all {time} - Guest {count}", { time, count: guestCount });
    }
    if (guestCount > 0) {
      return t("support_terminal_status_guest_connected", "Connected {time} - Guest {count}", { time, count: guestCount });
    }
    return t("support_terminal_status_waiting_guest", "Waiting {time} - Guest {count}", { time, count: guestCount });
  }

  function discordLabel(discord) {
    if (!discord || discord.configured === false) return t("support_terminal_discord_not_sent", "Notification not sent");
    if (discord.disabled) return t("support_terminal_discord_disabled", "Notification disabled");
    if (discord.ok) return t("support_terminal_discord_sent", "Notification sent");
    if (discord.error) return t("support_terminal_discord_error", "Notification error");
    return t("support_terminal_discord_pending", "Notification pending");
  }

  function localizeState(value) {
    const key = String(value || "").toLowerCase().replace(/[^a-z0-9]+/g, "_");
    const map = {
      idle: ["support_terminal_state_idle", "Idle"],
      starting: ["support_terminal_state_starting", "Starting"],
      sharing: ["support_terminal_state_sharing", "Sharing"],
      stopped: ["support_terminal_state_stopped", "Stopped"],
      expired: ["support_terminal_state_expired", "Expired"],
      closed: ["support_terminal_state_closed", "Closed"],
      stopping: ["support_terminal_stopping", "Stopping..."],
    };
    const entry = map[key];
    return entry ? t(entry[0], entry[1]) : value;
  }

  function localizeStatusDetail(value) {
    const map = {
      "Preparing terminal session": ["support_terminal_detail_preparing", "Preparing terminal session"],
      "Preparing shared terminal session": ["support_terminal_detail_preparing", "Preparing shared terminal session"],
      "Starting support page": ["support_terminal_detail_page", "Starting support page"],
      "Downloading cloudflared": ["support_terminal_detail_downloading", "Downloading cloudflared"],
      "Starting secure tunnel": ["support_terminal_detail_tunnel", "Starting secure tunnel"],
      "cloudflared unavailable": ["support_terminal_detail_unavailable", "cloudflared unavailable"],
      "Secure tunnel ready": ["support_terminal_detail_tunnel_ready", "Secure tunnel ready"],
      "Sending Carrot server notification": ["support_terminal_detail_discord", "Sending Carrot server notification"],
      "Ready": ["support_terminal_detail_ready", "Ready"],
      "Start failed": ["support_terminal_detail_failed", "Start failed"],
    };
    const entry = map[String(value || "")];
    return entry ? t(entry[0], entry[1]) : value;
  }

  function renderDialogHtml() {
    const snapshot = state.snapshot || {};
    const active = Boolean(snapshot.active);
    const status = supportStatusLabel();
    const issueDisabled = state.busy || active ? "disabled" : "";
    const startDisabled = state.busy || active ? "disabled" : "";
    const stopDisabled = (state.busy && state.busyAction !== "start") || !active ? "disabled" : "";
    const actionButton = active
      ? `<button id="btnSupportTerminalStop" class="smallBtn" type="button" ${stopDisabled}>${escapeHtml(t("support_terminal_stop", "Stop"))}</button>`
      : `<button id="btnSupportTerminalStart" class="smallBtn btn--filled" type="button" ${startDisabled}>${escapeHtml(state.busy ? busyLabel() : t("support_terminal_start", "Start"))}</button>`;

    return `
      <div class="support-terminal-dialog">
        <div class="support-terminal-actions">
          <div class="support-terminal-status">${escapeHtml(status)}</div>
          ${actionButton}
        </div>
        <label class="support-terminal-issue">
          <span>${escapeHtml(t("support_terminal_issue", "Issue"))}</span>
          <input id="supportTerminalIssue" type="text" autocomplete="off" value="${escapeHtml(state.issueDraft)}" placeholder="${escapeHtml(t("support_terminal_issue_placeholder", "Optional issue note"))}" ${issueDisabled}>
        </label>
        ${renderSupportSettingsHtml(active)}
      </div>
    `;
  }

  function busyLabel() {
    return state.snapshot?.state === "stopping"
      ? t("support_terminal_stopping", "Stopping...")
      : t("support_terminal_starting", "Starting...");
  }

  function renderSupportSettingsHtml(active) {
    const disabled = state.busy || active ? "disabled" : "";
    return `
      <div class="support-terminal-options">
        ${renderSettingRow("ttl", t("support_terminal_share_time", "Share time"), TTL_OPTIONS.map((seconds) => [seconds, ttlLabel(seconds)]), state.settings.ttlSeconds, disabled)}
        ${renderSettingRow("permission", t("support_terminal_permission", "Command permission"), [
          ["approve_each", t("support_terminal_permission_approve_each", "Approve each")],
          ["allow_all", t("support_terminal_permission_allow_all", "Allow all")],
        ], state.settings.permissionMode, disabled)}
        ${renderSettingRow("timeout", t("support_terminal_approval_timeout", "Approval timeout"), COMMAND_TIMEOUT_OPTIONS.map((seconds) => [seconds, timeoutLabel(seconds)]), state.settings.commandTimeoutSeconds, disabled)}
      </div>
    `;
  }

  function renderSettingRow(kind, label, options, selectedValue, disabled) {
    return `
      <label class="support-terminal-optionRow">
        <div class="support-terminal-optionRow__copy">
          <div class="support-terminal-optionRow__label">${escapeHtml(label)}</div>
        </div>
        <select
          class="support-terminal-select"
          data-support-setting-kind="${escapeHtml(kind)}"
          ${disabled}
        >
          ${options.map(([value, optionLabel]) => `<option value="${escapeHtml(value)}" ${String(value) === String(selectedValue) ? "selected" : ""}>${escapeHtml(optionLabel)}</option>`).join("")}
        </select>
      </label>
    `;
  }

  function ttlLabel(seconds) {
    const labels = {
      900: ["support_terminal_time_15m", "15m"],
      1800: ["support_terminal_time_30m", "30m"],
      3600: ["support_terminal_time_1h", "1h"],
    };
    const entry = labels[seconds] || labels[1800];
    return t(entry[0], entry[1]);
  }

  function timeoutLabel(seconds) {
    return t("support_terminal_seconds", "{seconds}s", { seconds });
  }

  function permissionModeLabel(mode) {
    return mode === "allow_all"
      ? t("support_terminal_permission_allow_all", "Allow all")
      : t("support_terminal_permission_approve_each", "Approve each");
  }

  function renderDialog() {
    if (!state.dialogOpen || typeof appDialogBody === "undefined" || !appDialogBody) return;
    appDialogBody.innerHTML = renderDialogHtml();
    bindDialogEvents();
  }

  function bindDialogEvents() {
    const start = $("btnSupportTerminalStart");
    const stop = $("btnSupportTerminalStop");
    const issue = $("supportTerminalIssue");
    if (start) start.addEventListener("click", startSession);
    if (stop) stop.addEventListener("click", stopSession);
    if (issue) issue.addEventListener("input", () => { state.issueDraft = issue.value || ""; });
    if (issue) issue.addEventListener("keydown", (event) => {
      if (event.key === "Enter") event.preventDefault();
    });
    document.querySelectorAll("[data-support-setting-kind]").forEach((select) => {
      select.addEventListener("change", () => {
        updateSetting(select.dataset.supportSettingKind || "", select.value || "");
      });
    });
  }

  // Persist a support setting to the server-side web settings store so the
  // owner's last choice survives reloads. Fire-and-forget; a failed write just
  // keeps the in-memory value for this session.
  function persistSetting(key, value) {
    try {
      const result = window.setWebSettingByKey?.(key, value);
      if (result && typeof result.catch === "function") result.catch(() => {});
    } catch (err) {}
  }

  // Read persisted support settings (seeded from bootstrap by the web settings
  // state module) into local state. Called on init so the dialog opens with the
  // owner's last choices instead of the hardcoded defaults.
  function seedPersistedSettings() {
    if (typeof window.getWebSettingByKey !== "function") return;
    const permission = String(window.getWebSettingByKey("support_permission_mode", "approve_each") || "approve_each");
    state.settings.permissionMode = permission === "allow_all" ? "allow_all" : "approve_each";
    const ttl = Number(window.getWebSettingByKey("support_ttl_seconds", 1800));
    if (TTL_OPTIONS.includes(ttl)) state.settings.ttlSeconds = ttl;
    const timeout = Number(window.getWebSettingByKey("support_command_timeout_seconds", 30));
    if (COMMAND_TIMEOUT_OPTIONS.includes(timeout)) state.settings.commandTimeoutSeconds = timeout;
  }

  function updateSetting(kind, rawValue) {
    if (state.busy || state.active) return;
    if (kind === "ttl") {
      state.settings.ttlSeconds = Number(rawValue);
      persistSetting("support_ttl_seconds", String(state.settings.ttlSeconds));
    } else if (kind === "permission") {
      state.settings.permissionMode = rawValue === "allow_all" ? "allow_all" : "approve_each";
      persistSetting("support_permission_mode", state.settings.permissionMode);
    } else if (kind === "timeout") {
      state.settings.commandTimeoutSeconds = Number(rawValue);
      persistSetting("support_command_timeout_seconds", String(state.settings.commandTimeoutSeconds));
    }
    renderDialog();
  }

  function bindCommandButtons(root = document) {
    root.querySelectorAll("[data-support-command-action]").forEach((button) => {
      button.addEventListener("click", () => {
        const id = button.dataset.supportCommandId || "";
        const action = button.dataset.supportCommandAction || "";
        if (id && (action === "approve" || action === "reject")) resolveCommand(id, action);
      });
    });
  }

  async function openSupportDialog() {
    bindElements();
    if (state.dialogOpen) return;
    await loadStatus({ quiet: true });
    state.dialogOpen = true;
    document.body.classList.add("support-terminal-dialog-open");
    const promise = appAlert("", {
      title: t("support_terminal_title", "Remote Support"),
      html: true,
      messageHtml: renderDialogHtml(),
      confirmLabel: t("support_terminal_close", "Close"),
    });
    if (typeof appDialog !== "undefined" && appDialog) appDialog.classList.add("app-dialog--support-terminal");
    window.setTimeout(bindDialogEvents, 0);
    promise.finally(() => {
      state.dialogOpen = false;
      document.body.classList.remove("support-terminal-dialog-open");
      if (typeof appDialog !== "undefined" && appDialog) appDialog.classList.remove("app-dialog--support-terminal");
    });
    return promise;
  }

  function showTyping(active, value = "") {
    state.typingActive = active;
    state.typingText = active ? value.slice(0, 160) : "";
    clearTimeout(state.typingTimer);
    if (active) {
      state.typingTimer = window.setTimeout(() => {
        state.typingActive = false;
        state.typingText = "";
        scheduleTypingRender();
      }, 2200);
    }
    scheduleTypingRender();
  }

  function scheduleTypingRender() {
    if (state.typingRenderRaf) return;
    state.typingRenderRaf = requestAnimationFrame(() => {
      state.typingRenderRaf = 0;
      renderTypingOverlay();
    });
  }

  function applyPrintableTypingInput(buffer, data) {
    let next = String(buffer || "");
    let changed = false;
    let submitted = false;
    const input = String(data || "");

    for (let index = 0; index < input.length; index += 1) {
      const char = input[index];
      if (char === "\x1b") {
        const kind = input[index + 1];
        if (kind === "[") {
          index += 2;
          while (index < input.length) {
            const code = input.charCodeAt(index);
            if (code >= 0x40 && code <= 0x7e) break;
            index += 1;
          }
        } else if (kind === "]") {
          index += 2;
          while (index < input.length) {
            if (input[index] === "\x07") break;
            if (input[index] === "\x1b" && input[index + 1] === "\\") {
              index += 1;
              break;
            }
            index += 1;
          }
        } else {
          index += 1;
        }
        continue;
      }
      if (char === "\r" || char === "\n") {
        if (next) changed = true;
        next = "";
        submitted = true;
        continue;
      }
      if (char === "\x7f" || char === "\b") {
        const trimmed = next.slice(0, -1);
        changed ||= trimmed !== next;
        next = trimmed;
        continue;
      }
      if (char >= " " && char !== "\x7f") {
        next = (next + char).slice(-160);
        changed = true;
      }
    }
    return { text: next, changed, submitted };
  }

  function reportHostRawTyping(data) {
    if (!state.active || state.snapshot?.permission_mode !== "allow_all") return;
    const result = applyPrintableTypingInput(state.rawTypingBuffer, data);
    state.rawTypingBuffer = result.text;
    if (state.rawTypingTimer) clearTimeout(state.rawTypingTimer);
    if (result.submitted) {
      state.rawTypingTimer = 0;
      if (state.ownerWs?.readyState === WebSocket.OPEN) state.ownerWs.send(JSON.stringify({ type: "typing", active: false, text: "" }));
      return;
    }
    if (!result.changed && !state.rawTypingBuffer) return;
    if (state.ownerWs?.readyState !== WebSocket.OPEN) return;
    state.ownerWs.send(JSON.stringify({ type: "typing", active: true, text: state.rawTypingBuffer }));
    state.rawTypingTimer = window.setTimeout(() => {
      state.rawTypingTimer = 0;
      if (state.ownerWs?.readyState === WebSocket.OPEN) state.ownerWs.send(JSON.stringify({ type: "typing", active: false, text: "" }));
    }, 1400);
  }

  function renderTypingOverlay() {
    const host = ensureTypingHost();
    if (!host) return;
    const active = state.typingActive;
    document.documentElement.style.setProperty("--terminal-collab-toast-offset", active ? "46px" : "0px");
    if (!typingIndicator && window.CarrotTerminalTypingIndicator) {
      typingIndicator = window.CarrotTerminalTypingIndicator.create(host);
    }
    typingIndicator?.update({
      active,
      actor: t("support_terminal_guest_actor", "Guest"),
      text: state.typingText,
      emptyLabel: t("support_terminal_typing", "Typing"),
    });
  }

  function activeRemainingSeconds() {
    if (!state.active) return 0;
    if (!state.remainingDeadlineMs) return null;
    return Math.max(0, Math.ceil((state.remainingDeadlineMs - Date.now()) / 1000));
  }

  function formatRemaining(seconds) {
    if (seconds == null) return t("support_terminal_time_unlimited", "Unlimited");
    const total = Math.max(0, Math.floor(Number(seconds || 0)));
    const min = Math.floor(total / 60);
    const sec = total % 60;
    if (min <= 0) return `${sec}s`;
    return `${min}:${String(sec).padStart(2, "0")}`;
  }

  function init() {
    state.pageActive = true;
    bindElements();
    seedPersistedSettings();
    loadStatus({ quiet: true });
  }

  function teardown() {
    state.pageActive = false;
    state.dialogOpen = false;
    document.body.classList.remove("support-terminal-dialog-open");
    document.documentElement.style.removeProperty("--terminal-collab-toast-offset");
    showTyping(false);
    clearStartPollTimer();
    clearRemainingTimer();
    closeOwnerSocket();
  }

  window.CarrotSupportTerminal = {
    init,
    teardown,
    refresh: loadStatus,
    open: openSupportDialog,
    reportHostRawTyping,
  };
})();
