"use strict";

// Tools notifications are split into a shared entry model and two light
// renderers. Each tool run or notice is one card in both orientations.
(function initToolsNotifications(global) {
  const MODE = Object.freeze({
    LANDSCAPE: "landscape",
    PORTRAIT: "portrait",
  });
  const HISTORY_LIMIT = 20;
  const SWIPE_THRESHOLD_PX = 24;
  const DRAG_OPEN_THRESHOLD_PX = 72;
  const DRAG_CLOSE_THRESHOLD_PX = 44;
  const UPDATE_ACK_STORAGE_KEY = "carrot_tools_update_acknowledged_v1";
  const UPDATE_ACK_LIMIT = 80;

  let activeNotificationId = "";
  let lastHost = null;
  let lastState = {};
  let lastOptions = {};
  let modeSyncBound = false;
  let pendingEntryFocus = null;
  let entryFocusToken = 0;
  let entryFocusTimer = null;
  let relativeTimeTimer = null;
  let collapseRenderTimer = null;
  let collapsingNotificationId = "";
  let collapsingUntil = 0;
  let collapseHost = null;
  let lastPublishedUnreadCount = 0;
  const detailScrollState = new Map();
  let lastRenderSignature = "";
  let lastAutoFocusedEntryId = "";
  const acknowledgedUpdateIds = loadAcknowledgedUpdateIds();

  function uiText(key, fallback, vars = null) {
    return typeof getUIText === "function" ? getUIText(key, fallback, vars) : fallback;
  }

  function normalizeText(value) {
    return String(value ?? "").replace(/\s+$/, "");
  }

  function safeNumber(value, fallback = 0) {
    const num = Number(value);
    return Number.isFinite(num) ? num : fallback;
  }

  function hashText(value) {
    let hash = 0;
    const text = String(value || "");
    for (let i = 0; i < text.length; i += 1) {
      hash = ((hash << 5) - hash + text.charCodeAt(i)) | 0;
    }
    return Math.abs(hash).toString(36);
  }

  function loadAcknowledgedUpdateIds() {
    try {
      const raw = global.localStorage?.getItem?.(UPDATE_ACK_STORAGE_KEY);
      const parsed = JSON.parse(raw || "[]");
      return new Set(Array.isArray(parsed) ? parsed.filter(Boolean).slice(-UPDATE_ACK_LIMIT) : []);
    } catch {
      return new Set();
    }
  }

  function persistAcknowledgedUpdateIds() {
    try {
      global.localStorage?.setItem?.(
        UPDATE_ACK_STORAGE_KEY,
        JSON.stringify(Array.from(acknowledgedUpdateIds).slice(-UPDATE_ACK_LIMIT))
      );
    } catch {}
  }

  function acknowledgeUpdateEntry(entry, card = null) {
    if (!entry?.ackId || !entry.highlight) return;
    acknowledgedUpdateIds.add(entry.ackId);
    while (acknowledgedUpdateIds.size > UPDATE_ACK_LIMIT) {
      acknowledgedUpdateIds.delete(acknowledgedUpdateIds.values().next().value);
    }
    persistAcknowledgedUpdateIds();
    if (card) {
      card.classList.remove("is-unread-update");
      card.classList.add("is-update-read");
    }
    global.renderToolsMeta?.();
  }

  function getMode() {
    return global.matchMedia?.("(orientation: portrait)")?.matches ? MODE.PORTRAIT : MODE.LANDSCAPE;
  }

  function syncHostMode(out, mode = getMode()) {
    const dock = out?.closest?.(".tools-console-dock");
    const shell = out?.closest?.(".tools-console-shell");
    [dock, shell, out].forEach((node) => {
      if (!node) return;
      node.classList.toggle("tools-notifications--portrait", mode === MODE.PORTRAIT);
      node.classList.toggle("tools-notifications--landscape", mode === MODE.LANDSCAPE);
      node.dataset.toolsNotificationMode = mode;
    });
    lastHost = out || lastHost;
  }

  function getToolsPage() {
    return document.getElementById("pageTools");
  }

  function getPanelFrom(node) {
    return node?.closest?.(".tools-console-dock") || document.querySelector("#pageTools .tools-console-dock");
  }

  function isExpanded() {
    return Boolean(getToolsPage()?.classList.contains("tools-log-expanded"));
  }

  function isPortraitMode() {
    return getMode() === MODE.PORTRAIT;
  }

  function applyPanelDrag(panel, offsetPx, expandedAtStart) {
    if (!panel) return;
    panel.classList.add("is-dragging");
    panel.dataset.toolsNotificationDrag = expandedAtStart ? "closing" : "opening";
    panel.style.setProperty("--tools-notification-drag-y", `${Math.round(offsetPx)}px`);
  }

  function clearPanelDrag(panel) {
    if (!panel) return;
    panel.classList.remove("is-dragging");
    delete panel.dataset.toolsNotificationDrag;
    panel.style.removeProperty("--tools-notification-drag-y");
  }

  function isPrimaryPointer(event) {
    return !(event.pointerType === "mouse" && event.button !== 0);
  }

  function shouldIgnorePanelDragTarget(target) {
    return Boolean(target?.closest?.(".tools-console-log__clearBtn, .tools-console-log__body, select, input, textarea, a[href]"));
  }

  function suppressNextClick(node) {
    if (!node) return;
    node.dataset.toolsNotificationSuppressClick = "1";
    global.setTimeout(() => {
      delete node.dataset.toolsNotificationSuppressClick;
    }, 0);
  }

  function bindSuppressedClick(node) {
    if (!node || node.dataset.toolsNotificationSuppressClickBound === "1") return;
    node.dataset.toolsNotificationSuppressClickBound = "1";
    node.addEventListener("click", (event) => {
      if (node.dataset.toolsNotificationSuppressClick !== "1") return;
      event.preventDefault();
      event.stopImmediatePropagation();
    }, true);
  }

  function bindDragSurface(surface, callbacks = {}, options = {}) {
    if (!surface || surface.dataset[options.boundKey] === "1") return;
    surface.dataset[options.boundKey] = "1";
    bindSuppressedClick(surface);

    let startX = 0;
    let startY = 0;
    let tracking = false;
    let dragging = false;
    let expandedAtStart = false;
    let pointerId = null;
    let panel = null;
    let lastX = 0;
    let lastY = 0;

    const stopTracking = () => {
      global.removeEventListener("pointermove", onPointerMove, true);
      global.removeEventListener("pointerup", onPointerUp, true);
      global.removeEventListener("pointercancel", onPointerCancel, true);
      pointerId = null;
      tracking = false;
    };

    const finishTracking = (event = null) => {
      if (!tracking || (event && pointerId !== null && event.pointerId !== pointerId)) return;
      const endX = Number.isFinite(event?.clientX) ? event.clientX : lastX;
      const endY = Number.isFinite(event?.clientY) ? event.clientY : lastY;
      const dx = endX - startX;
      const dy = endY - startY;
      const isVerticalSwipe = Math.abs(dy) >= SWIPE_THRESHOLD_PX && Math.abs(dy) > Math.abs(dx) * 1.15;
      const threshold = expandedAtStart ? DRAG_CLOSE_THRESHOLD_PX : DRAG_OPEN_THRESHOLD_PX;
      const nextExpanded = expandedAtStart ? dy > -threshold : dy > threshold;

      clearPanelDrag(panel);
      if (dragging || isVerticalSwipe) {
        suppressNextClick(surface);
        callbacks.setExpanded?.(nextExpanded);
      }
      dragging = false;
      panel = null;
      stopTracking();
    };

    function onPointerMove(event) {
      if (!tracking || (pointerId !== null && event.pointerId !== pointerId) || !panel || !isPortraitMode()) return;
      const dx = event.clientX - startX;
      const dy = event.clientY - startY;
      lastX = event.clientX;
      lastY = event.clientY;
      const isVertical = Math.abs(dy) > Math.abs(dx) * 1.15;
      if (!isVertical) return;

      const offset = expandedAtStart ? Math.min(0, dy) : Math.max(0, dy);
      if (Math.abs(offset) >= 1) {
        dragging = true;
        applyPanelDrag(panel, offset, expandedAtStart);
      }
      event.preventDefault();
    }

    function onPointerUp(event) {
      finishTracking(event);
    }

    function onPointerCancel(event) {
      if (pointerId !== null && event.pointerId !== pointerId) return;
      if (dragging) {
        finishTracking(null);
        return;
      }
      clearPanelDrag(panel);
      dragging = false;
      panel = null;
      stopTracking();
    }

    surface.addEventListener("pointerdown", (event) => {
      if (!isPortraitMode() || !isPrimaryPointer(event)) return;
      if (options.ignoreTarget?.(event.target)) return;

      panel = options.getPanel?.(surface) || getPanelFrom(surface);
      if (!panel) return;
      tracking = true;
      dragging = false;
      expandedAtStart = isExpanded();
      pointerId = event.pointerId;
      startX = event.clientX;
      startY = event.clientY;
      lastX = startX;
      lastY = startY;

      global.addEventListener("pointermove", onPointerMove, { capture: true, passive: false });
      global.addEventListener("pointerup", onPointerUp, { capture: true, passive: true });
      global.addEventListener("pointercancel", onPointerCancel, { capture: true, passive: true });
      try {
        surface.setPointerCapture(event.pointerId);
      } catch {}
    }, { passive: true });
  }

  function bindModeSync() {
    if (modeSyncBound) return;
    modeSyncBound = true;
    global.addEventListener("resize", () => {
      if (lastHost) render(lastHost, lastState, lastOptions);
    }, { passive: true });
  }

  function entryTitle(text) {
    const firstLine = String(text || "").split("\n").map((line) => line.trim()).find(Boolean) || "";
    const bracket = firstLine.match(/^\[([^\]]+)\]/);
    if (bracket) return bracket[1];
    const command = firstLine.match(/^>\s*(.+)$/);
    if (command) return command[1];
    return firstLine || uiText("notice", "Notice");
  }

  function entryBody(text) {
    return String(text || "")
      .replace(/^\[[^\]]+\]\n?/, "")
      .replace(/^>\s*.+\n?/, "")
      .trim();
  }

  function entrySummary(text) {
    const body = entryBody(text);
    if (!body) return uiText("tools_notifications_no_output", "(no output)");
    return body.split("\n").map((line) => line.trim()).filter(Boolean).slice(0, 2).join("\n");
  }

  function relativeTime(timestamp) {
    const seconds = Number(timestamp || 0);
    if (!Number.isFinite(seconds) || seconds <= 0) return "";
    const diffSeconds = Math.round(seconds - Date.now() / 1000);
    const abs = Math.abs(diffSeconds);
    const units = [
      ["day", 86400],
      ["hour", 3600],
      ["minute", 60],
      ["second", 1],
    ];
    const [unit, unitSeconds] = units.find((item) => abs >= item[1]) || units[units.length - 1];
    const value = Math.round(diffSeconds / unitSeconds);
    try {
      const lang = typeof LANG !== "undefined" ? LANG : undefined;
      return new Intl.RelativeTimeFormat(lang, { numeric: "always" }).format(value, unit);
    } catch {
      if (unit === "second") return "just now";
      return `${Math.abs(value)} ${unit}${Math.abs(value) === 1 ? "" : "s"} ago`;
    }
  }

  function nextRelativeTimeDelayMs(entries) {
    const now = Date.now() / 1000;
    const hasRecent = entries.some((entry) => {
      const seconds = Number(entry.timestamp || 0);
      return Number.isFinite(seconds) && seconds > 0 && Math.abs(now - seconds) < 60;
    });
    return hasRecent ? 1000 : 30000;
  }

  function scheduleRelativeTimeRefresh(entries) {
    if (relativeTimeTimer) {
      global.clearTimeout(relativeTimeTimer);
      relativeTimeTimer = null;
    }
    if (!lastHost || !entries.some((entry) => Number(entry.timestamp || 0) > 0)) return;
    relativeTimeTimer = global.setTimeout(() => {
      relativeTimeTimer = null;
      if (!lastHost) return;
      const model = buildModel(lastState);
      updateRelativeTimeLabels(lastHost, model.entries);
      scheduleRelativeTimeRefresh(model.entries);
    }, nextRelativeTimeDelayMs(entries));
  }

  function jobCommand(job) {
    const action = String(job?.action || "").trim();
    const payload = job?.payload && typeof job.payload === "object" ? job.payload : {};
    try {
      if (typeof getToolCommandPreview === "function") {
        const preview = getToolCommandPreview(action, payload);
        if (preview) return preview;
      }
    } catch {}
    return action ? action.replace(/_/g, " ") : uiText("notice", "Notice");
  }

  function jobFallbackMessage(job) {
    const status = String(job?.status || "");
    if (status === "running") {
      return String(job?.message || "").trim() || uiText("tools_notifications_running", "Running");
    }
    if (status === "failed") {
      return String(job?.error || job?.error_detail || "").trim() || uiText("tools_notifications_failed", "Failed");
    }
    return uiText("tools_notifications_done", "Done");
  }

  function actionDoneLabel(action) {
    try {
      const labels = typeof getActionLabel === "function" ? getActionLabel(action) : null;
      return String(labels?.done || "").trim();
    } catch {
      return "";
    }
  }

  function actionRunningLabel(action) {
    try {
      const labels = typeof getActionLabel === "function" ? getActionLabel(action) : null;
      return String(labels?.running || "").trim();
    } catch {
      return "";
    }
  }

  function actionFailedLabel(action) {
    try {
      const labels = typeof getActionLabel === "function" ? getActionLabel(action) : null;
      return String(labels?.failed || "").trim();
    } catch {
      return "";
    }
  }

  function jobOutput(job) {
    const result = job?.result && typeof job.result === "object" ? job.result : {};
    return normalizeText(job?.log || result.out || job?.error_detail || job?.error || job?.message || "");
  }

  function publishUnreadCount(state = lastState) {
    const count = unreadCount(state);
    if (count === lastPublishedUnreadCount) return;
    lastPublishedUnreadCount = count;
    global.renderToolsMeta?.();
  }

  function isNoOutputText(text) {
    const value = String(text || "").trim().toLowerCase();
    return value === "(no output)" || value === "no output" || value === "(no data)" || value === "no data" || value === "no sync";
  }

  function summaryVars(job) {
    const result = job?.result && typeof job.result === "object" ? job.result : {};
    return result.summary_vars && typeof result.summary_vars === "object" ? result.summary_vars : {};
  }

  function localizedResultSummary(job) {
    const result = job?.result && typeof job.result === "object" ? job.result : {};
    const key = String(result.summary_key || "").trim();
    if (!key) return "";
    return String(uiText(key, "", summaryVars(job)) || "").trim();
  }

  function friendlyEmptyOutput(job) {
    const action = String(job?.action || "").trim();
    switch (action) {
      case "git_sync":
        return uiText("git_result_sync_done", "Sync complete. No remote changes.");
      case "git_pull":
        return uiText("git_update_up_to_date", "Already up to date");
      case "git_reset":
        return uiText("git_result_reset_done", "Reset complete", summaryVars(job));
      case "git_checkout":
        return uiText("git_result_checkout_done", "Branch changed", summaryVars(job));
      case "git_remote_set":
        return uiText("git_result_remote_set_done", "Repository changed");
      case "git_remote_add":
        return uiText("git_result_remote_add_done", "Remote added/updated", summaryVars(job));
      case "git_reset_repo_checkout":
        return uiText("git_result_reset_repo_checkout_done", "Repository reset complete", summaryVars(job));
      case "shell_cmd":
        return uiText("tools_command_completed_no_output", "Command complete. No output.");
      default:
        return localizedResultSummary(job) || uiText("tools_command_completed_no_output", "Command complete. No output.");
    }
  }

  function displayJobOutput(job) {
    const raw = jobOutput(job);
    const result = job?.result && typeof job.result === "object" ? job.result : {};
    if (result.empty_output || isNoOutputText(raw)) return friendlyEmptyOutput(job);
    return raw;
  }

  function gitUpdateSummary(job) {
    const action = String(job?.action || "").trim();
    const result = job?.result && typeof job.result === "object" ? job.result : {};
    const summary = result.update_summary && typeof result.update_summary === "object" ? result.update_summary : null;
    return action === "git_pull" ? summary : null;
  }

  function formatGitUpdateSummary(job, updateSummary) {
    if (!updateSummary || typeof updateSummary !== "object") return null;
    if (!updateSummary.updated) {
      const text = uiText("git_update_up_to_date", "Already up to date");
      return {
        card: text,
        label: text,
        messages: [],
        stats: null,
        detail: `${text}\n\n${uiText("git_update_output", "Git output")}\n${displayJobOutput(job)}`.trim(),
      };
    }

    const commits = Array.isArray(updateSummary.commits) ? updateSummary.commits : [];
    const commitCount = Math.max(0, safeNumber(updateSummary.commit_count, commits.length));
    const filesChanged = Math.max(0, safeNumber(updateSummary.files_changed));
    const insertions = Math.max(0, safeNumber(updateSummary.insertions));
    const deletions = Math.max(0, safeNumber(updateSummary.deletions));
    const shown = commits.slice(0, 3);
    const remaining = Math.max(0, commitCount - shown.length);

    const lines = [uiText("git_update_new_updates", "New Updates"), ""];
    const messages = [];
    shown.forEach((commit) => {
      const message = String(commit?.message || commit?.hash || "").trim();
      if (message) {
        messages.push(message);
        lines.push(message);
      }
    });
    if (remaining > 0) {
      const moreText = uiText("git_update_more_commits", "{count} more commits", { count: remaining });
      messages.push(moreText);
      lines.push(moreText);
    }

    const commitText = uiText("git_update_commit_count", "{count} commits", { count: commitCount });
    const statText = filesChanged > 0
      ? uiText("git_update_shortstat", "{commits} | {files} | +{insertions} -{deletions}", {
          commits: commitText,
          files: uiText("git_update_file_count", "{count} files", { count: filesChanged }),
          insertions,
          deletions,
        })
      : uiText("git_update_shortstat_no_files", "{commits} | +{insertions} -{deletions}", {
          commits: commitText,
          insertions,
          deletions,
        });
    lines.push("", statText);

    const card = lines.join("\n").trim();
    const commitLog = commits
      .map((commit) => `${String(commit?.hash || "").trim()} ${String(commit?.message || "").trim()}`.trim())
      .filter(Boolean)
      .join("\n");
    const raw = jobOutput(job);
    const detailParts = [card];
    if (commitLog) detailParts.push("", uiText("git_update_commit_log", "Commit log"), commitLog);
    if (raw) detailParts.push("", uiText("git_update_output", "Git output"), raw);
    return {
      card,
      label: uiText("git_update_new_updates", "New Updates"),
      messages,
      stats: {
        commits: commitText,
        files: filesChanged > 0 ? uiText("git_update_file_count", "{count} files", { count: filesChanged }) : "",
        insertions,
        deletions,
      },
      detail: detailParts.join("\n").trim(),
    };
  }

  function formatJobText(job) {
    const updateSummary = gitUpdateSummary(job);
    const updateText = formatGitUpdateSummary(job, updateSummary);
    const updateDetail = normalizeText(updateText?.detail || updateSummary?.detail || "");
    let body = updateDetail;
    if (!body) {
      const resultSummary = localizedResultSummary(job);
      const raw = displayJobOutput(job);
      if (resultSummary && raw && raw !== resultSummary && !isNoOutputText(jobOutput(job))) {
        body = `${resultSummary}\n\n${uiText("tools_raw_output", "Raw output")}\n${raw}`;
      } else {
        body = resultSummary || raw || jobFallbackMessage(job);
      }
    }
    return `> ${jobCommand(job)}\n${body}`;
  }

  function makeJobEntry(job, index) {
    const status = String(job?.status || "");
    const action = String(job?.action || "").trim();
    const updateSummary = gitUpdateSummary(job);
    const updateText = updateSummary ? formatGitUpdateSummary(job, updateSummary) : null;
    const text = formatJobText(job);
    const timestamp = Number(job?.updated_at || job?.created_at || 0);
    const isNotice = Boolean(job?.payload?.notice);
    const entryId = `job-${job?.id || index}`;
    const ackId = updateSummary ? `${entryId}:${updateSummary.after || updateSummary.before || ""}` : "";
    let summary = jobFallbackMessage(job);
    if (status === "running") {
      summary = String(job?.message || "").trim() || actionRunningLabel(action) || summary;
    } else if (status === "failed") {
      summary = actionFailedLabel(action) || summary;
    } else if (isNotice) {
      summary = entrySummary(text);
    } else if (updateSummary) {
      summary = normalizeText(updateText?.card || updateSummary.card_summary || updateSummary.display || "") || entrySummary(text);
    } else {
      summary = localizedResultSummary(job) || actionDoneLabel(action) || summary;
    }
    return {
      id: entryId,
      ackId,
      source: status === "running" ? "current" : "history",
      text,
      title: jobCommand(job),
      summary,
      updateCard: updateText ? {
        label: updateText.label || "",
        messages: Array.isArray(updateText.messages) ? updateText.messages : [],
        stats: updateText.stats || null,
      } : null,
      status,
      highlight: Boolean(updateSummary?.updated && status === "done" && !acknowledgedUpdateIds.has(ackId)),
      updateKind: updateSummary ? "git_pull" : "",
      timestamp,
      timeLabel: relativeTime(timestamp),
    };
  }

  function makeEntry(text, source, index) {
    const normalized = normalizeText(text);
    return {
      id: `${source}-${index}-${hashText(normalized)}`,
      source,
      text: normalized,
      title: entryTitle(normalized),
      summary: entrySummary(normalized),
      timestamp: 0,
      timeLabel: "",
    };
  }

  function buildModel(state = {}) {
    const currentText = normalizeText(state.currentText);
    const historyText = normalizeText(state.historyText);
    const jobs = Array.isArray(state.jobs) ? state.jobs.filter(Boolean) : [];
    const entries = [];

    jobs
      .slice(0, HISTORY_LIMIT)
      .forEach((job, index) => entries.push(makeJobEntry(job, index)));

    if (currentText) entries.push(makeEntry(currentText, "current", 0));

    if (historyText) {
      historyText
        .split(/\n{2,}/)
        .map((entry) => entry.trim())
        .filter(Boolean)
        .slice(-HISTORY_LIMIT)
        .forEach((entry, index) => entries.push(makeEntry(entry, "history", index)));
    }

    return {
      entries,
      hasHistory: Boolean(historyText) || jobs.some((job) => String(job?.status || "") !== "running"),
      state: { jobs, historyText, currentText },
    };
  }

  function renderClearButton(model, context) {
    const clearButton = document.createElement("button");
    clearButton.type = "button";
    clearButton.className = "tools-console-log__clearBtn";
    clearButton.textContent = uiText("tools_notifications_clear", "Clear");
    clearButton.disabled = !model.hasHistory;
    clearButton.addEventListener("click", (event) => {
      event.stopPropagation();
      activeNotificationId = "";
      context.options.onClear?.();
    });
    return clearButton;
  }

  function renderHeader(context) {
    const header = document.createElement("span");
    header.className = "tools-console-log__header";
    header.dataset.toolsNotificationMode = context.mode;

    const title = document.createElement("span");
    title.className = "tools-console-log__headerTitle";
    title.textContent = uiText("tools_notifications", "Notifications");
    header.appendChild(title);

    if (context.mode === MODE.LANDSCAPE) {
      const actions = document.createElement("span");
      actions.className = "tools-console-log__headerActions";
      actions.appendChild(renderClearButton(context.model, context));
      header.appendChild(actions);
    }
    return header;
  }

  function renderEmpty() {
    const empty = document.createElement("span");
    empty.className = "tools-console-log__empty";
    empty.textContent = uiText("tools_notifications_empty", "No notifications");
    return empty;
  }

  function prefersReducedMotion() {
    return Boolean(global.matchMedia?.("(prefers-reduced-motion: reduce)")?.matches);
  }

  function parseTimeMs(value) {
    const text = String(value || "").trim().split(",")[0].trim();
    if (!text) return 0;
    const amount = Number.parseFloat(text);
    if (!Number.isFinite(amount)) return 0;
    return text.endsWith("ms") ? amount : amount * 1000;
  }

  function detailAnimationMs(node) {
    if (prefersReducedMotion()) return 0;
    try {
      const styles = global.getComputedStyle?.(node);
      const duration = parseTimeMs(styles?.getPropertyValue("--tools-detail-duration"));
      return Math.max(180, duration || 320) + 40;
    } catch {
      return 360;
    }
  }

  function clearCollapseRenderTimer() {
    if (!collapseRenderTimer) return;
    global.clearTimeout(collapseRenderTimer);
    collapseRenderTimer = null;
    collapsingNotificationId = "";
    collapsingUntil = 0;
    collapseHost = null;
  }

  function isCollapseInProgress(out) {
    return Boolean(
      collapseRenderTimer &&
      collapsingNotificationId &&
      Date.now() < collapsingUntil &&
      (!collapseHost || !out || collapseHost === out)
    );
  }

  function scrollDetailToLatest(out, id) {
    if (!id) return;
    const scroller = getLogScroller(out);
    const card = findCardById(scroller, id);
    const detail = card?.querySelector?.(".tools-console-log__detail");
    if (!detail) return;
    detail.scrollTop = detail.scrollHeight;
    detailScrollState.set(id, {
      scrollTop: detail.scrollTop,
      scrollHeight: detail.scrollHeight,
      clientHeight: detail.clientHeight,
    });
  }

  function rememberDetailScroll(id, detail) {
    if (!id || !detail) return;
    detailScrollState.set(id, {
      scrollTop: detail.scrollTop,
      scrollHeight: detail.scrollHeight,
      clientHeight: detail.clientHeight,
    });
  }

  function restoreDetailScroll(out, id) {
    if (!id) return;
    const scroller = getLogScroller(out);
    const card = findCardById(scroller, id);
    const detail = card?.querySelector?.(".tools-console-log__detail");
    const saved = detailScrollState.get(id);
    if (!detail || !saved) return;
    const max = Math.max(0, detail.scrollHeight - detail.clientHeight);
    detail.scrollTop = Math.min(Math.max(0, saved.scrollTop || 0), max);
  }

  function bindDetailScroller(detail, entry) {
    if (!detail || detail.dataset.toolsDetailScrollBound === "1") return;
    detail.dataset.toolsDetailScrollBound = "1";
    detail.addEventListener("scroll", () => rememberDetailScroll(entry.id, detail), { passive: true });
  }

  function setMeasuredDetailHeight(card) {
    const wrap = card?.querySelector?.(".tools-console-log__detailWrap");
    if (!wrap) return;
    wrap.style.maxHeight = `${wrap.scrollHeight}px`;
  }

  function stabilizeExpandedDetail(card) {
    const wrap = card?.querySelector?.(".tools-console-log__detailWrap");
    if (!wrap) return;
    wrap.style.transition = "none";
    wrap.style.animation = "none";
    wrap.style.opacity = "1";
    wrap.style.transform = "translateY(0)";
    setMeasuredDetailHeight(card);
    wrap.getBoundingClientRect();
  }

  function animateDetailCollapse(card) {
    const wrap = card?.querySelector?.(".tools-console-log__detailWrap");
    if (!wrap) return;
    wrap.style.maxHeight = `${wrap.scrollHeight}px`;
    wrap.style.opacity = "1";
    wrap.style.transform = "translateY(0)";
    wrap.getBoundingClientRect();
    card.classList.add("is-collapsing");
    card.classList.remove("is-expanded");
    card.setAttribute("aria-expanded", "false");
    global.requestAnimationFrame(() => {
      wrap.style.maxHeight = "0px";
      wrap.style.opacity = "0";
      wrap.style.transform = "translateY(-6px)";
    });
  }

  function getLogScroller(out) {
    if (!out) return null;
    const mode = out.dataset.toolsNotificationMode || getMode();
    if (mode === MODE.PORTRAIT) return out;
    return out?.querySelector?.(".tools-console-log__body") || out || null;
  }

  function findCardById(scroller, id) {
    if (!scroller || !id) return null;
    return Array.from(scroller.querySelectorAll("[data-notification-id]"))
      .find((card) => card.dataset.notificationId === id) || null;
  }

  function renderSignature(model, mode) {
    return JSON.stringify({
      mode,
      active: activeNotificationId,
      hasHistory: model.hasHistory,
      entries: model.entries.map((entry) => ({
        id: entry.id,
        source: entry.source,
        title: entry.title,
        summary: entry.summary,
        updateCard: entry.updateCard || null,
        text: entry.text,
        status: entry.status || "",
        highlight: Boolean(entry.highlight),
        updateKind: entry.updateKind || "",
        hasTime: Number(entry.timestamp || 0) > 0,
      })),
    });
  }

  function isRunningEntry(entry) {
    return String(entry?.status || "") === "running";
  }

  function canAutoFocusEntry(entry) {
    return Boolean(entry) && !isRunningEntry(entry);
  }

  function currentCards(out) {
    const scroller = getLogScroller(out);
    return scroller ? Array.from(scroller.querySelectorAll("[data-notification-id]")) : [];
  }

  function canPatchExistingCards(out, model) {
    const cards = currentCards(out);
    return cards.length === model.entries.length && cards.every((card, index) => (
      card.dataset.notificationId === model.entries[index]?.id
    ));
  }

  function setNodeText(node, value) {
    if (node && node.textContent !== value) node.textContent = value;
  }

  function appendTextNode(parent, className, text) {
    const node = document.createElement("span");
    node.className = className;
    node.textContent = text;
    parent.appendChild(node);
    return node;
  }

  function renderSummaryBody(body, entry) {
    if (!body) return;
    body.replaceChildren();
    body.classList.toggle("tools-console-log__cardBody--update", Boolean(entry.updateCard));
    if (!entry.updateCard) {
      body.textContent = entry.summary;
      return;
    }

    const label = String(entry.updateCard.label || "").trim();
    if (label) appendTextNode(body, "tools-console-log__updateLabel", label);

    const messages = Array.isArray(entry.updateCard.messages) ? entry.updateCard.messages.filter(Boolean) : [];
    if (messages.length) {
      const messageWrap = document.createElement("span");
      messageWrap.className = "tools-console-log__updateMessages";
      messages.slice(0, 3).forEach((message) => appendTextNode(messageWrap, "tools-console-log__updateMessage", message));
      body.appendChild(messageWrap);
    }

    const stats = entry.updateCard.stats && typeof entry.updateCard.stats === "object" ? entry.updateCard.stats : null;
    if (stats) {
      const statWrap = document.createElement("span");
      statWrap.className = "tools-console-log__updateStats";
      if (stats.commits) appendTextNode(statWrap, "tools-console-log__updateStat", stats.commits);
      if (stats.files) appendTextNode(statWrap, "tools-console-log__updateStat", stats.files);
      appendTextNode(statWrap, "tools-console-log__updateStat tools-console-log__updateStat--add", `+${Math.max(0, safeNumber(stats.insertions))}`);
      appendTextNode(statWrap, "tools-console-log__updateStat tools-console-log__updateStat--delete", `-${Math.max(0, safeNumber(stats.deletions))}`);
      body.appendChild(statWrap);
    } else if (!messages.length && label) {
      body.textContent = label;
    }
  }

  function patchCard(card, entry, context) {
    const expanded = activeNotificationId === entry.id;
    card.classList.toggle("tools-console-log__current", entry.source === "current");
    card.classList.toggle("tools-console-log__history", entry.source === "history");
    card.classList.toggle("is-expanded", expanded);
    card.classList.toggle("is-git-update", entry.updateKind === "git_pull");
    card.classList.toggle("is-unread-update", Boolean(entry.highlight));
    card.dataset.toolsNotificationMode = context.mode;
    card.setAttribute("aria-expanded", expanded ? "true" : "false");

    setNodeText(card.querySelector(".tools-console-log__cardTitle"), entry.title);
    const head = card.querySelector(".tools-console-log__cardHead");
    let time = card.querySelector(".tools-console-log__cardTime");
    if (entry.timeLabel) {
      if (!time && head) {
        time = document.createElement("span");
        time.className = "tools-console-log__cardTime";
        head.appendChild(time);
      }
      setNodeText(time, entry.timeLabel);
    } else if (time) {
      time.remove();
    }
    renderSummaryBody(card.querySelector(".tools-console-log__cardBody"), entry);
    setNodeText(card.querySelector(".tools-console-log__detail"), entry.text);
  }

  function patchExistingCards(out, model, context) {
    currentCards(out).forEach((card, index) => patchCard(card, model.entries[index], context));
    out.querySelectorAll(".tools-console-log__clearBtn").forEach((button) => {
      button.disabled = !model.hasHistory;
    });
  }

  function updateRelativeTimeLabels(out, entries) {
    if (!out) return;
    const labels = new Map(entries.map((entry) => [entry.id, entry.timeLabel || ""]));
    out.querySelectorAll("[data-notification-id]").forEach((card) => {
      const label = labels.get(card.dataset.notificationId) || "";
      const time = card.querySelector(".tools-console-log__cardTime");
      if (time && time.textContent !== label) time.textContent = label;
    });
  }

  function normalizeActiveEntry(model) {
    if (!activeNotificationId) return;
    if (!model.entries.some((entry) => entry.id === activeNotificationId)) activeNotificationId = "";
  }

  function latestEntry(model) {
    if (!model?.entries?.length) return null;
    let latest = null;
    let latestTime = 0;
    model.entries.forEach((entry) => {
      const timestamp = Number(entry.timestamp || 0);
      if (Number.isFinite(timestamp) && timestamp > 0 && timestamp >= latestTime) {
        latest = entry;
        latestTime = timestamp;
      }
    });
    if (latest) return latest;
    return model.entries.slice().reverse().find((entry) => entry.source === "current") || model.entries[model.entries.length - 1];
  }

  function createEntryFocus(out, entryId, expanded = true, options = {}) {
    const scroller = getLogScroller(out);
    const card = findCardById(scroller, entryId);
    const cardRect = card?.getBoundingClientRect?.();
    return {
      id: entryId,
      expanded,
      keyboard: false,
      instant: options.instant === true,
      smoothOnce: options.smoothOnce === true,
      stableDetail: options.stableDetail === true,
      mode: out?.dataset?.toolsNotificationMode || getMode(),
      scrollTop: scroller?.scrollTop || 0,
      cardTop: cardRect ? cardRect.top : null,
    };
  }

  function focusEntry(out, entryId, options = {}) {
    if (!entryId) return "";
    const expanded = options.expand !== false;
    clearCollapseRenderTimer();
    activeNotificationId = expanded ? entryId : "";
    pendingEntryFocus = createEntryFocus(out, entryId, expanded, options);
    detailScrollState.delete(entryId);
    return entryId;
  }

  function focusLatestEntry(out = lastHost, options = {}) {
    const model = buildModel(lastState);
    const entry = latestEntry(model);
    if (!entry) return "";
    const focusedId = focusEntry(out, entry.id, options);
    if (out) render(out, model.state, lastOptions, { force: true, preserveScroll: false });
    return focusedId;
  }

  function captureScrollAnchor(out, anchorId = activeNotificationId) {
    const scroller = getLogScroller(out);
    if (!scroller) return null;
    const card = findCardById(scroller, anchorId);
    const mode = out?.dataset?.toolsNotificationMode || getMode();
    return {
      anchorId,
      mode,
      scrollTop: scroller.scrollTop || 0,
      cardTop: card ? card.getBoundingClientRect().top : null,
    };
  }

  function captureEntryInteraction(out, entryId, expanded, keyboard) {
    const scroller = getLogScroller(out);
    const card = findCardById(scroller, entryId);
    const mode = out?.dataset?.toolsNotificationMode || getMode();
    const cardRect = card?.getBoundingClientRect?.();
    return {
      id: entryId,
      expanded,
      keyboard,
      mode,
      scrollTop: scroller?.scrollTop || 0,
      cardTop: cardRect ? cardRect.top : null,
    };
  }

  function restoreScrollAnchor(out, anchor) {
    const scroller = getLogScroller(out);
    if (!scroller || !anchor) return;
    let nextScrollTop = anchor.scrollTop;
    if (anchor.mode !== MODE.LANDSCAPE && anchor.anchorId && anchor.cardTop !== null) {
      const card = findCardById(scroller, anchor.anchorId);
      if (card) nextScrollTop += card.getBoundingClientRect().top - anchor.cardTop;
    }
    scroller.scrollTop = Math.max(0, nextScrollTop);
  }

  function clampScrollTop(scroller, value) {
    const max = Math.max(0, (scroller?.scrollHeight || 0) - (scroller?.clientHeight || 0));
    return Math.min(Math.max(0, value), max);
  }

  function restoreEntryInteraction(out, focus) {
    const scroller = getLogScroller(out);
    const card = findCardById(scroller, focus?.id);
    if (!scroller || !focus) return;
    if (!card || focus.cardTop === null) {
      scroller.scrollTop = clampScrollTop(scroller, focus.scrollTop || 0);
      return;
    }
    const cardDelta = card.getBoundingClientRect().top - focus.cardTop;
    if (Math.abs(cardDelta) < 1) return;
    const nextTop = (focus.scrollTop || 0) + cardDelta;
    scroller.scrollTop = clampScrollTop(scroller, nextTop);
  }

  function scrollEntryIntoView(out, focus, phase = "settled", opts = {}) {
    const scroller = getLogScroller(out);
    const card = findCardById(scroller, focus?.id);
    if (!scroller || !card) return;

    if (focus.keyboard) {
      try {
        card.focus({ preventScroll: true });
      } catch {}
    }

    const scrollerRect = scroller.getBoundingClientRect();
    const cardRect = card.getBoundingClientRect();
    const headRect = card.querySelector(".tools-console-log__cardHead")?.getBoundingClientRect?.() || cardRect;
    const mode = out.dataset.toolsNotificationMode || getMode();
    const topInset = mode === MODE.PORTRAIT ? 18 : 12;
    const bottomInset = mode === MODE.PORTRAIT ? 92 : 20;
    const viewportHeight = Math.max(scrollerRect.height - topInset - bottomInset, 1);
    const viewTop = scrollerRect.top + topInset;
    const viewBottom = scrollerRect.bottom - bottomInset;
    const heightDelta = Math.max(0, Number(opts.expandedHeightDelta) || 0);
    const effectiveCard = heightDelta > 0
      ? { top: cardRect.top, bottom: cardRect.bottom + heightDelta, height: cardRect.height + heightDelta }
      : cardRect;
    const targetRect = phase === "opening" ? headRect : effectiveCard;
    let delta = 0;

    // User-click expand policy:
    //   - if the whole expanded card already fits in view, no-op
    //   - if the card is taller than the viewport but its head is in view, no-op (let the
    //     detail's own scrollbar handle overflow instead of yanking the body)
    //   - otherwise, fall through and align so the detail box is visible
    const isUserInteraction = !focus.smoothOnce && !focus.stableDetail && !focus.instant;
    if (isUserInteraction) {
      const expandedFullyVisible = effectiveCard.top >= viewTop && effectiveCard.bottom <= viewBottom;
      if (expandedFullyVisible) return;
      const headVisible = headRect.top >= viewTop && headRect.bottom <= viewBottom;
      if (headVisible && effectiveCard.height >= viewportHeight) return;
    }

    if (phase === "opening" && targetRect.top >= viewTop && targetRect.bottom <= viewBottom) {
      return;
    }

    if (focus?.expanded && phase !== "opening" && effectiveCard.height >= viewportHeight) {
      delta = effectiveCard.top - viewTop;
    } else if (targetRect.top < viewTop) {
      delta = targetRect.top - viewTop;
    } else if (targetRect.bottom > viewBottom) {
      delta = targetRect.bottom - viewBottom;
    }

    if (Math.abs(delta) < 2) return;
    const target = clampScrollTop(scroller, scroller.scrollTop + delta);
    const behavior = opts.behavior || (prefersReducedMotion() ? "auto" : "smooth");
    try {
      scroller.scrollTo({ top: target, behavior });
    } catch {
      scroller.scrollTop = target;
    }
  }

  function predictedExpandedHeightDelta(card) {
    const wrap = card?.querySelector?.(".tools-console-log__detailWrap");
    if (!wrap) return 0;
    const rect = wrap.getBoundingClientRect();
    return Math.max(0, wrap.scrollHeight - rect.height);
  }

  function scheduleEntryFocus(out, focus) {
    if (!focus?.id) return;
    entryFocusToken += 1;
    const token = entryFocusToken;
    if (entryFocusTimer) {
      global.clearTimeout(entryFocusTimer);
      entryFocusTimer = null;
    }
    const mode = getMode();
    global.requestAnimationFrame(() => {
      if (token !== entryFocusToken) return;
      if (focus.instant) {
        scrollEntryIntoView(out, focus, "settled", { behavior: "auto" });
        return;
      }
      if (focus.smoothOnce) {
        scrollEntryIntoView(out, focus, "settled");
        return;
      }
      // User-click expand/collapse policy:
      //   - mouse/touch: never move the body scroll. The user clicked something they were
      //     already looking at; nudging the page underneath them feels like the card is
      //     "sliding up from the bottom". If the detail overflows below the viewport, the
      //     user can wheel down themselves; the detail's own scrollbar handles the rest.
      //   - keyboard activate (Tab→Enter): the activated card may be off-screen, so bring
      //     the head into view via the opening pass (which itself no-ops when the head is
      //     already visible).
      if (!focus.stableDetail) {
        if (focus.keyboard) {
          scrollEntryIntoView(out, focus, "opening");
        }
        return;
      }
      // Auto-focus with stableDetail (e.g. autoFocusLatest on a fresh notification): keep the
      // two-phase concurrent + settled flow so the freshly expanded card lands cleanly.
      if (focus.expanded && !prefersReducedMotion()) {
        const scroller = getLogScroller(out);
        const card = findCardById(scroller, focus.id);
        const heightDelta = predictedExpandedHeightDelta(card);
        scrollEntryIntoView(out, focus, "concurrent", { expandedHeightDelta: heightDelta });
        const settleDelay = mode === MODE.PORTRAIT ? 380 : 360;
        entryFocusTimer = global.setTimeout(() => {
          entryFocusTimer = null;
          if (token !== entryFocusToken) return;
          scrollEntryIntoView(out, focus, "settled");
        }, settleDelay);
        return;
      }
      scrollEntryIntoView(out, focus, "opening");
      const delay = focus.expanded
        ? (mode === MODE.PORTRAIT ? 380 : 360)
        : (mode === MODE.PORTRAIT ? 180 : 160);
      entryFocusTimer = global.setTimeout(() => {
        entryFocusTimer = null;
        if (token !== entryFocusToken) return;
        scrollEntryIntoView(out, focus, "settled");
      }, delay);
    });
  }

  function renderDetail(entry) {
    const wrap = document.createElement("span");
    wrap.className = "tools-console-log__detailWrap";

    const inner = document.createElement("span");
    inner.className = "tools-console-log__detailInner";

    const title = document.createElement("span");
    title.className = "tools-console-log__detailTitle";
    title.textContent = uiText("tools_notification_detail", "Detail log");
    inner.appendChild(title);

    const detail = document.createElement("span");
    detail.className = "tools-console-log__detail";
    detail.textContent = entry.text;
    bindDetailScroller(detail, entry);
    inner.appendChild(detail);
    wrap.appendChild(inner);

    return wrap;
  }

  function renderCard(entry, context) {
    const expanded = activeNotificationId === entry.id;
    const card = document.createElement("div");
    card.className = `tools-console-log__card tools-console-log__${entry.source}`;
    card.classList.toggle("is-expanded", expanded);
    card.classList.toggle("is-git-update", entry.updateKind === "git_pull");
    card.classList.toggle("is-unread-update", Boolean(entry.highlight));
    card.dataset.notificationId = entry.id;
    card.dataset.toolsNotificationMode = context.mode;
    card.setAttribute("role", "button");
    card.setAttribute("tabindex", "0");
    card.setAttribute("aria-expanded", expanded ? "true" : "false");

    const title = document.createElement("span");
    title.className = "tools-console-log__cardTitle";
    title.textContent = entry.title;

    const head = document.createElement("span");
    head.className = "tools-console-log__cardHead";
    head.appendChild(title);

    if (entry.timeLabel) {
      const time = document.createElement("span");
      time.className = "tools-console-log__cardTime";
      time.textContent = entry.timeLabel;
      head.appendChild(time);
    }

    card.appendChild(head);

    const body = document.createElement("span");
    body.className = "tools-console-log__cardBody";
    renderSummaryBody(body, entry);
    card.appendChild(body);

    card.appendChild(renderDetail(entry));

    let startX = 0;
    let startY = 0;
    let pointerMoved = false;

    const activateCard = (event, keyboard = false) => {
      if (pointerMoved) {
        pointerMoved = false;
        event.preventDefault();
        return;
      }
      const nextExpanded = !expanded;
      pendingEntryFocus = captureEntryInteraction(context.out, entry.id, nextExpanded, keyboard);
      acknowledgeUpdateEntry(entry, card);

      if (!nextExpanded) {
        clearCollapseRenderTimer();
        activeNotificationId = "";
        detailScrollState.delete(entry.id);
        collapsingNotificationId = entry.id;
        collapseHost = context.out;
        animateDetailCollapse(card);
        collapsingUntil = Date.now() + detailAnimationMs(card);
        collapseRenderTimer = global.setTimeout(() => {
          collapseRenderTimer = null;
          collapsingNotificationId = "";
          collapsingUntil = 0;
          collapseHost = null;
          render(context.out, context.model.state, context.options, { force: true });
        }, detailAnimationMs(card));
        return;
      }

      clearCollapseRenderTimer();
      activeNotificationId = nextExpanded ? entry.id : "";
      render(context.out, context.model.state, context.options);
    };

    card.addEventListener("pointerdown", (event) => {
      startX = event.clientX;
      startY = event.clientY;
      pointerMoved = false;
    }, { passive: true });

    card.addEventListener("pointermove", (event) => {
      if (Math.hypot(event.clientX - startX, event.clientY - startY) > 8) pointerMoved = true;
    }, { passive: true });

    card.addEventListener("click", (event) => {
      activateCard(event, event.detail === 0);
    });

    card.addEventListener("keydown", (event) => {
      if (event.key !== "Enter" && event.key !== " ") return;
      event.preventDefault();
      activateCard(event, true);
    });

    return card;
  }

  function renderFooter(model, context) {
    const footer = document.createElement("span");
    footer.className = "tools-console-log__footer";
    footer.dataset.toolsNotificationMode = context.mode;
    footer.appendChild(renderClearButton(model, context));
    return footer;
  }

  function renderNotificationCards(model, context) {
    const frag = document.createDocumentFragment();
    if (!model.entries.length) {
      frag.appendChild(renderEmpty());
      return frag;
    }
    model.entries.forEach((entry) => frag.appendChild(renderCard(entry, context)));
    return frag;
  }

  function renderBody(context) {
    const body = document.createElement("span");
    body.className = "tools-console-log__body";
    body.appendChild(renderNotificationCards(context.model, context));
    return body;
  }

  function renderLandscapePanel(context) {
    const frag = document.createDocumentFragment();
    frag.appendChild(renderHeader(context));
    frag.appendChild(renderBody(context));
    return frag;
  }

  function renderPortraitCenter(context) {
    const frag = document.createDocumentFragment();
    frag.appendChild(renderHeader(context));
    frag.appendChild(renderBody(context));
    frag.appendChild(renderFooter(context.model, context));
    return frag;
  }

  function render(out, state = {}, options = {}, renderOptions = {}) {
    if (!out) return;
    bindModeSync();
    let interactionFocus = pendingEntryFocus;
    let scrollAnchor = renderOptions.preserveScroll === false || interactionFocus ? null : captureScrollAnchor(out);
    const mode = getMode();
    const model = buildModel(state);
    lastState = model.state;
    lastOptions = options;
    publishUnreadCount(model.state);
    syncHostMode(out, mode);

    if (!renderOptions.force && isCollapseInProgress(out)) {
      updateRelativeTimeLabels(out, model.entries);
      scheduleRelativeTimeRefresh(model.entries);
      return;
    }

    normalizeActiveEntry(model);
    const autoFocusLatest = options.autoFocusLatest === true && !renderOptions.skipAutoFocusLatest;
    const latest = autoFocusLatest ? latestEntry(model) : null;
    if (canAutoFocusEntry(latest) && latest.id !== lastAutoFocusedEntryId) {
      lastAutoFocusedEntryId = latest.id;
      focusEntry(out, latest.id, { expand: true, smoothOnce: true, stableDetail: true });
      interactionFocus = pendingEntryFocus;
      scrollAnchor = null;
    }
    const signature = renderSignature(model, mode);
    const context = { out, mode, model, options };

    if (!interactionFocus && signature === lastRenderSignature && out.childElementCount > 0) {
      updateRelativeTimeLabels(out, model.entries);
      scheduleRelativeTimeRefresh(model.entries);
      return;
    }
    if (!interactionFocus && out.childElementCount > 0 && canPatchExistingCards(out, model)) {
      patchExistingCards(out, model, context);
      lastRenderSignature = signature;
      scheduleRelativeTimeRefresh(model.entries);
      if (activeNotificationId) {
        global.requestAnimationFrame(() => {
          const scroller = getLogScroller(out);
          const card = findCardById(scroller, activeNotificationId);
          setMeasuredDetailHeight(card);
          restoreDetailScroll(out, activeNotificationId);
        });
      }
      return;
    }
    out.replaceChildren(mode === MODE.PORTRAIT ? renderPortraitCenter(context) : renderLandscapePanel(context));
    lastRenderSignature = signature;
    if (interactionFocus) {
      restoreEntryInteraction(out, interactionFocus);
      if (interactionFocus.instant || interactionFocus.stableDetail) {
        const scroller = getLogScroller(out);
        stabilizeExpandedDetail(findCardById(scroller, interactionFocus.id));
      }
    } else {
      restoreScrollAnchor(out, scrollAnchor);
      if (activeNotificationId) {
        const scroller = getLogScroller(out);
        stabilizeExpandedDetail(findCardById(scroller, activeNotificationId));
      }
    }
    scheduleRelativeTimeRefresh(model.entries);
    if (interactionFocus) {
      pendingEntryFocus = null;
      if (interactionFocus.expanded) {
        global.requestAnimationFrame(() => {
          const scroller = getLogScroller(out);
          const card = findCardById(scroller, interactionFocus.id);
          setMeasuredDetailHeight(card);
          scrollDetailToLatest(out, interactionFocus.id);
        });
      }
      scheduleEntryFocus(out, interactionFocus);
    } else if (activeNotificationId) {
      global.requestAnimationFrame(() => {
        const scroller = getLogScroller(out);
        const card = findCardById(scroller, activeNotificationId);
        setMeasuredDetailHeight(card);
        restoreDetailScroll(out, activeNotificationId);
      });
    }
  }

  function bindStatusGesture(statusEl, callbacks = {}) {
    bindDragSurface(statusEl, callbacks, {
      boundKey: "toolsNotificationGestureBound",
      getPanel: getPanelFrom,
    });
  }

  function bindPanelDrag(panel, callbacks = {}) {
    bindDragSurface(panel, callbacks, {
      boundKey: "toolsNotificationPanelDragBound",
      getPanel: () => panel,
      ignoreTarget: shouldIgnorePanelDragTarget,
    });
  }

  function unreadCount(state = lastState) {
    return buildModel(state).entries.filter((entry) => Boolean(entry.highlight)).length;
  }

  global.CarrotToolsNotifications = {
    bindPanelDrag,
    bindStatusGesture,
    unreadCount,
    render,
    resetDetail() {
      activeNotificationId = "";
      lastAutoFocusedEntryId = "";
      detailScrollState.clear();
    },
    focusLatest(options = {}) {
      return focusLatestEntry(options.out || lastHost, options);
    },
    syncMode(out = lastHost) {
      if (out) syncHostMode(out);
    },
  };
})(window);
