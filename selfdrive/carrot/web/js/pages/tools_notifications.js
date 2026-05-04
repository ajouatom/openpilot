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
  const detailScrollState = new Map();
  let lastRenderSignature = "";

  function uiText(key, fallback, vars = null) {
    return typeof getUIText === "function" ? getUIText(key, fallback, vars) : fallback;
  }

  function normalizeText(value) {
    return String(value ?? "").replace(/\s+$/, "");
  }

  function hashText(value) {
    let hash = 0;
    const text = String(value || "");
    for (let i = 0; i < text.length; i += 1) {
      hash = ((hash << 5) - hash + text.charCodeAt(i)) | 0;
    }
    return Math.abs(hash).toString(36);
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

  function formatJobText(job) {
    const body = jobOutput(job) || jobFallbackMessage(job);
    return `> ${jobCommand(job)}\n${body}`;
  }

  function makeJobEntry(job, index) {
    const text = formatJobText(job);
    const status = String(job?.status || "");
    const action = String(job?.action || "").trim();
    const timestamp = Number(job?.updated_at || job?.created_at || 0);
    const isNotice = Boolean(job?.payload?.notice);
    let summary = jobFallbackMessage(job);
    if (status === "running") {
      summary = String(job?.message || "").trim() || actionRunningLabel(action) || summary;
    } else if (status === "failed") {
      summary = actionFailedLabel(action) || summary;
    } else if (isNotice) {
      summary = entrySummary(text);
    } else {
      summary = actionDoneLabel(action) || summary;
    }
    return {
      id: `job-${job?.id || index}`,
      source: status === "running" ? "current" : "history",
      text,
      title: jobCommand(job),
      summary,
      status,
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
    detail.addEventListener("wheel", (event) => event.stopPropagation(), { passive: true });
    detail.addEventListener("touchmove", (event) => event.stopPropagation(), { passive: true });
  }

  function setMeasuredDetailHeight(card) {
    const wrap = card?.querySelector?.(".tools-console-log__detailWrap");
    if (!wrap) return;
    wrap.style.maxHeight = `${wrap.scrollHeight}px`;
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
        text: entry.text,
        status: entry.status || "",
        hasTime: Number(entry.timestamp || 0) > 0,
      })),
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
    const nextTop = (focus.scrollTop || 0) + card.getBoundingClientRect().top - focus.cardTop;
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
    try {
      scroller.scrollTo({ top: target, behavior: prefersReducedMotion() ? "auto" : "smooth" });
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
    body.textContent = entry.summary;
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
    const interactionFocus = pendingEntryFocus;
    const scrollAnchor = renderOptions.preserveScroll === false || interactionFocus ? null : captureScrollAnchor(out);
    const mode = getMode();
    const model = buildModel(state);
    lastState = model.state;
    lastOptions = options;
    syncHostMode(out, mode);

    if (!renderOptions.force && isCollapseInProgress(out)) {
      updateRelativeTimeLabels(out, model.entries);
      scheduleRelativeTimeRefresh(model.entries);
      return;
    }

    normalizeActiveEntry(model);
    const signature = renderSignature(model, mode);
    const canPatchExisting = !interactionFocus && signature === lastRenderSignature && out.childElementCount > 0;
    const context = { out, mode, model, options };

    if (canPatchExisting) {
      updateRelativeTimeLabels(out, model.entries);
      scheduleRelativeTimeRefresh(model.entries);
      return;
    }
    out.replaceChildren(mode === MODE.PORTRAIT ? renderPortraitCenter(context) : renderLandscapePanel(context));
    lastRenderSignature = signature;
    if (interactionFocus) {
      restoreEntryInteraction(out, interactionFocus);
    } else {
      restoreScrollAnchor(out, scrollAnchor);
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

  global.CarrotToolsNotifications = {
    bindPanelDrag,
    bindStatusGesture,
    render,
    resetDetail() {
      activeNotificationId = "";
      detailScrollState.clear();
    },
    syncMode(out = lastHost) {
      if (out) syncHostMode(out);
    },
  };
})(window);
