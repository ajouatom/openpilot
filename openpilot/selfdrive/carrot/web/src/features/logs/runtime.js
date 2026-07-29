"use strict";

import { createDashcamPlayerSession } from "./dashcam_player_session.js";
import {
  createLogsPlayerDialog,
  createLogsPlayerSegmentList,
  normalizeLogsPlayerSegments,
} from "./player/components.js";
import {
  cancelDashcamRouteRender,
  dashcamDefaultRouteHeight,
  dashcamLayoutKey,
  dashcamSelectedForRoute,
  dashcamSortDirection,
  dashcamState,
  dashcamWindowNeedsRender,
  loadDashcamReadState,
  loadDashcamRoutes,
  markDashcamScrollBusy,
  maybeLoadMoreDashcamRoutes,
  openDashcamPlayer,
  renderDashcamRoute,
  renderDashcamRoutes,
  resumeDashcamUploadJobIfNeeded,
  scheduleDashcamWindowRender,
  scheduleSegmentListScrollPersist,
  setDashcamLoadingMoreUi,
  setDashcamSort,
  showDashcamRouteMenu,
  showDashcamSegmentMenu,
  startDashcamAutoRefresh,
  toggleDashcamRouteSelectAll,
  updateDashcamRouteSelectionUi,
  uploadDashcamSegments,
  uploadRecentDashcamSegments,
} from "./dashcam.js";
import {
  loadScreenrecordVideos,
  openScreenrecordPlayer,
  renderScreenrecordVideos,
  scheduleScreenrecordWindowRender,
  screenrecordApiPath,
  screenrecordShouldLoadMore,
  screenrecordState,
} from "./screenrecord.js";

// Logs page — shared infra used by both the Dashcam and Screen Recording tabs.
// Owns: tab state, scroll persistence, lazy-image observer, generic helpers,
// the video player, and page bind/init/teardown.

const LOGS_TAB_SELECTOR = "[data-logs-tab]";
const LOGS_MENU_SORT = "sort";
const LOGS_MENU_UPLOAD = "upload_recent";
const LOGS_RECENT_UPLOAD_LIMITS = Object.freeze([2, 5, 10]);

let logsActiveTab = "dashcam";
const logsScrollTops = { dashcam: 0, screen: 0 };
let logsLazyImageObserver = null;
let logsMenuOpen = false;
let logsTabsController = null;

function normalizeLogsTab(tab) {
  return tab === "screen" ? "screen" : "dashcam";
}

function logsTabItems() {
  const tabList = document.getElementById("logsTabs");
  return tabList ? Array.from(tabList.querySelectorAll(LOGS_TAB_SELECTOR)) : [];
}

function isLogsPageActive() {
  return CURRENT_PAGE === "logs";
}

function getLogsScroller(tab = logsActiveTab) {
  return document.getElementById(tab === "screen" ? "screenrecordVideos" : "dashcamRoutes");
}

function saveLogsScrollTop(tab = logsActiveTab) {
  const scroller = getLogsScroller(tab);
  if (!scroller) return;
  logsScrollTops[tab === "screen" ? "screen" : "dashcam"] = scroller.scrollTop || 0;
}

function restoreLogsScrollTop(tab = logsActiveTab, options = {}) {
  const scroller = getLogsScroller(tab);
  if (!scroller) return;
  const key = tab === "screen" ? "screen" : "dashcam";
  const nextTop = options.reset === true ? 0 : (logsScrollTops[key] || 0);
  if (CURRENT_PAGE === "logs") {
    window.scrollTo(0, 0);
    document.documentElement.scrollTop = 0;
    document.body.scrollTop = 0;
  }
  requestAnimationFrame(() => {
    if (!isLogsPageActive()) return;
    scroller.scrollTop = nextTop;
    requestAnimationFrame(() => {
      if (!isLogsPageActive()) return;
      scroller.scrollTop = nextTop;
      if (key === "dashcam" && typeof scheduleDashcamWindowRender === "function") scheduleDashcamWindowRender();
      if (key === "screen" && typeof scheduleScreenrecordWindowRender === "function") scheduleScreenrecordWindowRender();
    });
  });
}

function syncLogsTabs() {
  document.getElementById("logsTabs")?.setAttribute("aria-label", getUIText("logs", "Logs"));
}

// The menu body lives in the shared choice dialog, so only the trigger label
// needs to follow the active language.
function syncLogsMenu() {
  const button = document.getElementById("logsMenuButton");
  const menuLabel = getUIText("logs_menu", "Log menu");
  button?.setAttribute("aria-label", menuLabel);
  button?.setAttribute("title", menuLabel);
}

// Built on open so sort state and translations are always current.
function logsMenuChoices() {
  const sort = typeof dashcamSortDirection === "function" ? dashcamSortDirection() : "asc";
  return [
    { heading: getUIText("logs_sort", "Sort") },
    {
      label: getUIText("sort_ascending", "Sort: ascending"),
      value: `${LOGS_MENU_SORT}:asc`,
      selected: sort === "asc",
    },
    {
      label: getUIText("sort_descending", "Sort: descending"),
      value: `${LOGS_MENU_SORT}:desc`,
      selected: sort === "desc",
    },
    { heading: getUIText("recent_log_upload", "Upload recent logs") },
    ...LOGS_RECENT_UPLOAD_LIMITS.map((count) => ({
      label: getUIText("upload_recent_logs", "Upload recent {count}", { count }),
      value: `${LOGS_MENU_UPLOAD}:${count}`,
    })),
  ];
}

async function runLogsMenuAction(selected) {
  const [action, argument] = String(selected || "").split(":");
  if (action === LOGS_MENU_SORT) await setDashcamSort(argument === "desc" ? "desc" : "asc");
  else if (action === LOGS_MENU_UPLOAD) await uploadRecentDashcamSegments(Number(argument) || 0);
}

async function openLogsMenu() {
  if (logsMenuOpen) return;
  const menu = document.getElementById("logsMenu");
  logsMenuOpen = true;
  // Shared trigger styling marks the control as active while its popup is up.
  menu?.classList.add("is-open");
  let selected = null;
  try {
    selected = await openAppDialog({
      mode: "choice",
      title: getUIText("logs_menu", "Log menu"),
      choiceLayout: "list",
      choices: logsMenuChoices(),
    });
  } finally {
    logsMenuOpen = false;
    menu?.classList.remove("is-open");
  }
  await runLogsMenuAction(selected);
}

function bindLogsMenu() {
  const button = document.getElementById("logsMenuButton");
  if (!button) return;
  syncLogsMenu();
  if (button.dataset.bound === "1") return;
  button.dataset.bound = "1";
  button.addEventListener("click", () => { openLogsMenu().catch(() => {}); });
}

function formatRelativeEpoch(epochSeconds) {
  const epoch = Number(epochSeconds || 0);
  if (!Number.isFinite(epoch) || epoch <= 0) return "";
  const delta = Math.max(0, Math.floor(Date.now() / 1000) - Math.floor(epoch));
  if (delta < 60) return getUIText("just_now", "just now");
  if (delta < 3600) return getUIText("minutes_ago", "{count} min ago", { count: Math.floor(delta / 60) });
  if (delta < 86400) return getUIText("hours_ago", "{count} hr ago", { count: Math.floor(delta / 3600) });
  return getUIText("days_ago", "{count} days ago", { count: Math.floor(delta / 86400) });
}

function localizeRelativeLabel(label) {
  const text = String(label || "").trim();
  if (!text) return "";
  if (/^(방금\s*전|just\s*now)$/i.test(text)) return getUIText("just_now", "just now");
  const minuteMatch = text.match(/^(\d+)\s*(?:분\s*전|min(?:ute)?s?\s*ago)$/i);
  if (minuteMatch) return getUIText("minutes_ago", "{count} min ago", { count: minuteMatch[1] });
  const hourMatch = text.match(/^(\d+)\s*(?:시간\s*전|hr?s?\s*ago|hour?s?\s*ago)$/i);
  if (hourMatch) return getUIText("hours_ago", "{count} hr ago", { count: hourMatch[1] });
  const dayMatch = text.match(/^(\d+)\s*(?:일\s*전|day?s?\s*ago)$/i);
  if (dayMatch) return getUIText("days_ago", "{count} days ago", { count: dayMatch[1] });
  return text;
}

function formatLogBytes(bytes) {
  const n = Number(bytes) || 0;
  if (n < 1024) return `${n} B`;
  if (n < 1024 * 1024) return `${(n / 1024).toFixed(1)} KB`;
  if (n < 1024 * 1024 * 1024) return `${(n / (1024 * 1024)).toFixed(1)} MB`;
  return `${(n / (1024 * 1024 * 1024)).toFixed(1)} GB`;
}

// A completed segment appears as soon as its writer locks disappear. Keep a
// short thumbnail retry for conversion/startup races so a transient 404 does
// not leave the newly visible tile blank. Route previews keep their own
// preview->thumbnail fallback.
const LOGS_THUMB_MAX_RETRY = 8;
const LOGS_THUMB_RETRY_BASE_MS = 900;

function loadLogsLazyImage(img) {
  if (!img) return;
  const src = img.dataset?.src || "";
  if (!src) return;
  const thumb = img.closest(".dashcam-segment-thumb");
  const canRetry = !!thumb && !img.hasAttribute("data-fallback");
  if (thumb) thumb.classList.add("is-thumb-loading");

  const clearLoading = () => { if (thumb) thumb.classList.remove("is-thumb-loading"); };
  const detach = () => {
    img.removeEventListener("load", onLoad);
    img.removeEventListener("error", onError);
  };
  const onLoad = () => { detach(); clearLoading(); };
  const onError = () => {
    detach();
    const attempt = Number(img.dataset.thumbRetry || 0);
    if (!canRetry || attempt >= LOGS_THUMB_MAX_RETRY) { clearLoading(); return; }
    img.dataset.thumbRetry = String(attempt + 1);
    const delay = Math.round(LOGS_THUMB_RETRY_BASE_MS * Math.pow(1.5, attempt));
    window.setTimeout(() => {
      if (!img.isConnected || !isLogsPageActive()) { clearLoading(); return; }
      img.addEventListener("load", onLoad);
      img.addEventListener("error", onError);
      // cache-bust so a prior 404 is never replayed from the browser cache
      img.src = src + (src.includes("?") ? "&" : "?") + "ready=" + (attempt + 1);
    }, delay);
  };

  img.addEventListener("load", onLoad);
  img.addEventListener("error", onError);
  img.src = src;
  img.removeAttribute("data-src");
}

function hydrateLogsLazyImages(root) {
  if (!isLogsPageActive()) return;
  const scope = root || document;
  const images = Array.from(scope.querySelectorAll?.("img[data-src]") || []);
  if (!images.length) return;

  if (!("IntersectionObserver" in window)) {
    images.forEach(loadLogsLazyImage);
    return;
  }

  if (!logsLazyImageObserver) {
    logsLazyImageObserver = new IntersectionObserver((entries) => {
      entries.forEach((entry) => {
        if (!entry.isIntersecting) return;
        logsLazyImageObserver.unobserve(entry.target);
        loadLogsLazyImage(entry.target);
      });
    }, { root: null, rootMargin: "720px 0px", threshold: 0.01 });
  }

  images.forEach((img) => logsLazyImageObserver.observe(img));
}

function disconnectLogsLazyImages() {
  if (!logsLazyImageObserver) return;
  logsLazyImageObserver.disconnect();
  logsLazyImageObserver = null;
}

function unobserveLogsLazyImages(root) {
  if (!logsLazyImageObserver || !root) return;
  root.querySelectorAll?.("img[data-src]").forEach((img) => {
    logsLazyImageObserver.unobserve(img);
  });
}

function logsLoadingSkeletonHtml(type = "dashcam") {
  const count = type === "screen" ? 6 : 4;
  const itemClass = type === "screen" ? "logs-loading-row" : "logs-loading-card";
  return `<div class="logs-loading-list" aria-hidden="true">${Array.from({ length: count }, (_, i) =>
    `<div class="${itemClass}" style="--i:${i}"></div>`
  ).join("")}</div>`;
}

function logsEmptyStateHtml(type = "dashcam") {
  const isScreen = type === "screen";
  const title = isScreen
    ? getUIText("screenrecord_empty_title", "No screen recordings")
    : getUIText("dashcam_empty_title", "No dashcam records");

  return `
    <div class="logs-empty-state" role="status">
      <div class="logs-empty-state__title">${escapeHtml(title)}</div>
    </div>`;
}

function openLogsVideoPlayer(title, src, options = {}) {
  const overlay = document.createElement("div");
  const kind = String(options.kind || "video").replace(/[^a-z0-9_-]/gi, "");
  const playerSegments = normalizeLogsPlayerSegments(options.segments);
  const segmentSession = playerSegments.length && options.currentGroup
    ? createDashcamPlayerSession({
        group: options.currentGroup,
        segments: playerSegments.map((segment) => segment.id),
        activeSegment: options.activeSegment,
        previousSegment: options.previousSegment,
      })
    : null;
  const hasSendAction = typeof options.onSegmentSend === "function" || typeof options.onSend === "function";
  const hasMenuAction = typeof options.onSegmentMenu === "function" || typeof options.onMenu === "function";
  const sendButton = hasSendAction
    ? `<button class="dashcam-menu-btn dashcam-player-action dashcam-player-send" type="button" aria-label="${escapeHtml(getUIText("log_upload", "Upload Logs"))}" title="${escapeHtml(getUIText("log_upload", "Upload Logs"))}">
        <svg viewBox="0 0 24 24" aria-hidden="true"><path fill="currentColor" d="M2.01 21 23 12 2.01 3 2 10l15 2-15 2z"/></svg>
      </button>`
    : "";
  const menuButton = hasMenuAction
    ? `<button class="dashcam-menu-btn dashcam-player-action dashcam-player-menu" type="button" aria-label="${escapeHtml(getUIText("segment_menu", "Segment menu"))}" title="${escapeHtml(getUIText("segment_menu", "Segment menu"))}">
        <svg viewBox="0 0 24 24"><path fill="currentColor" d="M12 8a2 2 0 1 0 0-4 2 2 0 0 0 0 4m0 2a2 2 0 1 0 0 4 2 2 0 0 0 0-4m0 6a2 2 0 1 0 0 4 2 2 0 0 0 0-4"/></svg>
      </button>`
    : "";
  let segmentItems = playerSegments;
  let segmentItemById = new Map(segmentItems.map((segment) => [segment.id, segment]));
  const initialSegmentId = segmentSession?.snapshot().activeSegment || "";
  const fallbackMedia = Object.freeze({
    id: "",
    name: title || "Video",
    timeLabel: "",
    title: title || "Video",
    subtitle: String(options.subtitle || ""),
    src: String(src || ""),
    thumbnailSrc: "",
  });
  let currentMedia = segmentItemById.get(initialSegmentId) || fallbackMedia;
  overlay.className = `dashcam-player-overlay dashcam-player-overlay--${kind}`;
  const playerDialog = createLogsPlayerDialog({
    currentGroup: options.currentGroup,
    labels: {
      browser: getUIText("segment_browser", "Segments"),
      currentGroup: getUIText("current_group", "Current group"),
      segmentCount: getUIText("segment_count", "{count} segments"),
    },
  });
  playerDialog.frame.innerHTML = `
    <video class="dashcam-player-video" playsinline webkit-playsinline disablepictureinpicture disableremoteplayback controlslist="nodownload noplaybackrate noremoteplayback"></video>
    <div class="dashcam-player-toast" aria-live="polite"></div>
    <div class="dashcam-player-transport carrot-media-action-group" role="group" aria-label="${escapeHtml(getUIText("replay_playback_controls", "Playback controls"))}"></div>
    <div class="dashcam-player-top">
      <div class="dashcam-player-heading">
        <div class="dashcam-player-title">${escapeHtml(currentMedia.title)}</div>
        <div class="dashcam-player-subtitle"${currentMedia.subtitle ? "" : " hidden"}>${escapeHtml(currentMedia.subtitle)}</div>
      </div>
      ${sendButton}
      ${menuButton}
      <button class="dashcam-player-close c-close" type="button" aria-label="${escapeHtml(getUIText("close", "Close"))}" title="${escapeHtml(getUIText("close", "Close"))}">
        <svg viewBox="0 0 24 24" aria-hidden="true"><path d="M7 7l10 10M17 7L7 17"/></svg>
      </button>
    </div>`;
  const playerTitle = playerDialog.frame.querySelector(".dashcam-player-title");
  if (playerTitle) {
    playerTitle.id = "dashcamPlayerDialogTitle";
    playerDialog.dialog.setAttribute("aria-labelledby", playerTitle.id);
  }
  overlay.append(playerDialog.dialog);
  const videoEl = overlay.querySelector("video");
  videoEl.controls = false;
  videoEl.removeAttribute("controls");
  const toastEl = overlay.querySelector(".dashcam-player-toast");
  const subtitleEl = overlay.querySelector(".dashcam-player-subtitle");
  const downloadUrl = src + (src.includes("?") ? "&" : "?") + "download=1";
  let toastTimer = null;
  let suppressToasts = true;
  let player = null;
  let topActionPending = false;
  let navigationPending = false;
  let closed = false;
  let mediaSwitchToken = 0;
  let inputController = null;
  let segmentList = null;
  const transportEl = overlay.querySelector(".dashcam-player-transport");
  const mediaTransport = window.CarrotMediaTransport;
  let previousButton = null;
  let transportPlayButton = null;
  let nextButton = null;
  const downloadUrlFor = (value) => {
    const target = String(value || "");
    return target + (target.includes("?") ? "&" : "?") + "download=1";
  };
  const activeSegmentId = () => segmentSession?.snapshot().activeSegment || "";
  const resolveSubtitle = () => {
    const duration = Number(player?.duration ?? videoEl.duration);
    if (segmentSession && typeof options.subtitleForSegmentDuration === "function") {
      return options.subtitleForSegmentDuration(activeSegmentId(), duration) || currentMedia.subtitle;
    }
    if (typeof options.subtitleForDuration === "function") {
      return options.subtitleForDuration(duration) || currentMedia.subtitle;
    }
    return currentMedia.subtitle;
  };
  const updateSubtitle = () => {
    if (!subtitleEl) return;
    const resolved = resolveSubtitle();
    subtitleEl.textContent = resolved || "";
    subtitleEl.hidden = !resolved;
  };
  videoEl.addEventListener("loadedmetadata", updateSubtitle);
  const showToast = (text) => {
    if (!toastEl || suppressToasts || !text) return;
    toastEl.textContent = text;
    toastEl.classList.add("is-visible");
    if (toastTimer) window.clearTimeout(toastTimer);
    toastTimer = window.setTimeout(() => toastEl.classList.remove("is-visible"), 850);
  };
  const close = () => {
    if (closed) return;
    closed = true;
    try { options.onClose?.(activeSegmentId()); } catch {}
    mediaSwitchToken += 1;
    if (toastTimer) window.clearTimeout(toastTimer);
    inputController?.dispose?.();
    inputController = null;
    try { player?.destroy?.(); } catch {}
    overlay.remove();
  };
  const runNavigationAction = (handler) => {
    if (navigationPending || typeof handler !== "function") return;
    navigationPending = true;
    Promise.resolve()
      .then(() => handler({ close }))
      .catch((error) => {
        if (typeof showAppToast === "function") showAppToast(error?.message || String(error), { tone: "error" });
      })
      .finally(() => { navigationPending = false; });
  };
  const syncNavigationState = () => {
    if (!previousButton || !nextButton) return;
    if (segmentSession) {
      const state = segmentSession.snapshot();
      previousButton.disabled = !state.canMovePrevious;
      nextButton.disabled = !state.canMoveNext;
      return;
    }
    previousButton.disabled = typeof options.onPrevious !== "function";
    nextButton.disabled = typeof options.onNext !== "function";
  };
  const syncTransportPlayState = () => {
    if (!transportPlayButton || !mediaTransport?.setActionState) return;
    const paused = (typeof player?.paused === "boolean" ? player.paused : videoEl.paused)
      || (typeof player?.ended === "boolean" ? player.ended : videoEl.ended);
    mediaTransport.setActionState(transportPlayButton, paused ? "play" : "pause", {
      label: paused
        ? getUIText("replay_play", "Play replay")
        : getUIText("replay_pause", "Pause replay"),
    });
    transportPlayButton.setAttribute("aria-pressed", String(!paused));
  };
  const pauseCurrentPlayback = () => {
    try {
      if (player?.pause) player.pause();
      else videoEl.pause();
    } finally {
      window.requestAnimationFrame(syncTransportPlayState);
    }
  };
  const toggleTransportPlayback = async () => {
    try {
      const ended = typeof player?.ended === "boolean" ? player.ended : videoEl.ended;
      const paused = typeof player?.paused === "boolean" ? player.paused : videoEl.paused;
      if (ended) {
        if (player) player.currentTime = 0;
        else videoEl.currentTime = 0;
      }
      if (paused || ended) {
        const playResult = player?.play ? player.play() : videoEl.play();
        if (playResult && typeof playResult.then === "function") await playResult;
      } else {
        pauseCurrentPlayback();
      }
    } catch (error) {
      if (typeof showAppToast === "function") {
        showAppToast(error?.message || getUIText("playback_failed", "Playback failed"), { tone: "error" });
      }
    } finally {
      window.requestAnimationFrame(syncTransportPlayState);
    }
  };
  const requestCurrentPlayback = (token) => {
    const start = () => {
      if (closed || token !== mediaSwitchToken) return;
      const result = player?.play ? player.play() : videoEl.play();
      if (result && typeof result.catch === "function") result.catch(() => {});
    };
    window.setTimeout(() => {
      if (closed || token !== mediaSwitchToken) return;
      const media = player?.media || videoEl;
      if (media.readyState >= 2) start();
      else media.addEventListener("canplay", start, { once: true });
    }, 0);
  };
  const applyCurrentMedia = (media, optionsForChange = {}) => {
    if (!media?.src || closed) return false;
    const autoplay = optionsForChange.autoplay !== false;
    currentMedia = media;
    mediaSwitchToken += 1;
    const token = mediaSwitchToken;
    if (playerTitle) playerTitle.textContent = currentMedia.title || currentMedia.name;
    updateSubtitle();
    if (player) {
      player.source = {
        type: "video",
        title: currentMedia.title || currentMedia.name,
        sources: [{ src: currentMedia.src, type: "video/mp4" }],
      };
      player.download = downloadUrlFor(currentMedia.src);
    } else {
      videoEl.src = currentMedia.src;
      videoEl.load();
    }
    if (autoplay) requestCurrentPlayback(token);
    return true;
  };
  const applySegmentResult = (result, autoplay = true) => {
    if (!result?.changed) return false;
    const media = segmentItemById.get(result.state.activeSegment);
    if (!media) return false;
    segmentList?.setActive(result.state.activeSegment, true);
    syncNavigationState();
    applyCurrentMedia(media, { autoplay });
    options.onSegmentChange?.(result.state.activeSegment);
    return true;
  };
  const selectPlayerSegment = (segmentId) => {
    if (!segmentSession) return false;
    return applySegmentResult(segmentSession.select(segmentId));
  };
  const movePlayerSegment = (offset) => {
    if (!segmentSession) return false;
    return applySegmentResult(segmentSession.move(offset));
  };
  if (segmentSession && playerDialog.segmentHost) {
    segmentList = createLogsPlayerSegmentList(segmentItems, {
      activeSegmentId: activeSegmentId(),
      labels: {
        segments: getUIText("segment_browser", "Segments"),
        reading: getUIText("segment_reading", "Reading"),
        recent: getUIText("segment_recently_read", "Recently viewed"),
      },
      onSelect: selectPlayerSegment,
      statusFor: (segmentId) => segmentSession.statusFor(segmentId),
    });
    playerDialog.segmentHost.hidden = false;
    playerDialog.segmentHost.append(segmentList.element);
  }
  if (transportEl && mediaTransport?.createActionButton) {
    previousButton = mediaTransport.createActionButton("previous", {
      label: getUIText("replay_previous_segment", "Previous segment"),
      disabled: segmentSession ? !segmentSession.snapshot().canMovePrevious : typeof options.onPrevious !== "function",
      onActivate: () => runNavigationAction(segmentSession ? () => movePlayerSegment(-1) : options.onPrevious),
    });
    transportPlayButton = mediaTransport.createActionButton("play", {
      playLabel: getUIText("replay_play", "Play replay"),
      pauseLabel: getUIText("replay_pause", "Pause replay"),
      onActivate: toggleTransportPlayback,
    });
    nextButton = mediaTransport.createActionButton("next", {
      label: getUIText("replay_next_segment", "Next segment"),
      disabled: segmentSession ? !segmentSession.snapshot().canMoveNext : typeof options.onNext !== "function",
      onActivate: () => runNavigationAction(segmentSession ? () => movePlayerSegment(1) : options.onNext),
    });
    transportEl.append(previousButton, transportPlayButton, nextButton);
    ["pointerdown", "mousedown", "touchstart"].forEach((eventName) => {
      transportEl.addEventListener(eventName, (event) => event.stopPropagation(), { passive: true });
    });
    ["play", "pause", "ended"].forEach((eventName) => {
      videoEl.addEventListener(eventName, syncTransportPlayState);
    });
    transportPlayButton.setAttribute("aria-keyshortcuts", "Space K");
    previousButton.setAttribute("aria-keyshortcuts", "Shift+P");
    nextButton.setAttribute("aria-keyshortcuts", "Shift+N");
    syncTransportPlayState();
  } else if (transportEl) {
    transportEl.hidden = true;
  }
  inputController = mediaTransport?.createInputController?.({
    media: videoEl,
    stage: playerDialog.frame,
    rates: [0.5, 1, 1.5, 2],
    canInteract: () => !closed && overlay.isConnected,
    canPrevious: () => Boolean(previousButton && !previousButton.disabled),
    canNext: () => Boolean(nextButton && !nextButton.disabled),
    onTogglePlayback: () => transportPlayButton?.click(),
    onPrevious: () => previousButton?.click(),
    onNext: () => nextButton?.click(),
    onRate: (rate) => {
      if (player) player.speed = rate;
      else videoEl.playbackRate = rate;
    },
    onSeek: syncTransportPlayState,
    blockedPointerSelector: [
      "button",
      "input",
      "select",
      "a",
      ".plyr__controls",
      ".dashcam-player-top",
      ".dashcam-player-transport",
    ].join(", "),
  }) || null;
  overlay.addEventListener("click", (ev) => {
    if (ev.target === overlay) close();
  });
  overlay.querySelector(".dashcam-player-close")?.addEventListener("click", close);
  const runTopAction = (event, handler, stateClass) => {
    event.stopPropagation();
    if (topActionPending || typeof handler !== "function") return;
    topActionPending = true;
    const trigger = event.currentTarget;
    trigger.disabled = true;
    overlay.classList.add(stateClass);
    Promise.resolve()
      .then(() => handler({ close }))
      .catch((error) => {
        if (typeof showAppToast === "function") showAppToast(error?.message || String(error), { tone: "error" });
      })
      .finally(() => {
        overlay.classList.remove(stateClass);
        trigger.disabled = false;
        topActionPending = false;
      });
  };
  overlay.querySelector(".dashcam-player-send")?.addEventListener("click", (event) => {
    const targetSegment = activeSegmentId();
    const handler = typeof options.onSegmentSend === "function"
      ? ({ close: closePlayer } = {}) => options.onSegmentSend(targetSegment, { close: closePlayer })
      : options.onSend;
    pauseCurrentPlayback();
    runTopAction(event, handler, "is-action-open");
  });
  overlay.querySelector(".dashcam-player-menu")?.addEventListener("click", (event) => {
    const targetSegment = activeSegmentId();
    const handler = typeof options.onSegmentMenu === "function"
      ? ({ close: closePlayer } = {}) => options.onSegmentMenu(targetSegment, { close: closePlayer })
      : options.onMenu;
    runTopAction(event, handler, "is-menu-open");
  });
  document.body.appendChild(overlay);
  requestAnimationFrame(() => {
    if (closed) return;
    overlay.classList.add("is-open");
    try {
      player = new Plyr(videoEl, {
        controls: ["play", "progress", "current-time", "duration", "settings", "fullscreen", "download"],
        hideControls: false,
        seekTime: 5,
        settings: ["speed"],
        speed: { selected: 1, options: [0.5, 1, 1.5, 2] },
        keyboard: { focused: true, global: false },
        fullscreen: { enabled: true, fallback: true, iosNative: true },
        urls: { download: downloadUrlFor(currentMedia.src) },
      });
      videoEl.controls = false;
      videoEl.removeAttribute("controls");
      player.source = {
        type: "video",
        title: currentMedia.title || currentMedia.name,
        sources: [{ src: currentMedia.src, type: "video/mp4" }],
      };
      player.once("ready", () => {
        const container = player.elements?.container || overlay;
        if (transportEl && transportEl.parentElement !== container) container.appendChild(transportEl);
        const bindBtn = (sel, label) => {
          container.querySelectorAll(sel).forEach((btn) => btn.addEventListener("click", () => showToast(label)));
        };
        bindBtn('[data-plyr="download"]', `⤓ ${getUIText("download", "Download")}`);
        container.addEventListener("keydown", (ev) => {
          if (ev.key === "ArrowLeft") showToast(`⏪ ${getUIText("rewind_5", "5s")}`);
          else if (ev.key === "ArrowRight") showToast(`${getUIText("forward_5", "5s")} ⏩`);
        });
        player.on("play", () => showToast(`▶ ${getUIText("play", "Play")}`));
        player.on("pause", () => showToast(`⏸ ${getUIText("pause", "Pause")}`));
        player.on("ended", () => showToast(getUIText("ended", "End")));
        player.on("ratechange", () => showToast(`⚡ ${player.speed}x`));
        player.on("enterfullscreen", () => showToast(`⛶ ${getUIText("fullscreen", "Fullscreen")}`));
        player.on("exitfullscreen", () => showToast(getUIText("fullscreen_exit", "Exit fullscreen")));
        videoEl.addEventListener("enterpictureinpicture", () => showToast("⊞ PiP"));
        videoEl.addEventListener("leavepictureinpicture", () => showToast(`⊟ ${getUIText("pip_exit", "Exit PiP")}`));
        window.setTimeout(() => { suppressToasts = false; }, 350);
      });
    } catch (err) {
      // Plyr can be unavailable on older embedded browsers. Preserve the
      // browser's native video transport as the fallback.
      videoEl.controls = true;
      videoEl.setAttribute("controls", "");
      videoEl.src = currentMedia.src;
    }
  });
  const updateSegments = (values) => {
    if (!segmentSession || closed) return null;
    const nextItems = normalizeLogsPlayerSegments(values);
    if (!nextItems.length) return segmentSession.snapshot();
    const previousActive = activeSegmentId();
    const previousSource = currentMedia.src;
    segmentItems = nextItems;
    segmentItemById = new Map(segmentItems.map((segment) => [segment.id, segment]));
    const result = segmentSession.replaceSegments(segmentItems.map((segment) => segment.id), {
      activeSegment: previousActive,
    });
    const nextMedia = segmentItemById.get(result.state.activeSegment);
    segmentList?.render(segmentItems, result.state.activeSegment);
    syncNavigationState();
    if (nextMedia) {
      if (result.state.activeSegment !== previousActive || nextMedia.src !== previousSource) {
        applyCurrentMedia(nextMedia, { autoplay: false });
      } else {
        currentMedia = nextMedia;
        if (playerTitle) playerTitle.textContent = currentMedia.title || currentMedia.name;
        updateSubtitle();
      }
    }
    return result.state;
  };
  return Object.freeze({
    close,
    isOpen: () => !closed && overlay.isConnected,
    updateSegments,
  });
}

function activateLogsTab(tab, options = {}) {
  const nextTab = normalizeLogsTab(tab);
  const shouldLoad = options.load !== false;
  if (nextTab !== logsActiveTab) saveLogsScrollTop(logsActiveTab);
  logsActiveTab = nextTab;

  // Each tab owns its panel through aria-controls, so selection state and panel
  // visibility stay in sync from one source without per-tab element lookups.
  for (const item of logsTabItems()) {
    const selected = normalizeLogsTab(item.dataset.logsTab) === logsActiveTab;
    item.classList.toggle("is-active", selected);
    item.setAttribute("aria-selected", selected ? "true" : "false");
    const panel = document.getElementById(item.getAttribute("aria-controls") || "");
    if (panel) panel.hidden = !selected;
  }
  logsTabsController?.sync();

  if (shouldLoad) {
    if (logsActiveTab === "screen" && !screenrecordState.initialized) {
      screenrecordState.initialized = true;
      loadScreenrecordVideos().catch(() => {});
    } else if (logsActiveTab === "screen") {
      renderScreenrecordVideos();
      loadScreenrecordVideos({ silent: true }).catch(() => {});
    } else if (dashcamState.initialized) {
      loadDashcamRoutes({ silent: true }).catch(() => {});
    }
  }
  if (options.restoreScroll !== false) restoreLogsScrollTop(logsActiveTab);
}

function bindLogsTabs(tabList) {
  if (!tabList) return;
  syncLogsTabs();
  if (tabList.dataset.bound === "1") return;
  tabList.dataset.bound = "1";

  tabList.addEventListener("click", (event) => {
    const item = event.target?.closest?.(LOGS_TAB_SELECTOR);
    if (item && tabList.contains(item)) activateLogsTab(item.dataset.logsTab);
  });

  // Shared control owns tablist keyboard semantics: arrow/Home/End move and
  // activate, and the roving tabindex keeps a single tab stop.
  logsTabsController = window.CarrotUI?.segmentedControl?.create(tabList, {
    itemSelector: LOGS_TAB_SELECTOR,
    onActivate: (item) => activateLogsTab(item?.dataset?.logsTab),
  }) || null;
}

function handleLogsPageChange(event) {
  const page = event?.detail?.page || "";
  if (page === "logs") return;
  // The menu is a modal dialog now: it owns its own dismissal, and its backdrop
  // blocks the navigation that would fire this handler while it is open.
  saveLogsScrollTop(logsActiveTab);
  cancelDashcamRouteRender();
  dashcamState.loadSeq += 1;
  screenrecordState.loadSeq += 1;
  dashcamState.loading = false;
  dashcamState.loadingMore = false;
  dashcamState.loadingSegments?.clear?.();
  setDashcamLoadingMoreUi(false);
  screenrecordState.loading = false;
  dashcamState.scrollBusy = false;
  if (dashcamState.scrollTimer) {
    window.clearTimeout(dashcamState.scrollTimer);
    dashcamState.scrollTimer = null;
  }
  if (dashcamState.layoutTimer) {
    window.clearTimeout(dashcamState.layoutTimer);
    dashcamState.layoutTimer = null;
  }
  disconnectLogsLazyImages();
}

function bindLogsPage() {
  const tabList = document.getElementById("logsTabs");
  const routesHost = document.getElementById("dashcamRoutes");
  const screenHost = document.getElementById("screenrecordVideos");
  bindLogsMenu();
  bindLogsTabs(tabList);

  if (!dashcamState.layoutBound) {
    dashcamState.layoutBound = true;
    dashcamState.landscape = isCompactLandscapeMode();
    dashcamState.layoutKey = dashcamLayoutKey();
    window.addEventListener("carrot:pagechange", handleLogsPageChange);
    window.addEventListener("carrot:languagechange", () => {
      syncLogsMenu();
      syncLogsTabs();
      dashcamState.signature = "";
      screenrecordState.signature = "";
      dashcamState.routeHeights = Object.create(null);
      const dashcamHost = document.getElementById("dashcamRoutes");
      if (dashcamHost) dashcamHost.dataset.signature = "";
      const screenHost = document.getElementById("screenrecordVideos");
      if (screenHost) screenHost.dataset.signature = "";

      if (isLogsPageActive()) {
        renderDashcamRoutes({ animate: false });
        if (typeof renderScreenrecordVideos === "function") renderScreenrecordVideos({ animate: false });
      }
    });
    window.addEventListener("resize", () => {
      if (CURRENT_PAGE !== "logs") return;
      if (dashcamState.layoutTimer) window.clearTimeout(dashcamState.layoutTimer);
      dashcamState.layoutTimer = window.setTimeout(() => {
        dashcamState.layoutTimer = null;
        if (!isLogsPageActive()) return;
        const nextLandscape = isCompactLandscapeMode();
        const nextLayoutKey = dashcamLayoutKey();
        if (dashcamState.layoutKey === nextLayoutKey) return;
        dashcamState.landscape = nextLandscape;
        dashcamState.layoutKey = nextLayoutKey;
        dashcamState.routeHeights = Object.create(null);
        dashcamState.routeHeight = dashcamDefaultRouteHeight();
        const dashcamHost = document.getElementById("dashcamRoutes");
        if (dashcamHost) dashcamHost.dataset.signature = "";
        renderDashcamRoutes({ animate: false });
        if (typeof renderScreenrecordVideos === "function") renderScreenrecordVideos({ preserve: true, animate: false });
      }, 120);
    }, { passive: true });
  }

  if (routesHost && routesHost.dataset.bound !== "1") {
    routesHost.dataset.bound = "1";
    routesHost.addEventListener("scroll", () => {
      markDashcamScrollBusy();
      saveLogsScrollTop("dashcam");
      if (dashcamWindowNeedsRender(routesHost)) scheduleDashcamWindowRender();
      maybeLoadMoreDashcamRoutes(routesHost);
    }, { passive: true });
    routesHost.addEventListener("scroll", (ev) => {
      const segmentList = ev.target?.closest?.(".dashcam-segment-list");
      if (!segmentList || segmentList === routesHost) return;
      scheduleSegmentListScrollPersist(segmentList);
    }, { passive: true, capture: true });
    routesHost.addEventListener("click", (ev) => {
      const actionEl = ev.target?.closest?.("[data-action]");
      if (!actionEl) return;
      const action = actionEl.dataset.action;
      const route = actionEl.dataset.route || "";
      const segment = actionEl.dataset.segment || "";
      if (action === "toggle-route") {
        if (dashcamState.expanded.has(route)) dashcamState.expanded.delete(route);
        else dashcamState.expanded.add(route);
        if (route && dashcamState.routeHeights) delete dashcamState.routeHeights[route];
        if (!renderDashcamRoute(route)) renderDashcamRoutes({ animate: false });
      } else if (action === "play") {
        openDashcamPlayer(route, segment);
      } else if (action === "segment-menu") {
        ev.stopPropagation();
        showDashcamSegmentMenu(route, segment).catch(() => {});
      } else if (action === "route-menu") {
        ev.stopPropagation();
        showDashcamRouteMenu(route).catch(() => {});
      } else if (action === "select-route") {
        const shouldClear = actionEl.dataset.selected === "1";
        toggleDashcamRouteSelectAll(route, shouldClear).catch(() => {});
      } else if (action === "upload-selected") {
        const entry = dashcamState.routes.find((item) => item.route === route);
        const targets = dashcamSelectedForRoute(entry || { segmentFolders: [] });
        uploadDashcamSegments(targets).catch(() => {});
      }
    });
    routesHost.addEventListener("change", (ev) => {
      const input = ev.target;
      if (!input?.matches?.('input[data-action="select-segment"]')) return;
      const segment = input.dataset.segment || "";
      if (input.checked) dashcamState.selected.add(segment);
      else dashcamState.selected.delete(segment);
      const route = input.closest("[data-route-card]")?.dataset.routeCard || "";
      if (!updateDashcamRouteSelectionUi(route)) renderDashcamRoutes({ animate: false });
    });
  }

  if (screenHost && screenHost.dataset.bound !== "1") {
    screenHost.dataset.bound = "1";
    screenHost.addEventListener("scroll", () => {
      markDashcamScrollBusy();
      saveLogsScrollTop("screen");
      scheduleScreenrecordWindowRender();
      if (screenrecordShouldLoadMore(screenHost)) {
        loadScreenrecordVideos({ silent: true, append: true }).catch(() => {});
      }
    }, { passive: true });
    screenHost.addEventListener("click", (ev) => {
      const actionEl = ev.target?.closest?.("[data-action]");
      if (!actionEl) return;
      if (actionEl.dataset.action === "download-screenrecord") {
        const id = actionEl.dataset.id || "";
        if (id) window.open(screenrecordApiPath("download", id), "_blank", "noopener");
      } else if (actionEl.dataset.action === "play-screenrecord") {
        openScreenrecordPlayer(actionEl.dataset.id || "", actionEl.dataset.name || "");
      }
    });
  }
}

function initLogsPage() {
  bindLogsPage();
  activateLogsTab(logsActiveTab, { load: false });
  startDashcamAutoRefresh();
  loadDashcamReadState().catch(() => {});
  resumeDashcamUploadJobIfNeeded().catch(() => {});
  if (logsActiveTab === "screen") {
    if (!screenrecordState.initialized) {
      screenrecordState.initialized = true;
      loadScreenrecordVideos().catch(() => {});
    } else {
      renderScreenrecordVideos({ preserve: true });
      loadScreenrecordVideos({ silent: true }).catch(() => {});
    }
  } else if (!dashcamState.initialized) {
    dashcamState.initialized = true;
    loadDashcamRoutes().catch(() => {});
  } else {
    renderDashcamRoutes({ animate: false, preserve: true });
    loadDashcamRoutes({ silent: true }).catch(() => {});
  }
}

const CarrotLogsRuntime = Object.freeze({
  init: initLogsPage,
  activateTab: activateLogsTab,
  dashcam: Object.freeze({
    load: loadDashcamRoutes,
    render: renderDashcamRoutes,
  }),
  screenrecord: Object.freeze({
    load: loadScreenrecordVideos,
    render: renderScreenrecordVideos,
  }),
});

globalThis.CarrotLogsRuntime = CarrotLogsRuntime;

export {
  CarrotLogsRuntime,
  activateLogsTab,
  formatLogBytes,
  formatRelativeEpoch,
  getLogsScroller,
  hydrateLogsLazyImages,
  initLogsPage,
  isLogsPageActive,
  localizeRelativeLabel,
  logsActiveTab,
  logsEmptyStateHtml,
  logsLoadingSkeletonHtml,
  logsScrollTops,
  openLogsVideoPlayer,
  restoreLogsScrollTop,
  syncLogsMenu,
  unobserveLogsLazyImages,
};
