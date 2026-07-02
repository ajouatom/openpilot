"use strict";

// Logs page — shared infra used by both the Dashcam and Screen Recording tabs.
// Owns: tab state, scroll persistence, lazy-image observer, generic helpers,
// the video player, and page bind/init/teardown.

let logsActiveTab = "dashcam";
const logsScrollTops = { dashcam: 0, screen: 0 };
let logsLazyImageObserver = null;

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

function loadLogsLazyImage(img) {
  if (!img) return;
  const src = img.dataset?.src || "";
  if (!src) return;
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
  overlay.className = `dashcam-player-overlay dashcam-player-overlay--${kind}`;
  overlay.innerHTML = `<div class="dashcam-player-dialog" role="dialog" aria-modal="true">
    <div class="dashcam-player-frame">
      <video class="dashcam-player-video" playsinline></video>
      <div class="dashcam-player-toast" aria-live="polite"></div>
      <div class="dashcam-player-top">
        <div class="dashcam-player-title">${escapeHtml(title || "Video")}</div>
        <button class="dashcam-player-close" type="button" aria-label="${escapeHtml(getUIText("close", "Close"))}" title="${escapeHtml(getUIText("close", "Close"))}">
          <svg viewBox="0 0 24 24"><path fill="currentColor" d="M18.3 5.71 12 12l6.3 6.29-1.41 1.41L10.59 13.41 4.29 19.71 2.88 18.3 9.17 12 2.88 5.7 4.29 4.29l6.3 6.3 6.29-6.3z"/></svg>
        </button>
      </div>
    </div>
  </div>`;
  const videoEl = overlay.querySelector("video");
  const toastEl = overlay.querySelector(".dashcam-player-toast");
  const downloadUrl = src + (src.includes("?") ? "&" : "?") + "download=1";
  let toastTimer = null;
  let suppressToasts = true;
  const showToast = (text) => {
    if (!toastEl || suppressToasts || !text) return;
    toastEl.textContent = text;
    toastEl.classList.add("is-visible");
    if (toastTimer) window.clearTimeout(toastTimer);
    toastTimer = window.setTimeout(() => toastEl.classList.remove("is-visible"), 850);
  };
  let player = null;
  const close = () => {
    if (toastTimer) window.clearTimeout(toastTimer);
    try { player?.destroy?.(); } catch {}
    overlay.remove();
  };
  overlay.addEventListener("click", (ev) => {
    if (ev.target === overlay) close();
  });
  overlay.querySelector(".dashcam-player-close")?.addEventListener("click", close);
  document.body.appendChild(overlay);
  requestAnimationFrame(() => {
    overlay.classList.add("is-open");
    try {
      player = new Plyr(videoEl, {
        controls: ["play-large","rewind","play","fast-forward","progress","current-time","fullscreen","download"],
        hideControls: false,
        seekTime: 5,
        keyboard: { focused: true, global: false },
        fullscreen: { enabled: true, fallback: true, iosNative: true },
        urls: { download: downloadUrl },
      });
      player.source = {
        type: "video",
        title: title || "Video",
        sources: [{ src, type: "video/mp4" }],
      };
      player.once("ready", () => {
        const container = player.elements?.container || overlay;
        const bindBtn = (sel, label) => {
          container.querySelectorAll(sel).forEach((btn) => btn.addEventListener("click", () => showToast(label)));
        };
        bindBtn('[data-plyr="rewind"]', `⏪ ${getUIText("rewind_5", "5s")}`);
        bindBtn('[data-plyr="fast-forward"]', `${getUIText("forward_5", "5s")} ⏩`);
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
      videoEl.controls = true;
      videoEl.src = src;
    }
  });
}

function activateLogsTab(tab, options = {}) {
  const nextTab = tab === "screen" ? "screen" : "dashcam";
  const shouldLoad = options.load !== false;
  if (nextTab !== logsActiveTab) saveLogsScrollTop(logsActiveTab);
  logsActiveTab = nextTab;
  const dashTab = document.getElementById("logsTabDashcam");
  const screenTab = document.getElementById("logsTabScreen");
  const dashPanel = document.getElementById("logsDashcamPanel");
  const screenPanel = document.getElementById("logsScreenPanel");

  dashTab?.classList.toggle("is-active", logsActiveTab === "dashcam");
  screenTab?.classList.toggle("is-active", logsActiveTab === "screen");
  dashTab?.setAttribute("aria-selected", logsActiveTab === "dashcam" ? "true" : "false");
  screenTab?.setAttribute("aria-selected", logsActiveTab === "screen" ? "true" : "false");
  if (dashPanel) dashPanel.hidden = logsActiveTab !== "dashcam";
  if (screenPanel) screenPanel.hidden = logsActiveTab !== "screen";

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

function handleLogsPageChange(event) {
  const page = event?.detail?.page || "";
  if (page === "logs") return;
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
  const dashTab = document.getElementById("logsTabDashcam");
  const screenTab = document.getElementById("logsTabScreen");
  const routesHost = document.getElementById("dashcamRoutes");
  const screenHost = document.getElementById("screenrecordVideos");

  if (!dashcamState.layoutBound) {
    dashcamState.layoutBound = true;
    dashcamState.landscape = isCompactLandscapeMode();
    dashcamState.layoutKey = dashcamLayoutKey();
    window.addEventListener("carrot:pagechange", handleLogsPageChange);
    window.addEventListener("carrot:languagechange", () => {
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

  if (dashTab && dashTab.dataset.bound !== "1") {
    dashTab.dataset.bound = "1";
    dashTab.addEventListener("click", () => activateLogsTab("dashcam"));
  }

  if (screenTab && screenTab.dataset.bound !== "1") {
    screenTab.dataset.bound = "1";
    screenTab.addEventListener("click", () => activateLogsTab("screen"));
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
