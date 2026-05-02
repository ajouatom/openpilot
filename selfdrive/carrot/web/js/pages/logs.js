"use strict";

// Logs page — Dashcam (route+segment listing, FFmpeg thumb/preview, FTP upload)
// + Screen Recording listing/playback. Tab switching between the two.

const DASHCAM_UPLOAD_JOB_STORAGE_KEY = "carrot_dashcam_upload_job_id";
const DASHCAM_ROUTE_INITIAL_BATCH = 6;
const DASHCAM_ROUTE_BATCH_SIZE = 8;
let dashcamUploadActiveJobId = null;
let dashcamUploadResumePromise = null;
let dashcamRouteRenderToken = 0;
let dashcamRouteRenderIdleId = null;

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
    });
  });
}

function dashcamSegmentIndex(segment) {
  const parts = String(segment || "").split("--");
  const n = Number.parseInt(parts[parts.length - 1] || "0", 10);
  return Number.isFinite(n) ? n : 0;
}

function dashcamRouteTitle(route) {
  return String(route || "").replace(/^0+(?=\d{3})/, "");
}

function dashcamApiPath(kind, segment) {
  return `/api/dashcam/${kind}/${encodeURIComponent(segment)}`;
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

function setDashcamStatus(message, tone = "") {
  const status = document.getElementById("dashcamStatus");
  if (!status) return;
  status.textContent = message || "";
  status.hidden = !message;
  status.classList.toggle("is-error", tone === "error");
}

function setDashcamMeta(message) {
  const meta = document.getElementById("dashcamMeta");
  if (meta) meta.textContent = message;
}

function setScreenrecordStatus(message, tone = "") {
  const status = document.getElementById("screenrecordStatus");
  if (!status) return;
  status.textContent = message || "";
  status.hidden = !message;
  status.classList.toggle("is-error", tone === "error");
}

function formatLogBytes(bytes) {
  const n = Number(bytes) || 0;
  if (n < 1024) return `${n} B`;
  if (n < 1024 * 1024) return `${(n / 1024).toFixed(1)} KB`;
  if (n < 1024 * 1024 * 1024) return `${(n / (1024 * 1024)).toFixed(1)} MB`;
  return `${(n / (1024 * 1024 * 1024)).toFixed(1)} GB`;
}

function screenrecordApiPath(kind, fileId) {
  return `/api/screenrecord/${kind}/${encodeURIComponent(fileId)}`;
}

function dashcamRoutesSignature(routes) {
  return (routes || []).map((entry) => [
    entry.route || "",
    entry.latestModifiedLabel || "",
    ...(entry.segmentFolders || []),
  ].join("|")).join("\n") + "|" + (typeof LANG !== "undefined" ? LANG : "");
}

function screenrecordVideosSignature(videos) {
  return (videos || []).map((video) => [
    video.id || "",
    video.name || "",
    video.modifiedLabel || video.relativeModifiedLabel || "",
    video.size || 0,
  ].join("|")).join("\n") + "|" + (typeof LANG !== "undefined" ? LANG : "");
}

function hasCompleteDashcamDom(host, routes) {
  if (!host || !Array.isArray(routes) || !routes.length) return false;
  const renderedCount = Number.parseInt(host.dataset.renderCount || "0", 10);
  return host.dataset.signature === dashcamState.signature
    && renderedCount >= routes.length
    && host.querySelectorAll("[data-route-card]").length >= routes.length;
}

function hasCompleteScreenrecordDom(host, videos) {
  if (!host || !Array.isArray(videos) || !videos.length) return false;
  const renderedCount = Number.parseInt(host.dataset.renderCount || "0", 10);
  return host.dataset.signature === screenrecordState.signature
    && renderedCount >= videos.length
    && host.querySelectorAll(".screenrecord-row").length >= videos.length;
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

function cancelDashcamRouteRender() {
  dashcamRouteRenderToken += 1;
  if (dashcamRouteRenderIdleId != null) {
    if (typeof window.cancelIdleCallback === "function") window.cancelIdleCallback(dashcamRouteRenderIdleId);
    else window.clearTimeout(dashcamRouteRenderIdleId);
    dashcamRouteRenderIdleId = null;
  }
}

function appendDashcamRouteBatch(host, routes, options) {
  const token = options.token;
  if (token !== dashcamRouteRenderToken || !isLogsPageActive() || !host || !Array.isArray(routes)) return;

  const start = options.start || 0;
  const batchSize = start === 0 ? DASHCAM_ROUTE_INITIAL_BATCH : DASHCAM_ROUTE_BATCH_SIZE;
  const end = Math.min(start + batchSize, routes.length);
  const template = document.createElement("template");
  template.innerHTML = routes
    .slice(start, end)
    .map((entry, offset) => dashcamRouteCardHtml(entry, start + offset, {
      animate: options.animate,
      animateIndex: offset,
    }))
    .join("");
  const nodes = Array.from(template.content.children);
  host.appendChild(template.content);
  nodes.forEach((node) => hydrateLogsLazyImages(node));
  host.dataset.renderCount = String(end);

  if (end >= routes.length) {
    host.dataset.signature = dashcamState.signature || dashcamRoutesSignature(routes);
    return;
  }
  dashcamRouteRenderIdleId = requestIdleTask(() => {
    dashcamRouteRenderIdleId = null;
    appendDashcamRouteBatch(host, routes, {
      ...options,
      start: end,
    });
  }, 180);
}

function dashcamSelectedForRoute(entry) {
  return (entry.segmentFolders || []).filter((segment) => dashcamState.selected.has(segment));
}

function dashcamRouteCardHtml(entry, index = 0, options = {}) {
  const animate = options.animate !== false;
  const animateIndex = Number.isFinite(options.animateIndex) ? options.animateIndex : index;
  const route = String(entry.route || "");
  const segments = Array.isArray(entry.segmentFolders) ? entry.segmentFolders : [];
  const expanded = dashcamState.expanded.has(route);
  const compactSegments = isCompactLandscapeMode();
  const shouldRenderSegments = expanded || compactSegments;
  const selected = dashcamSelectedForRoute(entry);
  const allSelected = segments.length > 0 && selected.length === segments.length;
  const representative = segments[0] || "";
  const routeAttr = escapeHtml(route);
  const title = escapeHtml(entry.title || dashcamRouteTitle(route));
  const dateLabel = escapeHtml(entry.dateLabel || route);
  const latest = escapeHtml(formatRelativeEpoch(entry.latestModifiedEpoch) || localizeRelativeLabel(entry.latestModifiedLabel) || "-");
  const preview = representative
    ? `<div class="dashcam-route-media">
        <div class="dashcam-route-preview" data-action="play" data-route="${routeAttr}" data-segment="${escapeHtml(representative)}">
          <img class="logs-lazy-img" loading="lazy" decoding="async" fetchpriority="low" data-src="${dashcamApiPath("preview", representative)}" data-fallback="${dashcamApiPath("thumbnail", representative)}" onerror="this.onerror=null;if(this.dataset.fallback)this.src=this.dataset.fallback;" alt="">
          <div class="dashcam-route-preview__shade"></div>
          <div class="dashcam-route-preview__chips">
            <span class="dashcam-chip">${escapeHtml(getUIText("segment_count", "{count} segments", { count: segments.length }))}</span>
            <span class="dashcam-chip">${latest}</span>
          </div>
          <div class="dashcam-play-mark" aria-hidden="true">
            <svg viewBox="0 0 24 24"><path fill="currentColor" d="M8 5v14l11-7z"/></svg>
          </div>
        </div>
        <div class="dashcam-route-media-info" data-action="toggle-route" data-route="${routeAttr}">
          <div class="dashcam-route-title">${title}</div>
          <div class="dashcam-route-subtitle">${dateLabel}</div>
        </div>
      </div>`
    : "";
  const segmentList = shouldRenderSegments ? segments.map((segment, segmentIndex) => {
    const segAttr = escapeHtml(segment);
    const checked = dashcamState.selected.has(segment) ? " checked" : "";
    if (compactSegments) {
      return `<div class="dashcam-segment-tile dashcam-segment-tile--compact ui-stagger-item" style="--i:${segmentIndex}" data-action="play" data-route="${routeAttr}" data-segment="${segAttr}">
        <div class="dashcam-segment-thumb dashcam-segment-thumb--compact">
          <img class="logs-lazy-img" loading="lazy" decoding="async" fetchpriority="low" data-src="${dashcamApiPath("thumbnail", segment)}" alt="">
          <label class="dashcam-segment-check dashcam-segment-check--compact" title="${escapeHtml(getUIText("select_all", "Select"))}" onclick="event.stopPropagation()">
            <input type="checkbox" data-action="select-segment" data-segment="${segAttr}"${checked}>
          </label>
        </div>
        <div class="dashcam-segment-body">
          <div class="dashcam-segment-badge">SEG ${dashcamSegmentIndex(segment)}</div>
          <div class="dashcam-segment-name">${segAttr}</div>
        </div>
        <button class="dashcam-menu-btn" type="button" data-action="segment-menu" data-route="${routeAttr}" data-segment="${segAttr}" aria-label="${escapeHtml(getUIText("segment_menu", "Segment menu"))}" title="${escapeHtml(getUIText("segment_menu", "Segment menu"))}">
          <svg viewBox="0 0 24 24"><path fill="currentColor" d="M12 8a2 2 0 1 0 0-4 2 2 0 0 0 0 4m0 2a2 2 0 1 0 0 4 2 2 0 0 0 0-4m0 6a2 2 0 1 0 0 4 2 2 0 0 0 0-4"/></svg>
        </button>
      </div>`;
    }
    return `<div class="dashcam-segment-tile ui-stagger-item" style="--i:${segmentIndex}" data-action="play" data-route="${routeAttr}" data-segment="${segAttr}">
      <div class="dashcam-segment-thumb">
        <img class="logs-lazy-img" loading="lazy" decoding="async" fetchpriority="low" data-src="${dashcamApiPath("thumbnail", segment)}" alt="">
        <label class="dashcam-segment-check" title="${escapeHtml(getUIText("select_all", "Select"))}" onclick="event.stopPropagation()">
          <input type="checkbox" data-action="select-segment" data-segment="${segAttr}"${checked}>
        </label>
      </div>
      <div class="dashcam-segment-body">
        <div class="dashcam-segment-badge">SEG ${dashcamSegmentIndex(segment)}</div>
        <div class="dashcam-segment-name">${segAttr}</div>
      </div>
      <button class="dashcam-menu-btn" type="button" data-action="segment-menu" data-route="${routeAttr}" data-segment="${segAttr}" aria-label="${escapeHtml(getUIText("segment_menu", "Segment menu"))}" title="${escapeHtml(getUIText("segment_menu", "Segment menu"))}">
        <svg viewBox="0 0 24 24"><path fill="currentColor" d="M12 8a2 2 0 1 0 0-4 2 2 0 0 0 0 4m0 2a2 2 0 1 0 0 4 2 2 0 0 0 0-4m0 6a2 2 0 1 0 0 4 2 2 0 0 0 0-4"/></svg>
      </button>
    </div>`;
  }).join("") : "";

  return `<article class="dashcam-route-card${animate ? " ui-stagger-item" : ""}"${animate ? ` style="--i:${animateIndex}"` : ""} data-route-card="${routeAttr}">
    ${preview}
    <div class="dashcam-route-main">
      <div class="dashcam-route-head" data-action="toggle-route" data-route="${routeAttr}">
        <div class="dashcam-route-titleblock">
          <div class="dashcam-route-title">${title}</div>
          <div class="dashcam-route-subtitle">${dateLabel}</div>
        </div>
        <button class="dashcam-expand-btn" type="button" data-action="toggle-route" data-route="${routeAttr}" aria-expanded="${expanded ? "true" : "false"}" title="${escapeHtml(expanded ? getUIText("collapse", "Collapse") : getUIText("show_segments", "Show segments"))}">
          <svg viewBox="0 0 24 24"><path fill="currentColor" d="${expanded ? "M7.41 15.41 12 10.83l4.59 4.58L18 14l-6-6-6 6z" : "M7.41 8.59 12 13.17l4.59-4.58L18 10l-6 6-6-6z"}"/></svg>
        </button>
      </div>
      <div class="dashcam-segments ${expanded ? "" : "is-collapsed"}">
        <div class="dashcam-selection-row">
          <span class="dashcam-selection-count">${escapeHtml(getUIText("selected_count", "{count} selected", { count: selected.length }))}</span>
          <button class="smallBtn" type="button" data-action="select-route" data-route="${routeAttr}" data-selected="${allSelected ? "1" : "0"}">${escapeHtml(allSelected ? getUIText("deselect_all", "Deselect all") : getUIText("select_all", "Select all"))}</button>
          <button class="smallBtn btn--filled" type="button" data-action="upload-selected" data-route="${routeAttr}" ${selected.length ? "" : "disabled"}>${escapeHtml(getUIText("upload_selected", "Upload selected"))}</button>
        </div>
        <div class="dashcam-segment-list">${segmentList}</div>
      </div>
    </div>
  </article>`;
}

function renderDashcamRoutes(options = {}) {
  const host = document.getElementById("dashcamRoutes");
  if (!host) return;
  const animate = options.animate !== false;
  const preserve = options.preserve === true;
  const routes = dashcamState.routes || [];
  cancelDashcamRouteRender();
  if (!isLogsPageActive()) return;
  if (dashcamState.loading && !routes.length) {
    setDashcamStatus(getUIText("loading", "Loading..."));
    host.innerHTML = "";
    host.dataset.signature = "";
    host.dataset.renderCount = "0";
    return;
  }
  if (!routes.length) {
    host.innerHTML = logsEmptyStateHtml("dashcam");
    host.dataset.signature = "";
    host.dataset.renderCount = "0";
    setDashcamStatus("");
    return;
  }
  setDashcamStatus("");
  if (preserve && hasCompleteDashcamDom(host, routes)) {
    hydrateLogsLazyImages(host);
    return;
  }
  host.innerHTML = "";
  host.dataset.signature = "";
  host.dataset.renderCount = "0";
  appendDashcamRouteBatch(host, routes, {
    animate,
    start: 0,
    token: dashcamRouteRenderToken,
  });
}

function renderDashcamRoute(route) {
  const host = document.getElementById("dashcamRoutes");
  if (!host) return false;
  const routes = dashcamState.routes || [];
  const index = routes.findIndex((entry) => entry.route === route);
  if (index < 0) return false;

  const current = Array.from(host.querySelectorAll("[data-route-card]"))
    .find((node) => node.dataset.routeCard === route);
  if (!current) return false;

  const tpl = document.createElement("template");
  tpl.innerHTML = dashcamRouteCardHtml(routes[index], index, { animate: false });
  const nextMain = tpl.content.querySelector(".dashcam-route-main");
  const currentMain = current.querySelector(".dashcam-route-main");
  if (!nextMain || !currentMain) return false;

  currentMain.replaceWith(nextMain);
  hydrateLogsLazyImages(nextMain);
  return true;
}

function updateDashcamRouteSelectionUi(route) {
  const host = document.getElementById("dashcamRoutes");
  if (!host) return false;
  const entry = (dashcamState.routes || []).find((item) => item.route === route);
  if (!entry) return false;

  const card = Array.from(host.querySelectorAll("[data-route-card]"))
    .find((node) => node.dataset.routeCard === route);
  if (!card) return false;

  const segments = Array.isArray(entry.segmentFolders) ? entry.segmentFolders : [];
  const selected = dashcamSelectedForRoute(entry);
  const allSelected = segments.length > 0 && selected.length === segments.length;

  const countEl = card.querySelector(".dashcam-selection-count");
  if (countEl) countEl.textContent = getUIText("selected_count", "{count} selected", { count: selected.length });

  const selectBtn = card.querySelector('[data-action="select-route"]');
  if (selectBtn) {
    selectBtn.dataset.selected = allSelected ? "1" : "0";
    selectBtn.textContent = allSelected ? getUIText("deselect_all", "Deselect all") : getUIText("select_all", "Select all");
  }

  const uploadBtn = card.querySelector('[data-action="upload-selected"]');
  if (uploadBtn) uploadBtn.disabled = selected.length === 0;

  card.querySelectorAll('input[data-action="select-segment"]').forEach((input) => {
    const segment = input.dataset.segment || "";
    input.checked = dashcamState.selected.has(segment);
  });

  return true;
}

async function loadDashcamRoutes({ silent = false } = {}) {
  const seq = ++dashcamState.loadSeq;
  if (!silent) {
    dashcamState.loading = true;
    renderDashcamRoutes();
  }
  try {
    const json = await getJson("/api/dashcam/routes");
    if (seq !== dashcamState.loadSeq) return;
    if (!isLogsPageActive()) {
      dashcamState.loading = false;
      return;
    }
    const routes = Array.isArray(json.routes) ? json.routes : [];
    const nextSignature = dashcamRoutesSignature(routes);
    if (silent && nextSignature === dashcamState.signature) {
      dashcamState.loading = false;
      return;
    }
    const validRoutes = new Set(routes.map((entry) => entry.route));
    const validSegments = new Set(routes.flatMap((entry) => entry.segmentFolders || []));
    dashcamState.expanded = new Set(Array.from(dashcamState.expanded).filter((route) => validRoutes.has(route)));
    dashcamState.selected = new Set(Array.from(dashcamState.selected).filter((segment) => validSegments.has(segment)));
    dashcamState.routes = routes;
    dashcamState.signature = nextSignature;
    dashcamState.loading = false;
    renderDashcamRoutes({ animate: !silent });
    if (!silent && logsScrollTops.dashcam === 0) restoreLogsScrollTop("dashcam", { reset: true });
  } catch (e) {
    if (seq !== dashcamState.loadSeq) return;
    dashcamState.loading = false;
    if (!silent && isLogsPageActive()) {
      setDashcamStatus(`${getUIText("dashcam_load_failed", "Failed to load dashcam list")}: ${e.message || e}`, "error");
      showAppToast(e.message || getUIText("dashcam_load_failed", "Failed to load dashcam list"), { tone: "error" });
    }
  }
}

function startDashcamAutoRefresh() {
  if (dashcamState.refreshTimer) return;
  dashcamState.refreshTimer = window.setInterval(() => {
    if (CURRENT_PAGE !== "logs" || dashcamState.scrollBusy) return;
    if (logsActiveTab === "screen") loadScreenrecordVideos({ silent: true }).catch(() => {});
    else loadDashcamRoutes({ silent: true }).catch(() => {});
  }, 10000);
}

function markDashcamScrollBusy() {
  dashcamState.scrollBusy = true;
  if (dashcamState.scrollTimer) window.clearTimeout(dashcamState.scrollTimer);
  dashcamState.scrollTimer = window.setTimeout(() => {
    dashcamState.scrollBusy = false;
  }, 380);
}

function openLogsVideoPlayer(title, src, options = {}) {
  const overlay = document.createElement("div");
  const kind = String(options.kind || "video").replace(/[^a-z0-9_-]/gi, "");
  overlay.className = `dashcam-player-overlay dashcam-player-overlay--${kind}`;
  overlay.innerHTML = `<div class="dashcam-player-dialog" role="dialog" aria-modal="true">
    <div class="dashcam-player-frame">
      <video class="dashcam-player-video" autoplay controls playsinline src="${src}"></video>
      <div class="dashcam-player-controls" aria-label="${escapeHtml(getUIText("video_controls", "Video controls"))}">
        <button class="dashcam-player-control" type="button" data-skip="-5" aria-label="${escapeHtml(getUIText("rewind_5", "Back 5 seconds"))}" title="${escapeHtml(getUIText("rewind_5", "Back 5 seconds"))}">
          <span aria-hidden="true">-5</span>
        </button>
        <button class="dashcam-player-control" type="button" data-skip="5" aria-label="${escapeHtml(getUIText("forward_5", "Forward 5 seconds"))}" title="${escapeHtml(getUIText("forward_5", "Forward 5 seconds"))}">
          <span aria-hidden="true">+5</span>
        </button>
      </div>
      <div class="dashcam-player-top">
        <div class="dashcam-player-title">${escapeHtml(title || "Video")}</div>
        <button class="dashcam-player-close" type="button" aria-label="${escapeHtml(getUIText("close", "Close"))}" title="${escapeHtml(getUIText("close", "Close"))}">
          <svg viewBox="0 0 24 24"><path fill="currentColor" d="M18.3 5.71 12 12l6.3 6.29-1.41 1.41L10.59 13.41 4.29 19.71 2.88 18.3 9.17 12 2.88 5.7 4.29 4.29l6.3 6.3 6.29-6.3z"/></svg>
        </button>
      </div>
    </div>
  </div>`;
  const close = () => {
    const video = overlay.querySelector("video");
    clearHideControlsTimer();
    try { video?.pause?.(); } catch {}
    overlay.remove();
  };
  overlay.addEventListener("click", (ev) => {
    if (ev.target === overlay) close();
  });
  overlay.querySelector(".dashcam-player-close")?.addEventListener("click", close);
  const video = overlay.querySelector("video");
  const frame = overlay.querySelector(".dashcam-player-frame");
  let hideControlsTimer = null;
  if (video) video.controls = true;
  const clearHideControlsTimer = () => {
    if (!hideControlsTimer) return;
    window.clearTimeout(hideControlsTimer);
    hideControlsTimer = null;
  };
  const scheduleHidePlayerControls = () => {
    clearHideControlsTimer();
    if (!video || video.paused || video.ended) return;
    hideControlsTimer = window.setTimeout(() => {
      if (!video || video.paused || video.ended) return;
      overlay.classList.add("is-player-controls-hidden");
    }, 2200);
  };
  const showPlayerControls = () => {
    overlay.classList.remove("is-player-controls-hidden");
    scheduleHidePlayerControls();
  };
  const seekVideo = (delta) => {
    if (!video) return;
    const duration = Number.isFinite(video.duration) ? video.duration : Infinity;
    const current = Number.isFinite(video.currentTime) ? video.currentTime : 0;
    video.currentTime = Math.max(0, Math.min(duration, current + delta));
  };
  const isPlayerControlTarget = (target) => {
    if (!(target instanceof Element)) return false;
    return Boolean(target.closest("button, .dashcam-player-top, .dashcam-player-controls"));
  };
  const syncPlayerControlsVisibility = () => {
    if (!video) return;
    const paused = video.paused || video.ended;
    if (paused) {
      clearHideControlsTimer();
      overlay.classList.remove("is-player-controls-hidden");
    } else {
      scheduleHidePlayerControls();
    }
  };
  video?.addEventListener("play", syncPlayerControlsVisibility);
  video?.addEventListener("pause", syncPlayerControlsVisibility);
  video?.addEventListener("ended", syncPlayerControlsVisibility);
  overlay.querySelectorAll("[data-skip]").forEach((button) => {
    button.addEventListener("click", () => {
      const delta = Number(button.dataset.skip || 0);
      seekVideo(delta);
      showPlayerControls();
    });
  });
  frame?.addEventListener("mousemove", showPlayerControls);
  frame?.addEventListener("touchstart", showPlayerControls, { passive: true });
  frame?.addEventListener("click", (ev) => {
    if (isPlayerControlTarget(ev.target)) return;
    showPlayerControls();
  });
  frame?.addEventListener("dblclick", (ev) => {
    if (isPlayerControlTarget(ev.target)) return;
    const rect = frame.getBoundingClientRect();
    const x = ev.clientX - rect.left;
    seekVideo(x < rect.width / 2 ? -5 : 5);
    showPlayerControls();
  });
  let lastPlayerTap = { time: 0, x: 0, y: 0 };
  frame?.addEventListener("touchend", (ev) => {
    if (isPlayerControlTarget(ev.target)) return;
    const touch = ev.changedTouches?.[0];
    if (!touch) return;
    const now = performance.now();
    const dx = touch.clientX - lastPlayerTap.x;
    const dy = touch.clientY - lastPlayerTap.y;
    const isDoubleTap = now - lastPlayerTap.time < 320 && Math.hypot(dx, dy) < 34;
    if (isDoubleTap) {
      ev.preventDefault();
      const rect = frame.getBoundingClientRect();
      const x = touch.clientX - rect.left;
      seekVideo(x < rect.width / 2 ? -5 : 5);
      showPlayerControls();
      lastPlayerTap = { time: 0, x: 0, y: 0 };
      return;
    }
    lastPlayerTap = { time: now, x: touch.clientX, y: touch.clientY };
  }, { passive: false });
  document.body.appendChild(overlay);
  requestAnimationFrame(() => {
    overlay.classList.add("is-open");
    syncPlayerControlsVisibility();
    showPlayerControls();
  });
}

function openDashcamPlayer(route, segment) {
  openLogsVideoPlayer(
    `${dashcamRouteTitle(route)} · Segment ${dashcamSegmentIndex(segment)}`,
    dashcamApiPath("video", segment),
    { kind: "dashcam" },
  );
}

function openScreenrecordPlayer(id, name) {
  if (!id) return;
  openLogsVideoPlayer(name || getUIText("logs_screenrecord", "Screen Record"), screenrecordApiPath("video", id), { kind: "screenrecord" });
}

function dashcamUploadStats(items) {
  const list = Array.isArray(items) ? items : [];
  return list.reduce((stats, item) => {
    const files = Array.isArray(item?.files) ? item.files : [];
    const totalSize = Number(item?.totalSize) || files.reduce((sum, file) => sum + (Number(file?.size) || 0), 0);
    stats.segments += 1;
    stats.files += files.length;
    stats.bytes += totalSize;
    return stats;
  }, { segments: 0, files: 0, bytes: 0 });
}

function dashcamUploadSummaryLabel(stats) {
  const fileCount = Number(stats?.files || 0);
  const bytes = Number(stats?.bytes || 0);
  const fileLabel = fileCount > 0
    ? getUIText("upload_file_count", "{count} files", { count: fileCount })
    : getUIText("upload_files_unknown", "files unknown");
  const sizeLabel = bytes > 0 ? formatLogBytes(bytes) : getUIText("upload_size_unknown", "size unknown");
  return `${fileLabel} · ${sizeLabel}`;
}

function dashcamUploadResultHtml(result) {
  const text = String(result?.shareText || result?.message || "");
  const stats = dashcamUploadStats(result?.results || []);
  return `<div class="dashcam-share-card">
    <div class="dashcam-share-card__summary">
      <span>${escapeHtml(getUIText("upload_count", "Upload {uploaded}/{total}", { uploaded: Number(result?.uploaded || 0), total: Number(result?.total || 0) }))}</span>
      <span>${escapeHtml(dashcamUploadSummaryLabel(stats))}</span>
    </div>
    <pre>${escapeHtml(text)}</pre>
  </div>`;
}

async function showDashcamUploadResult(result) {
  const text = String(result?.shareText || result?.message || "").trim();
  await openAppDialog({
    mode: "choice",
    title: getUIText("log_upload_result", "Upload Result"),
    html: true,
    messageHtml: `<div class="dashcam-share-dialog">${dashcamUploadResultHtml(result)}</div>`,
    cancelLabel: getUIText("close", "Close"),
    copyText: text,
    copyLabel: getUIText("copy", "Copy"),
  });
}

function openDashcamUploadProgress(total, stats = null) {
  const overlay = document.createElement("div");
  overlay.className = "dashcam-upload-progress";
  overlay.innerHTML = `<div class="dashcam-upload-progress__sheet" role="dialog" aria-modal="true">
    <div class="dashcam-upload-progress__title">${escapeHtml(getUIText("log_uploading", "Uploading logs"))}</div>
    <div class="dashcam-upload-progress__message">0/${Number(total || 0)}</div>
    <div class="dashcam-upload-progress__bar" aria-hidden="true"><span></span></div>
    <div class="dashcam-upload-progress__summary">${escapeHtml(stats ? dashcamUploadSummaryLabel(stats) : getUIText("loading", "Loading..."))}</div>
  </div>`;
  document.body.appendChild(overlay);
  document.body.classList.add("dialog-open");
  requestAnimationFrame(() => overlay.classList.add("is-open"));
  const message = overlay.querySelector(".dashcam-upload-progress__message");
  const summary = overlay.querySelector(".dashcam-upload-progress__summary");
  const bar = overlay.querySelector(".dashcam-upload-progress__bar span");
  return {
    setMessage(text) {
      if (message) message.textContent = text || "";
    },
    setProgress(percent) {
      if (!bar) return;
      const value = Number(percent);
      if (!Number.isFinite(value) || value <= 0) {
        bar.style.animation = "";
        bar.style.transform = "";
        bar.style.width = "";
        return;
      }
      bar.style.animation = "none";
      bar.style.transform = "none";
      bar.style.width = `${Math.max(4, Math.min(100, value))}%`;
    },
    setSummary(nextStats) {
      if (summary) summary.textContent = nextStats ? dashcamUploadSummaryLabel(nextStats) : "";
    },
    close() {
      overlay.classList.remove("is-open");
      window.setTimeout(() => {
        overlay.remove();
        syncModalBodyLock();
      }, 160);
    },
  };
}

function rememberDashcamUploadJob(jobId) {
  dashcamUploadActiveJobId = jobId || null;
  try {
    if (jobId) localStorage.setItem(DASHCAM_UPLOAD_JOB_STORAGE_KEY, jobId);
  } catch {}
}

function clearRememberedDashcamUploadJob(jobId = null) {
  if (!jobId || dashcamUploadActiveJobId === jobId) dashcamUploadActiveJobId = null;
  try {
    const saved = localStorage.getItem(DASHCAM_UPLOAD_JOB_STORAGE_KEY);
    if (!jobId || saved === jobId) localStorage.removeItem(DASHCAM_UPLOAD_JOB_STORAGE_KEY);
  } catch {}
}

function getRememberedDashcamUploadJob() {
  try {
    return localStorage.getItem(DASHCAM_UPLOAD_JOB_STORAGE_KEY) || "";
  } catch {
    return "";
  }
}

async function pollDashcamUploadJob(jobId, progress, totalFallback = 0) {
  let snapshot = null;
  while (jobId) {
    snapshot = await getJson(`/api/dashcam/upload/job?id=${encodeURIComponent(jobId)}`);
    const current = Number(snapshot.step_current || 0);
    const total = Number(snapshot.step_total || totalFallback || 0);
    const percent = Number(snapshot.progress);
    const message = snapshot.message || getUIText("log_uploading", "Uploading logs");
    progress.setMessage(`${current}/${total || totalFallback || 0} · ${message}`);
    progress.setProgress(percent);
    if (snapshot.done) break;
    await waitMs(850);
  }

  const result = snapshot?.result || {};
  if (!result || !Array.isArray(result.results)) {
    throw new Error(snapshot?.error || getUIText("error", "Error"));
  }
  return result;
}

async function resumeDashcamUploadJobIfNeeded() {
  if (dashcamUploadResumePromise) return dashcamUploadResumePromise;
  const jobId = getRememberedDashcamUploadJob();
  if (!jobId || jobId === dashcamUploadActiveJobId) return null;

  dashcamUploadResumePromise = (async () => {
    let snapshot = null;
    try {
      snapshot = await getJson(`/api/dashcam/upload/job?id=${encodeURIComponent(jobId)}`);
    } catch {
      clearRememberedDashcamUploadJob(jobId);
      dashcamUploadResumePromise = null;
      return null;
    }

    if (snapshot.done) {
      clearRememberedDashcamUploadJob(jobId);
      if (snapshot.result?.results) await showDashcamUploadResult(snapshot.result);
      dashcamUploadResumePromise = null;
      return snapshot.result || null;
    }

    rememberDashcamUploadJob(jobId);
    const total = Number(snapshot.step_total || 0);
    const progress = openDashcamUploadProgress(total, null);
    let activityId = typeof beginAppActivity === "function"
      ? beginAppActivity("logs", getUIText("log_uploading", "Uploading logs"))
      : null;

    try {
      progress.setMessage(`${Number(snapshot.step_current || 0)}/${total} · ${snapshot.message || getUIText("log_uploading", "Uploading logs")}`);
      progress.setProgress(Number(snapshot.progress));
      const result = await pollDashcamUploadJob(jobId, progress, total);
      clearRememberedDashcamUploadJob(jobId);
      progress.setMessage(`${Number(result.uploaded || 0)}/${Number(result.total || total)}`);
      progress.setProgress(100);
      progress.setSummary(dashcamUploadStats(result.results || []));
      showAppToast(result.message || getUIText("upload_complete_count", "Upload complete {uploaded}/{total}", {
        uploaded: result.uploaded || 0,
        total: result.total || total,
      }), { tone: result.ok ? "default" : "error", duration: 3600 });
      progress.close();
      if (activityId && typeof endAppActivity === "function") {
        endAppActivity(activityId);
        activityId = null;
      }
      await showDashcamUploadResult(result);
      return result;
    } catch (e) {
      progress.close();
      showAppToast(`${getUIText("log_upload", "Upload Logs")} ${getUIText("error", "Error")}: ${e.message || e}`, { tone: "error", duration: 4200 });
      return null;
    } finally {
      if (activityId && typeof endAppActivity === "function") endAppActivity(activityId);
      if (dashcamUploadActiveJobId === jobId) dashcamUploadActiveJobId = null;
      dashcamUploadResumePromise = null;
    }
  })();

  return dashcamUploadResumePromise;
}

async function uploadDashcamSegments(segments) {
  const targets = Array.from(new Set(segments || [])).filter(Boolean);
  if (!targets.length) {
    showAppToast(getUIText("no_selected_segments", "No segments selected."), { tone: "error" });
    return;
  }
  let uploadStats = { segments: targets.length, files: 0, bytes: 0 };
  try {
    const summary = await postJson("/api/dashcam/upload/summary", { segments: targets });
    if (Array.isArray(summary?.summaries)) uploadStats = dashcamUploadStats(summary.summaries);
  } catch {}
  const confirmMessage = [
    getUIText("log_upload_confirm", `Upload ${targets.length} logs to the Carrot server?`, { count: targets.length }),
    dashcamUploadSummaryLabel(uploadStats),
    getUIText("upload_data_warning", "This upload may use mobile data depending on your network connection."),
  ].join("\n\n");
  const ok = await appConfirm(confirmMessage, { title: getUIText("log_upload", "Upload Logs") });
  if (!ok) return;
  const progress = openDashcamUploadProgress(targets.length, uploadStats);
  let activityId = typeof beginAppActivity === "function"
    ? beginAppActivity("logs", getUIText("log_uploading", "Uploading logs"))
    : null;
  let jobId = null;
  try {
    progress.setMessage(`0/${targets.length} · ${getUIText("log_uploading", "Uploading logs")}`);
    const started = await postJson("/api/dashcam/upload/start", { segments: targets });
    jobId = started.job_id;
    rememberDashcamUploadJob(jobId);
    const result = await pollDashcamUploadJob(jobId, progress, targets.length);
    clearRememberedDashcamUploadJob(jobId);
    progress.setMessage(`${Number(result.uploaded || 0)}/${Number(result.total || targets.length)}`);
    progress.setProgress(100);
    progress.setSummary(dashcamUploadStats(result.results || []));
    const message = result.message || getUIText("upload_complete_count", "Upload complete {uploaded}/{total}", {
      uploaded: result.uploaded || 0,
      total: result.total || targets.length,
    });
    showAppToast(message, { tone: result.ok ? "default" : "error", duration: 3600 });
    progress.close();
    if (activityId && typeof endAppActivity === "function") {
      endAppActivity(activityId);
      activityId = null;
    }
    await showDashcamUploadResult(result);
  } catch (e) {
    progress.close();
    showAppToast(`${getUIText("log_upload", "Upload Logs")} ${getUIText("error", "Error")}: ${e.message || e}`, { tone: "error", duration: 4200 });
  } finally {
    if (activityId && typeof endAppActivity === "function") endAppActivity(activityId);
    if (jobId && dashcamUploadActiveJobId === jobId) dashcamUploadActiveJobId = null;
  }
}

async function showDashcamSegmentMenu(route, segment) {
  const selected = await openAppDialog({
    mode: "choice",
    title: `SEG ${dashcamSegmentIndex(segment)}`,
    message: segment,
    choices: [
      { label: getUIText("play", "Play"), value: "play" },
      { label: getUIText("log_upload", "Upload Logs"), value: "upload" },
      { label: `qcamera ${getUIText("download", "Download")}`, value: "download_qcamera" },
      { label: `rlog ${getUIText("download", "Download")}`, value: "download_rlog" },
      { label: `qlog ${getUIText("download", "Download")}`, value: "download_qlog" },
    ],
  });
  if (selected === "play") openDashcamPlayer(route, segment);
  else if (selected === "upload") await uploadDashcamSegments([segment]);
  else if (selected?.startsWith?.("download_")) {
    const kind = selected.replace("download_", "");
    window.open(dashcamApiPath(`download/${encodeURIComponent(segment)}`, kind), "_blank", "noopener");
  }
}

function screenrecordVideoRowHtml(video, index = 0) {
  const id = escapeHtml(video.id || "");
  const name = escapeHtml(video.name || "-");
  const date = escapeHtml(formatRelativeEpoch(video.modifiedEpoch) || localizeRelativeLabel(video.modifiedLabel || video.relativeModifiedLabel) || "-");
  const size = escapeHtml(formatLogBytes(video.size));
  const ext = escapeHtml((video.ext || "video").toUpperCase());
  return `<article class="screenrecord-row ui-stagger-item" style="--i:${index}" data-action="play-screenrecord" data-id="${id}" data-name="${name}">
    <div class="screenrecord-row__thumb" aria-hidden="true">
      <img class="logs-lazy-img" loading="lazy" decoding="async" fetchpriority="low" data-src="${screenrecordApiPath("thumbnail", video.id || "")}" alt="">
    </div>
    <div class="screenrecord-row__main">
      <div class="screenrecord-row__name">${name}</div>
      <div class="screenrecord-row__meta">
        <span>${date}</span>
        <span>${size}</span>
        <span>${ext}</span>
      </div>
    </div>
    <button class="screenrecord-download" type="button" data-action="download-screenrecord" data-id="${id}" aria-label="${escapeHtml(getUIText("download", "Download"))}" title="${escapeHtml(getUIText("download", "Download"))}">
      <svg viewBox="0 0 24 24"><path fill="currentColor" d="M5 20h14v-2H5m14-9h-4V3H9v6H5l7 7z"/></svg>
    </button>
  </article>`;
}

function renderScreenrecordVideos(options = {}) {
  const host = document.getElementById("screenrecordVideos");
  if (!host) return;
  if (!isLogsPageActive()) return;
  const preserve = options.preserve === true;
  const videos = screenrecordState.videos || [];
  if (screenrecordState.loading && !videos.length) {
    setScreenrecordStatus("");
    host.innerHTML = logsLoadingSkeletonHtml("screen");
    host.dataset.signature = "";
    host.dataset.renderCount = "0";
    return;
  }
  if (!videos.length) {
    host.innerHTML = logsEmptyStateHtml("screen");
    host.dataset.signature = "";
    host.dataset.renderCount = "0";
    setScreenrecordStatus("");
    return;
  }
  setScreenrecordStatus("");
  if (preserve && hasCompleteScreenrecordDom(host, videos)) {
    hydrateLogsLazyImages(host);
    return;
  }
  host.innerHTML = videos.map(screenrecordVideoRowHtml).join("");
  host.dataset.signature = screenrecordState.signature || screenrecordVideosSignature(videos);
  host.dataset.renderCount = String(videos.length);
  hydrateLogsLazyImages(host);
}

async function loadScreenrecordVideos({ silent = false } = {}) {
  const seq = ++screenrecordState.loadSeq;
  if (!silent) {
    screenrecordState.loading = true;
    renderScreenrecordVideos();
  }
  try {
    const json = await getJson("/api/screenrecord/videos");
    if (seq !== screenrecordState.loadSeq) return;
    if (!isLogsPageActive()) {
      screenrecordState.loading = false;
      return;
    }
    const videos = Array.isArray(json.videos) ? json.videos : [];
    const nextSignature = screenrecordVideosSignature(videos);
    if (silent && nextSignature === screenrecordState.signature) {
      screenrecordState.loading = false;
      return;
    }
    screenrecordState.videos = videos;
    screenrecordState.signature = nextSignature;
    screenrecordState.loading = false;
    renderScreenrecordVideos();
    if (!silent && logsScrollTops.screen === 0) restoreLogsScrollTop("screen", { reset: true });
  } catch (e) {
    if (seq !== screenrecordState.loadSeq) return;
    screenrecordState.loading = false;
    if (!silent && isLogsPageActive()) {
      setScreenrecordStatus(`${getUIText("screenrecord_load_failed", "Failed to load screen recordings")}: ${e.message || e}`, "error");
      showAppToast(e.message || getUIText("screenrecord_load_failed", "Failed to load screen recordings"), { tone: "error" });
    }
  }
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
    window.addEventListener("carrot:pagechange", handleLogsPageChange);
    window.addEventListener("carrot:languagechange", () => {
      dashcamState.signature = "";
      screenrecordState.signature = "";
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
        if (dashcamState.landscape === nextLandscape) return;
        dashcamState.landscape = nextLandscape;
        renderDashcamRoutes({ animate: false });
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
    }, { passive: true });
    routesHost.addEventListener("click", (ev) => {
      const actionEl = ev.target?.closest?.("[data-action]");
      if (!actionEl) return;
      const action = actionEl.dataset.action;
      const route = actionEl.dataset.route || "";
      const segment = actionEl.dataset.segment || "";
      if (action === "toggle-route") {
        if (dashcamState.expanded.has(route)) dashcamState.expanded.delete(route);
        else dashcamState.expanded.add(route);
        if (!renderDashcamRoute(route)) renderDashcamRoutes({ animate: false });
      } else if (action === "play") {
        openDashcamPlayer(route, segment);
      } else if (action === "segment-menu") {
        ev.stopPropagation();
        showDashcamSegmentMenu(route, segment).catch(() => {});
      } else if (action === "select-route") {
        const entry = dashcamState.routes.find((item) => item.route === route);
        if (!entry) return;
        const shouldClear = actionEl.dataset.selected === "1";
        for (const item of entry.segmentFolders || []) {
          if (shouldClear) dashcamState.selected.delete(item);
          else dashcamState.selected.add(item);
        }
        if (!updateDashcamRouteSelectionUi(route)) renderDashcamRoutes({ animate: false });
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
