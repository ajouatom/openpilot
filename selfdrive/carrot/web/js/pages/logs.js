"use strict";

// Logs page — Dashcam (route+segment listing, FFmpeg thumb/preview, FTP upload)
// + Screen Recording listing/playback. Tab switching between the two.

const DASHCAM_UPLOAD_JOB_STORAGE_KEY = "carrot_dashcam_upload_job_id";
const DASHCAM_PAGE_SIZE = 40;
const DASHCAM_LOAD_AHEAD_PX = 1200;
const DASHCAM_ROUTE_WINDOW_OVERSCAN = 10;
const SCREENRECORD_PAGE_SIZE = 40;
const SCREENRECORD_LOAD_AHEAD_PX = 720;
const SCREENRECORD_WINDOW_OVERSCAN = 8;
let dashcamUploadActiveJobId = null;
let dashcamUploadResumePromise = null;

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
    ...(entry.segmentFolders || []),
  ].join("|")).join("\n") + "|" + (typeof LANG !== "undefined" ? LANG : "");
}

function screenrecordVideosSignature(videos) {
  return (videos || []).map((video) => [
    video.id || "",
    video.name || "",
    video.modifiedLabel || "",
    video.size || 0,
  ].join("|")).join("\n") + "|" + (typeof LANG !== "undefined" ? LANG : "");
}

function dashcamDefaultRouteHeight() {
  return isCompactLandscapeMode() ? 210 : 310;
}

function dashcamLayoutKey() {
  const wide = window.matchMedia?.("(min-width: 900px)")?.matches ? "wide" : "narrow";
  const compact = isCompactLandscapeMode() ? "landscape" : "portrait";
  return `${compact}:${wide}:${window.innerWidth}x${window.innerHeight}`;
}

function dashcamRouteHeightFor(route) {
  const key = String(route || "");
  const cached = Number(dashcamState.routeHeights?.[key]);
  if (Number.isFinite(cached) && cached > 0) return cached;
  const fallback = Number(dashcamState.routeHeight) || dashcamDefaultRouteHeight();
  if (key && dashcamState.expanded.has(key) && !isCompactLandscapeMode()) {
    return Math.max(560, fallback);
  }
  return Math.max(120, fallback);
}

function dashcamRouteGap(host) {
  const styles = window.getComputedStyle?.(host);
  return Number.parseFloat(styles?.rowGap || styles?.gap || "0") || 0;
}

function dashcamWindowFor(host, routes) {
  const list = Array.isArray(routes) ? routes : [];
  const count = list.length;
  const viewportHeight = Math.max(1, host?.clientHeight || dashcamDefaultRouteHeight() * 2);
  const scrollTop = Math.max(0, host?.scrollTop || 0);
  const overscanPx = dashcamRouteHeightFor("") * DASHCAM_ROUTE_WINDOW_OVERSCAN;
  const minTop = Math.max(0, scrollTop - overscanPx);
  const maxBottom = scrollTop + viewportHeight + overscanPx;
  const gap = dashcamRouteGap(host);

  let start = 0;
  let end = 0;
  let topHeight = 0;
  let cursor = 0;

  while (start < count) {
    const height = dashcamRouteHeightFor(list[start]?.route) + (start > 0 ? gap : 0);
    if (cursor + height >= minTop) break;
    cursor += height;
    topHeight = cursor;
    start += 1;
  }

  end = start;
  let endHeight = topHeight;
  while (end < count && endHeight < maxBottom) {
    endHeight += dashcamRouteHeightFor(list[end]?.route) + (end > 0 ? gap : 0);
    end += 1;
  }
  const minEnd = Math.min(count, Math.max(end + DASHCAM_ROUTE_WINDOW_OVERSCAN, start + 1));
  while (end < minEnd) {
    endHeight += dashcamRouteHeightFor(list[end]?.route) + (end > 0 ? gap : 0);
    end += 1;
  }

  let totalHeight = topHeight;
  for (let i = start; i < count; i += 1) {
    totalHeight += dashcamRouteHeightFor(list[i]?.route) + (i > 0 ? gap : 0);
  }

  const bottomHeight = Math.max(0, totalHeight - endHeight);
  return { start, end, topHeight, bottomHeight };
}

function screenrecordShouldLoadMore(scroller) {
  if (!scroller || !screenrecordState.hasMore || screenrecordState.loading || screenrecordState.loadingMore) return false;
  const remaining = scroller.scrollHeight - scroller.scrollTop - scroller.clientHeight;
  return remaining <= SCREENRECORD_LOAD_AHEAD_PX;
}

function dashcamShouldLoadMore(scroller) {
  if (!scroller || !dashcamState.hasMore || dashcamState.loading || dashcamState.loadingMore) return false;
  const remaining = scroller.scrollHeight - scroller.scrollTop - scroller.clientHeight;
  return remaining <= DASHCAM_LOAD_AHEAD_PX;
}

function screenrecordWindowFor(host, count) {
  const rowHeight = Math.max(48, Number(screenrecordState.rowHeight) || 80);
  const viewportHeight = Math.max(1, host?.clientHeight || rowHeight * 8);
  const scrollTop = Math.max(0, host?.scrollTop || 0);
  const visibleRows = Math.ceil(viewportHeight / rowHeight);
  const start = Math.max(0, Math.floor(scrollTop / rowHeight) - SCREENRECORD_WINDOW_OVERSCAN);
  const end = Math.min(count, start + visibleRows + (SCREENRECORD_WINDOW_OVERSCAN * 2));
  return { start, end, rowHeight };
}

function screenrecordMeasureRowHeight(host) {
  const row = host?.querySelector?.(".screenrecord-row");
  if (!row) return;
  const styles = window.getComputedStyle?.(host);
  const gap = Number.parseFloat(styles?.rowGap || styles?.gap || "0") || 0;
  const nextHeight = Math.max(48, row.getBoundingClientRect().height + gap);
  if (Math.abs(nextHeight - screenrecordState.rowHeight) < 1) return;
  screenrecordState.rowHeight = nextHeight;
}

function screenrecordSpacerNode(height, position) {
  if (height <= 0) return null;
  const node = document.createElement("div");
  node.className = "screenrecord-virtual-spacer";
  node.dataset.spacer = position;
  node.style.height = `${Math.round(height)}px`;
  return node;
}

function screenrecordRowNode(video, index, existingRows) {
  const id = String(video?.id || "");
  const existing = id ? existingRows.get(id) : null;
  if (existing) {
    existing.style.setProperty("--i", String(index));
    existing.classList.remove("ui-stagger-item");
    return existing;
  }
  const template = document.createElement("template");
  template.innerHTML = screenrecordVideoRowHtml(video, index);
  return template.content.firstElementChild;
}

function patchScreenrecordWindow(host, videos, view) {
  const existingRows = new Map(
    Array.from(host.querySelectorAll(".screenrecord-row"))
      .map((node) => [node.dataset.id || "", node])
      .filter(([id]) => Boolean(id))
  );
  const frag = document.createDocumentFragment();
  const topSpacer = screenrecordSpacerNode(view.start * view.rowHeight, "top");
  const bottomSpacer = screenrecordSpacerNode((videos.length - view.end) * view.rowHeight, "bottom");
  if (topSpacer) frag.appendChild(topSpacer);
  videos.slice(view.start, view.end).forEach((video, offset) => {
    const row = screenrecordRowNode(video, view.start + offset, existingRows);
    if (row) frag.appendChild(row);
  });
  if (bottomSpacer) frag.appendChild(bottomSpacer);
  unobserveLogsLazyImages(host);
  host.replaceChildren(frag);
}

function setScreenrecordLoadingMoreUi(active) {
  const host = document.getElementById("screenrecordVideos");
  if (!host) return;
  host.classList.toggle("is-loading-more", Boolean(active));
}

function scheduleScreenrecordWindowRender() {
  if (screenrecordState.renderFrame) return;
  screenrecordState.renderFrame = requestAnimationFrame(() => {
    screenrecordState.renderFrame = 0;
    renderScreenrecordVideos({ preserve: true });
  });
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

function cancelDashcamRouteRender() {
  if (dashcamState.renderFrame) {
    window.cancelAnimationFrame(dashcamState.renderFrame);
    dashcamState.renderFrame = 0;
  }
}

function setDashcamLoadingMoreUi(active) {
  const host = document.getElementById("dashcamRoutes");
  if (!host) return;
  host.classList.toggle("is-loading-more", Boolean(active));
}

function dashcamSpacerNode(height, position) {
  if (height <= 0) return null;
  const node = document.createElement("div");
  node.className = "dashcam-virtual-spacer";
  node.dataset.spacer = position;
  node.style.height = `${Math.round(height)}px`;
  return node;
}

function dashcamRouteRenderKey(entry) {
  const route = String(entry?.route || "");
  const selected = dashcamSelectedForRoute(entry || { segmentFolders: [] }).join(",");
  const segments = Array.isArray(entry?.segmentFolders) ? entry.segmentFolders.join(",") : "";
  return [
    isCompactLandscapeMode() ? "landscape" : "portrait",
    dashcamState.expanded.has(route) ? "expanded" : "collapsed",
    typeof LANG !== "undefined" ? LANG : "",
    entry?.title || "",
    entry?.dateLabel || "",
    entry?.latestModifiedEpoch || "",
    entry?.latestModifiedLabel || "",
    segments,
    selected,
  ].join("|");
}

function dashcamRouteNode(entry, index, existingCards, options = {}) {
  const route = String(entry?.route || "");
  const nextRenderKey = dashcamRouteRenderKey(entry);
  const existing = route ? existingCards.get(route) : null;
  if (existing && existing.dataset.renderKey === nextRenderKey) {
    existing.style.setProperty("--i", String(index));
    existing.dataset.routeIndex = String(index);
    existing.classList.remove("ui-stagger-item");
    existing.querySelectorAll(".ui-stagger-item").forEach((node) => node.classList.remove("ui-stagger-item"));
    return existing;
  }
  const template = document.createElement("template");
  template.innerHTML = dashcamRouteCardHtml(entry, index, {
    animate: options.animate,
    animateIndex: index,
  });
  return template.content.firstElementChild;
}

function patchDashcamWindow(host, routes, view, options = {}) {
  const existingCards = new Map(
    Array.from(host.querySelectorAll("[data-route-card]"))
      .map((node) => [node.dataset.routeCard || "", node])
      .filter(([route]) => Boolean(route))
  );
  const frag = document.createDocumentFragment();
  const topSpacer = dashcamSpacerNode(view.topHeight, "top");
  const bottomSpacer = dashcamSpacerNode(view.bottomHeight, "bottom");
  if (topSpacer) frag.appendChild(topSpacer);
  routes.slice(view.start, view.end).forEach((entry, offset) => {
    const card = dashcamRouteNode(entry, view.start + offset, existingCards, options);
    if (card) frag.appendChild(card);
  });
  if (bottomSpacer) frag.appendChild(bottomSpacer);
  unobserveLogsLazyImages(host);
  host.replaceChildren(frag);
}

function measureDashcamRouteHeights(host) {
  if (!host) return false;
  const gap = dashcamRouteGap(host);
  const cards = Array.from(host.querySelectorAll("[data-route-card]"));
  let changed = false;
  let total = 0;
  let measured = 0;

  cards.forEach((card) => {
    const route = card.dataset.routeCard || "";
    const index = Number.parseInt(card.dataset.routeIndex || "0", 10) || 0;
    const height = Math.max(120, card.getBoundingClientRect().height + (index > 0 ? gap : 0));
    if (!route || !Number.isFinite(height)) return;
    if (!dashcamState.expanded.has(route) || isCompactLandscapeMode()) {
      total += height;
      measured += 1;
    }
    if (Math.abs((Number(dashcamState.routeHeights[route]) || 0) - height) > 1) {
      dashcamState.routeHeights[route] = height;
      changed = true;
    }
  });

  if (measured) {
    const average = total / measured;
    if (Number.isFinite(average) && Math.abs(average - dashcamState.routeHeight) > 1) {
      dashcamState.routeHeight = average;
      changed = true;
    }
  }

  return changed;
}

function scheduleDashcamWindowRender() {
  if (dashcamState.renderFrame) return;
  dashcamState.renderFrame = requestAnimationFrame(() => {
    dashcamState.renderFrame = 0;
    renderDashcamRoutes({ preserve: true, animate: false });
  });
}

function dashcamWindowNeedsRender(host) {
  if (!host || !(dashcamState.routes || []).length) return false;
  const cards = Array.from(host.querySelectorAll("[data-route-card]"));
  if (!cards.length) return true;
  const hostRect = host.getBoundingClientRect();
  const firstRect = cards[0].getBoundingClientRect();
  const lastRect = cards[cards.length - 1].getBoundingClientRect();
  const buffer = dashcamRouteHeightFor("") * 2;
  return firstRect.top > hostRect.top - buffer || lastRect.bottom < hostRect.bottom + buffer;
}

function maybeLoadMoreDashcamRoutes(scroller = document.getElementById("dashcamRoutes")) {
  if (!dashcamShouldLoadMore(scroller)) return;
  loadDashcamRoutes({ silent: true, append: true }).catch(() => {});
}

function dashcamSelectedForRoute(entry) {
  return (entry.segmentFolders || []).filter((segment) => dashcamState.selected.has(segment));
}

function dashcamRouteCardHtml(entry, index = 0, options = {}) {
  const animate = options.animate !== false;
  const animateIndex = Number.isFinite(options.animateIndex) ? options.animateIndex : index;
  const route = String(entry.route || "");
  const renderKey = escapeHtml(dashcamRouteRenderKey(entry));
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

  return `<article class="dashcam-route-card${animate ? " ui-stagger-item" : ""}"${animate ? ` style="--i:${animateIndex}"` : ""} data-route-card="${routeAttr}" data-route-index="${index}" data-render-key="${renderKey}">
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
  const view = dashcamWindowFor(host, routes);
  const nextSignature = `${dashcamState.signature || dashcamRoutesSignature(routes)}|${dashcamLayoutKey()}|${view.start}:${view.end}|${Math.round(view.topHeight)}:${Math.round(view.bottomHeight)}`;
  if (preserve && host.dataset.signature === nextSignature) {
    hydrateLogsLazyImages(host);
    return;
  }
  patchDashcamWindow(host, routes, view, {
    animate,
  });
  host.dataset.signature = nextSignature;
  host.dataset.renderCount = String(view.end - view.start);
  host.dataset.windowStart = String(view.start);
  host.dataset.windowEnd = String(view.end);
  dashcamState.windowStart = view.start;
  dashcamState.windowEnd = view.end;
  hydrateLogsLazyImages(host);
  requestAnimationFrame(() => {
    if (!isLogsPageActive()) return;
    if (measureDashcamRouteHeights(host) && !dashcamState.scrollBusy) scheduleDashcamWindowRender();
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
  current.dataset.renderKey = dashcamRouteRenderKey(routes[index]);
  hydrateLogsLazyImages(nextMain);
  requestAnimationFrame(() => {
    if (!isLogsPageActive()) return;
    if (measureDashcamRouteHeights(host)) scheduleDashcamWindowRender();
  });
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
  card.dataset.renderKey = dashcamRouteRenderKey(entry);

  return true;
}

async function loadDashcamRoutes({ silent = false, append = false } = {}) {
  if (append && (!dashcamState.hasMore || dashcamState.loading || dashcamState.loadingMore)) return;
  const seq = ++dashcamState.loadSeq;
  if (append) {
    dashcamState.loadingMore = true;
    setDashcamLoadingMoreUi(true);
  } else if (!silent) {
    dashcamState.loading = true;
    dashcamState.loadingMore = false;
    setDashcamLoadingMoreUi(false);
    renderDashcamRoutes();
  }
  try {
    const offset = append ? (dashcamState.nextOffset || dashcamState.routes.length || 0) : 0;
    const currentCount = dashcamState.routes.length || 0;
    const limit = append ? DASHCAM_PAGE_SIZE : Math.max(DASHCAM_PAGE_SIZE, currentCount || 0);
    const json = await getJson(`/api/dashcam/routes?offset=${offset}&limit=${limit}`);
    if (seq !== dashcamState.loadSeq) {
      if (append) {
        dashcamState.loadingMore = false;
        setDashcamLoadingMoreUi(false);
      }
      return;
    }
    if (!isLogsPageActive()) {
      dashcamState.loading = false;
      dashcamState.loadingMore = false;
      setDashcamLoadingMoreUi(false);
      return;
    }
    const incoming = Array.isArray(json.routes) ? json.routes : [];
    const routes = append ? dashcamState.routes.concat(incoming) : incoming;
    const nextSignature = dashcamRoutesSignature(routes);
    if (silent && nextSignature === dashcamState.signature) {
      dashcamState.loading = false;
      dashcamState.loadingMore = false;
      dashcamState.total = Number.isFinite(Number(json.total)) ? Number(json.total) : routes.length;
      dashcamState.nextOffset = json.nextOffset == null ? routes.length : Number(json.nextOffset) || routes.length;
      dashcamState.hasMore = Boolean(json.hasMore);
      setDashcamLoadingMoreUi(false);
      return;
    }
    const validRoutes = new Set(routes.map((entry) => entry.route));
    const validSegments = new Set(routes.flatMap((entry) => entry.segmentFolders || []));
    dashcamState.expanded = new Set(Array.from(dashcamState.expanded).filter((route) => validRoutes.has(route)));
    dashcamState.selected = new Set(Array.from(dashcamState.selected).filter((segment) => validSegments.has(segment)));
    dashcamState.routeHeights = Object.fromEntries(
      Object.entries(dashcamState.routeHeights || {}).filter(([route]) => validRoutes.has(route))
    );
    dashcamState.routes = routes;
    dashcamState.signature = nextSignature;
    dashcamState.total = Number.isFinite(Number(json.total)) ? Number(json.total) : routes.length;
    dashcamState.nextOffset = json.nextOffset == null ? routes.length : Number(json.nextOffset) || routes.length;
    dashcamState.hasMore = Boolean(json.hasMore);
    dashcamState.loading = false;
    dashcamState.loadingMore = false;
    setDashcamLoadingMoreUi(false);
    renderDashcamRoutes({ animate: !silent });
    if (!silent && logsScrollTops.dashcam === 0) restoreLogsScrollTop("dashcam", { reset: true });
    requestAnimationFrame(() => maybeLoadMoreDashcamRoutes());
  } catch (e) {
    if (seq !== dashcamState.loadSeq) {
      if (append) {
        dashcamState.loadingMore = false;
        setDashcamLoadingMoreUi(false);
      }
      return;
    }
    dashcamState.loading = false;
    dashcamState.loadingMore = false;
    setDashcamLoadingMoreUi(false);
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
    else if (!dashcamState.loading && !dashcamState.loadingMore) loadDashcamRoutes({ silent: true }).catch(() => {});
  }, 10000);
}

function markDashcamScrollBusy() {
  dashcamState.scrollBusy = true;
  if (dashcamState.scrollTimer) window.clearTimeout(dashcamState.scrollTimer);
  dashcamState.scrollTimer = window.setTimeout(() => {
    dashcamState.scrollBusy = false;
    if (isLogsPageActive() && logsActiveTab === "dashcam") scheduleDashcamWindowRender();
  }, 380);
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
        try { player.play()?.catch?.(() => {}); } catch {}
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
      videoEl.play?.().catch?.(() => {});
    }
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
  const view = screenrecordWindowFor(host, videos.length);
  const nextSignature = `${screenrecordState.signature || screenrecordVideosSignature(videos)}|${view.start}:${view.end}|${screenrecordState.loadingMore ? "more" : ""}`;
  if (preserve && host.dataset.signature === nextSignature) {
    hydrateLogsLazyImages(host);
    return;
  }
  patchScreenrecordWindow(host, videos, view);
  host.dataset.signature = nextSignature;
  host.dataset.renderCount = String(view.end - view.start);
  screenrecordState.windowStart = view.start;
  screenrecordState.windowEnd = view.end;
  setScreenrecordLoadingMoreUi(screenrecordState.loadingMore);
  hydrateLogsLazyImages(host);
  requestAnimationFrame(() => screenrecordMeasureRowHeight(host));
}

async function loadScreenrecordVideos({ silent = false, append = false } = {}) {
  if (append && (!screenrecordState.hasMore || screenrecordState.loading || screenrecordState.loadingMore)) return;
  const seq = ++screenrecordState.loadSeq;
  if (append) {
    screenrecordState.loadingMore = true;
    setScreenrecordLoadingMoreUi(true);
  } else if (!silent) {
    screenrecordState.loading = true;
    screenrecordState.loadingMore = false;
    setScreenrecordLoadingMoreUi(false);
    renderScreenrecordVideos();
  }
  try {
    const offset = append ? (screenrecordState.nextOffset || screenrecordState.videos.length || 0) : 0;
    const limit = append ? SCREENRECORD_PAGE_SIZE : Math.max(SCREENRECORD_PAGE_SIZE, screenrecordState.videos.length || 0);
    const json = await getJson(`/api/screenrecord/videos?offset=${offset}&limit=${limit}`);
    if (seq !== screenrecordState.loadSeq) return;
    if (!isLogsPageActive()) {
      screenrecordState.loading = false;
      screenrecordState.loadingMore = false;
      setScreenrecordLoadingMoreUi(false);
      return;
    }
    const incoming = Array.isArray(json.videos) ? json.videos : [];
    const videos = append ? screenrecordState.videos.concat(incoming) : incoming;
    const nextSignature = screenrecordVideosSignature(videos);
    if (silent && nextSignature === screenrecordState.signature) {
      screenrecordState.loading = false;
      screenrecordState.loadingMore = false;
      setScreenrecordLoadingMoreUi(false);
      return;
    }
    screenrecordState.videos = videos;
    screenrecordState.signature = nextSignature;
    screenrecordState.total = Number.isFinite(Number(json.total)) ? Number(json.total) : videos.length;
    screenrecordState.nextOffset = json.nextOffset == null ? videos.length : Number(json.nextOffset) || videos.length;
    screenrecordState.hasMore = Boolean(json.hasMore);
    screenrecordState.loading = false;
    screenrecordState.loadingMore = false;
    setScreenrecordLoadingMoreUi(false);
    renderScreenrecordVideos({ animate: !silent });
    if (!silent && logsScrollTops.screen === 0) restoreLogsScrollTop("screen", { reset: true });
  } catch (e) {
    if (seq !== screenrecordState.loadSeq) return;
    screenrecordState.loading = false;
    screenrecordState.loadingMore = false;
    setScreenrecordLoadingMoreUi(false);
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
  dashcamState.loadingMore = false;
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
