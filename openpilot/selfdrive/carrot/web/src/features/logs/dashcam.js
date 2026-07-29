"use strict";

import { dashcamReadStateStore } from "./dashcam_player_session.js";
import { createLogsSegmentStatusTag } from "./player/components.js";
import { loadScreenrecordVideos } from "./screenrecord.js";
import { openRouteSummary } from "./route_summary/index.js";
import {
  formatLogBytes,
  formatRelativeEpoch,
  getLogsScroller,
  hydrateLogsLazyImages,
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
} from "./runtime.js";
import {
  createDashcamUploadProgressModel,
  dashcamUploadProgressState,
} from "./upload_progress.js";
import { dashcamUploadStats } from "./upload_summary.js";

// Logs page — Dashcam tab.
// Route + segment virtual listing, FFmpeg thumb/preview lazy load,
// segment selection, web upload (with cancel/resume), segment menu, player.

const DASHCAM_UPLOAD_JOB_STORAGE_KEY = "carrot_dashcam_upload_job_id";
const DASHCAM_SORT_STORAGE_KEY = "carrot_dashcam_segment_sort";
const DASHCAM_SEGMENT_NAME_LIMIT_MAX = 2000;
const DASHCAM_ROUTE_PAGE_MIN = 10;
const DASHCAM_ROUTE_PAGE_MAX = 40;
const DASHCAM_ROUTE_PAGE_VIEWPORTS = 3;
const DASHCAM_SEGMENT_PAGE_SIZE = 10;
const DASHCAM_SEGMENT_SECONDS = 60;
const DASHCAM_REPLAY_TIME_TOLERANCE_SECONDS = 120;
const DASHCAM_LOAD_AHEAD_VIEWPORTS = 1.5;
const DASHCAM_ROUTE_WINDOW_OVERSCAN_VIEWPORTS = 1.25;
const DASHCAM_UPLOAD_POLL_INTERVAL_MS = 160;
const DASHCAM_UPLOAD_PROGRESS_MIN_VISIBLE_MS = 800;
const DASHCAM_UPLOAD_PROGRESS_FINISH_HOLD_MS = 240;
const DASHCAM_UPLOAD_RESULT_VISIBLE_LIMIT = 5;
let dashcamUploadActiveJobId = null;
let dashcamUploadResumePromise = null;
let dashcamReadStateLoadPromise = null;
let dashcamReadStateWritePromise = Promise.resolve();
let dashcamReadStateInteracted = false;

const dashcamState = {
  initialized: false,
  loading: false,
  routes: [],
  expanded: new Set(),
  selected: new Set(),
  refreshTimer: null,
  loadingMore: false,
  loadingSegments: new Set(),
  segmentScrollTops: Object.create(null),
  scrollBusy: false,
  scrollTimer: null,
  renderFrame: 0,
  loadSeq: 0,
  replayLoadSeq: 0,
  layoutBound: false,
  layoutTimer: null,
  landscape: null,
  layoutKey: "",
  total: 0,
  nextOffset: 0,
  hasMore: false,
  routeHeight: 300,
  routeHeights: Object.create(null),
  windowStart: 0,
  windowEnd: 0,
  signature: "",
  sort: "asc",
};

function readDashcamSortPreference() {
  try {
    const stored = localStorage.getItem(DASHCAM_SORT_STORAGE_KEY);
    return stored === "desc" ? "desc" : "asc";
  } catch {
    return "asc";
  }
}
dashcamState.sort = readDashcamSortPreference();

function dashcamSortDirection() {
  return dashcamState.sort === "desc" ? "desc" : "asc";
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

function dashcamRoutesSignature(routes) {
  return (routes || []).map((entry) => [
    entry.route || "",
    entry.segmentCount || 0,
    entry.routeStartEpoch || 0,
    entry.routeEndEpoch || 0,
    entry.segmentsNextOffset ?? "",
    entry.segmentsHasMore ? "1" : "0",
    ...(entry.segmentFolders || []),
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

function dashcamRoutePageSize(scroller = document.getElementById("dashcamRoutes")) {
  const rowHeight = Math.max(120, dashcamRouteHeightFor(""));
  const viewportHeight = Math.max(rowHeight, scroller?.clientHeight || window.innerHeight || rowHeight);
  const visibleRows = Math.max(1, Math.ceil(viewportHeight / rowHeight));
  return Math.max(
    DASHCAM_ROUTE_PAGE_MIN,
    Math.min(DASHCAM_ROUTE_PAGE_MAX, visibleRows * DASHCAM_ROUTE_PAGE_VIEWPORTS)
  );
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
  const rowHeight = Math.max(120, dashcamRouteHeightFor(""));
  const overscanPx = Math.max(rowHeight * 2, viewportHeight * DASHCAM_ROUTE_WINDOW_OVERSCAN_VIEWPORTS);
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
  const minEnd = Math.min(count, Math.max(end + Math.ceil(overscanPx / rowHeight), start + 1));
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

function dashcamShouldLoadMore(scroller) {
  if (!scroller || !dashcamState.hasMore || dashcamState.loading || dashcamState.loadingMore) return false;
  const remaining = scroller.scrollHeight - scroller.scrollTop - scroller.clientHeight;
  return remaining <= Math.max(360, scroller.clientHeight * DASHCAM_LOAD_AHEAD_VIEWPORTS);
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

function dashcamSegmentListRoute(list) {
  return list?.closest?.("[data-route-card]")?.dataset.routeCard || "";
}

function rememberDashcamSegmentScroll(list) {
  const route = dashcamSegmentListRoute(list);
  if (!route || !dashcamState.segmentScrollTops) return;
  dashcamState.segmentScrollTops[route] = Math.max(0, list.scrollTop || 0);
}

function rememberVisibleDashcamSegmentScrolls(host = document.getElementById("dashcamRoutes")) {
  if (!host) return;
  host.querySelectorAll(".dashcam-segment-list").forEach((list) => rememberDashcamSegmentScroll(list));
}

function restoreDashcamSegmentScroll(list) {
  const route = dashcamSegmentListRoute(list);
  if (!route || !dashcamState.segmentScrollTops) return;
  const nextTop = Number(dashcamState.segmentScrollTops[route]);
  if (!Number.isFinite(nextTop) || nextTop <= 0) return;
  list.scrollTop = nextTop;
}

function restoreVisibleDashcamSegmentScrolls(host = document.getElementById("dashcamRoutes")) {
  if (!host) return;
  host.querySelectorAll(".dashcam-segment-list").forEach((list) => restoreDashcamSegmentScroll(list));
  requestAnimationFrame(() => {
    if (!host.isConnected) return;
    host.querySelectorAll(".dashcam-segment-list").forEach((list) => restoreDashcamSegmentScroll(list));
  });
}

function dashcamRouteRenderKey(entry) {
  const route = String(entry?.route || "");
  const selected = dashcamSelectedForRoute(entry || { segmentFolders: [] }).join(",");
  const segments = dashcamSegmentsForRoute(entry).join(",");
  return [
    isCompactLandscapeMode() ? "landscape" : "portrait",
    dashcamState.expanded.has(route) ? "expanded" : "collapsed",
    typeof LANG !== "undefined" ? LANG : "",
    entry?.title || "",
    entry?.dateLabel || "",
    entry?.latestModifiedEpoch || "",
    entry?.latestModifiedLabel || "",
    entry?.routeStartEpoch || "",
    entry?.routeEndEpoch || "",
    dashcamSegmentCountForRoute(entry),
    entry?.segmentsNextOffset ?? "",
    entry?.segmentsHasMore ? "more" : "done",
    dashcamState.loadingSegments?.has(route) ? "loading" : "idle",
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
  rememberVisibleDashcamSegmentScrolls(host);
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
  unobserveDashcamSegmentLoaders(host);
  host.replaceChildren(frag);
  restoreVisibleDashcamSegmentScrolls(host);
}

function unobserveDashcamSegmentLoaders(host) {
  if (!dashcamSegmentLoaderObserver || !host) return;
  host.querySelectorAll?.("[data-segment-loader]").forEach((loader) => {
    dashcamSegmentLoaderObserver.unobserve(loader);
    delete loader.dataset.observed;
  });
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

function dashcamSegmentsForRoute(entry) {
  return Array.isArray(entry?.segmentFolders) ? entry.segmentFolders : [];
}

function dashcamSegmentCountForRoute(entry) {
  const total = Number(entry?.segmentCount);
  const loaded = dashcamSegmentsForRoute(entry).length;
  return Number.isFinite(total) && total >= loaded ? total : loaded;
}

function dashcamRouteHasMoreSegments(entry) {
  return Boolean(entry?.segmentsHasMore) || dashcamSegmentsForRoute(entry).length < dashcamSegmentCountForRoute(entry);
}

function dashcamSegmentNextOffset(entry) {
  const next = Number(entry?.segmentsNextOffset);
  if (Number.isFinite(next) && next >= 0) return next;
  return dashcamSegmentsForRoute(entry).length;
}

function mergeDashcamSegments(existing, incoming, sort = dashcamSortDirection()) {
  const merged = [];
  const seen = new Set();
  [...(existing || []), ...(incoming || [])].forEach((segment) => {
    if (!segment || seen.has(segment)) return;
    seen.add(segment);
    merged.push(segment);
  });
  const sign = sort === "desc" ? -1 : 1;
  return merged.sort((a, b) => sign * (dashcamSegmentIndex(a) - dashcamSegmentIndex(b)));
}

function mergeDashcamRoutePage(entry, existing) {
  if (!entry || !existing) return entry;
  const route = String(entry.route || "");
  if (!route || route !== existing.route) return entry;
  const incomingSegments = dashcamSegmentsForRoute(entry);
  const existingSegments = dashcamSegmentsForRoute(existing);
  if (existingSegments.length <= incomingSegments.length) return entry;

  const mergedSegments = mergeDashcamSegments(incomingSegments, existingSegments);
  const total = Math.max(dashcamSegmentCountForRoute(entry), mergedSegments.length);
  return {
    ...entry,
    segmentFolders: mergedSegments,
    segmentTimes: { ...(existing.segmentTimes || {}), ...(entry.segmentTimes || {}) },
    segmentCount: total,
    segmentsNextOffset: mergedSegments.length < total
      ? Math.max(dashcamSegmentNextOffset(entry), dashcamSegmentNextOffset(existing), mergedSegments.length)
      : null,
    segmentsHasMore: mergedSegments.length < total,
  };
}

function dashcamDateParts(epochSeconds) {
  const epoch = Number(epochSeconds || 0);
  if (!Number.isFinite(epoch) || epoch <= 0) return null;
  const date = new Date(epoch * 1000);
  if (!Number.isFinite(date.getTime())) return null;
  const pad = (value) => String(value).padStart(2, "0");
  return {
    date: `${date.getFullYear()}.${pad(date.getMonth() + 1)}.${pad(date.getDate())}`,
    time: `${pad(date.getHours())}:${pad(date.getMinutes())}:${pad(date.getSeconds())}`,
  };
}

function formatDashcamTimeRange(startEpoch, endEpoch, options = {}) {
  const start = dashcamDateParts(startEpoch);
  const end = dashcamDateParts(endEpoch);
  if (!start || !end) return "";
  if (start.date === end.date) {
    return options.multiline
      ? `${start.date}\n${start.time} – ${end.time}`
      : `${start.date} ${start.time} – ${end.time}`;
  }
  return options.multiline
    ? `${start.date} ${start.time}\n– ${end.date} ${end.time}`
    : `${start.date} ${start.time} – ${end.date} ${end.time}`;
}

function dashcamPlayerGroup(entry, route) {
  const start = dashcamDateParts(entry?.routeStartEpoch);
  const end = dashcamDateParts(entry?.routeEndEpoch);
  const sameDate = start && end && start.date === end.date;
  return {
    route,
    title: entry?.title || dashcamRouteTitle(route),
    dateLabel: sameDate ? start.date : "",
    timeRange: start && end
      ? (sameDate ? `${start.time} – ${end.time}` : `${start.date} ${start.time} – ${end.date} ${end.time}`)
      : String(entry?.dateLabel || ""),
    segmentCount: dashcamSegmentCountForRoute(entry),
  };
}

function dashcamPlayerSegmentTimeLabel(entry, segment) {
  const time = dashcamSegmentTime(entry, segment);
  const start = dashcamDateParts(time?.startEpoch);
  const end = dashcamDateParts(time?.endEpoch);
  if (start && end) {
    return start.date === end.date
      ? `${start.time}–${end.time}`
      : `${start.date} ${start.time}–${end.date} ${end.time}`;
  }
  return formatDashcamSegmentTimeLabel(entry, segment);
}

function dashcamPlayerSegment(entry, route, segment) {
  const segmentIndex = dashcamSegmentIndex(segment);
  const time = dashcamSegmentTime(entry, segment);
  return Object.freeze({
    id: segment,
    name: `${getUIText("segment_label", "Segment")} ${segmentIndex}`,
    timeLabel: dashcamPlayerSegmentTimeLabel(entry, segment),
    title: `${dashcamRouteTitle(route)} · ${getUIText("segment_label", "Segment")} ${segmentIndex}`,
    subtitle: formatDashcamTimeRange(time?.startEpoch, time?.endEpoch),
    src: dashcamApiPath("video", segment),
    thumbnailSrc: dashcamApiPath("thumbnail", segment),
  });
}

function dashcamPlayerSegments(entry, route, segments) {
  return (segments || []).map((segment) => dashcamPlayerSegment(entry, route, segment));
}

function dashcamSegmentTime(entry, segment) {
  const value = entry?.segmentTimes?.[segment];
  return value && typeof value === "object" ? value : null;
}

function formatDashcamClock(epochSeconds) {
  const parts = dashcamDateParts(epochSeconds);
  return parts ? parts.time.slice(0, 5) : "";
}

function formatDashcamSegmentTimeLabel(entry, segment) {
  const time = dashcamSegmentTime(entry, segment);
  return formatDashcamClock(time?.startEpoch) || `SEG ${dashcamSegmentIndex(segment)}`;
}

function formatDashcamSegmentFullTime(entry, segment) {
  const time = dashcamSegmentTime(entry, segment);
  return time ? formatDashcamTimeRange(time.startEpoch, time.endEpoch) : "";
}

function compactDashcamSegmentName(segment) {
  const value = String(segment || "").trim();
  if (value.length <= 22) return value;
  const parts = value.split("--");
  if (parts.length >= 3) {
    const device = parts[0].slice(0, 8);
    const route = parts[parts.length - 2].slice(-10);
    const index = parts[parts.length - 1];
    return `${device}…${route}--${index}`;
  }
  return `${value.slice(0, 10)}…${value.slice(-9)}`;
}

function normalizedDashcamReplayTime(entry, segment) {
  const raw = dashcamSegmentTime(entry, segment);
  const rawStart = Number(raw?.startEpoch || 0);
  const rawEnd = Number(raw?.endEpoch || 0);
  const candidates = Object.entries(entry?.segmentTimes || {})
    .map(([name, time]) => ({
      index: dashcamSegmentIndex(name),
      startEpoch: Number(time?.startEpoch || 0),
      endEpoch: Number(time?.endEpoch || 0),
    }))
    .filter((time) => time.startEpoch > 0 && time.endEpoch >= time.startEpoch)
    .sort((left, right) => left.index - right.index);
  const anchor = candidates[0];
  if (!anchor) return null;
  const targetIndex = dashcamSegmentIndex(segment);
  const expectedStart = anchor.startEpoch + (targetIndex - anchor.index) * DASHCAM_SEGMENT_SECONDS;
  const rawDuration = rawEnd - rawStart;
  const duration = rawDuration > 0 && rawDuration <= DASHCAM_REPLAY_TIME_TOLERANCE_SECONDS
    ? rawDuration
    : DASHCAM_SEGMENT_SECONDS;
  const useRawStart = rawStart > 0
    && Math.abs(rawStart - expectedStart) <= DASHCAM_REPLAY_TIME_TOLERANCE_SECONDS;
  const startEpoch = useRawStart ? rawStart : expectedStart;
  return { startEpoch, endEpoch: startEpoch + duration };
}

function formatDashcamReplayTitle(entry, segment) {
  const segmentName = compactDashcamSegmentName(segment);
  const time = normalizedDashcamReplayTime(entry, segment);
  const start = dashcamDateParts(time?.startEpoch);
  const end = dashcamDateParts(time?.endEpoch);
  if (start && end) {
    const timeLabel = `${start.time}–${end.time}`;
    return segmentName ? `${segmentName} · ${timeLabel}` : timeLabel;
  }
  return segmentName || `#${dashcamSegmentIndex(segment)}`;
}

let dashcamSegmentLoaderObserver = null;
function ensureDashcamSegmentLoaderObserver(scroller) {
  if (!scroller || !("IntersectionObserver" in window)) return null;
  if (dashcamSegmentLoaderObserver && dashcamSegmentLoaderObserver._root === scroller) {
    return dashcamSegmentLoaderObserver;
  }
  if (dashcamSegmentLoaderObserver) dashcamSegmentLoaderObserver.disconnect();
  dashcamSegmentLoaderObserver = new IntersectionObserver((entries) => {
    if (!isLogsPageActive()) return;
    entries.forEach((entry) => {
      if (!entry.isIntersecting) return;
      const route = entry.target.dataset?.route || "";
      if (!route || dashcamState.loadingSegments?.has(route)) return;
      loadDashcamSegments(route).catch(() => {});
    });
  }, { root: scroller, rootMargin: "240px 0px", threshold: 0.01 });
  dashcamSegmentLoaderObserver._root = scroller;
  return dashcamSegmentLoaderObserver;
}

function maybeLoadVisibleDashcamSegments(scroller = document.getElementById("dashcamRoutes")) {
  if (!scroller || !isLogsPageActive()) return;
  const observer = ensureDashcamSegmentLoaderObserver(scroller);
  if (observer) {
    scroller.querySelectorAll("[data-segment-loader]").forEach((loader) => {
      if (loader.dataset.observed === "1") return;
      loader.dataset.observed = "1";
      observer.observe(loader);
    });
    return;
  }
  // Fallback for environments without IntersectionObserver
  const hostRect = scroller.getBoundingClientRect();
  scroller.querySelectorAll("[data-segment-loader]").forEach((loader) => {
    const route = loader.dataset.route || "";
    if (!route || dashcamState.loadingSegments?.has(route)) return;
    const rect = loader.getBoundingClientRect();
    if (rect.top <= hostRect.bottom + 160 && rect.bottom >= hostRect.top - 40) {
      loadDashcamSegments(route).catch(() => {});
    }
  });
}

let segmentListPersistFrame = 0;
const segmentListPersistQueue = new Set();
function scheduleSegmentListScrollPersist(list) {
  if (!list) return;
  segmentListPersistQueue.add(list);
  if (segmentListPersistFrame) return;
  segmentListPersistFrame = requestAnimationFrame(() => {
    segmentListPersistFrame = 0;
    segmentListPersistQueue.forEach((node) => {
      if (node.isConnected) rememberDashcamSegmentScroll(node);
    });
    segmentListPersistQueue.clear();
  });
}

function dashcamSelectedForRoute(entry) {
  return dashcamSegmentsForRoute(entry).filter((segment) => dashcamState.selected.has(segment));
}

function dashcamSegmentReadStatusTag(segment) {
  return createLogsSegmentStatusTag(
    dashcamReadStateStore.statusFor(segment),
    {
      reading: getUIText("segment_reading", "Reading"),
      recent: getUIText("segment_recently_read", "Recently viewed"),
    },
  );
}

function dashcamSegmentReadStatusHtml(segment) {
  return dashcamSegmentReadStatusTag(segment)?.outerHTML || "";
}

function syncDashcamSegmentReadStatusUi() {
  const host = document.getElementById("dashcamRoutes");
  if (!host) return;
  host.querySelectorAll(".dashcam-segment-tile[data-segment]").forEach((tile) => {
    const statusHost = tile.querySelector(".dashcam-segment-read-status");
    if (!statusHost) return;
    const tag = dashcamSegmentReadStatusTag(tile.dataset.segment);
    statusHost.replaceChildren(...(tag ? [tag] : []));
    statusHost.hidden = !tag;
  });
}

function persistDashcamRecentSegment(segment) {
  const recentSegment = String(segment || "").trim();
  if (!recentSegment) return Promise.resolve(null);
  const write = dashcamReadStateWritePromise
    .catch(() => null)
    .then(() => postJson("/api/dashcam/read-state", { recentSegment }));
  dashcamReadStateWritePromise = write;
  return write;
}

function selectDashcamReadSegment(segment) {
  dashcamReadStateInteracted = true;
  const before = dashcamReadStateStore.snapshot();
  const result = dashcamReadStateStore.select(segment);
  const recentSegment = result.state.previousSegment;
  if (
    result.changed
    && recentSegment
    && recentSegment !== before.previousSegment
  ) {
    persistDashcamRecentSegment(recentSegment).catch(() => {});
  }
  return result;
}

function finishDashcamReadSegment() {
  const result = dashcamReadStateStore.finish();
  if (result.changed && result.state.previousSegment) {
    persistDashcamRecentSegment(result.state.previousSegment).catch(() => {});
  }
  return result;
}

function loadDashcamReadState() {
  if (dashcamReadStateLoadPromise) return dashcamReadStateLoadPromise;
  dashcamReadStateLoadPromise = getJson("/api/dashcam/read-state")
    .then((response) => {
      if (!dashcamReadStateInteracted) {
        dashcamReadStateStore.restoreRecent(response?.recentSegment);
      }
      return dashcamReadStateStore.snapshot();
    })
    .catch(() => dashcamReadStateStore.snapshot());
  return dashcamReadStateLoadPromise;
}

function dashcamSegmentTileHtml(route, segment, segmentIndex, options = {}) {
  const entry = options.entry || (dashcamState.routes || []).find((item) => item.route === route);
  const compactSegments = options.compact === true;
  const animate = options.animate === true;
  const routeAttr = escapeHtml(route);
  const segAttr = escapeHtml(segment);
  const timeLabel = escapeHtml(formatDashcamSegmentTimeLabel(entry, segment));
  const readStatus = dashcamSegmentReadStatusHtml(segment);
  const checked = dashcamState.selected.has(segment) ? " checked" : "";
  const tileClass = [
    "dashcam-segment-tile",
    compactSegments ? "dashcam-segment-tile--compact" : "",
    animate ? "dashcam-segment-tile--append" : "",
  ].filter(Boolean).join(" ");
  const thumbClass = compactSegments ? "dashcam-segment-thumb dashcam-segment-thumb--compact" : "dashcam-segment-thumb";
  const checkClass = compactSegments ? "dashcam-segment-check dashcam-segment-check--compact" : "dashcam-segment-check";
  return `<div class="${tileClass}" style="--i:${segmentIndex}" data-action="play" data-route="${routeAttr}" data-segment="${segAttr}">
    <div class="${thumbClass}">
      <img class="logs-lazy-img" loading="lazy" decoding="async" fetchpriority="low" data-src="${dashcamApiPath("thumbnail", segment)}" alt="">
      <label class="${checkClass}" title="${escapeHtml(getUIText("select_all", "Select"))}" onclick="event.stopPropagation()">
        <input type="checkbox" data-action="select-segment" data-segment="${segAttr}"${checked}>
      </label>
    </div>
    <div class="dashcam-segment-body">
      <div class="dashcam-segment-topline">
        <div class="dashcam-segment-badge">${timeLabel}</div>
        <span class="dashcam-segment-read-status"${readStatus ? "" : " hidden"}>${readStatus}</span>
      </div>
      <div class="dashcam-segment-name">${segAttr}</div>
    </div>
    <button class="dashcam-menu-btn" type="button" data-action="segment-menu" data-route="${routeAttr}" data-segment="${segAttr}" aria-label="${escapeHtml(getUIText("segment_menu", "Segment menu"))}" title="${escapeHtml(getUIText("segment_menu", "Segment menu"))}">
      <svg viewBox="0 0 24 24"><path fill="currentColor" d="M12 8a2 2 0 1 0 0-4 2 2 0 0 0 0 4m0 2a2 2 0 1 0 0 4 2 2 0 0 0 0-4m0 6a2 2 0 1 0 0 4 2 2 0 0 0 0-4"/></svg>
    </button>
  </div>`;
}

function dashcamRouteCardHtml(entry, index = 0, options = {}) {
  const animate = options.animate !== false;
  const animateIndex = Number.isFinite(options.animateIndex) ? options.animateIndex : index;
  const route = String(entry.route || "");
  const renderKey = escapeHtml(dashcamRouteRenderKey(entry));
  const segments = dashcamSegmentsForRoute(entry);
  const segmentCount = dashcamSegmentCountForRoute(entry);
  const loadedCount = segments.length;
  const hasMoreSegments = dashcamRouteHasMoreSegments(entry);
  const loadingSegments = dashcamState.loadingSegments?.has(route);
  const expanded = dashcamState.expanded.has(route);
  const compactSegments = isCompactLandscapeMode();
  const shouldRenderSegments = expanded || compactSegments;
  const selected = dashcamSelectedForRoute(entry);
  const allSelected = segmentCount > 0 && selected.length === segmentCount;
  const selectLabel = allSelected
    ? getUIText("deselect_all", "Deselect all")
    : getUIText("select_all", "Select all");
  const representative = segments[0] || "";
  const routeAttr = escapeHtml(route);
  const title = escapeHtml(entry.title || dashcamRouteTitle(route));
  const dateLabel = escapeHtml(entry.dateLabel || route);
  const latest = escapeHtml(formatRelativeEpoch(entry.latestModifiedEpoch) || localizeRelativeLabel(entry.latestModifiedLabel) || "-");
  const routeTimeRange = formatDashcamTimeRange(entry.routeStartEpoch, entry.routeEndEpoch, { multiline: true });
  const routeTimestamp = routeTimeRange
    ? `<div class="dashcam-route-preview__timestamp">${escapeHtml(routeTimeRange).replace("\n", "<br>")}</div>`
    : "";
  const preview = representative
    ? `<div class="dashcam-route-media">
        <div class="dashcam-route-preview" data-action="play" data-route="${routeAttr}" data-segment="${escapeHtml(representative)}">
          <img class="logs-lazy-img" loading="lazy" decoding="async" fetchpriority="low" data-src="${dashcamApiPath("preview", representative)}" data-fallback="${dashcamApiPath("thumbnail", representative)}" onerror="this.onerror=null;if(this.dataset.fallback)this.src=this.dataset.fallback;" alt="">
          <div class="dashcam-route-preview__shade"></div>
          ${routeTimestamp}
          <div class="dashcam-route-preview__chips">
            <span class="dashcam-chip">${escapeHtml(getUIText("segment_count", "{count} segments", { count: segmentCount }))}</span>
            <span class="dashcam-chip">${latest}</span>
          </div>
          <button class="dashcam-menu-btn dashcam-route-preview__menu" type="button" data-action="segment-menu" data-route="${routeAttr}" data-segment="${escapeHtml(representative)}" aria-label="${escapeHtml(getUIText("segment_menu", "Segment menu"))}" title="${escapeHtml(getUIText("segment_menu", "Segment menu"))}">
            <svg viewBox="0 0 24 24"><path fill="currentColor" d="M12 8a2 2 0 1 0 0-4 2 2 0 0 0 0 4m0 2a2 2 0 1 0 0 4 2 2 0 0 0 0-4m0 6a2 2 0 1 0 0 4 2 2 0 0 0 0-4"/></svg>
          </button>
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
    return dashcamSegmentTileHtml(route, segment, segmentIndex, { compact: compactSegments, entry });
  }).join("") : "";
  const segmentLoader = shouldRenderSegments && hasMoreSegments
    ? `<div class="dashcam-segment-loader${loadingSegments ? " is-loading" : ""}" data-segment-loader="1" data-route="${routeAttr}" aria-hidden="true"></div>`
    : "";

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
          <button class="smallBtn" type="button" data-action="select-route" data-route="${routeAttr}" data-selected="${allSelected ? "1" : "0"}">${escapeHtml(selectLabel)}</button>
          <button class="smallBtn btn--filled" type="button" data-action="upload-selected" data-route="${routeAttr}" ${selected.length ? "" : "disabled"}>${escapeHtml(getUIText("upload_selected", "Upload selected"))}</button>
          <button class="smallBtn dashcam-group-menu-btn dashcam-group-menu-btn--row" type="button" data-action="route-menu" data-route="${routeAttr}" aria-label="${escapeHtml(getUIText("group_menu", "Group menu"))}" title="${escapeHtml(getUIText("group_menu", "Group menu"))}">
            <svg viewBox="0 0 24 24"><path fill="currentColor" d="M6 10c-1.1 0-2 .9-2 2s.9 2 2 2 2-.9 2-2-.9-2-2-2m12 0c-1.1 0-2 .9-2 2s.9 2 2 2 2-.9 2-2-.9-2-2-2m-6 0c-1.1 0-2 .9-2 2s.9 2 2 2 2-.9 2-2-.9-2-2-2"/></svg>
          </button>
        </div>
        <div class="dashcam-segment-list">${segmentList}${segmentLoader}</div>
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
    host.innerHTML = logsLoadingSkeletonHtml("dashcam");
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
    maybeLoadVisibleDashcamSegments(host);
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
    maybeLoadVisibleDashcamSegments(host);
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

  const selected = dashcamSelectedForRoute(entry);
  const segmentCount = dashcamSegmentCountForRoute(entry);
  const allSelected = segmentCount > 0 && selected.length === segmentCount;

  const countEl = card.querySelector(".dashcam-selection-count");
  if (countEl) countEl.textContent = getUIText("selected_count", "{count} selected", { count: selected.length });

  const selectBtn = card.querySelector('[data-action="select-route"]');
  if (selectBtn) {
    selectBtn.dataset.selected = allSelected ? "1" : "0";
    selectBtn.textContent = allSelected
      ? getUIText("deselect_all", "Deselect all")
      : getUIText("select_all", "Select all");
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

function updateDashcamSegmentLoaderUi(route, loading = false) {
  const host = document.getElementById("dashcamRoutes");
  const entry = (dashcamState.routes || []).find((item) => item.route === route);
  if (!host || !entry) return false;
  const card = Array.from(host.querySelectorAll("[data-route-card]"))
    .find((node) => node.dataset.routeCard === route);
  if (!card) return false;
  const loader = card.querySelector("[data-segment-loader]");
  if (!loader) return false;
  if (!dashcamRouteHasMoreSegments(entry)) {
    loader.remove();
    return true;
  }
  loader.classList.toggle("is-loading", Boolean(loading));
  return true;
}

function appendDashcamSegmentsToRoute(route, newSegments, startIndex = 0) {
  if (!newSegments.length) return false;
  const host = document.getElementById("dashcamRoutes");
  const entry = (dashcamState.routes || []).find((item) => item.route === route);
  if (!host || !entry) return false;
  const card = Array.from(host.querySelectorAll("[data-route-card]"))
    .find((node) => node.dataset.routeCard === route);
  const list = card?.querySelector(".dashcam-segment-list");
  if (!card || !list) return false;

  const scrollTop = list.scrollTop;
  const wasScrollable = list.scrollHeight > list.clientHeight + 1;
  const wasNearBottom = wasScrollable && (list.scrollHeight - list.scrollTop - list.clientHeight <= 48);
  const isScrolling = Boolean(dashcamState.scrollBusy);
  rememberDashcamSegmentScroll(list);
  const compact = isCompactLandscapeMode();
  const template = document.createElement("template");
  // Don't animate tile entry while the user is actively scrolling — the
  // simultaneous animation + scrollTop adjustment is what causes "shake".
  const animate = !isScrolling;
  template.innerHTML = newSegments
    .map((segment, offset) => dashcamSegmentTileHtml(route, segment, startIndex + offset, { compact, animate, entry }))
    .join("");
  const loader = list.querySelector("[data-segment-loader]");
  list.insertBefore(template.content, loader || null);
  hydrateLogsLazyImages(list);
  updateDashcamRouteSelectionUi(route);
  updateDashcamSegmentLoaderUi(route, false);
  // Pin to bottom only when the user wasn't actively scrolling — otherwise
  // setting scrollTop fights inertia and produces a visible jump/shake.
  if (wasNearBottom && !isScrolling) {
    const nextTop = Math.max(0, list.scrollHeight - list.clientHeight);
    list.scrollTop = nextTop;
    rememberDashcamSegmentScroll(list);
  } else {
    // Preserve current position; browser will keep inertia smooth.
    list.scrollTop = scrollTop;
    rememberDashcamSegmentScroll(list);
  }
  card.dataset.renderKey = dashcamRouteRenderKey(entry);
  return true;
}

async function loadDashcamSegments(route) {
  if (!route || dashcamState.loadingSegments?.has(route)) return;
  const entry = (dashcamState.routes || []).find((item) => item.route === route);
  if (!entry || !dashcamRouteHasMoreSegments(entry)) return;
  const previousCount = dashcamSegmentsForRoute(entry).length;
  dashcamState.loadingSegments.add(route);
  markDashcamScrollBusy({ renderOnIdle: false });
  updateDashcamSegmentLoaderUi(route, true);

  try {
    const offset = dashcamSegmentNextOffset(entry);
    const json = await getJson(`/api/dashcam/segments/${encodeURIComponent(route)}?offset=${offset}&limit=${DASHCAM_SEGMENT_PAGE_SIZE}&sort=${dashcamSortDirection()}`);
    const current = (dashcamState.routes || []).find((item) => item.route === route);
    if (!current) return;
    const incoming = Array.isArray(json.segments) ? json.segments : [];
    const existing = new Set(dashcamSegmentsForRoute(current));
    const appended = incoming.filter((segment) => segment && !existing.has(segment));
    current.segmentFolders = mergeDashcamSegments(dashcamSegmentsForRoute(current), incoming);
    current.segmentTimes = { ...(current.segmentTimes || {}), ...(json.segmentTimes || {}) };
    current.segmentCount = Number.isFinite(Number(json.total)) ? Number(json.total) : dashcamSegmentCountForRoute(current);
    current.segmentsNextOffset = json.nextOffset == null ? current.segmentFolders.length : Number(json.nextOffset) || current.segmentFolders.length;
    current.segmentsHasMore = Boolean(json.hasMore);
    dashcamState.signature = dashcamRoutesSignature(dashcamState.routes);
    if (appended.length && appendDashcamSegmentsToRoute(route, appended, previousCount)) return;
  } catch (e) {
    if (isLogsPageActive()) {
      showAppToast(e.message || getUIText("dashcam_load_failed", "Failed to load dashcam list"), { tone: "error" });
    }
  } finally {
    dashcamState.loadingSegments.delete(route);
    updateDashcamSegmentLoaderUi(route, false);
    requestAnimationFrame(() => maybeLoadVisibleDashcamSegments());
  }
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
    const routePageSize = dashcamRoutePageSize();
    const limit = append ? routePageSize : Math.max(routePageSize, currentCount || 0);
    const json = await getJson(`/api/dashcam/routes?offset=${offset}&limit=${limit}&segment_limit=${DASHCAM_SEGMENT_PAGE_SIZE}&sort=${dashcamSortDirection()}`);
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
    const existingRoutes = new Map((dashcamState.routes || []).map((entry) => [entry.route, entry]));
    const nextIncoming = append ? incoming : incoming.map((entry) => mergeDashcamRoutePage(entry, existingRoutes.get(entry.route)));
    const routes = append ? dashcamState.routes.concat(nextIncoming) : nextIncoming;
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
    const validSegments = new Set(routes.flatMap((entry) => dashcamSegmentsForRoute(entry)));
    dashcamState.expanded = new Set(Array.from(dashcamState.expanded).filter((route) => validRoutes.has(route)));
    dashcamState.selected = new Set(Array.from(dashcamState.selected).filter((segment) => validSegments.has(segment)));
    dashcamState.routeHeights = Object.fromEntries(
      Object.entries(dashcamState.routeHeights || {}).filter(([route]) => validRoutes.has(route))
    );
    dashcamState.segmentScrollTops = Object.fromEntries(
      Object.entries(dashcamState.segmentScrollTops || {}).filter(([route]) => validRoutes.has(route))
    );
    dashcamState.routes = routes;
    dashcamState.signature = nextSignature;
    dashcamState.total = Number.isFinite(Number(json.total)) ? Number(json.total) : routes.length;
    dashcamState.nextOffset = json.nextOffset == null ? routes.length : Number(json.nextOffset) || routes.length;
    dashcamState.hasMore = Boolean(json.hasMore);
    dashcamState.loading = false;
    dashcamState.loadingMore = false;
    setDashcamLoadingMoreUi(false);
    renderDashcamRoutes({ animate: append });
    if (!silent && logsScrollTops.dashcam === 0) restoreLogsScrollTop("dashcam", { reset: true });
    requestAnimationFrame(() => {
      maybeLoadMoreDashcamRoutes();
      maybeLoadVisibleDashcamSegments();
    });
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
    else if (!dashcamState.loading && !dashcamState.loadingMore && !dashcamState.loadingSegments?.size) {
      loadDashcamRoutes({ silent: true }).catch(() => {});
    }
  }, 10000);
}

function markDashcamScrollBusy(options = {}) {
  const renderOnIdle = options.renderOnIdle !== false;
  dashcamState.scrollBusy = true;
  if (dashcamState.scrollTimer) window.clearTimeout(dashcamState.scrollTimer);
  dashcamState.scrollTimer = window.setTimeout(() => {
    dashcamState.scrollBusy = false;
    if (renderOnIdle && isLogsPageActive() && logsActiveTab === "dashcam") {
      const host = getLogsScroller("dashcam");
      if (dashcamWindowNeedsRender(host)) scheduleDashcamWindowRender();
    }
  }, 380);
}

function openDashcamPlayer(route, segment) {
  const entry = (dashcamState.routes || []).find((item) => item.route === route);
  const orderedSegments = mergeDashcamSegments(dashcamSegmentsForRoute(entry), [], "asc");
  const readState = selectDashcamReadSegment(segment).state;
  const time = dashcamSegmentTime(entry, segment);
  const endEpoch = Number(time?.endEpoch || 0);
  const initialSubtitle = formatDashcamTimeRange(time?.startEpoch, endEpoch);
  const playerController = openLogsVideoPlayer(
    `${dashcamRouteTitle(route)} · ${getUIText("segment_label", "Segment")} ${dashcamSegmentIndex(segment)}`,
    dashcamApiPath("video", segment),
    {
      kind: "dashcam",
      currentGroup: dashcamPlayerGroup(entry, route),
      segments: dashcamPlayerSegments(entry, route, orderedSegments),
      activeSegment: segment,
      previousSegment: readState.previousSegment,
      subtitle: initialSubtitle,
      subtitleForSegmentDuration: (target, duration) => {
        const targetTime = dashcamSegmentTime(entry, target);
        const targetEndEpoch = Number(targetTime?.endEpoch || 0);
        const targetSubtitle = formatDashcamTimeRange(targetTime?.startEpoch, targetEndEpoch);
        const seconds = Number(duration || 0);
        if (!Number.isFinite(seconds) || seconds <= 0 || targetEndEpoch <= 0) return targetSubtitle;
        return formatDashcamTimeRange(targetEndEpoch - seconds, targetEndEpoch);
      },
      onSegmentSend: (target) => uploadDashcamSegments([target], {
        showProgress: true,
        showResult: true,
        showSuccessToast: false,
      }),
      onSegmentMenu: (target, { close } = {}) => {
        return showDashcamSegmentMenu(route, target, { activePlayerClose: close });
      },
      onSegmentChange: (target) => {
        selectDashcamReadSegment(target);
      },
      onClose: () => {
        finishDashcamReadSegment();
      },
    },
  );
  fetchAllDashcamSegmentNames(route).then((allSegments) => {
    if (!playerController?.isOpen?.()) return;
    const currentEntry = (dashcamState.routes || []).find((item) => item.route === route) || entry;
    const completeSegments = mergeDashcamSegments(allSegments, orderedSegments, "asc");
    playerController.updateSegments?.(dashcamPlayerSegments(currentEntry, route, completeSegments));
  }).catch(() => {});
}

dashcamReadStateStore.subscribe(syncDashcamSegmentReadStatusUi);

async function openDashcamDriveReplay(route, segment) {
  const requestId = ++dashcamState.replayLoadSeq;
  const replay = window.CarrotVisionReplay;
  if (!replay?.start) {
    showAppToast(getUIText("replay_unavailable", "Drive replay is not available in this browser."), { tone: "error" });
    return;
  }
  const entry = (dashcamState.routes || []).find((item) => item.route === route);
  const segments = mergeDashcamSegments(dashcamSegmentsForRoute(entry), [], "asc");
  const titleForSegment = (target) => formatDashcamReplayTitle(entry, target);
  const startPromise = replay.start({
    route,
    segment,
    segments,
    title: titleForSegment(segment),
    titleForSegment,
  });
  fetchAllDashcamSegmentNames(route).then((allSegments) => {
    if (requestId !== dashcamState.replayLoadSeq || !replay.isActive?.()) return;
    replay.updateSegments?.(mergeDashcamSegments(allSegments, [], "asc"), route);
  }).catch(() => {});
  await startPromise;
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

function dashcamUploadConfirmHtml(stats) {
  const segmentCountLabel = getUIText("upload_segment_count", "{count} segments", {
    count: Number(stats?.segments || 0),
  });
  const qcameraLabel = getUIText("upload_kind_count", "{name} {count}", {
    name: "qcamera",
    count: Number(stats?.qcamera || 0),
  });
  const rlogLabel = getUIText("upload_kind_count", "{name} {count}", {
    name: "rlog",
    count: Number(stats?.rlog || 0),
  });
  const totalCountLabel = getUIText("upload_total_count", "{count} files total", {
    count: Number(stats?.files || 0),
  });
  const bytes = Number(stats?.bytes || 0);
  const sizeLabel = bytes > 0 ? formatLogBytes(bytes) : getUIText("upload_size_unknown", "size unknown");
  return `<section class="app-dialog__uploadBrief" aria-label="${escapeHtml(getUIText("upload_summary_label", "Upload summary"))}">
    <div class="app-dialog__uploadBriefCopy">
      <strong class="app-dialog__uploadBriefTitle">${escapeHtml(segmentCountLabel)}</strong>
      <span class="app-dialog__uploadBriefKinds">${escapeHtml(`${qcameraLabel} · ${rlogLabel}`)}</span>
    </div>
    <div class="app-dialog__uploadBriefAmount">
      <strong>${escapeHtml(sizeLabel)}</strong>
      <span>${escapeHtml(totalCountLabel)}</span>
    </div>
  </section>`;
}

function dashcamUploadResultHtml(result) {
  const results = Array.isArray(result?.results) ? result.results : [];
  const stats = dashcamUploadStats(results);
  const failedResults = results.filter((item) => item?.ok !== true);
  const visibleResults = results.length <= DASHCAM_UPLOAD_RESULT_VISIBLE_LIMIT
    ? results
    : failedResults.slice(0, DASHCAM_UPLOAD_RESULT_VISIBLE_LIMIT);
  const hiddenFailureCount = Math.max(0, failedResults.length - visibleResults.length);
  const segmentRows = visibleResults
    .map((item) => {
      const ok = item?.ok === true;
      const status = ok
        ? getUIText("upload_status_complete", "Complete")
        : getUIText("upload_status_failed", "Failed");
      return `<div class="app-dialog__metaResult">
        <code class="app-dialog__metaCode">${escapeHtml(String(item?.segment || ""))}</code>
        <span class="app-dialog__metaState" data-tone="${ok ? "success" : "error"}">${escapeHtml(status)}</span>
      </div>`;
    })
    .join("");
  const overflowRow = hiddenFailureCount > 0
    ? `<div class="app-dialog__metaOverflow">${escapeHtml(getUIText("upload_more_failed", "{count} more failed", {
      count: hiddenFailureCount,
    }))}</div>`
    : "";
  const resultList = segmentRows || overflowRow
    ? `<div class="app-dialog__metaResultList">${segmentRows}${overflowRow}</div>`
    : "";
  const qcameraLabel = getUIText("upload_kind_count", "{name} {count}", {
    name: "qcamera",
    count: Number(stats.qcamera || 0),
  });
  const rlogLabel = getUIText("upload_kind_count", "{name} {count}", {
    name: "rlog",
    count: Number(stats.rlog || 0),
  });
  return `<div class="app-dialog__metaList">
    <div class="app-dialog__metaLine">${escapeHtml(getUIText("upload_complete_count", "Upload complete {uploaded}/{total}", {
      uploaded: Number(result?.uploaded || 0),
      total: Number(result?.total || 0),
    }))}</div>
    ${resultList}
    <div class="app-dialog__metaSummary">
      <span>${escapeHtml(qcameraLabel)}</span>
      <span>${escapeHtml(rlogLabel)}</span>
      <strong>${escapeHtml(dashcamUploadSummaryLabel(stats))}</strong>
    </div>
  </div>`;
}

async function showDashcamUploadResult(result) {
  const text = String(result?.shareText || result?.message || "").trim();
  await openAppDialog({
    mode: "choice",
    title: getUIText("log_upload_result", "Upload Result"),
    html: true,
    messageHtml: dashcamUploadResultHtml(result),
    cancelLabel: getUIText("close", "Close"),
    copyText: text,
    copyLabel: getUIText("copy", "Copy"),
  });
}

function makeDashcamUploadCanceledError() {
  const error = new Error(getUIText("upload_canceled", "Upload canceled"));
  error.name = "DashcamUploadCanceled";
  return error;
}

function isDashcamUploadCanceledError(error) {
  return error?.name === "DashcamUploadCanceled" || /cancell?ed/i.test(String(error?.message || ""));
}

async function cancelDashcamUploadJob(jobId) {
  if (!jobId) return null;
  return postJson("/api/dashcam/upload/cancel", { id: jobId });
}

const DASHCAM_UPLOAD_PHASE_TEXT = Object.freeze({
  queued: ["upload_phase_queued", "Waiting to upload"],
  preparing: ["upload_phase_preparing", "Preparing files"],
  uploading: ["upload_phase_uploading", "Uploading files"],
  notifying: ["upload_phase_notifying", "Sending completion notice"],
  canceling: ["upload_phase_canceling", "Canceling upload"],
  complete: ["upload_phase_complete", "Upload complete"],
  canceled: ["upload_phase_canceled", "Upload canceled"],
  failed: ["upload_phase_failed", "Upload failed"],
});

function dashcamUploadPhaseLabel(phase, status = "") {
  let normalized = String(phase || "").trim().toLowerCase();
  if (!DASHCAM_UPLOAD_PHASE_TEXT[normalized]) {
    normalized = status === "failed"
      ? "failed"
      : status === "canceled"
        ? "canceled"
        : status === "done"
          ? "complete"
          : "uploading";
  }
  const [key, fallback] = DASHCAM_UPLOAD_PHASE_TEXT[normalized];
  return getUIText(key, fallback);
}

function dashcamUploadProgressView(snapshot = {}, totalFallback = 0, stats = null, stateOverride = null) {
  const current = Math.max(0, Number(snapshot.step_current || 0));
  const total = Math.max(0, Number(snapshot.step_total || totalFallback || 0));
  const phase = String(snapshot.phase || "preparing").trim().toLowerCase();
  const progressState = stateOverride || dashcamUploadProgressState(snapshot);
  const showSegmentCount = total > 0 && (phase === "queued" || phase === "preparing" || phase === "uploading");
  const segmentLabel = showSegmentCount
    ? getUIText("upload_segment_progress", "Segments {current}/{total}", { current, total })
    : "";
  const message = [
    dashcamUploadPhaseLabel(phase, snapshot.status),
    segmentLabel,
  ].filter(Boolean).join(" · ");
  const bytesCurrent = Math.max(0, Number(snapshot.bytes_current || 0));
  const bytesTotal = Math.max(0, Number(snapshot.bytes_total || stats?.bytes || 0));
  const bytesPerSecond = Math.max(0, Number(snapshot.bytes_per_second || 0));
  const isPreparing = phase === "queued" || phase === "preparing";
  let summary = stats ? dashcamUploadSummaryLabel(stats) : getUIText("loading", "Loading...");
  if (!isPreparing && bytesTotal > 0) {
    summary = `${formatLogBytes(Math.min(bytesCurrent, bytesTotal))} / ${formatLogBytes(bytesTotal)}`;
    if (bytesPerSecond > 0) summary += ` · ${formatLogBytes(bytesPerSecond)}/s`;
  }
  return {
    message,
    progressState,
    summary,
  };
}

function openDashcamUploadProgress(total, stats = null, options = {}) {
  const openedAt = Date.now();
  const model = createDashcamUploadProgressModel();
  const initialSnapshot = {
    phase: "queued",
    step_current: 0,
    step_total: total,
    progress: 0,
  };
  const initialState = model.update(initialSnapshot);
  const initial = dashcamUploadProgressView(
    initialSnapshot,
    total,
    stats,
    initialState.progressState,
  );
  const dialog = openAppProgressDialog({
    title: getUIText("log_uploading", "Uploading logs"),
    message: initial.message,
    progressState: initial.progressState,
    progressLabel: getUIText("upload_progress_label", "Upload progress"),
    summary: initial.summary,
    cancelLabel: options.cancelLabel || getUIText("cancel", "Cancel"),
    cancelingLabel: getUIText("upload_canceling", "Canceling..."),
    onCancel: options.onCancel,
  });
  return Object.freeze({
    completion: dialog.completion,
    setCancelHandler: dialog.setCancelHandler,
    setCanceling: dialog.setCanceling,
    setMessage: dialog.setMessage,
    setProgressState: dialog.setProgressState,
    update(snapshot, totalFallback = total) {
      const modelState = model.update(snapshot);
      if (!modelState.accepted) return;
      const view = dashcamUploadProgressView(
        snapshot,
        totalFallback,
        stats,
        modelState.progressState,
      );
      dialog.setMessage(view.message);
      dialog.setProgressState(view.progressState);
      dialog.setSummary(view.summary);
    },
    setSummary(nextStats) {
      const summary = typeof nextStats === "string"
        ? nextStats
        : nextStats
          ? dashcamUploadSummaryLabel(nextStats)
          : "";
      dialog.setSummary(summary);
    },
    async settle() {
      const remaining = DASHCAM_UPLOAD_PROGRESS_MIN_VISIBLE_MS - (Date.now() - openedAt);
      if (remaining > 0) await waitMs(remaining);
      await waitMs(DASHCAM_UPLOAD_PROGRESS_FINISH_HOLD_MS);
    },
    close: dialog.close,
  });
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
    progress.update(snapshot, totalFallback);
    if (snapshot.status === "canceled" || snapshot.result?.canceled) {
      throw makeDashcamUploadCanceledError();
    }
    if (snapshot.done) break;
    await waitMs(DASHCAM_UPLOAD_POLL_INTERVAL_MS);
  }

  const result = snapshot?.result || {};
  if (result.canceled) throw makeDashcamUploadCanceledError();
  if (!result || !Array.isArray(result.results)) {
    throw new Error(snapshot?.error || getUIText("error", "Error"));
  }
  return result;
}

async function resumeDashcamUploadJobIfNeeded(options = {}) {
  if (dashcamUploadResumePromise) return dashcamUploadResumePromise;
  const jobId = getRememberedDashcamUploadJob();
  const force = options.force === true;
  if (!jobId || (!force && jobId === dashcamUploadActiveJobId)) return null;

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
    let cancelRequested = false;
    const progress = openDashcamUploadProgress(total, null, {
      onCancel: async () => {
        cancelRequested = true;
        const cancelSnapshot = await cancelDashcamUploadJob(jobId);
        progress.update(cancelSnapshot, total);
      },
    });
    let activityId = typeof beginAppActivity === "function"
      ? beginAppActivity("logs", getUIText("log_uploading", "Uploading logs"))
      : null;

    try {
      progress.update(snapshot, total);
      const result = await pollDashcamUploadJob(jobId, progress, total);
      clearRememberedDashcamUploadJob(jobId);
      const resultStats = dashcamUploadStats(result.results || []);
      progress.update({
        phase: result.ok ? "complete" : "failed",
        status: result.ok ? "done" : "failed",
        step_current: Number(result.uploaded || 0),
        step_total: Number(result.total || total),
        progress: result.ok ? 100 : null,
        bytes_current: resultStats.bytes,
        bytes_total: resultStats.bytes,
      }, total);
      progress.setSummary(resultStats);
      await progress.settle();
      showAppToast(getUIText("upload_complete_count", "Upload complete {uploaded}/{total}", {
        uploaded: result.uploaded || 0,
        total: result.total || total,
      }), { tone: result.ok ? "default" : "error", duration: 3600 });
      await progress.close();
      if (activityId && typeof endAppActivity === "function") {
        endAppActivity(activityId);
        activityId = null;
      }
      await showDashcamUploadResult(result);
      return result;
    } catch (e) {
      progress.close();
      clearRememberedDashcamUploadJob(jobId);
      if (isDashcamUploadCanceledError(e)) {
        showAppToast(getUIText("upload_canceled", "Upload canceled"), { duration: 2600 });
      } else {
        showAppToast(`${getUIText("log_upload", "Upload Logs")} ${getUIText("error", "Error")}: ${e.message || e}`, { tone: "error", duration: 4200 });
      }
      return null;
    } finally {
      if (activityId && typeof endAppActivity === "function") endAppActivity(activityId);
      if (dashcamUploadActiveJobId === jobId) dashcamUploadActiveJobId = null;
      dashcamUploadResumePromise = null;
    }
  })();

  return dashcamUploadResumePromise;
}

async function uploadDashcamSegments(segments, options = {}) {
  if (dashcamUploadActiveJobId) {
    showAppToast(getUIText("upload_already_running", "Upload already running."), { tone: "error", duration: 3200 });
    return;
  }
  if (getRememberedDashcamUploadJob()) {
    resumeDashcamUploadJobIfNeeded().catch(() => {});
    return;
  }

  const targets = Array.from(new Set(segments || [])).filter(Boolean);
  if (!targets.length) {
    showAppToast(getUIText("no_selected_segments", "No segments selected."), { tone: "error" });
    return;
  }
  let uploadStats = {
    segments: targets.length,
    segmentNames: targets,
    files: 0,
    qcamera: 0,
    rlog: 0,
    bytes: 0,
  };
  if (options.confirm !== false) {
    try {
      const summary = await postJson("/api/dashcam/upload/summary", { segments: targets });
      if (!Array.isArray(summary?.summaries) || summary.summaries.length !== targets.length) {
        throw new Error(getUIText("upload_summary_unavailable", "Upload information is unavailable."));
      }
      uploadStats = dashcamUploadStats(summary.summaries);
    } catch (error) {
      showAppToast(`${getUIText("log_upload", "Upload Logs")} ${getUIText("error", "Error")}: ${error?.message || error}`, {
        tone: "error",
        duration: 4200,
      });
      return;
    }
    const ok = await appConfirm("", {
      title: getUIText("log_upload", "Upload Logs"),
      html: true,
      messageHtml: dashcamUploadConfirmHtml(uploadStats),
      confirmLabel: getUIText("upload_send", "Send"),
    });
    if (!ok) return;
  }
  const silentProgress = {
    setCancelHandler() {},
    setCanceling() {},
    setMessage() {},
    setProgressState() {},
    update() {},
    setSummary() {},
    settle() {
      return Promise.resolve();
    },
    close() {
      return Promise.resolve();
    },
  };
  let cancelRequested = false;
  const progress = options.showProgress === false
    ? silentProgress
    : openDashcamUploadProgress(targets.length, uploadStats, {
      onCancel: async () => {
        cancelRequested = true;
        if (jobId) {
          const cancelSnapshot = await cancelDashcamUploadJob(jobId);
          progress.update(cancelSnapshot, targets.length);
        } else {
          progress.update({
            phase: "canceling",
            step_current: 0,
            step_total: targets.length,
            progress: null,
          }, targets.length);
        }
      },
    });
  let activityId = options.showProgress !== false && typeof beginAppActivity === "function"
    ? beginAppActivity("logs", getUIText("log_uploading", "Uploading logs"))
    : null;
  let jobId = null;
  try {
    progress.update({
      phase: "preparing",
      step_current: 0,
      step_total: targets.length,
      progress: null,
    }, targets.length);
    if (cancelRequested) throw makeDashcamUploadCanceledError();
    const started = await postJson("/api/dashcam/upload/start", { segments: targets });
    jobId = started.job_id;
    rememberDashcamUploadJob(jobId);
    if (cancelRequested) {
      await cancelDashcamUploadJob(jobId);
      throw makeDashcamUploadCanceledError();
    }
    const result = await pollDashcamUploadJob(jobId, progress, targets.length);
    clearRememberedDashcamUploadJob(jobId);
    const resultStats = dashcamUploadStats(result.results || []);
    progress.update({
      phase: result.ok ? "complete" : "failed",
      status: result.ok ? "done" : "failed",
      step_current: Number(result.uploaded || 0),
      step_total: Number(result.total || targets.length),
      progress: result.ok ? 100 : null,
      bytes_current: resultStats.bytes,
      bytes_total: resultStats.bytes,
    }, targets.length);
    progress.setSummary(resultStats);
    await progress.settle();
    const message = getUIText("upload_complete_count", "Upload complete {uploaded}/{total}", {
      uploaded: result.uploaded || 0,
      total: result.total || targets.length,
    });
    if (options.showSuccessToast !== false) {
      showAppToast(message, {
        tone: result.ok ? "default" : "error",
        duration: Number(options.toastDuration) || 3600,
      });
    }
    await progress.close();
    if (activityId && typeof endAppActivity === "function") {
      endAppActivity(activityId);
      activityId = null;
    }
    if (options.showResult !== false) await showDashcamUploadResult(result);
  } catch (e) {
    progress.close();
    if (jobId) clearRememberedDashcamUploadJob(jobId);
    const runningJobId = e?.payload?.job_id || e?.payload?.job?.id || null;
    if (runningJobId) {
      rememberDashcamUploadJob(runningJobId);
      showAppToast(getUIText("upload_already_running", "Upload already running"), { tone: "error", duration: 3200 });
      resumeDashcamUploadJobIfNeeded({ force: true }).catch(() => {});
    } else if (isDashcamUploadCanceledError(e)) {
      showAppToast(getUIText("upload_canceled", "Upload canceled"), { duration: 2600 });
    } else {
      showAppToast(`${getUIText("log_upload", "Upload Logs")} ${getUIText("error", "Error")}: ${e.message || e}`, {
        tone: "error",
        duration: Number(options.toastDuration) || 4200,
      });
    }
  } finally {
    if (activityId && typeof endAppActivity === "function") endAppActivity(activityId);
    if (jobId && dashcamUploadActiveJobId === jobId) dashcamUploadActiveJobId = null;
  }
}

async function uploadRecentDashcamSegments(limit) {
  const count = Number(limit);
  if (![2, 5, 10].includes(count)) return;
  try {
    const payload = await getJson(`/api/dashcam/recent?limit=${count}`);
    const segments = Array.isArray(payload?.segments) ? payload.segments : [];
    if (!segments.length) {
      showAppToast(getUIText("no_completed_logs", "No completed logs available."), { tone: "error" });
      return;
    }
    await uploadDashcamSegments(segments);
  } catch (error) {
    showAppToast(`${getUIText("recent_log_upload", "Upload recent logs")}: ${error.message || error}`, {
      tone: "error",
      duration: 4200,
    });
  }
}

async function showDashcamSegmentMenu(route, segment, options = {}) {
  const entry = (dashcamState.routes || []).find((item) => item.route === route);
  const timeLabel = formatDashcamSegmentTimeLabel(entry, segment);
  const fullTime = formatDashcamSegmentFullTime(entry, segment);
  const message = fullTime ? `${segment}\n${fullTime}` : segment;
  const selected = await openAppDialog({
    mode: "choice",
    title: timeLabel,
    message,
    choiceLayout: "list",
    choices: [
      { label: getUIText("play", "Play"), value: "play" },
      { label: getUIText("drive_replay", "Replay"), value: "drive_replay" },
      { label: getUIText("log_upload", "Upload Logs"), value: "upload" },
      { label: `qcamera ${getUIText("download", "Download")}`, value: "download_qcamera" },
      { label: `rlog ${getUIText("download", "Download")}`, value: "download_rlog" },
      { label: `qlog ${getUIText("download", "Download")}`, value: "download_qlog" },
    ],
  });
  if (selected === "drive_replay") {
    options.activePlayerClose?.();
    await openDashcamDriveReplay(route, segment);
  } else if (selected === "play" && typeof options.activePlayerClose !== "function") openDashcamPlayer(route, segment);
  else if (selected === "upload") await uploadDashcamSegments([segment]);
  else if (selected?.startsWith?.("download_")) {
    const kind = selected.replace("download_", "");
    window.open(dashcamApiPath(`download/${encodeURIComponent(segment)}`, kind), "_blank", "noopener");
  }
}

// Parse a range expression like "1, 1-2, 1,2,3,4" into a Set of segment indices.
// Supports single numbers and "a-b" / "a~b" ranges (a>b is auto-swapped).
function parseDashcamRangeInput(input) {
  const out = new Set();
  String(input || "").split(",").forEach((token) => {
    const t = token.trim();
    if (!t) return;
    const single = t.match(/^(\d+)$/);
    if (single) {
      out.add(Number.parseInt(single[1], 10));
      return;
    }
    const range = t.match(/^(\d+)\s*[-~]\s*(\d+)$/);
    if (!range) return;
    let a = Number.parseInt(range[1], 10);
    let b = Number.parseInt(range[2], 10);
    if (a > b) [a, b] = [b, a];
    if (b - a > 100000) b = a + 100000; // guard against runaway ranges
    for (let i = a; i <= b; i += 1) out.add(i);
  });
  return out;
}

// Pull every segment folder name for a route (paged), so range selection works
// even for not-yet-loaded segments and skips gaps (missing indices).
async function fetchAllDashcamSegmentNames(route) {
  const all = [];
  const seen = new Set();
  const segmentTimes = {};
  let offset = 0;
  for (let guard = 0; guard < 1000; guard += 1) {
    const json = await getJson(`/api/dashcam/segments/${encodeURIComponent(route)}?offset=${offset}&limit=${DASHCAM_SEGMENT_NAME_LIMIT_MAX}&sort=${dashcamSortDirection()}`);
    const segs = Array.isArray(json.segments) ? json.segments : [];
    Object.assign(segmentTimes, json.segmentTimes || {});
    for (const segment of segs) {
      if (segment && !seen.has(segment)) {
        seen.add(segment);
        all.push(segment);
      }
    }
    if (!json.hasMore || !segs.length) break;
    offset = json.nextOffset == null ? all.length : (Number(json.nextOffset) || all.length);
  }
  const entry = (dashcamState.routes || []).find((item) => item.route === route);
  if (entry) entry.segmentTimes = { ...(entry.segmentTimes || {}), ...segmentTimes };
  return all;
}

async function showDashcamRangeSelect(route) {
  const entry = (dashcamState.routes || []).find((item) => item.route === route);
  if (!entry) return;

  // Load the full name list up front so we can show the available index range
  // in the dialog title and select segments that aren't lazily loaded yet.
  let names;
  try {
    names = await fetchAllDashcamSegmentNames(route);
  } catch {
    names = dashcamSegmentsForRoute(entry);
  }
  if (!names.length) names = dashcamSegmentsForRoute(entry);

  const presentIndices = names.map((name) => dashcamSegmentIndex(name));
  const minIndex = presentIndices.length ? Math.min(...presentIndices) : 0;
  const maxIndex = presentIndices.length ? Math.max(...presentIndices) : 0;
  const rangeLabel = presentIndices.length
    ? (minIndex === maxIndex ? `${minIndex}` : `${minIndex}–${maxIndex}`)
    : "";
  const title = rangeLabel
    ? `${getUIText("select_range", "Select range")} (${rangeLabel})`
    : getUIText("select_range", "Select range");

  const input = await appPrompt("", {
    title,
    placeholder: getUIText("range_input_hint", "1, 1-2, 1,2,3,4"),
  });
  if (input == null) return; // canceled
  const indices = parseDashcamRangeInput(input);
  if (!indices.size) {
    showAppToast(getUIText("range_invalid", "Enter a valid range"), { tone: "error" });
    return;
  }

  const current = (dashcamState.routes || []).find((item) => item.route === route);
  if (!current) return;

  const byIndex = new Map();
  names.forEach((name) => byIndex.set(dashcamSegmentIndex(name), name));
  let added = 0;
  indices.forEach((index) => {
    const name = byIndex.get(index);
    if (name && !dashcamState.selected.has(name)) {
      dashcamState.selected.add(name);
      added += 1;
    }
  });

  // Merge the full name list into the route so the selection count and tile
  // checkboxes reflect segments that weren't lazily loaded yet.
  if (names.length > dashcamSegmentsForRoute(current).length) {
    current.segmentFolders = mergeDashcamSegments(dashcamSegmentsForRoute(current), names);
    current.segmentCount = Math.max(dashcamSegmentCountForRoute(current), current.segmentFolders.length);
    current.segmentsHasMore = current.segmentFolders.length < current.segmentCount;
    current.segmentsNextOffset = current.segmentsHasMore ? current.segmentFolders.length : null;
    dashcamState.signature = dashcamRoutesSignature(dashcamState.routes);
  }

  if (!renderDashcamRoute(route)) renderDashcamRoutes({ animate: false });
  const total = dashcamSelectedForRoute(current).length;
  showAppToast(getUIText("range_selected", "{count} selected", { count: total }), {
    tone: added ? "default" : "error",
  });
}

async function setDashcamSort(next) {
  const dir = next === "desc" ? "desc" : "asc";
  // No menu state to refresh: the logs menu reads the direction when it opens.
  if (dashcamState.sort === dir) return;
  dashcamState.sort = dir;
  try {
    localStorage.setItem(DASHCAM_SORT_STORAGE_KEY, dir);
  } catch {}

  // Reorder sub-segments in place — no route-list reload (no full-page refresh).
  // Fully-loaded visible groups flip instantly (no network). Partially-loaded
  // visible groups keep their order until a single background fetch completes.
  // Off-screen / empty groups drop their page and lazy-load fresh in the new
  // order when shown (offset paging is order-relative).
  const routes = dashcamState.routes || [];
  const needFull = [];
  routes.forEach((entry) => {
    const rendered = dashcamState.expanded.has(entry.route) || isCompactLandscapeMode();
    const loaded = dashcamSegmentsForRoute(entry).length;
    if (rendered && loaded > 0) {
      if (dashcamRouteHasMoreSegments(entry)) {
        needFull.push(entry);
      } else {
        entry.segmentFolders = mergeDashcamSegments(dashcamSegmentsForRoute(entry), [], dir);
      }
    } else {
      const total = dashcamSegmentCountForRoute(entry);
      entry.segmentFolders = [];
      entry.segmentsNextOffset = 0;
      entry.segmentsHasMore = total > 0;
    }
  });

  const host = document.getElementById("dashcamRoutes");
  dashcamState.signature = dashcamRoutesSignature(routes);
  if (host) host.dataset.signature = "";
  if (isLogsPageActive()) renderDashcamRoutes({ animate: false });
  requestAnimationFrame(() => maybeLoadVisibleDashcamSegments());

  if (!needFull.length) return;
  await Promise.all(needFull.map(async (entry) => {
    let names = [];
    try {
      names = await fetchAllDashcamSegmentNames(entry.route);
    } catch {
      names = [];
    }
    if (dashcamSortDirection() !== dir) return; // sort changed again mid-flight
    if (names.length) {
      entry.segmentFolders = mergeDashcamSegments(names, [], dir);
      entry.segmentCount = Math.max(dashcamSegmentCountForRoute(entry), entry.segmentFolders.length);
      entry.segmentsHasMore = entry.segmentFolders.length < entry.segmentCount;
      entry.segmentsNextOffset = entry.segmentsHasMore ? entry.segmentFolders.length : null;
    } else {
      entry.segmentFolders = mergeDashcamSegments(dashcamSegmentsForRoute(entry), [], dir);
    }
  }));
  if (dashcamSortDirection() !== dir) return;
  dashcamState.signature = dashcamRoutesSignature(routes);
  if (host) host.dataset.signature = "";
  if (isLogsPageActive()) renderDashcamRoutes({ animate: false });
}

// "Select all" for a group: pull the full segment list (covers not-yet-loaded
// segments) then select every one. Clearing only needs the loaded set.
async function toggleDashcamRouteSelectAll(route, shouldClear) {
  const entry = (dashcamState.routes || []).find((item) => item.route === route);
  if (!entry) return;

  if (shouldClear) {
    for (const item of dashcamSegmentsForRoute(entry)) dashcamState.selected.delete(item);
    if (!updateDashcamRouteSelectionUi(route)) renderDashcamRoutes({ animate: false });
    return;
  }

  let names;
  try {
    names = await fetchAllDashcamSegmentNames(route);
  } catch {
    names = dashcamSegmentsForRoute(entry);
  }
  const current = (dashcamState.routes || []).find((item) => item.route === route);
  if (!current) return;
  if (!names.length) names = dashcamSegmentsForRoute(current);

  if (names.length > dashcamSegmentsForRoute(current).length) {
    current.segmentFolders = mergeDashcamSegments(dashcamSegmentsForRoute(current), names);
    current.segmentCount = Math.max(dashcamSegmentCountForRoute(current), current.segmentFolders.length);
    current.segmentsHasMore = current.segmentFolders.length < current.segmentCount;
    current.segmentsNextOffset = current.segmentsHasMore ? current.segmentFolders.length : null;
    dashcamState.signature = dashcamRoutesSignature(dashcamState.routes);
  }

  for (const item of dashcamSegmentsForRoute(current)) dashcamState.selected.add(item);
  if (!renderDashcamRoute(route)) renderDashcamRoutes({ animate: false });
}

async function showDashcamRouteMenu(route) {
  if (!route) return;
  const selected = await openAppDialog({
    mode: "choice",
    title: getUIText("group_menu", "Group menu"),
    message: dashcamRouteTitle(route),
    choiceLayout: "list",
    choices: [
      { label: getUIText("route_summary", "주행요약"), value: "summary" },
      { label: `${getUIText("select_range", "Select range")}…`, value: "range" },
    ],
  });
  if (selected === "summary") await openRouteSummary(route);
  else if (selected === "range") await showDashcamRangeSelect(route);
}

export {
  cancelDashcamRouteRender,
  dashcamDefaultRouteHeight,
  dashcamLayoutKey,
  loadDashcamReadState,
  dashcamSelectedForRoute,
  dashcamSortDirection,
  dashcamState,
  dashcamWindowNeedsRender,
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
};
