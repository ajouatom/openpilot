"use strict";

// Logs page — Screen Recording tab.
// Virtual list of saved screen recordings with lazy thumbnails, paged loading,
// and a download/playback action row.

const SCREENRECORD_PAGE_SIZE = 40;
const SCREENRECORD_LOAD_AHEAD_PX = 720;
const SCREENRECORD_WINDOW_OVERSCAN = 8;

const screenrecordState = {
  initialized: false,
  loading: false,
  loadingMore: false,
  loadSeq: 0,
  videos: [],
  rowHeight: 80,
  windowStart: 0,
  windowEnd: 0,
  total: 0,
  nextOffset: 0,
  hasMore: false,
  signature: "",
  renderFrame: 0,
};

function setScreenrecordStatus(message, tone = "") {
  const status = document.getElementById("screenrecordStatus");
  if (!status) return;
  status.textContent = message || "";
  status.hidden = !message;
  status.classList.toggle("is-error", tone === "error");
}

function screenrecordApiPath(kind, fileId) {
  return `/api/screenrecord/${kind}/${encodeURIComponent(fileId)}`;
}

function screenrecordVideosSignature(videos) {
  return (videos || []).map((video) => [
    video.id || "",
    video.name || "",
    video.modifiedLabel || "",
    video.size || 0,
  ].join("|")).join("\n") + "|" + (typeof LANG !== "undefined" ? LANG : "");
}

function screenrecordShouldLoadMore(scroller) {
  if (!scroller || !screenrecordState.hasMore || screenrecordState.loading || screenrecordState.loadingMore) return false;
  const remaining = scroller.scrollHeight - scroller.scrollTop - scroller.clientHeight;
  return remaining <= SCREENRECORD_LOAD_AHEAD_PX;
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

function openScreenrecordPlayer(id, name) {
  if (!id) return;
  openLogsVideoPlayer(name || getUIText("logs_screenrecord", "Screen Record"), screenrecordApiPath("video", id), { kind: "screenrecord" });
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
