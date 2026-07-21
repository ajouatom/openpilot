import { createTelemetryForwardSurface } from "../telemetry/forward_surface.js";
import {
  TELEMETRY_FORWARD_SOURCE_ORDER,
  TELEMETRY_FORWARD_SOURCE_STYLE,
  normalizeTelemetryForwardSource,
  telemetryForwardSourceLabel,
} from "../telemetry/forward_sources.js";

"use strict";

window.CarrotReplaySensorTopview = window.CarrotReplaySensorTopview || (() => {
  const SENSOR_WORKER_URL = "/js/realtime/replay_sensor_worker.js?v=2607-01";
  const rootEl = document.getElementById("carrotReplaySensorTopview");
  const viewportEl = document.getElementById("carrotReplaySensorViewport");
  const plotEl = document.getElementById("carrotReplaySensorPlot");
  const canvasEl = document.getElementById("carrotReplaySensorCanvas");
  const egoEl = document.getElementById("carrotReplaySensorEgo");
  const legendEl = document.getElementById("carrotReplaySensorLegend");
  const messageEl = document.getElementById("carrotReplaySensorMessage");
  const tooltipEl = document.getElementById("carrotReplaySensorTooltip");
  const titleEl = document.getElementById("carrotReplaySensorTitle");
  const statusEl = document.getElementById("carrotReplaySensorStatus");
  const surface = createTelemetryForwardSurface({
    adopt: true,
    root: rootEl,
    viewport: viewportEl,
    plot: plotEl,
    canvas: canvasEl,
    ego: egoEl,
    legend: legendEl,
    message: messageEl,
    tooltip: tooltipEl,
    title: titleEl,
    status: statusEl,
  });

  const MAX_DISTANCE_M = 100;
  const HALF_WIDTH_M = 12;
  const LANE_HALF_WIDTH_M = 1.8;
  const RADAR_TO_CAMERA_M = 1.52;
  const GRID_STEP_M = 20;
  const GRID_MINOR_M = 10;
  const GRID_LATERAL_M = 4;
  const TRACK_STRIDE = 8;
  const FRAME_MAX_AGE_MS = 600;
  const HIT_RADIUS_PX = 20;
  const MARK_RADIUS_PX = 5.5;
  const MARK_RADIUS_LEAD_PX = 7;
  const SOURCES = TELEMETRY_FORWARD_SOURCE_ORDER;
  const SOURCE_STYLE = TELEMETRY_FORWARD_SOURCE_STYLE;

  // Every colour resolves from the app's own tokens at runtime, the same way
  // replay_insights.js paints its event rail and graphs. Nothing here is a
  // hardcoded hex, so retheming the app retheme this panel too.
  const palette = {
    lead: "#ffb06d",
    vision: "#7dd3fc",
    track: "#7c8594",
    grid: "rgba(124, 133, 148, 0.2)",
    gridMinor: "rgba(124, 133, 148, 0.09)",
    lane: "rgba(238, 242, 248, 0.55)",
    label: "rgba(238, 242, 248, 0.5)",
    leadLabel: "rgba(238, 242, 248, 0.96)",
    plate: "#090c10",
  };
  const gridCanvas = document.createElement("canvas");
  const gridContext = gridCanvas.getContext?.("2d") || null;
  let gridCacheKey = "";

  function canvasToken(name, fallback) {
    const value = window.getComputedStyle?.(document.documentElement)?.getPropertyValue(name)?.trim();
    return value || fallback;
  }

  // Tokens are plain hex, so alpha is applied here rather than via color-mix():
  // an unregistered custom property computes to its literal token string, which
  // canvas would not accept as a colour.
  function withAlpha(color, alpha) {
    const match = /^#([0-9a-f]{3}|[0-9a-f]{6})$/i.exec(String(color).trim());
    if (!match) return color;
    const hex = match[1].length === 3
      ? match[1].split("").map((part) => part + part).join("")
      : match[1];
    const value = Number.parseInt(hex, 16);
    return `rgba(${(value >> 16) & 255}, ${(value >> 8) & 255}, ${value & 255}, ${alpha})`;
  }

  function readPalette() {
    const primary = canvasToken("--md-primary", "#ffb06d");
    const info = canvasToken("--md-info", "#7dd3fc");
    const outline = canvasToken("--md-outline-var", "#7c8594");
    const ink = canvasToken("--md-on-surface-var", "#eef2f8");
    palette.lead = primary;
    palette.vision = info;
    palette.track = outline;
    palette.grid = withAlpha(outline, 0.20);
    palette.gridMinor = withAlpha(outline, 0.09);
    palette.lane = withAlpha(ink, 0.55);
    palette.label = withAlpha(ink, 0.50);
    palette.leadLabel = withAlpha(ink, 0.96);
    palette.plate = canvasToken("--md-surface", "#090c10");
    gridCacheKey = "";
  }

  function toneColor(tone) {
    if (tone === "lead") return palette.lead;
    if (tone === "vision") return palette.vision;
    return palette.track;
  }

  function markTone(target) {
    if (target.selected) return "lead";
    return SOURCE_STYLE[target.source]?.tone || "track";
  }

  const state = {
    token: 0,
    records: [],
    decodeRecord: null,
    manifest: null,
    series: { modelV2: [], radarState: [], liveTracks: [] },
    status: "idle",
    trackStatus: "idle",
    trackWorker: null,
    visible: false,
    currentMs: 0,
    rangeM: MAX_DISTANCE_M,
    ctx: null,
    monoFont: "",
    leadFontPx: 13,
    resizeObserver: null,
    drawRaf: 0,
    scrollNearPinned: true,
    hitTargets: [],
    tooltipTimer: 0,
    // Retained pointer, so hover survives marks moving during playback.
    pointer: { x: 0, y: 0, active: false, pinned: false },
  };

  function text(key, fallback) {
    return typeof getUIText === "function" ? getUIText(key, fallback) : fallback;
  }

  function sourceLabel(source) {
    return telemetryForwardSourceLabel(source, text);
  }

  function setMessage(message = "") {
    surface?.setMessage(message);
  }

  function hasAnySeries() {
    return Boolean(
      state.series.modelV2.length || state.series.radarState.length || state.series.liveTracks.length,
    );
  }

  // Radar-track loading is a side channel: it must never blank the panel, but it
  // also must not fail silently the way the old status readout did.
  function trackNote() {
    if (state.trackStatus === "loading") return text("replay_sensor_tracks_loading", "loading radar");
    if (state.trackStatus === "error") return text("replay_sensor_tracks_error", "radar unreadable");
    if (state.trackStatus === "unavailable") return text("replay_sensor_tracks_unavailable", "no radar log");
    return "";
  }

  function syncHeaderStatus() {
    const range = `${state.rangeM} m`;
    const note = trackNote();
    surface?.setHeader(
      titleEl?.textContent || text("replay_surroundings", "Forward perception"),
      note ? `${range} · ${note}` : range,
      state.trackStatus === "error",
    );
  }

  function syncMessage() {
    if (state.status === "loading") {
      setMessage(text("replay_sensor_loading", "Reading sensor data..."));
      return;
    }
    if (state.status === "error") {
      setMessage(text("replay_sensor_failed", "Sensor data could not be read."));
      return;
    }
    if (state.status === "ready" && !hasAnySeries()) {
      setMessage(text("replay_sensor_empty", "This segment has no sensor data."));
      return;
    }
    setMessage("");
  }

  function syncStatus() {
    syncHeaderStatus();
    syncMessage();
  }

  // Generated from SOURCE_STYLE so a legend glyph can never drift from the mark
  // the canvas actually paints.
  function renderLegend() {
    surface?.setLegend(SOURCES.map((source) => {
      const style = SOURCE_STYLE[source];
      return { shape: style.shape, color: toneColor(style.tone), label: sourceLabel(source) };
    }), text("replay_sensor_sources", "Sensor sources"));
  }

  function normalizeSource(value) {
    return normalizeTelemetryForwardSource(value, "front");
  }

  function forEachTrack(decoded, callback) {
    if (Array.isArray(decoded)) {
      decoded.forEach(callback);
      return;
    }
    const compact = Array.isArray(decoded?.tracks) ? decoded.tracks : null;
    if (compact) {
      const stride = Math.max(TRACK_STRIDE, Number(decoded?.stride) || TRACK_STRIDE);
      for (let index = 0; index + TRACK_STRIDE - 1 < compact.length; index += stride) {
        callback({
          trackId: Number(compact[index]) || 0,
          dRel: Number(compact[index + 1]),
          yRel: Number(compact[index + 2]),
          vRel: Number(compact[index + 3]),
          aRel: Number(compact[index + 4]),
          yvRel: Number(compact[index + 5]),
          measured: Boolean(compact[index + 6]),
          source: normalizeSource(compact[index + 7]),
        });
      }
      return;
    }
    for (const point of Array.isArray(decoded?.points) ? decoded.points : []) {
      callback({
        trackId: Number(point?.trackId) || 0,
        dRel: Number(point?.dRel),
        yRel: Number(point?.yRel),
        vRel: Number(point?.vRel),
        aRel: Number(point?.aRel),
        yvRel: Number(point?.yvRel),
        measured: Boolean(point?.measured),
        source: normalizeSource(point?.radarSource),
      });
    }
  }

  function scheduleWork(callback) {
    if (typeof window.requestIdleCallback === "function") {
      window.requestIdleCallback(callback, { timeout: 90 });
    } else {
      window.setTimeout(() => callback({ timeRemaining: () => 8 }), 0);
    }
  }

  function prepare() {
    if (state.status === "loading" || state.status === "ready" || !state.records.length) {
      if (!state.records.length && state.status !== "ready") {
        state.status = "ready";
        ensureTrackFrames();
        syncStatus();
        requestDraw();
      }
      return;
    }
    const token = state.token;
    state.status = "loading";
    syncStatus();
    ensureTrackFrames();
    let index = 0;
    const scan = (deadline) => {
      if (token !== state.token) return;
      const start = index;
      while (index < state.records.length && (index - start < 120 || deadline.timeRemaining() > 1)) {
        const record = state.records[index++];
        let frames = [];
        try { frames = state.decodeRecord?.(record) || []; } catch {}
        for (const frame of frames) {
          if (!state.series[frame?.service] || !frame?.decoded) continue;
          const sample = { timeMs: Math.max(0, Number(record?.timeMs) || 0), decoded: frame.decoded };
          state.series[frame.service].push(sample);
        }
      }
      if (index < state.records.length) {
        scheduleWork(scan);
        return;
      }
      state.status = "ready";
      ensureTrackFrames();
      syncStatus();
      requestDraw();
    };
    scheduleWork(scan);
  }

  function ensureTrackFrames() {
    if (state.trackStatus !== "idle") return;
    if (state.series.liveTracks.length) {
      state.trackStatus = "ready";
      syncStatus();
      return;
    }
    const rlog = state.manifest?.rlog;
    const baseMonoTime = state.manifest?.rawFirstMonoTimeNanos;
    if (!rlog?.url || !rlog?.compression || !baseMonoTime || typeof Worker !== "function") {
      state.trackStatus = "unavailable";
      syncStatus();
      return;
    }
    const token = state.token;
    let worker = null;
    try { worker = new Worker(SENSOR_WORKER_URL); } catch {
      state.trackStatus = "error";
      syncStatus();
      return;
    }
    state.trackWorker = worker;
    state.trackStatus = "loading";
    syncStatus();
    const finish = () => {
      if (state.trackWorker === worker) state.trackWorker = null;
      worker.terminate();
    };
    worker.onmessage = (event) => {
      if (token !== state.token) {
        finish();
        return;
      }
      const payload = event?.data || {};
      if (payload.type === "complete") {
        state.series.liveTracks = Array.isArray(payload.frames) ? payload.frames : [];
        state.trackStatus = "ready";
        finish();
        syncStatus();
        requestDraw();
      } else if (payload.type === "error") {
        state.trackStatus = "error";
        finish();
        syncStatus();
      }
    };
    worker.onerror = () => {
      if (token !== state.token) return;
      state.trackStatus = "error";
      finish();
      syncStatus();
    };
    worker.postMessage({
      type: "start",
      url: rlog.url,
      compression: rlog.compression,
      baseMonoTime,
      durationMs: Number(state.manifest?.durationMs) || 0,
    });
  }

  function initializeContext() {
    if (state.ctx) return true;
    if (!canvasEl) return false;
    const ctx = canvasEl.getContext("2d");
    if (!ctx) {
      state.status = "error";
      syncStatus();
      return false;
    }
    state.ctx = ctx;
    state.monoFont = canvasToken("--font-mono", "ui-monospace, Menlo, monospace");
    state.leadFontPx = Math.round(Number.parseFloat(canvasToken("--fs-label-sm", "13px"))) || 13;
    readPalette();
    return true;
  }

  function viewportSize() {
    const rect = (plotEl || viewportEl)?.getBoundingClientRect();
    return {
      width: Math.max(1, Math.round(rect?.width || 0)),
      height: Math.max(1, Math.round(rect?.height || 0)),
    };
  }

  function resizeCanvas(size) {
    if (!canvasEl) return 1;
    const pixelRatio = Math.min(2, Math.max(1, Number(window.devicePixelRatio) || 1));
    const width = Math.round(size.width * pixelRatio);
    const height = Math.round(size.height * pixelRatio);
    if (canvasEl.width !== width) canvasEl.width = width;
    if (canvasEl.height !== height) canvasEl.height = height;
    state.ctx.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
    return pixelRatio;
  }

  // The two ends are anchored differently on purpose.
  // Below dRel 0 there is nothing to show but your own bumper, so that end is a
  // fixed pixel budget - exactly the ego glyph - and never grows into dead space.
  // Above the far range the emptiness IS the point: it reads as headroom, so that
  // end keeps a ratio and scales with the panel.
  // Lateral stays intentionally exaggerated relative to range (a radar-display
  // convention); the distance labels drawn in drawGrid are what make that
  // compression legible instead of guesswork.
  const EGO_GLYPH_HEIGHT_PX = 40;
  const EGO_BOTTOM_MARGIN_PX = 6;
  const TOP_MARGIN_RATIO = 0.16;
  const MIN_PX_PER_M = 2.4;
  // CSS uses this computed minimum as the overflow height, preserving the full
  // semantic range instead of silently changing the axis to 40/60 m.
  const MIN_PLOT_HEIGHT_PX = Math.ceil(
    (MAX_DISTANCE_M * MIN_PX_PER_M + EGO_GLYPH_HEIGHT_PX + EGO_BOTTOM_MARGIN_PX)
      / (1 - TOP_MARGIN_RATIO),
  );
  rootEl?.style.setProperty("--telemetry-forward-plot-min-height", `${MIN_PLOT_HEIGHT_PX}px`);

  function nearY(size) {
    return Math.max(size.height * 0.5, size.height - EGO_GLYPH_HEIGHT_PX - EGO_BOTTOM_MARGIN_PX);
  }

  function farY(size) {
    return size.height * TOP_MARGIN_RATIO;
  }

  function project(dRel, yRel, size) {
    const near = nearY(size);
    const far = farY(size);
    return [
      size.width * 0.5 - (Number(yRel) / HALF_WIDTH_M) * 0.45 * size.width,
      near - (Number(dRel) / state.rangeM) * (near - far),
    ];
  }

  function drawGrid(ctx, size) {
    ctx.lineWidth = 1;

    // Verticals run the full canvas height, including the headroom above the
    // range and the ego's strip below it. That is what stops the plot reading as
    // a void with a few rules floating in it - the mesh is continuous even where
    // there is no range to label.
    ctx.strokeStyle = palette.gridMinor;
    ctx.beginPath();
    for (let lateral = GRID_LATERAL_M; lateral <= HALF_WIDTH_M; lateral += GRID_LATERAL_M) {
      for (const side of [-lateral, lateral]) {
        const x = Math.round(project(0, side, size)[0]) + 0.5;
        ctx.moveTo(x, 0);
        ctx.lineTo(x, size.height);
      }
    }
    // Minor range rules between the labelled ones.
    for (let distance = 0; distance <= state.rangeM; distance += GRID_MINOR_M) {
      if (distance % GRID_STEP_M === 0) continue;
      const y = Math.round(project(distance, 0, size)[1]) + 0.5;
      ctx.moveTo(0, y);
      ctx.lineTo(size.width, y);
    }
    ctx.stroke();

    ctx.strokeStyle = palette.grid;
    ctx.beginPath();
    for (let distance = 0; distance <= state.rangeM; distance += GRID_STEP_M) {
      const y = Math.round(project(distance, 0, size)[1]) + 0.5;
      ctx.moveTo(0, y);
      ctx.lineTo(size.width, y);
    }
    ctx.stroke();

    // The ego lane is the one reference every reading is judged against, so it
    // is the brightest thing on the plot and runs the full height unbroken -
    // fading it with distance made the corridor dissolve exactly where the far
    // marks needed something to sit against.
    ctx.strokeStyle = palette.lane;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    for (const side of [-LANE_HALF_WIDTH_M, LANE_HALF_WIDTH_M]) {
      const x = Math.round(project(0, side, size)[0]) + 0.5;
      ctx.moveTo(x, 0);
      ctx.lineTo(x, size.height);
    }
    ctx.stroke();
    ctx.lineWidth = 1;

    // 0 m is labelled too: it is the ego's own bumper and the datum every other
    // reading is relative to, so leaving it implicit made the scale start nowhere.
    ctx.fillStyle = palette.label;
    ctx.font = `600 10px ${state.monoFont}`;
    ctx.textAlign = "left";
    ctx.textBaseline = "bottom";
    for (let distance = 0; distance <= state.rangeM; distance += GRID_STEP_M) {
      ctx.fillText(`${distance} m`, 6, project(distance, 0, size)[1] - 3);
    }
  }

  function drawGridLayer(ctx, size, pixelRatio) {
    if (!gridContext || typeof ctx.drawImage !== "function") {
      drawGrid(ctx, size);
      return;
    }
    const pixelWidth = Math.max(1, Math.round(size.width * pixelRatio));
    const pixelHeight = Math.max(1, Math.round(size.height * pixelRatio));
    const key = [
      pixelWidth,
      pixelHeight,
      state.rangeM,
      state.monoFont,
      palette.grid,
      palette.gridMinor,
      palette.lane,
      palette.label,
    ].join("|");
    if (key !== gridCacheKey) {
      gridCacheKey = key;
      if (gridCanvas.width !== pixelWidth) gridCanvas.width = pixelWidth;
      if (gridCanvas.height !== pixelHeight) gridCanvas.height = pixelHeight;
      gridContext.setTransform?.(pixelRatio, 0, 0, pixelRatio, 0, 0);
      gridContext.clearRect?.(0, 0, size.width, size.height);
      drawGrid(gridContext, size);
    }
    ctx.drawImage(gridCanvas, 0, 0, pixelWidth, pixelHeight, 0, 0, size.width, size.height);
  }

  // The ego marker is static chrome, so it lives in the DOM as an inline SVG
  // (matching the app's icon language) instead of being hand-carved from canvas
  // paths. Only its anchor tracks the projection.
  // dRel 0 is the ego's nose, and the glyph is exactly the bottom budget, so it
  // lands flush with the viewport bottom with no gap to spare.
  function syncEgo(size) {
    if (!egoEl) return;
    const top = `${nearY(size)}px`;
    if (egoEl.style.top !== top) egoEl.style.top = top;
  }

  function pathForShape(ctx, x, y, shape, radius) {
    ctx.beginPath();
    if (shape === "square") {
      ctx.rect(x - radius, y - radius, radius * 2, radius * 2);
    } else if (shape === "diamond") {
      ctx.moveTo(x, y - radius * 1.3);
      ctx.lineTo(x + radius * 1.15, y);
      ctx.lineTo(x, y + radius * 1.3);
      ctx.lineTo(x - radius * 1.15, y);
    } else if (shape === "triangle") {
      ctx.moveTo(x, y - radius * 1.35);
      ctx.lineTo(x + radius * 1.25, y + radius * 0.9);
      ctx.lineTo(x - radius * 1.25, y + radius * 0.9);
    } else {
      ctx.arc(x, y, radius, 0, Math.PI * 2);
    }
    ctx.closePath();
  }

  // fill = measured. A hollow mark is an unmeasured/extrapolated detection, so
  // confidence stays on its own channel instead of stealing the shape or tone.
  function drawMark(ctx, target) {
    const style = SOURCE_STYLE[target.source] || SOURCE_STYLE.fusion;
    const color = toneColor(markTone(target));
    pathForShape(ctx, target.screenX, target.screenY, style.shape, target.radius);
    ctx.lineJoin = "round";
    ctx.lineWidth = 3.5;
    ctx.strokeStyle = palette.plate;
    ctx.stroke();
    if (target.measured) {
      ctx.fillStyle = color;
      ctx.fill();
    }
    ctx.lineWidth = 2;
    ctx.strokeStyle = color;
    ctx.stroke();
  }

  // Closing speed is the point of a topview, so leads carry it directly instead
  // of hiding it behind a tooltip. Only leads are labeled - never every point.
  // Distance keeps the same precision as the tooltip so one target never reads
  // as two different numbers.
  function drawLeadLabel(ctx, target, size) {
    const relative = Number(target.vRel);
    const label = Number.isFinite(relative)
      ? `${target.dRel.toFixed(1)} m  ${relative >= 0 ? "+" : ""}${relative.toFixed(1)}`
      : `${target.dRel.toFixed(1)} m`;
    ctx.font = `800 ${state.leadFontPx}px ${state.monoFont}`;
    ctx.textBaseline = "middle";
    const width = ctx.measureText(label).width;
    const gap = target.radius + 14;
    const fitsRight = target.screenX + gap + width + 4 <= size.width;
    ctx.textAlign = fitsRight ? "left" : "right";
    const x = fitsRight ? target.screenX + gap : target.screenX - gap;
    const y = Math.max(8, Math.min(size.height - 8, target.screenY));
    ctx.lineWidth = 3;
    ctx.lineJoin = "round";
    ctx.strokeStyle = palette.plate;
    ctx.strokeText(label, x, y);
    ctx.fillStyle = palette.leadLabel;
    ctx.fillText(label, x, y);
  }

  function framePairAt(series, timeMs, interpolationGapMs = 180) {
    if (!series.length) return null;
    let low = 0;
    let high = series.length;
    while (low < high) {
      const middle = (low + high) >> 1;
      if (series[middle].timeMs <= timeMs) low = middle + 1;
      else high = middle;
    }
    if (low === 0 && series[0].timeMs > timeMs) return null;
    const previous = series[Math.max(0, low - 1)];
    if (!previous || timeMs - previous.timeMs > FRAME_MAX_AGE_MS) return null;
    const next = low < series.length ? series[low] : null;
    const gap = next ? next.timeMs - previous.timeMs : 0;
    const ratio = next && gap > 0 && gap <= interpolationGapMs
      ? Math.max(0, Math.min(1, (timeMs - previous.timeMs) / gap))
      : 0;
    return { previous, next: ratio > 0 ? next : null, ratio };
  }

  function sampleAt(series, timeMs) {
    return framePairAt(series, timeMs)?.previous?.decoded || null;
  }

  function interpolateNumber(first, second, ratio) {
    const left = Number(first);
    const right = Number(second);
    if (!Number.isFinite(left)) return Number.isFinite(right) ? right : 0;
    if (!Number.isFinite(right)) return left;
    return left + (right - left) * ratio;
  }

  function interpolatedVisionAt(series, timeMs) {
    const pair = framePairAt(series, timeMs);
    if (!pair) return null;
    const first = pair.previous.decoded || {};
    const second = pair.next?.decoded || {};
    const firstLeads = Array.isArray(first.leadsV3) ? first.leadsV3 : [];
    const secondLeads = Array.isArray(second.leadsV3) ? second.leadsV3 : [];
    const interpolateLeadValue = (lead, next, key) => [
      interpolateNumber(lead?.[key]?.[0], next?.[key]?.[0], pair.ratio),
    ];
    return { leadsV3: firstLeads.map((lead, index) => {
      const next = secondLeads[index];
      if (!next || pair.ratio <= 0) return lead;
      return {
        ...lead,
        prob: interpolateNumber(lead.prob, next.prob, pair.ratio),
        x: interpolateLeadValue(lead, next, "x"),
        y: interpolateLeadValue(lead, next, "y"),
        v: interpolateLeadValue(lead, next, "v"),
        a: interpolateLeadValue(lead, next, "a"),
      };
    }) };
  }

  function interpolatedTracksAt(series, timeMs) {
    const pair = framePairAt(series, timeMs);
    if (!pair) return null;
    const previous = [];
    forEachTrack(pair.previous.decoded, (track) => previous.push(track));
    if (!pair.next || pair.ratio <= 0) return previous;
    const upcoming = new Map();
    forEachTrack(pair.next.decoded, (track) => upcoming.set(`${track.source}:${track.trackId}`, track));
    return previous.map((track) => {
      const next = upcoming.get(`${track.source}:${track.trackId}`);
      if (!next) return track;
      return {
        ...track,
        dRel: interpolateNumber(track.dRel, next.dRel, pair.ratio),
        yRel: interpolateNumber(track.yRel, next.yRel, pair.ratio),
        vRel: interpolateNumber(track.vRel, next.vRel, pair.ratio),
        aRel: interpolateNumber(track.aRel, next.aRel, pair.ratio),
        yvRel: interpolateNumber(track.yvRel, next.yvRel, pair.ratio),
      };
    });
  }

  function radarLeads(radarState) {
    const leads = [];
    const seen = new Set();
    const append = (lead) => {
      if (!lead?.status) return;
      const dRel = Number(lead.dRel);
      const yRel = Number(lead.yRel);
      if (!Number.isFinite(dRel) || !Number.isFinite(yRel)) return;
      const trackId = Number(lead.radarTrackId);
      const key = Number.isFinite(trackId) && trackId >= 0
        ? `track:${trackId}`
        : `position:${dRel.toFixed(1)}:${yRel.toFixed(1)}`;
      if (seen.has(key)) return;
      seen.add(key);
      leads.push({ ...lead, dRel, yRel, trackId });
    };
    ["leadOne", "leadTwo", "leadLeft", "leadRight"].forEach((key) => append(radarState?.[key]));
    ["leadsLeft", "leadsCenter", "leadsRight", "leadsLeft2", "leadsRight2", "leadsCutIn"].forEach((key) => {
      for (const lead of Array.isArray(radarState?.[key]) ? radarState[key] : []) append(lead);
    });
    return leads;
  }

  function inRange(dRel, yRel) {
    return Number.isFinite(dRel) && Number.isFinite(yRel)
      && dRel >= 0 && dRel <= state.rangeM && Math.abs(yRel) <= HALF_WIDTH_M * 1.3;
  }

  function buildTargets(vision, radarState, liveTracks, size) {
    const leads = radarLeads(radarState);
    const selectedIds = new Set(leads.filter((lead) => lead.trackId >= 0).map((lead) => String(lead.trackId)));
    const matchedIds = new Set();
    const targets = [];

    forEachTrack(liveTracks, (track) => {
      if (!inRange(track.dRel, track.yRel)) return;
      const selected = selectedIds.has(String(track.trackId));
      if (selected) matchedIds.add(String(track.trackId));
      targets.push({ ...track, selected, radius: selected ? MARK_RADIUS_LEAD_PX : MARK_RADIUS_PX });
    });

    // Only leads with no matching live track land here; a matched lead already
    // rendered above and keeps its real source hue.
    for (const lead of leads) {
      if (lead.trackId >= 0 && matchedIds.has(String(lead.trackId))) continue;
      if (!inRange(lead.dRel, lead.yRel)) continue;
      targets.push({
        trackId: lead.trackId,
        dRel: lead.dRel,
        yRel: lead.yRel,
        vRel: Number(lead.vRel),
        selected: true,
        measured: Boolean(lead.radar),
        source: "fusion",
        radius: MARK_RADIUS_LEAD_PX,
      });
    }

    for (const lead of Array.isArray(vision?.leadsV3) ? vision.leadsV3 : []) {
      const probability = Number(lead?.prob) || 0;
      // modelV2 leads and radarState leads use OPPOSITE lateral signs: the model
      // reports device-frame y (right positive) while yRel is car-frame (left
      // positive). radard converts with `yRel = float(-lead_msg.y[0])`, so do the
      // same here - without it the vision marks land mirrored against the radar
      // marks they are supposed to corroborate.
      const dRel = Number(lead?.x?.[0]) - RADAR_TO_CAMERA_M;
      const yRel = -Number(lead?.y?.[0]);
      if (probability < 0.5 || !inRange(dRel, yRel)) continue;
      targets.push({
        trackId: -1,
        dRel,
        yRel,
        vRel: Number(lead?.v?.[0]),
        probability,
        selected: false,
        measured: false,
        source: "vision",
        radius: MARK_RADIUS_PX,
      });
    }

    return targets.map((target) => {
      const [screenX, screenY] = project(target.dRel, target.yRel, size);
      return { ...target, screenX, screenY };
    });
  }

  function draw() {
    state.drawRaf = 0;
    if (!state.visible || !rootEl || rootEl.offsetParent === null) return;
    prepare();
    if (!initializeContext()) return;
    const size = viewportSize();
    const pixelRatio = resizeCanvas(size);
    const ctx = state.ctx;
    ctx.clearRect(0, 0, size.width, size.height);

    drawGridLayer(ctx, size, pixelRatio);
    syncEgo(size);

    const vision = interpolatedVisionAt(state.series.modelV2, state.currentMs);
    const radarState = sampleAt(state.series.radarState, state.currentMs);
    const liveTracks = interpolatedTracksAt(state.series.liveTracks, state.currentMs);
    const targets = buildTargets(vision, radarState, liveTracks, size);

    // Leads paint last so they sit above the context marks. Emphasis is carried
    // by tone + size + the direct label; there is no ring, which previously drew
    // a circle around square/triangle marks and read as a stray shape.
    for (const target of targets) {
      if (!target.selected) drawMark(ctx, target);
    }
    for (const target of targets) {
      if (target.selected) drawMark(ctx, target);
    }
    for (const target of targets) {
      if (target.selected) drawLeadLabel(ctx, target, size);
    }

    state.hitTargets = targets;
    if (state.scrollNearPinned && viewportEl) viewportEl.scrollTop = viewportEl.scrollHeight;
    // Marks move under a stationary cursor during playback, so hover has to be
    // re-evaluated from the retained pointer rather than only on pointermove.
    refreshHover();
  }

  function requestDraw() {
    if (!state.visible || state.drawRaf) return;
    state.drawRaf = window.requestAnimationFrame(draw);
  }

  function hideTooltip() {
    if (state.tooltipTimer) window.clearTimeout(state.tooltipTimer);
    state.tooltipTimer = 0;
    state.pointer.pinned = false;
    surface?.hideTooltip();
  }

  function targetAt(clientX, clientY) {
    const rect = (plotEl || viewportEl)?.getBoundingClientRect();
    if (!rect) return null;
    const x = clientX - rect.left;
    const y = clientY - rect.top;
    let selected = null;
    let best = HIT_RADIUS_PX;
    for (const target of state.hitTargets) {
      const distance = Math.hypot(x - target.screenX, y - target.screenY) - (target.selected ? 4 : 0);
      if (distance < best) {
        best = distance;
        selected = target;
      }
    }
    return selected ? { target: selected, x, y, width: rect.width, height: rect.height } : null;
  }

  // Built on the same tinted icon/title/meta card the sibling popups in this
  // panel family use (scrubEventInfo, insights summary), keyed off the target's
  // own tone so the card, the glyph and the mark all agree.
  function showTooltip(hit, temporary = false) {
    if (!surface || !hit) {
      hideTooltip();
      return;
    }
    if (state.tooltipTimer) window.clearTimeout(state.tooltipTimer);
    const { target } = hit;
    const style = SOURCE_STYLE[target.source] || SOURCE_STYLE.fusion;
    const tone = markTone(target);
    const targetState = target.selected
      ? text("replay_sensor_source_lead", "Lead")
      : (target.measured
        ? text("replay_sensor_measured", "measured")
        : text("replay_sensor_unmeasured", "estimated"));

    const rows = [];
    rows.push({ label: text("replay_sensor_row_distance", "Distance"), value: `${target.dRel.toFixed(1)} m` });
    // vRel is lead speed minus ego speed, so a POSITIVE value means the gap is
    // opening. Labelling the row "closing" made "+0.5" self-contradictory.
    const relative = Number(target.vRel);
    if (Number.isFinite(relative)) {
      rows.push({
        label: text("replay_sensor_row_speed", "Rel. speed"),
        value: `${relative >= 0 ? "+" : ""}${relative.toFixed(1)} m/s`,
      });
    }
    // yRel is left-positive, which a bare "+0.3" does not convey to anyone.
    const lateral = Number(target.yRel);
    if (Number.isFinite(lateral)) {
      const magnitude = Math.abs(lateral);
      rows.push({
        label: text("replay_sensor_row_lateral", "Side"),
        value: magnitude < 0.2
          ? text("replay_sensor_side_ahead", "dead ahead")
          : `${lateral > 0 ? text("replay_sensor_side_left", "left") : text("replay_sensor_side_right", "right")} ${magnitude.toFixed(1)} m`,
      });
    }
    if (target.trackId >= 0) {
      rows.push({ label: text("replay_sensor_row_track", "No."), value: `#${Math.trunc(target.trackId)}` });
    }
    surface.showTooltip({
      shape: style.shape,
      color: toneColor(tone),
      title: sourceLabel(target.source),
      state: targetState,
      lead: Boolean(target.selected),
      rows,
      x: hit.x,
      y: hit.y,
      width: hit.width,
      height: hit.height,
    });
    if (temporary) state.tooltipTimer = window.setTimeout(hideTooltip, 2200);
  }

  // Re-runs the hit test from the retained pointer. Called on pointer moves and
  // on every redraw, so a stationary cursor keeps tracking a mark that moves
  // under it instead of showing stale values or silently dropping the popup.
  function refreshHover() {
    if (!state.pointer.active || state.pointer.pinned) return;
    const hit = targetAt(state.pointer.x, state.pointer.y);
    if (hit) showTooltip(hit);
    else hideTooltip();
  }

  function load(options = {}) {
    reset();
    state.records = Array.isArray(options.records) ? options.records : [];
    state.decodeRecord = typeof options.decodeRecord === "function" ? options.decodeRecord : () => [];
    state.manifest = options.manifest && typeof options.manifest === "object" ? options.manifest : null;
    state.status = "idle";
    syncStatus();
  }

  function reset() {
    state.token += 1;
    if (state.trackWorker) {
      try { state.trackWorker.postMessage({ type: "abort" }); } catch {}
      state.trackWorker.terminate();
    }
    state.trackWorker = null;
    state.records = [];
    state.decodeRecord = null;
    state.manifest = null;
    state.series = { modelV2: [], radarState: [], liveTracks: [] };
    state.status = "idle";
    state.trackStatus = "idle";
    state.currentMs = 0;
    state.rangeM = MAX_DISTANCE_M;
    state.scrollNearPinned = true;
    state.hitTargets = [];
    state.pointer.active = false;
    if (state.drawRaf) window.cancelAnimationFrame(state.drawRaf);
    state.drawRaf = 0;
    hideTooltip();
    syncStatus();
    if (state.ctx && canvasEl) {
      const size = viewportSize();
      state.ctx.clearRect(0, 0, size.width, size.height);
    }
  }

  function setVisible(visible) {
    state.visible = Boolean(visible);
    if (!state.visible) {
      hideTooltip();
      return;
    }
    state.scrollNearPinned = true;
    prepare();
    requestDraw();
  }

  function syncTime(seconds) {
    state.currentMs = Math.max(0, Number(seconds) || 0) * 1000;
    requestDraw();
  }

  function syncLabels() {
    const label = text("replay_surroundings", "Forward perception");
    surface?.setHeader(label, `${state.rangeM} m`);
    canvasEl?.setAttribute("aria-label", label);
    readPalette();
    renderLegend();
    syncStatus();
    requestDraw();
  }

  // Events stay on the scrollport so wheel/touch scrolling remains native; hit
  // coordinates are resolved against the complete plot inside it.
  viewportEl?.addEventListener("scroll", () => {
    const maxScrollTop = Math.max(0, viewportEl.scrollHeight - viewportEl.clientHeight);
    state.scrollNearPinned = maxScrollTop - viewportEl.scrollTop <= 2;
  }, { passive: true });
  viewportEl?.addEventListener("pointermove", (event) => {
    if (event.pointerType === "touch") return;
    state.pointer.x = event.clientX;
    state.pointer.y = event.clientY;
    state.pointer.active = true;
    state.pointer.pinned = false;
    refreshHover();
  }, { passive: true });
  viewportEl?.addEventListener("pointerleave", (event) => {
    if (event.pointerType === "touch") return;
    state.pointer.active = false;
    state.pointer.pinned = false;
    hideTooltip();
  }, { passive: true });
  viewportEl?.addEventListener("pointerdown", (event) => {
    const hit = targetAt(event.clientX, event.clientY);
    state.pointer.x = event.clientX;
    state.pointer.y = event.clientY;
    state.pointer.active = event.pointerType !== "touch";
    if (!hit) {
      state.pointer.pinned = false;
      hideTooltip();
      return;
    }
    event.preventDefault();
    // A tap pins the popup to the tapped target; the finger is gone straight
    // after, so a redraw must not re-run the hit test and drop it.
    state.pointer.pinned = event.pointerType === "touch";
    showTooltip(hit, event.pointerType === "touch");
  }, { passive: false });
  if (viewportEl && typeof ResizeObserver === "function") {
    state.resizeObserver = new ResizeObserver(requestDraw);
    state.resizeObserver.observe(viewportEl);
    if (plotEl) state.resizeObserver.observe(plotEl);
  } else {
    window.addEventListener("resize", requestDraw, { passive: true });
  }

  syncLabels();
  reset();
  return { load, reset, setVisible, syncTime, syncLabels, resize: requestDraw };
})();
