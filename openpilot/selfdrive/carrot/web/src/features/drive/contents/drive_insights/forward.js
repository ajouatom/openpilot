import { createTelemetryForwardSurface } from "../../../telemetry/forward_surface.js";
import {
  TELEMETRY_FORWARD_SOURCE_ORDER,
  TELEMETRY_FORWARD_SOURCE_STYLE,
  telemetryForwardSourceLabel,
  telemetryForwardSourceStyle,
} from "../../../telemetry/forward_sources.js";

export const DRIVE_INSIGHTS_FORWARD_MAX_DISTANCE_M = 100;

const FORWARD_SHAPES = Object.freeze(["circle", "square", "diamond", "triangle"]);
const FORWARD_TONES = Object.freeze(["lead", "accent", "neutral"]);
const FORWARD_HALF_WIDTH_M = 12;
const FORWARD_LATERAL_CUTOFF_RATIO = 1.3;
const FORWARD_LATERAL_VIEW_RATIO = 0.45;
const FORWARD_GRID_MAJOR_STEP_M = 20;
const FORWARD_GRID_MINOR_STEP_M = 10;
const FORWARD_GRID_LATERAL_STEP_M = 4;

function finiteNumber(value) {
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

function boundedNumber(value, fallback, minimum, maximum) {
  const number = finiteNumber(value);
  return number === null ? fallback : Math.max(minimum, Math.min(maximum, number));
}

function nonEmptyText(value) {
  if (typeof value !== "string") return null;
  const text = value.trim();
  return text || null;
}

export function calculateDriveInsightsCanvasSize(rect = {}, devicePixelRatio = 1) {
  const width = Math.max(1, Math.round(finiteNumber(rect.width) ?? 1));
  const height = Math.max(1, Math.round(finiteNumber(rect.height) ?? 1));
  const dpr = Math.max(1, Math.min(3, finiteNumber(devicePixelRatio) ?? 1));
  return Object.freeze({
    width,
    height,
    dpr,
    pixelWidth: Math.max(1, Math.round(width * dpr)),
    pixelHeight: Math.max(1, Math.round(height * dpr)),
  });
}

export function projectDriveInsightsForwardPoint(point, viewport = {}) {
  const xM = finiteNumber(point?.xM);
  const yM = finiteNumber(point?.yM);
  const width = Math.max(1, finiteNumber(viewport.width) ?? 1);
  const height = Math.max(1, finiteNumber(viewport.height) ?? 1);
  const maxDistanceM = Math.max(1, finiteNumber(viewport.maxDistanceM) ?? DRIVE_INSIGHTS_FORWARD_MAX_DISTANCE_M);
  // Same bound as the replay forward view: targets far outside the drawn
  // corridor would otherwise pile up against the left and right edges.
  if (xM === null || yM === null || xM < 0 || xM > maxDistanceM) return null;
  if (Math.abs(yM) > FORWARD_HALF_WIDTH_M * FORWARD_LATERAL_CUTOFF_RATIO) return null;
  const farY = height * 0.16;
  const egoY = Math.max(height * 0.5, height - 46);
  const longitudinalScale = (egoY - farY) / maxDistanceM;
  const lateralScale = Math.max(1, width * FORWARD_LATERAL_VIEW_RATIO / FORWARD_HALF_WIDTH_M);
  return Object.freeze({
    x: width / 2 - yM * lateralScale,
    y: egoY - xM * longitudinalScale,
    xM,
    yM,
  });
}

function normalizedEntities(snapshot) {
  const result = [];
  const append = (entries, kind) => {
    for (const [index, entry] of (Array.isArray(entries) ? entries : []).entries()) {
      const xM = finiteNumber(entry?.xM);
      const yM = finiteNumber(entry?.yM);
      if (xM === null || yM === null) continue;
      result.push(Object.freeze({
        id: String(entry?.id || `${kind}:${index}`),
        kind,
        source: String(entry?.source || "unknown"),
        xM,
        yM,
        relativeSpeedMps: finiteNumber(entry?.relativeSpeedMps),
        measured: kind === "radar" ? entry?.measured === true : null,
        // A radar entry the normalizer matched to a radarState lead, or any
        // vision lead: these are the targets the planner is actually following.
        selected: kind === "lead" || entry?.selected === true,
      }));
    }
  };
  append(snapshot?.leads, "lead");
  append(snapshot?.radar, "radar");
  return Object.freeze(result);
}

function presentationValue(presentation, key) {
  const values = presentation?.entities ?? presentation;
  if (values instanceof Map) return values.get(key) ?? null;
  if (!values || typeof values !== "object") return null;
  return Object.prototype.hasOwnProperty.call(values, key) ? values[key] : null;
}

function normalizePresentation(entity, presentation) {
  const general = presentationValue(presentation, entity.id);
  const scoped = presentationValue(presentation, `${entity.kind}:${entity.id}`);
  const combined = {
    ...(general && typeof general === "object" ? general : {}),
    ...(scoped && typeof scoped === "object" ? scoped : {}),
    meta: {
      ...(general?.meta && typeof general.meta === "object" ? general.meta : {}),
      ...(scoped?.meta && typeof scoped.meta === "object" ? scoped.meta : {}),
    },
  };
  // Presentation overrides win, but the entity's own state is the default so
  // leads stand out without every caller having to supply a presentation map.
  const selected = combined.selected === undefined
    ? entity.selected === true
    : combined.selected === true;
  const fallbackRadius = entity.kind === "lead" ? 6 : 4;
  const radiusPx = boundedNumber(combined.radiusPx, selected ? Math.max(7, fallbackRadius) : fallbackRadius, 2, 14);
  const tone = FORWARD_TONES.includes(combined.tone)
    ? combined.tone
    : selected ? "lead" : (entity.kind === "lead" ? "lead" : "neutral");
  const defaultShape = telemetryForwardSourceStyle(entity.source).shape;
  const shape = FORWARD_SHAPES.includes(combined.shape) ? combined.shape : defaultShape;
  const label = nonEmptyText(combined.label);
  return Object.freeze({
    shape,
    tone,
    selected,
    visible: combined.visible !== false,
    filled: combined.filled !== false,
    showLabel: combined.showLabel ?? entity.kind === "lead",
    label,
    sourceLabel: nonEmptyText(combined.sourceLabel),
    radiusPx,
    hitRadiusPx: boundedNumber(combined.hitRadiusPx, Math.max(20, radiusPx + 8), radiusPx, 48),
    meta: Object.freeze({ ...combined.meta }),
  });
}

export function createDriveInsightsForwardScene(snapshot, viewport = {}, presentation = null) {
  const width = Math.max(1, finiteNumber(viewport.width) ?? 1);
  const height = Math.max(1, finiteNumber(viewport.height) ?? 1);
  const maxDistanceM = Math.max(1, finiteNumber(viewport.maxDistanceM) ?? DRIVE_INSIGHTS_FORWARD_MAX_DISTANCE_M);
  const entities = normalizedEntities(snapshot).flatMap((entity) => {
    const point = projectDriveInsightsForwardPoint(entity, { width, height, maxDistanceM });
    if (!point) return [];
    const entityPresentation = normalizePresentation(entity, presentation);
    if (!entityPresentation.visible) return [];
    return [Object.freeze({
      ...entity,
      point,
      screenX: point.x,
      screenY: point.y,
      selected: entityPresentation.selected,
      hitRadiusPx: entityPresentation.hitRadiusPx,
      presentation: entityPresentation,
    })];
  });
  const freshness = ["lanes", "leads", "radar"].map((domain) => (
    String(snapshot?.freshness?.[domain]?.state || "missing")
  ));
  const state = freshness.includes("fresh")
    ? "valid"
    : (freshness.includes("stale") ? "stale" : "empty");
  const frozenEntities = Object.freeze(entities);
  return Object.freeze({
    state,
    width,
    height,
    maxDistanceM,
    ego: Object.freeze({ x: width / 2, y: Math.max(height * 0.5, height - 46) }),
    laneOffsetsM: Object.freeze([-1.8, 1.8]),
    entities: frozenEntities,
    hitTargets: frozenEntities,
  });
}

function cssPalette(options, root) {
  const style = options.getComputedStyle?.(root)
    || root?.ownerDocument?.defaultView?.getComputedStyle?.(root)
    || globalThis.getComputedStyle?.(root);
  const read = (name) => String(style?.getPropertyValue?.(name) || "").trim();
  return Object.freeze({
    grid: read("--drive-insights-forward-grid"),
    gridMinor: read("--drive-insights-forward-grid-minor"),
    gridLabel: read("--drive-insights-forward-grid-label"),
    gridFont: read("--drive-insights-forward-grid-font"),
    lane: read("--drive-insights-forward-lane"),
    lead: read("--drive-insights-forward-lead"),
    radar: read("--drive-insights-forward-radar"),
    neutral: read("--drive-insights-forward-neutral"),
    text: read("--drive-insights-forward-text"),
    plate: read("--drive-insights-panel") || read("--md-surface"),
  });
}

function toneColor(tone, palette) {
  if (tone === "lead") return palette.lead;
  if (tone === "neutral") return palette.neutral;
  return palette.radar;
}

function shapePath(context, x, y, shape, radius) {
  context.beginPath?.();
  if (shape === "square") {
    context.rect?.(x - radius, y - radius, radius * 2, radius * 2);
  } else if (shape === "diamond") {
    context.moveTo?.(x, y - radius * 1.3);
    context.lineTo?.(x + radius * 1.15, y);
    context.lineTo?.(x, y + radius * 1.3);
    context.lineTo?.(x - radius * 1.15, y);
  } else if (shape === "triangle") {
    context.moveTo?.(x, y - radius * 1.35);
    context.lineTo?.(x + radius * 1.25, y + radius * 0.9);
    context.lineTo?.(x - radius * 1.25, y + radius * 0.9);
  } else {
    context.arc?.(x, y, radius, 0, Math.PI * 2);
  }
  context.closePath?.();
}

function entityLabel(entity) {
  if (entity.presentation.label) return entity.presentation.label;
  return `${entity.xM.toFixed(0)} m`;
}

function drawEntity(context, entity, palette, scene) {
  const { presentation, point } = entity;
  const color = toneColor(presentation.tone, palette);
  if (!color) return;
  shapePath(context, point.x, point.y, presentation.shape, presentation.radiusPx);
  context.lineJoin = "round";
  if (palette.plate) {
    context.lineWidth = 3.5;
    context.strokeStyle = palette.plate;
    context.stroke?.();
  }
  if (presentation.filled) {
    context.fillStyle = color;
    context.fill?.();
  }
  context.lineWidth = 2;
  context.strokeStyle = color;
  context.stroke?.();
  if (!presentation.showLabel || !palette.text) return;
  const label = entityLabel(entity);
  context.font = presentation.selected ? "800 12px system-ui" : "600 11px system-ui";
  context.textBaseline = "middle";
  const measuredWidth = context.measureText?.(label)?.width ?? 0;
  const gap = presentation.radiusPx + 10;
  const fitsRight = point.x + gap + measuredWidth + 4 <= scene.width;
  context.textAlign = fitsRight ? "left" : "right";
  const labelX = fitsRight ? point.x + gap : point.x - gap;
  const labelY = Math.max(8, Math.min(scene.height - 8, point.y));
  if (palette.plate) {
    context.lineWidth = 3;
    context.strokeStyle = palette.plate;
    context.strokeText?.(label, labelX, labelY);
  }
  context.fillStyle = palette.text;
  context.fillText?.(label, labelX, labelY);
}

function drawForwardGrid(context, scene, palette) {
  const { width, height, maxDistanceM } = scene;
  if (palette.gridMinor) {
    context.strokeStyle = palette.gridMinor;
    context.lineWidth = 1;
    context.beginPath?.();
    for (
      let lateralM = FORWARD_GRID_LATERAL_STEP_M;
      lateralM <= FORWARD_HALF_WIDTH_M;
      lateralM += FORWARD_GRID_LATERAL_STEP_M
    ) {
      for (const sideM of [-lateralM, lateralM]) {
        const point = projectDriveInsightsForwardPoint({ xM: 0, yM: sideM }, scene);
        if (!point) continue;
        const x = Math.round(point.x) + 0.5;
        context.moveTo?.(x, 0);
        context.lineTo?.(x, height);
      }
    }
    for (
      let distanceM = FORWARD_GRID_MINOR_STEP_M;
      distanceM <= maxDistanceM;
      distanceM += FORWARD_GRID_MINOR_STEP_M
    ) {
      if (distanceM % FORWARD_GRID_MAJOR_STEP_M === 0) continue;
      const point = projectDriveInsightsForwardPoint({ xM: distanceM, yM: 0 }, scene);
      if (!point) continue;
      const y = Math.round(point.y) + 0.5;
      context.moveTo?.(0, y);
      context.lineTo?.(width, y);
    }
    context.stroke?.();
  }

  if (palette.grid) {
    context.strokeStyle = palette.grid;
    context.lineWidth = 1;
    context.beginPath?.();
    for (
      let distanceM = 0;
      distanceM <= maxDistanceM;
      distanceM += FORWARD_GRID_MAJOR_STEP_M
    ) {
      const point = projectDriveInsightsForwardPoint({ xM: distanceM, yM: 0 }, scene);
      if (!point) continue;
      const y = Math.round(point.y) + 0.5;
      context.moveTo?.(0, y);
      context.lineTo?.(width, y);
    }
    context.stroke?.();
  }

  if (palette.gridLabel && palette.gridFont) {
    context.fillStyle = palette.gridLabel;
    context.font = palette.gridFont;
    context.textAlign = "left";
    context.textBaseline = "bottom";
    for (
      let distanceM = 0;
      distanceM <= maxDistanceM;
      distanceM += FORWARD_GRID_MAJOR_STEP_M
    ) {
      const point = projectDriveInsightsForwardPoint({ xM: distanceM, yM: 0 }, scene);
      if (!point) continue;
      context.fillText?.(`${distanceM} m`, 6, point.y - 3);
    }
  }
}

function drawForwardBackground(context, scene, palette) {
  if (!context) return;
  const { height } = scene;
  drawForwardGrid(context, scene, palette);
  if (!palette.lane) return;
  context.strokeStyle = palette.lane;
  context.lineWidth = 1.5;
  for (const offset of scene.laneOffsetsM) {
    const near = projectDriveInsightsForwardPoint({ xM: 0, yM: offset }, scene);
    if (!near) continue;
    context.beginPath?.();
    context.moveTo?.(near.x, 0);
    context.lineTo?.(near.x, height);
    context.stroke?.();
  }
}

function drawForwardScene(context, scene, palette, background = null) {
  if (!context) return;
  const { width, height } = scene;
  context.clearRect?.(0, 0, width, height);
  if (background && typeof context.drawImage === "function") {
    context.drawImage(background, 0, 0, background.width, background.height, 0, 0, width, height);
  } else drawForwardBackground(context, scene, palette);

  for (const selected of [false, true]) {
    for (const entity of scene.entities) {
      if (entity.selected === selected) drawEntity(context, entity, palette, scene);
    }
  }
}

export function createDriveInsightsForwardRenderer(options = {}) {
  const root = options.root;
  const documentRoot = options.document || root?.ownerDocument || globalThis.document;
  if (!root || !documentRoot?.createElement || typeof root.append !== "function") return null;
  const target = options.target || documentRoot.defaultView || globalThis;
  root.classList?.add("drive-insights__forward");
  const surface = createTelemetryForwardSurface({
    root,
    document: documentRoot,
    target,
    // A short viewport scrolls the plot; keep the ego end visible the way the
    // replay forward view does, without stealing a scroll the reader started.
    anchorBottom: true,
  });
  if (!surface) return null;
  const { canvas, ego } = surface.elements;
  const context = canvas.getContext?.("2d") || null;
  let lastSnapshot = null;
  let lastPresentation = null;
  let lastRect = null;
  let currentScene = createDriveInsightsForwardScene(null, { width: 1, height: 1 });
  let palette = cssPalette(options, root);
  let paletteReadAtMs = Number.NEGATIVE_INFINITY;
  const backgroundCanvas = documentRoot.createElement("canvas");
  const backgroundContext = backgroundCanvas.getContext?.("2d") || null;
  let backgroundKey = "";
  let lastHeaderKey = "";
  const listeners = [];
  const pointer = { x: 0, y: 0, active: false };
  let tooltipTimer = null;
  let destroyed = false;

  function uiText(key, fallback) {
    return typeof options.text === "function" ? options.text(key, fallback) : fallback;
  }

  function refreshPalette(force = false) {
    const now = Number(target.performance?.now?.() ?? Date.now());
    if (!force && now - paletteReadAtMs < 1000) return palette;
    paletteReadAtMs = now;
    palette = cssPalette(options, root);
    return palette;
  }

  function syncLabels() {
    if (destroyed) return false;
    refreshPalette(true);
    backgroundKey = "";
    lastHeaderKey = "";
    const title = uiText("drive_insights_forward_title", "Forward perception");
    surface.setHeader(title, `${DRIVE_INSIGHTS_FORWARD_MAX_DISTANCE_M} m`);
    surface.setLegend(TELEMETRY_FORWARD_SOURCE_ORDER.map((source) => {
      const style = TELEMETRY_FORWARD_SOURCE_STYLE[source];
      const color = style.tone === "lead"
        ? "var(--md-primary)"
        : style.tone === "vision" ? "var(--md-info)" : "var(--md-outline-var)";
      return {
        shape: style.shape,
        color,
        label: telemetryForwardSourceLabel(source, uiText),
      };
    }), uiText("replay_sensor_sources", "Sensor sources"));
    canvas.setAttribute?.(
      "aria-label",
      uiText("drive_insights_forward_canvas", "Forward driving data"),
    );
    return true;
  }

  function hitAt(clientX, clientY) {
    const rect = canvas.getBoundingClientRect?.();
    if (!rect?.width || !rect?.height) return null;
    const x = (clientX - rect.left) * currentScene.width / rect.width;
    const y = (clientY - rect.top) * currentScene.height / rect.height;
    let selected = null;
    let best = 20;
    for (const entity of currentScene.hitTargets) {
      const distance = Math.hypot(x - entity.screenX, y - entity.screenY);
      if (distance >= Math.min(best, entity.hitRadiusPx)) continue;
      best = distance;
      selected = entity;
    }
    return selected;
  }

  function tooltipSource(entity) {
    return telemetryForwardSourceLabel(entity.source, uiText);
  }

  function showEntityTooltip(entity, temporary = false) {
    if (!entity) {
      surface.hideTooltip();
      return false;
    }
    if (tooltipTimer !== null) target.clearTimeout?.(tooltipTimer);
    tooltipTimer = null;
    const lateral = entity.yM;
    const magnitude = Math.abs(lateral);
    const rows = [
      { label: uiText("replay_sensor_row_distance", "Distance"), value: `${entity.xM.toFixed(1)} m` },
      {
        label: uiText("replay_sensor_row_lateral", "Side"),
        value: magnitude < 0.2
          ? uiText("replay_sensor_side_ahead", "dead ahead")
          : `${lateral > 0 ? uiText("replay_sensor_side_left", "left") : uiText("replay_sensor_side_right", "right")} ${magnitude.toFixed(1)} m`,
      },
    ];
    if (entity.relativeSpeedMps !== null) {
      rows.splice(1, 0, {
        label: uiText("replay_sensor_row_speed", "Rel. speed"),
        value: `${entity.relativeSpeedMps >= 0 ? "+" : ""}${entity.relativeSpeedMps.toFixed(1)} m/s`,
      });
    }
    const isLead = entity.kind === "lead" || entity.selected;
    surface.showTooltip({
      shape: entity.presentation.shape,
      color: toneColor(entity.presentation.tone, palette),
      title: tooltipSource(entity),
      state: isLead
        ? uiText("replay_sensor_source_lead", "Lead")
        : (entity.measured
          ? uiText("replay_sensor_measured", "measured")
          : uiText("replay_sensor_unmeasured", "estimated")),
      lead: isLead,
      rows,
      x: entity.point.x,
      y: entity.point.y,
      width: currentScene.width,
      height: currentScene.height,
    });
    if (temporary && typeof target.setTimeout === "function") {
      tooltipTimer = target.setTimeout(() => {
        tooltipTimer = null;
        surface.hideTooltip();
      }, 2200);
    }
    return true;
  }

  function refreshTooltip() {
    if (!pointer.active) return;
    showEntityTooltip(hitAt(pointer.x, pointer.y));
  }

  function prepareBackground(scene, size) {
    if (!backgroundContext) return null;
    const key = [
      size.pixelWidth,
      size.pixelHeight,
      scene.maxDistanceM,
      palette.grid,
      palette.gridMinor,
      palette.gridLabel,
      palette.gridFont,
      palette.lane,
    ].join("|");
    if (key === backgroundKey) return backgroundCanvas;
    backgroundKey = key;
    if (backgroundCanvas.width !== size.pixelWidth) backgroundCanvas.width = size.pixelWidth;
    if (backgroundCanvas.height !== size.pixelHeight) backgroundCanvas.height = size.pixelHeight;
    backgroundContext.setTransform?.(size.dpr, 0, 0, size.dpr, 0, 0);
    backgroundContext.clearRect?.(0, 0, size.width, size.height);
    drawForwardBackground(backgroundContext, scene, palette);
    return backgroundCanvas;
  }

  function paint() {
    if (destroyed) return false;
    refreshPalette();
    const measuredRect = canvas.getBoundingClientRect?.() || {};
    const rect = Number(measuredRect.width) > 1 && Number(measuredRect.height) > 1
      ? measuredRect
      : (lastRect || root.getBoundingClientRect?.() || {});
    const size = calculateDriveInsightsCanvasSize(rect, target.devicePixelRatio || 1);
    if (canvas.width !== size.pixelWidth) canvas.width = size.pixelWidth;
    if (canvas.height !== size.pixelHeight) canvas.height = size.pixelHeight;
    context?.setTransform?.(size.dpr, 0, 0, size.dpr, 0, 0);
    currentScene = createDriveInsightsForwardScene(lastSnapshot, size, lastPresentation);
    root.dataset.state = currentScene.state;
    const egoTop = `${currentScene.ego.y}px`;
    if (ego.style.top !== egoTop) ego.style.top = egoTop;
    const title = uiText("drive_insights_forward_title", "Forward perception");
    const headerKey = `${title}|${currentScene.maxDistanceM}|${currentScene.state}`;
    if (headerKey !== lastHeaderKey) {
      lastHeaderKey = headerKey;
      surface.setHeader(title, `${currentScene.maxDistanceM} m`, currentScene.state === "stale");
    }
    drawForwardScene(context, currentScene, palette, prepareBackground(currentScene, size));
    refreshTooltip();
    return true;
  }

  function render(snapshot, presentation = null) {
    if (destroyed) return false;
    lastSnapshot = snapshot && typeof snapshot === "object" ? snapshot : null;
    lastPresentation = presentation && typeof presentation === "object" ? presentation : null;
    return paint();
  }

  function resize(rect) {
    if (destroyed) return false;
    lastRect = rect && typeof rect === "object" ? rect : null;
    const painted = paint();
    // Layout handed us a new size; keep the ego end of the plot in view unless
    // the reader has scrolled away from it.
    surface.anchorBottom();
    return painted;
  }

  function scene() {
    return currentScene;
  }

  function status() {
    return Object.freeze({
      destroyed,
      state: currentScene.state,
      entityCount: currentScene.entities.length,
      selectedCount: currentScene.entities.filter(({ selected }) => selected).length,
    });
  }

  function destroy() {
    if (destroyed) return false;
    destroyed = true;
    lastSnapshot = null;
    lastPresentation = null;
    if (tooltipTimer !== null) target.clearTimeout?.(tooltipTimer);
    tooltipTimer = null;
    for (const [eventTarget, name, listener] of listeners.splice(0)) {
      eventTarget.removeEventListener?.(name, listener);
    }
    surface.destroy?.();
    root.replaceChildren?.();
    return true;
  }

  const pointerTarget = surface.elements.viewport || canvas;
  const onPointerMove = (event) => {
    if (event.pointerType === "touch") return;
    pointer.x = event.clientX;
    pointer.y = event.clientY;
    pointer.active = true;
    refreshTooltip();
  };
  const onPointerLeave = (event) => {
    if (event.pointerType === "touch") return;
    pointer.active = false;
    surface.hideTooltip();
  };
  const onPointerDown = (event) => {
    const entity = hitAt(event.clientX, event.clientY);
    if (!entity) {
      surface.hideTooltip();
      return;
    }
    if (event.pointerType === "touch") event.preventDefault?.();
    showEntityTooltip(entity, event.pointerType === "touch");
  };
  pointerTarget.addEventListener?.("pointermove", onPointerMove, { passive: true });
  pointerTarget.addEventListener?.("pointerleave", onPointerLeave, { passive: true });
  pointerTarget.addEventListener?.("pointerdown", onPointerDown, { passive: false });
  listeners.push(
    [pointerTarget, "pointermove", onPointerMove],
    [pointerTarget, "pointerleave", onPointerLeave],
    [pointerTarget, "pointerdown", onPointerDown],
  );

  syncLabels();
  return Object.freeze({ render, resize, syncLabels, scene, status, destroy });
}
