import { createTelemetryGraphSurface } from "../../../telemetry/graph_surface.js";

export const DRIVE_INSIGHTS_GRAPH_METRICS = Object.freeze([
  Object.freeze({
    id: "speed",
    label: "Speed",
    unit: "km/h",
    digits: 1,
    minimumSpan: 20,
    value(snapshot) {
      const speedMps = finiteNumber(snapshot?.ego?.speedMps);
      return speedMps === null ? null : speedMps * 3.6;
    },
  }),
  Object.freeze({
    id: "steering",
    label: "Steering",
    unit: "°",
    digits: 2,
    minimumSpan: 20,
    value(snapshot) { return snapshot?.ego?.steeringAngleDeg; },
  }),
  Object.freeze({
    id: "acceleration",
    label: "Acceleration",
    unit: "m/s²",
    digits: 2,
    minimumSpan: 2,
    value(snapshot) { return snapshot?.ego?.accelMps2; },
  }),
  Object.freeze({
    id: "lead_distance",
    label: "Lead distance",
    unit: "m",
    digits: 1,
    minimumSpan: 20,
    value(snapshot) {
      const leads = Array.isArray(snapshot?.leads) ? snapshot.leads : [];
      let nearest = null;
      for (const lead of leads) {
        const distance = finiteNumber(lead?.xM);
        if (distance !== null && distance >= 0 && (nearest === null || distance < nearest)) nearest = distance;
      }
      return nearest;
    },
  }),
]);

const FRESHNESS_PRIORITY = Object.freeze({ fresh: 0, stale: 1, missing: 2 });

function finiteNumber(value) {
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

function sampleSnapshot(sample) {
  if (!sample || typeof sample !== "object") return null;
  return sample.snapshot && typeof sample.snapshot === "object" ? sample.snapshot : sample;
}

function sampleTimestamp(sample, snapshot) {
  return finiteNumber(sample?.timestampMs) ?? finiteNumber(snapshot?.timestampMs);
}

function frozenSeries(values) {
  return Object.freeze(values.map((value) => Object.freeze(value)));
}

function freshnessState(snapshot, domains) {
  let worst = "fresh";
  for (const domain of domains) {
    const state = String(snapshot?.freshness?.[domain]?.state || "missing");
    const normalized = Object.hasOwn(FRESHNESS_PRIORITY, state) ? state : "missing";
    if (FRESHNESS_PRIORITY[normalized] > FRESHNESS_PRIORITY[worst]) worst = normalized;
  }
  return worst;
}

function graphMetrics(presentation) {
  const overrides = presentation && typeof presentation === "object" && !Array.isArray(presentation)
    ? presentation
    : {};
  const metrics = DRIVE_INSIGHTS_GRAPH_METRICS.map((metric) => {
    const override = overrides[metric.id];
    const value = override && typeof override === "object" && !Array.isArray(override)
      ? override
      : {};
    const digits = Number.isSafeInteger(value.digits) && value.digits >= 0 && value.digits <= 6
      ? value.digits
      : metric.digits;
    return Object.freeze({
      ...metric,
      label: typeof value.label === "string" && value.label ? value.label : metric.label,
      labelKey: typeof value.labelKey === "string" && value.labelKey
        ? value.labelKey
        : `drive_insights_graph_${metric.id}`,
      unit: typeof value.unit === "string" ? value.unit : metric.unit,
      digits,
      minimumSpan: finiteNumber(value.minimumSpan) ?? metric.minimumSpan ?? 0,
      value: typeof value.value === "function" ? value.value : metric.value,
    });
  });
  if (!Array.isArray(overrides.metricOrder)) return metrics;
  const order = new Map(overrides.metricOrder.map((id, index) => [String(id), index]));
  return metrics.sort((left, right) => (
    (order.get(left.id) ?? Number.MAX_SAFE_INTEGER)
      - (order.get(right.id) ?? Number.MAX_SAFE_INTEGER)
  ));
}

export function createDriveInsightsGraphModel(windowValue, options = {}) {
  const samples = Array.isArray(windowValue?.samples) ? windowValue.samples : [];
  const metrics = graphMetrics(options.presentation).map((metric) => {
    const values = [];
    for (const sample of samples) {
      const snapshot = sampleSnapshot(sample);
      const timestampMs = sampleTimestamp(sample, snapshot);
      const value = finiteNumber(metric.value(snapshot));
      if (timestampMs === null || timestampMs < 0 || value === null) continue;
      values.push({ timestampMs, value });
    }
    values.sort((left, right) => left.timestampMs - right.timestampMs);
    const numericValues = values.map(({ value }) => value);
    let minimum = numericValues.length ? Math.min(...numericValues) : null;
    let maximum = numericValues.length ? Math.max(...numericValues) : null;
    // Autoscaling to the exact data range turns a steady signal into a wall of
    // noise, because a 0.2 km/h ripple gets stretched over the full card
    // height. Hold the axis open to a metric-specific minimum span instead.
    if (minimum !== null && maximum !== null) {
      const span = maximum - minimum;
      const minimumSpan = finiteNumber(metric.minimumSpan) ?? 0;
      const wanted = Math.max(minimumSpan, 1e-6);
      if (span < wanted) {
        const midpoint = (minimum + maximum) / 2;
        minimum = midpoint - wanted / 2;
        maximum = midpoint + wanted / 2;
      }
    }
    return Object.freeze({
      id: metric.id,
      label: metric.label,
      unit: metric.unit,
      digits: metric.digits,
      minimumSpan: finiteNumber(metric.minimumSpan) ?? 0,
      current: numericValues.length ? numericValues.at(-1) : null,
      minimum,
      maximum,
      values: frozenSeries(values),
    });
  });

  const latestSnapshot = samples.length ? sampleSnapshot(samples.at(-1)) : null;
  const hasValues = metrics.some(({ values }) => values.length > 0);
  const freshness = latestSnapshot
    ? freshnessState(latestSnapshot, ["ego"])
    : "missing";
  const state = !hasValues ? "empty" : (freshness === "fresh" ? "valid" : "stale");
  const firstTimestamp = samples.length ? sampleTimestamp(samples[0], sampleSnapshot(samples[0])) : null;
  const lastTimestamp = samples.length ? sampleTimestamp(samples.at(-1), latestSnapshot) : null;
  const startTimestampMs = finiteNumber(windowValue?.startTimestampMs) ?? firstTimestamp ?? 0;
  const endTimestampMs = finiteNumber(windowValue?.endTimestampMs) ?? lastTimestamp ?? startTimestampMs;
  const normalizedStartTimestampMs = Math.max(0, startTimestampMs);

  return Object.freeze({
    state,
    freshness,
    startTimestampMs: normalizedStartTimestampMs,
    endTimestampMs: Math.max(normalizedStartTimestampMs, endTimestampMs),
    cadenceMs: Math.max(0, finiteNumber(windowValue?.cadenceMs) ?? 0),
    metrics: Object.freeze(metrics),
  });
}

function requireDocument(options, root) {
  return options.document || root?.ownerDocument || globalThis.document;
}

function textValue(options, key, fallback) {
  return typeof options.text === "function" ? options.text(key, fallback) : fallback;
}

function cssValue(options, root, name) {
  const style = options.getComputedStyle?.(root)
    || root?.ownerDocument?.defaultView?.getComputedStyle?.(root)
    || globalThis.getComputedStyle?.(root);
  return String(style?.getPropertyValue?.(name) || "").trim();
}

function canvasSize(canvas, target) {
  const rect = canvas.getBoundingClientRect?.() || {};
  const width = Math.max(1, Math.round(Number(rect.width || canvas.clientWidth || 1)));
  const height = Math.max(1, Math.round(Number(rect.height || canvas.clientHeight || 1)));
  const dpr = Math.max(1, Math.min(3, Number(target?.devicePixelRatio || 1)));
  const pixelWidth = Math.max(1, Math.round(width * dpr));
  const pixelHeight = Math.max(1, Math.round(height * dpr));
  if (canvas.width !== pixelWidth) canvas.width = pixelWidth;
  if (canvas.height !== pixelHeight) canvas.height = pixelHeight;
  return { width, height, dpr };
}

const RANGE_SHRINK_RATIO = 0.6;
const RANGE_SHRINK_STEP = 0.12;

// The window is a moving 10 s slice, so when an old peak scrolls out the raw
// min/max snaps and the whole trace jumps. Grow the drawn axis immediately but
// let it contract slowly, so the line moves and the axis mostly does not.
function stabilizeRange(previous, minimum, maximum) {
  if (minimum === null || maximum === null) return null;
  if (!previous) return { minimum, maximum };
  let nextMinimum = Math.min(previous.minimum, minimum);
  let nextMaximum = Math.max(previous.maximum, maximum);
  const wantedSpan = maximum - minimum;
  const currentSpan = nextMaximum - nextMinimum;
  if (wantedSpan > 0 && currentSpan > wantedSpan / RANGE_SHRINK_RATIO) {
    nextMinimum += (minimum - nextMinimum) * RANGE_SHRINK_STEP;
    nextMaximum += (maximum - nextMaximum) * RANGE_SHRINK_STEP;
  }
  return { minimum: nextMinimum, maximum: nextMaximum };
}

function drawMetric(context, canvas, metric, model, lineColor, target, ranges) {
  if (!context) return;
  const { width, height, dpr } = canvasSize(canvas, target);
  context.setTransform?.(dpr, 0, 0, dpr, 0, 0);
  context.clearRect?.(0, 0, width, height);
  if (!lineColor || metric.values.length < 2 || metric.minimum === null || metric.maximum === null) {
    if (!metric.values.length) ranges?.delete(metric.id);
    return;
  }
  const bounds = stabilizeRange(ranges?.get(metric.id), metric.minimum, metric.maximum);
  ranges?.set(metric.id, bounds);
  const start = finiteNumber(model?.startTimestampMs) ?? metric.values[0].timestampMs;
  const end = finiteNumber(model?.endTimestampMs) ?? metric.values.at(-1).timestampMs;
  const duration = Math.max(1, end - start);
  const range = Math.max(1e-9, bounds.maximum - bounds.minimum);
  context.strokeStyle = lineColor;
  context.lineWidth = metric.id === "speed" ? 1.8 : 1.6;
  context.lineJoin = "round";
  context.lineCap = "round";
  context.beginPath?.();
  metric.values.forEach((sample, index) => {
    const x = Math.max(0, Math.min(width, (sample.timestampMs - start) / duration * width));
    const y = height - ((sample.value - bounds.minimum) / range * Math.max(1, height - 4)) - 2;
    if (index === 0) context.moveTo?.(x, y);
    else context.lineTo?.(x, y);
  });
  context.stroke?.();
}

export function createDriveInsightsGraphRenderer(options = {}) {
  const root = options.root;
  const documentRoot = requireDocument(options, root);
  if (!root || !documentRoot?.createElement || typeof root.append !== "function") return null;
  const target = options.target || documentRoot.defaultView || globalThis;
  const metrics = graphMetrics(options.presentation);
  const surface = createTelemetryGraphSurface(root, metrics.map((metric) => ({
    ...metric,
    label: textValue(options, metric.labelKey, metric.label),
  })), { document: documentRoot });
  if (!surface) return null;
  const cards = new Map(Array.from(surface.cards, ([id, card]) => [id, {
    ...card,
    output: card.number,
    context: card.canvas.getContext?.("2d") || null,
  }]));
  let model = createDriveInsightsGraphModel(null, { presentation: options.presentation });
  const ranges = new Map();
  let destroyed = false;
  let colorsReadAtMs = Number.NEGATIVE_INFINITY;

  root.classList?.add("drive-insights__graph");
  function refreshLineColors(force = false) {
    const now = Number(target.performance?.now?.() ?? Date.now());
    if (!force && now - colorsReadAtMs < 1000) return false;
    colorsReadAtMs = now;
    for (const card of cards.values()) {
      card.lineColor = cssValue(options, card.card, "--graph-color");
    }
    return true;
  }

  for (const metric of metrics) {
    const card = cards.get(metric.id);
    if (card) card.card.setAttribute?.("aria-label", card.label.textContent);
  }
  refreshLineColors(true);

  function paint() {
    if (destroyed) return false;
    refreshLineColors();
    root.dataset.state = model.state;
    for (const metric of model.metrics) {
      const card = cards.get(metric.id);
      if (!card) continue;
      const output = metric.current === null ? "--" : metric.current.toFixed(metric.digits);
      if (card.output.textContent !== output) card.output.textContent = output;
      drawMetric(card.context, card.canvas, metric, model, card.lineColor, target, ranges);
    }
    return true;
  }

  function render(windowValue) {
    if (destroyed) return false;
    model = createDriveInsightsGraphModel(windowValue, { presentation: options.presentation });
    return paint();
  }

  function resize() {
    return paint();
  }

  function syncLabels() {
    if (destroyed) return false;
    for (const metric of metrics) {
      const card = cards.get(metric.id);
      if (!card) continue;
      card.label.textContent = textValue(options, metric.labelKey, metric.label);
      card.card.setAttribute?.("aria-label", card.label.textContent);
      card.unit.textContent = metric.unit;
    }
    refreshLineColors(true);
    return true;
  }

  function status() {
    return Object.freeze({ destroyed, state: model.state, freshness: model.freshness });
  }

  function destroy() {
    if (destroyed) return false;
    destroyed = true;
    ranges.clear();
    cards.clear();
    root.replaceChildren?.();
    return true;
  }

  return Object.freeze({ render, resize, syncLabels, status, destroy });
}
