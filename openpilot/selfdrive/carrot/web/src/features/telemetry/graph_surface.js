const METRIC_TONES = Object.freeze({
  speed: "info",
  steering: "primary",
  acceleration: "success",
  lead_distance: "warning",
});

function requireDocument(root, options) {
  return options?.document || root?.ownerDocument || globalThis.document;
}

function metricTone(metric) {
  return METRIC_TONES[metric?.id] || "info";
}

function decorateCard(card, metricId) {
  if (!card) return null;
  card.classList?.add("telemetry-graph");
  card.dataset.telemetryMetric = metricId;
  const head = card.querySelector?.(".carrot-replay-insights__graphHead, header");
  const plot = card.querySelector?.("svg, canvas");
  const cursor = card.querySelector?.(".carrot-replay-insights__cursor, .telemetry-graph__cursor");
  head?.classList?.add("telemetry-graph__head");
  plot?.classList?.add("telemetry-graph__plot");
  cursor?.classList?.add("telemetry-graph__cursor");
  return Object.freeze({ card, head, plot, cursor });
}

export function adoptTelemetryGraphSurface(root, cards = []) {
  if (!root) return null;
  root.classList?.add("telemetry-graphs");
  const adopted = new Map();
  for (const card of cards) {
    const metricId = String(card?.dataset?.replayGraph || card?.dataset?.telemetryMetric || "");
    if (!metricId) continue;
    const sharedId = metricId === "steer"
      ? "steering"
      : metricId === "accel"
        ? "acceleration"
        : metricId === "lead"
          ? "lead_distance"
          : metricId;
    card.dataset.telemetryTone = METRIC_TONES[sharedId] || "info";
    const elements = decorateCard(card, sharedId);
    if (elements) adopted.set(sharedId, elements);
  }
  return Object.freeze({ root, cards: adopted });
}

export function createTelemetryGraphSurface(root, metrics, options = {}) {
  const documentRoot = requireDocument(root, options);
  if (!root || !documentRoot?.createElement || !Array.isArray(metrics)) return null;
  root.classList?.add("telemetry-graphs");
  const cards = new Map();

  for (const metric of metrics) {
    const card = documentRoot.createElement("article");
    card.className = "telemetry-graph";
    card.dataset.telemetryMetric = metric.id;
    card.dataset.telemetryTone = metricTone(metric);

    const head = documentRoot.createElement("header");
    head.className = "telemetry-graph__head";
    const label = documentRoot.createElement("strong");
    label.textContent = metric.label;
    const value = documentRoot.createElement("output");
    const number = documentRoot.createTextNode("--");
    const unit = documentRoot.createElement("small");
    unit.textContent = metric.unit;
    value.append(number, unit);
    head.append(label, value);

    const canvas = documentRoot.createElement("canvas");
    canvas.className = "telemetry-graph__plot";
    canvas.setAttribute("aria-hidden", "true");
    const cursor = documentRoot.createElement("i");
    cursor.className = "telemetry-graph__cursor telemetry-graph__cursor--live";
    cursor.setAttribute("aria-hidden", "true");
    card.append(head, canvas, cursor);
    root.append(card);
    cards.set(metric.id, Object.freeze({ card, head, label, value, number, unit, canvas, cursor }));
  }

  return Object.freeze({ root, cards });
}
