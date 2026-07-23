function node(tag, className = "", text = "") {
  const element = document.createElement(tag);
  if (className) element.className = className;
  if (text !== "") element.textContent = String(text);
  return element;
}

export function createSummarySection(title) {
  const section = node("section", "route-summary__section");
  const heading = node("h3", "route-summary__section-title", title);
  section.append(heading);
  return Object.freeze({ section, heading });
}

export function createSummaryMetric(label, metric, options = {}) {
  const item = node("div", "route-summary-metric");
  if (options.variant) item.dataset.variant = options.variant;
  const term = node("dt", "route-summary-metric__label", label);
  if (options.hideLabel) term.classList.add("visually-hidden");
  const description = node("dd", "route-summary-metric__value");
  description.append(node("span", "route-summary-metric__number", metric.value));
  if (metric.unit) description.append(node("span", "route-summary-metric__unit", metric.unit));
  item.append(term, description);
  return item;
}

export function createCompositionStat(label, item) {
  const row = node("div", "route-summary-composition__stat");
  row.dataset.kind = item.key;
  if (item.emphasis) row.dataset.emphasis = "true";
  row.append(
    node("span", "route-summary-composition__label", label),
    node("span", "route-summary-composition__duration", item.duration),
    node("span", "route-summary-composition__ratio", `${item.ratio.toFixed(1)}%`),
  );
  return row;
}

export function compositionDistributionSegments(items) {
  const byKey = Object.fromEntries(items.map((item) => [item.key, Math.max(0, Number(item.seconds) || 0)]));
  const auto = byKey.auto || 0;
  const manual = byKey.manual || 0;
  let gas = byKey.gas || 0;
  let brake = byKey.brake || 0;
  if (gas + brake > manual && gas + brake > 0) {
    const scale = manual / (gas + brake);
    gas *= scale;
    brake *= scale;
  }
  return Object.freeze({
    auto,
    manual,
    gas,
    brake,
    other: Math.max(0, manual - gas - brake),
  });
}

function weightedSegment(kind, weight) {
  const segment = node("span", "route-summary-distribution__segment");
  segment.dataset.kind = kind;
  segment.style.setProperty("--route-summary-weight", String(weight));
  return segment;
}

export function createCompositionDistribution(items) {
  const track = node("div", "route-summary-distribution");
  track.setAttribute("role", "img");
  track.setAttribute("aria-label", items
    .map((item) => `${item.label} ${item.ratio.toFixed(1)}%`)
    .join(", "));
  const segments = compositionDistributionSegments(items);
  if (segments.auto > 0) track.append(weightedSegment("auto", segments.auto));
  if (segments.manual > 0) {
    const manual = node("span", "route-summary-distribution__manual");
    manual.style.setProperty("--route-summary-weight", String(segments.manual));
    if (segments.gas > 0) manual.append(weightedSegment("gas", segments.gas));
    if (segments.brake > 0) manual.append(weightedSegment("brake", segments.brake));
    if (segments.other > 0) manual.append(weightedSegment("other", segments.other));
    track.append(manual);
  }
  return track;
}

export function createEventRow(label, event, options = {}) {
  const row = node("div", "route-summary-event");
  if (options.kind) row.dataset.kind = options.kind;
  if (options.critical && event.count > 0) row.dataset.tone = "danger";
  const head = node("div", "route-summary-event__head");
  head.append(
    node("span", "route-summary-event__label", label),
    node("span", "route-summary-event__count", String(event.count)),
  );
  row.append(head);
  if (event.items.length) {
    const list = node("ul", "route-summary-event__times");
    for (const item of event.items) {
      const chip = node("li", "c-chip route-summary-event__time");
      chip.append(node("time", "", item.time));
      if (item.peak) chip.append(node("span", "route-summary-event__peak", item.peak));
      list.append(chip);
    }
    row.append(list);
  }
  return row;
}

export function createDetailRow(label, item, countLabel) {
  const row = node("div", "route-summary-detail");
  const value = item.unit === "count" ? `${item.value}${countLabel}` : `${item.value} ${item.unit}`.trim();
  row.append(node("dt", "route-summary-detail__label", label), node("dd", "route-summary-detail__value", value));
  return row;
}

export function createSummaryState(title, description = "") {
  const state = node("div", "route-summary-state");
  state.setAttribute("role", "status");
  state.setAttribute("aria-live", "polite");
  state.append(node("strong", "route-summary-state__title", title));
  const detail = node("span", "route-summary-state__description", description);
  detail.hidden = !description;
  const progress = node("div", "route-summary-state__progress");
  progress.setAttribute("role", "progressbar");
  progress.setAttribute("aria-label", title);
  progress.setAttribute("aria-valuemin", "0");
  progress.setAttribute("aria-valuemax", "100");
  progress.dataset.mode = "indeterminate";
  const progressFill = node("span", "route-summary-state__progress-fill");
  progress.append(progressFill);
  state.append(detail, progress);
  return Object.freeze({
    element: state,
    update(value, nextDescription) {
      const normalized = Number(value);
      if (Number.isFinite(normalized)) {
        const ratio = Math.max(0, Math.min(1, normalized));
        const percent = Math.round(ratio * 100);
        progress.dataset.mode = "determinate";
        progress.setAttribute("aria-valuenow", String(percent));
        progressFill.style.setProperty("--route-summary-progress", String(percent));
      } else {
        progress.dataset.mode = "indeterminate";
        progress.removeAttribute("aria-valuenow");
        progressFill.style.removeProperty("--route-summary-progress");
      }
      detail.textContent = String(nextDescription || "");
      detail.hidden = !detail.textContent;
    },
  });
}
