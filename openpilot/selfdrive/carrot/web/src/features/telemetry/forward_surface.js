const SVG_NS = "http://www.w3.org/2000/svg";

function append(parent, ...children) {
  if (typeof parent?.append === "function") parent.append(...children);
  else for (const child of children) parent?.appendChild?.(child);
}

function addClass(element, name) {
  element?.classList?.add(name);
  return element;
}

function createEgo(documentRoot) {
  const ego = documentRoot.createElementNS(SVG_NS, "svg");
  ego.setAttribute("viewBox", "0 0 24 40");
  ego.setAttribute("aria-hidden", "true");
  ego.setAttribute("focusable", "false");
  ego.classList.add("telemetry-forward__ego");
  ego.innerHTML = [
    '<rect class="telemetry-forward__ego-mirror" x="1.4" y="11.4" width="2.9" height="3.2" rx="1.1"></rect>',
    '<rect class="telemetry-forward__ego-mirror" x="19.7" y="11.4" width="2.9" height="3.2" rx="1.1"></rect>',
    '<path d="M12 1.6c2.2 0 3.6.5 4.4 1.5l2.3 2.9c1 1.3 1.5 2.9 1.5 4.7v20.1c0 2.4-.6 4.1-1.9 5.2l-1.7 1.5c-1.2 1-2.8 1.4-4.6 1.4s-3.4-.4-4.6-1.4l-1.7-1.5c-1.3-1.1-1.9-2.8-1.9-5.2V10.7c0-1.8.5-3.4 1.5-4.7l2.3-2.9C8.4 2.1 9.8 1.6 12 1.6z"></path>',
    '<path class="telemetry-forward__ego-glass" d="M8.8 10.4h6.4l1.5 4.4H7.3z"></path>',
    '<path class="telemetry-forward__ego-glass" d="M7.7 28h8.6l-1.1 3.6H8.8z"></path>',
  ].join("");
  return ego;
}

function createTooltip(documentRoot) {
  const tooltip = documentRoot.createElement("div");
  tooltip.className = "telemetry-forward__tooltip";
  tooltip.hidden = true;
  const icon = documentRoot.createElement("span");
  icon.className = "telemetry-forward__tooltip-icon";
  icon.setAttribute("aria-hidden", "true");
  const title = documentRoot.createElement("strong");
  title.className = "telemetry-forward__tooltip-title";
  const state = documentRoot.createElement("span");
  state.className = "telemetry-forward__tooltip-state";
  const rows = documentRoot.createElement("dl");
  rows.className = "telemetry-forward__tooltip-rows";
  append(tooltip, icon, title, state, rows);
  return Object.freeze({ tooltip, icon, title, state, rows });
}

function decorate(elements) {
  addClass(elements.root, "telemetry-forward");
  addClass(elements.head, "telemetry-forward__head");
  addClass(elements.legend, "telemetry-forward__legend");
  addClass(elements.viewport, "telemetry-forward__viewport");
  addClass(elements.plot, "telemetry-forward__plot");
  addClass(elements.canvas, "telemetry-forward__canvas");
  addClass(elements.ego, "telemetry-forward__ego");
  addClass(elements.message, "telemetry-forward__message");
  addClass(elements.tooltip, "telemetry-forward__tooltip");
  addClass(elements.tooltipIcon, "telemetry-forward__tooltip-icon");
  addClass(elements.tooltipTitle, "telemetry-forward__tooltip-title");
  addClass(elements.tooltipState, "telemetry-forward__tooltip-state");
  addClass(elements.tooltipRows, "telemetry-forward__tooltip-rows");
  addClass(elements.ego?.querySelector?.(".carrot-replay-sensor__egoMirror"), "telemetry-forward__ego-mirror");
  for (const mirror of elements.ego?.querySelectorAll?.(".carrot-replay-sensor__egoMirror") || []) {
    addClass(mirror, "telemetry-forward__ego-mirror");
  }
  for (const glass of elements.ego?.querySelectorAll?.(".carrot-replay-sensor__egoGlass") || []) {
    addClass(glass, "telemetry-forward__ego-glass");
  }
  return elements;
}

function adoptedElements(options) {
  const root = options.root;
  const tooltip = options.tooltip || root?.querySelector?.(".carrot-replay-sensor__tooltip");
  return {
    root,
    head: options.head || root?.querySelector?.(".carrot-replay-sensor__head"),
    title: options.title || root?.querySelector?.(".carrot-replay-sensor__head strong"),
    status: options.status || root?.querySelector?.(".carrot-replay-sensor__head span"),
    legend: options.legend || root?.querySelector?.(".carrot-replay-sensor__legend"),
    viewport: options.viewport || root?.querySelector?.(".carrot-replay-sensor__viewport"),
    plot: options.plot || root?.querySelector?.(".carrot-replay-sensor__plot"),
    canvas: options.canvas || root?.querySelector?.("canvas"),
    ego: options.ego || root?.querySelector?.(".carrot-replay-sensor__ego"),
    message: options.message || root?.querySelector?.(".carrot-replay-sensor__message"),
    tooltip,
    tooltipIcon: options.tooltipIcon || tooltip?.querySelector?.(".carrot-replay-sensor__tooltipIcon"),
    tooltipTitle: options.tooltipTitle || tooltip?.querySelector?.(".carrot-replay-sensor__tooltipTitle"),
    tooltipState: options.tooltipState || tooltip?.querySelector?.(".carrot-replay-sensor__tooltipState"),
    tooltipRows: options.tooltipRows || tooltip?.querySelector?.(".carrot-replay-sensor__tooltipRows"),
  };
}

function createdElements(root, options) {
  const documentRoot = options.document || root?.ownerDocument || globalThis.document;
  if (!root || !documentRoot?.createElement) return null;
  const head = documentRoot.createElement("div");
  const title = documentRoot.createElement("strong");
  const status = documentRoot.createElement("span");
  append(head, title, status);
  const legend = documentRoot.createElement("div");
  const viewport = documentRoot.createElement("div");
  viewport.setAttribute("role", "region");
  viewport.tabIndex = 0;
  const plot = documentRoot.createElement("div");
  const canvas = documentRoot.createElement("canvas");
  const ego = createEgo(documentRoot);
  const tooltipParts = createTooltip(documentRoot);
  append(plot, canvas, ego, tooltipParts.tooltip);
  viewport.append(plot);
  const message = documentRoot.createElement("div");
  message.setAttribute("role", "status");
  message.hidden = true;
  append(root, head, legend, viewport, message);
  return {
    root, head, title, status, legend, viewport, plot, canvas, ego, message,
    tooltip: tooltipParts.tooltip,
    tooltipIcon: tooltipParts.icon,
    tooltipTitle: tooltipParts.title,
    tooltipState: tooltipParts.state,
    tooltipRows: tooltipParts.rows,
  };
}

export function createTelemetryForwardSurface(options = {}) {
  const elements = decorate(options.adopt ? adoptedElements(options) : createdElements(options.root, options));
  if (!elements?.root || !elements.canvas) return null;

  function setHeader(title, status = "", warning = false) {
    if (elements.title) elements.title.textContent = title;
    if (elements.status) {
      elements.status.textContent = status;
      elements.status.classList?.toggle("is-warn", warning);
    }
  }

  function setMessage(message = "") {
    if (!elements.message) return;
    elements.message.textContent = message;
    elements.message.hidden = !message;
  }

  function setLegend(entries = [], label = "") {
    if (!elements.legend) return;
    const documentRoot = elements.legend.ownerDocument || globalThis.document;
    const items = entries.map((entry) => {
      const item = documentRoot.createElement("span");
      item.className = "telemetry-forward__legend-item";
      const glyph = documentRoot.createElement("i");
      glyph.className = `telemetry-forward__glyph telemetry-forward__glyph--${entry.shape || "circle"}`;
      glyph.style.setProperty("--glyph-color", entry.color || "currentColor");
      glyph.setAttribute("aria-hidden", "true");
      append(item, glyph, documentRoot.createTextNode(String(entry.label || "")));
      return item;
    });
    elements.legend.replaceChildren(...items);
    if (label) elements.legend.setAttribute("aria-label", label);
  }

  function hideTooltip() {
    if (elements.tooltip) elements.tooltip.hidden = true;
  }

  function showTooltip(descriptor = {}) {
    if (!elements.tooltip || !elements.tooltipIcon || !elements.tooltipRows) return false;
    const shape = String(descriptor.shape || "circle");
    elements.tooltip.style.setProperty("--glyph-color", descriptor.color || "currentColor");
    elements.tooltipIcon.className = `telemetry-forward__tooltip-icon telemetry-forward__glyph telemetry-forward__glyph--${shape}`;
    if (elements.tooltipTitle) elements.tooltipTitle.textContent = String(descriptor.title || "");
    if (elements.tooltipState) {
      elements.tooltipState.textContent = String(descriptor.state || "");
      elements.tooltipState.classList.toggle("is-lead", descriptor.lead === true);
    }
    const documentRoot = elements.tooltipRows.ownerDocument || globalThis.document;
    const rows = [];
    for (const row of Array.isArray(descriptor.rows) ? descriptor.rows : []) {
      const term = documentRoot.createElement("dt");
      term.textContent = String(row?.label || "");
      const detail = documentRoot.createElement("dd");
      detail.textContent = String(row?.value || "");
      rows.push(term, detail);
    }
    elements.tooltipRows.replaceChildren(...rows);
    const width = Math.max(1, Number(descriptor.width) || elements.plot?.clientWidth || 1);
    const height = Math.max(1, Number(descriptor.height) || elements.plot?.clientHeight || 1);
    const x = Number(descriptor.x) || 0;
    const y = Number(descriptor.y) || 0;
    const horizontalInset = Math.min(110, Math.max(8, width / 2));
    elements.tooltip.style.left = `${Math.max(horizontalInset, Math.min(width - horizontalInset, x))}px`;
    elements.tooltip.style.top = `${Math.max(8, Math.min(height - 8, y))}px`;
    elements.tooltip.classList.toggle("is-below", descriptor.below ?? y < 128);
    elements.tooltip.hidden = false;
    return true;
  }

  return Object.freeze({
    elements: Object.freeze(elements),
    setHeader,
    setMessage,
    setLegend,
    showTooltip,
    hideTooltip,
  });
}
