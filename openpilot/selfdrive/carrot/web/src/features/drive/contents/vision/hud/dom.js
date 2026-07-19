"use strict";

// 작은 DOM/SVG 헬퍼. 위젯은 이 헬퍼로만 노드를 만든다(문자열 innerHTML 지양).
const SVG_NS = "http://www.w3.org/2000/svg";
const centeredTextTargets = new WeakMap();
const centeredTextSignatures = new WeakMap();
const fontReadySubscriptions = new WeakSet();

export function svg(doc, name, attrs = {}, children = []) {
  const node = doc.createElementNS(SVG_NS, name);
  for (const key in attrs) {
    if (attrs[key] == null) continue;
    node.setAttribute(key, String(attrs[key]));
  }
  for (const child of children) if (child) node.appendChild(child);
  return node;
}

export function el(doc, name, opts = {}, children = []) {
  const node = doc.createElement(name);
  if (opts.class) node.className = opts.class;
  if (opts.text != null) node.textContent = String(opts.text);
  if (opts.attrs) for (const key in opts.attrs) node.setAttribute(key, String(opts.attrs[key]));
  for (const child of children) if (child) node.appendChild(child);
  return node;
}

export function setText(node, value) {
  const next = value == null ? "" : String(value);
  if (node && node.textContent !== next) node.textContent = next;
}

function applySvgTextCenter(node, centerX, centerY) {
  if (!node || typeof node.getBBox !== "function") return false;
  let bounds;
  try {
    bounds = node.getBBox();
  } catch {
    return false;
  }
  if (![bounds.x, bounds.y, bounds.width, bounds.height].every(Number.isFinite)
      || bounds.width <= 0 || bounds.height <= 0) return false;
  const dx = centerX - (bounds.x + bounds.width * 0.5);
  const dy = centerY - (bounds.y + bounds.height * 0.5);
  node.setAttribute("transform", `translate(${dx.toFixed(3)} ${dy.toFixed(3)})`);
  return true;
}

// SVG baseline behavior varies by browser and font. Center the rendered glyph
// bounds on a fixed design-space point so every responsive scale is identical.
export function setCenteredSvgText(node, value, centerX, centerY) {
  const text = value == null ? "" : String(value);
  setText(node, text);
  const x = Number(centerX);
  const y = Number(centerY);
  if (!Number.isFinite(x) || !Number.isFinite(y)) return;
  centeredTextTargets.set(node, { x, y });
  const signature = `${text}\u0000${node.getAttribute?.("font-size") || ""}\u0000${x}\u0000${y}`;
  if (centeredTextSignatures.get(node) !== signature || !node.hasAttribute?.("transform")) {
    if (applySvgTextCenter(node, x, y)) centeredTextSignatures.set(node, signature);
  }

  const fonts = node?.ownerDocument?.fonts;
  if (!fonts?.ready || fontReadySubscriptions.has(node)) return;
  fontReadySubscriptions.add(node);
  fonts.ready.then(() => {
    const target = centeredTextTargets.get(node);
    if (target) applySvgTextCenter(node, target.x, target.y);
  }).catch(() => {});
}

export function setHidden(node, hidden) {
  node?.toggleAttribute?.("hidden", Boolean(hidden));
}
