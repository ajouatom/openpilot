import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import test from "node:test";

import { createDeviceTemp, formatDeviceTemp } from "../src/features/drive/contents/vision/hud/widgets/device_temp.js";
import { createHudOverlay, mapPayload } from "../src/features/drive/contents/vision/hud/index.js";
import { CSS } from "../src/features/drive/contents/vision/hud/style.js";

const root = path.resolve(import.meta.dirname, "..");
const read = (relativePath) => fs.readFileSync(path.join(root, relativePath), "utf8");

/* Same minimal DOM stub the other HUD render tests use, plus the few methods the
 * overlay composition itself touches (head/getElementById/removeAttribute). */
class FakeNode {
  constructor(name) {
    this.nodeName = name;
    this.attrs = Object.create(null);
    this.children = [];
    this._text = "";
    this.className = "";
    this.style = {};
    this.dataset = Object.create(null);
    const classes = new Set();
    this.classList = {
      toggle: (c, force) => {
        const on = force === undefined ? !classes.has(c) : Boolean(force);
        if (on) classes.add(c); else classes.delete(c);
        return on;
      },
      add: (c) => classes.add(c),
      remove: (c) => classes.delete(c),
      contains: (c) => classes.has(c),
    };
  }
  get textContent() { return this._text; }
  set textContent(v) { this._text = v == null ? "" : String(v); }
  setAttribute(k, v) { this.attrs[k] = String(v); }
  getAttribute(k) { return k in this.attrs ? this.attrs[k] : null; }
  removeAttribute(k) { delete this.attrs[k]; }
  hasAttribute(k) { return k in this.attrs; }
  toggleAttribute(k, force) {
    const has = k in this.attrs;
    const on = force === undefined ? !has : Boolean(force);
    if (on) this.attrs[k] = ""; else delete this.attrs[k];
    return on;
  }
  appendChild(c) { if (c) this.children.push(c); return c; }
  append(...cs) { for (const c of cs) if (c) this.children.push(c); }
  querySelectorAll() { return []; }
  getBBox() { return { x: 0, y: 0, width: 10, height: 10 }; }
}

function createFakeDocument() {
  return {
    head: new FakeNode("head"),
    createElement: (name) => new FakeNode(name),
    createElementNS: (_ns, name) => new FakeNode(name),
    getElementById: () => null,
  };
}

const classesOf = (node) => String(node?.className || node?.getAttribute?.("class") || "").split(/\s+/);
const hasClass = (node, cls) => classesOf(node).includes(cls);

function findByClass(node, cls) {
  const stack = [node];
  while (stack.length) {
    const current = stack.pop();
    if (!current || typeof current !== "object") continue;
    if (hasClass(current, cls)) return current;
    if (current.children) stack.push(...current.children);
  }
  return null;
}

test("the hottest core is shown with one decimal, and missing data keeps a placeholder", () => {
  assert.equal(formatDeviceTemp(40.44), "40.4°C");
  assert.equal(formatDeviceTemp(40.45), "40.5°C");
  assert.equal(formatDeviceTemp(40), "40.0°C");
  assert.equal(formatDeviceTemp(null), "--°C");
  assert.equal(formatDeviceTemp(undefined), "--°C");
  assert.equal(formatDeviceTemp(Number.NaN), "--°C");
  assert.equal(formatDeviceTemp("52.75"), "52.8°C");
});

test("the widget renders the placeholder before any data and never hides itself", () => {
  const widget = createDeviceTemp(createFakeDocument());
  assert.equal(widget.el.textContent, "--°C");
  assert.equal(widget.el.getAttribute("role"), "status");

  widget.update({ cpuTemp: 61.28 });
  assert.equal(widget.el.textContent, "61.3°C");
  assert.equal(widget.el.hasAttribute("hidden"), false);

  // Data dropping out must not blank or hide the bar.
  widget.update({});
  assert.equal(widget.el.textContent, "--°C");
  assert.equal(widget.el.hasAttribute("hidden"), false);
});

test("the payload prefers the max core temperature over the averaged one", () => {
  assert.equal(mapPayload({ cpuTempMaxC: 55.2, cpuTempC: 48.1 }).cpuTemp, 55.2);
  // Paths that only carry the normalized average still render something.
  assert.equal(mapPayload({ cpuTempC: 48.1 }).cpuTemp, 48.1);
  assert.equal(mapPayload({}).cpuTemp, null);
  assert.equal(mapPayload({ cpuTempMaxC: "" }).cpuTemp, null);
});

test("the live and replay payload exposes both the averaged and the hottest core", () => {
  const visionRaw = read("js/realtime/vision_raw.js");

  assert.match(visionRaw, /function maxFiniteMetric\(values\)/);
  assert.match(visionRaw, /const cpuTempMaxC = pickFiniteMetric\(\s*maxFiniteMetric\(liveDeviceState\.cpuTempC\),\s*maxFiniteMetric\(rawDeviceState\.cpuTempC\),\s*\);/s);
  assert.match(visionRaw, /return \{ cpuTempC, cpuTempMaxC, memPct, diskPct, voltageV \};/);
});

test("the mini HUD shows the same reading as the vision corner bar", () => {
  // Legacy IIFE module, so the shared value is asserted at the source seam.
  const miniHud = read("js/realtime/mini_hud_model.js");
  assert.match(miniHud, /cpu: integer\(payload\?\.cpuTempMaxC \?\? payload\?\.cpuTempC\)/);
});

test("the temperature bar sits in a corner strip, behind every zone, and outside degradation", () => {
  const overlay = createHudOverlay(createFakeDocument());
  const corner = findByClass(overlay.root, "chud-corner--bl");

  assert.ok(corner, "the bottom-left corner strip is mounted");
  assert.ok(findByClass(corner, "chud-devtemp"), "the temperature bar lives in the corner strip");
  assert.equal(hasClass(corner, "chud-zone"), false, "the strip is not a layout zone");

  // Same z-index as the zones, but first in DOM order, so any overlap paints the
  // zones on top of the bar rather than the other way around.
  const order = overlay.root.children.map((child) => classesOf(child)[1] || classesOf(child)[0]);
  assert.equal(order[0], "chud-corner--bl");
  assert.deepEqual(order.slice(1), [
    "chud-zone--tl",
    "chud-zone--tc",
    "chud-zone--tr",
    "chud-zone--bl",
    "chud-zone--br",
  ]);

  overlay.update({ cpuTempMaxC: 47.62 });
  assert.equal(findByClass(corner, "chud-devtemp").textContent, "47.6°C");
});

test("the corner bar hugs the viewport and lifts the speed panel by its own height", () => {
  const rule = (selector) => {
    const start = CSS.indexOf(`${selector}{`);
    assert.notEqual(start, -1, `missing rule for ${selector}`);
    return CSS.slice(start, CSS.indexOf("}", start));
  };

  assert.match(rule(".chud-corner--bl"), /left:0;bottom:0/);
  assert.match(rule(".chud-corner"), /position:absolute;z-index:1/);
  // Flush to the corner: the strip itself carries no inset.
  assert.doesNotMatch(rule(".chud-corner--bl"), /padding/);

  const bar = rule(".chud-devtemp");
  assert.match(bar, /background:#000/);
  assert.match(bar, /color:var\(--chud-white\)/);
  assert.match(bar, /font-size:var\(--chud-devtemp-font\)/);
  assert.match(bar, /line-height:1/);

  // The zone anchor (not its padding) reserves the bar height, so the shed rules
  // that re-set `padding` cannot eat the reservation.
  assert.match(rule(".chud-zone--bl"), /bottom:var\(--chud-devtemp-height\)/);
  assert.match(CSS, /--chud-devtemp-height:calc\(var\(--chud-devtemp-font\) \+ var\(--chud-devtemp-pad-block\) \* 2\)/);

  // Never shed: no degradation rule may target the strip or the bar.
  for (const shedRule of CSS.match(/\.chud\[data-shed[^}]*\}/g) || []) {
    assert.doesNotMatch(shedRule, /chud-corner|chud-devtemp/);
  }
});
