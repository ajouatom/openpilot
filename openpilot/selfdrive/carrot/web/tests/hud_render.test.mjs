import assert from "node:assert/strict";
import test from "node:test";

import { createSpeedPanel } from "../src/features/drive/contents/vision/hud/widgets/speed_panel.js";
import { createLfaIcon } from "../src/features/drive/contents/vision/hud/widgets/lfa_icon.js";
import { mapPayload } from "../src/features/drive/contents/vision/hud/index.js";
import { deriveVehicleHudPayload } from "../src/features/drive/contents/vision/hud/data_bridge.js";
import { COLORS } from "../src/features/drive/contents/vision/hud/tokens.js";

/* Minimal DOM stub — enough for the SVG/DOM helpers the widgets use.
 * The widgets only touch: create(NS), setAttribute, className, textContent,
 * classList.toggle, style.*, append(Child), toggleAttribute. */
class FakeNode {
  constructor(name) {
    this.nodeName = name;
    this.attrs = Object.create(null);
    this.children = [];
    this._text = "";
    this.className = "";
    this.style = {};
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
  hasAttribute(k) { return k in this.attrs; }
  toggleAttribute(k, force) {
    const has = k in this.attrs;
    const on = force === undefined ? !has : Boolean(force);
    if (on) this.attrs[k] = ""; else delete this.attrs[k];
    return on;
  }
  appendChild(c) { if (c) this.children.push(c); return c; }
  append(...cs) { for (const c of cs) if (c) this.children.push(c); }
  getBBox() { return { x: 0, y: 0, width: 10, height: 10 }; }
}

const doc = {
  createElementNS: (_ns, name) => new FakeNode(name),
  createElement: (name) => new FakeNode(name),
};

function findByClass(root, cls) {
  const stack = [root];
  while (stack.length) {
    const node = stack.pop();
    if (!node || typeof node !== "object") continue;
    const classAttr = (node.getAttribute?.("class") || node.className || "").split(/\s+/);
    if (classAttr.includes(cls)) return node;
    if (node.children) stack.push(...node.children);
  }
  return null;
}

const isVisible = (node) => node != null && node.style.display !== "none";

/* Mirrors the live/replay glue in js/realtime/vision_raw.js deriveCompactHudPayload:
 * the vehicle-derived fields (evActive/activeLaneLine/cruiseOverride) are spread
 * onto the flat payload, then normalized by mapPayload. Both live and replay feed
 * the identical cereal-state shape, so one fixture exercises both. */
function renderFromCereal(state) {
  const vehicle = deriveVehicleHudPayload(state);
  const flat = {
    vEgoKph: Number(state.carState?.vEgoCluster ?? state.carState?.vEgo),
    vSetKph: Number(state.carState?.vCruiseCluster),
    isMetric: true,
    gear: vehicle.gear,
    gearStep: vehicle.gearStep,
    evActive: vehicle.evActive,
    activeLaneLine: vehicle.activeLaneLine,
    cruiseOverride: vehicle.cruiseOverride,
    latActive: vehicle.lfaActive,
  };
  const data = mapPayload(flat);
  const panel = createSpeedPanel(doc);
  const lfa = createLfaIcon(doc);
  panel.update(data);
  lfa.update(data);
  return {
    data,
    ev: findByClass(panel.el, "chud-t-ev"),
    overrideSpeed: findByClass(panel.el, "chud-t-override"),
    overrideLabel: findByClass(panel.el, "chud-t-override-label"),
    speed: findByClass(panel.el, "chud-t-speed"),
    lane: findByClass(lfa.el, "chud-lfa-lane"),
  };
}

// 당근비전(라이브): EV 켜짐 + 레인모드 + 감속(카메라) 오버라이드.
test("live: EV + green lane wings + orange decel override all render", () => {
  const r = renderFromCereal({
    carState: { vEgoCluster: 53, vCruiseCluster: 88, evModeValid: true, evModeActive: true, gearShifter: "drive" },
    controlsState: { activeLaneLine: true },
    carrotMan: { desiredSpeed: 77, desiredSource: "cam" },
  });
  assert.equal(r.speed.textContent, "53");
  assert.ok(isVisible(r.ev), "EV telltale visible");
  assert.equal(r.ev.textContent, "EV");
  assert.ok(isVisible(r.lane), "green lane wings visible");
  assert.ok(isVisible(r.overrideSpeed), "override visible");
  assert.equal(r.overrideSpeed.textContent, "77");
  assert.equal(r.overrideLabel.textContent, "cam:n");
  assert.equal(r.overrideSpeed.style.fill, COLORS.override); // orange (decel, mode 2)
});

// 리플레이: 동일 파이프라인. eco(초록) 오버라이드, EV/레인은 꺼진 케이스로 변형 검증.
test("replay: eco (green) override renders; EV and lane correctly hidden", () => {
  const r = renderFromCereal({
    carState: { vEgoCluster: 60, vCruiseCluster: 88, evModeValid: false, evModeActive: false, gearShifter: "drive" },
    controlsState: { activeLaneLine: false },
    longitudinalPlan: { cruiseTarget: 95 },
  });
  assert.equal(r.speed.textContent, "60");
  assert.ok(!isVisible(r.ev), "EV hidden when not in EV mode");
  assert.ok(!isVisible(r.lane), "lane wings hidden when lane mode off");
  assert.ok(isVisible(r.overrideSpeed), "eco override visible");
  assert.equal(r.overrideSpeed.textContent, "95");
  assert.equal(r.overrideLabel.textContent, "eco");
  assert.equal(r.overrideSpeed.style.fill, COLORS.carrot); // green (eco, mode 1)
});

// 주행 전/대기 화면: 조건 미충족 → 신규 3종 전부 숨김(스크린샷 상태).
test("pre-drive idle: EV, override and lane wings are all hidden", () => {
  const r = renderFromCereal({});
  assert.ok(!isVisible(r.ev), "EV hidden");
  assert.ok(!isVisible(r.overrideSpeed), "override hidden");
  assert.ok(!isVisible(r.lane), "lane wings hidden");
});

// mph 표시(비미터)에서도 오버라이드 속도가 변환되는지.
test("non-metric: override speed converts to mph", () => {
  const vehicle = deriveVehicleHudPayload({
    carState: { vCruiseCluster: 120 }, // decel override needs desiredSpeed < set speed
    carrotMan: { desiredSpeed: 100, desiredSource: "section" },
  });
  const data = mapPayload({ isMetric: false, cruiseOverride: vehicle.cruiseOverride, vSetKph: 120 });
  const panel = createSpeedPanel(doc);
  panel.update(data);
  const override = findByClass(panel.el, "chud-t-override");
  assert.equal(override.textContent, String(Math.round(100 * 0.621371))); // 62
  assert.equal(findByClass(panel.el, "chud-t-override-label").textContent, "section:n");
});
