import assert from "node:assert/strict";
import test from "node:test";

import {
  RISK_LEVELS,
  getSettingRiskLevel,
  renderSettingRiskBadge,
} from "../src/features/settings/risk.js";

const escape = (value) => String(value ?? "").replace(/[&<>"]/g, (ch) => (
  { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[ch]
));

test("only declared risk levels are recognised", () => {
  assert.deepEqual(RISK_LEVELS, ["high", "medium"]);
  assert.equal(getSettingRiskLevel({ risk: "high" }), "high");
  assert.equal(getSettingRiskLevel({ risk: "HIGH" }), "high");
  assert.equal(getSettingRiskLevel({ risk: "medium" }), "medium");
});

test("an unknown or missing risk resolves to null", () => {
  assert.equal(getSettingRiskLevel({ risk: "spicy" }), null);
  assert.equal(getSettingRiskLevel({}), null);
  assert.equal(getSettingRiskLevel(null), null);
});

test("a risky parameter renders a level-tagged badge", () => {
  const html = renderSettingRiskBadge({ risk: "high" }, {
    escape,
    text: (key, fallback) => (key === "setting_risk_high" ? "주의" : fallback),
  });
  assert.match(html, /chip chip--compact chip--warning/);
  assert.match(html, /setting-risk-badge--high/);
  assert.match(html, /data-risk-level="high"/);
  assert.match(html, />주의</);
});

test("a non-risky parameter renders nothing", () => {
  assert.equal(renderSettingRiskBadge({}, { escape }), "");
  assert.equal(renderSettingRiskBadge({ risk: "none" }, { escape }), "");
});

test("the badge label is escaped and translated", () => {
  const html = renderSettingRiskBadge({ risk: "medium" }, {
    escape,
    text: () => "<b>x</b>",
  });
  assert.doesNotMatch(html, /<b>x<\/b>/);
  assert.match(html, /&lt;b&gt;x&lt;\/b&gt;/);
});
