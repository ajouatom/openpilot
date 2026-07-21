import assert from "node:assert/strict";
import test from "node:test";

import {
  CHANGE_SOURCE_LABEL_KEYS,
  formatChangeTimestamp,
  renderSettingHistoryHtml,
  toIsoTimestamp,
} from "../src/features/settings/history/change_history.js";

const NOW_SEC = 1_800_000_000;

const escape = (value) => String(value ?? "").replace(/[&<>"]/g, (ch) => (
  { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[ch]
));

function render(changes, options = {}) {
  return renderSettingHistoryHtml(changes, {
    escape,
    nowSec: NOW_SEC,
    locale: "en-US",
    formatValue: (value) => `${value}%`,
    ...options,
  });
}

test("an epoch second becomes an ISO 8601 stamp", () => {
  assert.equal(toIsoTimestamp(0), "");
  assert.equal(toIsoTimestamp(-5), "");
  assert.equal(toIsoTimestamp("nope"), "");
  assert.equal(toIsoTimestamp(NOW_SEC).endsWith("Z"), true);
});

// A relative string cannot separate two changes made seconds apart, which is
// exactly what this block is used for.
test("a timestamp carries the machine value and an exact time with seconds", () => {
  const stamp = formatChangeTimestamp(NOW_SEC - 120, { locale: "en-US" });
  assert.equal(stamp.iso.endsWith("Z"), true);
  assert.match(stamp.absolute, /\d{2}:\d{2}:\d{2}/, "seconds must be shown");
  assert.deepEqual(formatChangeTimestamp(0), { iso: "", absolute: "" });
});

test("an empty history renders the empty state, not a broken list", () => {
  const html = render([]);
  assert.match(html, /setting-history-detail--empty/);
  assert.match(html, /기록된 변경이 없습니다/);
  assert.doesNotMatch(html, /setting-history-detail__row/);
  assert.equal(render(null), render([]));
});

test("a record shows what the value was and what it became", () => {
  const html = render([{ ts: NOW_SEC - 60, name: "ApplyModelSpeed", prev: 0, next: -1, source: "web_ui" }]);
  assert.match(html, /setting-history-detail__prev">0%/);
  assert.match(html, /setting-history-detail__next">-1%/);
});

test("the time cell is a <time> element with an ISO datetime", () => {
  const html = render([{ ts: NOW_SEC - 120, prev: 1, next: 2, source: "web_ui" }]);
  const iso = toIsoTimestamp(NOW_SEC - 120);
  assert.match(html, new RegExp(`<time class="setting-history-detail__time" datetime="${iso}">`));
});

test("the visible time is the exact timestamp, not a relative phrase", () => {
  const html = render([{ ts: NOW_SEC - 120, prev: 1, next: 2, source: "web_ui" }]);
  assert.match(html, /\d{2}:\d{2}:\d{2}/);
  assert.doesNotMatch(html, /전<\/time>/, "no relative phrasing should remain");
  assert.doesNotMatch(html, /<abbr/, "the full time needs no abbreviation");
});

test("every change carries a source badge", () => {
  const html = render([{ ts: NOW_SEC, prev: 1, next: 2, source: "web_ui" }], {
    text: (key, fallback) => (key === CHANGE_SOURCE_LABEL_KEYS.web_ui ? "웹 설정" : fallback),
  });
  assert.match(html, /setting-history-detail__badge">웹 설정/);
});

test("a change made outside the web UI is labelled as such", () => {
  const html = render([{ ts: NOW_SEC, prev: 1, next: 2, source: "device" }], {
    text: (key, fallback) => (key === CHANGE_SOURCE_LABEL_KEYS.device ? "기기에서 변경" : fallback),
  });
  assert.match(html, /setting-history-detail__badge">기기에서 변경/);
});

test("every source maps to its own label key", () => {
  for (const [source, key] of Object.entries(CHANGE_SOURCE_LABEL_KEYS)) {
    const html = render([{ ts: NOW_SEC, prev: 1, next: 2, source }], {
      text: (lookup, fallback) => (lookup === key ? `LABEL:${source}` : fallback),
    });
    assert.match(html, new RegExp(`LABEL:${source}`));
  }
});

test("an unrecognised source falls back to the unknown label", () => {
  const html = render([{ ts: NOW_SEC, prev: 1, next: 2, source: "something_else" }], {
    text: (key, fallback) => (key === CHANGE_SOURCE_LABEL_KEYS.unknown ? "UNKNOWN" : fallback),
  });
  assert.match(html, /UNKNOWN/);
});

test("a change made while driving is marked", () => {
  const driving = render([{ ts: NOW_SEC, prev: 1, next: 2, source: "web_ui", engaged: true }]);
  const parked = render([{ ts: NOW_SEC, prev: 1, next: 2, source: "web_ui", engaged: false }]);
  assert.match(driving, /badge--engaged/);
  assert.doesNotMatch(parked, /badge--engaged/);
});

// Undoing an older entry would silently drop everything after it.
test("only the newest record offers undo", () => {
  const html = render([
    { ts: NOW_SEC, prev: 5, next: 6, source: "web_ui" },
    { ts: NOW_SEC - 60, prev: 4, next: 5, source: "web_ui" },
  ]);
  assert.equal(html.match(/data-setting-history-undo/g).length, 1);
  assert.match(html, /data-setting-history-undo="5"/);
});

test("the record count is shown in the header", () => {
  const html = render([
    { ts: NOW_SEC, prev: 1, next: 2, source: "web_ui" },
    { ts: NOW_SEC - 10, prev: 0, next: 1, source: "web_ui" },
  ]);
  assert.match(html, /최근 2건/);
});

test("the block mirrors the popular-value structure", () => {
  const html = render([{ ts: NOW_SEC, prev: 1, next: 2, source: "web_ui" }]);
  for (const part of ["__head", "__name", "__rows", "__row", "__marker", "__main"]) {
    assert.match(html, new RegExp(`setting-history-detail${part}`));
  }
});

test("values and labels are escaped", () => {
  const html = render([{ ts: NOW_SEC, prev: '<img src=x>', next: '"&"', source: "web_ui" }], {
    formatValue: (value) => String(value),
  });
  assert.doesNotMatch(html, /<img/);
  assert.match(html, /&lt;img/);
  assert.match(html, /&quot;&amp;&quot;/);
});

test("withName shows the parameter name and drops undo (across-parameter list)", () => {
  const html = render([
    { ts: NOW_SEC, name: "ApplyModelSpeed", prev: 0, next: -1, source: "web_ui" },
    { ts: NOW_SEC - 60, name: "TFollowDecelBoost", prev: 50, next: 40, source: "profile" },
  ], { withName: true });
  assert.match(html, /setting-history-detail__param">ApplyModelSpeed/);
  assert.match(html, /setting-history-detail__param">TFollowDecelBoost/);
  assert.doesNotMatch(html, /data-setting-history-undo/, "an across-parameter list has no single row to undo");
});

test("without withName no parameter name is shown", () => {
  const html = render([{ ts: NOW_SEC, name: "ApplyModelSpeed", prev: 0, next: -1, source: "web_ui" }]);
  assert.doesNotMatch(html, /setting-history-detail__param/);
});

// A non-developer reads the human title, not the internal key.
test("displayName maps the raw key to a human title", () => {
  const html = render([{ ts: NOW_SEC, name: "ShowTpms", prev: 0, next: 1, source: "web_ui" }], {
    withName: true,
    displayName: (record) => (record.name === "ShowTpms" ? "타이어공기압 표시" : record.name),
  });
  assert.match(html, /setting-history-detail__param">타이어공기압 표시/);
  assert.doesNotMatch(html, /ShowTpms/);
});

// Each value is formatted by its own parameter (units, ON/OFF), not raw.
test("formatFor formats each row's value by its own parameter", () => {
  const html = render([
    { ts: NOW_SEC, name: "ShowTpms", prev: 0, next: 1, source: "web_ui" },
    { ts: NOW_SEC - 10, name: "ApplyModelSpeed", prev: 0, next: -1, source: "web_ui" },
  ], {
    withName: true,
    formatFor: (record, value) => (record.name === "ShowTpms"
      ? (value ? "켜짐" : "꺼짐")
      : `${value}%`),
  });
  assert.match(html, /setting-history-detail__prev">꺼짐/);
  assert.match(html, /setting-history-detail__next">켜짐/);
  assert.match(html, /setting-history-detail__next">-1%/);
});

test("a record without a usable timestamp still renders", () => {
  const html = render([{ prev: 1, next: 2, source: "web_ui" }]);
  assert.match(html, /setting-history-detail__time">-/);
  assert.doesNotMatch(html, /<time/);
});
