import assert from "node:assert/strict";
import test from "node:test";

import {
  renderPopularChipHtml,
  renderPopularChipText,
  renderPopularDetailHtml,
} from "../src/features/settings/popular/popular_render.js";

const escape = (value) => String(value ?? "").replace(/[&<>"]/g, (ch) => (
  { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[ch]
));

const opts = { escape, formatValue: (v) => String(v) };

test("a clear winner renders a single-value chip", () => {
  const entry = { top_values: [{ value: 1, count: 5 }, { value: 2, count: 3 }] };
  assert.equal(renderPopularChipText(entry, opts), "내 차종 인기값 (5대) 1");
});

test("a two-way tie renders both values", () => {
  const entry = { top_values: [{ value: 1, count: 5 }, { value: 2, count: 5 }] };
  assert.equal(renderPopularChipText(entry, opts), "내 차종 인기값 (각 5대) 1 · 2");
});

test("nothing summarizable renders an empty chip", () => {
  assert.equal(renderPopularChipText({ top_values: [{ value: 1, count: 1 }] }, opts), "");
  assert.equal(renderPopularChipText({ top_values: [] }, opts), "");
});

test("the chip html carries the car label and accent spans", () => {
  const entry = { top_values: [{ value: 1, count: 5 }] };
  const html = renderPopularChipHtml(entry, opts);
  assert.match(html, /setting-popular-value-chip__car">내 차종 인기값/);
  assert.match(html, /setting-popular-value-chip__accent">5대/);
  assert.match(html, /setting-popular-value-chip__accent">1</);
});

test("the detail lists every candidate with a semantic progress bar", () => {
  const entry = { top_values: [{ value: 1, count: 10 }, { value: 2, count: 5 }] };
  const html = renderPopularDetailHtml(entry, { ...opts, title: "Casper EV 인기값" });
  assert.match(html, /setting-popular-detail__name">Casper EV 인기값/);
  assert.equal(html.match(/setting-popular-detail__row"/g).length, 2);
  assert.match(html, /<progress[^>]+max="10"[^>]+value="10"/);
  assert.match(html, /<progress[^>]+max="10"[^>]+value="5"/);
  assert.match(html, /data-setting-popular-value="1"/);
  assert.doesNotMatch(html, /style=/);
});

test("the detail bar is omitted when there is a single value", () => {
  const html = renderPopularDetailHtml({ top_values: [{ value: 1, count: 3 }] }, opts);
  assert.match(html, /setting-popular-detail--single/);
  assert.doesNotMatch(html, /setting-popular-detail__bar/);
});

test("an empty detail renders the empty state", () => {
  const html = renderPopularDetailHtml({ top_values: [] }, opts);
  assert.match(html, /setting-popular-detail__empty/);
  assert.doesNotMatch(html, /setting-popular-detail__row/);
});

test("the updated line appears only when a time is supplied", () => {
  const entry = { top_values: [{ value: 1, count: 3 }] };
  assert.doesNotMatch(renderPopularDetailHtml(entry, opts), /__updated/);
  const withTime = renderPopularDetailHtml(entry, { ...opts, updatedText: "2026-07-20 20:00" });
  assert.match(withTime, /setting-popular-detail__updated/);
  assert.match(withTime, /최근 업데이트: 2026-07-20 20:00/);
  assert.doesNotMatch(withTime, /style=/);
});

test("values, title and raw data-value are escaped", () => {
  const entry = { top_values: [{ value: "<v>", count: 3 }] };
  const html = renderPopularDetailHtml(entry, {
    escape,
    formatValue: (v) => String(v),
    title: "<t>",
  });
  assert.doesNotMatch(html, /<v>|<t>/);
  assert.match(html, /&lt;v&gt;/);
  assert.match(html, /data-setting-popular-value="&lt;v&gt;"/);
  assert.match(html, /__name">&lt;t&gt;/);
});

test("format and translation providers are used", () => {
  const entry = { top_values: [{ value: 1, count: 5 }] };
  const text = renderPopularChipText(entry, {
    ...opts,
    formatValue: (v) => `${v}%`,
    text: (key, fallback, vars) => (key === "setting_popular_value_chip"
      ? `인기 ${vars.value} (${vars.sample})`
      : fallback),
  });
  assert.equal(text, "인기 1% (5)");
});
