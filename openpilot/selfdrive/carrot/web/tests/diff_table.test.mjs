import assert from "node:assert/strict";
import test from "node:test";

import {
  getDiffSelectedCount,
  renderDiffSummary,
  renderDiffTable,
} from "../src/ui/components/diff_table/diff_table.js";

const escape = (value) => String(value ?? "").replace(/[&<>"]/g, (ch) => (
  { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[ch]
));

function render(preview, options = {}) {
  return renderDiffTable(preview, { escape, ...options });
}

test("selected count prefers the summary, then falls back to counting entries", () => {
  assert.equal(getDiffSelectedCount({ summary: { selected: 3 } }), 3);
  assert.equal(getDiffSelectedCount({ entries: [{ apply: true }, { apply: false }, { apply: true }] }), 2);
  assert.equal(getDiffSelectedCount({}), 0);
  assert.equal(getDiffSelectedCount(null), 0);
});

test("the summary shows every bucket with its count", () => {
  const html = renderDiffSummary({ changed: 2, same: 162, skipped: 0, invalid: 3 }, { escape });
  assert.match(html, /settings-diff-summary__item--changed[^]*<strong>2<\/strong>/);
  assert.match(html, /settings-diff-summary__item--same[^]*<strong>162<\/strong>/);
  assert.match(html, /settings-diff-summary__item--invalid[^]*<strong>3<\/strong>/);
});

test("a changed entry renders its current and next value", () => {
  const html = render({
    summary: { changed: 1 },
    entries: [{ key: "ApplyModelSpeed", current: 0, value: -1, status: "changed", apply: true }],
  });
  assert.match(html, /settings-diff__key">ApplyModelSpeed/);
  assert.match(html, /settings-diff__value--old[^]*<code>0<\/code>/);
  assert.match(html, /settings-diff__value--new[^]*<code>-1<\/code>/);
});

test("nothing to apply shows the empty state, not a broken list", () => {
  const html = render({ summary: { changed: 0, same: 5 }, entries: [{ key: "A", status: "same" }] });
  assert.match(html, /settings-diff-empty/);
  assert.doesNotMatch(html, /settings-diff__row/);
});

// The reason the reported bug was invisible: "invalid 3" with no detail.
test("invalid entries are disclosed with their key and reason", () => {
  const html = render({
    summary: { changed: 0, same: 162, invalid: 3 },
    entries: [
      { key: "NewParamA", status: "invalid", reason: "unknown parameter" },
      { key: "NewParamB", status: "invalid", reason: "unknown parameter" },
    ],
  });
  assert.match(html, /<details class="settings-diff-invalid">/);
  assert.match(html, /settings-diff-invalid__key">NewParamA/);
  assert.match(html, /settings-diff-invalid__reason">unknown parameter/);
});

test("the invalid detail appears even when there is nothing to apply", () => {
  const html = render({
    summary: { changed: 0, invalid: 1 },
    entries: [{ key: "X", status: "invalid", reason: "bad" }],
  });
  assert.match(html, /settings-diff-empty/);
  assert.match(html, /settings-diff-invalid__key">X/);
});

test("an invalid entry without a reason still lists its key", () => {
  const html = render({ entries: [{ key: "X", status: "invalid" }] });
  assert.match(html, /settings-diff-invalid__key">X/);
  assert.doesNotMatch(html, /settings-diff-invalid__reason/);
});

test("with no invalid entries the disclosure is omitted", () => {
  const html = render({
    summary: { changed: 1 },
    entries: [{ key: "A", current: 1, value: 2, status: "changed", apply: true }],
  });
  assert.doesNotMatch(html, /settings-diff-invalid/);
});

test("a long change list is capped and the overflow is reported", () => {
  const entries = Array.from({ length: 90 }, (_, i) => (
    { key: `P${i}`, current: 0, value: 1, status: "changed", apply: true }
  ));
  const html = render({ summary: { changed: 90 }, entries }, { limit: 80 });
  assert.equal(html.match(/settings-diff__row/g).length, 80);
  assert.match(html, /settings-diff-more/);
});

test("escaping is applied to keys, values and reasons", () => {
  const html = render({
    entries: [
      { key: "<k>", current: '"a"', value: "<b>", status: "changed", apply: true },
      { key: "<i>", status: "invalid", reason: "<r>" },
    ],
  });
  assert.doesNotMatch(html, /<k>|<b>|<r>/);
  assert.match(html, /&lt;k&gt;/);
  assert.match(html, /&lt;r&gt;/);
});

test("the text provider supplies labels and interpolates counts", () => {
  const html = render(
    { summary: { changed: 0 }, entries: [{ key: "X", status: "invalid" }] },
    { text: (key, fallback, vars) => (key === "settings_diff_invalid_detail" ? `오류 ${vars.count}건` : fallback) },
  );
  assert.match(html, /오류 1건/);
});

test("without a text provider the fallback strings interpolate on their own", () => {
  const entries = Array.from({ length: 82 }, (_, i) => (
    { key: `P${i}`, current: 0, value: 1, status: "changed", apply: true }
  ));
  const html = render({ entries }, { limit: 80 });
  assert.match(html, /2 more changes hidden/);
});
