import assert from "node:assert/strict";
import test from "node:test";

import {
  integrityText,
  renderFingerprintSummary,
} from "../src/features/settings/fingerprint/fingerprint_summary.js";

const escape = (value) => String(value ?? "").replace(/[&<>"]/g, (ch) => (
  { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[ch]
));

test("the code, one hint line and a note are rendered", () => {
  const html = renderFingerprintSummary(
    { fingerprint: "b0d232ad", count: 165, integrity: { valid: true, checked: 73 } },
    { escape },
  );
  assert.match(html, /setting-fingerprint__code">b0d232ad/);
  assert.match(html, /setting-fingerprint__hint">설정이 같으면 코드도 같아요/);
  assert.match(html, /setting-fingerprint__note">설정 165개 · 기록 73건 · 이상 없음/);
});

test("a missing code shows a dash, not blank", () => {
  const html = renderFingerprintSummary({ count: 0 }, { escape });
  assert.match(html, /setting-fingerprint__code">-/);
});

test("intact integrity reports the checked count", () => {
  assert.equal(integrityText({ valid: true, checked: 12 }), "기록 12건 · 이상 없음");
});

test("broken integrity reports the 1-based position", () => {
  assert.equal(integrityText({ valid: false, broken_at: 4 }), "기록이 손상됨 (5)");
});

test("no integrity yields empty text", () => {
  assert.equal(integrityText(null), "");
});

test("the code is escaped", () => {
  const html = renderFingerprintSummary({ fingerprint: "<x>" }, { escape });
  assert.doesNotMatch(html, /<x>/);
  assert.match(html, /&lt;x&gt;/);
});

test("a changed state shows how many settings differ and offers re-baseline", () => {
  const html = renderFingerprintSummary({
    fingerprint: "new", count: 165, integrity: { valid: true, checked: 5 },
    baseline: { fingerprint: "old", ts: 1000 }, changed: true, changed_count: 3,
  }, { escape });
  assert.match(html, /setting-fingerprint__status--changed/);
  assert.match(html, /기준 이후 3개 바뀜/);
  assert.match(html, /data-setting-fingerprint-save/);
});

test("an unchanged state says so and offers no re-baseline", () => {
  const html = renderFingerprintSummary({
    fingerprint: "same", count: 165, baseline: { fingerprint: "same", ts: 1000 }, changed: false,
  }, { escape });
  assert.match(html, /setting-fingerprint__status--same/);
  assert.match(html, /기준과 같아요/);
  assert.doesNotMatch(html, /data-setting-fingerprint-save/);
});

test("without a baseline no status or save button is shown", () => {
  const html = renderFingerprintSummary({ fingerprint: "x", count: 1 }, { escape });
  assert.doesNotMatch(html, /setting-fingerprint__status/);
  assert.doesNotMatch(html, /data-setting-fingerprint-save/);
});

test("translation and count interpolation go through the text provider", () => {
  const html = renderFingerprintSummary(
    { fingerprint: "ab", count: 3, integrity: { valid: true, checked: 1 } },
    { escape, text: (key, fallback, vars) => (key === "setting_fingerprint_count" ? `${vars.n} settings` : fallback) },
  );
  assert.match(html, /3 settings/);
});
