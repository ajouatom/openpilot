import assert from "node:assert/strict";
import test from "node:test";

import {
  buildPopularDisplayEntry,
  comparePopularItems,
  isPopularValueInRange,
  normalizePopularNumericValue,
  popularPrimaryCount,
  popularSummaryValues,
} from "../src/features/settings/popular/popular_values.js";

const RANGE = { min: -120, max: 120 };
const BOOL = { min: 0, max: 1 };

test("numeric values normalize; blanks and junk become null", () => {
  assert.equal(normalizePopularNumericValue(RANGE, "5"), 5);
  assert.equal(normalizePopularNumericValue(RANGE, -1), -1);
  assert.equal(normalizePopularNumericValue(RANGE, ""), null);
  assert.equal(normalizePopularNumericValue(RANGE, null), null);
  assert.equal(normalizePopularNumericValue(RANGE, "abc"), null);
});

test("a 0..1 parameter accepts on/off spellings", () => {
  assert.equal(normalizePopularNumericValue(BOOL, "on"), 1);
  assert.equal(normalizePopularNumericValue(BOOL, "true"), 1);
  assert.equal(normalizePopularNumericValue(BOOL, "off"), 0);
  assert.equal(normalizePopularNumericValue(BOOL, "false"), 0);
});

test("range check rejects out-of-bounds and unparseable values", () => {
  assert.equal(isPopularValueInRange(RANGE, 0), true);
  assert.equal(isPopularValueInRange(RANGE, -120), true);
  assert.equal(isPopularValueInRange(RANGE, 121), false);
  assert.equal(isPopularValueInRange(RANGE, "nope"), false);
});

test("a parameter without numeric bounds accepts anything", () => {
  assert.equal(isPopularValueInRange({}, "whatever"), true);
});

test("higher count sorts first", () => {
  assert.ok(comparePopularItems(RANGE, { count: 2, value: 1 }, { count: 5, value: 2 }) > 0);
  assert.ok(comparePopularItems(RANGE, { count: 9, value: 1 }, { count: 1, value: 2 }) < 0);
});

test("equal counts break the tie by numeric value", () => {
  assert.ok(comparePopularItems(RANGE, { count: 3, value: 10 }, { count: 3, value: 20 }) < 0);
});

test("a numeric value sorts ahead of an unparseable one at equal count", () => {
  assert.ok(comparePopularItems(RANGE, { count: 3, value: 10 }, { count: 3, value: "x" }) < 0);
  assert.ok(comparePopularItems(RANGE, { count: 3, value: "x" }, { count: 3, value: 10 }) > 0);
});

test("original index is the final, stable tiebreaker", () => {
  const a = { count: 3, value: "x" };
  const b = { count: 3, value: "x" };
  assert.ok(comparePopularItems(RANGE, a, b, { aIndex: 0, bIndex: 1 }) < 0);
});

test("the display entry drops out-of-range and non-positive candidates and caps at ten", () => {
  const entry = {
    sample: 40,
    value: 0,
    top_values: [
      { value: 0, count: 10 },
      { value: 999, count: 8 },   // out of range
      { value: 5, count: 0 },     // non-positive count
      ...Array.from({ length: 15 }, (_, i) => ({ value: i + 1, count: 2 })),
    ],
  };
  const built = buildPopularDisplayEntry(RANGE, entry);
  assert.equal(built.top_values.length, 10);
  assert.equal(built.top_values.every((item) => item.count > 0), true);
  assert.equal(built.top_values.some((item) => item.value === 999), false);
  assert.equal(built.top_values[0].value, 0, "the count-10 value leads");
});

test("no sample means no entry", () => {
  assert.equal(buildPopularDisplayEntry(RANGE, { sample: 0, value: 0, top_values: [] }), null);
  assert.equal(buildPopularDisplayEntry(RANGE, null), null);
});

test("an out-of-range headline value suppresses the entry", () => {
  assert.equal(buildPopularDisplayEntry(RANGE, { sample: 10, value: 999, top_values: [] }), null);
});

test("primary count reads the top value, then falls back", () => {
  assert.equal(popularPrimaryCount({ top_values: [{ count: 7 }] }), 7);
  assert.equal(popularPrimaryCount({ top_count: 4 }), 4);
  assert.equal(popularPrimaryCount({ count: 3 }), 3);
  assert.equal(popularPrimaryCount({}), 0);
});

test("summary shows a clear winner or a clean two-way tie only", () => {
  assert.deepEqual(
    popularSummaryValues({ top_values: [{ value: 1, count: 5 }, { value: 2, count: 3 }] }).map((v) => v.value),
    [1],
  );
  assert.deepEqual(
    popularSummaryValues({ top_values: [{ value: 1, count: 5 }, { value: 2, count: 5 }] }).map((v) => v.value),
    [1, 2],
  );
  // A three-way tie is ambiguous, so nothing is summarized.
  assert.deepEqual(
    popularSummaryValues({ top_values: [{ value: 1, count: 5 }, { value: 2, count: 5 }, { value: 3, count: 5 }] }),
    [],
  );
});

test("a lone top count below two is not summarized", () => {
  assert.deepEqual(popularSummaryValues({ top_values: [{ value: 1, count: 1 }] }), []);
  assert.deepEqual(popularSummaryValues({ top_values: [] }), []);
});

test("format and locale are injected into ordering", () => {
  const calls = [];
  comparePopularItems(RANGE, { count: 3, value: "b" }, { count: 3, value: "a" }, {
    formatValue: (value) => { calls.push(value); return String(value); },
    locale: "ko-KR",
  });
  assert.deepEqual(calls, ["b", "a"]);
});
