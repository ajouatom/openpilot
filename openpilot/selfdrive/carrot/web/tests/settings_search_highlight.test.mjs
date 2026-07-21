import assert from "node:assert/strict";
import test from "node:test";

import { highlightSearchText } from "../src/features/settings/search/highlight.js";

const escape = (value) => String(value ?? "").replace(/[&<>"]/g, (ch) => (
  { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[ch]
));

const mark = (text, query) => highlightSearchText(text, query, { escape });

test("the matched span is wrapped in a mark", () => {
  assert.equal(
    mark("Cruise Speed", "speed"),
    'Cruise <mark class="setting-search-result__mark">Speed</mark>',
  );
});

test("matching is case-insensitive but preserves the original casing", () => {
  assert.match(mark("ApplyModelSpeed", "model"), /<mark[^>]*>Model<\/mark>/);
});

test("only the first occurrence is highlighted", () => {
  const html = mark("speed speed", "speed");
  assert.equal(html.match(/<mark/g).length, 1);
});

test("an empty query returns the escaped text with no mark", () => {
  assert.equal(mark("A & B", ""), "A &amp; B");
  assert.equal(mark("A & B", "   "), "A &amp; B");
});

test("no match returns the escaped text with no mark", () => {
  assert.equal(mark("Cruise", "zzz"), "Cruise");
  assert.doesNotMatch(mark("Cruise", "zzz"), /<mark/);
});

// The reason the escape matters: a label with markup must not inject it.
test("every segment is escaped, including inside the mark", () => {
  const html = mark("<b>x</b> speed <i>y</i>", "speed");
  assert.doesNotMatch(html, /<b>|<i>/);
  assert.match(html, /&lt;b&gt;x&lt;\/b&gt;/);
  assert.match(html, /&lt;i&gt;y&lt;\/i&gt;/);
});

test("a query that is itself markup is escaped inside the mark", () => {
  const html = mark("value <script> here", "<script>");
  assert.match(html, /<mark[^>]*>&lt;script&gt;<\/mark>/);
  assert.doesNotMatch(html, /<script>/);
});

test("nullish text and query are handled", () => {
  assert.equal(mark(null, "x"), "");
  assert.equal(mark(undefined, "x"), "");
  assert.equal(highlightSearchText("text", null, { escape }), "text");
});

test("the mark class can be overridden", () => {
  const html = highlightSearchText("abc", "b", { escape, markClass: "hl" });
  assert.match(html, /<mark class="hl">b<\/mark>/);
});

test("without an escape function the text passes through unescaped", () => {
  // Defensive default; callers in the app always inject escape.
  assert.equal(highlightSearchText("a<b", "x"), "a<b");
});
