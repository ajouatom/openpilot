import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import test from "node:test";

const root = path.resolve(import.meta.dirname, "..");
const read = (relativePath) => fs.readFileSync(path.join(root, relativePath), "utf8");

const shell = read("index.html");
const logsCss = read("src/features/logs/style.css");
const segmentedCss = read("src/ui/components/segmented_control/style.css");

const rule = (css, selector) => {
  const start = css.indexOf(`${selector} {`);
  assert.notEqual(start, -1, `missing rule for ${selector}`);
  return css.slice(start, css.indexOf("}", start));
};

test("logs tabs render as an equal-width shared segmented control", () => {
  const tabList = shell.slice(shell.indexOf("<div id=\"logsTabs\""), shell.indexOf("id=\"logsMenu\""));

  assert.match(tabList, /class="logs-tabs c-segmented-control c-segmented-control--equal"/);
  assert.match(tabList, /role="tablist"/);

  const tabs = tabList.match(/data-logs-tab="([a-z]+)"/g) || [];
  assert.deepEqual(tabs, ["data-logs-tab=\"dashcam\"", "data-logs-tab=\"screen\""]);

  for (const [tab, panel] of [["dashcam", "logsDashcamPanel"], ["screen", "logsScreenPanel"]]) {
    assert.match(tabList, new RegExp(`aria-controls="${panel}"[^>]*data-logs-tab="${tab}"`, "s"));
    assert.match(shell, new RegExp(`id="${panel}"[^>]*role="tabpanel"`));
  }

  // Every label is its own element so a longer translation is bounded by the
  // segment instead of resizing it.
  assert.equal((tabList.match(/class="c-segmented-control__label"/g) || []).length, 2);
});

test("equal segments share the track independently of label length", () => {
  assert.match(rule(segmentedCss, ".c-segmented-control--equal"), /--segmented-control-item-flex: 1 1 0;/);
  assert.match(rule(segmentedCss, ".c-segmented-control--equal > .c-segmented-control__item"), /min-inline-size: 0;/);
  assert.match(rule(segmentedCss, ".c-segmented-control__label"), /text-overflow: ellipsis;/);
});

test("the logs tab track never borrows the route media column width", () => {
  // Sharing that variable made the tabs follow the route thumbnail column and
  // stop being equal on wide and short-landscape breakpoints.
  assert.doesNotMatch(rule(logsCss, ".logs-tabs"), /--logs-route-media-column/);
  assert.doesNotMatch(rule(logsCss, ".logs-tabs"), /grid-template-columns/);
  assert.match(logsCss, /\.dashcam-route-card \{[^}]*--logs-route-media-column/s);
});

test("logs tab geometry and typography stay tokenized", () => {
  const tabs = rule(logsCss, ".logs-tabs");

  assert.match(tabs, /--segmented-control-gap: var\(--logs-tab-inset\);/);
  assert.match(tabs, /--segmented-control-padding: var\(--logs-tab-inset\);/);
  assert.match(tabs, /--segmented-control-item-min-height: var\(--logs-tab-min-height\);/);
  assert.match(tabs, /--segmented-control-item-font-size: var\(--logs-tab-font-size\);/);
  assert.match(tabs, /--segmented-control-item-font-weight: var\(--logs-tab-font-weight\);/);
  // Segment corners stay concentric with the track instead of a fixed radius.
  assert.match(tabs, /--segmented-control-item-radius: calc\(var\(--logs-tab-track-radius\) - var\(--logs-tab-inset\)\);/);

  for (const token of [
    "--logs-tab-track-radius",
    "--logs-tab-inset",
    "--logs-tab-min-height",
    "--logs-tab-font-size",
    "--logs-tab-font-weight",
  ]) {
    assert.match(rule(logsCss, ".logs-shell"), new RegExp(`${token}:`));
  }
});
