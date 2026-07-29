import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import test from "node:test";

import {
  appDialogChoiceGroups,
  appDialogChoiceLayout,
  inferAppDialogChoiceLayout,
} from "../src/ui/components/dialog/dialog.js";

const root = path.resolve(import.meta.dirname, "..");
const read = (relativePath) => fs.readFileSync(path.join(root, relativePath), "utf8");

test("an ungrouped choice list stays a single unlabelled section", () => {
  const groups = appDialogChoiceGroups([
    { label: "Play", value: "play" },
    { label: "Upload", value: "upload" },
  ]);

  assert.equal(groups.length, 1);
  assert.equal(groups[0].heading, "");
  assert.deepEqual(groups[0].items.map((item) => item.value), ["play", "upload"]);
});

test("a heading opens a section that owns the choices after it", () => {
  const groups = appDialogChoiceGroups([
    { heading: "정렬" },
    { label: "정렬: 오름차순", value: "sort:asc", selected: true },
    { label: "정렬: 내림차순", value: "sort:desc" },
    { heading: "최근 로그 전송" },
    { label: "최근 2개 전송", value: "upload_recent:2" },
  ]);

  assert.deepEqual(groups.map((group) => group.heading), ["정렬", "최근 로그 전송"]);
  assert.deepEqual(groups[0].items.map((item) => item.value), ["sort:asc", "sort:desc"]);
  assert.deepEqual(groups[1].items.map((item) => item.value), ["upload_recent:2"]);
});

test("choices before the first heading keep their own leading section", () => {
  const groups = appDialogChoiceGroups([
    { label: "Summary", value: "summary" },
    { heading: "Sort" },
    { label: "Ascending", value: "asc" },
  ]);

  assert.deepEqual(groups.map((group) => group.heading), ["", "Sort"]);
  assert.deepEqual(groups[0].items.map((item) => item.value), ["summary"]);
});

test("empty sections and unusable entries never reach the dialog", () => {
  const groups = appDialogChoiceGroups([
    { heading: "Empty" },
    { heading: "Real" },
    { label: "Only", value: "only" },
    null,
    { value: "no-label" },
    { heading: "   " },
  ]);

  assert.deepEqual(groups.map((group) => group.heading), ["Real"]);
  assert.deepEqual(groups[0].items.map((item) => item.value), ["only"]);
  assert.deepEqual(appDialogChoiceGroups(undefined), []);
});

test("headings force the stacked list even when values look grid-shaped", () => {
  const gridShaped = [
    { label: "1", value: 1 },
    { label: "2", value: 2 },
    { label: "3", value: 3 },
    { label: "4", value: 4 },
    { label: "5", value: 5 },
  ];

  // Without a heading these short values still earn the compact value grid.
  assert.equal(inferAppDialogChoiceLayout(gridShaped), "value-grid");
  assert.equal(appDialogChoiceLayout(appDialogChoiceGroups(gridShaped)), "value-grid");
  assert.equal(appDialogChoiceLayout(appDialogChoiceGroups([{ heading: "Step" }, ...gridShaped])), "list");
  assert.equal(appDialogChoiceLayout([]), "");
});

test("grouped sections are labelled and keep the flat list row rhythm", () => {
  const dialogJs = read("src/ui/components/dialog/dialog.js");
  const dialogCss = read("src/ui/components/dialog/style.css");

  assert.match(dialogJs, /section\.setAttribute\("role", "group"\)/);
  assert.match(dialogJs, /section\.setAttribute\("aria-labelledby", headingId\)/);
  // A heading must not be a button: the dialog focuses the first button in the
  // list, and a focusable heading would steal that focus.
  assert.match(dialogJs, /const heading = documentRoot\.createElement\("div"\)/);
  assert.match(dialogCss, /\.app-dialog__choices--list \.app-dialog__choiceGroup \{[^}]*gap: inherit;/s);
});

test("the logs menu is a grouped choice dialog instead of an inline dropdown", () => {
  const shell = read("index.html");
  const runtime = read("src/features/logs/runtime.js");
  const logsCss = read("src/features/logs/style.css");

  assert.match(shell, /id="logsMenuButton"[^>]*aria-haspopup="dialog"/s);
  assert.doesNotMatch(shell, /logsMenuPanel|data-logs-menu-action/);
  assert.doesNotMatch(logsCss, /\.logs-menu__(panel|group|label|item)/);

  assert.match(runtime, /mode: "choice"/);
  assert.match(runtime, /\{ heading: getUIText\("logs_sort", "Sort"\) \}/);
  assert.match(runtime, /\{ heading: getUIText\("recent_log_upload", "Upload recent logs"\) \}/);
  // Sort state and labels are read when the menu opens, not cached in markup.
  assert.match(runtime, /choices: logsMenuChoices\(\)/);
});
