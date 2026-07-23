import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

const dashcam = await readFile(new URL("../src/features/logs/dashcam.js", import.meta.url), "utf8");
const runtime = await readFile(new URL("../src/features/logs/runtime.js", import.meta.url), "utf8");
const style = await readFile(new URL("../src/features/logs/style.css", import.meta.url), "utf8");
const ko = await readFile(new URL("../js/translations/ko.js", import.meta.url), "utf8");
const en = await readFile(new URL("../js/translations/en.js", import.meta.url), "utf8");

test("route summary is exposed only through the route overflow menu", () => {
  assert.doesNotMatch(dashcam, /dashcam-report-btn|data-action="route-report"/);
  assert.doesNotMatch(runtime, /showDashcamRouteReport|action === "route-report"/);
  assert.match(dashcam, /import \{ openRouteSummary \} from "\.\/route_summary\/index\.js"/);
  assert.match(dashcam, /getUIText\("route_summary", "주행요약"\), value: "summary"/);
  assert.match(dashcam, /selected === "summary"\) await openRouteSummary\(route\)/);
});

test("portrait and compact layouts share one route menu in the selection action row", () => {
  assert.doesNotMatch(dashcam, /dashcam-group-menu-btn--head/);
  assert.match(dashcam, /dashcam-group-menu-btn dashcam-group-menu-btn--row[^>]+data-action="route-menu"/);
  assert.match(style, /\.dashcam-group-menu-btn--row\s*\{[^}]*display:\s*inline-flex;[^}]*margin-left:\s*auto;/s);
  assert.match(style, /\.dashcam-selection-row\s*\{[^}]*flex-direction:\s*column;[^}]*justify-content:\s*center;[^}]*align-items:\s*center;/s);
  assert.match(style, /\.dashcam-route-main\s*\{[^}]*padding-right:\s*0;/s);
  assert.match(style, /\.dashcam-selection-row\s*\{[^}]*padding:\s*8px;[^}]*scrollbar-width:\s*none;/s);
  assert.match(style, /\.dashcam-selection-row \.dashcam-group-menu-btn--row\s*\{[^}]*margin-left:\s*0;/s);
});

test("route summary labels contain no report icon or legacy title", () => {
  assert.match(ko, /route_summary:\s*"주행요약"/);
  assert.match(en, /route_summary:\s*"Drive summary"/);
  assert.doesNotMatch(`${dashcam}\n${ko}\n${en}`, /route_report|주행 리포트|Drive report/);
});
