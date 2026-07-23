import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import test from "node:test";

import {
  normalizeLogsPlayerGroup,
  normalizeLogsPlayerSegments,
} from "../src/features/logs/player/components.js";

const root = path.resolve(import.meta.dirname, "..");
const read = (relativePath) => fs.readFileSync(path.join(root, relativePath), "utf8");

test("player group metadata is normalized at the component boundary", () => {
  assert.deepEqual(normalizeLogsPlayerGroup({
    route: " route-id ",
    title: " Drive ",
    dateLabel: " 2026.07.23 ",
    timeRange: " 08:08–08:19 ",
    segmentCount: 11.8,
  }), {
    route: "route-id",
    title: "Drive",
    dateLabel: "2026.07.23",
    timeRange: "08:08–08:19",
    segmentCount: 11,
  });
  assert.equal(normalizeLogsPlayerGroup(null), null);
});

test("player segment metadata is de-duplicated and normalized before rendering", () => {
  assert.deepEqual(normalizeLogsPlayerSegments([
    {
      id: " route--1 ",
      name: " Segment 1 ",
      timeLabel: " 08:00–08:01 ",
      title: " Route · Segment 1 ",
      subtitle: " 2026.07.23 08:00–08:01 ",
      src: " /video/1 ",
      thumbnailSrc: " /thumbnail/1 ",
    },
    { id: "route--1", name: "duplicate" },
    null,
  ]), [{
    id: "route--1",
    name: "Segment 1",
    timeLabel: "08:00–08:01",
    title: "Route · Segment 1",
    subtitle: "2026.07.23 08:00–08:01",
    src: "/video/1",
    thumbnailSrc: "/thumbnail/1",
  }]);
});

test("dashcam player uses a modular adaptive dialog while screen recordings stay single-pane", () => {
  const components = read("src/features/logs/player/components.js");
  const dashcam = read("src/features/logs/dashcam.js");
  const runtime = read("src/features/logs/runtime.js");
  const entry = read("src/entries/logs.css");
  const style = read("src/features/logs/player/style.css");
  const legacyStyle = read("src/features/logs/style.css");
  const tokens = read("src/features/logs/player/tokens.css");

  assert.match(runtime, /createLogsPlayerDialog/);
  assert.match(runtime, /createLogsPlayerSegmentList/);
  assert.match(runtime, /statusFor:\s*\(segmentId\)\s*=>\s*segmentSession\.statusFor\(segmentId\)/);
  assert.match(runtime, /segment_reading/);
  assert.match(runtime, /segment_recently_read/);
  assert.match(runtime, /createDashcamPlayerSession/);
  assert.match(runtime, /createLogsPlayerDialog\(\{\s*currentGroup:\s*options\.currentGroup/);
  assert.match(dashcam, /currentGroup:\s*dashcamPlayerGroup\(entry,\s*route\)/);
  assert.match(dashcam, /segments:\s*dashcamPlayerSegments\(entry,\s*route,\s*orderedSegments\)/);
  assert.match(dashcam, /activeSegment:\s*segment/);
  assert.match(dashcam, /previousSegment:\s*readState\.previousSegment/);
  assert.match(dashcam, /selectDashcamReadSegment\(target\)/);
  assert.match(dashcam, /onClose:\s*\(\)\s*=>\s*\{\s*finishDashcamReadSegment\(\)/);
  assert.match(dashcam, /postJson\("\/api\/dashcam\/read-state"/);
  assert.match(dashcam, /getJson\("\/api\/dashcam\/read-state"\)/);
  assert.match(runtime, /loadDashcamReadState\(\)/);
  assert.match(dashcam, /dashcam-segment-read-status/);
  assert.match(dashcam, /createLogsSegmentStatusTag/);
  assert.match(runtime, /player\.source\s*=\s*\{[\s\S]*?currentMedia\.src/);
  assert.match(runtime, /updateSegments/);
  assert.doesNotMatch(dashcam, /onPrevious:[\s\S]*?close\?\.\(\)[\s\S]*?openDashcamPlayer/);
  assert.doesNotMatch(dashcam, /onNext:[\s\S]*?close\?\.\(\)[\s\S]*?openDashcamPlayer/);
  assert.match(style, /\.dashcam-player-dialog--browsable/);
  assert.match(style, /container-name:\s*logs-player-dialog/);
  assert.match(style, /@container logs-player-dialog/);
  assert.match(style, /aspect-ratio:\s*var\(--logs-player-dialog-landscape-aspect\)/);
  assert.match(style, /@media \(orientation:\s*portrait\)/);
  assert.match(style, /@media \(orientation:\s*landscape\) and \(max-height:\s*30rem\)/);
  assert.doesNotMatch(style, /@media \(max-aspect-ratio:\s*13\/10\)/);
  assert.match(style, /var\(--logs-player-browser-current-width\)/);
  assert.match(tokens, /--logs-player-dialog-wide:\s*88rem/);
  assert.match(tokens, /--logs-player-browser-pane-width:\s*clamp/);
  assert.match(legacyStyle, /\.dashcam-player-dialog:not\(\.dashcam-player-dialog--browsable\)/);
  assert.match(style, /\.dashcam-player-browser/);
  assert.match(style, /\.dashcam-player-group-summary/);
  assert.match(style, /\.dashcam-player-segment-list/);
  assert.match(style, /\.dashcam-player-segment-item\.is-active/);
  assert.match(style, /\.dashcam-player-segment-status/);
  assert.match(entry, /features\/logs\/player\/tokens\.css/);
  assert.match(entry, /features\/logs\/player\/style\.css/);
  assert.doesNotMatch(components, /innerHTML/);
  assert.match(components, /textContent/);
  assert.match(components, /dashcamPlayerCurrentGroupTitle/);
  assert.doesNotMatch(components, /Math\.random/);
});

test("reading and recently viewed use one localized status-tag component with state variants", () => {
  const components = read("src/features/logs/player/components.js");
  const style = read("src/features/logs/player/style.css");
  const legacyStyle = read("src/features/logs/style.css");
  const tokens = read("src/features/logs/player/tokens.css");

  assert.match(components, /createLogsSegmentStatusTag/);
  assert.match(components, /statusFor\?\.\(button\.dataset\.segmentId\)/);
  assert.match(components, /setStatusFor/);
  assert.match(components, /"dashcam-player-status-tag"/);
  assert.match(components, /tag\.dataset\.state\s*=\s*normalizedState/);
  assert.match(style, /\.dashcam-player-status-tag\[data-state="reading"\]/);
  assert.match(style, /\.dashcam-player-status-tag\[data-state="recent"\]/);
  assert.match(tokens, /\.dashcam-player-dialog,\s*\.dashcam-segment-read-status/);
  assert.match(legacyStyle, /\.dashcam-segment-tile--compact \.dashcam-segment-read-status/);

  for (const locale of ["ko", "en", "zh"]) {
    const translation = read(`js/translations/${locale}.js`);
    assert.match(translation, /current_group:/);
    assert.match(translation, /segment_browser:/);
    assert.match(translation, /segment_reading:/);
    assert.match(translation, /segment_recently_read:/);
  }
});
