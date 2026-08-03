import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

globalThis.window = {};

const { WEB_SETTINGS_GROUPS } = await import("../src/features/tools/web_settings/schema.js");
const { selectCurrentGuidanceImageName } = await import("../src/features/drive/contents/carrot_navi/overlay.js");
const appRealtimeSource = await readFile(
  new URL("../js/realtime/app_realtime.js", import.meta.url),
  "utf8",
);

function navigationItems() {
  return WEB_SETTINGS_GROUPS.find((group) => group.id === "navigation")?.items || [];
}

test("Carrot Navi settings expose tap fullscreen and every supported map theme", () => {
  const items = navigationItems();
  assert.ok(items.some((item) => item.id === "carrot_navi_fullscreen_on_tap"));

  const theme = items
    .find((item) => item.id === "carrot_navi_map_appearance")
    ?.fields.find((field) => field.paramName === "ClusterNaviMapTheme");
  assert.deepEqual(theme?.options.map((option) => option.value), ["0", "1", "2"]);
});

test("current maneuver prefers the full native card and uses compact only as fallback", () => {
  const selectFrom = (...names) => selectCurrentGuidanceImageName((name) => names.includes(name));
  assert.equal(selectFrom("tbt_current_compact", "tbt_current_full"), "tbt_current_full");
  assert.equal(selectFrom("tbt_current_compact"), "tbt_current_compact");
  assert.equal(selectFrom(), "");
});

test("Carrot Navi fullscreen remains user-gesture driven and follows workspace lifecycle", () => {
  assert.match(appRealtimeSource, /getElementById\("carrotNaviPane"\)/);
  assert.match(appRealtimeSource, /pane\.addEventListener\("click"/);
  assert.match(appRealtimeSource, /carrot_navi_fullscreen_on_tap/);
  assert.match(appRealtimeSource, /requestCarrotFullscreen\(\{ quiet: false \}\)/);
  assert.match(appRealtimeSource, /drive:workspacelayoutchange/);
  assert.match(appRealtimeSource, /drive:workspacecontentchange/);
});
