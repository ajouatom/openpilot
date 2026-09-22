import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";

const root = new URL("../", import.meta.url);
const source = readFileSync(new URL("src/features/tools/web_settings/drive_layout.js", root), "utf8");
const styles = readFileSync(new URL("src/features/tools/styles/drive_layout.css", root), "utf8");

test("the layout section renders a defaults button on the orientation row", () => {
  assert.match(source, /data-drive-layout-reset/);
  assert.match(source, /webSettingsText\("default_value"\)/);
  assert.match(source, /webSettingsText\("web_drive_layout_reset_hint"\)/);
  assert.match(styles, /\.drive-layout-header-actions\b/);
  assert.match(styles, /\.drive-layout-reset\b/);
});

test("the defaults button restores Area 1 full screen with Carrot Vision", () => {
  // The reset block is everything from its bind selector up to the component
  // registration. Both orientations must end at 영역 1 전체 with Area 1 =
  // Carrot Vision and Area 2 = Carrot Navi; mode and contents use the spec's
  // own constants so a stale page cannot leave them as-is, and the split
  // ratio follows the injected spec default.
  const start = source.indexOf('"[data-drive-layout-reset]"');
  const resetBlock = source.slice(start, source.indexOf("componentRegistry.register", start));
  assert.match(resetBlock, /keys\.mode/);
  assert.match(resetBlock, /spec\.MODE\.AREA_1/);
  assert.match(resetBlock, /keys\.area1Content/);
  assert.match(resetBlock, /spec\.CONTENT\.VISION/);
  assert.match(resetBlock, /keys\.area2Content/);
  assert.match(resetBlock, /spec\.CONTENT\.NAVIGATION/);
  assert.match(resetBlock, /keys\.ratio/);
  assert.match(resetBlock, /CarrotWebSettingDefaults/);
  assert.match(resetBlock, /setWebSettingsByKeys\(values\)/);
});

test("the reset hint is translated in every bundled language", () => {
  for (const language of ["ko", "en", "zh"]) {
    const pack = readFileSync(new URL(`js/translations/${language}.js`, root), "utf8");
    assert.match(pack, /web_drive_layout_reset_hint:/, `${language} is missing the reset hint`);
  }
});
