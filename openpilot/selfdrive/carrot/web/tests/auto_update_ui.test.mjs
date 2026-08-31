import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

globalThis.window = globalThis;

const { WEB_SETTINGS_GROUPS } = await import("../src/features/tools/web_settings/schema.js");
const autoUpdateControllerSource = await readFile(
  new URL("../src/features/tools/web_settings/controller.js", import.meta.url),
  "utf8",
);
const toolsRuntimeSource = await readFile(
  new URL("../src/features/tools/runtime.js", import.meta.url),
  "utf8",
);

test("auto update reboot exposes off, Park, and one-second disengage or offroad choices", () => {
  const generalItems = WEB_SETTINGS_GROUPS.find((group) => group.id === "general")?.items || [];
  const reboot = generalItems.find((item) => item.id === "auto_update_reboot");

  assert.ok(reboot);
  assert.deepEqual(reboot.options.map((option) => option.value), ["off", "park", "disengaged"]);
  assert.equal(reboot.options[1].labelKey, "web_auto_update_reboot_park");
  assert.equal(reboot.options[2].labelKey, "web_auto_update_reboot_disengaged_or_offroad");
});

test("persistent automatic update failures surface once in Web and override stale update counts", () => {
  assert.match(autoUpdateControllerSource, /\["error", "reboot_blocked"\]/);
  assert.match(autoUpdateControllerSource, /lastAutoUpdateErrorEventId/);
  assert.match(autoUpdateControllerSource, /showAppToast/);
  assert.match(toolsRuntimeSource, /status\.auto_update/);
  assert.match(toolsRuntimeSource, /const hasUpdates = behind > 0 && !hasError/);
});
