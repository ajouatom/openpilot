import assert from "node:assert/strict";
import test from "node:test";

globalThis.window = globalThis;

const { WEB_SETTINGS_GROUPS } = await import("../src/features/tools/web_settings/schema.js");

test("auto update reboot exposes off, Park, and one-second disengage choices", () => {
  const generalItems = WEB_SETTINGS_GROUPS.find((group) => group.id === "general")?.items || [];
  const reboot = generalItems.find((item) => item.id === "auto_update_reboot");

  assert.ok(reboot);
  assert.deepEqual(reboot.options.map((option) => option.value), ["off", "park", "disengaged"]);
  assert.equal(reboot.options[1].labelKey, "web_auto_update_reboot_park");
  assert.equal(reboot.options[2].labelKey, "web_auto_update_reboot_disengaged");
});
