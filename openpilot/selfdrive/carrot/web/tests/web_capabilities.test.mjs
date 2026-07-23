import assert from "node:assert/strict";
import test from "node:test";

import {
  findWebCapabilitySpec,
  getWebSettingCapabilitySpec,
  readWebCapabilityEnabled,
  readWebSettingUnlocked,
  requiredWebCapability,
} from "../src/shared/web/capabilities.js";

function target(enabled) {
  return {
    CarrotWebCapabilitiesSpec: [{
      id: "web_lab",
      settingKey: "web_lab_enabled",
      labelKey: "web_lab",
      lockedLabelKey: "web_lab_locked",
    }],
    CarrotWebSettingsSpec: [
      { key: "ordinary", type: "bool", default: false },
      {
        key: "vision_ar_enabled",
        type: "bool",
        default: false,
        requiresCapability: "web_lab",
      },
    ],
    CarrotWebCapabilitiesState: { web_lab: enabled },
  };
}

test("web capabilities expose one reusable setting lock contract", () => {
  const locked = target(false);
  assert.equal(requiredWebCapability("ordinary", locked), null);
  assert.equal(requiredWebCapability("vision_ar_enabled", locked), "web_lab");
  assert.equal(readWebCapabilityEnabled("web_lab", locked), false);
  assert.equal(readWebSettingUnlocked("ordinary", locked), true);
  assert.equal(readWebSettingUnlocked("vision_ar_enabled", locked), false);
  assert.equal(getWebSettingCapabilitySpec("vision_ar_enabled", locked)?.lockedLabelKey, "web_lab_locked");

  locked.CarrotWebCapabilitiesState.web_lab = true;
  assert.equal(readWebSettingUnlocked("vision_ar_enabled", locked), true);
});

test("capability state falls back to its declared setting source", () => {
  const fallback = target(false);
  delete fallback.CarrotWebCapabilitiesState;
  fallback.CarrotWebSettingsState = { web_lab_enabled: true };
  assert.equal(readWebCapabilityEnabled("web_lab", fallback), true);
  assert.equal(findWebCapabilitySpec("missing", fallback), null);
});
