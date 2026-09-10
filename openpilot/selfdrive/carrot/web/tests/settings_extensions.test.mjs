import assert from "node:assert/strict";
import test from "node:test";

import { createFeaturePanelLifecycle } from "../src/shared/feature_panels/lifecycle.js";
import { createSettingsExtensionRegistry } from "../src/features/settings/extensions/registry.js";

test("a feature-panel lifecycle only owns resources explicitly added by its adapter", () => {
  const lifecycle = createFeaturePanelLifecycle();
  let aborted = 0;
  let cleaned = 0;
  lifecycle.signal.addEventListener("abort", () => { aborted += 1; });
  lifecycle.addCleanup(() => { cleaned += 1; });

  assert.equal(lifecycle.destroyed, false);
  assert.equal(lifecycle.destroy(), true);
  assert.equal(lifecycle.destroy(), false);
  assert.equal(lifecycle.destroyed, true);
  assert.equal(aborted, 1);
  assert.equal(cleaned, 1);
});

test("settings extensions mount once for their active group and clean up when it changes", () => {
  const registry = createSettingsExtensionRegistry();
  const mountedRoot = { isConnected: true };
  let mounted = 0;
  let synced = 0;
  let destroyed = 0;

  registry.register({
    id: "test-panel",
    matches: ({ group }) => group === "STEER",
    mount: () => {
      mounted += 1;
      return {
        root: mountedRoot,
        sync: () => { synced += 1; },
        destroy: () => { destroyed += 1; },
      };
    },
  });

  registry.sync({ group: "STEER", root: {} });
  registry.sync({ group: "STEER", root: {} });
  assert.equal(mounted, 1);
  assert.equal(synced, 1);

  registry.sync({ group: "SPEED", root: {} });
  assert.equal(destroyed, 1);

  registry.sync({ group: "STEER", root: {} });
  mountedRoot.isConnected = false;
  registry.sync({ group: "STEER", root: {} });
  assert.equal(mounted, 3, "a detached rendered section mounts again after Settings redraws");
  assert.equal(destroyed, 2);
});

test("settings extensions preserve the selected detail name in their context", () => {
  const registry = createSettingsExtensionRegistry();
  let received = null;
  registry.register({
    id: "detail-context",
    matches(context) {
      received = context;
      return false;
    },
    mount: () => null,
  });

  registry.sync({ group: "STEER", detailMode: true, detailName: "ShareData", root: {} });
  assert.equal(received.detailMode, true);
  assert.equal(received.detailName, "ShareData");
});
