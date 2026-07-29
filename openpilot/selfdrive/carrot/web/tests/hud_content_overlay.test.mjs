import assert from "node:assert/strict";
import test from "node:test";

import { createDriveVisionHudContent } from "../src/features/drive/contents/vision/hud_content.js";

function classList() {
  const values = new Set();
  return {
    add: (...names) => names.forEach((name) => values.add(name)),
    remove: (...names) => names.forEach((name) => values.delete(name)),
    contains: (name) => values.has(name),
    toggle(name, force) {
      const enabled = force === undefined ? !values.has(name) : Boolean(force);
      if (enabled) values.add(name);
      else values.delete(name);
      return enabled;
    },
  };
}

function element() {
  const attributes = new Map();
  return {
    classList: classList(),
    dataset: {},
    hidden: false,
    inert: false,
    setAttribute: (name, value) => attributes.set(name, String(value)),
    removeAttribute: (name) => attributes.delete(name),
  };
}

test("vision HUD content drives the overlay for every lifecycle step", () => {
  const root = element();
  const workspace = element();
  const stage = element();
  const overlayCalls = [];
  const overlay = {
    activate: () => overlayCalls.push("activate"),
    deactivate: () => overlayCalls.push("deactivate"),
    update: (payload) => overlayCalls.push(["update", payload]),
    relayout: () => overlayCalls.push("relayout"),
    setSuppressed: (reason, value) => overlayCalls.push(["suppress", reason, value]),
    status: () => ({ visible: true }),
    destroy: () => overlayCalls.push("destroy"),
  };
  const documentRoot = {
    documentElement: { clientWidth: 1280, clientHeight: 720 },
    getElementById(id) {
      return { driveHudCard: root, carrotDriveWorkspace: workspace, carrotStage: stage }[id] || null;
    },
  };
  const target = {
    document: documentRoot,
    CarrotHudOverlay: overlay,
    addEventListener() {},
  };

  const content = createDriveVisionHudContent({ target, document: documentRoot });
  assert.ok(content);
  content.activate();
  content.update({ evActive: true });
  content.setSuppressed("replay-insights", true);
  content.resize();
  content.deactivate();
  content.destroy();

  // Every step is expressed as an overlay call: there is no second renderer left
  // to fall back to.
  assert.deepEqual(overlayCalls, [
    "activate",
    "relayout",
    ["update", { evActive: true }],
    ["suppress", "replay-insights", true],
    "relayout",
    "deactivate",
    "destroy",
  ]);
});
