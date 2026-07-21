import assert from "node:assert/strict";
import test from "node:test";

import { createLiveStateProvider } from "../src/features/drive/core/live_state_provider.js";

test("a compact service batch publishes one coherent live snapshot", () => {
  let now = 42;
  const activity = { subscribe() { return () => {}; }, isActive() { return true; } };
  const provider = createLiveStateProvider({
    target: {},
    nowMs: () => now,
    getHudState: () => ({}),
    getOverlayState: () => ({}),
    getLiveRuntimeState: () => ({ ok: true, services: {} }),
    getActivity: () => activity,
  });
  let publications = 0;
  provider.subscribe(() => { publications += 1; });

  now = 100;
  const snapshot = provider.noteServicesReceived(["carState", "modelV2", "carState"]);
  assert.equal(publications, 1);
  assert.equal(snapshot.receivedAtMonotonic.carState, 100);
  assert.equal(snapshot.receivedAtMonotonic.modelV2, 100);

  now = 120;
  provider.noteServiceReceived("radarState");
  assert.equal(publications, 2);
  assert.equal(provider.snapshot().receivedAtMonotonic.radarState, 120);
});
