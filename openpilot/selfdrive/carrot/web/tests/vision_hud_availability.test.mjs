import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import vm from "node:vm";

const source = readFileSync(new URL("../js/realtime/app_realtime.js", import.meta.url), "utf8");
const start = source.indexOf("async function syncCarrotVisionAvailability() {");
const end = source.indexOf("window.CarrotVisionSyncAvailability", start);
assert.ok(start >= 0 && end > start);

for (const clusterHud of [0, 1]) {
  for (const enabled of [0, 1]) {
    test(`web video availability: HUD=${clusterHud}, vision=${enabled}`, async () => {
      let availability;
      const context = vm.createContext({
        fetchCarrotDeviceRuntimeState: async () => ({ clusterHud, carrotVisionEnabled: enabled, changed: true }),
        isCarrotRecordedReplayActive: () => false,
        isCarrotVisionTestActive: () => false,
        updateCarrotVisionAvailabilityUi: (available) => { availability = available; },
        syncCarrotRealtimeLifecycle: () => {},
        getUIText: (_key, fallback) => fallback,
      });
      vm.runInContext(source.slice(start, end), context);
      assert.equal(await context.syncCarrotVisionAvailability(), Boolean(enabled));
      assert.equal(availability, Boolean(enabled));
    });
  }
}
