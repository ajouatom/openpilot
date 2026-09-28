import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import vm from "node:vm";

const api = await readFile(new URL("../js/shared/api.js", import.meta.url), "utf8");
const runtime = await readFile(new URL("../js/realtime/app_realtime.js", import.meta.url), "utf8");

test("experimental DM requires acknowledgement before a settings write", async () => {
  const writes = [];
  let accepted = false;
  let prompts = 0;
  const context = vm.createContext({
    window: { CarrotParamCommit: { create: () => ({ commit: async (...args) => writes.push(args) }) } },
    appConfirm: async () => { prompts++; return accepted; },
    getUIText: (_key, fallback) => fallback,
  });
  vm.runInContext(api, context);
  await assert.rejects(context.setParam("DriverMonitoringMode", 1));
  assert.equal(writes.length, 0);
  accepted = true;
  await context.setParam("DriverMonitoringMode", "1");
  assert.equal(writes.length, 1);
  assert.equal(prompts, 2);
  await context.setParam("DriverMonitoringMode", 0);
  await context.setParam("CarrotVisionEnabled", 1);
  assert.equal(prompts, 2);
  assert.equal(writes.length, 3);
});

test("Carrot Vision accepts typed booleans from Params and raw legacy values", () => {
  const start = runtime.indexOf("function normalizeRuntimeBool(");
  const end = runtime.indexOf("let _carrotVisionEnvironmentSignature", start);
  const context = vm.createContext({ CARROT_DEVICE_RUNTIME_STATE: {}, window: {} });
  vm.runInContext(runtime.slice(start, end), context);
  for (const value of [true, 1, "1", "true"]) {
    context.updateCarrotDeviceRuntimeState({ carrotVisionEnabled: value });
    assert.equal(context.CARROT_DEVICE_RUNTIME_STATE.carrotVisionEnabled, 1);
  }
  for (const value of [false, 0, "0", "false"]) {
    context.updateCarrotDeviceRuntimeState({ carrotVisionEnabled: value });
    assert.equal(context.CARROT_DEVICE_RUNTIME_STATE.carrotVisionEnabled, 0);
  }
});

function visionContext(values) {
  const segments = [
    ["function normalizeRuntimeBool(", "let _carrotVisionEnvironmentSignature"],
    ["async function fetchCarrotDeviceRuntimeState(", "function updateCarrotVisionAvailabilityUi("],
    ["async function syncCarrotVisionAvailability(", "window.CarrotVisionSyncAvailability"],
  ].map(([start, end]) => runtime.slice(runtime.indexOf(start), runtime.indexOf(end, runtime.indexOf(start))));
  const context = vm.createContext({
    CARROT_DEVICE_RUNTIME_STATE: {}, window: {},
    fetch: async (url) => ({ ok: true, json: async () => ({ ok: true, values: Object.fromEntries(
      new URL(url, "http://localhost").searchParams.get("names").split(",").map(name => [name, values[name]])
    ) }) }),
    syncCarrotVisionEnvironmentState() {}, updateCarrotVisionAvailabilityUi() {}, syncCarrotRealtimeLifecycle() {},
    isCarrotRecordedReplayActive: () => false, isCarrotVisionTestActive: () => false,
    getUIText: (_key, fallback) => fallback,
  });
  vm.runInContext(segments.join("\n"), context);
  return context;
}

test("Carrot Vision supports every applied DM, vision and cluster combination", async () => {
  for (const dm of [0, 1, 2]) {
    for (const vision of [false, true]) {
      for (const cluster of [0, 1]) {
        const values = { DisableDM: 2 - dm, DisableDMActive: String(dm), CarrotVisionEnabled: vision,
          ClusterHud: cluster, IsOffroad: false, IsOnroad: true };
        const context = visionContext(values);
        assert.equal(await context.syncCarrotVisionAvailability(), cluster === 0 && (vision || dm === 2),
          JSON.stringify(values));
      }
    }
  }
});

test("saving DisableDM does not change video until manager applies it", async () => {
  for (const initial of [0, 1, 2]) {
    for (const saved of [0, 1, 2]) {
      const values = { DisableDM: initial, DisableDMActive: initial, CarrotVisionEnabled: false, ClusterHud: 0 };
      const context = visionContext(values);
      assert.equal(await context.syncCarrotVisionAvailability(), initial === 2);
      values.DisableDM = saved;
      assert.equal(await context.syncCarrotVisionAvailability(), initial === 2);
      values.DisableDMActive = saved;
      assert.equal(await context.syncCarrotVisionAvailability(), saved === 2);
    }
  }
});
