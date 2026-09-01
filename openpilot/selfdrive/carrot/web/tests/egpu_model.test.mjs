import assert from "node:assert/strict";
import test from "node:test";

import { modelDisplayName, modelDisplayTitle } from "../src/features/tools/egpu_model.js";

test("TGC model gets a friendly eGPU title", () => {
  const status = { model_id: "comma-pr38739-tgc-a2e422ee-1791d594" };
  assert.equal(modelDisplayName(status), "TGC");
  assert.equal(modelDisplayTitle(status, "eGPU big model"), "TGC · eGPU");
});

test("Time-to-Go model gets a friendly eGPU title", () => {
  const status = { model_id: "comma-pr38726-time-to-go-5a658611-39131097" };
  assert.equal(modelDisplayName(status), "Time to Go");
  assert.equal(modelDisplayTitle(status, "eGPU big model"), "Time to Go · eGPU");
});

test("explicit model display name takes priority", () => {
  const status = { display_name: "Experimental model", model_id: "comma-pr38726-time-to-go" };
  assert.equal(modelDisplayTitle(status, "eGPU big model"), "Experimental model · eGPU");
});

test("unknown models keep the localized generic title", () => {
  assert.equal(modelDisplayTitle({ model_id: "unknown-model" }, "eGPU 빅모델"), "eGPU 빅모델");
});
