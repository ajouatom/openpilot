import assert from "node:assert/strict";
import test from "node:test";

import { CarrotEgpuModel } from "../src/features/tools/egpu_model.js";

test("model update card shows download progress before compilation is available", (t) => {
  const elements = new Map();
  const previousDocument = Object.getOwnPropertyDescriptor(globalThis, "document");
  t.after(() => {
    if (previousDocument) Object.defineProperty(globalThis, "document", previousDocument);
    else delete globalThis.document;
  });
  globalThis.document = {
    getElementById(id) {
      if (!elements.has(id)) elements.set(id, { dataset: {}, style: {}, classList: { toggle() {} } });
      return elements.get(id);
    },
  };
  const status = {
    available: true, state: "downloading", model_id: "new-model", compiled: false, can_restart: false,
    downloaded_bytes: 50 * 1024 * 1024, total_bytes: 200 * 1024 * 1024, progress: 25,
  };
  CarrotEgpuModel.render(status);
  assert.equal(elements.get("egpuModelState").textContent, "Downloading");
  assert.equal(elements.get("egpuModelProgress").hidden, false);
  assert.equal(elements.get("egpuModelProgressBar").style.width, "25%");
  assert.equal(elements.get("egpuModelAmount").textContent, "50.0 MB / 200.0 MB · 25.0%");
  assert.equal(elements.get("btnEgpuCompileRestart").hidden, true);

  CarrotEgpuModel.render({ ...status, state: "verifying", progress: 100 });
  assert.equal(elements.get("egpuModelState").textContent, "Verifying download");
  assert.equal(elements.get("btnEgpuCompileRestart").hidden, true);

  CarrotEgpuModel.render({ ...status, state: "ready", progress: 100, can_restart: true });
  assert.equal(elements.get("egpuModelState").textContent, "Ready to compile");
  assert.equal(elements.get("btnEgpuCompileRestart").hidden, false);
  assert.equal(elements.get("btnEgpuCompileRestart").disabled, false);
});
