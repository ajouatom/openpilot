import assert from "node:assert/strict";
import test from "node:test";

import {
  loadOnnxVisionConfig,
  loadOnnxVisionSnapshot,
  loadOnnxVisionStatus,
  restoreOnnxVisionConfig,
  saveOnnxVisionConfig,
  updateOnnxVisionSettings,
} from "../src/features/settings/extensions/onnx_vision/api.js";

function jsonResponse(payload = {}, { ok = true, status = 200, statusText = "OK" } = {}) {
  return {
    ok,
    status,
    statusText,
    headers: { get: () => "application/json" },
    async json() { return payload; },
  };
}

test("ONNX API keeps status, config, and settings on the Carrot Web origin", async () => {
  const calls = [];
  const target = {
    async fetch(url, options) {
      calls.push([url, options]);
      return jsonResponse({ ok: true });
    },
  };

  await loadOnnxVisionStatus(target);
  await loadOnnxVisionConfig(target);
  await saveOnnxVisionConfig({ poly_left: [[1, 2]] }, target);
  await restoreOnnxVisionConfig(target);
  await updateOnnxVisionSettings({ bsd_threshold: 0.5 }, target);

  assert.deepEqual(calls.map(([url, options]) => [url, options.method]), [
    ["/xiaoge/api/status", "GET"],
    ["/xiaoge/api/config", "GET"],
    ["/xiaoge/api/config", "POST"],
    ["/xiaoge/api/config", "DELETE"],
    ["/xiaoge/api/settings", "POST"],
  ]);
  assert.equal(calls[2][1].headers["Content-Type"], "application/json");
  assert.deepEqual(JSON.parse(calls[2][1].body), { poly_left: [[1, 2]] });
});

test("ONNX API preserves structured service errors", async () => {
  const target = {
    async fetch() {
      return jsonResponse(
        { error: "service disabled", code: "XIAOGE_SERVICE_UNAVAILABLE" },
        { ok: false, status: 503, statusText: "Unavailable" },
      );
    },
  };

  await assert.rejects(
    () => loadOnnxVisionStatus(target),
    (error) => error.message === "service disabled"
      && error.code === "XIAOGE_SERVICE_UNAVAILABLE"
      && error.status === 503,
  );
});

test("ONNX snapshot requests exact streams and accepts only JPEG", async () => {
  const calls = [];
  const image = new Blob([new Uint8Array([0xff, 0xd8, 0xff])], { type: "image/jpeg" });
  const target = {
    async fetch(url, options) {
      calls.push([url, options]);
      return {
        ok: true,
        headers: { get: () => "image/jpeg" },
        async json() { return {}; },
        async blob() { return image; },
      };
    },
  };

  assert.equal(await loadOnnxVisionSnapshot("road", target), image);
  assert.equal(await loadOnnxVisionSnapshot("wide", target), image);
  assert.deepEqual(calls.map(([url]) => url), [
    "/xiaoge/api/snapshot?stream=road",
    "/xiaoge/api/snapshot?stream=wide",
  ]);

  const invalidTarget = {
    async fetch() {
      return {
        ok: true,
        headers: { get: () => "text/html" },
        async blob() { return new Blob([]); },
      };
    },
  };
  await assert.rejects(() => loadOnnxVisionSnapshot("road", invalidTarget), /invalid image/);
});
