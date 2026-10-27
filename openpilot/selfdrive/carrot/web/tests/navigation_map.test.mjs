import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import vm from "node:vm";

const source = await readFile(new URL("../js/pages/navigation_map.js", import.meta.url), "utf8");
const settle = () => new Promise((resolve) => setImmediate(resolve));

async function page(responses, language = "en") {
  const elements = new Map(), timers = [], revoked = [];
  let blobs = 0;
  const element = (id) => {
    if (!elements.has(id)) elements.set(id, {
      hidden: false, textContent: "", addEventListener() {},
      removeAttribute(name) { delete this[name]; },
    });
    return elements.get(id);
  };
  vm.runInNewContext(source, {
    document: { hidden: false, getElementById: element },
    localStorage: { getItem: () => language }, navigator: {},
    URL: { createObjectURL: () => `blob:${++blobs}`, revokeObjectURL: (url) => revoked.push(url) },
    setTimeout: (callback) => timers.push(callback),
    fetch: async (url) => {
      if (url.endsWith("destination")) return { ok: true, json: async () => ({ destination: null }) };
      const response = responses.shift();
      if (response instanceof Error) throw response;
      assert.ok(response, "unexpected map request");
      return { status: response.status, headers: { get: () => response.reason }, blob: async () => ({}) };
    },
  });
  await settle();
  return { element, revoked, blobs: () => blobs, next: async () => { await timers.shift()(); } };
}

test("keys without a route show destination instructions instead of a black image", async () => {
  const p = await page([{ status: 200, reason: "route" }]);
  assert.equal(p.element("map").hidden, true);
  assert.equal(p.element("mapStatus").hidden, false);
  assert.match(p.element("mapStatus").textContent, /select a destination below/);
  assert.match(p.element("mapStatus").textContent, /Saving keys alone/);
  assert.equal(p.blobs(), 0);
});

test("offroad explains when the route map becomes available", async () => {
  const p = await page([{ status: 204, reason: "offroad" }]);
  assert.equal(p.element("mapStatus").hidden, false);
  assert.match(p.element("mapStatus").textContent, /driving screen is active/);
  assert.equal(p.element("map").hidden, true);
});

test("ready map replaces instructions and GPS loss removes the stale image", async () => {
  const p = await page([{ status: 200, reason: "ready" }, { status: 200, reason: "position" }]);
  assert.equal(p.element("map").hidden, false);
  assert.equal(p.element("map").src, "blob:1");
  assert.equal(p.element("mapStatus").hidden, true);
  await p.next();
  assert.equal(p.element("map").hidden, true);
  assert.equal(p.element("map").src, undefined);
  assert.equal(p.element("mapStatus").hidden, false);
  assert.match(p.element("mapStatus").textContent, /GPS/);
  assert.deepEqual(p.revoked, ["blob:1"]);
});

test("network failure displays a localized connection message", async () => {
  const p = await page([new Error("offline")], "ko");
  assert.equal(p.element("map").hidden, true);
  assert.equal(p.element("mapStatus").hidden, false);
  assert.match(p.element("mapStatus").textContent, /차량과의 연결/);
});
