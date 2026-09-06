import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { setImmediate } from "node:timers/promises";
import test from "node:test";
import vm from "node:vm";

const html = readFileSync(new URL("../../xiaoge/v_asm_web.html", import.meta.url), "utf8");
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
const ready = {
  configuredSides: ["left", "right"],
  vehicleSide: { left: { valid: true, active: true, confidence: 0.8 } },
  camera: { available: true, lastFrameAgeSeconds: 0 },
  lane: {
    cameraAvailable: true, resultFresh: true, threshold: 0.25, intervalSeconds: 0.4,
    result: { leftLine: 1, rightLine: 0, leftConf: 0.9, rightConf: 0.8 },
  },
};
const reply = (data, ok = true) => ({ ok, json: async () => data });

function page(url, fetchResponse, browserLanguage = "en-US") {
  const elements = [...html.matchAll(/<[a-z][\w-]*\b([^<>]*)>/gi)].map((match) => {
    const attrs = Object.fromEntries([...match[1].matchAll(/([\w-]+)="([^"]*)"/g)].map((a) => [a[1], a[2]]));
    return {
      id: attrs.id, dataset: { i18n: attrs["data-i18n"], side: attrs["data-side"] },
      textContent: "", value: "", style: {}, hidden: false, classList: { toggle() {} },
      getContext: () => ({}),
    };
  });
  const ids = new Map(elements.filter((el) => el.id).map((el) => [el.id, el]));
  const calls = [], images = [];
  const document = {
    documentElement: {}, hidden: false,
    querySelector(selector) {
      assert.ok(ids.has(selector.slice(1)), selector);
      return ids.get(selector.slice(1));
    },
    querySelectorAll(selector) {
      const key = { "[data-i18n]": "i18n", "[data-side]": "side" }[selector];
      assert.ok(key, selector);
      return elements.filter((el) => el.dataset[key]);
    },
  };
  const context = vm.createContext({
    document, location: new URL(url), navigator: { language: browserLanguage }, URLSearchParams,
    performance, setInterval() {},
    fetch(path, options) {
      const absolute = new URL(path, url);
      calls.push({ url: absolute, options });
      return fetchResponse(absolute.pathname, options);
    },
    Image: class { set src(path) { images.push(new URL(path, url)); } },
  });
  vm.runInContext(`${script}\nglobalThis.diagnostics={poll,api,languages};`, context);
  return { ...context.diagnostics, ids, calls, images, document };
}

test("Korean unavailable notice reconnects and clears stale detections after a disconnect", async () => {
  let online = false;
  const p = page("http://comma.local:7000/xiaoge/?lang=ko", (path) => {
    if (!online) return reply({ code: "vision_unavailable" }, false);
    return reply(path.endsWith("/status") ? ready : { width: 1928, height: 1208, poly_left: [], poly_right: [] });
  });
  await setImmediate();
  assert.equal(p.document.documentElement.lang, "ko");
  assert.equal(p.document.title, "ONNX 차선·BSD 진단");
  assert.equal(p.ids.get("carrot-nav").hidden, false);
  assert.equal(p.ids.get("connection-error").hidden, false);
  assert.match(p.ids.get("connection-error").textContent, /설정.*자동으로.*연결/);
  assert.equal(p.ids.get("status-left").textContent, p.languages.ko.unknown);
  for (const dictionary of Object.values(p.languages)) {
    assert.deepEqual(Object.keys(dictionary).sort(), Object.keys(p.languages.en).sort());
  }

  online = true;
  await p.poll();
  assert.equal(p.ids.get("connection-error").hidden, true);
  assert.equal(p.ids.get("status-lane-left").textContent, p.languages.ko.solid);
  assert.equal(p.ids.get("status-left").textContent, p.languages.ko.hasCar);
  online = false;
  await p.poll();
  assert.equal(p.ids.get("status-lane-left").textContent, p.languages.ko.unknown);
  assert.equal(p.ids.get("status-left").textContent, p.languages.ko.unknown);
  assert.equal(p.ids.get("conf-val-left").textContent, "0%");
});

test("API calls and snapshots work through port 7000 and directly on port 8082", async () => {
  for (const [url, prefix, hasBackLink] of [
    ["http://comma.local:7000/xiaoge/?lang=en", "/xiaoge/api/", true],
    ["http://comma.local:8082/", "/api/", false],
  ]) {
    const p = page(url, (path) => reply(path.endsWith("/status") ? ready : {}));
    await setImmediate();
    await p.api("config", { method: "POST", body: "{}" });
    await p.api("config", { method: "DELETE" });
    await p.api("settings", { method: "POST", body: "{}" });
    assert.equal(p.ids.get("carrot-nav").hidden, !hasBackLink);
    assert.deepEqual(p.calls.map((c) => c.url.pathname), ["status", "config", "config", "config", "settings"].map((s) => prefix + s));
    assert.deepEqual(p.images.map((i) => i.pathname), [prefix + "snapshot", prefix + "snapshot"]);
    for (const request of [...p.calls.map((c) => c.url), ...p.images]) {
      assert.equal(request.origin, new URL(url).origin);
    }
  }
});

test("a slow service cannot accumulate overlapping status polls", async () => {
  let resolve;
  const first = new Promise((done) => { resolve = done; });
  const p = page("http://comma.local:7000/xiaoge/", (path) => path.endsWith("/status") ? first : reply({}));
  await p.poll();
  await p.poll();
  assert.equal(p.calls.length, 1);
  resolve(reply(ready));
  await setImmediate();
  assert.equal(p.calls.length, 2);
});

test("failed settings writes display the server error instead of reporting success", async () => {
  const p = page("http://comma.local:7000/xiaoge/?lang=ko", (path) => {
    if (path.endsWith("/settings")) return reply({ error: "invalid threshold" }, false);
    return reply(path.endsWith("/status") ? ready : {});
  });
  await setImmediate();
  await p.ids.get("settings-lane").onsubmit({ preventDefault() {} });
  assert.equal(p.ids.get("hint").textContent, "invalid threshold");
});
