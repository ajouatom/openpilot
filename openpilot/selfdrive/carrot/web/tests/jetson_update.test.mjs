import test from "node:test";
import assert from "node:assert/strict";
import { render, change } from "../src/features/tools/jetson_update.js";

test("legacy wait stays visible after disconnect and offers cancellation, never false completion", async t => {
  const saved = new Map(["document", "window", "getJson", "postJson", "appConfirm"].map(k => [k, globalThis[k]]));
  t.after(() => saved.forEach((value, key) => { globalThis[key] = value; }));
  const nodes = new Map();
  globalThis.document = { getElementById(id) {
    if (!nodes.has(id)) nodes.set(id, { dataset: {} });
    return nodes.get(id);
  } };
  globalThis.window = { clearTimeout() {}, setTimeout() { return 1; } };
  let value = { connected: true, migrated: false, pending: false };
  const writes = [];
  globalThis.getJson = async () => value;
  globalThis.postJson = async (url, body) => {
    writes.push([url, body]);
    value = { ...value, pending: body.enabled };
    return value;
  };
  globalThis.appConfirm = async () => true;
  render(value);
  assert.equal(nodes.get("jetsonUpdateCard").hidden, false);
  await change();
  assert.deepEqual(writes[0], ["/api/tools/jetson-update", { enabled: true }]);
  assert.match(nodes.get("jetsonUpdateDetail").textContent, /cannot report download completion/);
  value.connected = false;
  render(value);
  assert.equal(nodes.get("jetsonUpdateCard").hidden, false);
  assert.match(nodes.get("btnJetsonUpdateWait").textContent, /Cancel/);
  await change();
  assert.deepEqual(writes[1][1], { enabled: false });
  render({ connected: true, migrated: true, pending: false });
  assert.equal(nodes.get("jetsonUpdateCard").hidden, true);
});
