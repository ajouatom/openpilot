import assert from "node:assert/strict";
import test from "node:test";

import {
  PARAM_CHANGE_SOURCES,
  createSettingCommitter,
} from "../src/shared/params/commit_value.js";

function createHarness(options = {}) {
  const requests = [];
  const committed = [];
  const response = options.response;
  const committer = createSettingCommitter({
    postJson: async (url, body) => {
      requests.push({ url, body });
      if (typeof response === "function") return response(body);
      return response ?? { ok: true, name: body.name, value: body.value };
    },
    onCommitted: (result) => committed.push(result),
    ...options.overrides,
  });
  return { committer, requests, committed };
}

test("a postJson function is required", () => {
  assert.throws(() => createSettingCommitter({}), TypeError);
});

test("a commit posts the name, value and source together", async () => {
  const { committer, requests } = createHarness();
  await committer.commit("ApplyModelSpeed", -1);

  assert.deepEqual(requests, [{
    url: "/api/param_set",
    body: { name: "ApplyModelSpeed", value: -1, source: "web_ui" },
  }]);
});

test("an empty name is rejected before any request is made", async () => {
  const { committer, requests } = createHarness();
  await assert.rejects(() => committer.commit("", 1), TypeError);
  await assert.rejects(() => committer.commit("   ", 1), TypeError);
  assert.deepEqual(requests, []);
});

// Otherwise the screen can show a value the device does not hold.
test("the stored value from the server wins over the requested one", async () => {
  const { committer } = createHarness({
    response: (body) => ({ ok: true, name: body.name, value: 100 }),
  });

  const result = await committer.commit("TFollowDecelBoost", 150);
  assert.equal(result.value, 100, "the clamped server value must be adopted");
  assert.equal(result.requested, 150);
  assert.equal(result.adjusted, true, "an adjusted write must be visible to callers");
});

test("an unchanged value is not reported as adjusted", async () => {
  const { committer } = createHarness();
  const result = await committer.commit("A", 5);

  assert.equal(result.value, 5);
  assert.equal(result.adjusted, false);
});

test("a response without a value falls back to what was requested", async () => {
  const { committer } = createHarness({ response: { ok: true } });
  const result = await committer.commit("A", 7);

  assert.equal(result.value, 7);
  assert.equal(result.adjusted, false);
});

test("a null value from the server is still adopted", async () => {
  const { committer } = createHarness({ response: { ok: true, value: null } });
  const result = await committer.commit("A", 7);

  assert.equal(result.value, null);
  assert.equal(result.adjusted, true);
});

test("every known source is passed through unchanged", async () => {
  const { committer, requests } = createHarness();
  for (const source of Object.values(PARAM_CHANGE_SOURCES)) {
    await committer.commit("A", 1, { source });
  }

  assert.deepEqual(
    requests.map((request) => request.body.source),
    Object.values(PARAM_CHANGE_SOURCES),
  );
});

test("an unrecognised source falls back to the web UI rather than being invented", async () => {
  const { committer, requests } = createHarness();
  await committer.commit("A", 1, { source: "made_up" });
  await committer.commit("A", 2, {});

  assert.deepEqual(requests.map((request) => request.body.source), ["web_ui", "web_ui"]);
});

test("committed results reach the observer and the event sink", async () => {
  const events = [];
  const { committer, committed } = createHarness({
    overrides: { dispatchEvent: (result) => events.push(result) },
  });

  await committer.commit("A", 3, { source: PARAM_CHANGE_SOURCES.undo });

  assert.equal(committed.length, 1);
  assert.equal(events.length, 1);
  assert.equal(events[0].source, "undo");
  assert.equal(Object.isFrozen(events[0]), true);
});

test("a failed request propagates and notifies nobody", async () => {
  const committed = [];
  const committer = createSettingCommitter({
    postJson: async () => { throw new Error("HTTP 500"); },
    onCommitted: (result) => committed.push(result),
  });

  await assert.rejects(() => committer.commit("A", 1), /HTTP 500/);
  assert.deepEqual(committed, [], "a failed write must not look like a successful one");
});

test("the endpoint can be overridden for tests and future routes", async () => {
  const { committer, requests } = createHarness({ overrides: { endpoint: "/api/other" } });
  await committer.commit("A", 1);

  assert.equal(requests[0].url, "/api/other");
});
