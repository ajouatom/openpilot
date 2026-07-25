import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import vm from "node:vm";

const identitySource = await readFile(
  new URL("../js/shared/stream_identity.js", import.meta.url),
  "utf8",
);
const indexSource = await readFile(new URL("../index.html", import.meta.url), "utf8");

class MemoryStorage {
  constructor(entries = {}) {
    this.values = new Map(Object.entries(entries));
  }

  getItem(key) {
    return this.values.has(key) ? this.values.get(key) : null;
  }

  setItem(key, value) {
    this.values.set(key, String(value));
  }
}

function installIdentity({
  persistentStorage = new MemoryStorage(),
  sessionStorage = new MemoryStorage(),
  legacyIdentity = null,
  uuids = [],
} = {}) {
  let uuidIndex = 0;
  const window = {
    localStorage: persistentStorage,
    sessionStorage,
    crypto: {
      randomUUID() {
        const value = uuids[uuidIndex] || `uuid-${uuidIndex + 1}`;
        uuidIndex += 1;
        return value;
      },
    },
    CarrotStreamIdentity: legacyIdentity,
  };
  const context = vm.createContext({ window });
  vm.runInContext(identitySource, context, { filename: "stream_identity.js" });
  return {
    identity: window.CarrotStreamIdentity,
    policy: window.CarrotStreamIdentityPolicy,
    persistentStorage,
    sessionStorage,
  };
}

test("legacy tab client id migrates into the persistent device id", () => {
  const sessionStorage = new MemoryStorage({ carrot_stream_client_id: "legacy-client" });
  const installed = installIdentity({
    sessionStorage,
    uuids: ["tab-uuid"],
  });

  assert.equal(installed.identity.deviceId, "legacy-client");
  assert.equal(installed.identity.clientId, "legacy-client");
  assert.equal(installed.persistentStorage.getItem("carrot_stream_device_id_v1"), "legacy-client");
  assert.equal(installed.identity.tabId, "carrot-tab-tab-uuid");
});

test("identity policy loads after legacy constants and before WebRTC", () => {
  const constantsIndex = indexSource.indexOf("/js/shared/constants.js");
  const identityIndex = indexSource.indexOf("/js/shared/stream_identity.js");
  const rtcIndex = indexSource.indexOf("/js/realtime/vision_rtc.js");

  assert.ok(constantsIndex >= 0);
  assert.ok(identityIndex > constantsIndex);
  assert.ok(rtcIndex > identityIndex);
});

test("device id survives a new tab while tab id remains tab-scoped", () => {
  const persistentStorage = new MemoryStorage();
  const first = installIdentity({
    persistentStorage,
    sessionStorage: new MemoryStorage(),
    uuids: ["device-one", "tab-one"],
  });
  const second = installIdentity({
    persistentStorage,
    sessionStorage: new MemoryStorage(),
    uuids: ["tab-two"],
  });

  assert.equal(first.identity.deviceId, second.identity.deviceId);
  assert.notEqual(first.identity.tabId, second.identity.tabId);
});

test("tab id survives reloads in the same browser tab", () => {
  const persistentStorage = new MemoryStorage();
  const sessionStorage = new MemoryStorage();
  const first = installIdentity({
    persistentStorage,
    sessionStorage,
    uuids: ["device-one", "tab-one"],
  });
  const second = installIdentity({
    persistentStorage,
    sessionStorage,
    uuids: ["unused"],
  });

  assert.equal(first.identity.deviceId, second.identity.deviceId);
  assert.equal(first.identity.tabId, second.identity.tabId);
});

test("every connection attempt receives a separate bounded id", () => {
  const { identity } = installIdentity({
    uuids: ["device-one", "tab-one", "attempt-one", "attempt-two"],
  });

  const firstAttempt = identity.createAttemptId();
  const secondAttempt = identity.createAttemptId();

  assert.notEqual(firstAttempt, secondAttempt);
  assert.match(firstAttempt, /^carrot-attempt-/);
  assert.ok(firstAttempt.length <= 128);
  assert.ok(secondAttempt.length <= 128);
});

test("unavailable storage falls back to in-memory opaque identifiers", () => {
  const unavailableStorage = {
    getItem() {
      throw new Error("storage unavailable");
    },
    setItem() {
      throw new Error("storage unavailable");
    },
  };
  const installed = installIdentity({
    persistentStorage: unavailableStorage,
    sessionStorage: unavailableStorage,
    uuids: ["device-one", "tab-one", "attempt-one"],
  });

  assert.match(installed.identity.deviceId, /^carrot-device-/);
  assert.match(installed.identity.tabId, /^carrot-tab-/);
  assert.match(installed.identity.createAttemptId(), /^carrot-attempt-/);
});
