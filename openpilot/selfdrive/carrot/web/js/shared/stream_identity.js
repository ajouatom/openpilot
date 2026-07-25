(function installCarrotStreamIdentity(global) {
  "use strict";

  const DEVICE_STORAGE_KEY = "carrot_stream_device_id_v1";
  const TAB_STORAGE_KEY = "carrot_stream_tab_id_v1";
  const LEGACY_CLIENT_STORAGE_KEY = "carrot_stream_client_id";
  const MAX_IDENTIFIER_LENGTH = 128;

  function normalizeIdentifier(value) {
    return String(value || "")
      .trim()
      .replace(/[^a-zA-Z0-9._:-]/g, "-")
      .slice(0, MAX_IDENTIFIER_LENGTH);
  }

  function storageGet(storage, key) {
    try {
      return normalizeIdentifier(storage?.getItem?.(key));
    } catch (_) {
      return "";
    }
  }

  function storageSet(storage, key, value) {
    try {
      storage?.setItem?.(key, value);
      return true;
    } catch (_) {
      return false;
    }
  }

  function randomIdentifier(prefix, options) {
    let uuid = "";
    try {
      uuid = normalizeIdentifier(options.crypto?.randomUUID?.());
    } catch (_) {}
    if (uuid) return `${prefix}-${uuid}`.slice(0, MAX_IDENTIFIER_LENGTH);

    const now = Number(options.now?.()) || Date.now();
    const random = Number(options.random?.());
    const randomPart = Number.isFinite(random)
      ? Math.abs(random).toString(36).replace(".", "")
      : Math.random().toString(36).slice(2);
    return `${prefix}-${now.toString(36)}-${randomPart}`.slice(0, MAX_IDENTIFIER_LENGTH);
  }

  function createStreamIdentity(options = {}) {
    const persistentStorage = options.persistentStorage || null;
    const sessionStorage = options.sessionStorage || null;
    const idOptions = {
      crypto: options.crypto || null,
      now: typeof options.now === "function" ? options.now : Date.now,
      random: typeof options.random === "function" ? options.random : Math.random,
    };

    const legacyClientId = normalizeIdentifier(options.legacyClientId)
      || storageGet(sessionStorage, LEGACY_CLIENT_STORAGE_KEY);
    const deviceId = storageGet(persistentStorage, DEVICE_STORAGE_KEY)
      || legacyClientId
      || randomIdentifier("carrot-device", idOptions);
    storageSet(persistentStorage, DEVICE_STORAGE_KEY, deviceId);

    const tabId = storageGet(sessionStorage, TAB_STORAGE_KEY)
      || randomIdentifier("carrot-tab", idOptions);
    storageSet(sessionStorage, TAB_STORAGE_KEY, tabId);

    let attemptSequence = 0;
    function createAttemptId() {
      attemptSequence += 1;
      const generated = randomIdentifier("carrot-attempt", idOptions);
      return `${generated}-${attemptSequence.toString(36)}`.slice(0, MAX_IDENTIFIER_LENGTH);
    }

    return Object.freeze({
      version: 1,
      deviceId,
      tabId,
      clientId: deviceId,
      createAttemptId,
    });
  }

  const api = Object.freeze({
    deviceStorageKey: DEVICE_STORAGE_KEY,
    tabStorageKey: TAB_STORAGE_KEY,
    legacyClientStorageKey: LEGACY_CLIENT_STORAGE_KEY,
    create: createStreamIdentity,
  });

  let persistentStorage = null;
  let sessionStorage = null;
  try { persistentStorage = global.localStorage; } catch (_) {}
  try { sessionStorage = global.sessionStorage; } catch (_) {}

  const previousIdentity = global.CarrotStreamIdentity;
  global.CarrotStreamIdentityPolicy = api;
  global.CarrotStreamIdentity = createStreamIdentity({
    persistentStorage,
    sessionStorage,
    crypto: global.crypto,
    legacyClientId: previousIdentity?.clientId,
  });
})(typeof window !== "undefined" ? window : globalThis);
