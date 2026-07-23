import {
  createPublishedSettingDocumentationClient,
  PUBLISHED_SETTINGS_INDEX_URL,
} from "./published_documentation.js";

const DOC_LANGUAGES = new Set(["ko", "en", "zh"]);
export const SETTING_DOCUMENTATION_INDEX_URL = PUBLISHED_SETTINGS_INDEX_URL;

export function normalizeSettingDocLanguage(value) {
  const language = String(value || "").trim().toLowerCase();
  return DOC_LANGUAGES.has(language) ? language : "en";
}

export function resolveSettingDocumentationIndexUrl() {
  return SETTING_DOCUMENTATION_INDEX_URL;
}

export function createSettingDocumentationClient(options = {}) {
  const indexUrl = String(options.indexUrl || PUBLISHED_SETTINGS_INDEX_URL);
  const client = createPublishedSettingDocumentationClient({
    fetchImpl: options.fetchImpl,
    storage: options.storage,
    now: options.now,
    indexUrl,
    indexTtlMs: options.indexTtlMs,
    requestTimeoutMs: options.requestTimeoutMs,
    maxIndexBytes: options.maxIndexBytes,
    maxPageBytes: options.maxPageBytes,
    maxMemoryPages: options.maxMemoryPages,
    maxStorageBytes: options.maxStorageBytes,
    clock: options.clock,
  });

  function load(...args) {
    return client.load(...args);
  }

  function clear() {
    client.clear();
  }

  function diagnostics() {
    return {
      ...client.diagnostics(),
      channel: "wiki",
      indexUrl,
    };
  }

  return Object.freeze({ load, clear, diagnostics });
}
