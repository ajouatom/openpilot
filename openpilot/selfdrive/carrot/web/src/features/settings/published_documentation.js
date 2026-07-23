import { parseSettingDocumentationAst } from "./documentation_ast.js";
import { canonicalJson, sha256Hex, utf8Bytes } from "./documentation_hash.js";

export const PUBLISHED_SETTINGS_INDEX_URL =
  "https://raw.githubusercontent.com/wiki/ajouatom/openpilot/Settings-Catalog.json";

const SUPPORTED_LANGUAGES = new Set(["ko", "en", "zh"]);
const PARAM_RE = /^[A-Za-z][A-Za-z0-9_]*$/;
const HASH_RE = /^[a-f0-9]{64}$/;
const COMMIT_RE = /^[a-f0-9]{7,40}$/;
const PAGE_RE = /^(?:KO|EN|ZH)-[^\u0000-\u001f<>:"/\\|?*#%]+\.md$/u;
const REVIEW_STATES = new Set(["current", "needs_review"]);
const INDEX_KEYS = new Set([
  "schemaVersion", "catalogCommit", "wikiCommit", "generatedAt",
  "locales", "pages", "settings", "review", "contentHash",
]);
const DEFAULT_INDEX_TTL_MS = 30_000;
const DEFAULT_REQUEST_TIMEOUT_MS = 8_000;
const DEFAULT_MAX_INDEX_BYTES = 1_000_000;
const DEFAULT_MAX_PAGE_BYTES = 512_000;
const DEFAULT_MAX_MEMORY_PAGES = 12;
const DEFAULT_MAX_STORAGE_BYTES = 1_250_000;
const STORAGE_VERSION = 1;
const STORAGE_KEY = "carrot.settings.wiki-docs.v1";

export class PublishedSettingDocumentationError extends Error {
  constructor(message, code = "invalid") {
    super(message);
    this.name = "PublishedSettingDocumentationError";
    this.code = code;
  }
}

function fail(message, code) {
  throw new PublishedSettingDocumentationError(message, code);
}

function normalizedLanguage(value) {
  const language = String(value || "").trim().toLowerCase();
  return SUPPORTED_LANGUAGES.has(language) ? language : "en";
}

function assertExactKeys(value, allowed, context) {
  if (!value || typeof value !== "object" || Array.isArray(value)) fail(`${context} must be an object`);
  const unknown = Object.keys(value).filter((key) => !allowed.has(key));
  if (unknown.length) fail(`${context} contains unknown properties: ${unknown.join(", ")}`);
}

function validatePageMetadata(value, page) {
  assertExactKeys(value, new Set(["hash", "bytes"]), `page ${page}`);
  if (!HASH_RE.test(value.hash)) fail(`page ${page} has an invalid hash`);
  if (!Number.isInteger(value.bytes) || value.bytes < 1 || value.bytes > DEFAULT_MAX_PAGE_BYTES) {
    fail(`page ${page} has an invalid byte size`);
  }
}

export function validatePublishedSettingsIndex(value) {
  assertExactKeys(value, INDEX_KEYS, "settings index");
  if (value.schemaVersion !== 1) fail("unsupported settings index schema");
  if (!COMMIT_RE.test(value.catalogCommit || "")) fail("settings index catalogCommit is invalid");
  if (value.wikiCommit != null && !COMMIT_RE.test(value.wikiCommit)) {
    fail("settings index wikiCommit is invalid");
  }
  if (!Number.isFinite(Date.parse(value.generatedAt || ""))) fail("settings index generatedAt is invalid");
  if (!HASH_RE.test(value.contentHash || "")) fail("settings index contentHash is invalid");

  if (!Array.isArray(value.locales) || value.locales.length < 2 || value.locales.length > 3) {
    fail("settings index locales are invalid");
  }
  const locales = value.locales.map(normalizedLanguage);
  if (new Set(locales).size !== locales.length || !locales.includes("ko") || !locales.includes("en")) {
    fail("settings index must contain unique ko and en locales");
  }
  if (value.locales.some((locale, index) => locale !== locales[index])) {
    fail("settings index locale is unsupported");
  }

  assertExactKeys(value.pages, new Set(Object.keys(value.pages || {})), "settings index pages");
  const pageNames = Object.keys(value.pages);
  if (!pageNames.length || pageNames.length > 2_000) fail("settings index page count is invalid");
  pageNames.forEach((page) => {
    if (page !== "Settings-Catalog.md" && !PAGE_RE.test(page)) fail(`unsafe published page name: ${page}`);
    validatePageMetadata(value.pages[page], page);
  });

  assertExactKeys(value.settings, new Set(Object.keys(value.settings || {})), "settings index settings");
  const settingNames = Object.keys(value.settings);
  if (!settingNames.length || settingNames.length > 5_000) fail("settings index setting count is invalid");
  settingNames.forEach((param) => {
    if (!PARAM_RE.test(param)) fail(`invalid setting parameter: ${param}`);
    const setting = value.settings[param];
    assertExactKeys(setting, new Set(["group", "semanticHash", "reviewStatus", "locales"]), `setting ${param}`);
    if (!PARAM_RE.test(setting.group || "")) fail(`setting ${param} has an invalid group`);
    if (!HASH_RE.test(setting.semanticHash || "")) fail(`setting ${param} has an invalid semantic hash`);
    if (!REVIEW_STATES.has(setting.reviewStatus)) fail(`setting ${param} has an invalid review status`);
    assertExactKeys(setting.locales, new Set(locales), `setting ${param} locales`);
    if (Object.keys(setting.locales).length !== locales.length
        || locales.some((locale) => !setting.locales[locale])) {
      fail(`setting ${param} must provide every index locale`);
    }
    Object.entries(setting.locales).forEach(([locale, item]) => {
      if (!locales.includes(locale)) fail(`setting ${param} has an unsupported locale`);
      assertExactKeys(item, new Set(["page", "anchor", "factsHash", "reviewStatus"]), `setting ${param}/${locale}`);
      if (!PAGE_RE.test(item.page || "") || !value.pages[item.page]) {
        fail(`setting ${param}/${locale} references an unknown page`);
      }
      if (item.anchor !== param) fail(`setting ${param}/${locale} has a mismatched anchor`);
      if (!HASH_RE.test(item.factsHash || "")) fail(`setting ${param}/${locale} has an invalid facts hash`);
      if (!REVIEW_STATES.has(item.reviewStatus)) fail(`setting ${param}/${locale} has an invalid review status`);
    });
  });

  assertExactKeys(value.review, new Set(["current", "needs_review"]), "settings index review");
  const current = Number(value.review.current);
  const needsReview = Number(value.review.needs_review);
  if (!Number.isInteger(current) || !Number.isInteger(needsReview)
      || current < 0 || needsReview < 0 || current + needsReview !== settingNames.length) {
    fail("settings index review totals are invalid");
  }

  const canonical = { ...value };
  delete canonical.contentHash;
  const expectedHash = sha256Hex(canonicalJson(canonical));
  if (expectedHash !== value.contentHash) fail("settings index content hash does not match", "hash");
  return Object.freeze(value);
}

export function parsePublishedSettingsIndex(text, maxBytes = DEFAULT_MAX_INDEX_BYTES) {
  const source = String(text || "");
  if (utf8Bytes(source).byteLength > maxBytes) fail("settings index is too large", "capacity");
  let value;
  try {
    value = JSON.parse(source);
  } catch (_) {
    fail("settings index is not valid JSON");
  }
  return validatePublishedSettingsIndex(value);
}

function oneLineIndex(lines, exact, context) {
  const matches = [];
  lines.forEach((line, index) => {
    if (line === exact) matches.push(index);
  });
  if (matches.length !== 1) fail(`${context} marker must appear exactly once`);
  return matches[0];
}

export function extractPublishedSettingMarkdown(pageText, descriptor) {
  const source = String(pageText || "").replaceAll("\r\n", "\n");
  const lines = source.split("\n");
  const header =
    /^<!-- CARROT:GENERATED schema="1" locale="(ko|en|zh)" group="([A-Za-z][A-Za-z0-9_]*)" param="([A-Za-z][A-Za-z0-9_]*)" -->$/
      .exec(lines[0] || "");
  if (!header) fail("published page header is invalid");
  if (
    header[1] !== descriptor.language
    || header[2] !== descriptor.group
    || header[3] !== descriptor.param
  ) {
    fail("published page locale, group, or parameter does not match the index");
  }

  const param = descriptor.param;
  const settingBegin = oneLineIndex(lines, `<!-- CARROT:SETTING:BEGIN ${param} -->`, `${param} begin`);
  const settingEnd = oneLineIndex(lines, `<!-- CARROT:SETTING:END ${param} -->`, `${param} end`);
  if (settingEnd <= settingBegin) fail(`${param} setting markers are out of order`);
  const block = lines.slice(settingBegin + 1, settingEnd);

  const autoBegin = oneLineIndex(block, `<!-- CARROT:AUTO:BEGIN ${param} -->`, `${param} auto begin`);
  const autoEnd = oneLineIndex(block, `<!-- CARROT:AUTO:END ${param} -->`, `${param} auto end`);
  const manualBegin = oneLineIndex(block, `<!-- CARROT:MANUAL:BEGIN ${param} -->`, `${param} manual begin`);
  const manualEnd = oneLineIndex(block, `<!-- CARROT:MANUAL:END ${param} -->`, `${param} manual end`);
  if (!(autoBegin < autoEnd && autoEnd < manualBegin && manualBegin < manualEnd)) {
    fail(`${param} automatic and manual regions are out of order`);
  }

  const sourceLines = block.filter((line) => line.startsWith("<!-- CARROT:AUTO:SOURCE "));
  if (sourceLines.length !== 1) fail(`${param} automatic source metadata is missing`);
  const sourceMatch =
    /^<!-- CARROT:AUTO:SOURCE facts="([a-f0-9]{64})" semantic="([a-f0-9]{64})" review="(current|needs_review)" -->$/
      .exec(sourceLines[0]);
  if (!sourceMatch) fail(`${param} automatic source metadata is invalid`);
  if (sourceMatch[1] !== descriptor.factsHash
      || sourceMatch[2] !== descriptor.semanticHash
      || sourceMatch[3] !== descriptor.reviewStatus) {
    fail(`${param} automatic source metadata does not match the index`);
  }

  if (!block.includes(`<a id="${param}"></a>`)) fail(`${param} anchor is missing`);
  const visible = [];
  const sections = [];
  let currentSection = null;
  let sectionLines = [];
  block.forEach((line) => {
    const sectionBegin = /^<!-- CARROT:SECTION:BEGIN (OVERVIEW|BEHAVIOR|USAGE|REFERENCE) -->$/.exec(line);
    const sectionEnd = /^<!-- CARROT:SECTION:END (OVERVIEW|BEHAVIOR|USAGE|REFERENCE) -->$/.exec(line);
    if (sectionBegin) {
      if (currentSection) fail(`${param} contains nested sections`);
      currentSection = sectionBegin[1].toLowerCase();
      sectionLines = [];
      return;
    }
    if (sectionEnd) {
      if (currentSection !== sectionEnd[1].toLowerCase()) fail(`${param} section markers are unbalanced`);
      const markdown = sectionLines.join("\n").trim();
      if (!markdown) fail(`${param} contains an empty section`);
      sections.push(Object.freeze({
        kind: currentSection,
        markdown,
        ast: parseSettingDocumentationAst(markdown),
      }));
      currentSection = null;
      sectionLines = [];
      return;
    }
    if (line === `<a id="${param}"></a>` || /^<!-- CARROT:[A-Z:]+(?:\s.*)? -->$/.test(line)) return;
    visible.push(line);
    if (currentSection) sectionLines.push(line);
  });
  if (currentSection) fail(`${param} contains an unclosed section`);
  const sectionKinds = sections.map((section) => section.kind);
  const expectedOrder = ["overview", "behavior", "usage", "reference"];
  if (sectionKinds[0] !== "overview"
      || new Set(sectionKinds).size !== sectionKinds.length
      || sectionKinds.some((kind, index) => expectedOrder.indexOf(kind)
        <= (index ? expectedOrder.indexOf(sectionKinds[index - 1]) : -1))) {
    fail(`${param} sections are missing, duplicated, or out of order`);
  }

  const markdown = visible.join("\n").replace(/\n{3,}/g, "\n\n").trim();
  if (!markdown) fail(`${param} has no visible documentation`);
  return Object.freeze({
    markdown,
    ast: parseSettingDocumentationAst(markdown),
    sections: Object.freeze(sections),
  });
}

function abortError() {
  if (typeof DOMException === "function") return new DOMException("The operation was aborted", "AbortError");
  const error = new Error("The operation was aborted");
  error.name = "AbortError";
  return error;
}

function withCallerAbort(promise, signal) {
  if (!signal) return promise;
  if (signal.aborted) return Promise.reject(abortError());
  return new Promise((resolve, reject) => {
    const aborted = () => reject(abortError());
    signal.addEventListener("abort", aborted, { once: true });
    promise.then(resolve, reject).finally(() => signal.removeEventListener("abort", aborted));
  });
}

async function boundedResponseText(response, maxBytes) {
  const declared = Number(response.headers?.get?.("content-length"));
  if (Number.isFinite(declared) && declared > maxBytes) fail("published response is too large", "capacity");
  if (!response.body?.getReader) {
    const text = await response.text();
    if (utf8Bytes(text).byteLength > maxBytes) fail("published response is too large", "capacity");
    return text;
  }

  const reader = response.body.getReader();
  const chunks = [];
  let total = 0;
  while (true) {
    const { done, value } = await reader.read();
    if (done) break;
    total += value.byteLength;
    if (total > maxBytes) {
      await reader.cancel();
      fail("published response is too large", "capacity");
    }
    chunks.push(value);
  }
  const bytes = new Uint8Array(total);
  let offset = 0;
  chunks.forEach((chunk) => {
    bytes.set(chunk, offset);
    offset += chunk.byteLength;
  });
  try {
    return new TextDecoder("utf-8", { fatal: true }).decode(bytes);
  } catch (_) {
    fail("published response must be UTF-8");
  }
}

function pageDescriptor(index, param, requestedLanguage) {
  const setting = index.settings[param];
  if (!setting) return null;
  const language = setting.locales[requestedLanguage] ? requestedLanguage
    : setting.locales.en ? "en" : null;
  if (!language) return null;
  const localized = setting.locales[language];
  return Object.freeze({
    param,
    language,
    group: setting.group,
    semanticHash: setting.semanticHash,
    reviewStatus: localized.reviewStatus,
    page: localized.page,
    anchor: localized.anchor,
    factsHash: localized.factsHash,
    pageHash: index.pages[localized.page].hash,
    pageBytes: index.pages[localized.page].bytes,
  });
}

function buildPublishedPayload(index, descriptor, extracted, requestedLanguage, contentSource) {
  return Object.freeze({
    ok: true,
    available: true,
    name: descriptor.param,
    source: descriptor.page.replace(/\.md$/i, ""),
    anchor: descriptor.anchor,
    language_requested: requestedLanguage,
    language_resolved: descriptor.language,
    fallback: requestedLanguage !== descriptor.language,
    markdown: extracted.markdown,
    ast: extracted.ast,
    sections: extracted.sections,
    content_source: contentSource,
    catalog_hash: index.contentHash,
    catalog_commit: index.catalogCommit,
    wiki_commit: index.wikiCommit || "",
    review_status: descriptor.reviewStatus,
  });
}

function emptyStorageState() {
  return { version: STORAGE_VERSION, catalogs: [] };
}

function safeStorageRead(storage, storageKey, maxBytes) {
  try {
    const raw = storage?.getItem?.(storageKey);
    if (!raw || utf8Bytes(raw).byteLength > maxBytes) return emptyStorageState();
    const value = JSON.parse(raw);
    if (value?.version !== STORAGE_VERSION || !Array.isArray(value.catalogs)) return emptyStorageState();
    return value;
  } catch (_) {
    return emptyStorageState();
  }
}

function trimStorageState(state, maxBytes) {
  state.catalogs = state.catalogs
    .filter((catalog) => typeof catalog?.indexText === "string" && catalog.pages && typeof catalog.pages === "object")
    .sort((a, b) => Number(b.savedAt || 0) - Number(a.savedAt || 0))
    .slice(0, 2);
  state.catalogs.forEach((catalog) => {
    const pages = Object.entries(catalog.pages)
      .sort(([, a], [, b]) => Number(b.savedAt || 0) - Number(a.savedAt || 0))
      .slice(0, DEFAULT_MAX_MEMORY_PAGES);
    catalog.pages = Object.fromEntries(pages);
  });
  let encoded = JSON.stringify(state);
  while (utf8Bytes(encoded).byteLength > maxBytes) {
    const candidates = state.catalogs
      .flatMap((catalog) => Object.entries(catalog.pages)
        .map(([page, item]) => ({ catalog, page, savedAt: Number(item.savedAt || 0) })))
      .sort((a, b) => a.savedAt - b.savedAt);
    if (!candidates.length) return null;
    delete candidates[0].catalog.pages[candidates[0].page];
    state.catalogs = state.catalogs.filter((catalog) => Object.keys(catalog.pages).length);
    encoded = JSON.stringify(state);
  }
  return encoded;
}

function safeStorageWrite(storage, storageKey, state, maxBytes) {
  try {
    const encoded = trimStorageState(state, maxBytes);
    if (encoded) storage?.setItem?.(storageKey, encoded);
  } catch (_) {
    // Storage is a best-effort last-good cache. Settings must remain usable.
  }
}

export function createPublishedSettingDocumentationClient(options = {}) {
  const fetchImpl = options.fetchImpl || globalThis.fetch;
  const clock = options.clock || (() => globalThis.performance?.now?.() ?? Date.now());
  let defaultStorage = null;
  try {
    defaultStorage = globalThis.localStorage;
  } catch (_) {
    // Storage can be unavailable in private or opaque browser contexts.
  }
  const storage = options.storage === undefined ? defaultStorage : options.storage;
  const now = options.now || Date.now;
  const indexUrl = options.indexUrl || PUBLISHED_SETTINGS_INDEX_URL;
  const storageKey = options.storageKey || STORAGE_KEY;
  const indexTtlMs = options.indexTtlMs || DEFAULT_INDEX_TTL_MS;
  const requestTimeoutMs = options.requestTimeoutMs || DEFAULT_REQUEST_TIMEOUT_MS;
  const maxIndexBytes = options.maxIndexBytes || DEFAULT_MAX_INDEX_BYTES;
  const maxPageBytes = options.maxPageBytes || DEFAULT_MAX_PAGE_BYTES;
  const maxMemoryPages = options.maxMemoryPages || DEFAULT_MAX_MEMORY_PAGES;
  const maxStorageBytes = options.maxStorageBytes || DEFAULT_MAX_STORAGE_BYTES;
  const pageBaseUrl = indexUrl.slice(0, indexUrl.lastIndexOf("/") + 1);
  const inflight = new Map();
  const controllers = new Set();
  const memoryPages = new Map();
  const resultCache = new Map();
  let currentIndex = null;
  const diagnostics = {
    loadCount: 0,
    requestCount: 0,
    responseBytes: 0,
    unavailableCount: 0,
    networkErrorCount: 0,
    lastLoadMs: 0,
    lastParseMs: 0,
    lastContentSource: "",
    updatedAt: 0,
  };

  function elapsedSince(startedAt) {
    const elapsed = Number(clock()) - Number(startedAt);
    return Number.isFinite(elapsed) && elapsed > 0 ? elapsed : 0;
  }

  function diagnosticsSnapshot() {
    return Object.freeze({
      ...diagnostics,
      errorRate: diagnostics.loadCount
        ? diagnostics.unavailableCount / diagnostics.loadCount
        : 0,
    });
  }

  function sharedRequest(key, factory) {
    if (inflight.has(key)) return inflight.get(key);
    const pending = factory().finally(() => {
      if (inflight.get(key) === pending) inflight.delete(key);
    });
    inflight.set(key, pending);
    return pending;
  }

  async function requestText(url, maxBytes) {
    if (typeof fetchImpl !== "function") fail("published documentation fetch is unavailable", "network");
    diagnostics.requestCount += 1;
    const controller = new AbortController();
    controllers.add(controller);
    const timer = setTimeout(() => controller.abort(), requestTimeoutMs);
    try {
      const response = await fetchImpl(url, {
        cache: "no-store",
        credentials: "omit",
        referrerPolicy: "no-referrer",
        signal: controller.signal,
      });
      if (!response.ok) fail(`published documentation request failed: HTTP ${response.status}`, "network");
      const text = await boundedResponseText(response, maxBytes);
      diagnostics.responseBytes += utf8Bytes(text).byteLength;
      return text;
    } catch (error) {
      diagnostics.networkErrorCount += 1;
      throw error;
    } finally {
      clearTimeout(timer);
      controllers.delete(controller);
    }
  }

  function validatePageMeasured(indexRecord, descriptor, text, contentSource) {
    const startedAt = clock();
    try {
      return validatePage(indexRecord, descriptor, text, contentSource);
    } finally {
      diagnostics.lastParseMs = elapsedSince(startedAt);
    }
  }

  function loadIndex() {
    if (currentIndex && currentIndex.expiresAt > now()) return Promise.resolve(currentIndex);
    return sharedRequest("index", async () => {
      const bucket = Math.floor(now() / Math.max(1, indexTtlMs));
      const separator = indexUrl.includes("?") ? "&" : "?";
      const text = await requestText(`${indexUrl}${separator}v=${bucket}`, maxIndexBytes);
      const index = parsePublishedSettingsIndex(text, maxIndexBytes);
      currentIndex = { index, text, expiresAt: now() + indexTtlMs };
      return currentIndex;
    });
  }

  function rememberPage(key, value) {
    memoryPages.delete(key);
    memoryPages.set(key, value);
    while (memoryPages.size > maxMemoryPages) {
      memoryPages.delete(memoryPages.keys().next().value);
    }
  }

  function persistPage(indexRecord, descriptor, pageText) {
    const state = safeStorageRead(storage, storageKey, maxStorageBytes);
    let catalog = state.catalogs.find((item) => item.hash === indexRecord.index.contentHash);
    if (!catalog) {
      catalog = { hash: indexRecord.index.contentHash, indexText: indexRecord.text, savedAt: now(), pages: {} };
      state.catalogs.unshift(catalog);
    }
    catalog.indexText = indexRecord.text;
    catalog.savedAt = now();
    catalog.pages[descriptor.page] = { text: pageText, savedAt: now() };
    safeStorageWrite(storage, storageKey, state, maxStorageBytes);
  }

  function validatePage(indexRecord, descriptor, pageText, contentSource) {
    const bytes = utf8Bytes(pageText);
    if (bytes.byteLength !== descriptor.pageBytes || bytes.byteLength > maxPageBytes) {
      fail(`published page ${descriptor.page} byte size does not match`, "hash");
    }
    if (sha256Hex(bytes) !== descriptor.pageHash) {
      fail(`published page ${descriptor.page} hash does not match`, "hash");
    }
    const extracted = extractPublishedSettingMarkdown(pageText, descriptor);
    return buildPublishedPayload(
      indexRecord.index,
      descriptor,
      extracted,
      descriptor.requestedLanguage,
      contentSource,
    );
  }

  function loadLivePage(indexRecord, descriptor) {
    const key = `${indexRecord.index.contentHash}:${descriptor.page}`;
    const cached = memoryPages.get(key);
    if (cached) {
      rememberPage(key, cached);
      return Promise.resolve(
        validatePageMeasured(indexRecord, descriptor, cached, "published-memory"),
      );
    }
    const pendingText = sharedRequest(`page:${key}`, async () => {
      const pagePath = encodeURIComponent(descriptor.page);
      const url = `${pageBaseUrl}${pagePath}?catalog=${indexRecord.index.contentHash}`;
      const text = await requestText(url, Math.min(maxPageBytes, descriptor.pageBytes + 1));
      validatePageMeasured(indexRecord, descriptor, text, "published");
      rememberPage(key, text);
      persistPage(indexRecord, descriptor, text);
      return text;
    });
    return pendingText.then((text) => (
      validatePageMeasured(indexRecord, descriptor, text, "published")
    ));
  }

  async function loadStored(param, requestedLanguage) {
    const state = safeStorageRead(storage, storageKey, maxStorageBytes);
    const catalogs = [...state.catalogs].sort((a, b) => Number(b.savedAt || 0) - Number(a.savedAt || 0));
    for (const catalog of catalogs) {
      try {
        const index = parsePublishedSettingsIndex(catalog.indexText, maxIndexBytes);
        const descriptor = pageDescriptor(index, param, requestedLanguage);
        const page = descriptor && catalog.pages?.[descriptor.page];
        if (!descriptor || typeof page?.text !== "string") continue;
        const indexRecord = { index, text: catalog.indexText, expiresAt: 0 };
        return validatePageMeasured(
          indexRecord,
          { ...descriptor, requestedLanguage },
          page.text,
          "published-cache",
        );
      } catch (_) {
        // Try the previous validated catalog snapshot.
      }
    }
    return null;
  }

  function unavailablePayload(param, language) {
    return Object.freeze({
      available: false,
      name: param,
      language_requested: language,
      content_source: "published-unavailable",
    });
  }

  async function loadUncached(param, requestedLanguage) {
    try {
      const indexRecord = await loadIndex();
      const baseDescriptor = pageDescriptor(indexRecord.index, param, requestedLanguage);
      if (!baseDescriptor) return unavailablePayload(param, requestedLanguage);
      const descriptor = { ...baseDescriptor, requestedLanguage };
      return await loadLivePage(indexRecord, descriptor);
    } catch (_) {
      // Only a previously validated remote snapshot may replace a failed live request.
    }
    const stored = await loadStored(param, requestedLanguage);
    return stored || unavailablePayload(param, requestedLanguage);
  }

  function load(name, language, loadOptions = {}) {
    const param = String(name || "").trim();
    const requestedLanguage = normalizedLanguage(language);
    if (!PARAM_RE.test(param)) {
      return Promise.resolve(Object.freeze({ available: false, name: param, language_requested: requestedLanguage }));
    }
    const key = `${requestedLanguage}:${param}`;
    const cached = resultCache.get(key);
    if (cached && cached.expiresAt > now()) return withCallerAbort(cached.promise, loadOptions.signal);
    const startedAt = clock();
    diagnostics.loadCount += 1;
    const promise = loadUncached(param, requestedLanguage)
      .catch(() => unavailablePayload(param, requestedLanguage))
      .then((result) => {
        diagnostics.lastLoadMs = elapsedSince(startedAt);
        diagnostics.lastContentSource = String(result?.content_source || "");
        diagnostics.updatedAt = Number(now()) || 0;
        if (!result?.available) diagnostics.unavailableCount += 1;
        return result;
      });
    resultCache.set(key, { promise, expiresAt: now() + indexTtlMs });
    promise.catch(() => {
      if (resultCache.get(key)?.promise === promise) resultCache.delete(key);
    });
    return withCallerAbort(promise, loadOptions.signal);
  }

  function clear() {
    controllers.forEach((controller) => controller.abort());
    controllers.clear();
    inflight.clear();
    memoryPages.clear();
    resultCache.clear();
    currentIndex = null;
  }

  return Object.freeze({ load, clear, diagnostics: diagnosticsSnapshot });
}
