import assert from "node:assert/strict";
import test from "node:test";

import {
  createSettingDocumentationClient,
  normalizeSettingDocLanguage,
  resolveSettingDocumentationIndexUrl,
  SETTING_DOCUMENTATION_INDEX_URL,
} from "../src/features/settings/documentation.js";

test("documentation languages preserve ko/en/zh and otherwise fall back to English", () => {
  assert.equal(normalizeSettingDocLanguage("ko"), "ko");
  assert.equal(normalizeSettingDocLanguage("ZH"), "zh");
  assert.equal(normalizeSettingDocLanguage("../../ko"), "en");
});

test("documentation client reads only the verified remote publication path", async () => {
  const calls = [];
  const client = createSettingDocumentationClient({
    storage: null,
    indexUrl: "https://raw.githubusercontent.com/wiki/example/project/Settings-Catalog.json",
    fetchImpl: async (url) => {
      calls.push(url);
      throw new Error("offline");
    },
  });

  const payload = await client.load("AutoEngage", "ko");
  assert.equal(payload.available, false);
  assert.equal(payload.content_source, "published-unavailable");
  assert.equal(calls.length, 1);
  const requested = new URL(calls[0]);
  assert.equal(requested.origin, "https://raw.githubusercontent.com");
  assert.equal(requested.pathname, "/wiki/example/project/Settings-Catalog.json");
  assert.equal(calls.some((url) => String(url).startsWith("/api/settings/doc")), false);
  assert.deepEqual(
    {
      loadCount: client.diagnostics().loadCount,
      requestCount: client.diagnostics().requestCount,
      unavailableCount: client.diagnostics().unavailableCount,
      lastContentSource: client.diagnostics().lastContentSource,
      errorRate: client.diagnostics().errorRate,
    },
    {
      loadCount: 1,
      requestCount: 1,
      unavailableCount: 1,
      lastContentSource: "published-unavailable",
      errorRate: 1,
    },
  );
});

test("documentation always resolves the actual GitHub Wiki index", () => {
  assert.equal(resolveSettingDocumentationIndexUrl(), SETTING_DOCUMENTATION_INDEX_URL);
  assert.equal(
    SETTING_DOCUMENTATION_INDEX_URL,
    "https://raw.githubusercontent.com/wiki/ajouatom/openpilot/Settings-Catalog.json",
  );
});
