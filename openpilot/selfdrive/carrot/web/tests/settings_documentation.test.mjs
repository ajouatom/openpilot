import assert from "node:assert/strict";
import test from "node:test";

import {
  createSettingDocumentationClient,
  normalizeSettingDocLanguage,
  renderSettingDocMarkdown,
} from "../src/features/settings/documentation.js";


test("documentation languages preserve ko/en/zh and otherwise fall back to English", () => {
  assert.equal(normalizeSettingDocLanguage("ko"), "ko");
  assert.equal(normalizeSettingDocLanguage("ZH"), "zh");
  assert.equal(normalizeSettingDocLanguage("../../ko"), "en");
});

test("restricted Markdown renderer supports the guide structures", () => {
  const html = renderSettingDocMarkdown(`## 제목

설명 **강조**와 \`ParamName\`.

| 값 | 의미 |
|---:|---|
| 1 | 사용 |

- 첫째
- 둘째

> [!WARNING]
> 주의하세요.
`);

  assert.match(html, /<h2>제목<\/h2>/);
  assert.match(html, /<strong>강조<\/strong>/);
  assert.match(html, /<code>ParamName<\/code>/);
  assert.match(html, /setting-doc-table-wrap/);
  assert.match(html, /<th>값<\/th>/);
  assert.match(html, /<ul><li>첫째<\/li><li>둘째<\/li><\/ul>/);
  assert.match(html, /setting-doc-callout--warning/);
});

test("restricted Markdown renderer escapes raw HTML and rejects unsafe links", () => {
  const html = renderSettingDocMarkdown(`문장 <img src=x onerror=alert(1)>

[실행](javascript:alert(1)) [안전](https://example.com/guide)
`);

  assert.doesNotMatch(html, /<img/i);
  assert.match(html, /&lt;img src=x onerror=alert\(1\)&gt;/);
  assert.doesNotMatch(html, /javascript:/i);
  assert.match(html, /href="https:\/\/example\.com\/guide"/);
  assert.match(html, /rel="noopener noreferrer"/);
});

test("documentation client caches by language and setting name", async () => {
  const calls = [];
  const client = createSettingDocumentationClient({
    fetchImpl: async (url) => {
      calls.push(url);
      return {
        ok: true,
        status: 200,
        json: async () => ({ ok: true, available: true, markdown: "## Guide" }),
      };
    },
  });

  const first = client.load("AutoEngage", "ko");
  const second = client.load("AutoEngage", "ko");
  assert.equal(first, second);
  assert.equal((await first).markdown, "## Guide");
  assert.equal(calls.length, 1);
  assert.match(calls[0], /name=AutoEngage&lang=ko$/);

  await client.load("AutoEngage", "zh");
  assert.equal(calls.length, 2);
});

test("documentation client treats a 404 as an unavailable optional guide", async () => {
  const client = createSettingDocumentationClient({
    fetchImpl: async () => ({ ok: false, status: 404, json: async () => ({}) }),
  });
  const payload = await client.load("Unknown", "en");
  assert.equal(payload.available, false);
});
