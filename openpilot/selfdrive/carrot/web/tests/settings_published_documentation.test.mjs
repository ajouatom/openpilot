import assert from "node:assert/strict";
import test from "node:test";

import {
  parseSettingDocumentationAst,
  SettingDocumentationParseError,
} from "../src/features/settings/documentation_ast.js";
import {
  canonicalJson,
  sha256Hex,
  utf8Bytes,
} from "../src/features/settings/documentation_hash.js";
import {
  createPublishedSettingDocumentationClient,
  extractPublishedSettingMarkdown,
  parsePublishedSettingsIndex,
} from "../src/features/settings/published_documentation.js";

const HASH_A = "a".repeat(64);
const HASH_B = "b".repeat(64);

class MemoryStorage {
  constructor() {
    this.values = new Map();
  }

  getItem(key) {
    return this.values.get(key) ?? null;
  }

  setItem(key, value) {
    this.values.set(key, String(value));
  }
}

function settingBlock(param, title, description, factsHash, semanticHash) {
  return `<!-- CARROT:SETTING:BEGIN ${param} -->
<a id="${param}"></a>

## ${title}

\`${param}\`

${description}

<!-- CARROT:AUTO:BEGIN ${param} -->
<!-- CARROT:AUTO:SOURCE facts="${factsHash}" semantic="${semanticHash}" review="needs_review" -->
<!-- CARROT:SECTION:BEGIN OVERVIEW -->
- **기본값:** \`0\`
<!-- CARROT:SECTION:END OVERVIEW -->
<!-- CARROT:AUTO:END ${param} -->

<!-- CARROT:MANUAL:BEGIN ${param} -->
<!-- CARROT:REVIEW reviewed-for="" -->
<!-- CARROT:MANUAL:END ${param} -->

<!-- CARROT:SETTING:END ${param} -->
`;
}

function fixture() {
  const pageName = "KO-첫-번째.md";
  const page = `<!-- CARROT:GENERATED schema="1" locale="ko" group="SPEED_CAMERA" -->
# 속도 카메라 설정

${settingBlock("ParamA", "첫 번째", "첫 번째 설명", HASH_A, HASH_B)}`.replace(
    'group="SPEED_CAMERA" -->',
    'group="SPEED_CAMERA" param="ParamA" -->',
  );
  const secondPageName = "KO-두-번째.md";
  const secondPage = `<!-- CARROT:GENERATED schema="1" locale="ko" group="SPEED_CAMERA" param="ParamB" -->
# 두 번째

${settingBlock("ParamB", "두 번째", "두 번째 설명", HASH_B, HASH_A)}`;
  const enPageName = "EN-First.md";
  const enPage = `<!-- CARROT:GENERATED schema="1" locale="en" group="SPEED_CAMERA" -->
# Speed camera settings

${settingBlock("ParamA", "First", "First description", HASH_A, HASH_B)}`.replace(
    'group="SPEED_CAMERA" -->',
    'group="SPEED_CAMERA" param="ParamA" -->',
  );
  const secondEnPageName = "EN-Second.md";
  const secondEnPage = `<!-- CARROT:GENERATED schema="1" locale="en" group="SPEED_CAMERA" param="ParamB" -->
# Second

${settingBlock("ParamB", "Second", "Second description", HASH_B, HASH_A)}`;
  const pages = {
    [pageName]: page,
    [secondPageName]: secondPage,
    [enPageName]: enPage,
    [secondEnPageName]: secondEnPage,
  };
  const candidate = {
    schemaVersion: 1,
    catalogCommit: "1".repeat(40),
    wikiCommit: "2".repeat(40),
    generatedAt: "2026-07-23T00:00:00Z",
    locales: ["ko", "en"],
    pages: Object.fromEntries(Object.entries(pages).map(([name, text]) => [
      name,
      { hash: sha256Hex(text), bytes: utf8Bytes(text).byteLength },
    ])),
    settings: {
      ParamA: {
        group: "SPEED_CAMERA",
        semanticHash: HASH_B,
        reviewStatus: "needs_review",
        locales: {
          ko: { page: pageName, anchor: "ParamA", factsHash: HASH_A, reviewStatus: "needs_review" },
          en: { page: enPageName, anchor: "ParamA", factsHash: HASH_A, reviewStatus: "needs_review" },
        },
      },
      ParamB: {
        group: "SPEED_CAMERA",
        semanticHash: HASH_A,
        reviewStatus: "needs_review",
        locales: {
          ko: { page: secondPageName, anchor: "ParamB", factsHash: HASH_B, reviewStatus: "needs_review" },
          en: { page: secondEnPageName, anchor: "ParamB", factsHash: HASH_B, reviewStatus: "needs_review" },
        },
      },
    },
    review: { current: 0, needs_review: 2 },
  };
  const index = { ...candidate, contentHash: sha256Hex(canonicalJson(candidate)) };
  return {
    page,
    pageName,
    secondPage,
    secondPageName,
    enPage,
    enPageName,
    secondEnPage,
    secondEnPageName,
    pages,
    index,
    indexText: JSON.stringify(index),
  };
}

function response(body, status = 200) {
  return {
    ok: status >= 200 && status < 300,
    status,
    headers: { get: () => null },
    text: async () => body,
  };
}

function fixtureBodyForUrl(data, url) {
  if (url.includes("Settings-Catalog.json")) return data.indexText;
  const pageName = decodeURIComponent(new URL(url).pathname.split("/").at(-1));
  return data.pages[pageName];
}

function descriptorFor(index, param) {
  const item = index.settings[param];
  const locale = item.locales.ko;
  return {
    param,
    language: "ko",
    group: item.group,
    semanticHash: item.semanticHash,
    reviewStatus: locale.reviewStatus,
    factsHash: locale.factsHash,
  };
}

test("SHA-256 and canonical JSON match stable vectors", () => {
  assert.equal(
    sha256Hex("abc"),
    "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
  );
  assert.equal(canonicalJson({ z: 1, a: { d: 2, c: "한글" } }), '{"a":{"c":"한글","d":2},"z":1}');
});

test("GFM is normalized to an allowlisted AST and unsafe HTML or URLs are rejected", () => {
  const ast = parseSettingDocumentationAst(`## 제목

| 값 | 의미 |
|---|---|
| 1 | **사용** |

> [!WARNING]
> 주의
`);
  assert.deepEqual(ast.map((node) => node.type), ["heading", "table", "callout"]);
  assert.equal(ast[2].tone, "warning");
  assert.throws(
    () => parseSettingDocumentationAst("[실행](javascript:alert(1))"),
    SettingDocumentationParseError,
  );
  assert.throws(
    () => parseSettingDocumentationAst("<script>alert(1)</script>"),
    SettingDocumentationParseError,
  );
});

test("published index and page marker metadata are verified before extraction", () => {
  const { index, indexText, secondPage } = fixture();
  assert.equal(parsePublishedSettingsIndex(indexText).contentHash, index.contentHash);
  const extracted = extractPublishedSettingMarkdown(secondPage, descriptorFor(index, "ParamB"));
  assert.match(extracted.markdown, /두 번째 설명/);
  assert.doesNotMatch(extracted.markdown, /CARROT:/);
  assert.deepEqual(extracted.sections.map((section) => section.kind), ["overview"]);

  const tampered = JSON.parse(indexText);
  tampered.generatedAt = "2026-07-24T00:00:00Z";
  assert.throws(() => parsePublishedSettingsIndex(JSON.stringify(tampered)), /content hash/);
});

test("published index accepts the full 502-page localized settings catalog", () => {
  const data = fixture();
  const candidate = JSON.parse(data.indexText);
  delete candidate.contentHash;
  for (let index = Object.keys(candidate.pages).length; index < 502; index += 1) {
    candidate.pages[`KO-추가-설정-${index}.md`] = { hash: HASH_A, bytes: 1 };
  }
  const published = {
    ...candidate,
    contentHash: sha256Hex(canonicalJson(candidate)),
  };
  assert.equal(Object.keys(parsePublishedSettingsIndex(JSON.stringify(published)).pages).length, 502);
});

test("concurrent settings load their independent localized Wiki pages", async () => {
  const data = fixture();
  const calls = [];
  const client = createPublishedSettingDocumentationClient({
    storage: new MemoryStorage(),
    fetchImpl: async (url) => {
      calls.push(url);
      return response(fixtureBodyForUrl(data, url));
    },
  });

  const [first, second] = await Promise.all([
    client.load("ParamA", "ko"),
    client.load("ParamB", "ko"),
  ]);
  assert.match(first.markdown, /첫 번째 설명/);
  assert.doesNotMatch(first.markdown, /두 번째 설명/);
  assert.match(second.markdown, /두 번째 설명/);
  assert.equal(calls.filter((url) => url.includes("Settings-Catalog.json")).length, 1);
  assert.equal(calls.filter((url) => decodeURIComponent(url).includes(data.pageName)).length, 1);
  assert.equal(calls.filter((url) => decodeURIComponent(url).includes(data.secondPageName)).length, 1);
  const diagnostics = client.diagnostics();
  assert.equal(diagnostics.loadCount, 2);
  assert.equal(diagnostics.requestCount, 3);
  assert.ok(diagnostics.responseBytes > 0);
  assert.equal(diagnostics.lastContentSource, "published");
  assert.equal(diagnostics.errorRate, 0);
});

test("a page hash mismatch is never exposed without a validated remote snapshot", async () => {
  const data = fixture();
  const client = createPublishedSettingDocumentationClient({
    storage: new MemoryStorage(),
    fetchImpl: async (url) => response(url.includes("Settings-Catalog.json")
      ? data.indexText
      : `${fixtureBodyForUrl(data, url)}\ntampered`),
  });

  const result = await client.load("ParamA", "ko");
  assert.equal(result.available, false);
  assert.equal(result.content_source, "published-unavailable");
  assert.equal(result.markdown, undefined);
});

test("validated pages survive network failure as a bounded last-good cache", async () => {
  const data = fixture();
  const storage = new MemoryStorage();
  const online = createPublishedSettingDocumentationClient({
    storage,
    fetchImpl: async (url) => response(fixtureBodyForUrl(data, url)),
  });
  assert.equal((await online.load("ParamA", "ko")).content_source, "published");

  const offline = createPublishedSettingDocumentationClient({
    storage,
    fetchImpl: async () => {
      throw new Error("offline");
    },
  });
  const cached = await offline.load("ParamA", "ko");
  assert.equal(cached.content_source, "published-cache");
  assert.match(cached.markdown, /첫 번째 설명/);
});

test("one caller can cancel without aborting a shared request needed by another caller", async () => {
  const data = fixture();
  let releaseIndex;
  const indexResponse = new Promise((resolve) => {
    releaseIndex = () => resolve(response(data.indexText));
  });
  const client = createPublishedSettingDocumentationClient({
    storage: new MemoryStorage(),
    fetchImpl: async (url) => (
      url.includes("Settings-Catalog.json") ? indexResponse : response(fixtureBodyForUrl(data, url))
    ),
  });
  const controller = new AbortController();
  const cancelled = client.load("ParamA", "ko", { signal: controller.signal });
  const active = client.load("ParamA", "ko");
  controller.abort();
  releaseIndex();

  await assert.rejects(cancelled, { name: "AbortError" });
  assert.equal((await active).available, true);
});
