import assert from "node:assert/strict";
import test from "node:test";

import { parseSettingDocumentationAst } from "../src/features/settings/documentation_ast.js";
import {
  createSettingDocumentationView,
  renderSettingDocumentationAst,
  settingDocumentationPlainText,
} from "../src/features/settings/documentation_renderer.js";

class FakeDomNode {
  constructor(tagName = "", text = "") {
    this.tagName = tagName;
    this.text = text;
    this.children = [];
    this.attributes = new Map();
    this.className = "";
  }

  appendChild(child) {
    this.children.push(child);
    return child;
  }

  setAttribute(name, value) {
    this.attributes.set(name, String(value));
  }
}

const fakeDocument = Object.freeze({
  createElement: (tagName) => new FakeDomNode(tagName),
  createTextNode: (text) => new FakeDomNode("#text", String(text)),
});

function flatten(view, output = []) {
  output.push(view);
  (view.children || []).forEach((child) => flatten(child, output));
  return output;
}

test("documentation AST maps to native component descriptors in source order", () => {
  const markdown = `## Overview

Use **carefully**.

| Value | Meaning |
|---|---|
| 1 | Enabled |

> [!WARNING]
> Check the road first.
`;
  const ast = parseSettingDocumentationAst(markdown);
  const view = createSettingDocumentationView(ast, {
    text: (key, fallback) => (key === "setting_doc_callout_warning" ? "Warning" : fallback),
  });
  const nodes = flatten(view);

  assert.deepEqual(
    nodes.filter((node) => node.kind === "element").map((node) => node.tag),
    ["h2", "p", "strong", "div", "table", "thead", "tr", "th", "th", "tbody", "tr", "td", "td", "aside", "strong", "p"],
  );
  assert.equal(nodes.find((node) => node.tag === "aside").className, "setting-doc-callout setting-doc-callout--warning");
  assert.equal(
    settingDocumentationPlainText(ast),
    "Overview Use carefully . Value Meaning 1 Enabled Check the road first.",
  );
});

test("AST renderer creates native DOM without an HTML string sink", () => {
  const ast = parseSettingDocumentationAst("## Guide\n\n[Open](https://example.com/)");
  const article = renderSettingDocumentationAst(fakeDocument, ast);
  const heading = article.children[0];
  const paragraph = article.children[1];
  const link = paragraph.children[0];

  assert.equal(article.tagName, "article");
  assert.equal(article.className, "setting-doc-markdown setting-doc-ast");
  assert.equal(heading.tagName, "h2");
  assert.equal(link.tagName, "a");
  assert.equal(link.attributes.get("href"), "https://example.com/");
  assert.equal(link.attributes.get("rel"), "noopener noreferrer");
});

test("links and media retain safe accessibility attributes", () => {
  const ast = parseSettingDocumentationAst(
    "[Guide](https://example.com/guide) ![Road preview](https://example.com/road.png)",
  );
  const nodes = flatten(createSettingDocumentationView(ast));
  const link = nodes.find((node) => node.tag === "a");
  const image = nodes.find((node) => node.tag === "img");

  assert.equal(link.attrs.href, "https://example.com/guide");
  assert.equal(link.attrs.rel, "noopener noreferrer");
  assert.equal(image.attrs.alt, "Road preview");
  assert.equal(image.attrs.loading, "lazy");
});

test("details and task lists preserve state without relying on color", () => {
  const ast = [{
    type: "details",
    open: true,
    summary: [{ type: "text", text: "Advanced", children: [] }],
    children: [{
      type: "list",
      ordered: false,
      start: 1,
      items: [
        {
          type: "list_item",
          task: true,
          checked: true,
          children: [{ type: "paragraph", children: [{ type: "text", text: "Ready", children: [] }] }],
        },
        {
          type: "list_item",
          task: true,
          checked: false,
          children: [{ type: "paragraph", children: [{ type: "text", text: "Check", children: [] }] }],
        },
      ],
    }],
  }];
  const nodes = flatten(createSettingDocumentationView(ast));
  const details = nodes.find((node) => node.tag === "details");
  const inputs = nodes.filter((node) => node.tag === "input");

  assert.equal(details.attrs.open, true);
  assert.equal(inputs.length, 2);
  assert.equal(inputs[0].attrs.checked, true);
  assert.equal(inputs[1].attrs.checked, undefined);
  assert.equal(inputs.every((input) => input.attrs.disabled), true);
});
