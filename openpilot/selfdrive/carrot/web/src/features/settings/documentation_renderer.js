import { settingDocumentationSafeHref } from "./documentation_ast.js";

const CALLOUT_LABELS = Object.freeze({
  note: Object.freeze(["setting_doc_callout_note", "Note"]),
  tip: Object.freeze(["setting_doc_callout_tip", "Tip"]),
  important: Object.freeze(["setting_doc_callout_important", "Important"]),
  warning: Object.freeze(["setting_doc_callout_warning", "Warning"]),
  caution: Object.freeze(["setting_doc_callout_caution", "Caution"]),
});

function defaultText(_key, fallback) {
  return String(fallback ?? "");
}

function textView(text) {
  return Object.freeze({ kind: "text", text: String(text ?? "") });
}

function elementView(tag, options = {}) {
  return Object.freeze({
    kind: "element",
    tag,
    className: String(options.className || ""),
    attrs: Object.freeze({ ...(options.attrs || {}) }),
    children: Object.freeze([...(options.children || [])]),
  });
}

function fragmentView(children) {
  return Object.freeze({ kind: "fragment", children: Object.freeze(children) });
}

function inlineViews(nodes, options) {
  return (Array.isArray(nodes) ? nodes : []).map((node) => {
    switch (node?.type) {
      case "text":
        return fragmentView([
          ...(node.text ? [textView(node.text)] : []),
          ...inlineViews(node.children, options),
        ]);
      case "strong":
        return elementView("strong", { children: inlineViews(node.children, options) });
      case "em":
        return elementView("em", { children: inlineViews(node.children, options) });
      case "del":
        return elementView("del", { children: inlineViews(node.children, options) });
      case "code":
        return elementView("code", { children: [textView(node.text)] });
      case "break":
        return elementView("br");
      case "link": {
        const href = settingDocumentationSafeHref(node.href);
        if (!href) throw new TypeError("Unsafe documentation link");
        const external = /^https?:/i.test(href);
        return elementView("a", {
          attrs: {
            href,
            ...(node.title ? { title: node.title } : {}),
            ...(external ? { target: "_blank", rel: "noopener noreferrer" } : {}),
          },
          children: inlineViews(node.children, options),
        });
      }
      case "image": {
        const src = settingDocumentationSafeHref(node.href);
        if (!src) throw new TypeError("Unsafe documentation image");
        return elementView("img", {
          attrs: {
            src,
            alt: node.alt,
            loading: "lazy",
            decoding: "async",
            ...(node.title ? { title: node.title } : {}),
          },
        });
      }
      default:
        throw new TypeError(`Unsupported documentation inline node: ${node?.type || "unknown"}`);
    }
  });
}

function blockViews(nodes, options) {
  const text = options.text;
  return (Array.isArray(nodes) ? nodes : []).map((node) => {
    switch (node?.type) {
      case "heading": {
        const depth = Math.max(2, Math.min(6, Number(node.depth) || 2));
        return elementView(`h${depth}`, { children: inlineViews(node.children, options) });
      }
      case "paragraph":
        return elementView("p", { children: inlineViews(node.children, options) });
      case "list": {
        const tag = node.ordered ? "ol" : "ul";
        return elementView(tag, {
          attrs: node.ordered && Number(node.start) !== 1 ? { start: String(node.start) } : {},
          children: (node.items || []).map((item) => {
            const task = item.task
              ? [elementView("input", {
                className: "setting-doc-task",
                attrs: {
                  type: "checkbox",
                  disabled: true,
                  ...(item.checked ? { checked: true } : {}),
                },
              })]
              : [];
            return elementView("li", {
              className: item.task ? "setting-doc-task-item" : "",
              children: [...task, ...blockViews(item.children, options)],
            });
          }),
        });
      }
      case "table": {
        const header = elementView("tr", {
          children: (node.header || []).map((cell, index) => elementView("th", {
            attrs: {
              scope: "col",
              ...(node.align?.[index] ? { "data-align": node.align[index] } : {}),
            },
            children: inlineViews(cell, options),
          })),
        });
        const rows = (node.rows || []).map((row) => elementView("tr", {
          children: row.map((cell, index) => elementView("td", {
            attrs: node.align?.[index] ? { "data-align": node.align[index] } : {},
            children: inlineViews(cell, options),
          })),
        }));
        return elementView("div", {
          className: "setting-doc-table-wrap",
          children: [
            elementView("table", {
              children: [
                elementView("thead", { children: [header] }),
                elementView("tbody", { children: rows }),
              ],
            }),
          ],
        });
      }
      case "callout": {
        const tone = CALLOUT_LABELS[node.tone] ? node.tone : "note";
        const [labelKey, labelFallback] = CALLOUT_LABELS[tone];
        return elementView("aside", {
          className: `setting-doc-callout setting-doc-callout--${tone}`,
          attrs: { role: "note" },
          children: [
            elementView("strong", {
              className: "setting-doc-callout__label",
              children: [textView(text(labelKey, labelFallback))],
            }),
            ...blockViews(node.children, options),
          ],
        });
      }
      case "code_block":
        return elementView("pre", {
          children: [elementView("code", {
            attrs: node.language ? { "data-language": node.language } : {},
            children: [textView(node.text)],
          })],
        });
      case "separator":
        return elementView("hr");
      case "details":
        return elementView("details", {
          attrs: node.open ? { open: true } : {},
          children: [
            elementView("summary", { children: inlineViews(node.summary, options) }),
            ...blockViews(node.children, options),
          ],
        });
      default:
        throw new TypeError(`Unsupported documentation block node: ${node?.type || "unknown"}`);
    }
  });
}

function appendView(document, parent, view) {
  if (view.kind === "text") {
    parent.appendChild(document.createTextNode(view.text));
    return;
  }
  if (view.kind === "fragment") {
    view.children.forEach((child) => appendView(document, parent, child));
    return;
  }
  const element = document.createElement(view.tag);
  if (view.className) element.className = view.className;
  Object.entries(view.attrs).forEach(([name, value]) => {
    if (value === false || value == null) return;
    element.setAttribute(name, value === true ? "" : String(value));
    if (name === "checked") element.checked = true;
    if (name === "disabled") element.disabled = true;
  });
  view.children.forEach((child) => appendView(document, element, child));
  parent.appendChild(element);
}

function collectText(nodes, output) {
  (Array.isArray(nodes) ? nodes : []).forEach((node) => {
    if (Array.isArray(node)) {
      collectText(node, output);
      return;
    }
    if (!node || typeof node !== "object") return;
    if (typeof node.text === "string") output.push(node.text);
    if (typeof node.alt === "string") output.push(node.alt);
    collectText(node.children, output);
    collectText(node.summary, output);
    collectText(node.items, output);
    collectText(node.header, output);
    (node.rows || []).forEach((row) => collectText(row, output));
  });
}

export function createSettingDocumentationView(ast, options = {}) {
  const text = typeof options.text === "function" ? options.text : defaultText;
  return fragmentView(blockViews(ast, { text }));
}

export function renderSettingDocumentationAst(document, ast, options = {}) {
  if (!document?.createElement || !document?.createTextNode) {
    throw new TypeError("A document is required");
  }
  const article = document.createElement("article");
  article.className = "setting-doc-markdown setting-doc-ast";
  appendView(document, article, createSettingDocumentationView(ast, options));
  return article;
}

export function settingDocumentationPlainText(ast) {
  const output = [];
  collectText(ast, output);
  return output.join(" ").replace(/\s+/g, " ").trim();
}
