import { marked } from "marked";

const SAFE_URL_PROTOCOLS = new Set(["http:", "https:"]);
const ALERT_TYPES = new Set(["NOTE", "TIP", "IMPORTANT", "WARNING", "CAUTION"]);
const MAX_MARKDOWN_LENGTH = 60_000;
const MAX_AST_NODES = 2_000;
const MAX_AST_DEPTH = 12;

export class SettingDocumentationParseError extends Error {
  constructor(message) {
    super(message);
    this.name = "SettingDocumentationParseError";
  }
}

function safeHref(value) {
  const href = String(value || "").trim();
  if (href.startsWith("#") && /^#[A-Za-z][A-Za-z0-9_.:-]*$/.test(href)) return href;
  try {
    const parsed = new URL(href);
    return SAFE_URL_PROTOCOLS.has(parsed.protocol) ? parsed.href : "";
  } catch (_) {
    return "";
  }
}

function freezeNode(value) {
  if (Array.isArray(value)) {
    value.forEach(freezeNode);
    return Object.freeze(value);
  }
  if (value && typeof value === "object") {
    Object.values(value).forEach(freezeNode);
    return Object.freeze(value);
  }
  return value;
}

function createBudget() {
  return { nodes: 0 };
}

function useNode(budget, depth) {
  budget.nodes += 1;
  if (budget.nodes > MAX_AST_NODES) {
    throw new SettingDocumentationParseError("Markdown contains too many nodes");
  }
  if (depth > MAX_AST_DEPTH) {
    throw new SettingDocumentationParseError("Markdown nesting is too deep");
  }
}

function normalizeInlineTokens(tokens, budget, depth) {
  if (!Array.isArray(tokens)) return [];
  return tokens.map((token) => {
    useNode(budget, depth);
    switch (token.type) {
      case "text":
      case "escape": {
        const children = normalizeInlineTokens(token.tokens, budget, depth + 1);
        return {
          type: "text",
          text: children.length ? "" : String(token.text || ""),
          children,
        };
      }
      case "strong":
      case "em":
      case "del":
        return {
          type: token.type,
          children: normalizeInlineTokens(token.tokens, budget, depth + 1),
        };
      case "codespan":
        return { type: "code", text: String(token.text || "") };
      case "br":
        return { type: "break" };
      case "link": {
        const href = safeHref(token.href);
        if (!href) throw new SettingDocumentationParseError("Markdown contains an unsafe link");
        return {
          type: "link",
          href,
          title: token.title == null ? "" : String(token.title),
          children: normalizeInlineTokens(token.tokens, budget, depth + 1),
        };
      }
      case "image": {
        const href = safeHref(token.href);
        const alt = String(token.text || "").trim();
        if (!href) throw new SettingDocumentationParseError("Markdown contains an unsafe image URL");
        if (!alt) throw new SettingDocumentationParseError("Markdown image alt text is required");
        return {
          type: "image",
          href,
          alt,
          title: token.title == null ? "" : String(token.title),
        };
      }
      default:
        throw new SettingDocumentationParseError(`Unsupported inline Markdown node: ${token.type}`);
    }
  });
}

function normalizeDetails(token, budget, depth) {
  const source = String(token.raw || token.text || "").trim();
  const match = /^<details(\s+open)?>\s*<summary>([^<]*)<\/summary>\s*([\s\S]*?)\s*<\/details>$/i.exec(source);
  if (!match) {
    throw new SettingDocumentationParseError("Only simple details/summary HTML is supported");
  }
  const summaryTokens = marked.Lexer.lexInline(match[2], { gfm: true });
  const bodyTokens = marked.lexer(match[3], { gfm: true });
  return {
    type: "details",
    open: Boolean(match[1]),
    summary: normalizeInlineTokens(summaryTokens, budget, depth + 1),
    children: normalizeBlockTokens(bodyTokens, budget, depth + 1),
  };
}

function normalizeBlockTokens(tokens, budget, depth) {
  if (!Array.isArray(tokens)) return [];
  const result = [];
  tokens.forEach((token) => {
    if (token.type === "space") return;
    useNode(budget, depth);
    switch (token.type) {
      case "heading":
        if (!Number.isInteger(token.depth) || token.depth < 1 || token.depth > 6) {
          throw new SettingDocumentationParseError("Markdown heading depth is invalid");
        }
        result.push({
          type: "heading",
          depth: token.depth,
          children: normalizeInlineTokens(token.tokens, budget, depth + 1),
        });
        break;
      case "paragraph":
        result.push({
          type: "paragraph",
          children: normalizeInlineTokens(token.tokens, budget, depth + 1),
        });
        break;
      case "text":
        result.push({
          type: "paragraph",
          children: normalizeInlineTokens(token.tokens, budget, depth + 1),
        });
        break;
      case "list":
        result.push({
          type: "list",
          ordered: Boolean(token.ordered),
          start: token.ordered && Number.isFinite(Number(token.start)) ? Number(token.start) : 1,
          items: (token.items || []).map((item) => {
            useNode(budget, depth + 1);
            return {
              type: "list_item",
              task: Boolean(item.task),
              checked: item.checked === true,
              children: normalizeBlockTokens(item.tokens, budget, depth + 2),
            };
          }),
        });
        break;
      case "table": {
        const width = Array.isArray(token.header) ? token.header.length : 0;
        if (width < 1 || width > 3) {
          throw new SettingDocumentationParseError("Markdown tables must contain one to three columns");
        }
        if (!Array.isArray(token.rows) || token.rows.length > 100) {
          throw new SettingDocumentationParseError("Markdown table contains too many rows");
        }
        result.push({
          type: "table",
          align: (token.align || []).map((value) => value || ""),
          header: token.header.map((cell) => normalizeInlineTokens(cell.tokens, budget, depth + 1)),
          rows: token.rows.map((row) => {
            if (!Array.isArray(row) || row.length !== width) {
              throw new SettingDocumentationParseError("Markdown table rows must have a consistent width");
            }
            return row.map((cell) => normalizeInlineTokens(cell.tokens, budget, depth + 1));
          }),
        });
        break;
      }
      case "blockquote": {
        const alertMatch = /^\[!(NOTE|TIP|IMPORTANT|WARNING|CAUTION)\](?:\n|$)/.exec(String(token.text || ""));
        const alert = alertMatch && ALERT_TYPES.has(alertMatch[1]) ? alertMatch[1].toLowerCase() : "";
        let children = normalizeBlockTokens(token.tokens, budget, depth + 1);
        if (alert && children[0]?.type === "paragraph") {
          const first = children[0];
          const firstText = first.children?.[0];
          if (firstText?.type === "text") {
            firstText.text = firstText.text.replace(/^\[![A-Z]+\]\s*/, "");
            if (!firstText.text && !firstText.children?.length) {
              first.children.shift();
            }
          }
          if (!first.children.length) children = children.slice(1);
        }
        result.push({ type: "callout", tone: alert || "note", children });
        break;
      }
      case "code":
        result.push({
          type: "code_block",
          language: String(token.lang || "").slice(0, 32),
          text: String(token.text || ""),
        });
        break;
      case "hr":
        result.push({ type: "separator" });
        break;
      case "html":
        result.push(normalizeDetails(token, budget, depth));
        break;
      default:
        throw new SettingDocumentationParseError(`Unsupported block Markdown node: ${token.type}`);
    }
  });
  return result;
}

export function parseSettingDocumentationAst(markdown) {
  const source = String(markdown || "").replaceAll("\r\n", "\n");
  if (source.length > MAX_MARKDOWN_LENGTH) {
    throw new SettingDocumentationParseError("Markdown is too large");
  }
  const budget = createBudget();
  const tokens = marked.lexer(source, { gfm: true, breaks: false });
  return freezeNode(normalizeBlockTokens(tokens, budget, 0));
}

export function settingDocumentationSafeHref(value) {
  return safeHref(value);
}
