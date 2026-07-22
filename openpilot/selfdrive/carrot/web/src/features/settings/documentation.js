const DOC_LANGUAGES = new Set(["ko", "en", "zh"]);

function escapeHtml(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#39;");
}

function safeLinkHref(value) {
  const href = String(value || "").trim();
  if (href.startsWith("#")) return href;
  try {
    const parsed = new URL(href);
    return parsed.protocol === "http:" || parsed.protocol === "https:" ? parsed.href : "";
  } catch (_) {
    return "";
  }
}

function renderInline(value) {
  const replacements = [];
  const remember = (html) => {
    const token = `\u0000CARROT_DOC_${replacements.length}\u0000`;
    replacements.push(html);
    return token;
  };

  let source = String(value ?? "");
  source = source.replace(/`([^`\n]+)`/g, (_match, code) => remember(`<code>${escapeHtml(code)}</code>`));
  source = source.replace(/!\[([^\]]*)\]\([^)]+\)/g, (_match, alt) => remember(escapeHtml(alt)));
  source = source.replace(/\[([^\]]+)\]\(([^)]+)\)/g, (_match, label, rawHref) => {
    const href = safeLinkHref(rawHref);
    const safeLabel = escapeHtml(label);
    if (!href) return remember(safeLabel);
    const external = href.startsWith("http://") || href.startsWith("https://");
    const attributes = external ? ' target="_blank" rel="noopener noreferrer"' : "";
    return remember(`<a href="${escapeHtml(href)}"${attributes}>${safeLabel}</a>`);
  });

  let html = escapeHtml(source);
  html = html.replace(/\*\*([^*\n]+)\*\*/g, "<strong>$1</strong>");
  html = html.replace(/(^|[^*])\*([^*\n]+)\*/g, "$1<em>$2</em>");
  replacements.forEach((replacement, index) => {
    html = html.replaceAll(`\u0000CARROT_DOC_${index}\u0000`, replacement);
  });
  return html;
}

function tableCells(line) {
  let value = String(line || "").trim();
  if (value.startsWith("|")) value = value.slice(1);
  if (value.endsWith("|")) value = value.slice(0, -1);
  return value.split("|").map((cell) => cell.trim());
}

function isTableDelimiter(line) {
  const cells = tableCells(line);
  return cells.length > 0 && cells.every((cell) => /^:?-{3,}:?$/.test(cell));
}

function renderTable(lines, start) {
  const headings = tableCells(lines[start]);
  const rows = [];
  let index = start + 2;
  while (index < lines.length && lines[index].includes("|") && lines[index].trim()) {
    rows.push(tableCells(lines[index]));
    index += 1;
  }
  const width = headings.length;
  const headHtml = headings.map((cell) => `<th>${renderInline(cell)}</th>`).join("");
  const bodyHtml = rows.map((row) => {
    const cells = Array.from({ length: width }, (_unused, cellIndex) => row[cellIndex] || "");
    return `<tr>${cells.map((cell) => `<td>${renderInline(cell)}</td>`).join("")}</tr>`;
  }).join("");
  return {
    html: `<div class="setting-doc-table-wrap"><table><thead><tr>${headHtml}</tr></thead><tbody>${bodyHtml}</tbody></table></div>`,
    next: index,
  };
}

function isListLine(line) {
  return /^\s*[-+*]\s+\S/.test(line) || /^\s*\d+[.)]\s+\S/.test(line);
}

function isBlockStart(lines, index) {
  const line = lines[index] || "";
  const trimmed = line.trim();
  if (!trimmed) return true;
  if (/^#{1,6}\s+/.test(trimmed) || /^```/.test(trimmed) || /^>/.test(trimmed)) return true;
  if (/^(-{3,}|\*{3,}|_{3,})$/.test(trimmed) || isListLine(line) || /^( {4}|\t)/.test(line)) return true;
  return index + 1 < lines.length && line.includes("|") && isTableDelimiter(lines[index + 1]);
}

function renderQuote(lines, start) {
  const quoteLines = [];
  let index = start;
  while (index < lines.length && /^\s*>/.test(lines[index])) {
    quoteLines.push(lines[index].replace(/^\s*>\s?/, ""));
    index += 1;
  }
  let tone = "note";
  const marker = /^\[!(NOTE|TIP|IMPORTANT|WARNING|CAUTION)\]\s*$/i.exec(quoteLines[0] || "");
  if (marker) {
    tone = marker[1].toLowerCase();
    quoteLines.shift();
  }
  const body = renderSettingDocMarkdown(quoteLines.join("\n"));
  return {
    html: `<aside class="setting-doc-callout setting-doc-callout--${tone}">${body}</aside>`,
    next: index,
  };
}

export function normalizeSettingDocLanguage(value) {
  const language = String(value || "").trim().toLowerCase();
  return DOC_LANGUAGES.has(language) ? language : "en";
}

export function renderSettingDocMarkdown(markdown) {
  const lines = String(markdown || "").replaceAll("\r\n", "\n").split("\n");
  const output = [];
  let index = 0;

  while (index < lines.length) {
    const line = lines[index];
    const trimmed = line.trim();
    if (!trimmed) {
      index += 1;
      continue;
    }

    const fence = /^```([^\s`]*)\s*$/.exec(trimmed);
    if (fence) {
      const code = [];
      index += 1;
      while (index < lines.length && !/^```\s*$/.test(lines[index].trim())) {
        code.push(lines[index]);
        index += 1;
      }
      if (index < lines.length) index += 1;
      const languageClass = fence[1] ? ` class="language-${escapeHtml(fence[1])}"` : "";
      output.push(`<pre><code${languageClass}>${escapeHtml(code.join("\n"))}</code></pre>`);
      continue;
    }

    const heading = /^(#{1,6})\s+(.+)$/.exec(trimmed);
    if (heading) {
      const level = Math.min(6, Math.max(2, heading[1].length));
      output.push(`<h${level}>${renderInline(heading[2])}</h${level}>`);
      index += 1;
      continue;
    }

    if (/^\s*>/.test(line)) {
      const rendered = renderQuote(lines, index);
      output.push(rendered.html);
      index = rendered.next;
      continue;
    }

    if (index + 1 < lines.length && line.includes("|") && isTableDelimiter(lines[index + 1])) {
      const rendered = renderTable(lines, index);
      output.push(rendered.html);
      index = rendered.next;
      continue;
    }

    if (isListLine(line)) {
      const ordered = /^\s*\d+[.)]\s+/.test(line);
      const tag = ordered ? "ol" : "ul";
      const items = [];
      while (index < lines.length) {
        const pattern = ordered ? /^\s*\d+[.)]\s+(.+)$/ : /^\s*[-+*]\s+(.+)$/;
        const item = pattern.exec(lines[index]);
        if (!item) break;
        items.push(`<li>${renderInline(item[1])}</li>`);
        index += 1;
      }
      output.push(`<${tag}>${items.join("")}</${tag}>`);
      continue;
    }

    if (/^( {4}|\t)/.test(line)) {
      const code = [];
      while (index < lines.length && (/^( {4}|\t)/.test(lines[index]) || !lines[index].trim())) {
        code.push(lines[index].replace(/^( {4}|\t)/, ""));
        index += 1;
      }
      output.push(`<pre><code>${escapeHtml(code.join("\n").trimEnd())}</code></pre>`);
      continue;
    }

    if (/^(-{3,}|\*{3,}|_{3,})$/.test(trimmed)) {
      output.push("<hr>");
      index += 1;
      continue;
    }

    const paragraph = [trimmed];
    index += 1;
    while (index < lines.length && !isBlockStart(lines, index)) {
      paragraph.push(lines[index].trim());
      index += 1;
    }
    output.push(`<p>${renderInline(paragraph.join(" "))}</p>`);
  }

  return output.join("\n");
}

export function createSettingDocumentationClient(options = {}) {
  const fetchImpl = options.fetchImpl || globalThis.fetch;
  const cache = new Map();

  async function request(name, language) {
    if (typeof fetchImpl !== "function") throw new Error("Setting documentation fetch is unavailable");
    const response = await fetchImpl(
      `/api/settings/doc?name=${encodeURIComponent(name)}&lang=${encodeURIComponent(language)}`,
      { cache: "no-cache" },
    );
    if (response.status === 404) return Object.freeze({ available: false, name, language_requested: language });
    const payload = await response.json().catch(() => ({}));
    if (!response.ok || payload.ok === false) {
      throw new Error(payload.error || `HTTP ${response.status}`);
    }
    return Object.freeze({ ...payload, available: payload.available !== false });
  }

  function load(name, language) {
    const cleanName = String(name || "").trim();
    const cleanLanguage = normalizeSettingDocLanguage(language);
    if (!cleanName) return Promise.resolve(Object.freeze({ available: false, name: "" }));
    const key = `${cleanLanguage}:${cleanName}`;
    if (cache.has(key)) return cache.get(key);
    const pending = request(cleanName, cleanLanguage).catch((error) => {
      cache.delete(key);
      throw error;
    });
    cache.set(key, pending);
    return pending;
  }

  return Object.freeze({ load, clear: () => cache.clear() });
}
