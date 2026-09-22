import { collectSettingSearchMatches } from "./entries.js";

/* DOM rendering for the overlay search result list. All page-owned helpers
   (text lookup, escaping, highlighting, selection) are injected so this stays
   a view module and the page keeps navigation and history. */

function text(value) {
  return String(value ?? "");
}

export function renderSettingSearchResults(options = {}) {
  const container = options.container || null;
  if (!container) return 0;

  const documentRef = container.ownerDocument || null;
  if (!documentRef) return 0;

  const query = text(options.query).trim();
  const labels = options.labels || {};
  const escape = typeof options.escape === "function" ? options.escape : text;
  const highlight = typeof options.highlight === "function" ? options.highlight : (value) => escape(value);
  const onSelect = typeof options.onSelect === "function" ? options.onSelect : null;

  const { entries: matches, total } = collectSettingSearchMatches(options.entries, {
    query,
    scope: options.scope,
    limit: options.limit,
  });

  container.replaceChildren();
  if (!query || !matches.length) {
    if (!query) return 0;
    const empty = documentRef.createElement("div");
    empty.className = "setting-search-result setting-search-result--empty";
    empty.textContent = text(labels.empty);
    container.appendChild(empty);
    return 0;
  }

  // Tell the user when the result limit hides matches instead of dropping them
  // silently. The label uses the same {shown}/{total} template as the inline
  // search status.
  if (total > matches.length && labels.count) {
    const summary = documentRef.createElement("div");
    summary.className = "setting-search-summary";
    summary.textContent = text(labels.count)
      .replace("{shown}", String(matches.length))
      .replace("{total}", String(total));
    container.appendChild(summary);
  }

  const sections = [
    {
      key: "carrot",
      title: text(labels.sourceCarrot),
      entries: matches.filter((entry) => entry.source === "carrot"),
    },
    {
      key: "profile",
      title: text(labels.sourceProfile),
      entries: matches.filter((entry) => entry.source === "profile"),
    },
  ].filter((section) => section.entries.length);

  for (const section of sections) {
    const sectionEl = documentRef.createElement("div");
    sectionEl.className = "setting-search-section";
    sectionEl.innerHTML = `
      <div class="setting-search-section__title">
        <span>${escape(section.title)}</span>
        <strong>${section.entries.length}</strong>
      </div>
      <div class="setting-search-section__body"></div>
    `;
    container.appendChild(sectionEl);
    const sectionBody = sectionEl.querySelector(".setting-search-section__body");
    if (!sectionBody) continue;

    for (const entry of section.entries) {
      const metaLabel = entry.source === "profile"
        ? `${entry.profileName} / ${entry.contextGroupLabel || entry.groupLabel}`
        : entry.contextGroupLabel || entry.groupLabel;
      const button = documentRef.createElement("button");
      button.type = "button";
      button.className = "setting-search-result";
      button.innerHTML = `
        <div class="setting-search-result__group">${highlight(text(metaLabel), query)}</div>
        <div class="setting-search-result__title">${highlight(text(entry.title || entry.name), query)}</div>
        ${entry.name && entry.name !== entry.title ? `<div class="setting-search-result__name">${highlight(text(entry.name), query)}</div>` : ""}
        ${entry.descr ? `<div class="setting-search-result__descr">${highlight(text(entry.descr), query)}</div>` : ""}
      `;
      button.addEventListener("click", () => {
        if (onSelect) onSelect(entry);
      });
      sectionBody.appendChild(button);
    }
  }

  return matches.length;
}
