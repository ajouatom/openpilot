/**
 * Highlights the matched span of a search result.
 *
 * Pure and escape-injected so it can be tested for the thing that matters most
 * here: every part of the text — before, inside and after the match — is
 * escaped, so a result label containing markup can never inject it.
 */

function identityEscape(value) {
  return String(value ?? "");
}

export function highlightSearchText(text, query, options = {}) {
  const escape = typeof options.escape === "function" ? options.escape : identityEscape;
  const markClass = options.markClass || "setting-search-result__mark";
  const raw = String(text ?? "");
  const needle = String(query || "").trim().toLowerCase();
  if (!raw || !needle) return escape(raw);

  const start = raw.toLowerCase().indexOf(needle);
  if (start < 0) return escape(raw);

  const end = start + needle.length;
  return `${escape(raw.slice(0, start))}`
    + `<mark class="${markClass}">${escape(raw.slice(start, end))}</mark>`
    + `${escape(raw.slice(end))}`;
}
