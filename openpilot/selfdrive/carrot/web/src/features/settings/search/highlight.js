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

function findMatchRange(value, needle) {
  const index = value.toLowerCase().indexOf(needle.toLowerCase());
  if (index < 0) return null;
  return { start: index, end: index + needle.length };
}

export function highlightSearchText(text, query, options = {}) {
  const escape = typeof options.escape === "function" ? options.escape : identityEscape;
  const markClass = options.markClass || "setting-search-result__mark";
  const raw = String(text ?? "");
  // Compare in NFC: the catalog text is NFC, while a query can arrive in the
  // decomposed (NFD) form from some IMEs and macOS input.
  const needle = String(query || "").trim().normalize("NFC");
  if (!raw || !needle) return escape(raw);

  const display = raw.normalize("NFC");
  const range = findMatchRange(display, needle);
  if (!range) return escape(raw);

  // When the source text was NFD, highlight its NFC form: the glyphs are
  // identical and the match offsets stay valid.
  const source = display === raw ? raw : display;
  return `${escape(source.slice(0, range.start))}`
    + `<mark class="${markClass}">${escape(source.slice(range.start, range.end))}</mark>`
    + `${escape(source.slice(range.end))}`;
}
