/* Pure search-entry filtering for the setting search surfaces.

   `buildSearchEntries()` (derived_model.js) owns the haystack, the chosung
   projection and the ordering; this module only decides which entries survive
   a query and an optional profile scope, so both the inline field and the
   overlay panel share one behaviour instead of re-implementing the filter. */

import { settingSearchKeywords, settingSearchMatches } from "./match.js";

export const DEFAULT_SEARCH_RESULT_LIMIT = 36;

export function isProfileSearchScope(scope) {
  return Boolean(scope) && scope.type === "profile" && Boolean(scope.profileId);
}

export function normalizeSearchScope(scope) {
  return isProfileSearchScope(scope)
    ? Object.freeze({ type: "profile", profileId: String(scope.profileId) })
    : Object.freeze({ type: "all", profileId: "" });
}

// Counts every match while returning at most `limit` of them, so a surface can
// tell the user that results were truncated instead of silently dropping them.
export function collectSettingSearchMatches(entries, options = {}) {
  const limit = Number.isFinite(options.limit) ? Math.max(0, Math.floor(options.limit)) : DEFAULT_SEARCH_RESULT_LIMIT;
  // The shared matcher normalizes the query (NFC, lowercase), requires every
  // space-separated keyword and accepts consonant-only keywords via the
  // entry's chosung projection.
  const keywords = settingSearchKeywords(options.query);
  if (!keywords.length) return { entries: [], total: 0 };

  const profileId = isProfileSearchScope(options.scope) ? String(options.scope.profileId) : null;
  const list = Array.isArray(entries) ? entries : [];
  const matches = [];
  let total = 0;

  for (const entry of list) {
    if (!entry || typeof entry.haystack !== "string"
      || !settingSearchMatches(entry.haystack, entry.chosung, keywords)) continue;
    if (profileId !== null && !(entry.source === "profile" && entry.profileId === profileId)) continue;
    total += 1;
    if (matches.length < limit) matches.push(entry);
  }

  return { entries: matches, total };
}

export function filterSettingSearchEntries(entries, options = {}) {
  return collectSettingSearchMatches(entries, options).entries;
}
