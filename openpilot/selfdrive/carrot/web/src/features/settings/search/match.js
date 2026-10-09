/* Query matching shared by the setting search surfaces (inline field and the
   retired overlay panel).

   Text matching is a plain NFC-normalized substring, exactly as before. Two
   Korean-friendly additions ride on the same haystacks:

   - Space-separated keywords are all required (AND), so "감속 조절" finds a
     row whose title spans both words instead of failing on the exact phrase.
   - A keyword made only of Korean consonants (초성, e.g. "ㄱㅅ") also matches
     the precomputed chosung projection of a haystack, so "ㄱㅅ" finds "감속".
     Both the compatibility jamo a Korean IME emits (ㄱ U+3131) and the
     canonical jamo a decomposed (macOS/NFD) query can leave behind (ᄀ U+1100)
     are accepted. */

const CHOSUNG_UNITS = "ㄱㄲㄴㄷㄸㄹㅁㅂㅃㅅㅆㅇㅈㅉㅊㅋㅌㅍㅎ";
const CHOSUNG_JAMO = "ᄀᄁᄂᄃᄄᄅᄆᄇᄈᄉᄊᄋᄌᄍᄎᄏᄐᄑᄒ";
const CHOSUNG_JAMO_TO_UNIT = new Map([...CHOSUNG_JAMO].map((jamo, index) => [jamo, CHOSUNG_UNITS[index]]));
const CHOSUNG_KEYWORD_PATTERN = /^[ㄱ-ㅎ]+$/;
const HANGUL_SYLLABLE_FIRST = 0xac00;
const HANGUL_SYLLABLE_LAST = 0xd7a3;
const HANGUL_SYLLABLE_STRIDE = 588;

function normalizeChosungUnit(cha) {
  return CHOSUNG_JAMO_TO_UNIT.get(cha) || cha;
}

export function normalizeSettingSearchQuery(query) {
  return String(query || "").trim().normalize("NFC").toLowerCase();
}

// Projects text to its Korean initial consonants; every other character
// (Latin, digits, CJK, punctuation, line breaks) passes through unchanged so
// chosung matching can never match non-Hangul text by accident.
export function extractSettingChosung(text) {
  let out = "";
  for (const cha of String(text)) {
    const code = cha.codePointAt(0);
    if (code >= HANGUL_SYLLABLE_FIRST && code <= HANGUL_SYLLABLE_LAST) {
      out += CHOSUNG_UNITS[Math.floor((code - HANGUL_SYLLABLE_FIRST) / HANGUL_SYLLABLE_STRIDE)];
    } else {
      out += normalizeChosungUnit(cha);
    }
  }
  return out;
}

export function settingSearchKeywords(query) {
  const normalized = normalizeSettingSearchQuery(query);
  return normalized ? normalized.split(/\s+/).filter(Boolean) : [];
}

function chosungKeywordUnits(keyword) {
  const units = [...keyword].map(normalizeChosungUnit).join("");
  return CHOSUNG_KEYWORD_PATTERN.test(units) ? units : null;
}

export function settingSearchKeywordMatches(haystack, chosung, keyword) {
  if (typeof haystack !== "string" || !keyword) return false;
  if (haystack.includes(keyword)) return true;
  // Whitespace-free input ("감속조절") still finds "감속 조절".
  if (haystack.replace(/\s+/g, "").includes(keyword)) return true;
  const units = chosungKeywordUnits(keyword);
  return units !== null && typeof chosung === "string" && chosung.includes(units);
}

export function settingSearchMatches(haystack, chosung, keywords) {
  const tokens = Array.isArray(keywords) ? keywords : settingSearchKeywords(keywords);
  if (!tokens.length) return false;
  return tokens.every((token) => settingSearchKeywordMatches(haystack, chosung, token));
}
