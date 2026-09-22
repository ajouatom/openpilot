import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";

const root = new URL("../", import.meta.url);
const html = readFileSync(new URL("index.html", root), "utf8");
const settingJs = readFileSync(new URL("js/pages/setting.js", root), "utf8");

test("the retired overlay search keeps its entry points hidden", () => {
  // The hamburger action ships hidden, and the page gate is the single switch
  // every entry point (FAB, profile card, history restore) goes through.
  assert.match(html, /id="btnSettingFabSearch"[^>]*hidden/);
  assert.match(settingJs, /const SETTING_OVERLAY_SEARCH_ENABLED = false;/);
  assert.match(settingJs, /if \(!SETTING_OVERLAY_SEARCH_ENABLED\) return;/);
});

test("the inline search field carries its own icon and no title/hint", () => {
  assert.match(html, /class="setting-inline-search__icon"/);
  assert.doesNotMatch(html, /settingInlineSearchLabel|settingInlineSearchHint/);
  assert.match(html, /id="settingInlineSearchInput"[^>]*aria-label="설정 찾기"/);
});
