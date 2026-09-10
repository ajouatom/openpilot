import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import test from "node:test";

import {
  resolveSettingChoiceLayout,
} from "../src/features/settings/choice_dialog.js";

const root = path.resolve(import.meta.dirname, "..");
const settings = JSON.parse(fs.readFileSync(path.join(root, "..", "..", "carrot_settings.json"), "utf8"));

function settingItems(value, found = new Map()) {
  if (Array.isArray(value)) {
    value.forEach((item) => settingItems(item, found));
    return found;
  }
  if (!value || typeof value !== "object") return found;
  if (value.name && value.control === "select") found.set(value.name, value);
  Object.values(value).forEach((item) => settingItems(item, found));
  return found;
}

test("semantic settings select choices use the wrapping list in every locale", () => {
  const selects = settingItems(settings);
  const semantic = [...selects.values()].filter((item) => item.options);
  assert.ok(semantic.length > 0);
  for (const item of semantic) {
    for (const locale of ["ko", "en", "zh"]) {
      assert.ok(Array.isArray(item.options[locale]), `${item.name} has ${locale} labels`);
    }
    assert.equal(resolveSettingChoiceLayout(item), "list", item.name);
  }
  assert.equal(resolveSettingChoiceLayout(selects.get("SoundLanguageSetting")), "list");
});

test("raw numeric setting ranges keep the compact value grid", () => {
  const selects = settingItems(settings);
  const numeric = [...selects.values()].filter((item) => !item.options && item.name !== "SoundLanguageSetting");
  assert.ok(numeric.length > 0);
  for (const item of numeric) {
    assert.equal(resolveSettingChoiceLayout(item), "value-grid", item.name);
  }
});

test("a reviewed setting can explicitly choose its presentation", () => {
  assert.equal(resolveSettingChoiceLayout({ choice_layout: "list" }), "list");
  assert.equal(resolveSettingChoiceLayout({ choice_layout: "value-grid" }), "value-grid");
});
