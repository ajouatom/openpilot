import assert from "node:assert/strict";
import test from "node:test";

import {
  collectRestoredValues,
  selectProfileApplyValues,
} from "../src/features/settings/profiles/apply_plan.js";

test("only entries marked apply are sent, with the profile's own values", () => {
  const preview = {
    entries: [
      { key: "ApplyModelSpeed", apply: true },
      { key: "CruiseSpeed1", apply: false },
      { key: "TFollowDecelBoost", apply: true },
    ],
  };
  const profileValues = { ApplyModelSpeed: 0, CruiseSpeed1: 30, TFollowDecelBoost: 50, Extra: 9 };

  assert.deepEqual(selectProfileApplyValues(preview, profileValues), {
    ApplyModelSpeed: 0,
    TFollowDecelBoost: 50,
  });
});

test("a key marked apply but missing from the profile is skipped", () => {
  const preview = { entries: [{ key: "Ghost", apply: true }, { key: "A", apply: true }] };
  assert.deepEqual(selectProfileApplyValues(preview, { A: 1 }), { A: 1 });
});

test("selecting apply values tolerates missing or malformed input", () => {
  assert.deepEqual(selectProfileApplyValues(null, { A: 1 }), {});
  assert.deepEqual(selectProfileApplyValues({ entries: [{ key: "A", apply: true }] }, null), {});
  assert.deepEqual(selectProfileApplyValues({}, {}), {});
});

test("a falsy but real value (0) is still sent", () => {
  const preview = { entries: [{ key: "ApplyModelSpeed", apply: true }] };
  assert.deepEqual(selectProfileApplyValues(preview, { ApplyModelSpeed: 0 }), { ApplyModelSpeed: 0 });
});

test("restored values come from the server's re-confirmed apply entries", () => {
  const result = {
    preview: {
      entries: [
        { key: "A", apply: true, value: 2 },
        { key: "B", apply: false, value: 9 },
      ],
    },
    result: { fails: [] },
  };
  assert.deepEqual(collectRestoredValues(result), { A: 2 });
});

test("a failed key is not reported as restored", () => {
  const result = {
    preview: { entries: [{ key: "A", apply: true, value: 2 }, { key: "B", apply: true, value: 3 }] },
    result: { fails: [{ key: "B", err: "boom" }] },
  };
  assert.deepEqual(collectRestoredValues(result), { A: 2 });
});

test("the restored value is the server's stored value, not the requested one", () => {
  // The server clamps; the page must announce what was stored.
  const result = {
    preview: { entries: [{ key: "TFollowDecelBoost", apply: true, value: 100 }] },
    result: { fails: [] },
  };
  assert.deepEqual(collectRestoredValues(result), { TFollowDecelBoost: 100 });
});

test("collecting restored values tolerates missing sections", () => {
  assert.deepEqual(collectRestoredValues(null), {});
  assert.deepEqual(collectRestoredValues({}), {});
  assert.deepEqual(collectRestoredValues({ preview: {}, result: {} }), {});
});

test("a value of 0 survives the restored collection", () => {
  const result = { preview: { entries: [{ key: "A", apply: true, value: 0 }] }, result: {} };
  assert.deepEqual(collectRestoredValues(result), { A: 0 });
});
