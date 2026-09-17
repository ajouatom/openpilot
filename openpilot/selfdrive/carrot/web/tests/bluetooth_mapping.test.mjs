import assert from "node:assert/strict";
import test from "node:test";
import { mappingButtons, gestureToken, splitGesture, learnEvents, BUTTON_ACTIONS, REMOTE_ACTIONS } from "../src/features/tools/bluetooth_mapping.js";

test("every native button has a separately selectable one-shot long action", () => {
  for (const button of BUTTON_ACTIONS) {
    assert.ok(REMOTE_ACTIONS.includes(button));
    assert.ok(REMOTE_ACTIONS.includes(`${button}Long`));
  }
  assert.equal(new Set(REMOTE_ACTIONS).size, REMOTE_ACTIONS.length);
  const mapping = { up: 'accelCruiseLong' };
  assert.deepEqual(mappingButtons(mapping), ['up']);
  assert.equal(gestureToken('up', 'single'), 'up');
});

test("existing single mappings and new gestures share one button row", () => {
  assert.deepEqual(mappingButtons({ up: "accelCruise", "up@double": "carrotCruise", "key:115@long": "paddleDecel" }), ["up", "key:115"]);
  assert.equal(gestureToken("key:115", "single"), "key:115");
  assert.equal(gestureToken("key:115", "double"), "key:115@double");
  assert.deepEqual(splitGesture("tap:300:500@long"), ["tap:300:500", "long"]);
});

test("all test events between polls are learned without changing assigned actions", () => {
  const mapping = { "key:115": "accelCruise", "key:115@double": "carrotCruise" };
  assert.ok(learnEvents(mapping, [
    { button: "key:115@double", reason: "test" },
    { button: "key:114@long", reason: "test" },
    { button: "key:30", reason: "test" },
    { button: "key:31", reason: "sent" },
  ]));
  assert.deepEqual(mapping, { "key:115": "accelCruise", "key:115@double": "carrotCruise", "key:114": "none", "key:114@long": "none", "key:30": "none" });
  assert.equal(learnEvents(mapping, [{ button: "key:114@long", reason: "test" }]), false);
});
