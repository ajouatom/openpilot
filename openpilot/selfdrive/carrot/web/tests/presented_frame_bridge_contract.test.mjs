import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";

const rawSource = readFileSync(new URL("../js/realtime/vision_raw.js", import.meta.url), "utf8");
const replaySource = readFileSync(new URL("../js/realtime/vision_replay.js", import.meta.url), "utf8");
const homeSource = readFileSync(new URL("../js/realtime/home_drive.js", import.meta.url), "utf8");
const arSource = readFileSync(
  new URL("../src/features/drive/contents/vision/ar/runtime.js", import.meta.url),
  "utf8",
);

test("classic live and replay runtimes share the presented-frame channel", () => {
  assert.match(rawSource, /subscribePresented:\s*subscribePresentedFrame/);
  assert.match(rawSource, /noteReplayPresentedFrame/);
  assert.match(homeSource, /notifyPresentedFrame:[\s\S]*noteReplayPresentedFrame/);
  assert.match(replaySource, /presentAppliedState/);
  assert.match(replaySource, /render:\s*false,[\s\S]*flushHud:\s*true/);
  assert.match(replaySource, /services:\s*Object\.freeze\(\{ \.\.\.\(state\.manifest\?\.services/);
});

test("AR fallback uses the canonical render-request spelling", () => {
  assert.match(arSource, /addEventListener\?\.\("carrot:render-request"/);
  assert.doesNotMatch(arSource, /carrot:renderrequest/);
});
