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
  assert.match(rawSource, /cameraTimestampEof:\s*frame\?\.timestampEof/);
  assert.match(rawSource, /clockMappingConfidence:\s*frame\?\.confidence/);
  assert.match(rawSource, /noteReplayPresentedFrame/);
  assert.match(homeSource, /notifyPresentedFrame:[\s\S]*noteReplayPresentedFrame/);
  assert.match(replaySource, /presentAppliedState/);
  assert.match(replaySource, /render:\s*false,[\s\S]*flushHud:\s*true/);
  assert.match(replaySource, /services:\s*Object\.freeze\(\{ \.\.\.\(state\.manifest\?\.services/);
  assert.match(replaySource, /diagnosticPlaybackSnapshot/);
  assert.match(replaySource, /diagnosticPlayback,/);
});

test("replay redraws restored state after a temporal-only seek reset", () => {
  assert.match(
    replaySource,
    /!result\?\.applied && !result\?\.reset && !result\?\.resetTemporal && !presentFrame/,
  );
  assert.match(replaySource, /force:\s*Boolean\(result\.reset \|\| result\.resetTemporal\)/);
  assert.match(replaySource, /resetTemporal:\s*event\?\.type === "seeking"/);
  assert.match(replaySource, /presentFrame:\s*true/);
  assert.doesNotMatch(replaySource, /if \(resetTemporal\) window\.DriveVisionFacade/);
});

test("recorded replay excludes and rejects live compact websocket samples", () => {
  assert.match(rawSource, /function recordedReplayActive\(\)/);
  assert.match(rawSource, /if \(recordedReplayActive\(\)\) return \[\];/);
  assert.match(rawSource, /ws\.onmessage = async[\s\S]*if \(recordedReplayActive\(\)\) return;[\s\S]*await event\.data\.arrayBuffer\(\)[\s\S]*if \(recordedReplayActive\(\)\) return;/);
});

test("AR-only compact services never schedule the existing lane or HUD renderer", () => {
  assert.match(
    rawSource,
    /RAW_OVERLAY_HUD_ONLY_SERVICES\s*=\s*new Set\(\[[\s\S]*"cameraOdometry"[\s\S]*"livePose"[\s\S]*"carrotNavi"[\s\S]*\]\)/,
  );
  assert.match(
    rawSource,
    /if \(applied && options\.render !== false && \(hudDirty \|\| overlayDirty\)\)/,
  );
  assert.match(
    rawSource,
    /if \(applied && options\.render !== false && \(hudReady \|\| overlayDirty\)\)/,
  );
});

test("AR fallback uses the canonical render-request spelling", () => {
  assert.match(arSource, /addEventListener\?\.\("carrot:render-request"/);
  assert.doesNotMatch(arSource, /carrot:renderrequest/);
});
