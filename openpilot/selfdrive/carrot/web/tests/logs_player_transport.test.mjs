import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import test from "node:test";

const root = path.resolve(import.meta.dirname, "..");
const read = (relativePath) => fs.readFileSync(path.join(root, relativePath), "utf8");

test("log players reuse the persistent replay media transport", () => {
  const runtime = read("src/features/logs/runtime.js");
  const style = read("src/features/logs/style.css");

  assert.match(runtime, /dashcam-player-transport carrot-media-action-group/);
  assert.match(runtime, /CarrotMediaTransport/);
  assert.match(runtime, /createActionButton\("previous"/);
  assert.match(runtime, /createActionButton\("play"/);
  assert.match(runtime, /createActionButton\("next"/);
  assert.match(runtime, /createInputController\?\.\(\{/);
  assert.match(runtime, /aria-keyshortcuts", "Space K"/);
  assert.match(runtime, /onPrevious:\s*\(\)\s*=>\s*previousButton\?\.click\(\)/);
  assert.match(runtime, /onNext:\s*\(\)\s*=>\s*nextButton\?\.click\(\)/);
  assert.match(runtime, /inputController\?\.dispose\?\.\(\)/);
  assert.match(runtime, /hideControls:\s*false/);
  assert.match(runtime, /player\?\.play\s*\?\s*player\.play\(\)\s*:\s*videoEl\.play\(\)/);
  assert.match(runtime, /player\?\.pause/);
  assert.match(style, /\.dashcam-player-frame \.plyr__control--overlaid\s*\{[^}]*display:\s*none\s*!important/s);
  assert.match(style, /\.dashcam-player-toast\s*\{[^}]*display:\s*none/s);
});

test("shared media input matches replay keyboard and horizontal drag controls", () => {
  const transport = read("js/shared/ui/media_transport.js");

  for (const shortcut of [
    "Space",
    "KeyK",
    "ArrowLeft",
    "ArrowRight",
    "KeyJ",
    "KeyL",
    "Home",
    "End",
    "Comma",
    "Period",
    "KeyP",
    "KeyN",
  ]) {
    assert.match(transport, new RegExp(`event\\.code === "${shortcut}"`));
  }
  assert.match(transport, /dragSpanSeconds/);
  assert.match(transport, /setPointerCapture/);
  assert.match(transport, /is-media-scrubbing/);
  assert.match(transport, /event\.cancelable\) event\.preventDefault\(\)/);
});

test("global touch gesture cancellation only runs for cancelable events", () => {
  const navigation = read("js/shared/ui/navigation.js");

  assert.match(navigation, /if \(e\.cancelable\) e\.preventDefault\(\)/);
  assert.match(navigation, /if \(e\.cancelable && now - lastTouchEnd <= 300\) e\.preventDefault\(\)/);
});

test("record toggle does not report a failed follow-up read after a successful write", () => {
  const car = read("js/pages/car.js");

  assert.match(car, /await setParam\("ScreenRecord", next\);[\s\S]*?applyRecordFabState\(next\);/);
  assert.match(car, /const observed = await loadRecordState\(\{ force: true \}\);[\s\S]*?if \(observed === next\) return;/);
  assert.match(car, /await loadRecordState\(\{ force: true \}\);[\s\S]*?catch \{[\s\S]*?applyRecordFabState\(next\);/);
});

test("dashcam and screen-record players expose adjacent-item navigation", () => {
  const dashcam = read("src/features/logs/dashcam.js");
  const runtime = read("src/features/logs/runtime.js");
  const screenrecord = read("src/features/logs/screenrecord.js");

  assert.match(dashcam, /segments:\s*dashcamPlayerSegments/);
  assert.match(runtime, /movePlayerSegment\(-1\)/);
  assert.match(runtime, /movePlayerSegment\(1\)/);
  assert.match(runtime, /segmentSession\.move\(offset\)/);
  assert.match(screenrecord, /onPrevious:\s*previous/);
  assert.match(screenrecord, /onNext:\s*next/);
});

test("dashcam player direct send opens the shared upload result without a success toast", () => {
  const dashcam = read("src/features/logs/dashcam.js");

  assert.match(dashcam, /onSegmentSend:[\s\S]*?showProgress:\s*false,[\s\S]*?showResult:\s*true,[\s\S]*?showSuccessToast:\s*false/);
  assert.match(dashcam, /if \(options\.showSuccessToast !== false\)[\s\S]*?showAppToast\(message/);
  assert.match(dashcam, /if \(options\.showResult !== false\) await showDashcamUploadResult\(result\)/);
});

test("landscape settings keep the shared submenu back control", () => {
  const chrome = read("src/features/settings/styles/item_chrome.css");
  const navigation = read("js/shared/ui/navigation.js");

  assert.doesNotMatch(chrome, /setting-layout-split[^{}]*\.setting-title-backIcon\s*\{[^}]*display:\s*none/s);
  assert.match(navigation, /itemsTitle\.onclick\s*=\s*\(\)\s*=>\s*history\.back\(\)/);
});
