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

test("dashcam player direct send shows progress and the shared result without a success toast", () => {
  const dashcam = read("src/features/logs/dashcam.js");
  const runtime = read("src/features/logs/runtime.js");

  assert.match(dashcam, /onSegmentSend:[\s\S]*?showProgress:\s*true,[\s\S]*?showResult:\s*true,[\s\S]*?showSuccessToast:\s*false/);
  assert.doesNotMatch(dashcam, /onSegmentSend:[\s\S]*?showProgress:\s*false/);
  assert.match(runtime, /dashcam-player-send[\s\S]*?pauseCurrentPlayback\(\);[\s\S]*?runTopAction/);
  assert.match(dashcam, /if \(options\.showSuccessToast !== false\)[\s\S]*?showAppToast\(message/);
  assert.match(dashcam, /if \(options\.showResult !== false\) await showDashcamUploadResult\(result\)/);
});

test("dashcam upload confirmation stays compact and uses an explicit send action", () => {
  const dashcam = read("src/features/logs/dashcam.js");
  const dialog = read("src/ui/components/dialog/dialog.js");
  const dialogStyle = read("src/ui/components/dialog/style.css");

  assert.match(dashcam, /messageHtml:\s*dashcamUploadConfirmHtml\(uploadStats\)/);
  assert.match(dashcam, /upload_segment_count/);
  assert.match(dashcam, /upload_kind_count/);
  assert.match(dashcam, /confirmLabel:\s*getUIText\("upload_send",\s*"Send"\)/);
  assert.match(dashcam, /app-dialog__uploadBrief/);
  assert.doesNotMatch(
    dashcam.match(/function dashcamUploadConfirmHtml[\s\S]*?\n}\n/)?.[0] || "",
    /app-dialog__metaCodeList|upload_original_files_confirm/,
  );
  assert.doesNotMatch(dashcam, /getUIText\("upload_data_warning"/);
  assert.match(dialog, /messageHtml:\s*options\.messageHtml/);
  assert.match(dialog, /html:\s*options\.html/);
  assert.match(dialogStyle, /\.app-dialog__uploadBrief\s*\{[^}]*grid-template-columns:\s*minmax\(0,\s*1fr\)\s*auto/s);
  assert.match(dialogStyle, /\.app-dialog__uploadBriefAmount\s*\{[^}]*white-space:\s*nowrap/s);
  assert.doesNotMatch(dialogStyle, /\.app-dialog__uploadBrief\s*\{[^}]*background:/s);
  assert.doesNotMatch(dialogStyle, /\.app-dialog__metaResult\s*\{[^}]*border-radius:/s);
  assert.doesNotMatch(dialogStyle, /\.app-dialog__metaResult(List)?\s*\{[^}]*border(?:-top|-bottom)?:/s);
  assert.doesNotMatch(dialogStyle, /\.app-dialog__metaSummary\s*\{[^}]*border-top:/s);
  assert.doesNotMatch(
    dashcam.match(/function dashcamUploadResultHtml[\s\S]*?\n}\n/)?.[0] || "",
    /app-dialog__metaGroup|app-dialog__metaLabel/,
  );
  assert.match(dashcam, /onSegmentSend:\s*\(target\)\s*=>\s*uploadDashcamSegments\(\[target\],\s*\{[\s\S]*?showProgress:\s*true/);
});

test("dashcam upload progress and result reuse the shared dialog components", () => {
  const dashcam = read("src/features/logs/dashcam.js");
  const logsStyle = read("src/features/logs/style.css");
  const dialog = read("src/ui/components/dialog/dialog.js");
  const dialogFacade = read("src/ui/components/dialog/facade.js");
  const dialogStyle = read("src/ui/components/dialog/style.css");
  const progressComponent = read("src/ui/components/progress/progress.js");
  const progressStyle = read("src/ui/components/progress/style.css");
  const uploadProgress = read("src/features/logs/upload_progress.js");

  assert.match(dashcam, /openAppProgressDialog\(\{/);
  assert.match(dashcam, /function dashcamUploadProgressView/);
  assert.match(dashcam, /dashcamUploadProgressState\(snapshot\)/);
  assert.match(dashcam, /DASHCAM_UPLOAD_POLL_INTERVAL_MS\s*=\s*160/);
  assert.match(dashcam, /DASHCAM_UPLOAD_PROGRESS_MIN_VISIBLE_MS\s*=\s*800/);
  assert.match(dashcam, /await progress\.settle\(\)/);
  assert.match(dashcam, /createDashcamUploadProgressModel\(\)/);
  assert.match(dashcam, /bytes_per_second/);
  assert.match(dashcam, /formatLogBytes\(bytesPerSecond\).*\/s/);
  assert.match(uploadProgress, /mode:\s*"indeterminate"/);
  assert.match(uploadProgress, /Math\.min\(bytesCurrent,\s*bytesTotal\)\s*\/\s*bytesTotal/);
  assert.match(dashcam, /DASHCAM_UPLOAD_RESULT_VISIBLE_LIMIT\s*=\s*5/);
  assert.match(dashcam, /progress\.update\(snapshot,\s*totalFallback\)/);
  assert.doesNotMatch(dashcam, /snapshot\.message\s*\|\|/);
  assert.doesNotMatch(dashcam, /result\.message\s*\|\|\s*getUIText\("upload_complete_count"/);
  assert.match(dashcam, /app-dialog__metaResultList/);
  assert.doesNotMatch(dashcam, /dashcam-share-card|dashcam-upload-progress/);
  assert.doesNotMatch(logsStyle, /dashcam-share-card|dashcam-upload-progress/);
  assert.match(dialog, /function openAppProgressDialog/);
  assert.match(dialog, /closeOnCancel:\s*false/);
  assert.match(dialog, /state\.closeOnCancel && activeDialog === state/);
  assert.match(dialog, /createAppProgressView\(documentRoot, options\)/);
  assert.match(dialog, /appDialogBody\.replaceChildren\(progressView\.element\)/);
  assert.match(progressComponent, /function normalizeAppProgressState/);
  assert.match(progressComponent, /progress\.value = state\.value/);
  assert.match(progressComponent, /progress\.setAttribute\("value", String\(state\.value\)\)/);
  assert.match(progressComponent, /output\.textContent = `\$\{rounded\}%`/);
  assert.match(dialogFacade, /openAppProgressDialog/);
  assert.match(dialogStyle, /\.app-dialog--progress \.app-dialog__body/);
  assert.match(progressStyle, /\.app-progress__track\s*\{[^}]*width:\s*100%/s);
  assert.match(progressStyle, /\.app-progress__track::\-webkit-progress-value/);
  assert.match(progressStyle, /\.app-progress__track:indeterminate/);
  assert.match(progressStyle, /transition:\s*width var\(--motion-medium\) var\(--ease-linear\)/);
  assert.match(progressStyle, /@media \(prefers-reduced-motion:\s*reduce\)/);
});

test("landscape settings keep the shared submenu back control", () => {
  const chrome = read("src/features/settings/styles/item_chrome.css");
  const navigation = read("js/shared/ui/navigation.js");

  assert.doesNotMatch(chrome, /setting-layout-split[^{}]*\.setting-title-backIcon\s*\{[^}]*display:\s*none/s);
  assert.match(navigation, /itemsTitle\.onclick\s*=\s*\(\)\s*=>\s*history\.back\(\)/);
});
