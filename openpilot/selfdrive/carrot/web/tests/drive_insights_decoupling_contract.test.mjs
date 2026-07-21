import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

const rawPath = new URL("../js/realtime/vision_raw.js", import.meta.url);
const runtimePath = new URL("../src/features/drive/contents/drive_insights/runtime.js", import.meta.url);

test("compact batches publish once and Drive Insights paints on its own scheduler", async () => {
  const [raw, runtime] = await Promise.all([
    readFile(rawPath, "utf8"),
    readFile(runtimePath, "utf8"),
  ]);

  assert.match(raw, /notifyProvider:\s*false/);
  assert.match(raw, /noteServicesReceived\(appliedServices\)/);
  assert.match(raw, /applyCompactFrames\(frames,\s*\{\s*reason:\s*"compact websocket batch"\s*\}\)/);
  assert.match(runtime, /subscribeUpdates\(\(\)\s*=>\s*scheduleLiveUpdate\(\)\)/);
  assert.match(runtime, /renderScheduler\.request\(viewCadenceMs\(\),\s*scheduleOptions\)/);
  assert.doesNotMatch(runtime, /subscribe\?\.\(\(snapshot\)\s*=>\s*ingestLive\(snapshot\)\)/);
});
