import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

const schedulerUrl = new URL("../src/features/telemetry/render_scheduler.js", import.meta.url);
const driveGraphUrl = new URL("../src/features/drive/contents/drive_insights/graph.js", import.meta.url);
const driveForwardUrl = new URL("../src/features/drive/contents/drive_insights/forward.js", import.meta.url);
const replayForwardUrl = new URL("../src/features/replay/forward.js", import.meta.url);

test("auxiliary telemetry yields before painting without changing its source cadence", async () => {
  const source = await readFile(schedulerUrl, "utf8");

  assert.match(source, /requestIdleCallback/);
  assert.match(source, /idleTimeoutMs, 12/);
  assert.match(source, /setTimeoutFn\(yieldThenRun/);
});

test("drive graph caches style tokens outside its hot paint loop", async () => {
  const source = await readFile(driveGraphUrl, "utf8");
  const drawStart = source.indexOf("function drawMetric(");
  const drawEnd = source.indexOf("export function createDriveInsightsGraphRenderer", drawStart);
  const drawBody = source.slice(drawStart, drawEnd);

  assert.ok(drawBody);
  assert.doesNotMatch(drawBody, /getComputedStyle|cssValue/);
  assert.match(source, /card\.lineColor = cssValue/);
});

test("live and replay forward views cache their static grids", async () => {
  const [drive, replay] = await Promise.all([
    readFile(driveForwardUrl, "utf8"),
    readFile(replayForwardUrl, "utf8"),
  ]);

  assert.match(drive, /function prepareBackground\(/);
  assert.match(drive, /drawForwardScene\(context, currentScene, palette, prepareBackground/);
  assert.match(replay, /function drawGridLayer\(/);
  assert.match(replay, /drawGridLayer\(ctx, size, pixelRatio\)/);
});
