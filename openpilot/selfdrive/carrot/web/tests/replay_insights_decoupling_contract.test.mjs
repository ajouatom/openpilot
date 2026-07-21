import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

const sourceUrl = new URL("../src/features/replay/insights.js", import.meta.url);

test("replay detail surfaces consume playback time outside the video frame stack", async () => {
  const source = await readFile(sourceUrl, "utf8");

  assert.match(source, /createLatestOnlyRenderScheduler/);
  assert.match(source, /playbackSurfaceCadenceMs:\s*50/);
  assert.match(
    source,
    /playbackSurfaceScheduler\.request\(POLICY\.playbackSurfaceCadenceMs, options\)/,
  );
  const syncTimeStart = source.indexOf("  function syncTime(");
  const syncTimeEnd = source.indexOf("  function syncLabels()", syncTimeStart);
  const syncTimeBody = syncTimeStart >= 0 && syncTimeEnd > syncTimeStart
    ? source.slice(syncTimeStart, syncTimeEnd)
    : "";
  assert.ok(syncTimeBody, "syncTime body should remain directly inspectable");
  assert.doesNotMatch(syncTimeBody, /graphEls\.forEach|sensorTopview\?\.syncTime|syncAdvancedCurrent/);
});

test("replay paints only the currently visible detail surface", async () => {
  const source = await readFile(sourceUrl, "utf8");

  assert.match(source, /state\.open && state\.activeTab === "graphs"/);
  assert.match(source, /state\.open && state\.activeTab === "sensors"/);
  assert.match(source, /state\.open && state\.activeTab === "advanced"/);
  assert.match(source, /playbackSurfaceScheduler\?\.cancel\?\.\(\)/);
  assert.match(source, /renderStatus:\s*\(\)\s*=>\s*playbackSurfaceScheduler\?\.status/);
});
