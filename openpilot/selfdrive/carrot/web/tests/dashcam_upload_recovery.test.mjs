import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

const dashcam = await readFile(new URL("../src/features/logs/dashcam.js", import.meta.url), "utf8");

test("a remembered upload is verified before showing the already-running message", () => {
  assert.match(
    dashcam,
    /if \(dashcamUploadActiveJobId\) \{[\s\S]*?upload_already_running[\s\S]*?return;\s*\}\s*if \(getRememberedDashcamUploadJob\(\)\) \{\s*resumeDashcamUploadJobIfNeeded\(\)\.catch\(\(\) => \{\}\);\s*return;/,
  );
  assert.doesNotMatch(dashcam, /const existingJobId = dashcamUploadActiveJobId \|\| getRememberedDashcamUploadJob\(\)/);
});

test("a server-reported running upload bypasses the local active-id resume guard", () => {
  assert.match(dashcam, /async function resumeDashcamUploadJobIfNeeded\(options = \{\}\)/);
  assert.match(dashcam, /const force = options\.force === true;/);
  assert.match(dashcam, /if \(!jobId \|\| \(!force && jobId === dashcamUploadActiveJobId\)\) return null;/);
  assert.match(
    dashcam,
    /rememberDashcamUploadJob\(runningJobId\);[\s\S]*?resumeDashcamUploadJobIfNeeded\(\{ force: true \}\)\.catch\(\(\) => \{\}\);/,
  );
});
