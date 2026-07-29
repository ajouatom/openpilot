import assert from "node:assert/strict";
import test from "node:test";

import {
  createDashcamUploadProgressModel,
  dashcamUploadProgressState,
} from "../src/features/logs/upload_progress.js";
import { normalizeAppProgressState } from "../src/ui/components/dialog/dialog.js";
import { applyAppProgressState } from "../src/ui/components/progress/progress.js";

test("shared progress state preserves explicit indeterminate mode", () => {
  assert.deepEqual(
    normalizeAppProgressState({ mode: "indeterminate", value: 0 }),
    { mode: "indeterminate", value: null },
  );
  assert.deepEqual(
    normalizeAppProgressState({ mode: "determinate", value: 142 }),
    { mode: "determinate", value: 100 },
  );
  assert.deepEqual(
    normalizeAppProgressState(null),
    { mode: "indeterminate", value: null },
  );
});

test("queued and zero-byte upload stages stay explicitly determinate", () => {
  assert.deepEqual(
    dashcamUploadProgressState({ phase: "queued", progress: 0 }),
    { mode: "determinate", value: 0 },
  );
  assert.deepEqual(
    dashcamUploadProgressState({ phase: "uploading", progress: 0, bytes_current: 0, bytes_total: 1024 }),
    { mode: "determinate", value: 0 },
  );
});

test("reported or measured transfer progress becomes determinate", () => {
  assert.deepEqual(
    dashcamUploadProgressState({ phase: "uploading", progress: 27 }),
    { mode: "determinate", value: 27 },
  );
  assert.deepEqual(
    dashcamUploadProgressState({
      phase: "uploading",
      progress: null,
      bytes_current: 256,
      bytes_total: 1024,
    }),
    { mode: "determinate", value: 30.25 },
  );
});

test("completed uploads always end at 100 percent", () => {
  assert.deepEqual(
    dashcamUploadProgressState({ phase: "complete", status: "done", done: true, progress: 0 }),
    { mode: "determinate", value: 100 },
  );
});

test("cancel and failure phases preserve the last reported position", () => {
  const model = createDashcamUploadProgressModel();
  model.update({ id: "job-b", revision: 1, phase: "uploading", progress: 57 });
  assert.deepEqual(
    model.update({ id: "job-b", revision: 2, phase: "canceling", progress: 57 }).progressState,
    { mode: "determinate", value: 57 },
  );
  assert.deepEqual(
    model.update({ id: "job-b", revision: 3, phase: "canceled", status: "canceled", progress: 57 }).progressState,
    { mode: "determinate", value: 57 },
  );

  model.reset("job-c");
  model.update({ id: "job-c", revision: 1, phase: "uploading", progress: 33 });
  assert.deepEqual(
    model.update({ id: "job-c", revision: 2, phase: "failed", status: "failed", progress: 33 }).progressState,
    { mode: "determinate", value: 33 },
  );
});

test("upload progress model rejects stale snapshots and never moves backward", () => {
  const model = createDashcamUploadProgressModel();
  assert.deepEqual(
    model.update({ id: "job-a", revision: 1, phase: "preparing", progress: 4 }).progressState,
    { mode: "determinate", value: 4 },
  );
  assert.equal(
    model.update({ id: "job-a", revision: 0, phase: "uploading", progress: 80 }).accepted,
    false,
  );
  assert.deepEqual(
    model.update({ id: "job-a", revision: 2, phase: "uploading", progress: 2 }).progressState,
    { mode: "determinate", value: 4 },
  );
  assert.deepEqual(
    model.update({ id: "job-a", revision: 3, phase: "notifying", progress: 98 }).progressState,
    { mode: "determinate", value: 98 },
  );
  assert.deepEqual(
    model.update({ id: "job-a", revision: 4, phase: "complete", status: "done" }).progressState,
    { mode: "determinate", value: 100 },
  );
});

test("native progress element receives both value property and accessible state", () => {
  const makeElement = () => ({
    dataset: {},
    attributes: new Map(),
    textContent: "",
    setAttribute(name, value) {
      this.attributes.set(name, String(value));
    },
    removeAttribute(name) {
      this.attributes.delete(name);
    },
  });
  const progress = makeElement();
  const output = makeElement();

  applyAppProgressState(progress, output, { mode: "determinate", value: 42.4 });
  assert.equal(progress.value, 42.4);
  assert.equal(progress.attributes.get("value"), "42.4");
  assert.equal(progress.attributes.get("aria-valuenow"), "42");
  assert.equal(progress.dataset.progressValue, "42.4");
  assert.equal(output.textContent, "42%");

  applyAppProgressState(progress, output, { mode: "indeterminate" });
  assert.equal(progress.attributes.has("value"), false);
  assert.equal(progress.attributes.has("aria-valuenow"), false);
  assert.equal(output.textContent, "…");
});
