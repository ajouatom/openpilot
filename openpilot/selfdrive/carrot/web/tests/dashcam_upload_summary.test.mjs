import assert from "node:assert/strict";
import test from "node:test";

import {
  dashcamUploadArtifactKind,
  dashcamUploadStats,
} from "../src/features/logs/upload_summary.js";

test("upload artifacts are classified from server kind or original filename", () => {
  assert.equal(dashcamUploadArtifactKind({ kind: "qcamera", name: "anything" }), "qcamera");
  assert.equal(dashcamUploadArtifactKind({ name: "qcamera.ts" }), "qcamera");
  assert.equal(dashcamUploadArtifactKind({ name: "rlog.zst" }), "rlog");
  assert.equal(dashcamUploadArtifactKind({ name: "qlog.zst" }), "");
});

test("upload stats preserve original segment names and count selected artifacts", () => {
  const stats = dashcamUploadStats([
    {
      segment: "00001da7--8626316d57--0",
      files: [
        { kind: "qcamera", name: "qcamera.ts", size: 100 },
        { kind: "rlog", name: "rlog.zst", size: 40 },
      ],
      totalSize: 140,
    },
    {
      segment: "00001da7--8626316d57--1",
      files: [
        { name: "qcamera.mp4", size: 80 },
        { name: "rlog.bz2", size: 20 },
      ],
      totalSize: 100,
    },
  ]);

  assert.deepEqual(stats.segmentNames, [
    "00001da7--8626316d57--0",
    "00001da7--8626316d57--1",
  ]);
  assert.equal(stats.segments, 2);
  assert.equal(stats.qcamera, 2);
  assert.equal(stats.rlog, 2);
  assert.equal(stats.files, 4);
  assert.equal(stats.bytes, 240);
});
