import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import vm from "node:vm";
import { dashcamUploadStats } from "../src/features/logs/upload_summary.js";

const source = await readFile(new URL("../src/features/logs/dashcam.js", import.meta.url), "utf8");
const uploadFunction = source.slice(source.indexOf("async function uploadDashcamSegments("), source.indexOf("async function uploadRecentDashcamSegments("));

async function runUpload(scope, confirm = true, destination = null) {
  const requests = [];
  const summaries = [];
  const context = vm.createContext({
    dashcamUploadActiveJobId: null,
    getRememberedDashcamUploadJob: () => null,
    getUIText: (_key, fallback) => fallback,
    showAppToast: (message) => { throw new Error(message); },
    getJson: async (url) => {
      assert.equal(url, "/api/dashcam/upload/drive");
      return { configured: destination !== null };
    },
    openAppDialog: async (options) => {
      if (options.choices[0].value === "web") {  // asked only when Google Drive is configured
        assert.notEqual(destination, null);
        return destination === "cancel" ? null : destination;
      }
      assert.equal(options.choices[0].value, "default");
      assert.equal(options.choices[0].current, true);
      return scope;
    },
    appConfirm: async () => confirm,
    dashcamUploadConfirmHtml: (stats) => { summaries.push(stats); return "summary"; },
    dashcamUploadStats,
    postJson: async (url, body) => {
      requests.push({ url, ...body });
      if (url.endsWith("/summary")) return { summaries: [{
        segment: "route--0",
        files: [
          { name: "qcamera.ts", size: 10 }, { name: "rlog.zst", size: 20 },
          ...(body.includeAllFiles ? [{ name: "ecamera.hevc", size: 100 }, { name: "fcamera.hevc", size: 200 }] : []),
        ],
      }] };
      return { job_id: "job" };
    },
    rememberDashcamUploadJob() {},
    clearRememberedDashcamUploadJob() {},
    pollDashcamUploadJob: async () => ({ ok: true, uploaded: 1, total: 1, results: [] }),
    isDashcamUploadCanceledError: () => false,
  });
  vm.runInContext(uploadFunction, context);
  await context.uploadDashcamSegments(["route--0"], {
    showProgress: false, showResult: false, showSuccessToast: false,
  });
  return { requests, summaries };
}

for (const scope of ["default", "all"]) {
  test(`upload ${scope} uses the same scope for confirmation and transfer`, async () => {
    const { requests, summaries } = await runUpload(scope);
    assert.equal(requests.length, 2);
    assert.equal(requests[0].includeAllFiles, scope === "all");
    assert.equal(requests[1].includeAllFiles, scope === "all");
    assert.equal(summaries[0].files, scope === "all" ? 4 : 2);
    assert.equal(summaries[0].bytes, scope === "all" ? 330 : 30);
  });
}

test("without Google Drive the upload goes to the web server", async () => {
  const { requests } = await runUpload("default");
  assert.equal(requests[1].target, "web");
});

for (const destination of ["web", "drive"]) {
  test(`configured Google Drive offers a destination: ${destination}`, async () => {
    const { requests } = await runUpload("default", true, destination);
    assert.equal(requests[1].url, "/api/dashcam/upload/start");
    assert.equal(requests[1].target, destination);
  });
}

test("canceling the destination sends no request", async () => {
  assert.equal((await runUpload("default", true, "cancel")).requests.length, 0);
});

test("canceling file selection sends no request", async () => {
  assert.equal((await runUpload(null)).requests.length, 0);
});

test("canceling size confirmation never starts an upload", async () => {
  const { requests } = await runUpload("all", false);
  assert.equal(requests.length, 1);
  assert.equal(requests[0].url, "/api/dashcam/upload/summary");
});
