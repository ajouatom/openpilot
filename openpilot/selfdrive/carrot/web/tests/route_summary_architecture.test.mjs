import assert from "node:assert/strict";
import { readFile, readdir } from "node:fs/promises";
import test from "node:test";

const base = new URL("../src/features/logs/route_summary/", import.meta.url);
const files = await readdir(base);
const sources = Object.fromEntries(await Promise.all(files
  .filter((name) => /\.(?:css|js)$/.test(name))
  .map(async (name) => [name, await readFile(new URL(name, base), "utf8")])));
const dashcam = await readFile(new URL("../src/features/logs/dashcam.js", import.meta.url), "utf8");
const logsCss = await readFile(new URL("../src/entries/logs.css", import.meta.url), "utf8");
const build = await readFile(new URL("../build.mjs", import.meta.url), "utf8");
const serverSource = await readFile(new URL("../../server/features/dashcam/summary_sources.py", import.meta.url), "utf8");
const cerealServices = await readFile(new URL("../../../../cereal/services.py", import.meta.url), "utf8");

test("default route summary path is browser-worker based", () => {
  assert.doesNotMatch(dashcam, /\/api\/dashcam\/report\//);
  assert.doesNotMatch(dashcam, /dashcamReportHtml|showDashcamRouteSummary|DASHCAM_REPORT_BAR/);
  assert.match(sources["api.js"], /\/api\/dashcam\/summary-source\//);
  assert.match(sources["worker_client.js"], /new Worker\(ROUTE_SUMMARY_WORKER_URL\)/);
  assert.match(sources["worker.js"], /for \(let index = 0; index < segments\.length; index \+= 1\)/);
  assert.match(sources["worker.js"], /createRouteSummaryProgressEmitter/);
  assert.match(sources["worker.js"], /progress\.update/);
  assert.match(sources["worker.js"], /progress\.flush/);
  assert.ok(serverSource.indexOf("source_qlog(path)") < serverSource.indexOf("source_rlog(path)"));
  for (const service of ["selfdriveState", "carState", "clocks", "onroadEvents", "carParams"]) {
    assert.match(cerealServices, new RegExp(`"${service}"\\s*:\\s*\\(True,\\s*[0-9.]+,\\s*[1-9][0-9]*\\)`));
  }
  assert.match(build, /route_summary_worker\.js/);
});

test("route summary presentation uses Carrot Web tokens without a local palette", () => {
  const presentation = `${sources["tokens.css"]}\n${sources["style.css"]}\n${sources["components.js"]}`;
  assert.doesNotMatch(presentation, /#[0-9a-f]{3,8}\b|rgba?\(/i);
  assert.match(presentation, /var\(--md-primary\)/);
  assert.match(presentation, /var\(--sp-md\)/);
  assert.match(presentation, /var\(--r-md\)/);
  assert.match(presentation, /var\(--fs-body-sm\)/);
  assert.match(sources["components.js"], /style\.setProperty\("--route-summary-weight"/);
  assert.match(sources["components.js"], /setAttribute\("role", "progressbar"\)/);
  assert.match(sources["components.js"], /--route-summary-progress/);
  assert.doesNotMatch(sources["components.js"], /createElement\("progress"\)/);
  assert.match(sources["view.js"], /route-summary__dashboard/);
  assert.match(sources["view.js"], /createCompositionDistribution/);
  assert.match(sources["components.js"], /compositionDistributionSegments/);
  assert.match(sources["components.js"], /route-summary-distribution__manual/);
  assert.match(sources["view.js"], /route-summary-hero__date/);
  assert.match(sources["view.js"], /route-summary-hero__range/);
  assert.match(sources["controller.js"], /getCachedRouteSummary/);
  assert.match(sources["style.css"], /@container route-summary/);
  assert.match(sources["style.css"], /align-content:\s*start/);
  assert.match(sources["style.css"], /route-summary__dashboard[^{]*\{[^}]*align-items:\s*start/s);
  for (const kind of ["auto", "manual", "gas", "brake"]) {
    assert.match(sources["tokens.css"], new RegExp(`--route-summary-${kind}:\\s*var\\(--md-`));
    assert.match(sources["style.css"], new RegExp(`data-kind=["']${kind}["']`));
  }
  assert.match(sources["components.js"], /row\.dataset\.kind\s*=\s*item\.key/);
  assert.match(sources["components.js"], /row\.dataset\.kind\s*=\s*options\.kind/);
  assert.doesNotMatch(sources["components.js"], /route-summary-event__direction/);
  assert.match(sources["style.css"], /route-summary-event__time[^{]*\{[^}]*border-radius:\s*var\(--r-sm\)/s);
  assert.match(sources["tokens.css"], /--route-summary-event-over:\s*var\(--md-warning\)/);
  assert.match(sources["tokens.css"], /--route-summary-event-hard:\s*var\(--md-error\)/);
  assert.doesNotMatch(sources["style.css"], /orientation\s*:/);
  assert.match(sources["controller.js"], /mode: "choice"/);
  assert.match(sources["controller.js"], /cancelLabel: text\("cancel"/);
  assert.doesNotMatch(sources["components.js"], /innerHTML|insertAdjacentHTML/);
  assert.match(logsCss, /route_summary\/tokens\.css/);
  assert.match(logsCss, /route_summary\/style\.css/);
});
