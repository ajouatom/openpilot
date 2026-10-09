import assert from "node:assert/strict";
import test from "node:test";
import { readFileSync } from "node:fs";
import vm from "node:vm";

import { CarrotEgpuModel } from "../src/features/tools/egpu_model.js";

test("delivery failures explain automatic recovery and installed files do not imply active inference", (t) => {
  const saved = globalThis.document;
  t.after(() => { globalThis.document = saved; });
  const elements = new Map();
  globalThis.document = { getElementById(id) {
    if (!elements.has(id)) elements.set(id, { dataset: {}, style: {}, classList: { toggle() {} } });
    return elements.get(id);
  } };
  CarrotEgpuModel.render({ available: true, state: "waiting_for_network", error_code: "dns",
    detail: "Temporary failure in name resolution", can_restart: false });
  assert.match(elements.get("egpuModelState").textContent, /automatic retry/);
  assert.match(elements.get("egpuModelDetail").textContent, /resolve.*retry is automatic/);
  assert.doesNotMatch(elements.get("egpuModelDetail").textContent, /Temporary failure/);
  assert.equal(elements.get("egpuModelRecovery").hidden, true);
  assert.equal(elements.get("btnEgpuCompileRestart").hidden, true);
  CarrotEgpuModel.render({ available: true, state: "installed", compiled: true, active: false });
  assert.match(elements.get("egpuModelState").textContent, /next start/);
  assert.doesNotMatch(elements.get("egpuModelState").textContent, /running/);
  assert.equal(elements.get("btnEgpuCompileRestart").hidden, true);
  CarrotEgpuModel.render({ available: true, state: "compiled", compiled: true, active: true });
  assert.equal(elements.get("egpuModelState").textContent, "eGPU running");
  CarrotEgpuModel.render({ available: true, state: "error", error_code: "runtime", active: false });
  assert.match(elements.get("egpuModelDetail").textContent, /internal model is selected/);
  for (const [error_code, expected] of [["pcie", /connection could not start/], ["timeout", /too long to respond/], ["usb", /Communication.*interrupted/]]) {
    CarrotEgpuModel.render({ available: true, state: "error", error_code, active: false, compiled: true,
      detail: "Last worker stage: load_model" });
    assert.match(elements.get("egpuModelDetail").textContent, expected);
    assert.match(elements.get("egpuModelDetail").textContent, /internal model is selected/);
    assert.doesNotMatch(elements.get("egpuModelDetail").textContent, /load_model|worker|contains the cause/);
    assert.equal(elements.get("egpuModelState").textContent, "eGPU unavailable");
    assert.equal(elements.get("egpuModelRecovery").hidden, false);
    assert.match(elements.get("egpuModelRecovery").textContent, /parked.*User \/ System.*Reboot once.*screenshot/);
    assert.equal(elements.get("btnEgpuCompileRestart").hidden, true);
  }
  CarrotEgpuModel.render({ available: true, state: "compiled", compiled: true, active: true });
  assert.equal(elements.get("egpuModelRecovery").hidden, true);
  assert.equal(elements.get("egpuModelRecovery").textContent, "");
  CarrotEgpuModel.render({ available: true, state: "error", error_code: "future_code", detail: "internal stack trace" });
  assert.match(elements.get("egpuModelDetail").textContent, /screenshot/);
  assert.doesNotMatch(elements.get("egpuModelDetail").textContent, /future_code|stack trace/);
});

test("Korean timeout guidance names the actual reboot menu without exposing worker details", (t) => {
  const savedDocument = globalThis.document;
  const savedTranslator = globalThis.getUIText;
  t.after(() => {
    globalThis.document = savedDocument;
    if (savedTranslator === undefined) delete globalThis.getUIText;
    else globalThis.getUIText = savedTranslator;
  });
  let strings;
  vm.runInNewContext(readFileSync(new URL("../js/translations/ko.js", import.meta.url), "utf8"), {
    window: { CarrotTranslations: { register(_language, translation) { strings = translation.strings; } } },
  });
  const elements = new Map();
  globalThis.document = { getElementById(id) {
    if (!elements.has(id)) elements.set(id, { dataset: {}, style: {}, classList: { toggle() {} } });
    return elements.get(id);
  } };
  globalThis.getUIText = (key, fallback) => strings[key] || fallback;
  CarrotEgpuModel.render({ available: true, state: "error", compiled: true, error_code: "timeout",
    detail: "Last worker stage: load_model", can_restart: false });
  assert.equal(elements.get("egpuModelState").textContent, "eGPU 사용 중단");
  assert.equal(elements.get("egpuModelDetail").textContent, "eGPU 응답이 늦어 내부 모델을 선택했습니다.");
  const recovery = elements.get("egpuModelRecovery");
  assert.equal(recovery.hidden, false);
  assert.ok(recovery.textContent.includes(strings.user_system));
  assert.ok(recovery.textContent.includes(strings.reboot));
  assert.match(recovery.textContent, /주차 후.*한 번.*다시 발생하면/);
});

test("model update card shows download progress before compilation is available", (t) => {
  const elements = new Map();
  const previousDocument = Object.getOwnPropertyDescriptor(globalThis, "document");
  t.after(() => {
    if (previousDocument) Object.defineProperty(globalThis, "document", previousDocument);
    else delete globalThis.document;
  });
  globalThis.document = {
    getElementById(id) {
      if (!elements.has(id)) elements.set(id, { dataset: {}, style: {}, classList: { toggle() {} } });
      return elements.get(id);
    },
  };
  const status = {
    available: true, state: "downloading", model_id: "new-model", compiled: false, can_restart: false,
    downloaded_bytes: 50 * 1024 * 1024, total_bytes: 200 * 1024 * 1024, progress: 25,
  };
  CarrotEgpuModel.render(status);
  assert.equal(elements.get("egpuModelState").textContent, "Downloading");
  assert.equal(elements.get("egpuModelProgress").hidden, false);
  assert.equal(elements.get("egpuModelProgressBar").style.width, "25%");
  assert.equal(elements.get("egpuModelAmount").textContent, "50.0 MB / 200.0 MB · 25.0%");
  assert.equal(elements.get("btnEgpuCompileRestart").hidden, true);

  CarrotEgpuModel.render({ ...status, state: "verifying", progress: 100 });
  assert.equal(elements.get("egpuModelState").textContent, "Verifying download");
  assert.equal(elements.get("btnEgpuCompileRestart").hidden, true);

  CarrotEgpuModel.render({ ...status, state: "ready", progress: 100, can_restart: true });
  assert.equal(elements.get("egpuModelState").textContent, "Ready to compile");
  assert.equal(elements.get("btnEgpuCompileRestart").hidden, false);
  assert.equal(elements.get("btnEgpuCompileRestart").disabled, false);
});

test("Jetson-only card shows IP and temperature, clears stale values and cannot compile eGPU", (t) => {
  const saved = globalThis.document;
  t.after(() => { globalThis.document = saved; });
  const elements = new Map();
  globalThis.document = { getElementById(id) {
    if (!elements.has(id)) elements.set(id, { dataset: {}, style: {}, classList: { toggle() {}, remove() {} } });
    return elements.get(id);
  } };
  const jetlink = { label: "jetSON", severity: "ok", addresses: ["192.168.0.199"], temp_c: 64.5 };
  CarrotEgpuModel.render({ available: false, jetlink });
  assert.equal(elements.get("egpuModelCard").hidden, false);
  assert.match(elements.get("jetlinkHealth").textContent, /192\.168\.0\.199.*64\.5/);
  assert.equal(elements.get("btnEgpuCompileRestart").hidden, true);
  CarrotEgpuModel.render({ available: false, jetlink: { ...jetlink, severity: "error", addresses: [], temp_c: null } });
  assert.doesNotMatch(elements.get("jetlinkHealth").textContent, /192\.168|64\.5/);
  assert.equal(elements.get("egpuModelCard").dataset.state, "error");
});

test("host reasons use Korean translations while preserving unknown diagnostics", (t) => {
  const savedDocument = globalThis.document;
  const savedTranslator = globalThis.getUIText;
  t.after(() => {
    globalThis.document = savedDocument;
    if (savedTranslator === undefined) delete globalThis.getUIText;
    else globalThis.getUIText = savedTranslator;
  });
  const elements = new Map();
  globalThis.document = { getElementById(id) {
    if (!elements.has(id)) elements.set(id, { dataset: {}, style: {}, classList: { toggle() {}, remove() {} } });
    return elements.get(id);
  } };
  globalThis.getUIText = (key, fallback) => ({
    egpu_model_host_not_connected: "Jetson이 연결되지 않음",
    egpu_model_host_unavailable: "Jetson 연결을 사용할 수 없음",
    egpu_model_host_health_unavailable: "Jetson 상태 정보를 받을 수 없음",
  })[key] || fallback;
  for (const [reason, expected] of [
    ["Host not connected", "Jetson이 연결되지 않음"],
    ["Host connection unavailable", "Jetson 연결을 사용할 수 없음"],
    ["Host health unavailable", "Jetson 상태 정보를 받을 수 없음"],
    ["USB transport failed: EPIPE", "USB transport failed: EPIPE"],
  ]) {
    CarrotEgpuModel.render({ available: false, jetlink: { label: "jetSON", severity: "unknown", reason } });
    assert.ok(elements.get("jetlinkHealth").textContent.endsWith(expected));
  }
});
