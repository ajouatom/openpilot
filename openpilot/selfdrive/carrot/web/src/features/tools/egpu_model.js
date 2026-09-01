"use strict";

const POLL_INTERVAL_MS = 2000;
let pollTimer = null;
let lastStatus = null;
let restartPending = false;

const FALLBACK_STRINGS = {
  title: "eGPU big model",
  checking: "Checking model",
  downloading: "Downloading",
  verifying: "Verifying download",
  ready: "Ready to compile",
  waiting_for_ignition: "Waiting for ignition",
  compiling: "Compiling eGPU model",
  compiled: "Ready to use",
  error: "Needs attention",
  checking_detail: "Checking the model catalog. You can keep using openpilot.",
  downloading_detail: "Download may continue while driving. Do not restart or power off until it finishes.",
  verifying_detail: "Checking the complete file. This can take a moment.",
  ready_detail: "Park, turn ignition on, then restart once to compile.",
  waiting_for_ignition_detail: "Keep the car parked and ignition on, then restart to compile.",
  compiling_detail: "Keep the car parked and ignition on. The screen will update when compilation finishes.",
  compiled_detail: "The eGPU big model is compiled and will be selected automatically.",
  error_detail: "The internal model remains available. Check the connection and try again on the next restart.",
  restart: "Restart & compile",
  restart_confirm: "Park the car and turn ignition on. Restart now to compile the eGPU big model?",
  restart_requested: "Restart requested. Compilation will begin during boot.",
};

function t(key) {
  return typeof getUIText === "function" ? getUIText(`egpu_model_${key}`, FALLBACK_STRINGS[key] || key) : (FALLBACK_STRINGS[key] || key);
}

function formatBytes(value) {
  const bytes = Math.max(0, Number(value) || 0);
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(0)} KB`;
  return `${(bytes / (1024 * 1024)).toFixed(bytes >= 1024 * 1024 * 1024 ? 0 : 1)} MB`;
}

function elapsedText(startedAt) {
  const elapsed = Math.max(0, Math.floor(Date.now() / 1000 - (Number(startedAt) || Date.now() / 1000)));
  const minutes = Math.floor(elapsed / 60);
  return `${String(minutes).padStart(2, "0")}:${String(elapsed % 60).padStart(2, "0")}`;
}

function modelDisplayName(status) {
  const explicitName = String(status?.display_name || "").trim();
  if (explicitName) return explicitName;

  const modelId = String(status?.model_id || "").toLowerCase();
  if (modelId.includes("pr38739") || modelId.includes("tgc")) return "TGC";
  if (modelId.includes("pr38726") || modelId.includes("time-to-go")) return "Time to Go";
  return "";
}

function modelDisplayTitle(status, fallbackTitle = t("title")) {
  const name = modelDisplayName(status);
  return name ? `${name} · eGPU` : fallbackTitle;
}

function render(status = lastStatus) {
  const card = document.getElementById("egpuModelCard");
  if (!card) return;
  lastStatus = status;
  if (!status?.available) {
    card.hidden = true;
    return;
  }

  const state = String(status.state || "checking");
  const percent = Number(status.progress);
  const running = ["checking", "downloading", "verifying", "compiling"].includes(state);
  const stateEl = document.getElementById("egpuModelState");
  const detailEl = document.getElementById("egpuModelDetail");
  const progressEl = document.getElementById("egpuModelProgress");
  const progressBar = document.getElementById("egpuModelProgressBar");
  const amountEl = document.getElementById("egpuModelAmount");
  const action = document.getElementById("btnEgpuCompileRestart");

  card.hidden = false;
  card.dataset.state = state;
  card.classList.toggle("is-running", running);
  document.getElementById("egpuModelTitle").textContent = modelDisplayTitle(status);
  stateEl.textContent = t(state);
  detailEl.textContent = t(`${state}_detail`);

  const showProgress = state === "downloading" && Number.isFinite(percent);
  progressEl.hidden = !showProgress;
  progressBar.style.width = `${Math.min(100, Math.max(0, percent || 0))}%`;
  amountEl.textContent = showProgress
    ? `${formatBytes(status.downloaded_bytes)} / ${formatBytes(status.total_bytes)} · ${percent.toFixed(1)}%`
    : (state === "compiling" ? elapsedText(status.started_at) : "");

  action.textContent = restartPending ? t("checking") : t("restart");
  action.hidden = !["ready", "waiting_for_ignition", "error"].includes(state) || status.compiled;
  action.disabled = restartPending || !status.can_restart;
}

async function refresh() {
  try {
    render(await getJson("/api/egpu/model"));
  } catch (_error) {
    // An older/non-eGPU branch may not expose this endpoint. Stay invisible.
  } finally {
    window.clearTimeout(pollTimer);
    pollTimer = window.setTimeout(refresh, POLL_INTERVAL_MS);
  }
}

async function requestCompileRestart() {
  if (!await appConfirm(t("restart_confirm"), { title: t("restart") })) return;
  const button = document.getElementById("btnEgpuCompileRestart");
  restartPending = true;
  if (button) {
    button.disabled = true;
    button.textContent = t("checking");
  }
  try {
    await postJson("/api/egpu/model/compile-restart", {});
    if (typeof showAppToast === "function") showAppToast(t("restart_requested"));
    return;
  } catch (error) {
    restartPending = false;
    if (typeof showAppToast === "function") {
      showAppToast(`${t("error")}: ${error?.message || error}`, { tone: "error", duration: 5000 });
    }
    if (typeof showError === "function") showError("eGPU model", error);
    await refresh();
  }
}

function init() {
  const button = document.getElementById("btnEgpuCompileRestart");
  if (button && button.dataset.bound !== "1") {
    button.dataset.bound = "1";
    button.addEventListener("click", requestCompileRestart);
  }
  if (!pollTimer) refresh();
}

const CarrotEgpuModel = Object.freeze({ init, refresh, render });
globalThis.CarrotEgpuModel = CarrotEgpuModel;

export { CarrotEgpuModel, modelDisplayName, modelDisplayTitle };
