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
  waiting_for_network: "Waiting for network / automatic retry",
  installing: "Installing eGPU package",
  installed: "Installed · next start",
  active: "eGPU running",
  waiting_for_network_detail: "Keep an internet connection available. Installation retries automatically every 30 seconds.",
  installing_detail: "Preparing the verified model and runtime. The current driving model continues running.",
  installed_detail: "Installation is complete. The eGPU will be tried at the next ignition session; no restart is needed while driving.",
  active_detail: "The eGPU is currently running the driving model.",
  failure_dns: "Cannot resolve the download server address. Check the internet connection; retry is automatic.",
  failure_clock: "Certificate not yet valid. Waiting for the device clock/network to recover; retry is automatic.",
  failure_network: "Download connection interrupted. Retrying automatically.",
  failure_server: "The download server is temporarily unavailable. Retrying automatically.",
  failure_certificate: "Server certificate verification failed. The download was stopped; verification remains enabled.",
  failure_download: "The server refused the download or the requested file is unavailable.",
  failure_storage: "Not enough free storage to install the eGPU package.",
  failure_install: "The eGPU package could not be installed or verified. The saved diagnostic report contains the cause.",
  failure_rejected: "This model was blocked after an earlier runtime failure. The saved diagnostic report contains the cause.",
  failure_runtime: "The eGPU failed to start or run. The internal model is selected. Diagnostic details are saved automatically.",
  failure_pcie: "The eGPU's PCIe link did not become ready. The internal model is selected. This does not identify whether power, a connection, or software caused the failure. Diagnostics are saved automatically.",
  failure_timeout: "The eGPU stopped responding in time. The internal model is selected. The last worker stage is saved automatically; the underlying cause is not yet confirmed.",
  failure_usb: "The eGPU reported a USB communication error. The internal model is selected. This alone does not prove a faulty cable. Diagnostics are saved automatically.",
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
  host_ok: "Connected",
  host_warning: "High temperature",
  host_error: "Host error / connection lost",
  host_unknown: "Waiting for current host health",
  host_ip: "Host IP",
  host_no_ip: "Unavailable",
  host_not_connected: "Host not connected",
  host_unavailable: "Host connection unavailable",
  host_health_unavailable: "Host health unavailable",
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
  if (modelId.includes("4bfb5340-e20cde17")) return "Mountain Dew v1 (870a4823)";
  if (modelId.includes("pr39047") || modelId.includes("mdm-v1")) return "Mountain Dew v1";
  if (modelId.includes("pr38932") || modelId.includes("cinque-v3")) return "Cinque v3";
  if (modelId.includes("pr38823") || modelId.includes("cinque-v2")) return "Cinque v2";
  if (modelId.includes("pr38771") || modelId.includes("cinque-terre")) return "Cinque Terre";
  if (modelId.includes("bmrlanpv6")) return "BMRLNAP v6";
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
  if (!status?.available && !status?.jetlink) {
    card.hidden = true;
    return;
  }

  const state = String(status.state || "checking");
  const percent = Number(status.progress);
  const running = ["checking", "downloading", "verifying", "compiling", "waiting_for_network", "installing"].includes(state);
  const stateEl = document.getElementById("egpuModelState");
  const detailEl = document.getElementById("egpuModelDetail");
  const progressEl = document.getElementById("egpuModelProgress");
  const progressBar = document.getElementById("egpuModelProgressBar");
  const amountEl = document.getElementById("egpuModelAmount");
  const action = document.getElementById("btnEgpuCompileRestart");

  let hostEl = document.getElementById("jetlinkHealth");
  if (!hostEl) {
    hostEl = document.createElement("div");
    hostEl.id = "jetlinkHealth";
    hostEl.className = "egpu-model-card__detail";
    detailEl.after(hostEl);
  }
  const host = status.jetlink;
  hostEl.hidden = !host;
  if (host) {
    const severity = ["ok", "warning", "error", "unknown"].includes(host.severity) ? host.severity : "unknown";
    hostEl.textContent = `${host.label} · ${t(`host_${severity}`)} · ${t("host_ip")}: ${(host.addresses || []).join(", ") || t("host_no_ip")}`
      + (Number.isFinite(host.temp_c) ? ` · ${host.temp_c.toFixed(1)} °C` : "")
        + (host.reason ? ` · ${({
          "Host not connected": t("host_not_connected"),
          "Host connection unavailable": t("host_unavailable"),
          "Host health unavailable": t("host_health_unavailable"),
        })[host.reason] || host.reason}` : "");
    hostEl.style.color = severity === "error" ? "#ff7474" : severity === "warning" ? "#ffd166" : "";
  }
  if (!status.available) {
    card.hidden = false;
    card.dataset.state = host.severity === "error" ? "error" : "compiled";
    card.classList.remove("is-running");
    document.getElementById("egpuModelTitle").textContent = `${host.label} · Cinque v2`;
    stateEl.textContent = t(`host_${host.severity || "unknown"}`);
    detailEl.textContent = "";
    progressEl.hidden = true;
    amountEl.textContent = "";
    action.hidden = true;
    return;
  }

  card.hidden = false;
  card.dataset.state = state;
  card.classList.toggle("is-running", running);
  document.getElementById("egpuModelTitle").textContent = modelDisplayTitle(status);
  const active = status.active && ["compiled", "installed"].includes(state);
  stateEl.textContent = t(active ? "active" : state);
  detailEl.textContent = status.error_code ? t(`failure_${status.error_code}`) : t(active ? "active_detail" : `${state}_detail`);
  if (status.error_code && status.detail) detailEl.textContent += ` (${String(status.detail).slice(0, 500)})`;

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
    if (lastStatus?.jetlink) render({ ...lastStatus, jetlink: {
      ...lastStatus.jetlink, severity: "unknown", addresses: [], temp_c: null, reason: "", fresh: false,
    } });
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
