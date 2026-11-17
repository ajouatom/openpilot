"use strict";
import { renderLibrary } from "./model_library.js";

const POLL_INTERVAL_MS = 2000;
let pollTimer = null;
let lastStatus = null;
let restartPending = false;

const FALLBACK_STRINGS = {
  recovering: "eGPU reconnecting",
  recovering_detail: "The internal model remains in use while eGPU recovery is attempted, up to three times.",
  recovery_ready: "eGPU READY",
  recovery_ready_detail: "Test calculations passed. The internal model remains in use until the vehicle stops or you intervene with steering.",
  recovery_stopped: "eGPU recovery stopped",
  recovery_stopped_detail: "The internal model remains in use. Once parked, restart the comma. Recovery stopped because a check failed or the retry limit was reached.",
  title: "eGPU big model",
  checking: "Checking model",
  downloading: "Downloading",
  verifying: "Verifying download",
  ready: "Ready to compile",
  waiting_for_ignition: "Waiting for ignition",
  compiling: "Compiling eGPU model",
  compiled: "Ready to use",
  error: "eGPU setup failed",
  runtime_error: "eGPU unavailable",
  recovery_restart: "Once parked, open User / System below and press Reboot once. If this happens again, send a screenshot of this card for support.",
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
  failure_rejected: "A previous failure blocked this model. Compiling again does not resolve a runtime failure. If it remains after a reboot, download the diagnostics and send them for support.",
  diagnostics: "Download diagnostics",
  retry_verification: "Recheck blocked model after reboot",
  retry_confirm: "Keep the diagnostic report and retry file checks and a startup inference test after the next full reboot? This does not start the GPU now.",
  retry_pending: "Verification is scheduled. Reboot comma once while parked. No files or diagnostics have been deleted.",
  retry_offroad: "Turn ignition off and wait for the offroad screen to schedule verification. Then reboot comma while parked.",
  blocked_device: "GPU/USB timeout or hang: this model is blocked for this boot. A full reboot automatically retries file checks and startup inference. Repeated failures need device/power/USB diagnosis.",
  blocked_verify: "This model is blocked after a failed check. You can schedule one new verification after reboot. File and output checks remain mandatory; another failure falls back to the internal model.",
  blocked_missing: "This model is blocked, but its diagnostic identity is missing or inconsistent. Send the diagnostic report for support; automatic release is unavailable.",
  failure_runtime: "The eGPU could not start or continue running. The internal model is selected.",
  failure_pcie: "The eGPU connection could not start. The internal model is selected.",
  failure_timeout: "The eGPU took too long to respond. The internal model is selected.",
  failure_usb: "Communication with the eGPU was interrupted. The internal model is selected.",
  checking_detail: "Checking the model catalog. You can keep using openpilot.",
  downloading_detail: "Download may continue while driving. Do not restart or power off until it finishes.",
  verifying_detail: "Checking the complete file. This can take a moment.",
  ready_detail: "Park, turn ignition on, then restart once to compile.",
  waiting_for_ignition_detail: "Keep the car parked and ignition on, then restart to compile.",
  compiling_detail: "Keep the car parked and ignition on. The screen will update when compilation finishes.",
  compiled_detail: "The eGPU big model is compiled and will be selected automatically.",
  error_detail: "The eGPU is not ready. Send a screenshot of this card for support.",
  restart: "Restart & compile",
  restart_confirm: "Park the car and turn ignition on. Restart now to compile the eGPU big model?",
  restart_requested: "Restart requested. Compilation will begin during boot.",
  host_usb_error: "Comma USB setup blocked",
  handoff_opt_in_missing: "USB ADB handoff permission is missing.",
  handoff_opt_in_invalid: "USB ADB permission marker ownership, permissions or file type failed validation.",
  handoff_controller: "The USB controller is unsupported.",
  handoff_platform: "The required comma platform marker is missing.",
  handoff_offroad: "Turn ignition off and wait for comma to enter offroad.",
  handoff_usb_role: "Comma USB has not entered device role (ufp).",
  handoff_gadget_identity: "The g1 USB identity differs from the expected ADB gadget.",
  handoff_gadget_functions: "The g1 functions differ from the expected ADB and network configuration.",
  handoff_gadget_tree: "USB gadget entry count or file type validation failed.",
  handoff_controller_changed: "The g1 USB controller binding changed during validation.",
  handoff_other_owner: "Another USB gadget has a bound controller.",
  handoff_lock_invalid: "USB handoff lock ownership or file type is invalid.",
  handoff_lock_busy: "Another USB handoff is in progress.",
  handoff_udc_attribute: "The g1 controller attribute has an invalid file type.",
  handoff_write_failed: "The USB controller unbind write was incomplete.",
  handoff_still_bound: "The g1 gadget remains bound after the unbind request.",
  handoff_io_error: "USB handoff file access failed. See the stage and error number below.",
  handoff_not_root: "The USB handoff helper requires root privileges.",
  handoff_helper_failed: "The USB handoff helper exited without a reason.",
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

function hostReason(reason) {
  const code = String(reason || "").match(/\[adb-handoff:([a-z_]+)\]/)?.[1];
  const key = `handoff_${code}`;
  if (code && FALLBACK_STRINGS[key]) return `${t(key)} · ${reason}`;
  return ({
    "Host not connected": t("host_not_connected"),
    "Host connection unavailable": t("host_unavailable"),
    "Host health unavailable": t("host_health_unavailable"),
  })[reason] || reason;
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
  if (modelId.includes("light-speed-2")) return "Light speed 2";
  if (modelId.includes("light-speed")) return "Light speed";
  if (modelId.includes("flip-flop")) return "Flip Flop";
  if (modelId.includes("md-v3")) return "MD v3";
  if (modelId.includes("nav-n16")) return "N16 Nav (NOO)";
  if (modelId.includes("comma-nav-")) return "Nav Model (cf8f6120)";
  if (modelId.includes("pr39047") || modelId.includes("mdm-v1") || modelId.includes("6f4c0faf-256f2178") || modelId.includes("4bfb5340-e20cde17")) return "Mountain Dew";
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
  renderStatus(status);
  renderLibrary(status?.library);
}

function renderStatus(status) {
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
  const recoveryEl = document.getElementById("egpuModelRecovery");
  recoveryEl.hidden = true;
  recoveryEl.textContent = "";
  const progressEl = document.getElementById("egpuModelProgress");
  const progressBar = document.getElementById("egpuModelProgressBar");
  const amountEl = document.getElementById("egpuModelAmount");
  const action = document.getElementById("btnEgpuCompileRestart");
  const diagnostics = document.getElementById("egpuModelDiagnostics");
  let retryButton = document.getElementById("btnEgpuRetryVerification");
  if (!retryButton) {
    retryButton = document.createElement("button");
    retryButton.id = "btnEgpuRetryVerification";
    retryButton.className = action.className;
    retryButton.type = "button";
    retryButton.addEventListener("click", requestVerification);
    action.after(retryButton);
  }
  const blocked = !!status.rejection?.blocked && !status.active;
  retryButton.hidden = !status.available || !blocked;
  retryButton.textContent = t("retry_verification");
  retryButton.disabled = !status.rejection?.can_request || !!status.rejection?.retry_pending;
  if (diagnostics) {
    diagnostics.hidden = !status.available || state !== "error" || !!status.active;
    diagnostics.textContent = t("diagnostics");
  }

  let hostEl = document.getElementById("jetlinkHealth");
  if (!hostEl) {
    hostEl = document.createElement("div");
    hostEl.id = "jetlinkHealth";
    hostEl.className = "egpu-model-card__detail";
    detailEl.after(hostEl);
  }
  const host = status.jetlink;
  hostEl.hidden = !host;
  const usbBlocked = String(host?.reason || "").includes("[adb-handoff:");
  if (host) {
    const severity = ["ok", "warning", "error", "unknown"].includes(host.severity) ? host.severity : "unknown";
    hostEl.textContent = `${host.label} · ${usbBlocked ? t("host_usb_error") : t(`host_${severity}`)} · ${t("host_ip")}: ${(host.addresses || []).join(", ") || t("host_no_ip")}`
      + (Number.isFinite(host.temp_c) ? ` · ${host.temp_c.toFixed(1)} °C` : "")
        + (host.reason ? ` · ${hostReason(host.reason)}` : "");
    hostEl.style.color = severity === "error" ? "#ff7474" : severity === "warning" ? "#ffd166" : "";
  }
  if (!status.available) {
    card.hidden = false;
    card.dataset.state = host.severity === "error" ? "error" : "compiled";
    card.classList.remove("is-running");
    document.getElementById("egpuModelTitle").textContent = `${host.label} · ${status.library?.models?.find(m => m.id === status.library.effective)?.name || "Jetson"}`;
    stateEl.textContent = usbBlocked ? t("host_usb_error") : t(`host_${host.severity || "unknown"}`);
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
  const runtimeFailure = state === "error" && ["runtime", "pcie", "timeout", "usb"].includes(status.error_code);
  stateEl.textContent = t(active ? "active" : runtimeFailure ? "runtime_error" : state);
  const failureKey = `failure_${status.error_code}`;
  // Raw worker stages and exception messages remain in the API and saved logs.
  // The card explains the next user action, including a safe fallback for new error codes.
  detailEl.textContent = status.error_code
    ? t(Object.hasOwn(FALLBACK_STRINGS, failureKey) ? failureKey : "error_detail")
    : t(active ? "active_detail" : `${state}_detail`);
  recoveryEl.hidden = !runtimeFailure;
  recoveryEl.textContent = runtimeFailure ? t("recovery_restart") : "";
  if (blocked) {
    const rejection = status.rejection;
    detailEl.textContent = t(rejection.reason === "device_restart_required" ? "blocked_device"
      : rejection.reason === "diagnostics_missing" ? "blocked_missing" : "blocked_verify");
    if (rejection.error) detailEl.textContent += ` · ${rejection.phase || ""}: ${rejection.error}`;
    recoveryEl.hidden = rejection.reason === "diagnostics_missing";
    recoveryEl.textContent = t(rejection.retry_pending ? "retry_pending" : "retry_offroad");
  }

  const showProgress = state === "downloading" && Number.isFinite(percent);
  progressEl.hidden = !showProgress;
  progressBar.style.width = `${Math.min(100, Math.max(0, percent || 0))}%`;
  amountEl.textContent = showProgress
    ? `${formatBytes(status.downloaded_bytes)} / ${formatBytes(status.total_bytes)} · ${percent.toFixed(1)}%`
    : (state === "compiling" ? elapsedText(status.started_at) : "");

  action.textContent = restartPending ? t("checking") : t("restart");
  action.hidden = !["ready", "waiting_for_ignition", "error"].includes(state) || status.compiled || status.error_code === "rejected";
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
  document.getElementById("modelLibraryDialog")?.close();
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

async function requestVerification() {
  document.getElementById("modelLibraryDialog")?.close();
  const sha256 = lastStatus?.sha256;
  if (!await appConfirm(t("retry_confirm"), { title: t("retry_verification") })) return;
  const button = document.getElementById("btnEgpuRetryVerification");
  if (button) button.disabled = true;
  try {
    await postJson("/api/egpu/model/retry-verification", { sha256 });
    if (typeof showAppToast === "function") showAppToast(t("retry_pending"));
  } catch (error) {
    if (typeof showAppToast === "function") showAppToast(`${t("error")}: ${error?.message || error}`, { tone: "error" });
  }
  await refresh();
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
