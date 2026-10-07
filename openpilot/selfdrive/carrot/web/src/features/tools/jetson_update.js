"use strict";

let timer = null;
let busy = false;
let latest = null;
const fallback = {
  title: "Jetson first update",
  start: "Wait for first Jetson update",
  cancel: "Cancel Jetson first-update wait",
  detail: "For an older Jetson, park in P, disengage assistance, then press the wait button below. This keeps the comma offroad with driving assistance stopped, even while ignition is on, so Jetson can download its update. Keep ignition and Jetson Internet connected. No SSH access is needed.",
  waiting: "The comma is holding offroad with its driving program stopped and supplying Jetson with the update selection. Moving the vehicle does not cancel the wait; driving assistance stays off. Remain parked with ignition and Jetson Internet connected. Older Jetsons cannot report download completion. The hold clears only when the new runtime is confirmed after a Jetson restart, or you cancel it.",
  timing: "If you restart only Jetson just after entering this wait, the comma keeps waiting. Jetson checks about 2 minutes after boot; otherwise its next periodic check can take about 15 minutes. Scheduling and downloading add time. Restarting Jetson does not reset this elapsed timer and is not detected as an update check.",
  elapsed: "Wait elapsed",
  timer_note: "Time observed during this comma boot; refresh or Jetson restart keeps it, comma reboot starts it over. This is not download progress or time remaining. Even after 20 minutes, completion is unconfirmed.",
  connected: "Jetson USB connected · download completion unavailable",
  disconnected: "Waiting for Jetson USB connection · elapsed time continues",
  status_unavailable: "Unable to refresh comma status · elapsed time unavailable",
  confirm: "Stop driving assistance and enter the first-update wait? Remain parked with ignition and Internet on. This hold persists after a restart until the new Jetson runtime is confirmed, or you cancel it here.",
  cancel_confirm: "Cancel the wait and allow normal startup? This does not confirm the Jetson update is complete.",
  park_required: "Park in P and disengage driving assistance before entering this mode.",
  jetson_unavailable: "Connect an older Jetson first. An updated Jetson does not need this mode.",
  error: "Unable to change the Jetson update wait. Check the connection and retry.",
};
const t = key => typeof getUIText === "function" ? getUIText(`jetson_update_${key}`, fallback[key]) : fallback[key];

function render(value = latest, available = true) {
  latest = value;
  const card = document.getElementById("jetsonUpdateCard");
  if (!card) return;
  card.hidden = !value || (!value.pending && (!value.connected || value.migrated));
  document.getElementById("jetsonUpdateTitle").textContent = t("title");
  document.getElementById("jetsonUpdateDetail").textContent = t(value?.pending ? "waiting" : "detail");
  const timing = document.getElementById("jetsonUpdateTiming");
  timing.hidden = !value?.pending;
  timing.textContent = t("timing");
  document.getElementById("jetsonUpdateProgress").hidden = !value?.pending;
  document.getElementById("jetsonUpdateElapsedLabel").textContent = t("elapsed");
  const seconds = value?.wait_elapsed_seconds;
  document.getElementById("jetsonUpdateElapsed").textContent = available && value?.pending && Number.isFinite(seconds) && seconds >= 0
    ? `${Math.floor(seconds / 60).toString().padStart(2, "0")}:${Math.floor(seconds % 60).toString().padStart(2, "0")}` : "--:--";
  document.getElementById("jetsonUpdateTimerNote").textContent = t("timer_note");
  document.getElementById("jetsonUpdateConnection").textContent = t(!available ? "status_unavailable" : value?.connected ? "connected" : "disconnected");
  const button = document.getElementById("btnJetsonUpdateWait");
  button.textContent = t(value?.pending ? "cancel" : "start");
  button.disabled = busy || !value || !available;
}

async function refresh() {
  try {
    render(await getJson("/api/tools/jetson-update"));
  } catch (_) {
    // Do not enable an action using a stale successful response.
    render(latest, false);
  } finally {
    window.clearTimeout(timer);
    timer = window.setTimeout(refresh, 1000);
  }
}

async function change() {
  if (busy || !latest) return;
  const enabled = !latest.pending;
  if (!await appConfirm(t(enabled ? "confirm" : "cancel_confirm"), { title: t("title") })) return;
  busy = true;
  render();
  try {
    render(await postJson("/api/tools/jetson-update", { enabled }));
  } catch (error) {
    if (typeof showAppToast === "function") showAppToast(t(error?.errorCode || "error") || t("error"), { tone: "error" });
  } finally {
    busy = false;
    await refresh();
  }
}

function init() {
  const button = document.getElementById("btnJetsonUpdateWait");
  if (button && button.dataset.bound !== "1") {
    button.dataset.bound = "1";
    button.addEventListener("click", change);
  }
  if (!timer) refresh();
}

globalThis.CarrotJetsonUpdate = Object.freeze({ init, render });
export { render, change };
