"use strict";

let timer = null;
let busy = false;
let latest = null;
const fallback = {
  title: "Jetson first update",
  start: "Wait for first Jetson update",
  cancel: "Cancel Jetson first-update wait",
  detail: "For older Jetsons, enter this mode while parked in P with assistance disengaged. Keep ignition and Internet on. Driving assistance stops during the wait. After an ignition power cycle, the hold clears when the new runtime is confirmed.",
  waiting: "Waiting for the first update. Keep ignition and Internet on while parked. Older Jetsons cannot report download completion. If the next boot still has the old runtime, this wait continues.",
  timing: "Older Jetsons check for updates about every 15 minutes. The next check may take about 15 minutes, then downloading takes additional time. This screen cannot confirm completion: it checks the installed version after an ignition power cycle. Elapsed time alone does not mean the update is complete.",
  confirm: "Stop driving assistance and enter the first-update wait? Remain parked with ignition and Internet on. This hold persists after a restart until the new Jetson runtime is confirmed, or you cancel it here.",
  cancel_confirm: "Cancel the wait and allow normal startup? This does not confirm the Jetson update is complete.",
  park_required: "Park in P and disengage driving assistance before entering this mode.",
  jetson_unavailable: "Connect an older Jetson first. An updated Jetson does not need this mode.",
  error: "Unable to change the Jetson update wait. Check the connection and retry.",
};
const t = key => typeof getUIText === "function" ? getUIText(`jetson_update_${key}`, fallback[key]) : fallback[key];

function render(value = latest) {
  latest = value;
  const card = document.getElementById("jetsonUpdateCard");
  if (!card) return;
  card.hidden = !value || (!value.pending && (!value.connected || value.migrated));
  document.getElementById("jetsonUpdateTitle").textContent = t("title");
  document.getElementById("jetsonUpdateDetail").textContent = t(value?.pending ? "waiting" : "detail");
  const timing = document.getElementById("jetsonUpdateTiming");
  timing.hidden = !value?.pending;
  timing.textContent = t("timing");
  const button = document.getElementById("btnJetsonUpdateWait");
  button.textContent = t(value?.pending ? "cancel" : "start");
  button.disabled = busy || !value;
}

async function refresh() {
  try {
    render(await getJson("/api/tools/jetson-update"));
  } catch (_) {
    // Do not enable an action using a stale successful response.
    const button = document.getElementById("btnJetsonUpdateWait");
    if (button) button.disabled = true;
  } finally {
    window.clearTimeout(timer);
    timer = window.setTimeout(refresh, 2000);
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
