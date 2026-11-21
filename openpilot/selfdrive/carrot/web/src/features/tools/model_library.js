"use strict";
const EN = {
  title: "External driving model", internal: "Internal model",
  internal_help: "The comma's own driving model, used when no eGPU or Jetlink host runs the model. Applies from the next ignition.",
  internal_saved: "Saved. The internal model changes at the next ignition.",
  internal_running: "Last drive ran", internal_pending: "applies at the next ignition",
  internal_failed: "could not be loaded; the drive ran on CD210", internal_failed_short: "failed",
  internal_unavailable: "not installed",
  help: "One choice for eGPU and Jetlink hosts. eGPU applies it after the next reboot. A Jetlink App (phone, tablet, Mac) or Jetson switches to it while parked, at its next connection; if the host lacks the model, the comma downloads it and sends it over the cable. A model chosen later in the App wins over this one.",
  change: "Change model", close: "Close", pending_apply: "Apply after restart",
  current: "eGPU this boot", next: "Model", saved: "Saved. eGPU: reboot to apply. Jetlink host: switches while parked.",
  host_running: "Jetlink host running", host_preparing: "Jetlink host preparing", app_pick: "Sent to Jetlink host",
  app_pick_failed: "download or preparation failed; choose it again to retry",
  noo: "NOO supported", no_noo: "NOO not supported", shared: "N16 supports navigation inputs. Flip Flop, Light speed and Light speed 2 do not.",

};
function t(key) { return typeof getUIText === "function" ? getUIText(`model_library_${key}`, EN[key]) : EN[key]; }
function text(tag, value) { const e = document.createElement(tag); e.textContent = value; return e; }
async function action(kind, model) {
  try {
    const response = await postJson("/api/egpu/model/library", { action: kind, model });
    renderLibrary(response.library, true);
    if (typeof showAppToast === "function") showAppToast(t(kind === "internal" ? "internal_saved" : "saved"));
    globalThis.CarrotEgpuModel?.refresh();
    return true;
  } catch (error) {
    if (typeof showAppToast === "function") showAppToast(error?.message || String(error), { tone: "error" });
    return false;
  }
}
function hostModel(library, host) {
  if (!host?.sha256) return "";
  const known = library.models.find(m => m.jetson_sha256 === host.sha256);
  return known ? `${known.name} · ${t(known.noo ? "noo" : "no_noo")}` : host.sha256.slice(0, 12);
}
function progressText(preparation) {
  if (!preparation?.stage) return "";
  const fraction = Number(preparation.fraction);
  return `${preparation.message || preparation.stage}${Number.isFinite(fraction) && fraction > 0 ? ` · ${(fraction * 100).toFixed(0)}%` : ""}`;
}
// The Jetlink host's running or preparing model, shown both in the summary
// and in the dialog: a phone or tablet switches while parked, so its state
// matters without opening the dialog.
function hostLine(library, host) {
  const running = hostModel(library, host);
  if (!running) return "";
  const preparing = progressText(host.preparation);
  return `${t(preparing ? "host_preparing" : "host_running")}: ${running}${preparing ? ` · ${preparing}` : ""}`;
}
function internalModelName(internal, id) {
  return internal?.models?.find(m => m.id === id)?.name || id || "CD210";
}
// What drivingd loaded on the last start (`internal_model.last`), so a
// choice that failed to load is visible without reading logs.
function internalLastLine(internal) {
  const last = internal?.last;
  if (!last?.running) return "";
  const running = `${t("internal_running")}: ${internalModelName(internal, last.running)}`;
  if (last.error) return `${running} · ${internalModelName(internal, last.selected)} ${t("internal_failed")} (${last.error})`;
  if (last.selected !== internal.selected) return `${running} · ${internalModelName(internal, internal.selected)} ${t("internal_pending")}`;
  return running;
}
export function renderLibrary(library, force = false, host = null) {
  if (!library?.models) return;
  const root = document.getElementById("externalModelLibrary");
  if (!root) return;
  root.hidden = false;
  const dialog = document.getElementById("modelLibraryDialog");
  const editor = document.getElementById("modelLibraryEditor");
  // Keep the submitted choice stable until its response arrives.
  if (!force && (editor.dataset.busy || editor.querySelector("select")?.disabled)) return;
  const effective = library.models.find(m => m.id === library.effective);
  const selected = library.models.find(m => m.id === library.selected);
  const effectiveName = effective?.name || library.effective_name || library.effective;
  const hostText = hostLine(library, host);
  if (!root.querySelector("button")) {
    const summary = text("div", "");
    summary.className = "model-library__summary";
    summary.append(text("strong", ""), text("p", ""));
    const button = text("button", "");
    button.id = "btnChangeModel";
    button.type = "button";
    button.className = "btn";
    button.setAttribute("aria-haspopup", "dialog");
    button.setAttribute("aria-controls", "modelLibraryDialog");
    button.addEventListener("click", () => dialog.showModal());
    root.append(summary, button);
    document.getElementById("btnCloseModelLibrary").addEventListener("click", () => dialog.close());
  }
  root.querySelector("strong").textContent = selected?.name || library.selected;
  const card = document.getElementById("egpuModelCard");
  const internal = library.internal_model;
  const internalName = internalModelName(internal, internal?.selected);
  const failed = internal?.last?.error && internal.last.selected === internal.selected;
  const state = card && !card.hidden ? document.getElementById("egpuModelState").textContent
    : `${t("internal")}: ${failed ? `CD210 · ${internalName} ${t("internal_failed_short")}` : internalName}`;
  const amount = card && !card.hidden ? document.getElementById("egpuModelAmount").textContent : "";
  root.querySelector("p").textContent = library.reboot_required
    ? `${t("pending_apply")} · ${t("current")} ${effectiveName} · ${state}`
    : [state, amount, hostText].filter(Boolean).join(" · ");
  root.dataset.state = card?.dataset.state || "";
  root.querySelector("button").textContent = t("change");
  document.getElementById("modelLibraryTitle").textContent = t("title");
  document.getElementById("btnCloseModelLibrary").textContent = t("close");
  const signature = JSON.stringify([library.models, library.selected, library.effective,
    library.effective_name, library.effective_noo, library.reboot_required, library.app_pick, internal,
    host?.sha256, host?.preparation, t("title")]);
  if (!force && editor.dataset.signature === signature) return;
  editor.dataset.signature = signature;
  editor.replaceChildren();
  if (internal?.models?.length) {
    const internalLabel = text("label", t("internal"));
    const internalSelect = document.createElement("select");
    internalSelect.setAttribute("aria-label", t("internal"));
    for (const model of internal.models) {
      // A bundle that is not on the device cannot run; keep it visible but not selectable.
      const option = text("option", model.available === false ? `${model.name} (${t("internal_unavailable")})` : model.name);
      if (model.available === false && model.id !== internal.selected) option.disabled = true;
      option.value = model.id;
      internalSelect.append(option);
    }
    internalSelect.value = internal.selected;
    internalLabel.append(internalSelect);
    internalSelect.addEventListener("change", async () => {
      internalSelect.disabled = true;
      editor.dataset.busy = "1";
      const saved = await action("internal", internalSelect.value);
      delete editor.dataset.busy;
      if (!saved) renderLibrary(library, true, host);
      internalSelect.disabled = false;
    });
    editor.append(internalLabel);
    const lastLine = internalLastLine(internal);
    if (lastLine) editor.append(text("p", lastLine));
    editor.append(text("p", t("internal_help")));
  } else {
    editor.append(text("p", `${t("internal")}: ${internalName}`));
  }
  editor.append(text("h3", t("title")), text("p", t("help")));
  editor.append(text("p", `${t("current")} ${effectiveName} · ${t((effective?.noo ?? library.effective_noo) ? "noo" : "no_noo")}`));
  const label = text("label", t("next"));
  const select = document.createElement("select");
  select.setAttribute("aria-label", t("next"));
  for (const model of library.models) {
    const option = text("option", `${model.name} · ${t(model.noo ? "noo" : "no_noo")}`);
    option.value = model.id;
    select.append(option);
  }
  select.value = library.selected;
  label.append(select);
  select.addEventListener("change", async () => {
    select.disabled = true;
    editor.dataset.busy = "1";
    const saved = await action("select", select.value);
    delete editor.dataset.busy;
    if (!saved) renderLibrary(library, true, host);
    select.disabled = false;
  });
  editor.append(label);
  if (hostText) editor.append(text("p", hostText));
  const picked = library.models.find(m => m.id === library.app_pick?.model);
  if (picked) editor.append(text("p", `${t("app_pick")}: ${picked.name}${library.app_pick.failed ? ` · ${t("app_pick_failed")}` : ""}`));
  editor.append(text("p", t("shared")));
  if (library.reboot_required) editor.append(text("p", t("saved")));
}
