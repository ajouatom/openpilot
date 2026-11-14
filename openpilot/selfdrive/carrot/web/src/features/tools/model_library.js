"use strict";
const EN = {
  title: "External driving model", internal: "Internal model: CD210 (fixed)",
  help: "One choice for eGPU and Jetlink hosts. eGPU applies it after the next reboot. A Jetlink App (phone, tablet, Mac) or Jetson switches to it while parked, at its next connection; if the host lacks the model, the comma downloads it and sends it over the cable. A model chosen later in the App wins over this one.",
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
    if (kind === "select" && typeof showAppToast === "function") showAppToast(t("saved"));
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
export function renderLibrary(library, force = false, host = null) {
  if (!library?.models) return;
  const root = document.getElementById("externalModelLibrary");
  if (!root) return;
  root.hidden = false;
  const signature = JSON.stringify([library, host?.sha256, host?.preparation, t("title")]);
  if (!force && root.dataset.signature === signature) return;
  // Do not replace an in-flight selection with an older status poll.
  if (!force && root.querySelector("select")?.disabled) return;
  root.dataset.signature = signature;
  root.replaceChildren(text("h3", t("title")), text("p", t("internal")), text("p", t("help")));
  const effective = library.models.find(m => m.id === library.effective);
  root.append(text("p", `${t("current")}: ${effective?.name || library.effective_name || library.effective} · ${t((effective?.noo ?? library.effective_noo) ? "noo" : "no_noo")}`));
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
    if (!await action("select", select.value)) renderLibrary(library, true);
    select.disabled = false;
  });
  root.append(label);
  const running = hostModel(library, host);
  if (running) {
    const preparing = progressText(host.preparation);
    root.append(text("p", `${t(preparing ? "host_preparing" : "host_running")}: ${running}${preparing ? ` · ${preparing}` : ""}`));
  }
  const picked = library.models.find(m => m.id === library.app_pick?.model);
  if (picked) root.append(text("p", `${t("app_pick")}: ${picked.name}${library.app_pick.failed ? ` · ${t("app_pick_failed")}` : ""}`));
  root.append(text("p", t("shared")));
  if (library.reboot_required) root.append(text("p", t("saved")));
}
