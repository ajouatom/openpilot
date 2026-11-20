"use strict";
const EN = {
  title: "External driving model", internal: "Internal model: CD210 (fixed)",
  help: "One choice for eGPU and Jetson. Choosing saves the next-boot selection. Reboot comma and Jetson while parked with ignition off and internet available. First use downloads files; Jetson also prepares its engine.",
  change: "Change model", close: "Close", pending_apply: "Apply after restart",
  current: "This boot", next: "Next boot", saved: "Saved. Reboot comma and Jetson to apply.",
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
export function renderLibrary(library, force = false) {
  if (!library?.models) return;
  const root = document.getElementById("externalModelLibrary");
  if (!root) return;
  root.hidden = false;
  const dialog = document.getElementById("modelLibraryDialog");
  const editor = document.getElementById("modelLibraryEditor");
  // Keep the submitted choice stable until its response arrives.
  if (!force && editor.querySelector("select")?.disabled) return;
  const effective = library.models.find(m => m.id === library.effective);
  const selected = library.models.find(m => m.id === library.selected);
  const effectiveName = effective?.name || library.effective_name || library.effective;
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
  const state = card && !card.hidden ? document.getElementById("egpuModelState").textContent : t("internal");
  const amount = card && !card.hidden ? document.getElementById("egpuModelAmount").textContent : "";
  root.querySelector("p").textContent = library.reboot_required
    ? `${t("pending_apply")} · ${t("current")} ${effectiveName} · ${state}`
    : [state, amount].filter(Boolean).join(" · ");
  root.dataset.state = card?.dataset.state || "";
  root.querySelector("button").textContent = t("change");
  document.getElementById("modelLibraryTitle").textContent = t("title");
  document.getElementById("btnCloseModelLibrary").textContent = t("close");
  const signature = JSON.stringify([library.models, library.selected, library.effective,
    library.effective_name, library.effective_noo, library.reboot_required, t("title")]);
  if (!force && editor.dataset.signature === signature) return;
  editor.dataset.signature = signature;
  editor.replaceChildren(text("p", t("internal")), text("p", t("help")));
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
    if (!await action("select", select.value)) renderLibrary(library, true);
    select.disabled = false;
  });
  editor.append(label, text("p", t("shared")));
  if (library.reboot_required) editor.append(text("p", t("saved")));
}
