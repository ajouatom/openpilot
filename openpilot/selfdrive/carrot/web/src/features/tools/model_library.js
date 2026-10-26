"use strict";
const EN = {
  title: "External driving model", internal: "Internal model: CD210 (fixed)",
  help: "One choice for eGPU and Jetson. Save, then reboot comma and Jetson while parked with ignition off and internet available. First use downloads files; Jetson also prepares its engine.",
  current: "This boot", next: "Next boot", save: "Save for next reboot", saved: "Saved. Reboot comma and Jetson to apply.",
  noo: "NOO supported", no_noo: "NOO not supported", shared: "Flip Flop and MD v3 use different eGPU PKLs, but share the same Jetson ONNX checkpoint.",
  remove: "Delete unused files & hide", restore: "Show again", hidden: "Hidden models", pending: "Jetson cleanup pending (offroad connection required)",
  unknown: "Jetson cache: waiting for offroad status", protected: "Protected", confirm: "Delete this unused model's cached files and hide it? Selecting it again will download it again.",
};
function t(key) { return typeof getUIText === "function" ? getUIText(`model_library_${key}`, EN[key]) : EN[key]; }
function text(tag, value) { const e = document.createElement(tag); e.textContent = value; return e; }
function mb(n) { return `${(Math.max(0, Number(n) || 0) / 1048576).toFixed(1)} MB`; }
async function action(kind, model) {
  if (kind === "delete" && !await appConfirm(t("confirm"), { title: t("remove") })) return;
  try {
    const response = await postJson("/api/egpu/model/library", { action: kind, model });
    renderLibrary(response.library, true);
    if (kind === "select" && typeof showAppToast === "function") showAppToast(t("saved"));
    globalThis.CarrotEgpuModel?.refresh();
  } catch (error) {
    if (typeof showAppToast === "function") showAppToast(error?.message || String(error), { tone: "error" });
  }
}
export function renderLibrary(library, force = false) {
  if (!library?.models) return;
  const root = document.getElementById("externalModelLibrary");
  if (!root) return;
  root.hidden = false;
  const signature = JSON.stringify([library, t("title")]);
  if (!force && root.dataset.signature === signature) return;
  // Keep an unsaved choice while inventory/status polling updates the card.
  const oldChoice = root.querySelector("select")?.value;
  root.dataset.signature = signature;
  root.replaceChildren(text("h3", t("title")), text("p", t("internal")), text("p", t("help")));
  const effective = library.models.find(m => m.id === library.effective);
  root.append(text("p", `${t("current")}: ${effective?.name || library.effective} · ${t(effective?.noo ? "noo" : "no_noo")}`));
  const label = text("label", t("next"));
  const select = document.createElement("select");
  select.setAttribute("aria-label", t("next"));
  for (const model of library.models.filter(m => !m.hidden)) {
    const option = text("option", `${model.name} · ${t(model.noo ? "noo" : "no_noo")}`);
    option.value = model.id;
    select.append(option);
  }
  select.value = !force && library.models.some(m => m.id === oldChoice && !m.hidden) ? oldChoice : library.selected;
  label.append(select);
  const save = text("button", t("save")); save.type = "button"; save.className = "btn";
  save.addEventListener("click", () => action("select", select.value));
  root.append(label, save, text("p", t("shared")));
  if (library.reboot_required) root.append(text("p", t("saved")));
  for (const model of library.models) {
    const row = document.createElement("div"); row.className = "model-library__row";
    row.append(text("strong", `${model.name}${model.hidden ? ` (${t("hidden")})` : ""}`));
    row.append(text("span", `eGPU ${mb(model.egpu_bytes)} · ${model.jetson_bytes === null ? t("unknown") : `Jetson ${mb(model.jetson_bytes)}`}`));
    if (model.delete_pending) row.append(text("small", t("pending")));
    const button = text("button", t(model.hidden ? "restore" : model.protected ? "protected" : "remove"));
    button.type = "button"; button.className = "btn"; button.disabled = !model.hidden && model.protected;
    button.addEventListener("click", () => action(model.hidden ? "restore" : "delete", model.id));
    row.append(button); root.append(row);
  }
}
