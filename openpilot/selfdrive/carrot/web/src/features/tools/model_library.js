"use strict";
const EN = {
  title: "External driving model", internal: "Internal model: CD210 (fixed)",
  help: "One choice for eGPU and Jetson. Choosing saves the next-boot selection. Reboot comma and Jetson while parked with ignition off and internet available. First use downloads files; Jetson also prepares its engine.",
  current: "This boot", next: "Next boot", saved: "Saved. Reboot comma and Jetson to apply.",
  noo: "NOO supported", no_noo: "NOO not supported", shared: "N16 supports navigation inputs. Flip Flop and Light speed do not.",

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
  const signature = JSON.stringify([library, t("title")]);
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
  root.append(label, text("p", t("shared")));
  if (library.reboot_required) root.append(text("p", t("saved")));
}
