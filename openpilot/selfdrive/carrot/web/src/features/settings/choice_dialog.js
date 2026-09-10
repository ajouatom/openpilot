const SETTING_CHOICE_LAYOUTS = Object.freeze({
  LIST: "list",
  VALUE_GRID: "value-grid",
});

function configuredChoiceLayout(setting) {
  const value = String(
    setting?.choice_layout
    ?? setting?.choiceLayout
    ?? "",
  ).trim().toLowerCase();
  if (["list", "action-list", "actions"].includes(value)) return SETTING_CHOICE_LAYOUTS.LIST;
  if (["grid", "value-grid", "values"].includes(value)) return SETTING_CHOICE_LAYOUTS.VALUE_GRID;
  return "";
}

function hasLocalizedChoiceLabels(setting) {
  const options = setting?.options;
  return Boolean(options && typeof options === "object"
    && Object.values(options).some((labels) => Array.isArray(labels) && labels.length > 0));
}

/**
 * Semantic options need room for every locale and therefore render as stacked
 * action rows. A value grid is reserved for raw numeric ranges. A future
 * setting may opt in explicitly with `choice_layout` after its labels are
 * checked at narrow widths.
 */
export function resolveSettingChoiceLayout(setting, context = {}) {
  const configured = configuredChoiceLayout(setting);
  if (configured) return configured;

  const name = String(context.name ?? setting?.name ?? "").trim();
  if (name === "SoundLanguageSetting" || hasLocalizedChoiceLabels(setting)) {
    return SETTING_CHOICE_LAYOUTS.LIST;
  }
  return SETTING_CHOICE_LAYOUTS.VALUE_GRID;
}

export function installSettingsChoiceDialogFacade(target = globalThis) {
  if (!target || (typeof target !== "object" && typeof target !== "function")) {
    throw new TypeError("A facade target is required");
  }
  const currentNamespace = target.CarrotUI && typeof target.CarrotUI === "object"
    ? target.CarrotUI
    : null;
  const carrotUI = currentNamespace && Object.isExtensible(currentNamespace)
    ? currentNamespace
    : { ...(currentNamespace || {}) };
  carrotUI.settingsChoiceDialog = Object.freeze({
    layouts: SETTING_CHOICE_LAYOUTS,
    resolveLayout: resolveSettingChoiceLayout,
  });
  target.CarrotUI = carrotUI;
  return carrotUI.settingsChoiceDialog;
}
