"use strict";

function settingFormEscape(value) {
  if (typeof escapeHtml === "function") return escapeHtml(value);
  return String(value ?? "").replace(/[&<>"']/g, (ch) => ({
    "&": "&amp;",
    "<": "&lt;",
    ">": "&gt;",
    '"': "&quot;",
    "'": "&#39;",
  }[ch]));
}

function renderSettingFormControl(options = {}) {
  const value = String(options.value || options.placeholder || "-");
  const label = String(options.label || "");
  const editAction = String(options.editAction || "edit");
  const actionAttribute = options.actionAttribute === "data-ssh-action" ? "data-ssh-action" : "data-action";
  const stateClass = options.configured ? " is-configured" : "";
  const actions = Array.isArray(options.actions) ? options.actions : [];

  return `
    <div class="setting-form-shell">
      <div class="setting-form-control">
        <button class="setting-form-control__surface${stateClass}" type="button" ${actionAttribute}="${settingFormEscape(editAction)}" aria-label="${settingFormEscape(label)}">
          <span class="setting-form-control__value">${settingFormEscape(value)}</span>
        </button>
        <div class="setting-form-control__actions">
          ${actions.map((action) => `
            <button class="smallBtn setting-form-control__action" type="button" ${actionAttribute}="${settingFormEscape(action.action)}" ${action.disabled ? "disabled" : ""}>${settingFormEscape(action.label)}</button>
          `).join("")}
        </div>
      </div>
    </div>`;
}

function openSettingFormDialog(options = {}) {
  return appForm("", {
    title: options.title,
    defaultValue: options.defaultValue ?? "",
    placeholder: options.placeholder || "",
    inputType: options.inputType || "text",
    autocomplete: options.autocomplete || "off",
    autocapitalize: options.autocapitalize || "none",
    inputMode: options.inputMode,
    spellcheck: options.spellcheck === true,
    confirmLabel: options.saveLabel || (typeof getUIText === "function" ? getUIText("save", "Save") : "Save"),
    cancelLabel: options.cancelLabel || (typeof getUIText === "function" ? getUIText("cancel", "Cancel") : "Cancel"),
    submittingLabel: options.savingLabel || (typeof getUIText === "function" ? getUIText("saving", "Saving...") : "Saving..."),
    onSubmit: options.onSave,
  });
}
