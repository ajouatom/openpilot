"use strict";

import { getWebSettingByKey, setWebSettingsByKeys } from "./state.js";

// Registry for compound preference controls. Generic bool/enum rows remain in
// The generic renderer handles bool/enum rows; multi-key controls register here independently.
globalThis.WebSettingsComponents = (() => {
  const components = new Map();

  function register(name, definition) {
    const key = String(name || "").trim();
    if (!key || !definition || typeof definition.render !== "function") {
      throw new TypeError("WebSettingsComponents.register requires a renderable component");
    }
    if (components.has(key)) throw new Error(`Web settings component already registered: ${key}`);
    components.set(key, Object.freeze({ ...definition }));
  }

  function definition(name) {
    return components.get(String(name || "")) || null;
  }

  function isVisible(name, fields) {
    const component = definition(name);
    if (!component) return false;
    return (component.settingKeys || []).every((key) => Boolean(fields[key]));
  }

  function render(name, item) {
    return definition(name)?.render(item) || "";
  }

  function bind(root = document) {
    for (const component of components.values()) component.bind?.(root);
  }

  return Object.freeze({ register, isVisible, render, bind });
})();

globalThis.WebSettingsComponents.register("web-upload", {
  settingKeys: ["log_upload_target", "web_upload_url", "web_upload_token", "toss_upload_url", "toss_upload_token"],
  render() {
    const text = (key, fallback) => getUIText(key) || fallback;
    const target = String(getWebSettingByKey("log_upload_target", "carrot") || "carrot");
    const field = (targetName, key, type, titleKey, titleFallback, descKey, descFallback) => `
      <label class="web-settings-row web-settings-row--field" data-web-upload-target-fields="${targetName}" ${target === targetName ? "" : "hidden"}>
        <span class="web-settings-row__copy">
          <span class="web-settings-row__title">${escapeHtml(text(titleKey, titleFallback))}</span>
          <span class="web-settings-row__desc">${escapeHtml(text(descKey, descFallback))}</span>
        </span>
        <input class="web-settings-text" data-web-upload-field="${key}" type="${type}"
          ${type === "url" ? "inputmode=\"url\"" : "autocomplete=\"new-password\""}
          value="${escapeHtml(String(getWebSettingByKey(key, "") || ""))}" />
      </label>`;
    return `
      <div class="web-upload-settings" data-upload-target="${escapeHtml(target)}">
        <label class="web-settings-row web-settings-row--field">
          <span class="web-settings-row__copy">
            <span class="web-settings-row__title">${escapeHtml(text("web_log_upload_target", "Upload server"))}</span>
            <span class="web-settings-row__desc">${escapeHtml(text("web_log_upload_target_desc", "Choose where logs are uploaded."))}</span>
          </span>
          <select class="web-settings-select" data-web-upload-target aria-label="${escapeHtml(text("web_log_upload_target", "Upload server"))}">
            <option value="carrot" ${target === "carrot" ? "selected" : ""}>${escapeHtml(text("web_log_upload_target_carrot", "Carrot server"))}</option>
            <option value="toss" ${target === "toss" ? "selected" : ""}>${escapeHtml(text("web_log_upload_target_toss", "Toss server"))}</option>
          </select>
        </label>
        ${field("carrot", "web_upload_url", "url", "web_upload_url", "Carrot upload server", "web_upload_url_desc", "Carrot HTTPS API base URL")}
        ${field("carrot", "web_upload_token", "password", "web_upload_token", "Carrot access token", "web_upload_token_desc", "Bearer token issued by the Carrot server")}
        ${field("toss", "toss_upload_url", "url", "web_toss_upload_url", "Toss server URL", "web_toss_upload_url_desc", "Toss HTTPS API base URL")}
        ${field("toss", "toss_upload_token", "password", "web_toss_upload_token", "Toss access token", "web_toss_upload_token_desc", "Bearer token issued by the Toss server")}
        <div class="web-upload-settings__actions">
          <button class="btn btn-secondary web-upload-settings__test" type="button">${escapeHtml(text("web_upload_test", "Test connection"))}</button>
          <span class="web-upload-settings__status" aria-live="polite"></span>
        </div>
      </div>`;
  },
  bind(root = document) {
    const container = root.querySelector(".web-upload-settings");
    if (!container || container.dataset.bound === "1") return;
    container.dataset.bound = "1";
    const targetSelect = container.querySelector("[data-web-upload-target]");
    const fields = [...container.querySelectorAll("[data-web-upload-field]")];
    const status = container.querySelector(".web-upload-settings__status");
    const button = container.querySelector(".web-upload-settings__test");
    const currentTarget = () => targetSelect?.value === "toss" ? "toss" : "carrot";
    const syncTargetVisibility = () => {
      const target = currentTarget();
      container.dataset.uploadTarget = target;
      container.querySelectorAll("[data-web-upload-target-fields]").forEach((row) => {
        row.hidden = row.dataset.webUploadTargetFields !== target;
      });
    };
    const selectedValues = () => {
      const target = currentTarget();
      const values = { log_upload_target: target };
      fields.filter((input) => input.closest("[data-web-upload-target-fields]")?.dataset.webUploadTargetFields === target)
        .forEach((input) => { values[input.dataset.webUploadField] = input.value.trim(); });
      return values;
    };
    const saveSelected = () => setWebSettingsByKeys(selectedValues());

    targetSelect?.addEventListener("change", () => {
      syncTargetVisibility();
      saveSelected().catch((err) => {
        if (status) status.textContent = err?.message || String(err);
      });
    });
    fields.forEach((input) => {
      input.addEventListener("change", () => setWebSettingsByKeys({ [input.dataset.webUploadField]: input.value.trim() }).catch((err) => {
        if (status) status.textContent = err?.message || String(err);
      }));
    });
    button?.addEventListener("click", async () => {
      button.disabled = true;
      if (status) status.textContent = getUIText("web_upload_testing") || "Testing...";
      try {
        await saveSelected();
        const response = await fetch("/api/dashcam/upload/test", { method: "POST" });
        const payload = await response.json();
        if (!response.ok || payload?.ok === false) throw new Error(payload?.error || `HTTP ${response.status}`);
        const targetLabel = currentTarget() === "toss"
          ? (getUIText("web_log_upload_target_toss") || "Toss server")
          : (getUIText("web_log_upload_target_carrot") || "Carrot server");
        if (status) status.textContent = `${targetLabel}: ${getUIText("web_upload_test_ok") || "Connection OK"}`;
      } catch (err) {
        if (status) status.textContent = `${getUIText("web_upload_test_failed") || "Connection failed"}: ${err?.message || err}`;
      } finally {
        button.disabled = false;
      }
    });
    syncTargetVisibility();
  },
});
