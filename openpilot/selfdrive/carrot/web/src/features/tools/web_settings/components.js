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
  settingKeys: ["web_upload_url", "web_upload_token"],
  render() {
    const text = (key, fallback) => getUIText(key) || fallback;
    const url = String(getWebSettingByKey("web_upload_url", "") || "");
    const token = String(getWebSettingByKey("web_upload_token", "") || "");
    return `
      <div class="web-upload-settings">
        <label class="web-settings-row web-settings-row--field">
          <span class="web-settings-row__copy">
            <span class="web-settings-row__title">${escapeHtml(text("web_upload_url", "Upload server"))}</span>
            <span class="web-settings-row__desc">${escapeHtml(text("web_upload_url_desc", "HTTPS API base URL"))}</span>
          </span>
          <input class="web-settings-text" data-web-upload-field="web_upload_url" type="url" inputmode="url" value="${escapeHtml(url)}" />
        </label>
        <label class="web-settings-row web-settings-row--field">
          <span class="web-settings-row__copy">
            <span class="web-settings-row__title">${escapeHtml(text("web_upload_token", "Access token"))}</span>
            <span class="web-settings-row__desc">${escapeHtml(text("web_upload_token_desc", "Bearer token issued by the upload server"))}</span>
          </span>
          <input class="web-settings-text" data-web-upload-field="web_upload_token" type="password" autocomplete="new-password" value="${escapeHtml(token)}" />
        </label>
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
    const fields = [...container.querySelectorAll("[data-web-upload-field]")];
    const status = container.querySelector(".web-upload-settings__status");
    const button = container.querySelector(".web-upload-settings__test");
    const values = () => Object.fromEntries(fields.map((input) => [input.dataset.webUploadField, input.value.trim()]));
    const save = () => setWebSettingsByKeys(values());

    fields.forEach((input) => {
      input.addEventListener("change", () => save().catch((err) => {
        if (status) status.textContent = err?.message || String(err);
      }));
    });
    button?.addEventListener("click", async () => {
      button.disabled = true;
      if (status) status.textContent = getUIText("web_upload_testing") || "Testing...";
      try {
        await save();
        const response = await fetch("/api/dashcam/upload/test", { method: "POST" });
        const payload = await response.json();
        if (!response.ok || payload?.ok === false) throw new Error(payload?.error || `HTTP ${response.status}`);
        if (status) status.textContent = getUIText("web_upload_test_ok") || "Connection OK";
      } catch (err) {
        if (status) status.textContent = `${getUIText("web_upload_test_failed") || "Connection failed"}: ${err?.message || err}`;
      } finally {
        button.disabled = false;
      }
    });
  },
});
