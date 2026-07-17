"use strict";

(() => {
  const spec = globalThis.CarrotDriveLayoutSpec;
  const registry = globalThis.WebSettingsComponents;
  if (!spec || !registry) return;

  const COMPONENT_NAME = "drive-layout";
  const VIEW_OPTIONS = Object.freeze({
    orientation: Object.freeze([
      Object.freeze({ value: spec.ORIENTATION.HORIZONTAL, labelKey: "web_drive_layout_horizontal" }),
      Object.freeze({ value: spec.ORIENTATION.VERTICAL, labelKey: "web_drive_layout_vertical" }),
    ]),
    mode: Object.freeze([
      Object.freeze({ value: spec.MODE.SPLIT, labelKey: "web_drive_layout_mode_split" }),
      Object.freeze({ value: spec.MODE.AREA_1, labelKey: "web_drive_layout_mode_area_1" }),
      Object.freeze({ value: spec.MODE.AREA_2, labelKey: "web_drive_layout_mode_area_2" }),
    ]),
    content: Object.freeze([
      Object.freeze({ value: spec.CONTENT.VISION, labelKey: "web_drive_layout_content_vision" }),
      Object.freeze({ value: spec.CONTENT.NAVIGATION, labelKey: "web_drive_layout_content_navigation" }),
    ]),
  });
  const CONTENT_LABEL_KEYS = Object.freeze({
    [spec.CONTENT.VISION]: "web_drive_layout_content_vision",
    [spec.CONTENT.NAVIGATION]: "web_drive_layout_content_navigation",
  });
  let selectedOrientation = spec.ORIENTATION.HORIZONTAL;

  function renderButtons(control, options) {
    return options.map((option) => `
      <button type="button" data-drive-layout-control="${escapeHtml(control)}" data-value="${escapeHtml(option.value)}" aria-pressed="false">
        ${escapeHtml(option.label || webSettingsText(option.labelKey))}
      </button>`).join("");
  }

  function renderContentOptions() {
    return VIEW_OPTIONS.content.map((option) => `
      <option value="${escapeHtml(option.value)}">${escapeHtml(webSettingsText(option.labelKey))}</option>`).join("");
  }

  function render() {
    return `
      <section class="drive-layout-setting" data-web-settings-component="${COMPONENT_NAME}">
        <header class="drive-layout-setting__header">
          <h3>${escapeHtml(webSettingsText("web_drive_layout_title"))}</h3>
          <div class="drive-layout-segmented drive-layout-segmented--compact" role="group" aria-label="${escapeHtml(webSettingsText("web_drive_layout_orientation"))}">
            ${renderButtons("orientation", VIEW_OPTIONS.orientation)}
          </div>
        </header>

        <div class="drive-layout-preview" data-drive-layout-preview>
          <div class="drive-layout-preview__area" data-area="area_1">
            <span>${escapeHtml(webSettingsText("web_drive_layout_area_1"))}</span>
            <strong data-content-label></strong>
          </div>
          <div class="drive-layout-preview__divider" aria-hidden="true"></div>
          <div class="drive-layout-preview__area" data-area="area_2">
            <span>${escapeHtml(webSettingsText("web_drive_layout_area_2"))}</span>
            <strong data-content-label></strong>
          </div>
        </div>

        <div class="drive-layout-content-selects">
          <label>
            <span>${escapeHtml(webSettingsText("web_drive_layout_area_1_start"))}</span>
            <select data-drive-layout-content="area1Content" aria-label="${escapeHtml(webSettingsText("web_drive_layout_area_1_start"))}">
              ${renderContentOptions()}
            </select>
          </label>
          <button type="button" class="drive-layout-content-swap" data-drive-layout-swap
                  aria-label="${escapeHtml(webSettingsText("web_drive_layout_swap"))}"
                  title="${escapeHtml(webSettingsText("web_drive_layout_swap"))}">
            <span aria-hidden="true">⇄</span>
          </button>
          <label>
            <span>${escapeHtml(webSettingsText("web_drive_layout_area_2_start"))}</span>
            <select data-drive-layout-content="area2Content" aria-label="${escapeHtml(webSettingsText("web_drive_layout_area_2_start"))}">
              ${renderContentOptions()}
            </select>
          </label>
        </div>

        <div class="drive-layout-setting__controls">
          <div class="drive-layout-control-row">
            <span>${escapeHtml(webSettingsText("web_drive_layout_display"))}</span>
            <div class="drive-layout-segmented" role="group" aria-label="${escapeHtml(webSettingsText("web_drive_layout_display"))}">
              ${renderButtons("mode", VIEW_OPTIONS.mode)}
            </div>
          </div>
        </div>

      </section>`;
  }

  function setPressed(root, control, value) {
    root.querySelectorAll(`[data-drive-layout-control="${control}"]`).forEach((button) => {
      button.setAttribute("aria-pressed", String(button.dataset.value === String(value)));
    });
  }

  function syncArea(root, area, content) {
    const element = root.querySelector(`[data-area="${area}"]`);
    if (!element) return;
    element.dataset.content = content;
    const label = element.querySelector("[data-content-label]");
    if (label) label.textContent = webSettingsText(CONTENT_LABEL_KEYS[content]);
  }

  function syncContentSelect(root, property, content, unavailableContent) {
    const select = root.querySelector(`[data-drive-layout-content="${property}"]`);
    if (!select) return;
    select.value = content;
    Array.from(select.options).forEach((option) => {
      option.disabled = option.value === unavailableContent;
    });
  }

  function sync(root) {
    const layout = spec.read(window.CarrotWebSettingsState, selectedOrientation);
    root.dataset.orientation = layout.orientation;
    syncArea(root, "area_1", layout.area1Content);
    syncArea(root, "area_2", layout.area2Content);
    syncContentSelect(root, "area1Content", layout.area1Content, layout.area2Content);
    syncContentSelect(root, "area2Content", layout.area2Content, layout.area1Content);
    const swapIcon = root.querySelector("[data-drive-layout-swap] span");
    if (swapIcon) swapIcon.textContent = layout.orientation === spec.ORIENTATION.VERTICAL ? "⇅" : "⇄";
    setPressed(root, "orientation", layout.orientation);
    setPressed(root, "mode", layout.mode);
  }

  function reportFailure(error) {
    if (typeof showAppToast === "function") showAppToast(error?.message || String(error), { tone: "error" });
  }

  function bind(root) {
    root.querySelectorAll(`[data-web-settings-component="${COMPONENT_NAME}"]`).forEach((component) => {
      if (component.dataset.webSettingsBound === "1") return;
      component.dataset.webSettingsBound = "1";
      let pending = false;

      component.querySelectorAll("[data-drive-layout-control]").forEach((button) => {
        button.addEventListener("click", async () => {
          if (pending) return;
          const control = button.dataset.driveLayoutControl;
          const value = button.dataset.value;
          if (control === "orientation") {
            selectedOrientation = spec.normalizeOrientation(value);
            sync(component);
            return;
          }
          const keys = spec.keysFor(selectedOrientation);
          pending = true;
          try {
            const save = setWebSettingByKey(keys.mode, value);
            sync(component);
            await save;
          } catch (error) {
            reportFailure(error);
          } finally {
            pending = false;
            sync(component);
          }
        });
      });

      component.querySelectorAll("[data-drive-layout-content]").forEach((select) => {
        select.addEventListener("change", async () => {
          if (pending) return;
          const property = select.dataset.driveLayoutContent;
          const otherProperty = property === "area1Content" ? "area2Content" : "area1Content";
          const keys = spec.keysFor(selectedOrientation);
          const layout = spec.read(window.CarrotWebSettingsState, selectedOrientation);
          const nextContent = select.value;
          if (nextContent === layout[otherProperty]) {
            sync(component);
            return;
          }
          pending = true;
          try {
            const save = setWebSettingByKey(keys[property], nextContent);
            sync(component);
            await save;
          } catch (error) {
            reportFailure(error);
          } finally {
            pending = false;
            sync(component);
          }
        });
      });

      component.querySelector("[data-drive-layout-swap]")?.addEventListener("click", async () => {
        if (pending) return;
        const keys = spec.keysFor(selectedOrientation);
        const layout = spec.read(window.CarrotWebSettingsState, selectedOrientation);
        pending = true;
        try {
          const save = setWebSettingsByKeys({
            [keys.area1Content]: layout.area2Content,
            [keys.area2Content]: layout.area1Content,
          });
          sync(component);
          await save;
        } catch (error) {
          reportFailure(error);
        } finally {
          pending = false;
          sync(component);
        }
      });

      sync(component);
    });
  }

  registry.register(COMPONENT_NAME, {
    settingKeys: spec.settingKeys,
    render,
    bind,
  });
})();
