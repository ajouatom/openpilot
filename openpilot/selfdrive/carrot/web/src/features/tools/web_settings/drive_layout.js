"use strict";

import { webSettingsText } from "./render.js";
import { setWebSettingByKey, setWebSettingsByKeys } from "./state.js";

(() => {
  const spec = globalThis.CarrotDriveLayoutSpec;
  const componentRegistry = globalThis.WebSettingsComponents;
  if (!spec || !componentRegistry) return;

  const COMPONENT_NAME = "drive-layout";
  const SETTINGS_SOURCE = "live";
  const UNAVAILABLE_LABEL_KEY = "drive_content_status_unavailable";
  const CONTENT_FIELDS = Object.freeze({
    area1Content: Object.freeze({ slot: "primary", fallbackId: "vision" }),
    area2Content: Object.freeze({ slot: "secondary", fallbackId: "navigation" }),
  });
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
  });
  function viewportOrientation() {
    return typeof spec.orientationForViewport === "function"
      ? spec.orientationForViewport(globalThis, document)
      : spec.ORIENTATION.HORIZONTAL;
  }

  let selectedOrientation = viewportOrientation();

  function renderButtons(control, options) {
    return options.map((option) => `
      <button type="button" data-drive-layout-control="${escapeHtml(control)}" data-value="${escapeHtml(option.value)}" aria-pressed="false">
        ${escapeHtml(option.label || webSettingsText(option.labelKey))}
      </button>`).join("");
  }

  function driveContentRegistry() {
    return globalThis.DriveContentRegistry || null;
  }

  function contentDescriptor(content) {
    try {
      return driveContentRegistry()?.get?.(content) || null;
    } catch (_) {
      return null;
    }
  }

  function contentLabel(content) {
    const descriptor = contentDescriptor(content);
    return descriptor?.labelKey
      ? webSettingsText(descriptor.labelKey)
      : webSettingsText(UNAVAILABLE_LABEL_KEY);
  }

  function listContentOptions(property) {
    const field = CONTENT_FIELDS[property];
    const registry = driveContentRegistry();
    if (!field || typeof registry?.list !== "function") return [];
    try {
      const descriptors = registry.list({
        slot: field.slot,
        source: SETTINGS_SOURCE,
        fallbackId: field.fallbackId,
        selectedIds: [],
      });
      return (Array.isArray(descriptors) ? descriptors : [])
        .filter((descriptor) => descriptor?.id && descriptor?.labelKey);
    } catch (_) {
      return [];
    }
  }

  function renderContentOptions(options, selectedContent = "", unavailableContent = "") {
    const rendered = options.map((option) => {
      const unavailable = option.singleton === true && option.id === unavailableContent;
      return `
      <option value="${escapeHtml(option.id)}" ${option.id === selectedContent ? "selected" : ""} ${unavailable ? 'disabled data-unavailable="true"' : ""}>
        ${escapeHtml(webSettingsText(option.labelKey))}
      </option>`;
    });
    if (selectedContent && !options.some((option) => option.id === selectedContent)) {
      const descriptor = contentDescriptor(selectedContent);
      const label = descriptor?.labelKey
        ? webSettingsText(descriptor.labelKey)
        : String(selectedContent);
      rendered.unshift(`
        <option value="${escapeHtml(selectedContent)}" selected disabled data-unavailable="true">
          ${escapeHtml(`${label} (${webSettingsText(UNAVAILABLE_LABEL_KEY)})`)}
        </option>`);
    }
    if (!rendered.length) {
      rendered.push(`
        <option value="" selected disabled data-unavailable="true">
          ${escapeHtml(webSettingsText(UNAVAILABLE_LABEL_KEY))}
        </option>`);
    }
    return rendered.join("");
  }

  function readLayout() {
    try {
      return spec.read(window.CarrotWebSettingsState, selectedOrientation);
    } catch (_) {
      return null;
    }
  }

  function render() {
    selectedOrientation = viewportOrientation();
    const layout = readLayout();
    const area1Content = layout?.area1Content || "";
    const area2Content = layout?.area2Content || "";
    const area1Options = listContentOptions("area1Content");
    const area2Options = listContentOptions("area2Content");
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
            <select data-drive-layout-content="area1Content" aria-label="${escapeHtml(webSettingsText("web_drive_layout_area_1_start"))}" ${area1Options.length ? "" : "disabled"}>
              ${renderContentOptions(area1Options, area1Content, area2Content)}
            </select>
          </label>
          <button type="button" class="drive-layout-content-swap" data-drive-layout-swap
                  aria-label="${escapeHtml(webSettingsText("web_drive_layout_swap"))}"
                  title="${escapeHtml(webSettingsText("web_drive_layout_swap"))}">
            <span aria-hidden="true">⇄</span>
          </button>
          <label>
            <span>${escapeHtml(webSettingsText("web_drive_layout_area_2_start"))}</span>
            <select data-drive-layout-content="area2Content" aria-label="${escapeHtml(webSettingsText("web_drive_layout_area_2_start"))}" ${area2Options.length ? "" : "disabled"}>
              ${renderContentOptions(area2Options, area2Content, area1Content)}
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
    if (label) label.textContent = contentLabel(content);
  }

  function syncContentSelect(root, property, content, unavailableContent) {
    const select = root.querySelector(`[data-drive-layout-content="${property}"]`);
    if (!select) return;
    const options = listContentOptions(property);
    select.innerHTML = renderContentOptions(options, content, unavailableContent);
    select.disabled = options.length === 0;
    select.value = content;
  }

  function sync(root) {
    const layout = readLayout();
    if (!layout) {
      root.style.removeProperty("--drive-layout-preview-area-1-size");
      root.style.removeProperty("--drive-layout-preview-area-2-size");
      root.dataset.orientation = selectedOrientation;
      root.dataset.mode = "";
      root.dataset.contentUnavailable = "true";
      syncArea(root, "area_1", "");
      syncArea(root, "area_2", "");
      syncContentSelect(root, "area1Content", "", "");
      syncContentSelect(root, "area2Content", "", "");
      setPressed(root, "orientation", selectedOrientation);
      setPressed(root, "mode", "");
      return;
    }
    delete root.dataset.contentUnavailable;
    root.style.setProperty("--drive-layout-preview-area-1-size", `${layout.ratio}fr`);
    root.style.setProperty(
      "--drive-layout-preview-area-2-size",
      `${Number((1 - layout.ratio).toFixed(2))}fr`,
    );
    root.dataset.orientation = layout.orientation;
    root.dataset.mode = layout.mode;
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
          const keys = spec.keysFor(selectedOrientation);
          const nextContent = select.value;
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
        const layout = readLayout();
        if (!layout) {
          sync(component);
          return;
        }
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

  componentRegistry.register(COMPONENT_NAME, {
    settingKeys: spec.settingKeys,
    render,
    bind,
  });
})();
