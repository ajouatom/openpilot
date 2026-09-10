import { mountOnnxVisionPanel } from "./panel.js";

const ONNX_SETTING_NAME = "ShareData";

/**
 * Owns the supplementary panel shown only inside the ShareData detail screen.
 * The service remains on 8082 behind the existing same-origin gateway.
 */
export function registerOnnxVisionSettingsExtension(registry) {
  return registry.register({
    id: "onnx-vision",
    matches(context) {
      return context.group === "STEER"
        && context.detailMode
        && context.detailName === ONNX_SETTING_NAME
        && Boolean(context.root?.querySelector?.(`[data-setting-name="${ONNX_SETTING_NAME}"]`));
    },
    mount(context) {
      const setting = context.root.querySelector(`[data-setting-name="${ONNX_SETTING_NAME}"]`);
      if (!setting) return null;
      setting.dataset.settingsExtension = "onnx-vision";
      setting.dataset.settingsExtensionPhase = "migrated";
      const panel = mountOnnxVisionPanel({ root: context.root, setting, lifecycle: context.lifecycle });
      return {
        ...panel,
        destroy() {
          panel.destroy();
          setting.removeAttribute("data-settings-extension");
          setting.removeAttribute("data-settings-extension-phase");
        },
      };
    },
  });
}
