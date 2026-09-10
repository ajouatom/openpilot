import { loadOnnxVisionSnapshot } from "../api.js";

function text(key, fallback) {
  return typeof globalThis.getUIText === "function" ? globalThis.getUIText(key, fallback) : fallback;
}

export async function openOnnxSnapshotDialog(stream) {
  const isRoad = stream === "road";
  const title = isRoad
    ? text("onnx_vision_road_camera", "Road camera")
    : text("onnx_vision_wide_camera", "Wide camera");
  const completion = globalThis.appAlert?.("", {
    title,
    html: true,
    messageHtml: `
      <div class="onnx-snapshot-dialog" data-onnx-snapshot-dialog>
        <div class="onnx-snapshot-dialog__surface">
          <img alt="" data-role="image" hidden>
          <div class="onnx-snapshot-dialog__state" data-role="state" role="status" aria-live="polite"></div>
        </div>
        <button type="button" class="smallBtn" data-role="refresh"></button>
      </div>
    `,
    confirmLabel: text("close", "Close"),
  });
  if (!completion) return;

  const root = globalThis.document?.querySelector?.("[data-onnx-snapshot-dialog]");
  if (!root) return completion;
  const image = root.querySelector('[data-role="image"]');
  const state = root.querySelector('[data-role="state"]');
  const refresh = root.querySelector('[data-role="refresh"]');
  refresh.textContent = text("onnx_vision_refresh_image", "Refresh image");
  let activeRequest = null;
  let objectUrl = "";
  let closed = false;

  async function load() {
    activeRequest?.abort();
    activeRequest = new AbortController();
    refresh.disabled = true;
    state.dataset.tone = "info";
    state.textContent = text("onnx_vision_snapshot_loading", "Loading camera image...");
    try {
      const blob = await loadOnnxVisionSnapshot(isRoad ? "road" : "wide", globalThis, {
        signal: activeRequest.signal,
      });
      if (closed) return;
      if (objectUrl) globalThis.URL?.revokeObjectURL?.(objectUrl);
      objectUrl = globalThis.URL?.createObjectURL?.(blob) || "";
      if (!objectUrl) throw new Error("Camera image preview is unavailable");
      image.src = objectUrl;
      image.hidden = false;
      state.dataset.tone = "success";
      state.textContent = text("onnx_vision_snapshot_ready", "Current camera image");
    } catch (error) {
      if (error?.name === "AbortError" || closed) return;
      state.dataset.tone = "error";
      state.textContent = error?.message || text("onnx_vision_snapshot_failed", "Could not load the camera image.");
    } finally {
      if (!closed) refresh.disabled = false;
    }
  }

  refresh.addEventListener("click", load);
  load();
  try {
    await completion;
  } finally {
    closed = true;
    activeRequest?.abort();
    if (objectUrl) globalThis.URL?.revokeObjectURL?.(objectUrl);
  }
}
