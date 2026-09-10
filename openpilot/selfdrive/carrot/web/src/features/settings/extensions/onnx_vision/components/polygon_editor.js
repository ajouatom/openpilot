import {
  loadOnnxVisionConfig,
  loadOnnxVisionSnapshot,
  restoreOnnxVisionConfig,
  saveOnnxVisionConfig,
} from "../api.js";
import { fallbackRoadSceneMarkup, layoutFallbackRoadScene } from "./fallback_road_scene.js";
import { onnxSnapshotCache } from "./snapshot_cache.js";
import { openOnnxSnapshotDialog } from "./snapshot_dialog.js";

const SVG_NS = "http://www.w3.org/2000/svg";
const MAX_HISTORY = 40;

function text(key, fallback, vars) {
  if (typeof globalThis.getUIText === "function") return globalThis.getUIText(key, fallback, vars);
  let value = fallback;
  Object.entries(vars || {}).forEach(([name, replacement]) => {
    value = value.replaceAll(`{${name}}`, String(replacement));
  });
  return value;
}

export function canvasPointFromEvent(event, surface, width, height) {
  const bounds = surface.getBoundingClientRect();
  const x = Math.round((Number(event.clientX) - bounds.left) * width / Math.max(bounds.width, 1));
  const y = Math.round((Number(event.clientY) - bounds.top) * height / Math.max(bounds.height, 1));
  return [
    Math.min(Math.max(x, 0), Math.max(width - 1, 0)),
    Math.min(Math.max(y, 0), Math.max(height - 1, 0)),
  ];
}

export function mirroredPolygon(polygon, width) {
  const maxX = Math.max(0, Math.round(Number(width) || 1) - 1);
  return [...(Array.isArray(polygon) ? polygon : [])]
    .reverse()
    .map(([x, y]) => [Math.min(Math.max(maxX - Math.round(Number(x) || 0), 0), maxX), Math.round(Number(y) || 0)]);
}

export function movePolygonPoint(point, dx, dy, width, height) {
  const maxX = Math.max(0, Math.round(Number(width) || 1) - 1);
  const maxY = Math.max(0, Math.round(Number(height) || 1) - 1);
  return [
    Math.min(Math.max(Math.round(Number(point?.[0]) || 0) + Math.round(Number(dx) || 0), 0), maxX),
    Math.min(Math.max(Math.round(Number(point?.[1]) || 0) + Math.round(Number(dy) || 0), 0), maxY),
  ];
}

export function polygonPointLabel(index) {
  return String(Math.max(0, Math.round(Number(index) || 0)) + 1);
}

export function polygonCanvasGesture({ hasSelectedPoint = false } = {}) {
  if (hasSelectedPoint) return "move-selected-point";
  return "add-point";
}

export function polygonHandleScreenMetrics(renderedWidth) {
  const compact = Math.max(0, Number(renderedWidth) || 0) < 520;
  return Object.freeze({
    dotRadius: compact ? 4.5 : 5,
    labelSize: compact ? 12 : 13,
    labelHeight: compact ? 20 : 22,
    labelWidth: compact ? 24 : 26,
    labelWideWidth: compact ? 30 : 32,
    labelGap: 5,
    labelCornerRadius: 5,
    hitRadius: 22,
    hitInset: 4,
    selectionGap: 3.5,
  });
}

export function isPolygonConfigSavable(config) {
  const valid = (polygon) => Array.isArray(polygon) && (polygon.length === 0 || polygon.length >= 3) && polygon.length <= 64;
  return valid(config?.poly_left) && valid(config?.poly_right)
    && (config.poly_left.length > 0 || config.poly_right.length > 0);
}

export function hasFreshOnnxVisionSnapshot(snapshotState) {
  return snapshotState === "ready";
}

function editableConfig(value) {
  const width = Math.max(1, Math.round(Number(value?.width) || 1928));
  const height = Math.max(1, Math.round(Number(value?.height) || 1208));
  const points = (side) => (Array.isArray(value?.[`poly_${side}`]) ? value[`poly_${side}`] : [])
    .filter((point) => Array.isArray(point) && point.length === 2)
    .map((point) => movePolygonPoint(point, 0, 0, width, height));
  return { width, height, poly_left: points("left"), poly_right: points("right") };
}

function sameConfig(left, right) {
  return JSON.stringify(editableConfig(left)) === JSON.stringify(editableConfig(right));
}

function pointList(polygon) {
  return polygon.map(([x, y]) => `${x},${y}`).join(" ");
}

function createSvgElement(name, attributes = {}) {
  const element = globalThis.document.createElementNS(SVG_NS, name);
  Object.entries(attributes).forEach(([key, value]) => element.setAttribute(key, String(value)));
  return element;
}

function bindRepeatButton(button, action) {
  let delayTimer = null;
  let repeatTimer = null;
  const clear = () => {
    if (delayTimer !== null) globalThis.clearTimeout(delayTimer);
    if (repeatTimer !== null) globalThis.clearInterval(repeatTimer);
    delayTimer = null;
    repeatTimer = null;
  };
  button.addEventListener("pointerdown", (event) => {
    if (event.button !== undefined && event.button !== 0) return;
    event.preventDefault();
    action();
    delayTimer = globalThis.setTimeout(() => {
      repeatTimer = globalThis.setInterval(action, 75);
    }, 350);
  });
  button.addEventListener("pointerup", clear);
  button.addEventListener("pointercancel", clear);
  button.addEventListener("pointerleave", clear);
  button.addEventListener("click", (event) => {
    if (event.detail === 0) action();
  });
  return clear;
}

function editorMarkup() {
  return `
    <div class="onnx-polygon-dialog" data-onnx-polygon-dialog>
      <div class="c-segmented-control c-segmented-control--equal onnx-polygon-dialog__sides" data-role="sides" role="group">
        <button type="button" class="c-segmented-control__item is-active" data-side="left" aria-pressed="true"></button>
        <button type="button" class="c-segmented-control__item" data-side="right" aria-pressed="false"></button>
      </div>
      <div class="onnx-polygon-dialog__workspace">
        <div class="onnx-polygon-dialog__canvas-wrap" data-role="viewport">
          ${fallbackRoadSceneMarkup()}
          <svg data-role="scene" tabindex="0" role="application">
            <rect class="onnx-polygon-dialog__backdrop" data-role="backdrop"></rect>
            <image data-role="image" preserveAspectRatio="none" hidden></image>
            <polygon class="onnx-polygon-dialog__polygon" data-role="polygon-left" data-polygon-side="left" aria-hidden="true"></polygon>
            <polygon class="onnx-polygon-dialog__polygon" data-role="polygon-right" data-polygon-side="right" aria-hidden="true"></polygon>
            <g data-role="handles"></g>
          </svg>
          <select class="onnx-polygon-dialog__point-select" data-role="point-select"></select>
          <div class="onnx-fallback-road__notice" data-role="fallback-notice"></div>
          <div class="onnx-polygon-dialog__state" data-role="state" role="status"></div>
        </div>
      </div>
      <div class="ui-action-grid ui-action-grid--fixed-four onnx-polygon-dialog__primary-actions">
        <button type="button" class="smallBtn" data-action="refresh"></button>
        <button type="button" class="smallBtn" data-action="undo"></button>
        <button type="button" class="smallBtn" data-action="restore"></button>
        <button type="button" class="smallBtn btn--filled" data-action="save"></button>
      </div>
      <div class="ui-action-grid onnx-polygon-dialog__camera-actions" role="group">
        <button type="button" class="smallBtn" data-camera="road"></button>
        <button type="button" class="smallBtn" data-camera="wide"></button>
      </div>
      <span class="onnx-polygon-dialog__save-state" data-role="save-state" aria-live="polite"></span>
      <details class="onnx-polygon-dialog__advanced" data-role="advanced" hidden>
        <summary>
          <span data-role="advanced-title"></span>
          <span data-role="advanced-summary"></span>
        </summary>
        <div class="onnx-polygon-dialog__advanced-body">
          <section class="onnx-polygon-dialog__point-panel">
            <div class="onnx-polygon-dialog__point-head">
              <div>
                <strong data-role="point-title"></strong>
                <span data-role="point-position"></span>
              </div>
              <button type="button" class="smallBtn onnx-polygon-dialog__step" data-action="step" aria-pressed="false"></button>
            </div>
            <div class="onnx-polygon-dialog__point-tools">
              <div class="onnx-polygon-dialog__dpad" role="group">
                <button type="button" class="smallBtn" data-move="up" aria-label="Move up">↑</button>
                <button type="button" class="smallBtn" data-move="left" aria-label="Move left">←</button>
                <span data-role="step-value"></span>
                <button type="button" class="smallBtn" data-move="right" aria-label="Move right">→</button>
                <button type="button" class="smallBtn" data-move="down" aria-label="Move down">↓</button>
              </div>
              <div class="onnx-polygon-dialog__point-actions">
                <button type="button" class="smallBtn" data-action="delete"></button>
                <button type="button" class="smallBtn" data-action="mirror"></button>
              </div>
            </div>
          </section>
          <div class="ui-action-grid onnx-polygon-dialog__advanced-actions">
            <button type="button" class="smallBtn" data-action="clear-side"></button>
          </div>
        </div>
      </details>
    </div>
  `;
}

export function mountOnnxPolygonEditor(host, { embedded = false } = {}) {
  if (!host) return null;
  host.innerHTML = editorMarkup();
  const root = host.querySelector("[data-onnx-polygon-dialog]");
  if (!root) return null;
  root.dataset.embedded = embedded ? "true" : "false";
  root.dataset.preview = "fallback";
  const scene = root.querySelector('[data-role="scene"]');
  const backdrop = root.querySelector('[data-role="backdrop"]');
  const fallbackRoad = root.querySelector('[data-role="fallback-road"]');
  const fallbackNotice = root.querySelector('[data-role="fallback-notice"]');
  const snapshotImage = root.querySelector('[data-role="image"]');
  const handles = root.querySelector('[data-role="handles"]');
  const pointSelect = root.querySelector('[data-role="point-select"]');
  const cameraButtons = Object.fromEntries([...root.querySelectorAll("[data-camera]")]
    .map((button) => [button.dataset.camera, button]));
  const state = root.querySelector('[data-role="state"]');
  const advanced = root.querySelector('[data-role="advanced"]');
  const advancedTitle = root.querySelector('[data-role="advanced-title"]');
  const advancedSummary = root.querySelector('[data-role="advanced-summary"]');
  const saveState = root.querySelector('[data-role="save-state"]');
  const pointTitle = root.querySelector('[data-role="point-title"]');
  const pointPosition = root.querySelector('[data-role="point-position"]');
  const stepValue = root.querySelector('[data-role="step-value"]');
  const sideButtons = [...root.querySelectorAll("[data-side]:not(polygon)")];
  const moveButtons = Object.fromEntries([...root.querySelectorAll("[data-move]")].map((button) => [button.dataset.move, button]));
  const buttons = Object.fromEntries([...root.querySelectorAll("[data-action]")].map((button) => [button.dataset.action, button]));
  const polygons = {
    left: root.querySelector('[data-role="polygon-left"]'),
    right: root.querySelector('[data-role="polygon-right"]'),
  };

  advancedTitle.textContent = text("onnx_vision_advanced_edit", "Advanced edit");
  sideButtons[0].textContent = text("onnx_vision_side_left", "Left");
  sideButtons[1].textContent = text("onnx_vision_side_right", "Right");
  pointSelect.setAttribute("aria-label", text("onnx_vision_point_select", "Select point"));
  moveButtons.up.setAttribute("aria-label", text("onnx_vision_move_up", "Move up"));
  moveButtons.down.setAttribute("aria-label", text("onnx_vision_move_down", "Move down"));
  moveButtons.left.setAttribute("aria-label", text("onnx_vision_move_left", "Move left"));
  moveButtons.right.setAttribute("aria-label", text("onnx_vision_move_right", "Move right"));
  buttons.delete.textContent = text("onnx_vision_delete_point", "Delete point");
  buttons.mirror.textContent = text("onnx_vision_mirror_side", "Mirror copy");
  buttons.undo.textContent = text("onnx_vision_undo_point", "Undo point");
  buttons["clear-side"].textContent = text("onnx_vision_clear_side", "Clear selected side");
  buttons.refresh.textContent = text("onnx_vision_refresh_image", "Refresh image");
  buttons.restore.textContent = text("onnx_vision_restore_area", "Reset points");
  buttons.save.textContent = text("onnx_vision_save_area", "Save area");
  cameraButtons.road.textContent = text("onnx_vision_open_road_camera", "View road camera");
  cameraButtons.wide.textContent = text("onnx_vision_open_wide_camera", "View wide camera");
  if (fallbackNotice) fallbackNotice.textContent = text("onnx_vision_fallback_notice", "Example only · Use a real camera image");

  let side = "left";
  let selected = null;
  let config = editableConfig({});
  let savedConfig = editableConfig({});
  let objectUrl = "";
  let snapshotController = null;
  let closed = false;
  let restoreArmed = false;
  let restoreTimer = null;
  let snapshotState = "idle";
  let step = 1;
  let dragging = null;
  let addCandidate = null;
  let saving = false;
  const history = [];
  const repeatCleanups = [];

  function points(targetSide = side) {
    return config[`poly_${targetSide}`];
  }

  function pushHistory() {
    history.push(editableConfig(config));
    if (history.length > MAX_HISTORY) history.shift();
  }

  function selectedPoint() {
    if (!selected) return null;
    return config[`poly_${selected.side}`]?.[selected.index] || null;
  }

  function selectSide(next, preservePoint = false) {
    side = next === "right" ? "right" : "left";
    if (!preservePoint || selected?.side !== side) selected = null;
    sideButtons.forEach((button) => {
      const active = button.dataset.side === side;
      button.classList.toggle("is-active", active);
      button.setAttribute("aria-pressed", active ? "true" : "false");
    });
    render();
  }

  function selectPoint(nextSide, index) {
    side = nextSide === "right" ? "right" : "left";
    selected = { side, index };
    selectSide(side, true);
  }

  function updateState() {
    const incompleteSide = ["left", "right"].find((name) => points(name).length > 0 && points(name).length < 3);
    const point = selectedPoint();
    if (point) {
      state.dataset.tone = "selection";
      state.textContent = text("onnx_vision_selected_state", "{side} point {number} selected", {
        side: selected.side === "left" ? text("onnx_vision_side_left", "Left") : text("onnx_vision_side_right", "Right"),
        number: selected.index + 1,
      });
      return;
    }
    if (snapshotState === "loading") {
      state.dataset.tone = "info";
      state.textContent = text("onnx_vision_snapshot_loading", "Loading camera image...");
      return;
    }
    if (incompleteSide) {
      state.dataset.tone = "warning";
      state.textContent = text("onnx_vision_polygon_minimum", "{side} needs at least 3 points", {
        side: incompleteSide === "left" ? text("onnx_vision_side_left", "Left") : text("onnx_vision_side_right", "Right"),
      });
      return;
    }
    state.dataset.tone = snapshotState === "cached" ? "info" : "";
    if (snapshotState === "cached") {
      state.textContent = text("onnx_vision_snapshot_cached", "Select a point · Last image");
    } else if (snapshotState === "fallback") {
      state.textContent = text("onnx_vision_snapshot_fallback", "Select a point · Default road view");
    } else {
      state.textContent = text("onnx_vision_select_point_state", "Select a point to move");
    }
  }

  function updateControls() {
    const point = selectedPoint();
    const activePoints = points();
    const currentSide = side === "left" ? text("onnx_vision_side_left", "Left") : text("onnx_vision_side_right", "Right");
    const dirty = !sameConfig(config, savedConfig);
    pointTitle.textContent = point
      ? text("onnx_vision_selected_point", "{side} point {number}", { side: currentSide, number: selected.index + 1 })
      : text("onnx_vision_no_point_selected", "Select a point to fine-tune");
    pointPosition.textContent = point ? `${point[0]}, ${point[1]}` : text("onnx_vision_point_help", "Select a point, or tap empty space to add one");
    pointPosition.dataset.hasPoint = point ? "true" : "false";
    root.dataset.hasSelection = point ? "true" : "false";
    const pointOptions = [globalThis.document.createElement("option")];
    pointOptions[0].value = "";
    pointOptions[0].textContent = text("onnx_vision_point_select", "Select point");
    ["left", "right"].forEach((optionSide) => {
      const sideShort = optionSide === "left"
        ? text("onnx_vision_side_left_short", "L")
        : text("onnx_vision_side_right_short", "R");
      points(optionSide).forEach((_, index) => {
        const option = globalThis.document.createElement("option");
        option.value = `${optionSide}:${index}`;
        option.textContent = text("onnx_vision_point_option", "{number}({side})", {
          number: index + 1,
          side: sideShort,
        });
        pointOptions.push(option);
      });
    });
    pointSelect.replaceChildren(...pointOptions);
    pointSelect.value = point ? `${selected.side}:${selected.index}` : "";
    pointSelect.disabled = config.poly_left.length + config.poly_right.length === 0;
    advancedSummary.textContent = text("onnx_vision_polygon_count", "Left {left} · Right {right}", {
      left: config.poly_left.length,
      right: config.poly_right.length,
    });
    saveState.textContent = dirty
      ? text("onnx_vision_unsaved_state", "Unsaved changes")
      : text("onnx_vision_saved_state", "Saved");
    saveState.dataset.dirty = dirty ? "true" : "false";
    buttons.step.textContent = text("onnx_vision_adjust_step", "Step {step}px", { step });
    buttons.step.setAttribute("aria-pressed", step === 5 ? "true" : "false");
    stepValue.textContent = `${step}px`;
    Object.values(moveButtons).forEach((button) => { button.disabled = !point; });
    buttons.delete.disabled = !point || activePoints.length <= 3;
    buttons.mirror.disabled = activePoints.length < 3;
    buttons.undo.disabled = history.length === 0;
    buttons["clear-side"].disabled = activePoints.length === 0;
    const saveBlocked = !dirty || !isPolygonConfigSavable(config) || !hasFreshOnnxVisionSnapshot(snapshotState);
    buttons.save.disabled = saving;
    buttons.save.setAttribute("aria-disabled", saving || saveBlocked ? "true" : "false");
    buttons.save.classList.toggle("is-soft-disabled", !saving && saveBlocked);
  }

  function updateHandleSize() {
    const bounds = scene.getBoundingClientRect();
    if (!(bounds.width > 0)) return;
    const screenMatrix = scene.getScreenCTM?.();
    const matrixScale = screenMatrix ? Math.hypot(Number(screenMatrix.a) || 0, Number(screenMatrix.b) || 0) : 0;
    const imageUnitsPerPixel = matrixScale > 0 ? 1 / matrixScale : config.width / bounds.width;
    const metrics = polygonHandleScreenMetrics(bounds.width);
    const dotRadius = imageUnitsPerPixel * metrics.dotRadius;
    const hitRadius = imageUnitsPerPixel * metrics.hitRadius;
    const labelSize = imageUnitsPerPixel * metrics.labelSize;
    const labelHeight = imageUnitsPerPixel * metrics.labelHeight;
    const labelGap = imageUnitsPerPixel * metrics.labelGap;
    const edgeGap = imageUnitsPerPixel * 2;
    handles.querySelectorAll(".onnx-polygon-handle__hit").forEach((node) => node.setAttribute("r", String(hitRadius)));
    handles.querySelectorAll(".onnx-polygon-handle__selection").forEach((node) => node.setAttribute("r", String(dotRadius + imageUnitsPerPixel * metrics.selectionGap)));
    handles.querySelectorAll(".onnx-polygon-handle__dot").forEach((node) => {
      const isSelected = node.parentElement?.classList.contains("is-selected");
      node.setAttribute("r", String(dotRadius * (isSelected ? 1.12 : 1)));
    });
    handles.querySelectorAll(".onnx-polygon-handle__label").forEach((node) => {
      const x = Number(node.dataset.x) || 0;
      const y = Number(node.dataset.y) || 0;
      const labelWidth = imageUnitsPerPixel * (node.textContent.length > 1 ? metrics.labelWideWidth : metrics.labelWidth);
      const halfWidth = labelWidth / 2;
      const halfHeight = labelHeight / 2;
      const labelX = Math.min(Math.max(x, halfWidth + edgeGap), config.width - halfWidth - edgeGap);
      const preferredY = y - dotRadius - labelGap - halfHeight;
      const labelY = preferredY >= halfHeight + edgeGap
        ? preferredY
        : Math.min(y + dotRadius + labelGap + halfHeight, config.height - halfHeight - edgeGap);
      node.setAttribute("font-size", String(labelSize));
      node.setAttribute("x", String(labelX));
      node.setAttribute("y", String(labelY));
      node.setAttribute("text-anchor", "middle");
      const background = node.previousElementSibling;
      if (background?.classList.contains("onnx-polygon-handle__label-bg")) {
        background.setAttribute("x", String(labelX - halfWidth));
        background.setAttribute("y", String(labelY - halfHeight));
        background.setAttribute("width", String(labelWidth));
        background.setAttribute("height", String(labelHeight));
        background.setAttribute("rx", String(imageUnitsPerPixel * metrics.labelCornerRadius));
      }
      const labelHit = node.parentElement?.querySelector(".onnx-polygon-handle__label-hit");
      if (labelHit) {
        const hitInset = imageUnitsPerPixel * metrics.hitInset;
        labelHit.setAttribute("x", String(labelX - halfWidth - hitInset));
        labelHit.setAttribute("y", String(labelY - halfHeight - hitInset));
        labelHit.setAttribute("width", String(labelWidth + hitInset * 2));
        labelHit.setAttribute("height", String(labelHeight + hitInset * 2));
        labelHit.setAttribute("rx", String(imageUnitsPerPixel * (metrics.labelCornerRadius + metrics.hitInset)));
      }
    });
  }

  function renderGeometry() {
    polygons.left.setAttribute("points", pointList(config.poly_left));
    polygons.right.setAttribute("points", pointList(config.poly_right));
    polygons.left.classList.toggle("is-active", side === "left");
    polygons.right.classList.toggle("is-active", side === "right");
    handles.replaceChildren();
    ["left", "right"].forEach((handleSide) => {
      points(handleSide).forEach(([x, y], index) => {
        const group = createSvgElement("g", {
          class: `onnx-polygon-handle${handleSide === side ? "" : " is-inactive"}${selected?.side === handleSide && selected.index === index ? " is-selected" : ""}`,
          "data-handle-side": handleSide,
          "data-handle-index": index,
          role: "button",
          tabindex: handleSide === side ? "0" : "-1",
          "aria-pressed": selected?.side === handleSide && selected.index === index ? "true" : "false",
          "aria-label": text("onnx_vision_point_accessible", "{side} point {number}, {x}, {y}", {
            side: handleSide === "left" ? text("onnx_vision_side_left", "Left") : text("onnx_vision_side_right", "Right"),
            number: index + 1,
            x,
            y,
          }),
        });
        group.append(
          createSvgElement("circle", { class: "onnx-polygon-handle__hit", cx: x, cy: y }),
          createSvgElement("circle", { class: "onnx-polygon-handle__selection", cx: x, cy: y }),
          createSvgElement("circle", { class: "onnx-polygon-handle__dot", cx: x, cy: y }),
        );
        group.append(
          createSvgElement("rect", { class: "onnx-polygon-handle__label-hit" }),
          createSvgElement("rect", {
            class: "onnx-polygon-handle__label-bg",
            "aria-hidden": "true",
          }),
        );
        const label = createSvgElement("text", {
          class: "onnx-polygon-handle__label",
          x,
          y,
          "data-x": x,
          "data-y": y,
          "dominant-baseline": "middle",
          "aria-hidden": "true",
        });
        label.textContent = polygonPointLabel(index);
        group.append(label);
        handles.append(group);
      });
    });
    updateHandleSize();
  }

  function render() {
    scene.setAttribute("viewBox", `0 0 ${config.width} ${config.height}`);
    scene.setAttribute("width", String(config.width));
    scene.setAttribute("height", String(config.height));
    scene.style.setProperty("--onnx-camera-aspect", `${config.width} / ${config.height}`);
    scene.setAttribute("aria-label", text("onnx_vision_polygon_surface", "Blindspot detection area editor"));
    backdrop.setAttribute("width", String(config.width));
    backdrop.setAttribute("height", String(config.height));
    snapshotImage.setAttribute("width", String(config.width));
    snapshotImage.setAttribute("height", String(config.height));
    layoutFallbackRoadScene(fallbackRoad, config.width, config.height);
    renderGeometry();
    updateState();
    updateControls();
  }

  function mutateSelected(dx, dy) {
    const point = selectedPoint();
    if (!point) return;
    const next = movePolygonPoint(point, dx, dy, config.width, config.height);
    if (next[0] === point[0] && next[1] === point[1]) return;
    pushHistory();
    points(selected.side)[selected.index] = next;
    render();
  }

  async function displaySnapshot(blob, source) {
    const nextObjectUrl = globalThis.URL?.createObjectURL?.(blob) || "";
    if (!nextObjectUrl) throw new Error("Snapshot preview is unavailable");
    const preload = globalThis.document.createElement("img");
    try {
      await new Promise((resolve, reject) => {
        preload.onload = resolve;
        preload.onerror = () => reject(new Error("Snapshot preview is unavailable"));
        preload.src = nextObjectUrl;
      });
    } catch (error) {
      globalThis.URL?.revokeObjectURL?.(nextObjectUrl);
      throw error;
    }
    if (closed) {
      globalThis.URL?.revokeObjectURL?.(nextObjectUrl);
      return;
    }
    if (objectUrl) globalThis.URL?.revokeObjectURL?.(objectUrl);
    objectUrl = nextObjectUrl;
    snapshotImage.setAttribute("href", objectUrl);
    snapshotImage.removeAttribute("hidden");
    fallbackRoad?.setAttribute("hidden", "");
    fallbackNotice?.setAttribute("hidden", "");
    root.dataset.preview = "camera";
    snapshotState = source;
    updateState();
    updateControls();
  }

  function displayFallback() {
    snapshotImage.setAttribute("hidden", "");
    fallbackRoad?.removeAttribute("hidden");
    fallbackNotice?.removeAttribute("hidden");
    root.dataset.preview = "fallback";
    snapshotState = "fallback";
    updateState();
    updateControls();
  }

  async function loadSnapshot() {
    snapshotController?.abort();
    snapshotController = new AbortController();
    buttons.refresh.disabled = true;
    if (!objectUrl) {
      snapshotState = "loading";
      updateState();
      updateControls();
      const cached = await onnxSnapshotCache.load(globalThis);
      if (cached && !closed) {
        try {
          await displaySnapshot(cached, "cached");
        } catch {
          displayFallback();
        }
      }
    }
    try {
      const blob = await loadOnnxVisionSnapshot("wide", globalThis, { signal: snapshotController.signal });
      if (closed) return;
      await displaySnapshot(blob, "ready");
      void onnxSnapshotCache.store(blob, globalThis);
    } catch (error) {
      if (error?.name === "AbortError" || closed) return;
      if (objectUrl) {
        snapshotState = "cached";
        updateState();
        updateControls();
      } else {
        displayFallback();
      }
    } finally {
      if (!closed) buttons.refresh.disabled = false;
    }
  }

  sideButtons.forEach((button) => button.addEventListener("click", () => selectSide(button.dataset.side)));
  pointSelect.addEventListener("change", () => {
    if (pointSelect.value === "") {
      selected = null;
      render();
      return;
    }
    const [nextSide, rawIndex] = pointSelect.value.split(":");
    const index = Number(rawIndex);
    if (!(["left", "right"].includes(nextSide)) || !Number.isInteger(index) || !points(nextSide)[index]) return;
    selectPoint(nextSide, index);
  });
  globalThis.CarrotUI?.segmentedControl?.create?.(root.querySelector('[data-role="sides"]'), {
    itemSelector: "[data-side]",
    selectedAttribute: "aria-pressed",
    onActivate: (button) => selectSide(button.dataset.side),
  });
  advanced.addEventListener("toggle", () => {
    root.dataset.advanced = advanced.open ? "true" : "false";
    updateHandleSize();
  });

  scene.addEventListener("pointerdown", (event) => {
    const handle = event.target.closest?.("[data-handle-side]");
    if (event.button !== undefined && event.button !== 0) return;
    if (!handle) {
      if (event.isPrimary === false) return;
      const activePoint = selectedPoint();
      const gesture = polygonCanvasGesture({ hasSelectedPoint: Boolean(activePoint) });
      if (gesture === "add-point") {
        addCandidate = { pointerId: event.pointerId, side, x: event.clientX, y: event.clientY, moved: false };
        return;
      }
      event.preventDefault();
      event.stopPropagation();
      const start = canvasPointFromEvent(event, scene, config.width, config.height);
      if (gesture === "move-selected-point" && selected.side === side) {
        dragging = {
          type: "selected-point",
          pointerId: event.pointerId,
          side: selected.side,
          index: selected.index,
          start,
          source: [...activePoint],
          recorded: false,
        };
      }
      addCandidate = null;
      scene.setPointerCapture?.(event.pointerId);
      return;
    }
    event.preventDefault();
    event.stopPropagation();
    const handleSide = handle.dataset.handleSide;
    const index = Number(handle.dataset.handleIndex);
    const start = canvasPointFromEvent(event, scene, config.width, config.height);
    const source = config[`poly_${handleSide}`]?.[index];
    const wasSelected = selected?.side === handleSide && selected.index === index;
    if (!wasSelected) selectPoint(handleSide, index);
    dragging = {
      type: "point",
      pointerId: event.pointerId,
      side: handleSide,
      index,
      start,
      source: source ? [...source] : start,
      wasSelected,
      recorded: false,
    };
    scene.setPointerCapture?.(event.pointerId);
  });
  scene.addEventListener("pointermove", (event) => {
    if (addCandidate?.pointerId === event.pointerId
        && Math.hypot(event.clientX - addCandidate.x, event.clientY - addCandidate.y) > 8) {
      addCandidate.moved = true;
    }
    if (!dragging || dragging.pointerId !== event.pointerId) return;
    event.preventDefault();
    const next = canvasPointFromEvent(event, scene, config.width, config.height);
    if (dragging.type === "selected-point") {
      const current = config[`poly_${dragging.side}`]?.[dragging.index];
      const moved = movePolygonPoint(
        dragging.source,
        next[0] - dragging.start[0],
        next[1] - dragging.start[1],
        config.width,
        config.height,
      );
      if (!current || (moved[0] === current[0] && moved[1] === current[1])) return;
      if (!dragging.recorded) {
        pushHistory();
        dragging.recorded = true;
      }
      config[`poly_${dragging.side}`][dragging.index] = moved;
      render();
      return;
    }
    const current = config[`poly_${dragging.side}`]?.[dragging.index];
    const moved = movePolygonPoint(
      dragging.source,
      next[0] - dragging.start[0],
      next[1] - dragging.start[1],
      config.width,
      config.height,
    );
    if (!current || (moved[0] === current[0] && moved[1] === current[1])) return;
    if (!dragging.recorded) {
      pushHistory();
      dragging.recorded = true;
    }
    config[`poly_${dragging.side}`][dragging.index] = moved;
    render();
  });
  const endDrag = (event) => {
    if (!dragging || dragging.pointerId !== event.pointerId) return null;
    const completed = dragging;
    scene.releasePointerCapture?.(event.pointerId);
    dragging = null;
    return completed;
  };
  scene.addEventListener("pointerup", (event) => {
    const candidate = addCandidate?.pointerId === event.pointerId ? addCandidate : null;
    const completed = endDrag(event);
    addCandidate = null;
    if (completed?.type === "point") {
      if (completed.wasSelected && !completed.recorded) {
        selected = null;
        render();
      }
      return;
    }
    if (completed?.type === "selected-point") {
      if (!completed.recorded) {
        selected = null;
        render();
      }
      return;
    }
    if (!candidate || candidate.moved || completed?.recorded || (event.button !== undefined && event.button !== 0)) return;
    if (event.target.closest?.("[data-handle-side]")) return;
    if (points(candidate.side).length >= 64) return;
    pushHistory();
    const targetPoints = points(candidate.side);
    targetPoints.push(canvasPointFromEvent(event, scene, config.width, config.height));
    selected = { side: candidate.side, index: targetPoints.length - 1 };
    render();
  });
  scene.addEventListener("pointercancel", (event) => {
    if (addCandidate?.pointerId === event.pointerId) addCandidate = null;
    endDrag(event);
  });
  scene.addEventListener("keydown", (event) => {
    const handle = event.target.closest?.("[data-handle-side]");
    if (handle && (event.key === "Enter" || event.key === " ")) {
      event.preventDefault();
      selectPoint(handle.dataset.handleSide, Number(handle.dataset.handleIndex));
      return;
    }
    const delta = {
      ArrowUp: [0, -step],
      ArrowDown: [0, step],
      ArrowLeft: [-step, 0],
      ArrowRight: [step, 0],
    }[event.key];
    if (!delta || !selectedPoint()) return;
    event.preventDefault();
    mutateSelected(delta[0], delta[1]);
  });

  const ResizeObserverClass = globalThis.ResizeObserver;
  const handleResizeObserver = typeof ResizeObserverClass === "function"
    ? new ResizeObserverClass(updateHandleSize)
    : null;
  handleResizeObserver?.observe(scene);
  const onWindowResize = () => updateHandleSize();
  if (!handleResizeObserver) globalThis.addEventListener?.("resize", onWindowResize);

  const moveDeltas = { up: [0, -1], down: [0, 1], left: [-1, 0], right: [1, 0] };
  Object.entries(moveButtons).forEach(([name, button]) => {
    const [dx, dy] = moveDeltas[name];
    repeatCleanups.push(bindRepeatButton(button, () => mutateSelected(dx * step, dy * step)));
  });
  buttons.step.addEventListener("click", () => {
    step = step === 1 ? 5 : 1;
    updateControls();
  });
  buttons.delete.addEventListener("click", () => {
    const point = selectedPoint();
    if (!point || points(selected.side).length <= 3) return;
    pushHistory();
    points(selected.side).splice(selected.index, 1);
    selected = null;
    render();
  });
  buttons.mirror.addEventListener("click", () => {
    if (points().length < 3) return;
    pushHistory();
    const targetSide = side === "left" ? "right" : "left";
    config[`poly_${targetSide}`] = mirroredPolygon(points(), config.width);
    side = targetSide;
    selected = null;
    selectSide(targetSide);
  });
  buttons.undo.addEventListener("click", () => {
    const previous = history.pop();
    if (!previous) return;
    config = previous;
    selected = null;
    render();
  });
  buttons["clear-side"].addEventListener("click", () => {
    if (points().length === 0) return;
    pushHistory();
    config[`poly_${side}`] = [];
    selected = null;
    render();
  });
  buttons.refresh.addEventListener("click", loadSnapshot);
  buttons.save.addEventListener("click", async () => {
    if (!hasFreshOnnxVisionSnapshot(snapshotState)) {
      globalThis.showAppToast?.(
        text("onnx_vision_refresh_before_save", "Refresh the real camera image, then set the area again."),
        { tone: "warning" },
      );
      return;
    }
    if (!isPolygonConfigSavable(config)) return;
    saving = true;
    updateControls();
    try {
      config = editableConfig(await saveOnnxVisionConfig(config));
      savedConfig = editableConfig(config);
      history.length = 0;
      selected = null;
      render();
      globalThis.showAppToast?.(text("onnx_vision_area_saved", "Detection area saved"));
    } catch (error) {
      globalThis.showAppToast?.(error?.message || text("failed", "Failed"), { tone: "error" });
    } finally {
      saving = false;
      updateControls();
    }
  });
  buttons.restore.addEventListener("click", async () => {
    if (!restoreArmed) {
      restoreArmed = true;
      buttons.restore.classList.add("btn--danger");
      buttons.restore.textContent = text("onnx_vision_restore_area_confirm", "Tap again to reset");
      restoreTimer = globalThis.setTimeout(() => {
        restoreArmed = false;
        buttons.restore.classList.remove("btn--danger");
        buttons.restore.textContent = text("onnx_vision_restore_area", "Reset points");
      }, 4000);
      return;
    }
    restoreArmed = false;
    if (restoreTimer) globalThis.clearTimeout(restoreTimer);
    buttons.restore.classList.remove("btn--danger");
    buttons.restore.textContent = text("onnx_vision_restore_area", "Reset points");
    buttons.restore.disabled = true;
    try {
      pushHistory();
      await restoreOnnxVisionConfig();
      config = editableConfig(await loadOnnxVisionConfig());
      savedConfig = editableConfig(config);
      history.length = 0;
      selected = null;
      render();
      globalThis.showAppToast?.(text("onnx_vision_area_restored", "Default detection areas restored"));
    } catch (error) {
      history.pop();
      globalThis.showAppToast?.(error?.message || text("failed", "Failed"), { tone: "error" });
    } finally {
      buttons.restore.disabled = false;
    }
  });
  Object.entries(cameraButtons).forEach(([stream, button]) => {
    button.addEventListener("click", () => openOnnxSnapshotDialog(stream));
  });

  render();
  const ready = (async () => {
    try {
      config = editableConfig(await loadOnnxVisionConfig());
      savedConfig = editableConfig(config);
      render();
      loadSnapshot();
    } catch (error) {
      displayFallback();
    }
  })();

  function destroy() {
    if (closed) return;
    closed = true;
    snapshotController?.abort();
    repeatCleanups.forEach((cleanup) => cleanup());
    handleResizeObserver?.disconnect();
    if (!handleResizeObserver) globalThis.removeEventListener?.("resize", onWindowResize);
    if (restoreTimer) globalThis.clearTimeout(restoreTimer);
    if (objectUrl) globalThis.URL?.revokeObjectURL?.(objectUrl);
    host.replaceChildren();
  }

  function setCameraAvailable(available) {
    Object.values(cameraButtons).forEach((button) => { button.disabled = !available; });
  }

  return Object.freeze({ root, ready, setCameraAvailable, destroy });
}

export async function openOnnxPolygonEditor() {
  const completion = globalThis.appAlert?.("", {
    title: text("onnx_vision_detection_area", "BSD detection area"),
    html: true,
    messageHtml: '<div data-onnx-polygon-editor-host></div>',
    confirmLabel: text("close", "Close"),
  });
  if (!completion) return;
  const host = globalThis.document?.querySelector?.("#appDialog [data-onnx-polygon-editor-host]");
  const editor = mountOnnxPolygonEditor(host);
  try {
    await editor?.ready;
    await completion;
  } finally {
    editor?.destroy();
  }
}
