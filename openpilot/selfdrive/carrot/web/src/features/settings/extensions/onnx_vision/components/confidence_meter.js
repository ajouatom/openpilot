function clampPercent(value) {
  const parsed = Number(value);
  if (!Number.isFinite(parsed)) return 0;
  return Math.min(Math.max(parsed * 100, 0), 100);
}

export function createConfidenceMeter(documentRoot, options = {}) {
  const root = documentRoot.createElement("div");
  root.className = "onnx-confidence";

  const header = documentRoot.createElement("div");
  header.className = "onnx-confidence__header";
  const label = documentRoot.createElement("span");
  label.className = "onnx-confidence__label";
  const value = documentRoot.createElement("strong");
  value.className = "onnx-confidence__value";
  header.append(label, value);

  const progress = documentRoot.createElement("progress");
  progress.className = "onnx-confidence__track";
  progress.max = 100;
  root.append(header, progress);

  function update(input = {}) {
    const percent = clampPercent(input.confidence);
    label.textContent = String(input.label || options.label || "Confidence");
    value.textContent = `${percent.toFixed(1)}%`;
    progress.value = percent;
    progress.setAttribute("aria-label", label.textContent);
    progress.setAttribute("aria-valuetext", value.textContent);
    root.dataset.active = input.active === true ? "true" : "false";
    root.dataset.valid = input.valid === false ? "false" : "true";
  }

  update(options);
  return Object.freeze({ root, update });
}
