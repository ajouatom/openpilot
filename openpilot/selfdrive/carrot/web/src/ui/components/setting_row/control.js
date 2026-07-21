/**
 * Builds the control cluster of a setting row.
 *
 * The five kinds (toggle, segmented, select, stepper, slider) used to be
 * assembled inline inside the settings renderer, which is why each one ended
 * up with its own event wiring and its own idea of when a value is committed.
 * Producing the markup from one place is the first half of fixing that; the
 * behaviour already went through ui/components/commit_gesture.js.
 *
 * The markup is byte-for-byte what the renderer produced before, so the
 * existing stylesheet keeps matching. Behaviour is deliberately not attached
 * here: the caller wires events to the elements it gets back.
 */

export const SETTING_CONTROL_KINDS = Object.freeze(["toggle", "segmented", "select", "stepper", "slider"]);

function createElement(doc, tag, className) {
  const element = doc.createElement(tag);
  if (className) element.className = className;
  return element;
}

function createButton(doc, className, { text, label } = {}) {
  const button = createElement(doc, "button", className);
  button.type = "button";
  if (text !== undefined) button.textContent = text;
  if (label) button.setAttribute("aria-label", label);
  return button;
}

function buildToggle(doc, ctrl, refs, options) {
  const switchLabel = createElement(doc, "label", "c-switch");
  const input = createElement(doc, "input", "c-switch__input");
  input.type = "checkbox";
  input.setAttribute("aria-label", options.label);
  switchLabel.appendChild(input);
  switchLabel.appendChild(createElement(doc, "span", "c-switch__track"));
  ctrl.appendChild(switchLabel);
  refs.toggleInput = input;
}

function buildSegmented(doc, ctrl, refs, options) {
  const group = createElement(doc, "div", "setting-segments");
  options.optionValues.forEach((optionValue) => {
    const button = createButton(doc, "setting-segment", { text: options.optionLabel(optionValue) });
    button.dataset.value = String(optionValue);
    button.setAttribute("aria-pressed", "false");
    refs.segmentButtons.push(button);
    group.appendChild(button);
  });
  ctrl.appendChild(group);
  refs.segmentGroup = group;
}

function buildSelect(doc, ctrl, refs, options) {
  const select = createButton(doc, "setting-select setting-select--button", { label: options.label });
  select.setAttribute("aria-haspopup", "dialog");
  ctrl.appendChild(select);
  refs.selectInput = select;
}

function buildStepper(doc, ctrl, refs, options) {
  refs.minusButton = createButton(doc, "smallBtn setting-value-arrow setting-value-arrow--prev", {
    text: "-",
    label: options.previousLabel,
  });
  refs.plusButton = createButton(doc, "smallBtn setting-value-arrow setting-value-arrow--next", {
    text: "+",
    label: options.nextLabel,
  });
  refs.unitButton = createButton(doc, "setting-unit-cycle");

  ctrl.appendChild(refs.minusButton);
  ctrl.appendChild(refs.value);
  ctrl.appendChild(refs.plusButton);
}

function buildSlider(doc, ctrl, refs, options) {
  const wrap = createElement(doc, "div", "setting-slider");
  const input = createElement(doc, "input", "setting-slider__input");
  input.type = "range";
  input.min = String(options.min);
  input.max = String(options.max);
  input.step = String(options.step);
  input.setAttribute("aria-label", options.label);
  wrap.appendChild(input);

  refs.sliderInput = input;
  refs.minusButton = createButton(doc, "smallBtn setting-step setting-step--minus", { text: "-" });
  refs.plusButton = createButton(doc, "smallBtn setting-step setting-step--plus", { text: "+" });
  refs.unitButton = createButton(doc, "setting-unit-cycle");

  ctrl.appendChild(wrap);
  ctrl.appendChild(refs.minusButton);
  ctrl.appendChild(refs.value);
  ctrl.appendChild(refs.plusButton);
}

/**
 * @returns the control container plus every element the caller has to wire.
 * Elements that do not exist for the chosen kind come back as null.
 */
export function createSettingControl(options = {}) {
  const doc = options.document;
  if (!doc || typeof doc.createElement !== "function") {
    throw new TypeError("A document is required to build a setting control");
  }
  const kind = String(options.kind || "");
  if (!SETTING_CONTROL_KINDS.includes(kind)) {
    throw new TypeError(`Unknown setting control kind: ${kind}`);
  }

  // The stepper is a slider rendered compactly, so it keeps the value layout.
  const isStepper = kind === "stepper";
  const ctrl = createElement(doc, "div", `ctrl ctrl--${isStepper ? "value" : kind}`);
  const value = createButton(doc, isStepper ? "value-surface val setting-value-compact" : "value-surface val", {
    label: options.valueLabel,
  });

  const refs = {
    ctrl,
    value,
    toggleInput: null,
    segmentGroup: null,
    segmentButtons: [],
    selectInput: null,
    sliderInput: null,
    minusButton: null,
    plusButton: null,
    unitButton: null,
  };

  const resolved = {
    label: options.label || "",
    valueLabel: options.valueLabel || "",
    previousLabel: options.previousLabel || "",
    nextLabel: options.nextLabel || "",
    optionValues: Array.isArray(options.optionValues) ? options.optionValues : [],
    optionLabel: typeof options.optionLabel === "function" ? options.optionLabel : (value) => String(value),
    min: options.min,
    max: options.max,
    step: options.step,
  };

  if (kind === "toggle") buildToggle(doc, ctrl, refs, resolved);
  else if (kind === "segmented") buildSegmented(doc, ctrl, refs, resolved);
  else if (kind === "select") buildSelect(doc, ctrl, refs, resolved);
  else if (isStepper) buildStepper(doc, ctrl, refs, resolved);
  else buildSlider(doc, ctrl, refs, resolved);

  // The three kinds above append the value last; the numeric ones place it
  // between their buttons and have already done so.
  if (!isStepper && kind !== "slider") ctrl.appendChild(value);

  return refs;
}
