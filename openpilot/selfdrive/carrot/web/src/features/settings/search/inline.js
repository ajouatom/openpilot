/* Inline "find settings" field.

   Owns the field's own behaviour: debounced input (IME-safe), the clear
   button, slot mounting and the label/status chrome. Navigation — switching
   to the virtual search group, history and screen transitions — belongs to
   the page and is triggered through onApply(query). */

export function createSettingInlineSearchView(options = {}) {
  const elements = options.elements || {};
  const form = elements.form || null;
  const input = elements.input || null;
  const clearButton = elements.clear || null;
  const label = elements.label || null;
  const hint = elements.hint || null;
  const status = elements.status || null;

  const getLabels = typeof options.getLabels === "function" ? options.getLabels : () => ({});
  const getStatusText = typeof options.getStatusText === "function" ? options.getStatusText : () => "";
  const onApply = typeof options.onApply === "function" ? options.onApply : () => {};
  const onError = typeof options.onError === "function" ? options.onError : null;
  const debounceMs = Number.isFinite(options.debounceMs) ? Math.max(0, options.debounceMs) : 160;

  let debounceTimer = null;
  let bound = false;

  function cancelPending() {
    if (debounceTimer === null) return;
    clearTimeout(debounceTimer);
    debounceTimer = null;
  }

  function getQuery() {
    return String(input?.value || "").trim();
  }

  function setQuery(query) {
    if (!input) return;
    input.value = String(query || "").trim();
    syncClear();
  }

  function syncClear() {
    if (clearButton) clearButton.hidden = !input?.value;
  }

  function applyLabels() {
    const labels = getLabels();
    if (label) label.textContent = labels.label || "";
    if (hint) hint.textContent = labels.hint || "";
    if (input) {
      input.placeholder = labels.placeholder || "";
      // The visible title is optional, but the field still needs a name for
      // assistive tech when the caller provides one.
      if (labels.label) input.setAttribute("aria-label", labels.label);
    }
    if (clearButton) clearButton.setAttribute("aria-label", labels.clear || "");
  }

  function syncStatus() {
    if (status) status.textContent = getStatusText() || "";
  }

  function refresh() {
    applyLabels();
    syncClear();
    syncStatus();
  }

  function isFocused() {
    return Boolean(input) && form?.ownerDocument?.activeElement === input;
  }

  function focus() {
    input?.focus({ preventScroll: true });
  }

  function mount(target, insertBefore = null) {
    if (!form || !target || form.parentElement === target) return false;
    const focused = input !== null && form.ownerDocument?.activeElement === input;
    const selection = focused ? [input.selectionStart, input.selectionEnd] : null;
    if (insertBefore) target.insertBefore(form, insertBefore);
    else target.appendChild(form);
    if (focused) {
      input.focus({ preventScroll: true });
      input.setSelectionRange(...selection);
    }
    return true;
  }

  function applyFromInput() {
    cancelPending();
    syncClear();
    const query = getQuery();
    const task = Promise.resolve(onApply(query));
    if (onError) task.catch(onError);
    return task;
  }

  function schedule(event) {
    cancelPending();
    syncClear();
    if (event?.isComposing) return;
    debounceTimer = setTimeout(() => {
      debounceTimer = null;
      applyFromInput();
    }, debounceMs);
  }

  function bind() {
    if (bound) return;
    bound = true;
    input?.addEventListener("input", schedule);
    input?.addEventListener("compositionstart", cancelPending);
    input?.addEventListener("compositionend", schedule);
    form?.addEventListener("submit", (event) => {
      event.preventDefault();
      applyFromInput();
    });
    clearButton?.addEventListener("click", () => {
      if (input) input.value = "";
      applyFromInput().finally(() => input?.focus({ preventScroll: true }));
    });
  }

  return Object.freeze({
    getQuery,
    setQuery,
    cancelPending,
    applyFromInput,
    applyLabels,
    syncClear,
    syncStatus,
    refresh,
    isFocused,
    focus,
    mount,
    bind,
  });
}
