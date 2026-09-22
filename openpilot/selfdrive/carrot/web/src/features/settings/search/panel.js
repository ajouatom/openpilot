import { isProfileSearchScope, normalizeSearchScope } from "./entries.js";
import { renderSettingSearchResults } from "./results.js";

/* Overlay search panel view.

   Owns the panel DOM state: visibility, scope, labels, result rendering and
   the input debounce. The page keeps the page-level wiring that this view
   cannot know about — settings/profile loading, history entries, the FAB
   state and the modal scroll lock — through the injected callbacks. */

function callback(value, fallback) {
  return typeof value === "function" ? value : fallback;
}

export function createSettingSearchPanelView(options = {}) {
  const elements = options.elements || {};
  const panel = elements.panel || null;
  const backdrop = elements.backdrop || null;
  const form = elements.form || null;
  const input = elements.input || null;
  const results = elements.results || null;

  const getEntries = callback(options.getEntries, () => []);
  const getLabels = callback(options.getLabels, () => ({}));
  const getProfileName = callback(options.getProfileName, () => "");
  const escape = callback(options.escape, (value) => String(value ?? ""));
  const highlight = callback(options.highlight, (value) => escape(value));
  const renderResults = callback(options.renderResults, renderSettingSearchResults);
  const onSelect = options.onSelect || null;
  const onRequestClose = callback(options.onRequestClose, () => {});
  const onOpenChange = callback(options.onOpenChange, () => {});
  const onHidden = callback(options.onHidden, () => {});
  const debounceMs = Number.isFinite(options.debounceMs) ? Math.max(0, options.debounceMs) : 70;
  // Matches the shared dialog close delay (APP_DIALOG_CLOSE_DELAY) and
  // --motion-base, so the panel fades out before it leaves the layout.
  const closeDelayMs = Number.isFinite(options.closeDelayMs) ? Math.max(0, options.closeDelayMs) : 180;

  let scope = normalizeSearchScope(null);
  let debounceTimer = null;
  let bound = false;
  // Explicit open state: `hidden` lags by the close animation, so visibility
  // must not be derived from the DOM.
  let open = false;
  let openFrame = null;
  let closeTimer = null;

  function isOpen() {
    return open;
  }

  function getScope() {
    return scope;
  }

  function setScope(next) {
    scope = normalizeSearchScope(next);
    return scope;
  }

  function scopeLabel() {
    const labels = getLabels();
    if (isProfileSearchScope(scope)) return getProfileName(scope.profileId) || labels.sourceProfile || "";
    return labels.all || "";
  }

  function applyLabels() {
    if (!input) return;
    const labels = getLabels();
    input.placeholder = isProfileSearchScope(scope)
      ? labels.profilePlaceholder || labels.placeholder || ""
      : labels.placeholder || "";
    input.setAttribute("aria-label", scopeLabel());
  }

  function mountOverlay() {
    const documentRef = panel?.ownerDocument || backdrop?.ownerDocument;
    if (!documentRef || !documentRef.body) return;
    if (backdrop && backdrop.parentElement !== documentRef.body) documentRef.body.appendChild(backdrop);
    if (panel && panel.parentElement !== documentRef.body) documentRef.body.appendChild(panel);
  }

  function render(query) {
    const value = query !== undefined ? query : input?.value || "";
    if (!results) return 0;
    return renderResults({
      container: results,
      entries: getEntries(),
      query: value,
      scope,
      labels: getLabels(),
      escape,
      highlight,
      onSelect,
    });
  }

  function focusInput() {
    if (!input) return;
    const focus = () => {
      input.focus({ preventScroll: true });
      input.select();
    };
    if (typeof requestAnimationFrame === "function") requestAnimationFrame(focus);
    else focus();
  }

  function cancelPending() {
    if (debounceTimer === null) return;
    clearTimeout(debounceTimer);
    debounceTimer = null;
  }

  function cancelOpenFrame() {
    if (openFrame === null) return;
    if (typeof cancelAnimationFrame === "function") cancelAnimationFrame(openFrame);
    openFrame = null;
  }

  // Enter runs from the closed base state, so the open class is added one frame
  // after the element becomes visible (same as the shared dialog).
  function revealOpenState() {
    cancelOpenFrame();
    const apply = () => {
      openFrame = null;
      panel?.classList?.add("is-open");
      backdrop?.classList?.add("is-open");
    };
    if (typeof requestAnimationFrame === "function") openFrame = requestAnimationFrame(apply);
    else apply();
  }

  function cancelCloseTimer() {
    if (closeTimer === null) return;
    clearTimeout(closeTimer);
    closeTimer = null;
  }

  function scheduleClose() {
    cancelCloseTimer();
    closeTimer = setTimeout(() => {
      closeTimer = null;
      if (panel) panel.hidden = true;
      if (backdrop) backdrop.hidden = true;
      onHidden();
    }, closeDelayMs);
  }

  function show() {
    mountOverlay();
    if (!panel) return;
    cancelCloseTimer();
    open = true;
    applyLabels();
    panel.hidden = false;
    panel.setAttribute("aria-hidden", "false");
    if (backdrop) backdrop.hidden = false;
    onOpenChange(true);
    render();
    revealOpenState();
    focusInput();
  }

  function hide() {
    cancelPending();
    cancelOpenFrame();
    open = false;
    panel?.classList?.remove("is-open");
    backdrop?.classList?.remove("is-open");
    if (panel) panel.setAttribute("aria-hidden", "true");
    if (input) {
      input.value = "";
      input.placeholder = getLabels().placeholder || "";
      input.removeAttribute("aria-label");
    }
    if (results) results.replaceChildren();
    scope = normalizeSearchScope(null);
    onOpenChange(false);
    scheduleClose();
  }

  function scheduleRender(event) {
    cancelPending();
    // Mid-composition text (Hangul IME) is incomplete: rendering it would
    // flash an empty/incorrect result list until the composition commits.
    if (event?.isComposing) return;
    debounceTimer = setTimeout(() => {
      debounceTimer = null;
      render(input.value);
    }, debounceMs);
  }

  function bind() {
    if (bound) return;
    bound = true;
    if (backdrop) backdrop.onclick = () => onRequestClose();
    if (form) {
      form.addEventListener("submit", (event) => {
        event.preventDefault();
        results?.querySelector("button.setting-search-result")?.click();
      });
    }
    if (input) {
      input.addEventListener("input", scheduleRender);
      input.addEventListener("compositionstart", cancelPending);
      input.addEventListener("compositionend", scheduleRender);
    }
  }

  return Object.freeze({
    isOpen,
    getScope,
    setScope,
    scopeLabel,
    applyLabels,
    mountOverlay,
    render,
    show,
    hide,
    focusInput,
    cancelPending,
    bind,
  });
}
