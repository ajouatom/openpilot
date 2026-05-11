"use strict";

// Focus trap for modal-like surfaces (dialogs, sheets, full-screen overlays).
//
// Why:
//   When a modal opens, Tab/Shift+Tab should cycle inside it. Otherwise
//   keyboard users can tab "behind" the modal into the obscured page,
//   which is broken UX and a WCAG 2.1 violation (2.4.3, 2.4.11).
//
// Use:
//   const trap = createFocusTrap(overlayEl, { initialFocus: btn });
//   trap.activate();    // call when overlay becomes visible
//   trap.deactivate();  // call when overlay closes (restores prior focus)
//
//   `initialFocus` (optional): element or selector to focus first.
//     Defaults to the first focusable element inside the container.
//   `returnFocus`  (optional): override where focus goes on deactivate.
//     Defaults to whichever element had focus when activate() was called.
//   `escape`       (optional): callback fired when Esc is pressed.
//     If provided, Esc is intercepted and passed to this handler.
//     If omitted, Esc is left for the host element to handle.
//
// Pairs with .app-dialog and similar surfaces. dialog.js can adopt it
// by calling createFocusTrap when opening a dialog and deactivate() on
// resolve. New overlays should use this rather than re-implementing.

const FOCUSABLE_SELECTOR = [
  'a[href]',
  'area[href]',
  'button:not([disabled])',
  'input:not([disabled]):not([type="hidden"])',
  'select:not([disabled])',
  'textarea:not([disabled])',
  '[tabindex]:not([tabindex="-1"])',
  '[contenteditable]:not([contenteditable="false"])',
  'audio[controls]',
  'video[controls]',
  'iframe',
  'object',
  'embed',
  'summary',
].join(",");

function getFocusableElements(root) {
  if (!root) return [];
  return Array.from(root.querySelectorAll(FOCUSABLE_SELECTOR))
    .filter((el) => {
      if (el.hasAttribute("inert")) return false;
      if (el.getAttribute("aria-hidden") === "true") return false;
      // Reject hidden elements (display:none or visibility:hidden, including ancestors).
      const rect = el.getBoundingClientRect();
      if (rect.width === 0 && rect.height === 0) return false;
      const style = window.getComputedStyle(el);
      if (style.visibility === "hidden" || style.display === "none") return false;
      return true;
    });
}

function resolveInitial(container, value) {
  if (!value) return null;
  if (typeof value === "string") return container.querySelector(value);
  if (value instanceof Element) return value;
  return null;
}

function createFocusTrap(container, options = {}) {
  if (!container) return { activate() {}, deactivate() {} };
  let active = false;
  let restoreTo = null;
  let keydownHandler = null;

  function onKeydown(ev) {
    if (!active) return;
    if (ev.key === "Escape" && typeof options.escape === "function") {
      options.escape(ev);
      return;
    }
    if (ev.key !== "Tab") return;

    const focusable = getFocusableElements(container);
    if (!focusable.length) {
      // Nothing focusable inside; keep focus on the container itself.
      ev.preventDefault();
      container.focus?.();
      return;
    }
    const first = focusable[0];
    const last = focusable[focusable.length - 1];
    const activeEl = document.activeElement;

    if (ev.shiftKey) {
      if (activeEl === first || !container.contains(activeEl)) {
        ev.preventDefault();
        last.focus();
      }
    } else {
      if (activeEl === last || !container.contains(activeEl)) {
        ev.preventDefault();
        first.focus();
      }
    }
  }

  return {
    activate() {
      if (active) return;
      active = true;
      restoreTo = options.returnFocus || document.activeElement;
      // Make the container focusable as a fallback target.
      if (!container.hasAttribute("tabindex")) container.setAttribute("tabindex", "-1");

      keydownHandler = onKeydown;
      document.addEventListener("keydown", keydownHandler, true);

      const initial = resolveInitial(container, options.initialFocus)
        || getFocusableElements(container)[0]
        || container;
      // Defer to next frame so the container's open transition can start
      // before focus moves (some screen readers misannounce otherwise).
      requestAnimationFrame(() => {
        if (!active) return;
        initial?.focus?.({ preventScroll: true });
      });
    },

    deactivate() {
      if (!active) return;
      active = false;
      if (keydownHandler) {
        document.removeEventListener("keydown", keydownHandler, true);
        keydownHandler = null;
      }
      const target = restoreTo;
      restoreTo = null;
      if (target && typeof target.focus === "function" && document.contains(target)) {
        target.focus({ preventScroll: true });
      }
    },

    isActive() {
      return active;
    },
  };
}
