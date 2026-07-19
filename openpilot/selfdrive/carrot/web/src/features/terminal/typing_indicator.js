"use strict";

(function () {
  function commonPrefixLength(left, right) {
    const limit = Math.min(left.length, right.length);
    let index = 0;
    while (index < limit && left[index] === right[index]) index += 1;
    return index;
  }

  function create(host) {
    if (!host) return null;

    const indicator = document.createElement("div");
    indicator.className = "terminal-typing-indicator";

    const presence = document.createElement("span");
    presence.className = "terminal-typing-indicator__presence";
    presence.setAttribute("aria-hidden", "true");

    const actor = document.createElement("span");
    actor.className = "terminal-typing-indicator__actor";

    const draft = document.createElement("code");
    draft.className = "terminal-typing-indicator__draft";

    indicator.append(presence, actor, draft);
    host.replaceChildren(indicator);

    let previousText = "";

    function renderDraft(text, emptyLabel) {
      const nextText = String(text || "");
      draft.replaceChildren();
      draft.classList.toggle("is-empty", !nextText);

      if (!nextText) {
        const label = document.createElement("span");
        label.className = "terminal-typing-indicator__emptyLabel";
        label.textContent = emptyLabel;
        const dots = document.createElement("span");
        dots.className = "terminal-typing-indicator__dots";
        dots.setAttribute("aria-hidden", "true");
        for (let index = 0; index < 3; index += 1) {
          const dot = document.createElement("span");
          dot.style.setProperty("--typing-dot-index", String(index));
          dots.append(dot);
        }
        draft.append(label, dots);
        previousText = "";
        return;
      }

      const prefixLength = commonPrefixLength(previousText, nextText);
      if (prefixLength > 0) draft.append(document.createTextNode(nextText.slice(0, prefixLength)));
      const added = nextText.slice(prefixLength);
      for (let index = 0; index < added.length; index += 1) {
        const char = document.createElement("span");
        char.className = "terminal-typing-indicator__char";
        char.style.setProperty("--typing-char-index", String(Math.min(index, 5)));
        char.textContent = added[index];
        draft.append(char);
      }
      const caret = document.createElement("span");
      caret.className = "terminal-typing-indicator__caret";
      caret.setAttribute("aria-hidden", "true");
      draft.append(caret);
      previousText = nextText;
    }

    function update(options = {}) {
      const active = Boolean(options.active);
      host.classList.toggle("is-visible", active);
      host.setAttribute("aria-hidden", active ? "false" : "true");
      if (!active) {
        previousText = "";
        return;
      }
      actor.textContent = String(options.actor || "");
      renderDraft(options.text, String(options.emptyLabel || "Typing"));
    }

    function destroy() {
      previousText = "";
      indicator.remove();
    }

    return Object.freeze({ update, destroy });
  }

  window.CarrotTerminalTypingIndicator = Object.freeze({ create });
})();
