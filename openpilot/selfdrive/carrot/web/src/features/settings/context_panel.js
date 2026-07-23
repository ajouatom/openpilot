const TAB_DEFINITIONS = Object.freeze([
  Object.freeze({
    id: "description",
    labelKey: "setting_context_description",
    labelFallback: "Description",
    emptyKey: "setting_context_description_empty",
    emptyFallback: "No detailed description is available yet.",
  }),
  Object.freeze({
    id: "popular",
    labelKey: "setting_context_popular",
    labelFallback: "Popular",
    emptyKey: "setting_context_popular_empty",
    emptyFallback: "No popular-value data is available yet.",
  }),
  Object.freeze({
    id: "history",
    labelKey: "setting_context_history",
    labelFallback: "History",
    emptyKey: "setting_context_history_empty",
    emptyFallback: "No changes have been recorded yet.",
  }),
]);

const TAB_IDS = Object.freeze(TAB_DEFINITIONS.map((tab) => tab.id));
let panelSequence = 0;

export const SETTING_CONTEXT_HISTORY_PREVIEW_LIMIT = 3;
export const SETTING_CONTEXT_HISTORY_FETCH_LIMIT = 10;

function defaultText(_key, fallback, params) {
  let output = String(fallback ?? "");
  Object.entries(params || {}).forEach(([name, value]) => {
    output = output.replaceAll(`{${name}}`, String(value));
  });
  return output;
}

function tabDefinition(id) {
  return TAB_DEFINITIONS.find((tab) => tab.id === id) || TAB_DEFINITIONS[0];
}

function formattedCount(value, locale) {
  const count = Math.max(0, Number(value) || 0);
  try {
    return new Intl.NumberFormat(locale || undefined, { maximumFractionDigits: 0 }).format(count);
  } catch (_) {
    return String(Math.round(count));
  }
}

export function resolveSettingContextTabIndex(currentIndex, key, count = TAB_IDS.length) {
  const size = Math.max(1, Number(count) || 1);
  const current = Math.max(0, Math.min(size - 1, Number(currentIndex) || 0));
  if (key === "ArrowRight") return (current + 1) % size;
  if (key === "ArrowLeft") return (current - 1 + size) % size;
  if (key === "Home") return 0;
  if (key === "End") return size - 1;
  return current;
}

export function createSettingContextPanel(options = {}) {
  const document = options.document || globalThis.document;
  if (!document?.createElement) throw new TypeError("A document is required");
  const text = typeof options.text === "function" ? options.text : defaultText;
  const locale = String(options.locale || "").trim() || undefined;
  const instanceId = `setting-context-${++panelSequence}`;

  const root = document.createElement("section");
  root.className = "setting-context";
  root.setAttribute(
    "aria-label",
    text("setting_context_aria", "Setting information"),
  );

  const tabList = document.createElement("div");
  tabList.className = "setting-context__tabs c-segmented-control";
  tabList.setAttribute("role", "tablist");
  tabList.setAttribute(
    "aria-label",
    text("setting_context_tabs_aria", "Setting information sections"),
  );

  const tabs = new Map();
  const panels = new Map();
  const contents = new Map();

  function activate(id, moveFocus = false) {
    const nextId = TAB_IDS.includes(id) ? id : TAB_IDS[0];
    TAB_IDS.forEach((tabId) => {
      const selected = tabId === nextId;
      const tab = tabs.get(tabId);
      const panel = panels.get(tabId);
      tab.setAttribute("aria-selected", String(selected));
      tab.tabIndex = selected ? 0 : -1;
      panel.hidden = !selected;
    });
    if (moveFocus) tabs.get(nextId)?.focus();
    root.dataset.activePanel = nextId;
    return nextId;
  }

  TAB_DEFINITIONS.forEach((definition, index) => {
    const tab = document.createElement("button");
    tab.type = "button";
    tab.className = "setting-context__tab c-segmented-control__item";
    tab.id = `${instanceId}-tab-${definition.id}`;
    tab.setAttribute("role", "tab");
    tab.setAttribute("aria-controls", `${instanceId}-panel-${definition.id}`);
    tab.setAttribute("aria-selected", String(index === 0));
    tab.tabIndex = index === 0 ? 0 : -1;

    const label = document.createElement("span");
    label.className = "setting-context__tab-label";
    label.textContent = text(definition.labelKey, definition.labelFallback);
    tab.appendChild(label);
    if (definition.id === "history") {
      const count = document.createElement("span");
      count.className = "setting-context__tab-count";
      count.textContent = "0";
      count.setAttribute("aria-label", text("setting_context_history_count", "{n} changes", { n: 0 }));
      tab.appendChild(count);
    }

    const panel = document.createElement("div");
    panel.className = `setting-context__panel setting-context__panel--${definition.id}`;
    panel.id = `${instanceId}-panel-${definition.id}`;
    panel.setAttribute("role", "tabpanel");
    panel.setAttribute("aria-labelledby", tab.id);
    panel.setAttribute("aria-live", "polite");
    panel.hidden = index !== 0;

    const content = document.createElement("div");
    content.className = "setting-context__content";
    panel.appendChild(content);

    tab.addEventListener("click", () => activate(definition.id));
    tab.addEventListener("keydown", (event) => {
      if (!["ArrowLeft", "ArrowRight", "Home", "End"].includes(event.key)) return;
      event.preventDefault();
      const currentIndex = TAB_IDS.indexOf(definition.id);
      activate(TAB_IDS[resolveSettingContextTabIndex(currentIndex, event.key)], true);
    });

    tabs.set(definition.id, tab);
    panels.set(definition.id, panel);
    contents.set(definition.id, content);
    tabList.appendChild(tab);
    root.appendChild(panel);
  });
  root.prepend(tabList);

  const historyDialog = document.createElement("dialog");
  historyDialog.className = "setting-context__history-dialog";
  historyDialog.setAttribute("aria-labelledby", `${instanceId}-history-title`);
  const dialogHeader = document.createElement("header");
  dialogHeader.className = "setting-context__dialog-header";
  const dialogTitle = document.createElement("h3");
  dialogTitle.id = `${instanceId}-history-title`;
  dialogTitle.textContent = text("setting_context_history_all_title", "All change history");
  const dialogClose = document.createElement("button");
  dialogClose.type = "button";
  dialogClose.className = "setting-context__dialog-close";
  dialogClose.textContent = "×";
  dialogClose.setAttribute("aria-label", text("close", "Close"));
  const dialogBody = document.createElement("div");
  dialogBody.className = "setting-context__dialog-body";
  dialogHeader.append(dialogTitle, dialogClose);
  historyDialog.append(dialogHeader, dialogBody);
  root.appendChild(historyDialog);

  let historyTrigger = null;
  function closeHistoryDialog() {
    if (typeof historyDialog.close === "function" && historyDialog.open) historyDialog.close();
    else historyDialog.removeAttribute("open");
    historyTrigger?.focus();
  }
  dialogClose.addEventListener("click", closeHistoryDialog);
  historyDialog.addEventListener("cancel", (event) => {
    event.preventDefault();
    closeHistoryDialog();
  });
  historyDialog.addEventListener("click", (event) => {
    if (event.target === historyDialog) closeHistoryDialog();
  });

  function updateHistoryCount(count) {
    const badge = tabs.get("history")?.querySelector(".setting-context__tab-count");
    if (!badge) return;
    const visible = formattedCount(count, locale);
    badge.textContent = visible;
    badge.setAttribute(
      "aria-label",
      text("setting_context_history_count", "{n} changes", { n: visible }),
    );
  }

  function stateNode(id, message, state) {
    const node = document.createElement("div");
    node.className = `setting-context__state setting-context__state--${state}`;
    const title = document.createElement("strong");
    title.textContent = message || text(
      tabDefinition(id).emptyKey,
      tabDefinition(id).emptyFallback,
    );
    node.appendChild(title);
    return node;
  }

  function replacePanel(id, content, panelOptions = {}) {
    if (!contents.has(id)) throw new RangeError(`Unknown setting context panel: ${id}`);
    const target = contents.get(id);
    const state = panelOptions.state || "ready";
    target.replaceChildren(content);
    target.dataset.state = state;
    panels.get(id).setAttribute("aria-busy", String(state === "loading"));
    if (id === "history" && panelOptions.count !== undefined) {
      updateHistoryCount(panelOptions.count);
    }
  }

  function setLoading(id, message = "") {
    replacePanel(
      id,
      stateNode(
        id,
        message || text("setting_context_loading", "Loading..."),
        "loading",
      ),
      { state: "loading", ...(id === "history" ? { count: 0 } : {}) },
    );
  }

  function setEmpty(id, message = "") {
    replacePanel(
      id,
      stateNode(id, message, "empty"),
      { state: "empty", ...(id === "history" ? { count: 0 } : {}) },
    );
  }

  function setContent(id, content, panelOptions = {}) {
    if (!content) {
      setEmpty(id, panelOptions.emptyMessage || "");
      return;
    }
    replacePanel(id, content, { ...panelOptions, state: "ready" });
  }

  function setHistoryContent(summaryContent, fullContent, panelOptions = {}) {
    const count = Math.max(0, Number(panelOptions.count) || 0);
    if (!summaryContent || count === 0) {
      dialogBody.replaceChildren();
      setEmpty("history", panelOptions.emptyMessage || "");
      return;
    }

    const container = document.createElement("div");
    container.className = "setting-context__history-summary";
    container.appendChild(summaryContent);
    if (fullContent && count > Number(
      panelOptions.summaryCount || SETTING_CONTEXT_HISTORY_PREVIEW_LIMIT,
    )) {
      const openButton = document.createElement("button");
      openButton.type = "button";
      openButton.className = "setting-context__more";
      openButton.textContent = text("setting_context_history_all", "View all history");
      openButton.addEventListener("click", () => {
        historyTrigger = openButton;
        if (typeof historyDialog.showModal === "function") historyDialog.showModal();
        else historyDialog.setAttribute("open", "");
        dialogClose.focus();
      });
      container.appendChild(openButton);
      dialogBody.replaceChildren(fullContent);
    } else {
      dialogBody.replaceChildren();
    }
    setContent("history", container, { count });
  }

  setLoading("description", text("setting_doc_loading", "Loading guide..."));
  setEmpty("popular");
  setEmpty("history");
  activate("description");

  return Object.freeze({
    root,
    activate,
    getPanel: (id) => panels.get(id) || null,
    getContent: (id) => contents.get(id) || null,
    setLoading,
    setEmpty,
    setContent,
    setHistoryContent,
    closeHistoryDialog,
  });
}

export const SETTING_CONTEXT_TABS = TAB_IDS;
