"use strict";

const COMPONENT_NAME = "param-selects";
const PARAM_SOURCE = "web_ui";

function normalizedFields(item) {
  const fields = Array.isArray(item?.fields) ? item.fields : [];
  return fields.filter((field) => (
    typeof field?.id === "string"
    && field.id
    && typeof field?.paramName === "string"
    && field.paramName
    && typeof field?.titleKey === "string"
    && field.titleKey
    && Array.isArray(field?.options)
    && field.options.length > 0
    && field.options.every((option) => (
      typeof option?.value === "string"
      && typeof option?.labelKey === "string"
      && option.labelKey
    ))
  ));
}

function text(key) {
  return getUIText(key) || String(key || "");
}

function renderField(field) {
  const title = text(field.titleKey);
  const desc = field.descKey ? text(field.descKey) : "";
  const options = field.options.map((option) => `
    <option value="${escapeHtml(option.value)}">${escapeHtml(text(option.labelKey))}</option>
  `).join("");
  return `
    <label class="web-settings-row web-settings-row--select" data-web-param-setting="${escapeHtml(field.id)}">
      <span class="web-settings-row__copy">
        <span class="web-settings-row__title-line">
          <span class="web-settings-row__title">${escapeHtml(title)}</span>
        </span>
        ${desc ? `<span class="web-settings-row__desc">${escapeHtml(desc)}</span>` : ""}
      </span>
      <select
        class="web-settings-select"
        data-param-name="${escapeHtml(field.paramName)}"
        aria-label="${escapeHtml(title)}"
        disabled
      >
        <option value="" hidden>—</option>
        ${options}
      </select>
    </label>`;
}

function showError(error) {
  if (typeof showAppToast !== "function") return;
  showAppToast(error?.message || String(error), { tone: "error" });
}

async function readParamValues(names) {
  if (typeof globalThis.bulkGet !== "function") {
    throw new Error(text("web_navi_map_settings_unavailable"));
  }
  return globalThis.bulkGet(names);
}

async function writeParamValue(name, value) {
  if (typeof globalThis.setParam !== "function") {
    throw new Error(text("web_navi_map_settings_unavailable"));
  }
  return globalThis.setParam(name, value, { source: PARAM_SOURCE });
}

globalThis.WebSettingsComponents.register(COMPONENT_NAME, {
  isVisible(item) {
    return normalizedFields(item).length > 0;
  },
  render(item) {
    const fields = normalizedFields(item);
    return `
      <div class="web-settings-list" data-web-param-selects="${escapeHtml(item?.id || "")}">
        ${fields.map(renderField).join("")}
      </div>`;
  },
  async bind(root = document) {
    const containers = [...root.querySelectorAll("[data-web-param-selects]")];
    for (const container of containers) {
      if (container.dataset.bound === "1") continue;
      container.dataset.bound = "1";
      const rows = [...container.querySelectorAll("[data-web-param-setting]")];
      const selects = rows
        .map((row) => row.querySelector("select[data-param-name]"))
        .filter(Boolean);
      const names = [...new Set(selects.map((select) => select.dataset.paramName).filter(Boolean))];

      try {
        const values = await readParamValues(names);
        selects.forEach((select) => {
          const current = String(values?.[select.dataset.paramName] ?? "");
          select.value = [...select.options].some((option) => option.value === current) ? current : "";
          select.disabled = false;
        });
      } catch (error) {
        showError(error);
        continue;
      }

      selects.forEach((select) => {
        select.addEventListener("change", async () => {
          const previous = select.dataset.committedValue ?? select.value;
          const next = select.value;
          select.disabled = true;
          try {
            await writeParamValue(select.dataset.paramName, next);
            select.dataset.committedValue = next;
          } catch (error) {
            select.value = previous;
            showError(error);
          } finally {
            select.disabled = false;
          }
        });
        select.dataset.committedValue = select.value;
      });
    }
  },
});

export { COMPONENT_NAME, normalizedFields };
