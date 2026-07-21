import { getDiffSelectedCount, renderDiffSummary, renderDiffTable } from "./diff_table.js";

function isObject(value) {
  return (typeof value === "object" && value !== null) || typeof value === "function";
}

export function installDiffTableFacade(target = globalThis, environment = {}) {
  if (!isObject(target)) throw new TypeError("A facade target is required");

  const api = Object.freeze({
    renderTable: environment.renderDiffTable || renderDiffTable,
    renderSummary: environment.renderDiffSummary || renderDiffSummary,
    selectedCount: environment.getDiffSelectedCount || getDiffSelectedCount,
  });

  const currentNamespace = isObject(target.CarrotUI) ? target.CarrotUI : null;
  const namespace = currentNamespace && Object.isExtensible(currentNamespace)
    ? currentNamespace
    : Object.assign({}, currentNamespace || {});
  namespace.diffTable = api;
  target.CarrotUI = namespace;
  return api;
}
