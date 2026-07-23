import { createSettingsAuxLoaders, createSettingsSnapshotLoader } from "./api.js";
import { createSettingsAuxState } from "./aux_state.js";
import { createSettingsCatalogState } from "./catalog.js";
import {
  createSettingsDerivedModel,
  createSettingsDerivedModelMemo,
  localizedSettingItemText,
  localizedSettingNodeLabel,
  SETTING_DERIVED_IDS,
} from "./derived_model.js";
import { createSettingsEntryController } from "./entry_controller.js";
import {
  createSettingDocumentationClient,
  normalizeSettingDocLanguage,
  resolveSettingDocumentationIndexUrl,
} from "./documentation.js";
import {
  createSettingContextPanel,
  resolveSettingContextTabIndex,
  SETTING_CONTEXT_HISTORY_FETCH_LIMIT,
  SETTING_CONTEXT_HISTORY_PREVIEW_LIMIT,
  SETTING_CONTEXT_TABS,
} from "./context_panel.js";
import {
  createSettingDocumentationView,
  renderSettingDocumentationAst,
  settingDocumentationPlainText,
} from "./documentation_renderer.js";
import {
  CHANGE_SOURCE_LABEL_KEYS,
  formatChangeTimestamp,
  renderSettingHistoryHtml,
  toIsoTimestamp,
} from "./history/change_history.js";
import {
  renderPopularChipHtml,
  renderPopularChipText,
  renderPopularDetailHtml,
} from "./popular/popular_render.js";
import {
  buildPopularDisplayEntry,
  comparePopularItems,
  isPopularValueInRange,
  normalizePopularNumericValue,
  popularPrimaryCount,
  popularSummaryValues,
} from "./popular/popular_values.js";
import { renderFingerprintSummary } from "./fingerprint/fingerprint_summary.js";
import { collectRestoredValues, selectProfileApplyValues } from "./profiles/apply_plan.js";
import { getSettingRiskLevel, renderSettingRiskBadge } from "./risk.js";
import { highlightSearchText } from "./search/highlight.js";
import {
  appendSettingsItemSection,
  createSettingsGroupRenderPlan,
  createSettingsItemLayoutPlan,
  renderSettingsEmptyState,
  renderSettingsGroupList,
} from "./dom_renderer.js";
import { createSettingsStore } from "./store.js";
import { createSettingValueCache } from "./value_cache.js";

const installedTargets = new WeakMap();

function requireTarget(target) {
  if (!target || (typeof target !== "object" && typeof target !== "function")) {
    throw new TypeError("Settings runtime target must be an object");
  }
  return target;
}

export function installSettingsRuntimeFacade(target = globalThis, options = {}) {
  const normalizedTarget = requireTarget(target);
  const installed = installedTargets.get(normalizedTarget);
  if (installed) return installed;

  const store = createSettingsStore({
    loadSnapshot: options.loadSnapshot || createSettingsSnapshotLoader(normalizedTarget),
    eventTarget: options.eventTarget || normalizedTarget,
    now: options.now,
  });
  const aux = createSettingsAuxState(options.auxLoaders || createSettingsAuxLoaders(normalizedTarget));
  const documentation = createSettingDocumentationClient({
    fetchImpl: typeof normalizedTarget.fetch === "function" ? normalizedTarget.fetch.bind(normalizedTarget) : null,
    indexUrl: resolveSettingDocumentationIndexUrl(),
  });
  const derivedModelMemo = createSettingsDerivedModelMemo();
  store.subscribe((state) => {
    if (state.status === "ready" && state.snapshot) aux.hydrateSnapshot(state.snapshot);
  });

  const runtime = Object.freeze({
    store,
    aux,
    catalog: createSettingsCatalogState(),
    derived: Object.freeze({
      createModel: createSettingsDerivedModel,
      getModel: derivedModelMemo.get,
      localizedNodeLabel: localizedSettingNodeLabel,
      localizedItemText: localizedSettingItemText,
      ids: SETTING_DERIVED_IDS,
    }),
    entry: Object.freeze({ createController: createSettingsEntryController }),
    docs: Object.freeze({
      load: documentation.load,
      clear: documentation.clear,
      diagnostics: documentation.diagnostics,
      normalizeLanguage: normalizeSettingDocLanguage,
      createView: createSettingDocumentationView,
      renderAst: renderSettingDocumentationAst,
      plainText: settingDocumentationPlainText,
    }),
    context: Object.freeze({
      create: createSettingContextPanel,
      resolveTabIndex: resolveSettingContextTabIndex,
      tabs: SETTING_CONTEXT_TABS,
      historyFetchLimit: SETTING_CONTEXT_HISTORY_FETCH_LIMIT,
      historyPreviewLimit: SETTING_CONTEXT_HISTORY_PREVIEW_LIMIT,
    }),
    profiles: Object.freeze({
      selectApplyValues: selectProfileApplyValues,
      collectRestoredValues,
    }),
    search: Object.freeze({ highlight: highlightSearchText }),
    fingerprint: Object.freeze({ renderSummary: renderFingerprintSummary }),
    risk: Object.freeze({ level: getSettingRiskLevel, renderBadge: renderSettingRiskBadge }),
    popular: Object.freeze({
      normalizeNumeric: normalizePopularNumericValue,
      isInRange: isPopularValueInRange,
      compareItems: comparePopularItems,
      buildDisplayEntry: buildPopularDisplayEntry,
      primaryCount: popularPrimaryCount,
      summaryValues: popularSummaryValues,
      renderChipText: renderPopularChipText,
      renderChipHtml: renderPopularChipHtml,
      renderDetailHtml: renderPopularDetailHtml,
    }),
    history: Object.freeze({
      renderHtml: renderSettingHistoryHtml,
      formatTimestamp: formatChangeTimestamp,
      toIso: toIsoTimestamp,
      sourceLabelKeys: CHANGE_SOURCE_LABEL_KEYS,
    }),
    view: Object.freeze({
      appendItemSection: appendSettingsItemSection,
      createGroupPlan: createSettingsGroupRenderPlan,
      createItemLayoutPlan: createSettingsItemLayoutPlan,
      renderEmptyState: renderSettingsEmptyState,
      renderGroupList: renderSettingsGroupList,
    }),
    values: createSettingValueCache(),
  });
  const installation = Object.freeze({ store, runtime });

  normalizedTarget.CarrotSettingsStore = store;
  normalizedTarget.CarrotSettingsRuntime = runtime;
  installedTargets.set(normalizedTarget, installation);

  // Begin the no-store snapshot while the remaining classic scripts are parsed.
  // app.js retains its after-first-paint preload as a harmless fallback.
  if (options.preload !== false) store.preload();
  return installation;
}
