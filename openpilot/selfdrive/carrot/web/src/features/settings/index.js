import { createSettingsAuxLoaders, createSettingsSnapshotLoader } from "./api.js";
import { createSettingsAuxState } from "./aux_state.js";
import { createSettingsCatalogState } from "./catalog.js";
import {
  createSettingsDerivedModel,
  createSettingsDerivedModelMemo,
  localizedSettingNodeLabel,
  SETTING_DERIVED_IDS,
} from "./derived_model.js";
import { createSettingsEntryController } from "./entry_controller.js";
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
      ids: SETTING_DERIVED_IDS,
    }),
    entry: Object.freeze({ createController: createSettingsEntryController }),
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
