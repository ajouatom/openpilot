/**
 * AutoTuner Controller: Coordinates Dialog, Speech, AI Client, Telemetry, and Param Application
 */

import {
  AI_PROVIDERS,
  callAiTuner,
  clearRollbackSnapshot,
  fetchAvailableModels,
  loadAiConfig,
  loadRollbackSnapshot,
  saveAiConfig,
  saveRollbackSnapshot,
  syncAiConfigWithServer,
} from "./ai_client.js";
import { SpeechInputController } from "./speech.js";
import { renderAutoTunerHtml, renderLoadingHtml, renderResultsHtml, renderPlaceholderHtml, renderRollbackModalHtml } from "./view.js";

function t(key, fallback = "", vars = null) {
  if (typeof globalThis.getUIText === "function") {
    return globalThis.getUIText(key, fallback, vars);
  }
  return fallback;
}

function notify(message, type = "info") {
  if (typeof showToast === "function") {
    showToast(message, type);
  } else {
    alert(message);
  }
}

export function getInitialLayoutMode() {
  try {
    const saved = sessionStorage.getItem("autotuner_layout_mode");
    if (saved === "landscape" || saved === "portrait") {
      return saved;
    }
  } catch {}

  // PC web browser or wide screen default: landscape
  if (typeof window !== "undefined") {
    if (window.innerWidth >= 800) {
      return "landscape";
    }
    if (window.matchMedia && window.matchMedia("(orientation: landscape)").matches && window.innerWidth >= 540) {
      return "landscape";
    }
  }
  return "portrait";
}

export async function openAutoTuner(route, selectedSegments = [], options = {}) {
  if (!route) return;

  const aiConfig = await syncAiConfigWithServer();
  const rollbackSnap = loadRollbackSnapshot();
  const initialMode = getInitialLayoutMode();

  const count = Array.isArray(selectedSegments) ? selectedSegments.length : 0;
  const isAll = options?.isAll !== undefined ? Boolean(options.isAll) : (count === 0 || (options?.totalCount && count === options.totalCount));
  const segmentCount = count > 0 ? count : (options?.totalCount || 0);

  const speechCtrl = new SpeechInputController({
    onTranscript: (text, isFinal) => {
      const textarea = document.getElementById("userIssuePrompt");
      if (textarea) {
        textarea.value = text;
        if (isFinal) textarea.focus();
      }
    },
    onStatusChange: (status) => {
      const micBtn = document.getElementById("btnVoiceInput");
      const micLabel = document.getElementById("micLabel");
      if (micBtn) {
        if (status === "listening") {
          micBtn.classList.add("listening");
          if (micLabel) micLabel.textContent = t("ai_tuning_voice_listening", "듣는 중…");
        } else {
          micBtn.classList.remove("listening");
          if (micLabel) micLabel.textContent = t("ai_tuning_voice_btn", "음성");
        }
      }
    },
    onError: (err) => {
      notify(err, "warning");
    },
  });

  const state = {
    route,
    segmentCount,
    isAllSegments: isAll,
    selectedSegments: selectedSegments || [],
    includeDriverOverride: false,
    aiConfig,
    activeTab: "analysis",
    layoutMode: initialMode,
    manualLayoutOverride: false,
    hasSpeechSupport: speechCtrl.isSupported(),
    hasRollback: Boolean(rollbackSnap),
    currentResult: null,
  };

  const mountId = `autotunerMount_${Date.now()}`;
  const dialogPromise = openAppDialog({
    mode: "alert",
    html: true,
    title: `🤖 ${t("ai_tuning", "AI Tuning")}`,
    messageHtml: `<div id="${mountId}" class="autotuner-mount"></div>`,
    confirmLabel: t("close", "닫기"),
  });

  let resizeHandler = null;

  // Inject markup after dialog render
  setTimeout(() => {
    const mountEl = document.getElementById(mountId);
    if (!mountEl) return;
    mountEl.innerHTML = renderAutoTunerHtml(state);

    const dialogEl = document.getElementById("appDialog");
    if (dialogEl) {
      dialogEl.classList.add("app-dialog--autotuner");
      dialogEl.classList.toggle("app-dialog--autotuner-landscape", state.layoutMode === "landscape");
      dialogEl.classList.toggle("app-dialog--autotuner-portrait", state.layoutMode === "portrait");
    }

    resizeHandler = bindEvents(mountEl, state, speechCtrl);
  }, 50);

  dialogPromise.finally(() => {
    if (resizeHandler && typeof window !== "undefined") {
      window.removeEventListener("resize", resizeHandler);
      window.removeEventListener("orientationchange", resizeHandler);
    }
    const dialogEl = document.getElementById("appDialog");
    if (dialogEl) {
      dialogEl.classList.remove("app-dialog--autotuner", "app-dialog--autotuner-landscape", "app-dialog--autotuner-portrait");
    }
    speechCtrl.destroy();
  });

  return dialogPromise;
}

function bindEvents(root, state, speechCtrl) {
  // Layout Mode Toggling & Responsive Sync
  const applyLayoutMode = (mode, isManual = false) => {
    state.layoutMode = mode;
    if (isManual) {
      state.manualLayoutOverride = true;
      try {
        sessionStorage.setItem("autotuner_layout_mode", mode);
      } catch {}
    }

    const rootEl = root.querySelector("#autoTunerRoot") || root;
    rootEl.classList.toggle("autotuner-mode-landscape", mode === "landscape");
    rootEl.classList.toggle("autotuner-mode-portrait", mode === "portrait");

    const dialogEl = document.getElementById("appDialog");
    if (dialogEl) {
      dialogEl.classList.add("app-dialog--autotuner");
      dialogEl.classList.toggle("app-dialog--autotuner-landscape", mode === "landscape");
      dialogEl.classList.toggle("app-dialog--autotuner-portrait", mode === "portrait");
    }

    const btnLayout = root.querySelector("#btnToggleLayout");
    if (btnLayout) {
      const isLand = mode === "landscape";
      const iconEl = btnLayout.querySelector(".autotuner-layout-icon");
      const textEl = btnLayout.querySelector(".autotuner-layout-text");
      if (iconEl) iconEl.textContent = isLand ? "🖥️" : "📱";
      if (textEl) textEl.textContent = isLand ? t("ai_tuning_view_landscape", "가로 모드 (와이드)") : t("ai_tuning_view_portrait", "세로 모드");
    }
  };

  const btnToggleLayout = root.querySelector("#btnToggleLayout");
  if (btnToggleLayout) {
    btnToggleLayout.addEventListener("click", () => {
      const nextMode = state.layoutMode === "landscape" ? "portrait" : "landscape";
      applyLayoutMode(nextMode, true);
    });
  }

  const onWindowResize = () => {
    if (state.manualLayoutOverride) return;
    const detected = getInitialLayoutMode();
    if (detected !== state.layoutMode) {
      applyLayoutMode(detected, false);
    }
  };

  if (typeof window !== "undefined") {
    window.addEventListener("resize", onWindowResize);
    window.addEventListener("orientationchange", onWindowResize);
  }

  // Tab Switching
  root.querySelectorAll(".autotuner-tab").forEach((tabBtn) => {
    tabBtn.addEventListener("click", () => {
      const tab = tabBtn.dataset.tab;
      state.activeTab = tab;
      root.querySelectorAll(".autotuner-tab").forEach((b) => b.classList.toggle("active", b.dataset.tab === tab));
      root.querySelector("#paneSettings")?.classList.toggle("hidden", tab !== "settings");
      root.querySelector("#paneAnalysis")?.classList.toggle("hidden", tab !== "analysis");
    });
  });

  // Elements
  const providerSelect = root.querySelector("#aiProviderSelect");
  const modelSelect = root.querySelector("#aiModelSelect");
  const personalApiKeyWrap = root.querySelector("#personalApiKeyWrap");
  const personalApiModelWrap = root.querySelector("#personalApiModelWrap");
  const freeModeNotice = root.querySelector("#freeModeNotice");
  const customEndpointField = root.querySelector("#customEndpointField");
  const apiKeyInput = root.querySelector("#aiApiKeyInput");
  const endpointInput = root.querySelector("#aiEndpointInput");
  const btnFetchModels = root.querySelector("#btnFetchModels");
  const statusEl = root.querySelector("#aiConfigStatus");
  const analysisModeBanner = root.querySelector("#analysisModeBanner");
  const btnRollbackGlobal = root.querySelector("#btnRollbackGlobal");

  // Sync API Key from state initially
  const initialProvider = state.aiConfig.provider || "free";
  if (apiKeyInput && initialProvider !== "free") {
    apiKeyInput.value = state.aiConfig.keys?.[initialProvider] || state.aiConfig.apiKey || "";
  }

  const updateModelOptions = async (pKey, key, customUrl) => {
    if (pKey === "free") {
      if (modelSelect) {
        modelSelect.innerHTML = `<option value="built-in">built-in (${t("ai_tuning_free_mode_desc", "규칙 기반 경량 분석")})</option>`;
      }
      return;
    }

    if (!key && pKey !== "custom") {
      if (modelSelect) {
        modelSelect.innerHTML = `<option value="">${t("ai_tuning_enter_key_prompt", "API Key를 입력하면 제공 모델 목록이 표시됩니다")}</option>`;
      }
      return;
    }

    if (btnFetchModels) {
      btnFetchModels.disabled = true;
      btnFetchModels.textContent = t("ai_tuning_fetching_models", "선택 가능한 모델을 불러오는 중…");
    }

    try {
      const models = await fetchAvailableModels(pKey, key, customUrl);
      if (models.length) {
        if (!state.aiConfig.cachedModels) state.aiConfig.cachedModels = {};
        state.aiConfig.cachedModels[pKey] = models;

        const currentSelectedModel = state.aiConfig.selectedModels?.[pKey] || state.aiConfig.model;
        let selectedModel = currentSelectedModel;
        if (!selectedModel) {
          selectedModel = models[0];
        }
        state.aiConfig.model = selectedModel;
        if (!state.aiConfig.selectedModels) state.aiConfig.selectedModels = {};
        state.aiConfig.selectedModels[pKey] = selectedModel;
        saveAiConfig(state.aiConfig);

        if (modelSelect) {
          const list = [...models];
          if (selectedModel && !list.includes(selectedModel)) {
            list.unshift(selectedModel);
          }
          modelSelect.innerHTML = list.map((m) => `<option value="${m}" ${m === selectedModel ? "selected" : ""}>${m}</option>`).join("");
        }
      }
      if (statusEl) {
        statusEl.innerHTML = `<span style="color: #38ef7d;">✅ ${models.length} (${t("ai_tuning_model_label", "제공 모델")})</span>`;
        setTimeout(() => { if (statusEl) statusEl.innerHTML = ""; }, 3000);
      }
    } catch (e) {
      if (statusEl) statusEl.innerHTML = `<span style="color: #ff6b81;">⚠️ ${e.message}</span>`;
    } finally {
      if (btnFetchModels) {
        btnFetchModels.disabled = false;
        btnFetchModels.textContent = `🔄 ${t("ai_tuning_fetch_models_btn", "모델 목록 불러오기")}`;
      }
    }
  };

  // If user previously had a non-free provider configured, fetch models in background
  if (initialProvider !== "free" && state.aiConfig.apiKey) {
    updateModelOptions(initialProvider, state.aiConfig.apiKey, state.aiConfig.customEndpoint);
  }

  // Provider change
  if (providerSelect) {
    providerSelect.addEventListener("change", () => {
      const pKey = providerSelect.value;
      state.aiConfig.provider = pKey;
      const isFree = pKey === "free";

      if (freeModeNotice) freeModeNotice.classList.toggle("hidden", !isFree);
      if (personalApiKeyWrap) personalApiKeyWrap.classList.toggle("hidden", isFree);
      if (personalApiModelWrap) personalApiModelWrap.classList.toggle("hidden", isFree);
      if (customEndpointField) customEndpointField.classList.toggle("hidden", pKey !== "custom");

      // Load saved key & model for this provider
      if (!isFree) {
        const savedKey = state.aiConfig.keys?.[pKey] || "";
        if (apiKeyInput) apiKeyInput.value = savedKey;
        state.aiConfig.apiKey = savedKey;
        state.aiConfig.model = state.aiConfig.selectedModels?.[pKey] || (state.aiConfig.cachedModels?.[pKey]?.[0] || "");
      } else {
        state.aiConfig.model = "built-in";
      }

      saveAiConfig(state.aiConfig);

      const key = apiKeyInput?.value || "";
      const customUrl = endpointInput?.value || "";

      if (isFree) {
        if (modelSelect) modelSelect.innerHTML = `<option value="built-in">built-in (${t("ai_tuning_free_mode_desc", "규칙 기반 경량 분석")})</option>`;
      } else {
        updateModelOptions(pKey, key, customUrl);
      }

      // Update banner in analysis pane
      if (analysisModeBanner) {
        analysisModeBanner.innerHTML = isFree
          ? `💡 <strong>${t("ai_tuning_free_mode_title", "기본 무료 분석 모드")}</strong> (${t("ai_tuning_free_mode_desc", "API 키 불필요, 규칙 기반 경량 분석")})`
          : `✨ ${t("ai_tuning_personal_ai_desc", `개인 연동 AI (${AI_PROVIDERS[pKey]?.name || pKey} / ${state.aiConfig.model || "기본"}) 심층 분석`, { provider: AI_PROVIDERS[pKey]?.name || pKey, model: state.aiConfig.model || "기본" })}`;
      }
    });
  }

  // API Key input live-saving
  if (apiKeyInput) {
    const handleKeyChange = () => {
      const pKey = providerSelect?.value || state.aiConfig.provider || "free";
      const keyVal = apiKeyInput.value.trim();
      if (!state.aiConfig.keys) state.aiConfig.keys = {};
      if (pKey !== "free") {
        state.aiConfig.keys[pKey] = keyVal;
        state.aiConfig.apiKey = keyVal;
        saveAiConfig(state.aiConfig);
      }
    };
    apiKeyInput.addEventListener("input", handleKeyChange);
    apiKeyInput.addEventListener("change", handleKeyChange);
    apiKeyInput.addEventListener("blur", handleKeyChange);
  }

  // Model select change
  if (modelSelect) {
    modelSelect.addEventListener("change", () => {
      state.aiConfig.model = modelSelect.value;
      if (!state.aiConfig.selectedModels) state.aiConfig.selectedModels = {};
      if (state.aiConfig.provider && state.aiConfig.provider !== "free") {
        state.aiConfig.selectedModels[state.aiConfig.provider] = modelSelect.value;
      }
      saveAiConfig(state.aiConfig);
      if (analysisModeBanner && state.aiConfig.provider !== "free") {
        analysisModeBanner.innerHTML = `✨ ${t("ai_tuning_personal_ai_desc", `개인 연동 AI (${AI_PROVIDERS[state.aiConfig.provider]?.name || state.aiConfig.provider} / ${state.aiConfig.model}) 심층 분석`, { provider: AI_PROVIDERS[state.aiConfig.provider]?.name || state.aiConfig.provider, model: state.aiConfig.model })}`;
      }
    });
  }

  // Fetch Models Button
  if (btnFetchModels) {
    btnFetchModels.addEventListener("click", () => {
      const pKey = providerSelect?.value || "gemini";
      const key = apiKeyInput?.value || "";
      const customUrl = endpointInput?.value || "";
      updateModelOptions(pKey, key, customUrl);
    });
  }

  // Save AI Config button
  const btnSaveAiConfig = root.querySelector("#btnSaveAiConfig");
  if (btnSaveAiConfig) {
    btnSaveAiConfig.addEventListener("click", async () => {
      const provider = providerSelect?.value || "free";
      const isFree = provider === "free";
      const model = isFree ? "built-in" : (modelSelect?.value || state.aiConfig.model || "");
      const apiKey = isFree ? "" : (apiKeyInput?.value?.trim() || "");
      const customEndpoint = endpointInput?.value?.trim() || "";

      if (!state.aiConfig.keys) state.aiConfig.keys = {};
      if (!state.aiConfig.selectedModels) state.aiConfig.selectedModels = {};

      if (!isFree) {
        if (apiKey) state.aiConfig.keys[provider] = apiKey;
        if (model) state.aiConfig.selectedModels[provider] = model;
      }

      state.aiConfig.provider = provider;
      state.aiConfig.model = model;
      state.aiConfig.apiKey = apiKey;
      state.aiConfig.customEndpoint = customEndpoint;
      saveAiConfig(state.aiConfig);

      if (!isFree && apiKey && (!state.aiConfig.cachedModels?.[provider] || !state.aiConfig.cachedModels[provider].length)) {
        await updateModelOptions(provider, apiKey, customEndpoint);
      }

      if (analysisModeBanner) {
        analysisModeBanner.innerHTML = isFree
          ? `💡 <strong>${t("ai_tuning_free_mode_title", "기본 무료 분석 모드")}</strong> (${t("ai_tuning_free_mode_desc", "API 키 불필요, 규칙 기반 경량 분석")})`
          : `✨ ${t("ai_tuning_personal_ai_desc", `개인 연동 AI (${AI_PROVIDERS[provider]?.name || provider} / ${state.aiConfig.model}) 심층 분석`, { provider: AI_PROVIDERS[provider]?.name || provider, model: state.aiConfig.model })}`;
      }

      if (statusEl) {
        statusEl.innerHTML = `<span style="color: #38ef7d;">✅ ${t("ai_tuning_settings_saved", "AI 모델 설정이 저장되었습니다.")}</span>`;
        setTimeout(() => { if (statusEl) statusEl.innerHTML = ""; }, 3500);
      }
      notify(t("ai_tuning_settings_saved", "AI 모델 설정이 저장되었습니다."), "success");
    });
  }

  // Global Rollback Button
  if (btnRollbackGlobal) {
    btnRollbackGlobal.addEventListener("click", async () => {
      const snap = loadRollbackSnapshot();
      await showRollbackConfirmModal(root, snap, state, () => {
        root.querySelector("#autotunerRollbackBar")?.classList.add("hidden");
        const container = root.querySelector("#autotunerResultContainer");
        if (container && state.currentResult) {
          container.innerHTML = renderResultsHtml(state.currentResult, false);
          bindResultActions(container, state, root);
        }
      });
    });
  }

  // Voice Input Button
  const btnVoiceInput = root.querySelector("#btnVoiceInput");
  const textarea = root.querySelector("#userIssuePrompt");
  if (btnVoiceInput) {
    btnVoiceInput.addEventListener("click", () => {
      speechCtrl.toggle(textarea?.value || "");
    });
  }

  // Driver Override Checkbox
  const chkDriverOverride = root.querySelector("#chkIncludeDriverOverride");
  if (chkDriverOverride) {
    chkDriverOverride.addEventListener("change", () => {
      state.includeDriverOverride = Boolean(chkDriverOverride.checked);
    });
  }

  // Preset Issue Dropdown
  const presetSelect = root.querySelector("#presetIssueSelect");
  if (presetSelect && textarea) {
    presetSelect.addEventListener("change", () => {
      if (presetSelect.value) {
        textarea.value = presetSelect.value;
      }
    });
  }

  // Start Analysis Button
  const btnStart = root.querySelector("#btnStartAnalysis");
  const resultContainer = root.querySelector("#autotunerResultContainer");

  let activeAbortController = null;

  if (btnStart && resultContainer) {
    btnStart.addEventListener("click", async () => {
      const isFree = !state.aiConfig.provider || state.aiConfig.provider === "free";

      if (!isFree) {
        const currentInputKey = apiKeyInput?.value?.trim();
        if (currentInputKey) {
          if (!state.aiConfig.keys) state.aiConfig.keys = {};
          state.aiConfig.keys[state.aiConfig.provider] = currentInputKey;
          state.aiConfig.apiKey = currentInputKey;
          saveAiConfig(state.aiConfig);
        } else if (!state.aiConfig.apiKey && state.aiConfig.keys?.[state.aiConfig.provider]) {
          state.aiConfig.apiKey = state.aiConfig.keys[state.aiConfig.provider];
        }

        if (!state.aiConfig.apiKey && state.aiConfig.provider !== "custom") {
          notify(t("ai_tuning_enter_key_prompt", "선택한 AI 제공자의 API Key를 [AI 모델 설정]에서 입력해주세요."), "warning");
          root.querySelector('.autotuner-tab[data-tab="settings"]')?.click();
          return;
        }
      }

      if (activeAbortController) {
        activeAbortController.abort();
      }
      activeAbortController = new AbortController();
      const { signal } = activeAbortController;

      const userPrompt = textarea?.value?.trim() || presetSelect?.value || "";
      btnStart.disabled = true;
      resultContainer.innerHTML = renderLoadingHtml(
        isFree
          ? t("ai_tuning_analyzing", "주행로그 분석 중…")
          : t("ai_tuning_personal_ai_desc", `주행로그 추출 및 AI (${state.aiConfig.provider} / ${state.aiConfig.model || "기본"}) 추론 중…`, { provider: state.aiConfig.provider, model: state.aiConfig.model || "기본" })
      );

      // Wire cancel button
      const btnCancel = resultContainer.querySelector("#btnCancelAnalysis");
      if (btnCancel) {
        btnCancel.addEventListener("click", () => {
          if (activeAbortController) {
            activeAbortController.abort();
            activeAbortController = null;
          }
          btnStart.disabled = false;
          resultContainer.innerHTML = renderPlaceholderHtml();
          notify(t("ai_tuning_cancel_analysis_btn", "AI 주행로그 분석이 취소되었습니다."), "info");
        });
      }

      // Safety timeout: abort after 35 seconds if network or AI stalls
      const safetyTimeout = setTimeout(() => {
        if (activeAbortController) {
          activeAbortController.abort();
          activeAbortController = null;
          resultContainer.innerHTML = `
            <div style="background: rgba(255, 71, 87, 0.15); border: 1px solid #ff4757; padding: 10px; border-radius: 6px; color: #ff6b81; font-size: 12px; margin-top: 6px;">
              <strong>⏱️ ${t("timeout", "시간 초과")}:</strong> ${t("ai_tuning_analyzing", "AI 응답 시간이 초과되었습니다 (35초).")}
            </div>`;
          btnStart.disabled = false;
        }
      }, 35000);

      try {
        // Step 1: Extract telemetry from server
        const res = await fetch("/api/autotune/extract-telemetry", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            route: state.route,
            segments: state.selectedSegments,
            includeDriverOverride: Boolean(state.includeDriverOverride),
          }),
          signal,
        });
        const telemetryData = await res.json();
        if (!res.ok || !telemetryData.ok) {
          throw new Error(telemetryData?.error || "Failed to load telemetry.");
        }

        if (telemetryData.segmentCount) {
          state.segmentCount = telemetryData.segmentCount;
          const chipEl = root.querySelector("#autotunerSegmentChip");
          if (chipEl) {
            chipEl.textContent = state.isAllSegments
              ? t("ai_tuning_all_segments", `전체 ${telemetryData.segmentCount}개 세그먼트`, { count: telemetryData.segmentCount })
              : t("ai_tuning_selected_segments", `${telemetryData.segmentCount}개 세그먼트`, { count: telemetryData.segmentCount });
          }
        }

        // Step 2: Call AI Tuner (Built-in Free Engine or External LLM)
        if (!isFree) {
          resultContainer.innerHTML = renderLoadingHtml(
            t("ai_tuning_personal_ai_desc", `AI (${state.aiConfig.provider} / ${state.aiConfig.model}) 추론 중…`, { provider: state.aiConfig.provider, model: state.aiConfig.model })
          );
          const btnCancel2 = resultContainer.querySelector("#btnCancelAnalysis");
          if (btnCancel2) {
            btnCancel2.addEventListener("click", () => {
              if (activeAbortController) {
                activeAbortController.abort();
                activeAbortController = null;
              }
              btnStart.disabled = false;
              resultContainer.innerHTML = renderPlaceholderHtml();
              notify(t("ai_tuning_cancel_analysis_btn", "AI 주행로그 분석이 취소되었습니다."), "info");
            });
          }
        }
        const aiResult = await callAiTuner(telemetryData, userPrompt, state.aiConfig, signal, {
          includeDriverOverride: Boolean(state.includeDriverOverride),
        });
        state.currentResult = aiResult;

        // Step 3: Render results
        const hasRollback = Boolean(loadRollbackSnapshot());
        resultContainer.innerHTML = renderResultsHtml(aiResult, hasRollback);
        bindResultActions(resultContainer, state, root);

      } catch (err) {
        if (signal.aborted) {
          return;
        }
        resultContainer.innerHTML = `
          <div style="background: rgba(255, 71, 87, 0.15); border: 1px solid #ff4757; padding: 10px; border-radius: 6px; color: #ff6b81; font-size: 12px; margin-top: 6px;">
            <strong>❌ ${t("error", "오류")}:</strong> ${escapeHtml(err.message)}
          </div>`;
      } finally {
        clearTimeout(safetyTimeout);
        activeAbortController = null;
        btnStart.disabled = false;
      }
    });
  }

  return onWindowResize;
}

function bindResultActions(container, state, root) {
  // Toggle all checkboxes
  const btnToggleAll = container.querySelector("#btnToggleAllParams");
  if (btnToggleAll) {
    btnToggleAll.addEventListener("click", () => {
      const checkboxes = container.querySelectorAll(".autotuner-param-check");
      const allChecked = Array.from(checkboxes).every((c) => c.checked);
      checkboxes.forEach((c) => (c.checked = !allChecked));
    });
  }

  // Apply Selected Parameters
  const btnApply = container.querySelector("#btnApplySelectedParams");
  if (btnApply) {
    btnApply.addEventListener("click", async () => {
      const checkedBoxes = container.querySelectorAll(".autotuner-param-check:checked");
      if (!checkedBoxes.length) {
        notify(t("ai_tuning_apply_no_selection", "적용할 파라미터를 하나 이상 선택해주세요."), "warning");
        return;
      }

      const paramsToSet = {};
      checkedBoxes.forEach((cb) => {
        paramsToSet[cb.dataset.name] = cb.dataset.value;
      });

      btnApply.disabled = true;
      btnApply.textContent = t("ai_tuning_applying", "적용 중…");

      try {
        const res = await fetch("/api/autotune/apply-params", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ params: paramsToSet }),
        });
        const data = await res.json();
        if (!res.ok || !data.ok) {
          const detail = data?.error || (data?.errors ? Object.entries(data.errors).map(([k, v]) => `${k}: ${v}`).join(", ") : "");
          throw new Error(detail || "Failed to apply parameters.");
        }

        if (data.rollbackSnapshot && Object.keys(data.rollbackSnapshot).length > 0) {
          saveRollbackSnapshot(data.rollbackSnapshot, paramsToSet);
          if (root) {
            root.querySelector("#autotunerRollbackBar")?.classList.remove("hidden");
          }
        }

        const paramCount = Object.keys(paramsToSet).length;
        notify(t("ai_tuning_apply_success", `선택한 {count}개 파라미터가 성공적으로 적용되었습니다!`, { count: paramCount }), "success");
        btnApply.textContent = `✅ ${t("apply", "적용")} ${t("complete", "완료")}`;

        // Re-render results with rollback button visible
        container.innerHTML = renderResultsHtml(state.currentResult, true);
        bindResultActions(container, state, root);

      } catch (err) {
        notify(t("ai_tuning_apply_failed", `적용 실패: {error}`, { error: err.message }), "error");
        btnApply.disabled = false;
        btnApply.textContent = `✨ ${t("ai_tuning_apply_selected", "선택 파라미터 적용")}`;
      }
    });
  }

  // Rollback button
  const btnRollback = container.querySelector("#btnRollbackParams");
  if (btnRollback) {
    btnRollback.addEventListener("click", async () => {
      const snap = loadRollbackSnapshot();
      await showRollbackConfirmModal(root, snap, state, () => {
        container.innerHTML = renderResultsHtml(state.currentResult, false);
        bindResultActions(container, state, root);
        if (root) {
          root.querySelector("#autotunerRollbackBar")?.classList.add("hidden");
        }
      });
    });
  }
}

async function showRollbackConfirmModal(root, snap, state = null, onComplete = () => {}) {
  if (!snap || !snap.params || Object.keys(snap.params).length === 0) {
    notify(t("ai_tuning_rollback_no_backup", "복원할 롤백 백업이 없습니다."), "warning");
    return;
  }

  // Remove existing modal if any
  const existing = root.querySelector("#autotunerRollbackModalOverlay");
  if (existing) existing.remove();

  // Resolve current applied values
  const currentMap = { ...(snap.currentApplied || {}) };
  if (state?.currentResult?.recommendedParams) {
    for (const p of state.currentResult.recommendedParams) {
      if (p.name && !currentMap[p.name]) {
        currentMap[p.name] = p.recommended;
      }
    }
  }

  // Query live parameter values from openpilot if bulkGet is available
  const paramNames = Object.keys(snap.params);
  if (typeof globalThis.bulkGet === "function" && paramNames.length > 0) {
    try {
      const liveVals = await globalThis.bulkGet(paramNames);
      if (liveVals && typeof liveVals === "object") {
        for (const [k, v] of Object.entries(liveVals)) {
          if (v !== undefined && v !== null && v !== "") {
            currentMap[k] = v;
          }
        }
      }
    } catch {}
  }

  const temp = document.createElement("div");
  temp.innerHTML = renderRollbackModalHtml(snap.params, currentMap);
  const modalEl = temp.firstElementChild;
  if (!modalEl) return;

  const targetHost = root.querySelector("#autoTunerRoot") || root;
  targetHost.appendChild(modalEl);

  const closeModal = () => {
    modalEl.remove();
  };

  const btnClose = modalEl.querySelector("#btnCloseRollbackModal");
  const btnCancel = modalEl.querySelector("#btnCancelRollbackModal");
  const btnConfirm = modalEl.querySelector("#btnConfirmRollbackModal");
  const chkAll = modalEl.querySelector("#btnToggleAllRollbackParams");
  const getChecks = () => Array.from(modalEl.querySelectorAll(".autotuner-rollback-check"));

  const updateConfirmButton = () => {
    const checks = getChecks();
    const checkedCount = checks.filter(c => c.checked).length;
    if (btnConfirm) {
      btnConfirm.textContent = `↩️ ${t("ai_tuning_rollback_execute", `선택 파라미터 롤백 (${checkedCount}개)`, { count: checkedCount })}`;
      btnConfirm.disabled = checkedCount === 0;
    }
    if (chkAll) {
      chkAll.checked = checks.length > 0 && checkedCount === checks.length;
      chkAll.indeterminate = checkedCount > 0 && checkedCount < checks.length;
    }
  };

  if (btnClose) btnClose.addEventListener("click", closeModal);
  if (btnCancel) btnCancel.addEventListener("click", closeModal);

  modalEl.addEventListener("click", (e) => {
    if (e.target === modalEl) closeModal();
  });

  if (chkAll) {
    chkAll.addEventListener("change", () => {
      const isChecked = chkAll.checked;
      getChecks().forEach(c => { c.checked = isChecked; });
      updateConfirmButton();
    });
  }

  getChecks().forEach(c => {
    c.addEventListener("change", updateConfirmButton);
  });

  if (btnConfirm) {
    btnConfirm.addEventListener("click", async () => {
      const selectedChecks = getChecks().filter(c => c.checked);
      if (!selectedChecks.length) {
        notify(t("ai_tuning_rollback_select_warning", "되돌릴 파라미터를 하나 이상 선택해주세요."), "warning");
        return;
      }

      const paramsToRollback = {};
      selectedChecks.forEach(c => {
        paramsToRollback[c.dataset.name] = c.dataset.value;
      });

      btnConfirm.disabled = true;
      btnConfirm.textContent = t("ai_tuning_rollback_running", "롤백 실행 중…");
      if (btnCancel) btnCancel.disabled = true;

      try {
        const res = await fetch("/api/autotune/apply-params", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ params: paramsToRollback }),
        });
        const data = await res.json();
        if (!res.ok || !data.ok) {
          const detail = data?.error || (data?.errors ? Object.entries(data.errors).map(([k, v]) => `${k}: ${v}`).join(", ") : "");
          throw new Error(detail || "Failed to execute rollback.");
        }

        // Update or clear rollback snapshot
        const remainingParams = { ...(snap.params || {}) };
        const remainingApplied = { ...(snap.currentApplied || {}) };
        for (const k of Object.keys(paramsToRollback)) {
          delete remainingParams[k];
          delete remainingApplied[k];
        }

        if (Object.keys(remainingParams).length > 0) {
          saveRollbackSnapshot(remainingParams, remainingApplied);
        } else {
          clearRollbackSnapshot();
        }

        const paramCount = Object.keys(paramsToRollback).length;
        notify(`↩️ ${t("ai_tuning_rollback_success", `{count}개 파라미터가 직전 설정으로 안전하게 복원되었습니다.`, { count: paramCount })}`, "success");
        closeModal();
        onComplete(true);
      } catch (err) {
        notify(`${t("ai_tuning_rollback_failed", `롤백 실패: {error}`, { error: err.message })}`, "error");
        btnConfirm.disabled = false;
        updateConfirmButton();
        if (btnCancel) btnCancel.disabled = false;
      }
    });
  }
}

