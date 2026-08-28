/**
 * AutoTuner View Templates & Markup Generator
 */

import { AI_PROVIDERS } from "./ai_client.js";

function t(key, fallback = "", vars = null) {
  if (typeof globalThis.getUIText === "function") {
    return globalThis.getUIText(key, fallback, vars);
  }
  return fallback;
}

function escapeHtml(str) {
  if (typeof str !== "string") return String(str ?? "");
  return str.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;").replace(/"/g, "&quot;");
}

export function getLocalizedPresetIssues() {
  return [
    { label: t("ai_tuning_preset_default_label", "💡 종합 주행 상태 분석 (디폴트)"), value: "" },
    {
      label: t("ai_tuning_preset_lead_catchup_label", "⚡ 선행차 캐치업 급가속"),
      value: t("ai_tuning_preset_lead_catchup_text", "정차 후 선행차 출발 시 또는 주행 중 선행차를 따라갈 때 가속이 너무 급하게 튀어나가듯 느껴집니다. 급가속 상한을 낮추고 부드럽게 캐치업하도록 파라미터를 조정해주세요."),
    },
    {
      label: t("ai_tuning_preset_lead_depart_label", "🐢 선행차 출발시 가속 느림"),
      value: t("ai_tuning_preset_lead_depart_text", "신호 대기 정차 후 선행차가 출발할 때 내 차의 출발 반응이나 가속이 너무 느려 거리가 크게 벌어집니다. 신속하고 자연스럽게 출발하여 따라붙도록 파라미터를 조정해주세요."),
    },
    {
      label: t("ai_tuning_preset_stop_shock_label", "🛑 정지차 인식 급브레이크 (소프트랜딩)"),
      value: t("ai_tuning_preset_stop_shock_text", "전방 정지차나 감속 차량을 인식할 때 브레이크가 너무 급하게 걸리거나 정차 직전 울컥거림이 있습니다. 부드러운 사전 감속 및 정차(소프트랜딩)가 되도록 파라미터를 조정해주세요."),
    },
    {
      label: t("ai_tuning_preset_steer_straight_label", "〰️ 직선로 쏠림 / 피쉬테일"),
      value: t("ai_tuning_preset_steer_straight_text", "직선 도로 주행 중 차선 중앙을 유지하지 못하고 한쪽으로 쏠리거나 좌우로 미세하게 흔들림(털림)이 발생합니다. 직선 주행 안정성을 높이도록 조향 파라미터를 조정해주세요."),
    },
    {
      label: t("ai_tuning_preset_steer_curve_label", "↩️ 곡선로 쏠림 / 오버스티어"),
      value: t("ai_tuning_preset_steer_curve_text", "커브길 및 곡선로 진입 시 차선 바깥쪽으로 밀리거나 안쪽으로 파고드는 쏠림 현상이 있습니다. 곡선 곡률 추종 능력을 개선하도록 조향 파라미터를 조정해주세요."),
    },
  ];
}

export const PRESET_ISSUES = getLocalizedPresetIssues();

export function renderAutoTunerHtml(state) {
  const { route, segmentCount, isAllSegments, aiConfig: rawConfig, activeTab, hasSpeechSupport, hasRollback, layoutMode = "landscape", currentResult } = state || {};
  const aiConfig = rawConfig || { provider: "free", model: "built-in" };
  const isLandscape = layoutMode === "landscape";
  const isFree = !aiConfig.provider || aiConfig.provider === "free";
  const currentProvider = AI_PROVIDERS[aiConfig.provider] || AI_PROVIDERS.free;

  const countNum = typeof segmentCount === "number" ? segmentCount : parseInt(segmentCount, 10);
  let segmentBadgeText = "";
  if (!isNaN(countNum) && countNum > 0) {
    segmentBadgeText = isAllSegments
      ? t("ai_tuning_all_segments", `전체 ${countNum}개 세그먼트`, { count: countNum })
      : t("ai_tuning_selected_segments", `${countNum}개 세그먼트`, { count: countNum });
  } else if (segmentCount && segmentCount !== "전체") {
    segmentBadgeText = t("ai_tuning_selected_segments", `${segmentCount}개 세그먼트`, { count: segmentCount });
  } else {
    segmentBadgeText = t("ai_tuning_all_segments_full", "전체 세그먼트");
  }

  const providerOptions = Object.entries(AI_PROVIDERS)
    .map(([key, p]) => `<option value="${key}" ${key === (aiConfig.provider || "free") ? "selected" : ""}>${escapeHtml(p.name)}</option>`)
    .join("");

  const cachedList = aiConfig.cachedModels?.[aiConfig.provider] || currentProvider.models || [];
  let modelOptions = "";
  if (isFree) {
    modelOptions = `<option value="built-in">built-in (${t("ai_tuning_free_mode_desc", "규칙 기반 경량 분석")})</option>`;
  } else if (cachedList.length > 0) {
    const listToRender = [...cachedList];
    if (aiConfig.model && !listToRender.includes(aiConfig.model)) {
      listToRender.unshift(aiConfig.model);
    }
    modelOptions = listToRender
      .map((m) => `<option value="${m}" ${m === aiConfig.model ? "selected" : ""}>${escapeHtml(m)}</option>`)
      .join("");
  } else if (aiConfig.model) {
    modelOptions = `<option value="${escapeHtml(aiConfig.model)}" selected>${escapeHtml(aiConfig.model)}</option>`;
  } else {
    modelOptions = `<option value="">${escapeHtml(aiConfig.apiKey ? t("ai_tuning_fetching_models", "선택 가능한 모델을 불러오는 중…") : t("ai_tuning_enter_key_prompt", "API Key를 입력하면 제공 모델 목록이 표시됩니다"))}</option>`;
  }

  const presets = getLocalizedPresetIssues();
  const presetOptions = presets.map((issue) => `<option value="${escapeHtml(issue.value)}">${escapeHtml(issue.label)}</option>`).join("");

  return `
  <div class="autotuner-dialog ${isLandscape ? "autotuner-mode-landscape" : "autotuner-mode-portrait"}" id="autoTunerRoot">
    <!-- Top Header: Tabs & Layout Mode Switcher -->
    <div class="autotuner-top-bar">
      <div class="autotuner-tabs">
        <button class="autotuner-tab ${activeTab === "analysis" ? "active" : ""}" type="button" data-tab="analysis">
          📊 ${escapeHtml(t("ai_tuning_tab_diagnose", "주행 로그 분석"))}
        </button>
        <button class="autotuner-tab ${activeTab === "settings" ? "active" : ""}" type="button" data-tab="settings">
          ⚙️ ${escapeHtml(t("ai_tuning_tab_settings", "AI 모델 설정"))}
        </button>
      </div>
      <div class="autotuner-layout-controls">
        <button class="autotuner-layout-btn" type="button" id="btnToggleLayout" title="${escapeHtml(t("ai_tuning_view_landscape", "화면 가로/세로 레이아웃 전환"))}">
          <span class="autotuner-layout-icon">${isLandscape ? "🖥️" : "📱"}</span>
          <span class="autotuner-layout-text">${escapeHtml(isLandscape ? t("ai_tuning_view_landscape", "가로 모드 (와이드)") : t("ai_tuning_view_portrait", "세로 모드"))}</span>
          <span class="autotuner-layout-badge">${escapeHtml(t("ai_tuning_layout_toggle_badge", "전환 ⇄"))}</span>
        </button>
      </div>
    </div>

    <!-- Tab 1: Settings Pane -->
    <div class="autotuner-tab-pane ${activeTab === "settings" ? "" : "hidden"}" id="paneSettings">
      <div class="autotuner-columns autotuner-settings-columns">
        <div class="autotuner-col autotuner-col-left">
          <div class="autotuner-field">
            <label class="autotuner-label">${escapeHtml(t("ai_tuning_provider_label", "AI 서비스 제공자 (Provider)"))}</label>
            <select class="autotuner-select" id="aiProviderSelect">
              ${providerOptions}
            </select>
          </div>

          <!-- Free Mode Explanation -->
          <div class="autotuner-notice-banner ${isFree ? "" : "hidden"}" id="freeModeNotice">
            <strong>💡 ${escapeHtml(t("ai_tuning_free_desc", "기본 무료 AI 분석 모드"))}</strong> (${escapeHtml(t("ai_tuning_free_mode_desc", "API 키 불필요"))})<br />
            <span style="color: rgba(255, 235, 59, 0.85); font-size: 11px;">
              ${escapeHtml(t("ai_tuning_free_note", "※ 사전 정의된 규칙 기반 경량 분석으로 제공됩니다. 대용량 로그 심층 추론은 Google Gemini, Claude, OpenAI 등의 개인 API Key를 등록하세요."))}
            </span>
          </div>

          <!-- Paid/Personal API Fields (Key) -->
          <div id="personalApiKeyWrap" class="${isFree ? "hidden" : ""}">
            <div class="autotuner-field">
              <label class="autotuner-label">${escapeHtml(t("ai_tuning_api_key_label", "API Key"))}</label>
              <input class="autotuner-input" type="password" id="aiApiKeyInput" placeholder="${escapeHtml(t("ai_tuning_api_key_placeholder", "API Key 입력 (AIzaSy... / sk-...)"))}" value="${escapeHtml(aiConfig.keys?.[aiConfig.provider] || (aiConfig.provider !== "free" ? aiConfig.apiKey : "") || "")}" autocomplete="off" />
            </div>
          </div>
        </div>

        <div class="autotuner-col autotuner-col-right">
          <!-- Paid/Personal API Fields (Model & Endpoint) -->
          <div id="personalApiModelWrap" class="${isFree ? "hidden" : ""}" style="display: flex; flex-direction: column; gap: 8px;">
            <div class="autotuner-field">
              <div style="display: flex; justify-content: space-between; align-items: center;">
                <label class="autotuner-label">${escapeHtml(t("ai_tuning_model_label", "제공 모델 (Model)"))}</label>
                <button class="autotuner-btn-secondary" type="button" id="btnFetchModels" style="padding: 2px 6px; font-size: 11px;">🔄 ${escapeHtml(t("ai_tuning_fetch_models_btn", "모델 목록 불러오기"))}</button>
              </div>
              <select class="autotuner-select" id="aiModelSelect">
                ${modelOptions}
              </select>
            </div>

            <div class="autotuner-field ${aiConfig.provider === "custom" ? "" : "hidden"}" id="customEndpointField">
              <label class="autotuner-label">${escapeHtml(t("ai_tuning_custom_endpoint", "Custom API Endpoint URL"))}</label>
              <input class="autotuner-input" type="text" id="aiEndpointInput" placeholder="http://localhost:11434/v1/chat/completions" value="${escapeHtml(aiConfig.customEndpoint || "")}" />
            </div>
          </div>

          <div style="display: flex; justify-content: space-between; align-items: center; margin-top: auto; padding-top: 8px;">
            <div id="aiConfigStatus" style="font-size: 11.5px;"></div>
            <button class="autotuner-btn-secondary" type="button" id="btnSaveAiConfig">💾 ${escapeHtml(t("ai_tuning_save_settings_btn", "설정 저장"))}</button>
          </div>
        </div>
      </div>
    </div>

    <!-- Tab 2: Analysis Pane -->
    <div class="autotuner-tab-pane ${activeTab === "analysis" ? "" : "hidden"}" id="paneAnalysis">
      <div class="autotuner-columns autotuner-analysis-columns">
        <!-- Left Column: Controls & Prompt Input -->
        <div class="autotuner-col autotuner-col-left">
          <div class="autotuner-badge-bar">
            <span>🛣️ <strong>${escapeHtml(route)}</strong></span>
            <span class="autotuner-chip" id="autotunerSegmentChip">${escapeHtml(segmentBadgeText)}</span>
          </div>

          <!-- Mode indicator banner -->
          <div class="autotuner-notice-banner" id="analysisModeBanner">
            ${
              isFree
                ? `💡 <strong>${escapeHtml(t("ai_tuning_free_mode_title", "기본 무료 분석 모드"))}</strong> (${escapeHtml(t("ai_tuning_free_mode_desc", "API 키 불필요, 규칙 기반 경량 분석"))})`
                : `✨ ${escapeHtml(t("ai_tuning_personal_ai_desc", `개인 연동 AI (${currentProvider.name} / ${aiConfig.model || "기본"}) 심층 분석`, { provider: currentProvider.name, model: aiConfig.model || "기본" }))}`
            }
          </div>

          <!-- Rollback Quick Bar -->
          <div class="autotuner-rollback-bar ${hasRollback ? "" : "hidden"}" id="autotunerRollbackBar">
            <span>↩️ ${escapeHtml(t("ai_tuning_rollback_banner", "직전 적용 파라미터 백업 저장됨"))}</span>
            <button class="autotuner-btn-undo" type="button" id="btnRollbackGlobal" style="padding: 3px 8px; font-size: 11.5px;">↩️ ${escapeHtml(t("ai_tuning_rollback_btn", "직전 설정 롤백"))}</button>
          </div>

          <!-- Driver Override Option (Gas / Brake / Steer Interventions) -->
          <div class="autotuner-override-box">
            <label class="autotuner-override-label">
              <input type="checkbox" id="chkIncludeDriverOverride" class="autotuner-override-checkbox" ${state?.includeDriverOverride ? "checked" : ""} />
              <span class="autotuner-override-title">🎮 ${escapeHtml(t("ai_tuning_include_driver_override_label", "운전자 수동조작(Gas/Brake/Steer) 반영"))}</span>
            </label>
            <div class="autotuner-override-desc">
              ${escapeHtml(t("ai_tuning_include_driver_override_desc", "운전자의 페달/핸들 조작을 자동주행 불만(급가속에 대한 브레이킹, 굼뜬 출발에 대한 가속, 쏠림에 대한 조향 등) 피드백으로 간주하여 보완 파라미터를 추천합니다. (기본: 제외)"))}
            </div>
          </div>

          <!-- Preset Issues -->
          <div class="autotuner-field">
            <label class="autotuner-label">${escapeHtml(t("ai_tuning_preset_label", "자주 발생하는 문제점 (빠른 선택)"))}</label>
            <select class="autotuner-select" id="presetIssueSelect">
              ${presetOptions}
            </select>
          </div>

          <!-- Custom Prompt with Speech Input -->
          <div class="autotuner-field">
            <label class="autotuner-label">${escapeHtml(t("ai_tuning_issue_label", "체감 의견 / 상세 프롬프트 (음성 또는 직접 입력)"))}</label>
            <div class="autotuner-voice-row">
              <textarea class="autotuner-textarea" id="userIssuePrompt" placeholder="${escapeHtml(t("ai_tuning_issue_placeholder", "예: '선행차 급출발 시 너무 급하게 따라붙어 불편해'"))}"></textarea>
              ${
                hasSpeechSupport
                  ? `<button class="autotuner-mic-btn" type="button" id="btnVoiceInput" title="${escapeHtml(t("ai_tuning_voice_btn", "음성으로 말하기"))}">
                      <svg viewBox="0 0 24 24" width="16" height="16"><path fill="currentColor" d="M12 14c1.66 0 3-1.34 3-3V5c0-1.66-1.34-3-3-3S9 3.34 9 5v6c0 1.66 1.34 3 3 3m5.91-3c-.49 0-.9.36-.98.85C16.52 14.2 14.47 16 12 16s-4.52-1.8-4.93-4.15c-.08-.49-.49-.85-.98-.85-.61 0-1.09.54-1 1.14.49 3 2.89 5.35 5.91 5.78V20c0 .55.45 1 1 1s1-.45 1-1v-2.08c3.02-.43 5.42-2.78 5.91-5.78.1-.6-.39-1.14-1-1.14"/></svg>
                      <span id="micLabel">${escapeHtml(t("ai_tuning_voice_btn", "음성"))}</span>
                    </button>`
                  : ""
              }
            </div>
          </div>

          <!-- Start Button -->
          <button class="autotuner-btn-primary" type="button" id="btnStartAnalysis">
            <span>🤖 ${escapeHtml(t("ai_tuning_start_analysis_btn", "주행로그 AI 분석 시작"))}</span>
          </button>
        </div>

        <!-- Right Column: Results & Recommendations (or Placeholder) -->
        <div class="autotuner-col autotuner-col-right" id="autotunerResultContainer">
          ${
            currentResult
              ? renderResultsHtml(currentResult, hasRollback)
              : renderPlaceholderHtml()
          }
        </div>
      </div>
    </div>
  </div>`;
}

export function renderPlaceholderHtml() {
  return `
  <div class="autotuner-placeholder-box">
    <div class="autotuner-placeholder-icon">🤖</div>
    <div class="autotuner-placeholder-title">${escapeHtml(t("ai_tuning_placeholder_title", "AI 주행 데이터 진단 대기 중"))}</div>
    <div class="autotuner-placeholder-desc">
      ${t("ai_tuning_placeholder_desc", `좌측에서 자주 발생하는 증상을 선택하거나 체감 의견을 입력한 후 <strong>[주행로그 AI 분석 시작]</strong>을 누르세요.<br /><br />AI가 주행 로그의 조향(Steer), 가속(Accel), 저크(Jerk), 선행차 추종(Lead Track) 패턴을 정밀 진단하여 최적의 파라미터를 추천합니다.`)}
    </div>
  </div>`;
}

export function renderLoadingHtml(statusText = "") {
  const displayStatus = statusText || t("ai_tuning_analyzing", "주행로그 분석 중…");
  return `
  <div class="autotuner-loading-box">
    <div class="autotuner-spinner"></div>
    <div style="font-weight: 600; color: #38ef7d; font-size: 13px;">${escapeHtml(displayStatus)}</div>
    <button class="autotuner-btn-secondary" type="button" id="btnCancelAnalysis" style="margin-top: 4px; padding: 4px 12px; font-size: 11.5px; border-color: rgba(255, 71, 87, 0.4); color: #ff6b81;">
      🛑 ${escapeHtml(t("ai_tuning_cancel_analysis_btn", "분석 취소"))}
    </button>
  </div>`;
}

export function renderResultsHtml(result, hasRollback = false) {
  const { summary, rootCause, recommendedParams, isFreeMode } = result;

  const rows = (recommendedParams || []).map((p) => {
    return `
    <tr>
      <td style="width: 28px; text-align: center;">
        <input type="checkbox" class="autotuner-param-check" data-name="${escapeHtml(p.name)}" data-value="${escapeHtml(p.recommended)}" checked />
      </td>
      <td style="font-weight: 600; color: #ffffff; white-space: nowrap; width: 140px;">${escapeHtml(p.name)}</td>
      <td style="width: 100px; white-space: nowrap;">
        <div class="autotuner-diff-val">
          <span class="autotuner-diff-old">${escapeHtml(p.current)}</span>
          <span class="autotuner-diff-arrow">→</span>
          <span class="autotuner-diff-new">${escapeHtml(p.recommended)}</span>
        </div>
      </td>
      <td class="autotuner-reason-text">${escapeHtml(p.reason)}</td>
    </tr>`;
  }).join("");

  return `
  <div class="autotuner-results-view">
    <!-- Diagnosis Card (Compact, no excessive whitespace) -->
    <div class="autotuner-summary-card">
      <div class="autotuner-summary-title">
        <svg viewBox="0 0 24 24" width="15" height="15"><path fill="currentColor" d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2m1 15h-2v-6h2zm0-8h-2V7h2z"/></svg>
        <span>${escapeHtml(t("ai_tuning_report_title", "AI 진단 리포트"))}</span>
        ${isFreeMode ? `<span style="font-size: 10.5px; font-weight: normal; color: #ffeb3b; margin-left: 2px;">${escapeHtml(t("ai_tuning_free_badge", "[기본 무료]"))}</span>` : ""}
      </div>
      <div class="autotuner-summary-text">
        <strong>${escapeHtml(t("ai_tuning_diag_prefix", "[진단]"))}</strong> ${escapeHtml(summary)}
      </div>
      ${rootCause ? `<div class="autotuner-summary-text" style="color: #a0aec0;"><strong>${escapeHtml(t("ai_tuning_cause_prefix", "[원인]"))}</strong> ${escapeHtml(rootCause)}</div>` : ""}
    </div>

    <!-- Parameter Table (Sticky pinned header, scrollable body) -->
    <div class="autotuner-table-section">
      <div class="autotuner-table-header-row">
        <label class="autotuner-label">🎯 ${escapeHtml(t("ai_tuning_recommended_params", "추천 파라미터"))}</label>
        <button class="autotuner-btn-secondary" type="button" id="btnToggleAllParams" style="padding: 2px 6px; font-size: 11px;">${escapeHtml(t("ai_tuning_toggle_all", "전체 선택/해제"))}</button>
      </div>
      <div class="autotuner-table-wrap">
        <table class="autotuner-table">
          <thead>
            <tr>
              <th style="width: 28px; text-align: center;"></th>
              <th style="width: 140px;">${escapeHtml(t("ai_tuning_th_param", "파라미터"))}</th>
              <th style="width: 100px;">${escapeHtml(t("ai_tuning_th_change", "변경치"))}</th>
              <th>${escapeHtml(t("ai_tuning_th_reason", "조정 사유"))}</th>
            </tr>
          </thead>
          <tbody>
            ${rows || `<tr><td colspan="4" style="text-align: center; color: #a0aec0; padding: 12px;">${escapeHtml(t("ai_tuning_no_recommendations", "추천 파라미터 변경 사항이 없습니다."))}</td></tr>`}
          </tbody>
        </table>
      </div>
    </div>

    <!-- Footer Action Buttons -->
    <div class="autotuner-footer">
      <div>
        ${hasRollback ? `<button class="autotuner-btn-undo" type="button" id="btnRollbackParams">↩️ ${escapeHtml(t("ai_tuning_rollback_btn", "직전 설정 롤백"))}</button>` : ""}
      </div>
      <div style="display: flex; gap: 6px;">
        <button class="autotuner-btn-primary" type="button" id="btnApplySelectedParams" ${(!recommendedParams || !recommendedParams.length) ? "disabled" : ""}>
          ✨ ${escapeHtml(t("ai_tuning_apply_selected", "선택 파라미터 적용"))}
        </button>
      </div>
    </div>
  </div>`;
}

export function renderRollbackModalHtml(rollbackParams = {}, currentParams = {}) {
  const rows = Object.entries(rollbackParams).map(([k, rollbackVal]) => {
    const curVal = currentParams?.[k];
    const hasCurrent = curVal !== undefined && curVal !== null && curVal !== "";
    return `
    <div class="autotuner-modal-tr-row">
      <div class="autotuner-modal-td-check">
        <input type="checkbox" class="autotuner-rollback-check autotuner-checkbox" data-name="${escapeHtml(k)}" data-value="${escapeHtml(String(rollbackVal))}" checked />
      </div>
      <div class="autotuner-modal-td-param" title="${escapeHtml(k)}">${escapeHtml(k)}</div>
      <div class="autotuner-modal-td-diff">
        <div class="autotuner-diff-val">
          ${
            hasCurrent
              ? `<span class="autotuner-diff-old">${escapeHtml(String(curVal))}</span>
                 <span class="autotuner-diff-arrow">→</span>`
              : `<span class="autotuner-diff-old" style="text-decoration:none; opacity:0.65;">${escapeHtml(t("ai_tuning_current_setting_badge", "(현재)"))}</span>
                 <span class="autotuner-diff-arrow">→</span>`
          }
          <span class="autotuner-diff-rollback">${escapeHtml(String(rollbackVal))}</span>
        </div>
      </div>
    </div>`;
  }).join("");

  const count = Object.keys(rollbackParams).length;

  return `
  <div class="autotuner-modal-overlay" id="autotunerRollbackModalOverlay">
    <div class="autotuner-modal-card">
      <div class="autotuner-modal-header">
        <div class="autotuner-modal-title">
          <svg viewBox="0 0 24 24" width="16" height="16"><path fill="currentColor" d="M12.5 8c-2.65 0-5.05.99-6.9 2.6L2 7v9h9l-3.62-3.62c1.39-1.16 3.16-1.88 5.12-1.88 3.54 0 6.55 2.31 7.6 5.5l2.37-.78C21.08 11.03 17.15 8 12.5 8"/></svg>
          <span>${escapeHtml(t("ai_tuning_rollback_title", "직전 파라미터 설정 롤백 확인"))}</span>
        </div>
        <button type="button" class="autotuner-modal-close-btn" id="btnCloseRollbackModal" aria-label="${escapeHtml(t("close", "닫기"))}">✕</button>
      </div>

      <div class="autotuner-modal-desc">
        ${t("ai_tuning_rollback_desc", `직전에 적용했던 백업 파라미터(총 <strong>{count}개</strong>) 중 되돌릴 항목을 선택해주세요:`, { count })}
      </div>

      <div class="autotuner-modal-table-box">
        <div class="autotuner-modal-th-row">
          <div class="autotuner-modal-th-check">
            <input type="checkbox" id="btnToggleAllRollbackParams" class="autotuner-checkbox" checked title="${escapeHtml(t("ai_tuning_toggle_all", "전체 선택/해제"))}" />
          </div>
          <div class="autotuner-modal-th-param">${escapeHtml(t("ai_tuning_th_param", "파라미터"))}</div>
          <div class="autotuner-modal-th-diff">${escapeHtml(t("ai_tuning_th_current_restore", "현재값 → 복원값"))}</div>
        </div>
        <div class="autotuner-modal-tb-scroll">
          ${rows || `<div class="autotuner-modal-empty-row">${escapeHtml(t("ai_tuning_no_recommendations", "롤백할 파라미터가 없습니다."))}</div>`}
        </div>
      </div>

      <div class="autotuner-modal-actions">
        <button type="button" class="autotuner-btn-secondary" id="btnCancelRollbackModal">${escapeHtml(t("cancel", "취소"))}</button>
        <button type="button" class="autotuner-btn-undo" id="btnConfirmRollbackModal" style="padding: 5px 14px; font-weight: 700;">↩️ ${escapeHtml(t("ai_tuning_rollback_execute", `선택 파라미터 롤백 (${count}개)`, { count }))}</button>
      </div>
    </div>
  </div>`;
}

