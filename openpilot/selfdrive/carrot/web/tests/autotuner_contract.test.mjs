import test from "node:test";
import assert from "node:assert/strict";

import { AI_PROVIDERS, buildSystemPrompt, buildUserPrompt, callAiTuner, getActiveLanguage, loadAiConfig, saveAiConfig } from "../src/features/logs/autotuner/ai_client.js";
import { getInitialLayoutMode } from "../src/features/logs/autotuner/controller.js";
import { PRESET_ISSUES, renderAutoTunerHtml, renderPlaceholderHtml, renderResultsHtml, renderRollbackModalHtml } from "../src/features/logs/autotuner/view.js";

test("AI_PROVIDERS contains Free default mode, Gemini, OpenAI, Claude, Grok, and Custom", () => {
  assert.ok(AI_PROVIDERS.free);
  assert.ok(AI_PROVIDERS.free.isFree);
  assert.ok(AI_PROVIDERS.gemini);
  assert.ok(AI_PROVIDERS.openai);
  assert.ok(AI_PROVIDERS.claude);
  assert.ok(AI_PROVIDERS.grok);
  assert.ok(AI_PROVIDERS.custom);

  const defaultConfig = loadAiConfig();
  assert.equal(defaultConfig.provider, "free");
});

test("Default Free mode callAiTuner executes without API key and returns structured result", async () => {
  const telemetryData = {
    route: "000000a0--0d93fa7497",
    segmentCount: 3,
    stats: {
      maxTargetAccel: 1.95,
      minAccel: -1.2,
      avgSpeed: 45.0,
    },
    episodes: [
      { type: "harsh_catchup_accel", t: 1.5, aTarget: 1.95, aCmd: 2.1 },
    ],
    currentParams: {
      CruiseMaxVals1: "200",
      DynamicTFollow: "30",
    },
  };

  const freeResult = await callAiTuner(telemetryData, "선행차 캐치업 급가속", { provider: "free" });
  assert.ok(freeResult.isFreeMode);
  assert.ok(freeResult.summary);
  assert.ok(freeResult.rootCause);
  assert.ok(Array.isArray(freeResult.recommendedParams));
  assert.ok(freeResult.recommendedParams.length > 0);

  const paramNames = freeResult.recommendedParams.map((p) => p.name);
  assert.ok(paramNames.includes("CruiseMaxVals1"));
});

test("PRESET_ISSUES covers requested presets", () => {
  assert.ok(PRESET_ISSUES.length >= 6);
  const labels = PRESET_ISSUES.map((p) => p.label).join(" ");
  assert.match(labels, /종합 주행 상태 분석/);
  assert.match(labels, /선행차 캐치업 급가속/);
  assert.match(labels, /선행차 출발시 가속 느림/);
  assert.match(labels, /정지차 인식 급브레이크/);
  assert.match(labels, /직선로 쏠림/);
  assert.match(labels, /곡선로 쏠림/);
});

test("renderAutoTunerHtml renders tabs, mode switcher, and responsive 2-column layout", () => {
  const landscapeHtml = renderAutoTunerHtml({
    route: "000000a0--0d93fa7497",
    segmentCount: 5,
    isAllSegments: true,
    aiConfig: { provider: "free", model: "built-in", apiKey: "" },
    activeTab: "analysis",
    layoutMode: "landscape",
    hasSpeechSupport: true,
    hasRollback: false,
  });

  assert.match(landscapeHtml, /000000a0--0d93fa7497/);
  assert.match(landscapeHtml, /전체 5개 세그먼트/);
  assert.doesNotMatch(landscapeHtml, /전체개 세그먼트/);
  assert.match(landscapeHtml, /기본 무료 분석 모드/);
  assert.match(landscapeHtml, /주행 로그 분석/);
  assert.match(landscapeHtml, /AI 모델 설정/);
  assert.match(landscapeHtml, /btnStartAnalysis/);
  assert.match(landscapeHtml, /btnToggleLayout/);
  assert.match(landscapeHtml, /autotuner-mode-landscape/);
  assert.match(landscapeHtml, /가로 모드/);

  const portraitHtml = renderAutoTunerHtml({
    route: "000000a0--0d93fa7497",
    segmentCount: 2,
    isAllSegments: false,
    aiConfig: { provider: "free", model: "built-in", apiKey: "" },
    activeTab: "analysis",
    layoutMode: "portrait",
    hasSpeechSupport: true,
    hasRollback: false,
  });

  assert.match(portraitHtml, /2개 세그먼트/);
  assert.match(portraitHtml, /autotuner-mode-portrait/);
  assert.match(portraitHtml, /세로 모드/);

  const unselectedHtml = renderAutoTunerHtml({
    route: "000000a0--0d93fa7497",
    segmentCount: 0,
    isAllSegments: true,
    aiConfig: { provider: "free", model: "built-in", apiKey: "" },
    activeTab: "analysis",
  });
  assert.match(unselectedHtml, /전체 세그먼트/);
  assert.doesNotMatch(unselectedHtml, /전체개 세그먼트/);
});

test("renderPlaceholderHtml renders guide description for initial state", () => {
  const html = renderPlaceholderHtml();
  assert.match(html, /AI 주행 데이터 진단 대기 중/);
  assert.match(html, /주행로그 AI 분석 시작/);
});

test("renderResultsHtml renders diagnosis summary, parameter table, and rollback when active", () => {
  const mockResult = {
    isFreeMode: true,
    summary: "선행차 급출발 시 CruiseMaxVals1과 DynamicTFollow로 인한 급가속 현상 진단",
    rootCause: "DynamicTFollow 30으로 인해 목표거리가 급단축되고 가속도 페널티가 완화되어 피크 가속도 1.98 m/s^2 발생",
    recommendedParams: [
      {
        name: "CruiseMaxVals1",
        current: "200",
        recommended: "145",
        reason: "10km/h 가속 상한을 1.45 m/s^2로 제한하여 급출발 충격 완화",
      },
      {
        name: "DynamicTFollow",
        current: "30",
        recommended: "12",
        reason: "선행차 가속 시 차간거리 목표 급변동 억제",
      },
    ],
  };

  const html = renderResultsHtml(mockResult, true);
  assert.match(html, /CruiseMaxVals1/);
  assert.match(html, /200/);
  assert.match(html, /145/);
  assert.match(html, /DynamicTFollow/);
  assert.match(html, /btnApplySelectedParams/);
  assert.match(html, /btnRollbackParams/);
  assert.match(html, /롤백/);
  assert.match(html, /기본 무료/);
});

test("getInitialLayoutMode defaults safely in non-browser / node environment", () => {
  const mode = getInitialLayoutMode();
  assert.ok(mode === "landscape" || mode === "portrait");
});

test("renderRollbackModalHtml displays all parameters and their restoration values in table", () => {
  const mockRollbackParams = {
    LateralTorqueKpV: 120,
    SteerActuatorDelay: 30,
    LatSmoothSec: 13,
  };
  const mockCurrentParams = {
    LateralTorqueKpV: 135,
    SteerActuatorDelay: 25,
    LatSmoothSec: 8,
  };
  const html = renderRollbackModalHtml(mockRollbackParams, mockCurrentParams);
  assert.match(html, /직전 파라미터 설정 롤백 확인/);
  assert.match(html, /3개/);
  assert.match(html, /LateralTorqueKpV/);
  assert.match(html, /135/);
  assert.match(html, /120/);
  assert.match(html, /SteerActuatorDelay/);
  assert.match(html, /25/);
  assert.match(html, /30/);
  assert.match(html, /현재값 → 복원값/);
  assert.match(html, /btnConfirmRollbackModal/);
  assert.match(html, /btnCancelRollbackModal/);
  assert.match(html, /btnToggleAllRollbackParams/);
  assert.match(html, /autotuner-rollback-check/);
});

test("renderAutoTunerHtml, renderResultsHtml and renderRollbackModalHtml support English and Chinese i18n", () => {
  // Mock English getUIText
  const enStrings = {
    ai_tuning_tab_diagnose: "Driving Log Analysis",
    ai_tuning_tab_settings: "AI Model Settings",
    ai_tuning_start_analysis_btn: "Start AI Log Analysis",
    ai_tuning_report_title: "AI Diagnosis Report",
    ai_tuning_rollback_title: "Rollback Previous Parameters",
    ai_tuning_all_segments: "All {count} segments",
  };

  globalThis.getUIText = (key, fallback, vars) => {
    let s = enStrings[key] || fallback;
    if (vars && typeof vars === "object") {
      for (const [k, v] of Object.entries(vars)) {
        s = s.replace(new RegExp(`\\{${k}\\}`, "g"), String(v));
      }
    }
    return s;
  };

  const enHtml = renderAutoTunerHtml({
    route: "000000a0--0d93fa7497",
    segmentCount: 4,
    isAllSegments: true,
    aiConfig: { provider: "free", model: "built-in" },
    activeTab: "analysis",
  });

  assert.match(enHtml, /Driving Log Analysis/);
  assert.match(enHtml, /AI Model Settings/);
  assert.match(enHtml, /Start AI Log Analysis/);
  assert.match(enHtml, /All 4 segments/);

  // Mock Chinese getUIText
  const zhStrings = {
    ai_tuning_tab_diagnose: "行车日志分析",
    ai_tuning_tab_settings: "AI 模型设置",
    ai_tuning_start_analysis_btn: "开始 AI 日志分析",
    ai_tuning_report_title: "AI 诊断报告",
    ai_tuning_rollback_title: "确认回滚上一次参数设置",
    ai_tuning_all_segments: "全部 {count} 个分段",
  };

  globalThis.getUIText = (key, fallback, vars) => {
    let s = zhStrings[key] || fallback;
    if (vars && typeof vars === "object") {
      for (const [k, v] of Object.entries(vars)) {
        s = s.replace(new RegExp(`\\{${k}\\}`, "g"), String(v));
      }
    }
    return s;
  };

  const zhHtml = renderAutoTunerHtml({
    route: "000000a0--0d93fa7497",
    segmentCount: 6,
    isAllSegments: true,
    aiConfig: { provider: "free", model: "built-in" },
    activeTab: "analysis",
  });

  assert.match(zhHtml, /行车日志分析/);
  assert.match(zhHtml, /AI 模型设置/);
  assert.match(zhHtml, /开始 AI 日志分析/);
  assert.match(zhHtml, /全部 6 个分段/);

  const zhModalHtml = renderRollbackModalHtml({ TestParam: 10 }, { TestParam: 20 });
  assert.match(zhModalHtml, /确认回滚上一次参数设置/);

  delete globalThis.getUIText;
});

test("buildSystemPrompt and buildUserPrompt support Korean, English, and Chinese with driver overrides", () => {
  const promptKo = buildSystemPrompt("ko");
  assert.match(promptKo, /Korean/);
  const promptEn = buildSystemPrompt("en");
  assert.match(promptEn, /English/);
  const promptZh = buildSystemPrompt("zh");
  assert.match(promptZh, /Simplified Chinese/);

  const mockTelemetry = {
    route: "test-route",
    segmentCount: 2,
    stats: {
      driverOverrides: { gas: 1, brake: 2, steer: 1, total: 4, included: true }
    },
    episodes: [],
    currentParams: {},
  };

  // With driver overrides included
  const userPromptEn = buildUserPrompt(mockTelemetry, "", { lang: "en", includeDriverOverride: true });
  assert.match(userPromptEn, /DRIVER MANUAL OVERRIDE INTERVENTIONS/);
  assert.match(userPromptEn, /Gas: 1, Brake: 2, Steer: 1, Total: 4/);
  assert.match(userPromptEn, /strictly in English/);

  const userPromptZh = buildUserPrompt(mockTelemetry, "", { lang: "zh", includeDriverOverride: true });
  assert.match(userPromptZh, /DRIVER MANUAL OVERRIDE INTERVENTIONS/);
  assert.match(userPromptZh, /strictly in Simplified Chinese/);

  // With driver overrides excluded
  const userPromptExcluded = buildUserPrompt(mockTelemetry, "", { lang: "en", includeDriverOverride: false });
  assert.doesNotMatch(userPromptExcluded, /DRIVER MANUAL OVERRIDE INTERVENTIONS/);
});

test("Free heuristic mode generates localized outputs in English and Chinese with driver override feedback", async () => {
  const mockTelemetry = {
    route: "test-route",
    segmentCount: 3,
    stats: {
      maxTargetAccel: 1.8,
      minAccel: -1.5,
      driverOverrides: { gas: 0, brake: 2, steer: 0, total: 2, included: true }
    },
    episodes: [],
    currentParams: { CruiseMaxVals1: "200", DynamicTFollow: "30" },
  };

  // English Free Mode with Brake Overrides
  const enResult = await callAiTuner(mockTelemetry, "", { provider: "free" }, null, {
    lang: "en",
    includeDriverOverride: true,
  });
  assert.match(enResult.summary, /Cruise acceleration ceiling/);
  assert.match(enResult.rootCause, /driver manual brake interventions/);
  assert.match(enResult.recommendedParams[0].reason, /Lowers 10 km\/h acceleration ceiling/);

  // Chinese Free Mode with Brake Overrides
  const zhResult = await callAiTuner(mockTelemetry, "", { provider: "free" }, null, {
    lang: "zh",
    includeDriverOverride: true,
  });
  assert.match(zhResult.summary, /调低巡航加速度上限/);
  assert.match(zhResult.rootCause, /手动刹车接管/);
  assert.match(zhResult.recommendedParams[0].reason, /避免跟车起步时突兀窜车/);
});

test("renderAutoTunerHtml renders driver override option card and respects checked state", () => {
  const htmlUnchecked = renderAutoTunerHtml({
    route: "test-route",
    includeDriverOverride: false,
  });
  assert.match(htmlUnchecked, /chkIncludeDriverOverride/);
  assert.match(htmlUnchecked, /autotuner-override-box/);
  assert.match(htmlUnchecked, /운전자 수동조작\(Gas\/Brake\/Steer\) 반영/);
  assert.doesNotMatch(htmlUnchecked, /id="chkIncludeDriverOverride"[^>]*checked/);

  const htmlChecked = renderAutoTunerHtml({
    route: "test-route",
    includeDriverOverride: true,
  });
  assert.match(htmlChecked, /id="chkIncludeDriverOverride"[^>]*checked/);
});

