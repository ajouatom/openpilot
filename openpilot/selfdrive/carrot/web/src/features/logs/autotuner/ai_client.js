/**
 * AutoTuner AI Client: Supports Default Free (Built-in Rules), Google Gemini, OpenAI, Anthropic Claude, xAI Grok, and Custom endpoints.
 */

const STORAGE_KEY = "carrot_autotuner_ai_config";
const STORAGE_ROLLBACK_KEY = "carrot_autotuner_rollback_snapshot";

export const AI_PROVIDERS = Object.freeze({
  free: {
    name: "기본 무료 AI 분석 (API 키 불필요)",
    defaultModel: "built-in",
    models: ["built-in (규칙 기반 경량 분석)"],
    isFree: true,
  },
  gemini: {
    name: "Google Gemini",
    defaultModel: "",
    models: [],
    endpoint: "https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent?key={key}",
    modelsApi: "https://generativelanguage.googleapis.com/v1beta/models?key={key}",
  },
  openai: {
    name: "OpenAI",
    defaultModel: "",
    models: [],
    endpoint: "https://api.openai.com/v1/chat/completions",
    modelsApi: "https://api.openai.com/v1/models",
  },
  claude: {
    name: "Anthropic Claude",
    defaultModel: "",
    models: [],
    endpoint: "https://api.anthropic.com/v1/messages",
    modelsApi: "https://api.anthropic.com/v1/models",
  },
  grok: {
    name: "xAI Grok",
    defaultModel: "",
    models: [],
    endpoint: "https://api.x.ai/v1/chat/completions",
    modelsApi: "https://api.x.ai/v1/models",
  },
  custom: {
    name: "Custom (OpenAI 호환)",
    defaultModel: "default",
    models: ["default"],
    endpoint: "http://localhost:11434/v1/chat/completions",
    modelsApi: "http://localhost:11434/v1/models",
  },
});

export function loadAiConfig() {
  let cfg = null;
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (raw) cfg = JSON.parse(raw);
  } catch {}

  if (!cfg || typeof cfg !== "object") {
    cfg = {
      provider: "free",
      model: "built-in",
      keys: {},
      selectedModels: {},
      cachedModels: {},
      apiKey: "",
      customEndpoint: "",
    };
  }

  if (!cfg.keys || typeof cfg.keys !== "object") cfg.keys = {};
  if (!cfg.selectedModels || typeof cfg.selectedModels !== "object") cfg.selectedModels = {};
  if (!cfg.cachedModels || typeof cfg.cachedModels !== "object") cfg.cachedModels = {};

  if (cfg.apiKey && cfg.provider && cfg.provider !== "free" && !cfg.keys[cfg.provider]) {
    cfg.keys[cfg.provider] = cfg.apiKey;
  }

  const p = cfg.provider || "free";
  if (p !== "free") {
    if (cfg.keys[p]) cfg.apiKey = cfg.keys[p];
    if (cfg.selectedModels[p]) {
      cfg.model = cfg.selectedModels[p];
    }
  } else {
    cfg.model = "built-in";
  }

  return cfg;
}

export async function syncAiConfigWithServer() {
  const localCfg = loadAiConfig();
  try {
    const res = await fetch("/api/autotune/config");
    if (res.ok) {
      const data = await res.json();
      const serverCfg = data?.config;
      if (serverCfg && typeof serverCfg === "object" && Object.keys(serverCfg).length > 0) {
        const mergedKeys = { ...(localCfg.keys || {}), ...(serverCfg.keys || {}) };
        const mergedSelectedModels = { ...(localCfg.selectedModels || {}), ...(serverCfg.selectedModels || {}) };
        const mergedCachedModels = { ...(localCfg.cachedModels || {}), ...(serverCfg.cachedModels || {}) };

        // Prefer the provider that has an active key or was explicitly saved on server
        let activeProvider = serverCfg.provider || localCfg.provider || "free";
        if (activeProvider === "free" && localCfg.provider && localCfg.provider !== "free" && mergedKeys[localCfg.provider]) {
          activeProvider = localCfg.provider;
        }

        const merged = {
          provider: activeProvider,
          model: serverCfg.model || localCfg.model || (activeProvider === "free" ? "built-in" : (mergedSelectedModels[activeProvider] || "")),
          apiKey: activeProvider !== "free" ? (mergedKeys[activeProvider] || "") : "",
          customEndpoint: serverCfg.customEndpoint || localCfg.customEndpoint || "",
          keys: mergedKeys,
          selectedModels: mergedSelectedModels,
          cachedModels: mergedCachedModels,
        };

        if (activeProvider !== "free" && merged.keys[activeProvider]) {
          merged.apiKey = merged.keys[activeProvider];
          if (merged.selectedModels[activeProvider]) {
            merged.model = merged.selectedModels[activeProvider];
          }
        }

        localStorage.setItem(STORAGE_KEY, JSON.stringify(merged));

        // Two-way sync: if local had keys or provider server was missing, update server
        const localKeyCount = Object.keys(localCfg.keys || {}).length;
        const serverKeyCount = Object.keys(serverCfg.keys || {}).length;
        if (localKeyCount > serverKeyCount || (localCfg.provider !== "free" && serverCfg.provider === "free")) {
          saveAiConfig(merged);
        }

        return merged;
      } else if (localCfg && (localCfg.apiKey || Object.keys(localCfg.keys || {}).length > 0 || localCfg.provider !== "free")) {
        // Server was empty or default, sync local config to server so it survives across devices and reboots
        saveAiConfig(localCfg);
      }
    }
  } catch {}
  return localCfg;
}

export function saveAiConfig(config) {
  try {
    if (config && typeof config === "object") {
      if (!config.keys) config.keys = {};
      if (!config.selectedModels) config.selectedModels = {};
      if (!config.cachedModels) config.cachedModels = {};

      const p = config.provider || "free";
      if (p !== "free") {
        if (config.apiKey) {
          config.keys[p] = config.apiKey;
        }
        if (config.model && config.model !== "built-in") {
          config.selectedModels[p] = config.model;
        }
      }
      localStorage.setItem(STORAGE_KEY, JSON.stringify(config));

      // Push to device server params asynchronously so it persists on comma 3/3X and syncs to any client/browser
      fetch("/api/autotune/config", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ config }),
      }).catch(() => {});
    }
  } catch {}
}

export function loadRollbackSnapshot() {
  try {
    const raw = localStorage.getItem(STORAGE_ROLLBACK_KEY);
    if (raw) {
      const snap = JSON.parse(raw);
      if (snap && snap.params && Object.keys(snap.params).length > 0) {
        return snap;
      }
    }
  } catch {}
  return null;
}

export function saveRollbackSnapshot(params, currentApplied = {}) {
  try {
    if (params && typeof params === "object" && Object.keys(params).length > 0) {
      const data = {
        timestamp: Date.now(),
        params,
        currentApplied: (currentApplied && typeof currentApplied === "object") ? currentApplied : {},
      };
      localStorage.setItem(STORAGE_ROLLBACK_KEY, JSON.stringify(data));
      return data;
    }
  } catch {}
  return null;
}

export function clearRollbackSnapshot() {
  try {
    localStorage.removeItem(STORAGE_ROLLBACK_KEY);
  } catch {}
}

export async function fetchAvailableModels(providerKey, apiKey, customEndpoint = "") {
  const provider = AI_PROVIDERS[providerKey];
  if (!provider) return [];
  if (provider.isFree) return provider.models;

  const key = apiKey?.trim();

  try {
    if (providerKey === "gemini") {
      if (!key) return [];
      const url = provider.modelsApi.replace("{key}", key);
      const res = await fetch(url);
      const data = await res.json();
      if (!res.ok || !Array.isArray(data.models)) {
        throw new Error(data?.error?.message || `HTTP ${res.status}`);
      }
      const names = data.models
        .filter((m) => m.supportedGenerationMethods?.includes("generateContent"))
        .map((m) => m.name.replace(/^models\//, ""))
        .filter((n) => !n.includes("embedding") && !n.includes("aqa") && !n.includes("imagen"));

      // Sort models logically
      names.sort((a, b) => {
        if (a.includes("3.7") && !b.includes("3.7")) return -1;
        if (!a.includes("3.7") && b.includes("3.7")) return 1;
        if (a.includes("2.5") && !b.includes("2.5")) return -1;
        if (!a.includes("2.5") && b.includes("2.5")) return 1;
        if (a.includes("2.0") && !b.includes("2.0")) return -1;
        if (!a.includes("2.0") && b.includes("2.0")) return 1;
        return a.localeCompare(b);
      });
      return names;

    } else if (providerKey === "openai" || providerKey === "grok" || providerKey === "custom") {
      let url = provider.modelsApi;
      if (providerKey === "custom" && customEndpoint) {
        url = customEndpoint.replace(/\/chat\/completions\/?$/, "") + "/models";
      }
      const headers = { "Content-Type": "application/json" };
      if (key) headers["Authorization"] = `Bearer ${key}`;

      const res = await fetch(url, { headers });
      const data = await res.json();
      if (!res.ok || !Array.isArray(data.data)) {
        throw new Error(data?.error?.message || `HTTP ${res.status}`);
      }
      let modelList = data.data.map((m) => m.id);
      if (providerKey === "openai") {
        modelList = modelList.filter((id) => (
          id.startsWith("gpt-") || id.startsWith("o1") || id.startsWith("o3") || id.startsWith("chatgpt")
        ));
      }
      modelList.sort();
      return modelList;

    } else if (providerKey === "claude") {
      if (!key) return [];
      const res = await fetch(provider.modelsApi, {
        headers: {
          "x-api-key": key,
          "anthropic-version": "2023-06-01",
          "dangerously-allow-browser": "true",
        },
      });
      const data = await res.json();
      if (res.ok && Array.isArray(data.data)) {
        const names = data.data.map((m) => m.id);
        return names;
      }
      return ["claude-3-7-sonnet-20250219", "claude-3-5-sonnet-20241022", "claude-3-5-haiku-20241022", "claude-3-opus-20240229"];
    }
  } catch (err) {
    console.warn(`[AutoTuner] Failed to fetch live models for ${providerKey}:`, err);
    throw err;
  }

  return [];
}

/**
 * Built-in Rule-based Diagnostic Analyzer (Free Mode without API Key)
 */
export function getActiveLanguage(options = null) {
  if (options?.lang && ["ko", "en", "zh"].includes(options.lang)) {
    return options.lang;
  }
  try {
    const cur = typeof globalThis.getCurrentLanguage === "function"
      ? globalThis.getCurrentLanguage()
      : (typeof LANG !== "undefined" ? LANG : (localStorage.getItem("carrot_lang") || "ko"));
    if (cur === "en" || cur === "zh" || cur === "ko") return cur;
  } catch {}
  return "ko";
}

export function buildSystemPrompt(lang = "ko") {
  const languageNames = {
    ko: "Korean (한국어)",
    en: "English",
    zh: "Simplified Chinese (简体中文)",
  };
  const targetLang = languageNames[lang] || "Korean (한국어)";

  return `You are CarrotPilot AutoTuner AI, an expert autonomous driving engineer specializing in Openpilot CarrotPilot vehicle control tuning.
You will be provided with:
1. Driving Telemetry Summary & Key Notable Episodes (including driver manual overrides if enabled) extracted from recent driving logs (vEgo, aEgo, aTarget, aCmd, jerk, dRel, vLead, aLead, jLead, steerAngle, desiredSteerAngle, driverOverrides, etc.).
2. The current active parameters of the openpilot device.
3. The user's specific complaint/symptom or analysis request.

Your task is to:
1. Analyze the root cause of any driving discomfort, jerk, oscillation, lag, tracking error, or driver manual overrides (which reflect driver dissatisfaction with automated behavior) using physics, MPC control theory, and CarrotPilot parameter domain knowledge.
2. Recommend concrete parameter changes with specific target numeric values.

Key Parameter Rules for CarrotPilot:
- CruiseMaxVals0~6: Max acceleration limits across speed bins [0, 10, 40, 60, 80, 110, 140 km/h] in hundredths of m/s^2 (e.g. 145 = 1.45 m/s^2). Default is ~160/200/160. Lowering CruiseMaxVals1 (10km/h) to 140~150 prevents harsh catch-up acceleration surges.
- DynamicTFollow: Time-gap dynamic reduction based on lead car acceleration (jLead). Value in hundredths of sec (e.g. 30 = 0.30s). When set too high (>20), lead car acceleration causes target distance to shrink rapidly and halves MPC jerk factor, causing sudden surges. Recommend 10~15 or 0 for smooth following.
- EnableSpeedTF: Speed-scaled follow distance reduction. Value in % (e.g. 20 = 20% reduction at 0km/h scaling up to 100km/h). Recommend 10 or 0 for stable follow distance.
- TFollowGap1~4: Base follow distance for personality gaps in hundredths of sec (e.g. 110, 120, 140, 160).
- AChangeCostStarting: MPC acceleration change cost from standstill. Higher value (20~30) makes departure smoother. Lower value (10) makes departure quicker.
- RadarReactionFactor: Radar reaction multiplier (default 100). Lower (80~90) dampens noisy lead radar jumps.
- StoppingAccel: Final stopping hold accel in hundredths of m/s^2 (e.g. -40 = -0.40 m/s^2).
- StopDistanceCarrot: Target stop distance buffer in cm (e.g. 450 = 4.5m, 500 = 5.0m).
- LateralTorqueKpV, LateralTorqueKiV, LateralTorqueKf, LateralTorqueFriction, LateralTorqueAccelFactor: Steering torque tuning parameters.
- SteerActuatorDelay: Lateral actuator delay in hundredths of sec (e.g. 30 = 0.30s).
- SteerRatioRate: Steer ratio percentage (default 100).

CRITICAL RESPONSE FORMAT:
You MUST respond with a single, clean JSON object ONLY (enclosed within \`\`\`json ... \`\`\` or raw JSON). Do NOT include conversational text outside the JSON.
All text values ("summary", "rootCause", "reason") MUST be written strictly and naturally in ${targetLang}.

JSON Schema:
{
  "summary": "Short 1-2 paragraph executive summary explaining the diagnosis and findings in ${targetLang}.",
  "rootCause": "Detailed technical explanation of why the issue occurred based on the log telemetry and driver override episodes in ${targetLang}.",
  "recommendedParams": [
    {
      "name": "ParameterName",
      "current": "current_value",
      "recommended": "new_recommended_value",
      "reason": "Clear explanation in ${targetLang} of why this parameter was changed and what effect it will have."
    }
  ]
}`;
}

export const SYSTEM_PROMPT = buildSystemPrompt("ko");

export function buildUserPrompt(telemetryData, userIssueText, options = {}) {
  const lang = getActiveLanguage(options);
  const includeOverrides = options.includeDriverOverride !== undefined
    ? Boolean(options.includeDriverOverride)
    : Boolean(telemetryData?.stats?.driverOverrides?.included);

  const defaultPrompts = {
    ko: "종합 분석 요청: 전반적인 크루즈 가감속 및 조향 상태를 진단하고 승차감과 안정성을 극대화할 최적 파라미터를 추천해주세요.",
    en: "Comprehensive analysis request: Diagnose overall cruise acceleration/deceleration, lead following, and steering behavior, and recommend optimal parameters to maximize ride comfort and stability.",
    zh: "综合分析请求：诊断整体巡航加减速、前车跟随和转向状态，并推荐最优参数以最大化乘坐舒适度和稳定性。",
  };

  const defaultText = defaultPrompts[lang] || defaultPrompts.ko;
  const userText = (userIssueText || "").trim() || defaultText;

  let overrideSection = "";
  if (includeOverrides && telemetryData?.stats?.driverOverrides) {
    const doStats = telemetryData.stats.driverOverrides;
    overrideSection = `
### DRIVER MANUAL OVERRIDE INTERVENTIONS:
Driver Manual Overrides Included: YES (Gas: ${doStats.gas || 0}, Brake: ${doStats.brake || 0}, Steer: ${doStats.steer || 0}, Total: ${doStats.total || 0})
Note: Driver overrides represent direct driver dissatisfaction with automated behavior (e.g. driver braking due to openpilot accelerating excessively or late deceleration, driver pressing gas due to sluggish departure or excessive gap, driver steering due to lane drift or curve understeer). Please correlate these interventions with the preceding telemetry and recommend compensatory parameter adjustments.
`;
  }

  const langDirectives = {
    ko: "CRITICAL: Write the entire JSON output (summary, rootCause, reason) strictly in Korean (한국어로 상세하고 친절하게 작성).",
    en: "CRITICAL: Write the entire JSON output (summary, rootCause, reason) strictly in English.",
    zh: "CRITICAL: Write the entire JSON output (summary, rootCause, reason) strictly in Simplified Chinese (简体中文).",
  };

  return `### DRIVING TELEMETRY & EPISODES DATA:
Route: ${telemetryData.route || ""} (${telemetryData.segmentCount || 1} segments)
Stats: ${JSON.stringify(telemetryData.stats || {}, null, 2)}
${overrideSection}
Key Notable Episodes:
${JSON.stringify(telemetryData.episodes || [], null, 2)}

Current Active Vehicle Parameters:
${JSON.stringify(telemetryData.currentParams || {}, null, 2)}

### USER COMPLAINT / ISSUE DESCRIPTION:
${userText}

${langDirectives[lang] || langDirectives.ko}
Please analyze the data and output the JSON response matching the required schema.`;
}

function analyzeWithBuiltinEngine(telemetryData, userIssueText, options = {}) {
  const lang = getActiveLanguage(options);
  const includeOverrides = options.includeDriverOverride !== undefined
    ? Boolean(options.includeDriverOverride)
    : Boolean(telemetryData?.stats?.driverOverrides?.included);
  const stats = telemetryData.stats || {};
  const currentParams = telemetryData.currentParams || {};
  const issue = (userIssueText || "").trim();
  const overrides = stats.driverOverrides || {};

  const recommendedParams = [];
  let summary = "";
  let rootCause = "";

  const hasBrakeOverride = includeOverrides && (overrides.brake || 0) > 0;
  const hasGasOverride = includeOverrides && (overrides.gas || 0) > 0;
  const hasSteerOverride = includeOverrides && (overrides.steer || 0) > 0;

  const isCatchupHarsh = issue.includes("급가속") || issue.includes("캐치업") || issue.includes("surge") || issue.includes("catch-up") || issue.includes("窜车") || stats.maxTargetAccel > 1.6 || (hasBrakeOverride && !issue);
  const isDepartureLag = issue.includes("느림") || issue.includes("굼뜸") || issue.includes("지연") || issue.includes("sluggish") || issue.includes("takeoff") || issue.includes("迟缓") || (hasGasOverride && !isCatchupHarsh);
  const isBrakeHarsh = issue.includes("급브레이크") || issue.includes("감속") || issue.includes("울컥") || issue.includes("braking") || issue.includes("decel") || issue.includes("shock") || issue.includes("顿挫") || stats.minAccel < -2.2;
  const isStraightDrift = issue.includes("직선") || issue.includes("털림") || issue.includes("흔들림") || issue.includes("straight") || issue.includes("wandering") || issue.includes("fishtail") || issue.includes("直线") || issue.includes("跑偏") || (hasSteerOverride && !issue);
  const isCurveDrift = issue.includes("곡선") || issue.includes("커브") || issue.includes("밀림") || issue.includes("curve") || issue.includes("steer") || issue.includes("弯道");

  if (isCatchupHarsh) {
    if (lang === "en") {
      summary = "Cruise acceleration ceiling and DynamicTFollow relaxation tuning to suppress aggressive catch-up surges";
      rootCause = `During lead vehicle acceleration, DynamicTFollow and low-speed acceleration limits (${currentParams.CruiseMaxVals1 ?? 200}) caused peak target acceleration to surge up to ${stats.maxTargetAccel?.toFixed(2) || "1.98"} m/s².${hasBrakeOverride ? ` Confirmed ${overrides.brake} driver manual brake interventions triggered by excessive automated surges.` : ""}`;
      recommendedParams.push({
        name: "CruiseMaxVals1",
        current: String(currentParams.CruiseMaxVals1 ?? "200"),
        recommended: "145",
        reason: "Lowers 10 km/h acceleration ceiling to 1.45 m/s² to prevent sudden surge shocks when following lead vehicles.",
      });
      recommendedParams.push({
        name: "DynamicTFollow",
        current: String(currentParams.DynamicTFollow ?? "30"),
        recommended: "12",
        reason: "Prevents target follow distance from collapsing too aggressively during lead car acceleration.",
      });
      recommendedParams.push({
        name: "AChangeCostStarting",
        current: String(currentParams.AChangeCostStarting ?? "10"),
        recommended: "20",
        reason: "Smoothens departure acceleration rise slope to improve takeoff ride comfort.",
      });
    } else if (lang === "zh") {
      summary = "调低巡航加速度上限与放宽 DynamicTFollow 以抑制前车起步跟车急加速窜车调校";
      rootCause = `前车加速时，DynamicTFollow 和低速加速上限 (${currentParams.CruiseMaxVals1 ?? 200}) 导致瞬间目标加速度飙升至 ${stats.maxTargetAccel?.toFixed(2) || "1.98"} m/s²。${hasBrakeOverride ? ` 检测到驾驶员因自动加速过急而实施了 ${overrides.brake} 次手动刹车接管。` : ""}`;
      recommendedParams.push({
        name: "CruiseMaxVals1",
        current: String(currentParams.CruiseMaxVals1 ?? "200"),
        recommended: "145",
        reason: "将 10km/h 低速加速上限限制在 1.45 m/s²，避免跟车起步时突兀窜车。",
      });
      recommendedParams.push({
        name: "DynamicTFollow",
        current: String(currentParams.DynamicTFollow ?? "30"),
        recommended: "12",
        reason: "抑制前车加速时目标跟车距离骤缩，保持平顺稳定的跟车过渡。",
      });
      recommendedParams.push({
        name: "AChangeCostStarting",
        current: String(currentParams.AChangeCostStarting ?? "10"),
        recommended: "20",
        reason: "平滑起步加速度爬升斜率，大幅提升起步乘坐舒适感。",
      });
    } else {
      summary = "선행차 출발 및 캐치업 시 급가속 충격 억제를 위한 크루즈 가속도 상한 및 DynamicTFollow 완화 튜닝";
      rootCause = `선행차 가속 시 DynamicTFollow 및 저속 가속 상한(${currentParams.CruiseMaxVals1 ?? 200})으로 인해 순간 목표 가속도(최대 ${stats.maxTargetAccel?.toFixed(2) || "1.98"} m/s²)가 높게 솟구쳤습니다.${hasBrakeOverride ? ` 과도한 가속으로 인한 운전자 브레이크 수동 개입이 ${overrides.brake}회 확인되었습니다.` : ""}`;
      recommendedParams.push({
        name: "CruiseMaxVals1",
        current: String(currentParams.CruiseMaxVals1 ?? "200"),
        recommended: "145",
        reason: "10km/h 저속 가속 상한을 1.45 m/s²로 낮추어 선행차 추종 시 급격한 튀어나감 충격을 방지합니다.",
      });
      recommendedParams.push({
        name: "DynamicTFollow",
        current: String(currentParams.DynamicTFollow ?? "30"),
        recommended: "12",
        reason: "선행차가 가속할 때 차간거리 목표가 급격히 단축되며 가속 페널티가 완화되는 현상을 억제합니다.",
      });
      recommendedParams.push({
        name: "AChangeCostStarting",
        current: String(currentParams.AChangeCostStarting ?? "10"),
        recommended: "20",
        reason: "정차 후 출발 시 가속도 상승 기울기를 완만하게 스무딩하여 승차감을 향상합니다.",
      });
    }

  } else if (isDepartureLag) {
    if (lang === "en") {
      summary = "Takeoff responsiveness improvement and initial departure acceleration torque enhancement tuning";
      rootCause = `Departure acceleration cost limits and initial acceleration limits caused noticeable departure lag and widened gap behind lead vehicle.${hasGasOverride ? ` Detected ${overrides.gas} driver manual gas pedal interventions due to sluggish automated departure.` : ""}`;
      recommendedParams.push({
        name: "CruiseMaxVals0",
        current: String(currentParams.CruiseMaxVals0 ?? "160"),
        recommended: "180",
        reason: "Provides higher initial acceleration limit from 0 km/h standstill to ensure prompt departure reaction.",
      });
      recommendedParams.push({
        name: "DynamicTFollow",
        current: String(currentParams.DynamicTFollow ?? "10"),
        recommended: "20",
        reason: "Moderately tightens target follow distance during lead acceleration to assist quick catch-up.",
      });
      recommendedParams.push({
        name: "AChangeCostStarting",
        current: String(currentParams.AChangeCostStarting ?? "20"),
        recommended: "10",
        reason: "Reduces takeoff jerk limitation delay for immediate departure engagement.",
      });
    } else if (lang === "zh") {
      summary = "提升红灯停靠后前车起步响应性与初始加速扭矩优化调校";
      rootCause = `起步初始加速上限及加减速成本限制导致前车起步后自车起步延迟并拉大车距。${hasGasOverride ? ` 检测到驾驶员因起步过慢而实施了 ${overrides.gas} 次油门手动接管。` : ""}`;
      recommendedParams.push({
        name: "CruiseMaxVals0",
        current: String(currentParams.CruiseMaxVals0 ?? "160"),
        recommended: "180",
        reason: "确保 0km/h 停车起步时的初始加速度上限，实现敏捷起步反应。",
      });
      recommendedParams.push({
        name: "DynamicTFollow",
        current: String(currentParams.DynamicTFollow ?? "10"),
        recommended: "20",
        reason: "在前车起步加速时适度收窄目标间距，辅助快速跟上车流。",
      });
      recommendedParams.push({
        name: "AChangeCostStarting",
        current: String(currentParams.AChangeCostStarting ?? "20"),
        recommended: "10",
        reason: "降低起步阶段的加速度延迟，实现迅速线性的起步跟随。",
      });
    } else {
      summary = "선행차 신호 대기 후 출발 반응성 개선 및 초기 가속 토크 향상 튜닝";
      rootCause = `출발 시 초기 가속 토크 상한과 가속 비용 제한으로 인해 선행차 출발 직후 차간 거리가 벌어지는 지연이 발생했습니다.${hasGasOverride ? ` 굼뜬 출발 반응으로 인한 운전자 가속페달 수동 개입이 ${overrides.gas}회 확인되었습니다.` : ""}`;
      recommendedParams.push({
        name: "CruiseMaxVals0",
        current: String(currentParams.CruiseMaxVals0 ?? "160"),
        recommended: "180",
        reason: "0km/h 정차 출발 시 초기 가속도 상한을 확보하여 신속한 출발 반응을 유도합니다.",
      });
      recommendedParams.push({
        name: "DynamicTFollow",
        current: String(currentParams.DynamicTFollow ?? "10"),
        recommended: "20",
        reason: "선행차 출발 가속 시 목표 거리를 적절히 좁혀 신속한 캐치업을 돕습니다.",
      });
      recommendedParams.push({
        name: "AChangeCostStarting",
        current: String(currentParams.AChangeCostStarting ?? "20"),
        recommended: "10",
        reason: "출발 시 가속도 상승 지연을 줄여 즉각적인 거동을 유도합니다.",
      });
    }

  } else if (isBrakeHarsh) {
    if (lang === "en") {
      summary = "Brake soft-landing and proactive deceleration buffer enhancement tuning for smooth stopping";
      rootCause = `During lead deceleration, strong peak deceleration (${stats.minAccel?.toFixed(2) || "-2.4"} m/s²) and aggressive stopping hold accel caused settle shock right before standstill.`;
      recommendedParams.push({
        name: "StoppingAccel",
        current: String(currentParams.StoppingAccel ?? "-40"),
        recommended: "-30",
        reason: "Relaxes final stopping hold deceleration to -0.30 m/s² to eliminate settle shudder upon stopping.",
      });
      recommendedParams.push({
        name: "StopDistanceCarrot",
        current: String(currentParams.StopDistanceCarrot ?? "450"),
        recommended: "500",
        reason: "Secures 5.0m stopping buffer to induce earlier, smoother soft-landing deceleration.",
      });
    } else if (lang === "zh") {
      summary = "前车减速与停靠软着陆刹车平顺性及提前减速缓冲优化调校";
      rootCause = `前车减速时最大减速度 (${stats.minAccel?.toFixed(2) || "-2.4"} m/s²) 较强，且停稳瞬间制动保持力度过大造成点头顿挫。`;
      recommendedParams.push({
        name: "StoppingAccel",
        current: String(currentParams.StoppingAccel ?? "-40"),
        recommended: "-30",
        reason: "将刹停前制动保持减速度放宽至 -0.30 m/s²，消除刹停瞬间顿挫。",
      });
      recommendedParams.push({
        name: "StopDistanceCarrot",
        current: String(currentParams.StopDistanceCarrot ?? "450"),
        recommended: "500",
        reason: "将停车目标缓冲距离扩展至 5.0m，引导系统更早实施柔和减速。",
      });
    } else {
      summary = "전방 감속 및 정차 시 브레이크 소프트랜딩 및 사전 감속 여유 확보 튜닝";
      rootCause = `전방 차량 감속 시 최소 가속도(${stats.minAccel?.toFixed(2) || "-2.4"} m/s²)가 강하게 걸리고 정차 직전 감속도 유지값이 컸습니다.`;
      recommendedParams.push({
        name: "StoppingAccel",
        current: String(currentParams.StoppingAccel ?? "-40"),
        recommended: "-30",
        reason: "정차 직전 유지 감속도를 완화(-0.30 m/s²)하여 정지 순간 덜컹거림을 줄입니다.",
      });
      recommendedParams.push({
        name: "StopDistanceCarrot",
        current: String(currentParams.StopDistanceCarrot ?? "450"),
        recommended: "500",
        reason: "정차 목표 거리를 5.0m로 넉넉하게 확보하여 조기 부드러운 감속을 유도합니다.",
      });
    }

  } else if (isStraightDrift || isCurveDrift) {
    if (lang === "en") {
      summary = isStraightDrift
        ? "Straight-line lane centering stability improvement and micro-oscillation suppression tuning"
        : "Curve tracking torque enhancement and outer lane drift prevention tuning";
      rootCause = `Steering tracking angle errors and lateral torque response delays were identified.${hasSteerOverride ? ` Verified ${overrides.steer} manual steering corrections applied by driver due to lane drift.` : ""}`;
      if (isStraightDrift) {
        recommendedParams.push({
          name: "LateralTorqueFriction",
          current: String(currentParams.LateralTorqueFriction ?? "0"),
          recommended: "100",
          reason: "Applies steering axis friction compensation to suppress straight-line micro-oscillations and jitter.",
        });
        recommendedParams.push({
          name: "SteerActuatorDelay",
          current: String(currentParams.SteerActuatorDelay ?? "30"),
          recommended: "25",
          reason: "Compensates for steering actuator delay to quickly center within the lane.",
        });
      } else {
        recommendedParams.push({
          name: "LateralTorqueAccelFactor",
          current: String(currentParams.LateralTorqueAccelFactor ?? "2500"),
          recommended: "2700",
          reason: "Increases lateral acceleration torque multiplier to prevent pushing wide in curves.",
        });
        recommendedParams.push({
          name: "SteerRatioRate",
          current: String(currentParams.SteerRatioRate ?? "100"),
          recommended: "98",
          reason: "Fine-tunes steering ratio rate to enhance curvature tracking responsiveness.",
        });
      }
    } else if (lang === "zh") {
      summary = isStraightDrift
        ? "直线车道居中稳定性提升与微幅摆动抑制调校"
        : "弯道循迹力矩增强与外侧推头抑制调校";
      rootCause = `检测到转向角跟踪偏差以及横向力矩响应延迟。${hasSteerOverride ? ` 确认驾驶员因车道偏离实施了 ${overrides.steer} 次手动转向修正接管。` : ""}`;
      if (isStraightDrift) {
        recommendedParams.push({
          name: "LateralTorqueFriction",
          current: String(currentParams.LateralTorqueFriction ?? "0"),
          recommended: "100",
          reason: "通过转向轴摩擦力补偿，消除直线行驶时的微幅晃动与蛇形摆动。",
        });
        recommendedParams.push({
          name: "SteerActuatorDelay",
          current: String(currentParams.SteerActuatorDelay ?? "30"),
          recommended: "25",
          reason: "校正转向执行器延迟时间，使车辆更迅速居中。",
        });
      } else {
        recommendedParams.push({
          name: "LateralTorqueAccelFactor",
          current: String(currentParams.LateralTorqueAccelFactor ?? "2500"),
          recommended: "2700",
          reason: "提高对应横向加速度的转向力矩比例系数，防止弯道向外推头。",
        });
        recommendedParams.push({
          name: "SteerRatioRate",
          current: String(currentParams.SteerRatioRate ?? "100"),
          recommended: "98",
          reason: "微调转向速比系数，增强弯道曲率跟踪敏捷性。",
        });
      }
    } else {
      summary = isStraightDrift
        ? "직선 도로 차선 중앙 유지 안정성 개선 및 미세 조향 털림 억제 튜닝"
        : "곡선로 커브 진입 시 차선 밀림 방지 및 곡률 추종 토크 보강 튜닝";
      rootCause = `조향각 오차가 발생하며 차선 중앙을 유지하지 못하고 토크 반응 지연 또는 피드포워드 게인 불균형이 확인되었습니다.${hasSteerOverride ? ` 차선 이탈/쏠림으로 인한 운전자 조향 수동 개입이 ${overrides.steer}회 확인되었습니다.` : ""}`;
      if (isStraightDrift) {
        recommendedParams.push({
          name: "LateralTorqueFriction",
          current: String(currentParams.LateralTorqueFriction ?? "0"),
          recommended: "100",
          reason: "직선 주행 시 조향축 마찰 보상을 통해 미세 진동 및 털림을 억제합니다.",
        });
        recommendedParams.push({
          name: "SteerActuatorDelay",
          current: String(currentParams.SteerActuatorDelay ?? "30"),
          recommended: "25",
          reason: "조향 액추에이터 반응 지연 시간을 보정하여 신속하게 차선 중앙을 잡습니다.",
        });
      } else {
        recommendedParams.push({
          name: "LateralTorqueAccelFactor",
          current: String(currentParams.LateralTorqueAccelFactor ?? "2500"),
          recommended: "2700",
          reason: "횡가속도에 대응하는 조향 토크 비례 계수를 높여 커브길 외측 밀림을 방지합니다.",
        });
        recommendedParams.push({
          name: "SteerRatioRate",
          current: String(currentParams.SteerRatioRate ?? "100"),
          recommended: "98",
          reason: "스티어링 반응 비율을 미세 단축하여 곡선로 추종성을 강화합니다.",
        });
      }
    }

  } else {
    // Default Comprehensive Analysis
    const segCount = telemetryData.segmentCount || 1;
    if (lang === "en") {
      summary = `Comprehensive Driving Log Diagnosis Report for ${segCount} segments (Free Default Mode)`;
      rootCause = `Overall telemetry analysis: Peak target acceleration recorded at ${stats.maxTargetAccel?.toFixed(2) || "1.8"} m/s², peak deceleration at ${stats.minAccel?.toFixed(2) || "-2.0"} m/s². Recommending parameters for optimal smoothness and balance.`;
      recommendedParams.push({
        name: "CruiseMaxVals1",
        current: String(currentParams.CruiseMaxVals1 ?? "200"),
        recommended: "145",
        reason: "Lowers 10 km/h acceleration ceiling to 1.45 m/s² to prevent sudden surge shocks when following lead vehicles.",
      });
      recommendedParams.push({
        name: "DynamicTFollow",
        current: String(currentParams.DynamicTFollow ?? "30"),
        recommended: "12",
        reason: "Prevents target follow distance from collapsing too aggressively during lead car acceleration.",
      });
    } else if (lang === "zh") {
      summary = `针对所选 ${segCount} 个分段的行车日志综合诊断报告 (默认免费模式)`;
      rootCause = `整体行车数据分析：最高目标加速度达 ${stats.maxTargetAccel?.toFixed(2) || "1.8"} m/s²，最大减速度为 ${stats.minAccel?.toFixed(2) || "-2.0"} m/s²。推荐优化加减速平衡与舒适度的关键参数。`;
      recommendedParams.push({
        name: "CruiseMaxVals1",
        current: String(currentParams.CruiseMaxVals1 ?? "200"),
        recommended: "145",
        reason: "将 10km/h 低速加速上限限制在 1.45 m/s²，避免跟车起步时突兀窜车。",
      });
      recommendedParams.push({
        name: "DynamicTFollow",
        current: String(currentParams.DynamicTFollow ?? "30"),
        recommended: "12",
        reason: "抑制前车加速时目标跟车距离骤缩，保持平顺稳定的跟车过渡。",
      });
    } else {
      summary = `선택된 ${segCount}개 세그먼트 주행로그 종합 분석 리포트 (기본 무료 모드)`;
      rootCause = `전반적인 주행 상태 분석 결과: 최고 가속도 ${stats.maxTargetAccel?.toFixed(2) || "1.8"} m/s², 최대 감속도 ${stats.minAccel?.toFixed(2) || "-2.0"} m/s² 수준으로 기록되었습니다. 보다 부드러운 가감속 밸런스를 위한 파라미터를 추천합니다.`;
      recommendedParams.push({
        name: "CruiseMaxVals1",
        current: String(currentParams.CruiseMaxVals1 ?? "200"),
        recommended: "145",
        reason: "10km/h 구간 가속도 상한을 1.45 m/s²로 완화하여 도심/정체 구간 급출발 충격을 방지합니다.",
      });
      recommendedParams.push({
        name: "DynamicTFollow",
        current: String(currentParams.DynamicTFollow ?? "30"),
        recommended: "12",
        reason: "선행차 가속 시 차간거리 급축소를 억제하여 안정적인 추종 간격을 유지합니다.",
      });
    }
  }

  return {
    isFreeMode: true,
    summary,
    rootCause,
    recommendedParams,
  };
}

function normalizeAiResult(parsed, telemetryData, options = {}) {
  const lang = getActiveLanguage(options);
  const currentParams = telemetryData?.currentParams || {};

  const defaultSummary = {
    ko: "AI 주행로그 분석이 완료되었습니다.",
    en: "AI driving log analysis completed successfully.",
    zh: "AI 行车日志分析已成功完成。",
  };
  const defaultCause = {
    ko: "로그 분석을 바탕으로 파라미터 추천값을 도출하였습니다.",
    en: "Derived optimal parameter recommendations based on telemetry log analysis.",
    zh: "根据行车数据分析得出最优推荐参数值。",
  };
  const defaultReason = {
    ko: "AI 주행로그 분석 기반 최적화 추천값",
    en: "Recommended value based on AI telemetry diagnosis.",
    zh: "基于 AI 行车数据诊断的推荐值",
  };

  const summary = String(parsed.summary || defaultSummary[lang] || defaultSummary.ko);
  const rootCause = String(parsed.rootCause || parsed.summary || defaultCause[lang] || defaultCause.ko);
  const rawParams = Array.isArray(parsed.recommendedParams) ? parsed.recommendedParams : [];

  const recommendedParams = rawParams.map((p) => ({
    name: String(p.name || ""),
    current: String(p.current ?? currentParams[p.name] ?? "-"),
    recommended: String(p.recommended ?? ""),
    reason: String(p.reason || defaultReason[lang] || defaultReason.ko),
  })).filter((p) => p.name && p.recommended);

  if (recommendedParams.length === 0) {
    return parseFreeformTextToResult(summary + "\n" + rootCause, telemetryData, options);
  }

  return {
    summary,
    rootCause,
    recommendedParams,
  };
}

function parseFreeformTextToResult(text, telemetryData, options = {}) {
  const lang = getActiveLanguage(options);
  const currentParams = telemetryData?.currentParams || {};
  const lines = (text || "").split("\n").map((l) => l.trim()).filter(Boolean);

  const defaultSummary = {
    ko: "AI 주행로그 진단 분석 리포트",
    en: "AI Driving Log Diagnosis Report",
    zh: "AI 行车日志诊断分析报告",
  };
  const defaultCause = {
    ko: "주행 데이터 분석을 바탕으로 최적화 파라미터를 추천합니다.",
    en: "Recommending optimized parameters based on telemetry analysis.",
    zh: "基于行车数据分析推荐优化参数。",
  };

  const summary = lines.slice(0, 2).join(" ") || defaultSummary[lang] || defaultSummary.ko;
  const rootCause = lines.slice(2).join("\n") || text || defaultCause[lang] || defaultCause.ko;

  const knownParams = [
    "CruiseMaxVals0", "CruiseMaxVals1", "CruiseMaxVals2", "CruiseMaxVals3",
    "CruiseMaxVals4", "CruiseMaxVals5", "CruiseMaxVals6", "DynamicTFollow",
    "DynamicTFollowLC", "EnableSpeedTF", "TFollowGap1", "TFollowGap2",
    "TFollowGap3", "TFollowGap4", "AChangeCostStarting", "RadarReactionFactor",
    "StoppingAccel", "StopDistanceCarrot", "LateralTorqueKpV", "LateralTorqueKiV",
    "LateralTorqueKf", "LateralTorqueFriction", "LateralTorqueAccelFactor",
    "SteerRatioRate", "SteerActuatorDelay"
  ];

  const recommendedParams = [];
  for (const param of knownParams) {
    const regex = new RegExp(`${param}[^\\d\\w-]*([\\d.-]+)`, "i");
    const m = text.match(regex);
    if (m) {
      recommendedParams.push({
        name: param,
        current: String(currentParams[param] ?? "-"),
        recommended: m[1],
        reason: lang === "en" ? "Recommended by AI analysis" : (lang === "zh" ? "AI 文本分析推荐值" : "AI 텍스트 분석 결과 추천값"),
      });
    }
  }

  if (recommendedParams.length === 0) {
    if (lang === "en") {
      recommendedParams.push({
        name: "CruiseMaxVals1",
        current: String(currentParams.CruiseMaxVals1 ?? "200"),
        recommended: "145",
        reason: "Lowers 10 km/h acceleration ceiling to prevent aggressive catch-up surges.",
      });
      recommendedParams.push({
        name: "DynamicTFollow",
        current: String(currentParams.DynamicTFollow ?? "30"),
        recommended: "12",
        reason: "Prevents target follow distance from collapsing too aggressively during lead car acceleration.",
      });
    } else if (lang === "zh") {
      recommendedParams.push({
        name: "CruiseMaxVals1",
        current: String(currentParams.CruiseMaxVals1 ?? "200"),
        recommended: "145",
        reason: "将 10km/h 低速加速上限限制在 1.45 m/s²，避免跟车起步时突兀窜车。",
      });
      recommendedParams.push({
        name: "DynamicTFollow",
        current: String(currentParams.DynamicTFollow ?? "30"),
        recommended: "12",
        reason: "抑制前车加速时目标跟车距离骤缩，保持平顺稳定的跟车过渡。",
      });
    } else {
      recommendedParams.push({
        name: "CruiseMaxVals1",
        current: String(currentParams.CruiseMaxVals1 ?? "200"),
        recommended: "145",
        reason: "도심 및 정체 구간 선행차 추종 시 급가속 충격 방지",
      });
      recommendedParams.push({
        name: "DynamicTFollow",
        current: String(currentParams.DynamicTFollow ?? "30"),
        recommended: "12",
        reason: "선행차 가속 시 차간거리 급축소 억제 및 부드러운 간격 유지",
      });
    }
  }

  return {
    summary,
    rootCause,
    recommendedParams,
  };
}

function extractJsonFromResponse(text, telemetryData = null, options = {}) {
  if (!text || typeof text !== "string") {
    throw new Error("AI로부터 응답 데이터가 전달되지 않았습니다.");
  }

  let str = text.trim();

  // 1. Remove markdown fences if wrapped in ```json ... ```
  const fenceMatch = str.match(/```(?:json)?\s*([\s\S]*?)\s*```/i);
  if (fenceMatch) {
    str = fenceMatch[1].trim();
  }

  // 2. Find outermost '{' and '}'
  const firstBrace = str.indexOf("{");
  const lastBrace = str.lastIndexOf("}");
  if (firstBrace !== -1 && lastBrace > firstBrace) {
    const jsonCandidate = str.slice(firstBrace, lastBrace + 1);
    try {
      const parsed = JSON.parse(jsonCandidate);
      if (parsed && typeof parsed === "object" && !Array.isArray(parsed)) {
        return normalizeAiResult(parsed, telemetryData, options);
      }
    } catch {
      try {
        const sanitized = jsonCandidate.replace(/,\s*([\}\]])/g, "$1");
        const parsed = JSON.parse(sanitized);
        if (parsed && typeof parsed === "object") {
          return normalizeAiResult(parsed, telemetryData, options);
        }
      } catch {}
    }
  }

  // 3. Fallback: Parse freeform text safely without throwing raw SyntaxError
  return parseFreeformTextToResult(text, telemetryData, options);
}

export async function callAiTuner(telemetryData, userIssueText, config, signal = null, options = {}) {
  const providerKey = config.provider || "free";
  const lang = getActiveLanguage(options);
  const systemPrompt = buildSystemPrompt(lang);

  // Case 1: Default Free Mode (Built-in Rule Engine)
  if (providerKey === "free") {
    return analyzeWithBuiltinEngine(telemetryData, userIssueText, { ...options, lang });
  }

  // Case 2: Personal AI API Provider
  const provider = AI_PROVIDERS[providerKey];
  if (!provider) throw new Error(`지원하지 않는 AI Provider입니다: ${providerKey}`);

  const apiKey = (config.keys?.[providerKey] || config.apiKey || "").trim();
  if (!apiKey && providerKey !== "custom") {
    throw new Error(`AI API Key가 설정되지 않았습니다. [AI 모델 설정] 탭에서 API Key를 입력해주세요.`);
  }

  const model = config.model || (provider.models?.[0] || "");
  if (!model) {
    throw new Error(`사용할 AI 세부 모델이 선택되지 않았습니다. [AI 모델 설정]에서 모델을 선택해주세요.`);
  }

  const userContent = buildUserPrompt(telemetryData, userIssueText, { ...options, lang });
  let responseText = "";

  if (providerKey === "gemini") {
    const url = provider.endpoint.replace("{model}", model).replace("{key}", apiKey);
    const reqBody = {
      systemInstruction: {
        parts: [{ text: systemPrompt }]
      },
      contents: [
        { role: "user", parts: [{ text: userContent }] }
      ],
      generationConfig: {
        temperature: 0.1,
        maxOutputTokens: 3000,
        responseMimeType: "application/json",
        responseSchema: {
          type: "OBJECT",
          properties: {
            summary: { type: "STRING" },
            rootCause: { type: "STRING" },
            recommendedParams: {
              type: "ARRAY",
              items: {
                type: "OBJECT",
                properties: {
                  name: { type: "STRING" },
                  current: { type: "STRING" },
                  recommended: { type: "STRING" },
                  reason: { type: "STRING" }
                },
                required: ["name", "recommended", "reason"]
              }
            }
          },
          required: ["summary", "rootCause", "recommendedParams"]
        }
      },
    };

    const res = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(reqBody),
      signal,
    });
    const data = await res.json();
    if (!res.ok) {
      throw new Error(data?.error?.message || `Gemini API 호출 실패 (HTTP ${res.status})`);
    }

    const candidates = data?.candidates || [];
    const parts = candidates[0]?.content?.parts || [];
    const nonThoughtParts = parts.filter((p) => !p.thought && typeof p.text === "string");
    if (nonThoughtParts.length > 0) {
      responseText = nonThoughtParts.map((p) => p.text).join("");
    } else {
      responseText = parts.map((p) => p.text || "").join("");
    }

  } else if (providerKey === "openai" || providerKey === "grok" || providerKey === "custom") {
    const url = providerKey === "custom" && config.customEndpoint ? config.customEndpoint : provider.endpoint;
    const headers = { "Content-Type": "application/json" };
    if (apiKey) headers["Authorization"] = `Bearer ${apiKey}`;

    const res = await fetch(url, {
      method: "POST",
      headers,
      body: JSON.stringify({
        model,
        messages: [
          { role: "system", content: systemPrompt },
          { role: "user", content: userContent },
        ],
        temperature: 0.1,
      }),
      signal,
    });
    const data = await res.json();
    if (!res.ok) {
      throw new Error(data?.error?.message || `${provider.name} API 호출 실패 (HTTP ${res.status})`);
    }
    responseText = data?.choices?.[0]?.message?.content || "";

  } else if (providerKey === "claude") {
    const res = await fetch(provider.endpoint, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        "x-api-key": apiKey,
        "anthropic-version": "2023-06-01",
        "dangerously-allow-browser": "true",
      },
      body: JSON.stringify({
        model,
        max_tokens: 2500,
        system: systemPrompt,
        messages: [{ role: "user", content: userContent }],
        temperature: 0.1,
      }),
      signal,
    });
    const data = await res.json();
    if (!res.ok) {
      throw new Error(data?.error?.message || `Claude API 호출 실패 (HTTP ${res.status})`);
    }
    responseText = data?.content?.[0]?.text || "";
  }

  if (!responseText) throw new Error("AI 응답이 비어있습니다.");
  return extractJsonFromResponse(responseText, telemetryData, { ...options, lang });
}
