import "./model.js";
import "./renderers.js";
import "./data.js";
import "./forward.js";
import "./insights.js";

const FEATURE_GLOBAL = "CarrotReplayFeature";

function requireFeatureDependency(target, name) {
  const dependency = target?.[name];
  if (!dependency) throw new Error(`Replay feature dependency is unavailable: ${name}`);
  return dependency;
}

export function installCarrotReplayFeature(target = globalThis) {
  if (!target || (typeof target !== "object" && typeof target !== "function")) {
    throw new TypeError("Replay feature target must be an object");
  }
  if (target[FEATURE_GLOBAL]) return target[FEATURE_GLOBAL];

  const feature = Object.freeze({
    model: requireFeatureDependency(target, "CarrotReplayInsightsModel"),
    renderers: requireFeatureDependency(target, "CarrotReplayRendererRegistry"),
    data: requireFeatureDependency(target, "CarrotReplayInsightsData"),
    forward: requireFeatureDependency(target, "CarrotReplaySensorTopview"),
    insights: requireFeatureDependency(target, "CarrotReplayInsights"),
  });
  target[FEATURE_GLOBAL] = feature;
  return feature;
}
