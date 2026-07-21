import { PARAM_CHANGE_SOURCES, createSettingCommitter } from "./commit_value.js";

function isObject(value) {
  return (typeof value === "object" && value !== null) || typeof value === "function";
}

export function installParamCommitFacade(target = globalThis, environment = {}) {
  if (!isObject(target)) throw new TypeError("A facade target is required");

  const create = environment.createSettingCommitter || createSettingCommitter;
  if (typeof create !== "function") throw new TypeError("A committer factory is required");

  const api = Object.freeze({ create, sources: PARAM_CHANGE_SOURCES });
  target.CarrotParamCommit = api;
  return api;
}
