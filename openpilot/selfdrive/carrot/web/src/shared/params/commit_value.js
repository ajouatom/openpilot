/**
 * The single funnel every settings write goes through.
 *
 * Two things it fixes over calling setParam() directly:
 *
 * 1. It adopts the value the server reports back. The server clamps and rounds
 *    before storing, so echoing the value we sent can leave the screen showing
 *    something the device does not actually hold. Today the two happen to
 *    agree for every parameter; the moment one stops agreeing the divergence
 *    would be silent.
 *
 * 2. It labels where the write came from. A parameter can also be changed by a
 *    profile, a backup restore, or a steering-wheel button in the driving code,
 *    and the history is only useful if those are told apart.
 */

export const PARAM_CHANGE_SOURCES = Object.freeze({
  webUi: "web_ui",
  profile: "profile",
  restore: "restore",
  resetDefaults: "reset_defaults",
  intro: "intro",
  undo: "undo",
  device: "device",
});

function normalizeSource(source) {
  const text = String(source || "").trim();
  return Object.values(PARAM_CHANGE_SOURCES).includes(text) ? text : PARAM_CHANGE_SOURCES.webUi;
}

export function createSettingCommitter(options = {}) {
  const postJson = options.postJson;
  if (typeof postJson !== "function") throw new TypeError("postJson must be a function");

  const endpoint = options.endpoint || "/api/param_set";
  const onCommitted = typeof options.onCommitted === "function" ? options.onCommitted : null;
  const dispatchEvent = typeof options.dispatchEvent === "function" ? options.dispatchEvent : null;

  async function commit(name, value, commitOptions = {}) {
    const key = String(name || "").trim();
    if (!key) throw new TypeError("A parameter name is required");

    const source = normalizeSource(commitOptions.source);
    const payload = await postJson(endpoint, { name: key, value, source });

    // The server is the authority on what was actually stored.
    const stored = payload && Object.prototype.hasOwnProperty.call(payload, "value")
      ? payload.value
      : value;
    const adjusted = String(stored) !== String(value);

    const result = Object.freeze({ name: key, value: stored, requested: value, source, adjusted });
    onCommitted?.(result);
    dispatchEvent?.(result);
    return result;
  }

  return Object.freeze({ commit, sources: PARAM_CHANGE_SOURCES });
}
