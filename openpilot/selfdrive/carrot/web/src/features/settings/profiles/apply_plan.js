/**
 * Pure data steps of applying a profile, split out of the settings page so
 * they can be tested without a dialog or the network.
 *
 * These decide *what* to send and *what* changed; the page keeps the dialog,
 * the request and the events.
 */

function isObject(value) {
  return typeof value === "object" && value !== null;
}

function entriesOf(preview) {
  return Array.isArray(preview?.entries) ? preview.entries : [];
}

/**
 * The values to send to the apply endpoint: only the parameters the preview
 * marked `apply`, taken from the profile itself (raw values, so the server
 * re-normalizes and re-checks them). The 162 unchanged parameters are left
 * out, so a profile apply no longer ships and re-reads the whole set.
 */
export function selectProfileApplyValues(preview, profileValues) {
  const source = isObject(profileValues) ? profileValues : {};
  const applyValues = {};
  for (const entry of entriesOf(preview)) {
    const key = entry?.key;
    if (entry?.apply && key != null && Object.prototype.hasOwnProperty.call(source, key)) {
      applyValues[key] = source[key];
    }
  }
  return applyValues;
}

/**
 * The values that actually landed, from the apply response: entries the server
 * re-confirmed as `apply` minus the ones it reported as failed. This is what
 * the page announces so caches and other views update to the stored value.
 */
export function collectRestoredValues(applyResult) {
  const failed = new Set(
    (applyResult?.result?.fails || [])
      .map((entry) => String(entry?.key || ""))
      .filter(Boolean),
  );
  const restored = {};
  for (const entry of entriesOf(applyResult?.preview)) {
    const key = entry?.key;
    if (!entry?.apply || key == null || failed.has(String(key))) continue;
    restored[key] = entry.value;
  }
  return restored;
}
