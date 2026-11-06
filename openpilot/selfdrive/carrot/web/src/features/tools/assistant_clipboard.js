// LAN HTTP cannot use the secure-context Clipboard API. Keep its fallback
// synchronous inside the user's click and select the visible dialog field so
// the dialog focus trap cannot redirect selection to another control.
export async function copyPairingMessage(field, environment = globalThis) {
  if (!field?.value) return false;
  const value = field.value;
  if (environment.isSecureContext && environment.navigator?.clipboard?.writeText) {
    try {
      await environment.navigator.clipboard.writeText(value);
      return true;
    } catch { /* Denied clipboard permission: try the selected-text fallback. */ }
  }
  if (field.isConnected === false || field.value !== value) return false;
  try {
    field.focus();
    field.select();
    field.setSelectionRange(0, value.length);
    return environment.document.execCommand('copy') === true;
  } catch {
    // Keep the text selected for the browser's native Copy action.
    return false;
  }
}
