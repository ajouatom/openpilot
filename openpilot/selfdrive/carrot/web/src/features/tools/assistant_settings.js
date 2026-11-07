const KEY = 'carrot-assistant-settings-v1';
let credential = null, completed = null;
try { credential = localStorage.getItem(KEY); } catch {}
const t = (key, fallback) => globalThis.getUIText?.(`assistant_${key}`, fallback) || fallback;

function refreshCredential() {
  // Permission can be enabled/revoked in another same-origin WebView or tab.
  // Never keep a stale startup snapshot, and never retry an old write on refresh.
  try { credential = localStorage.getItem(KEY); } catch {}
}

export function settingCommandBlocked(command, active) {
  return command?.operation !== 'read' && /^(INPUT|TEXTAREA|SELECT)$/.test(active?.tagName) && !active.readOnly;
}

async function local(operation, body = {}, token = credential) {
  const response = await fetch(`/api/assistant/settings/${operation}`, {
    method: 'POST', headers: { 'Content-Type': 'application/json', ...(token ? { Authorization: `Bearer ${token}` } : {}) },
    body: JSON.stringify(body), signal: AbortSignal.timeout(10000),
  });
  const result = await response.json();
  if (!response.ok || !result.ok) {
    const error = new Error(result.error || 'Setting access failed');
    error.status = response.status;
    throw error;
  }
  return result;
}

function remember(value) {
  credential = value; completed = null;
  try { if (value) localStorage.setItem(KEY, value); else localStorage.removeItem(KEY); } catch {}
}

export async function revokeSettingAccess() {
  refreshCredential();
  if (credential) {
    try { await local('disable'); }
    catch (error) { if (error.status !== 403) throw error; }
  }
  remember(null);
}

export function mountSettingAccess(container) {
  refreshCredential();
  const section = document.createElement('section'), help = document.createElement('p');
  help.textContent = t('settings_help', 'Optional: let this connection read and change one setting at a time. Its current value and change result are sent to ChatGPT and the Carrot relay. Clear commands specifying a setting and value are applied after bounds checks without another question. Ambiguous requests need clarification. Supported driving settings may also be changed while driving. Initial permission must be granted while stationary and disengaged. Also authorize setting access when connecting ChatGPT.');
  const button = document.createElement('button'), result = document.createElement('p');
  button.type = 'button'; button.className = 'btn'; result.setAttribute('role', 'status');
  const render = () => { button.textContent = credential ? t('settings_disable', 'Revoke setting access') : t('settings_enable', 'Allow setting reads and changes'); };
  button.onclick = async () => {
    refreshCredential();
    button.disabled = true;
    try {
      if (credential) await revokeSettingAccess();
      else { const response = await local('enable', { confirmed: true }, null); remember(response.token); }
      result.textContent = credential ? t('settings_enabled', 'Setting access allowed on this device. Authorize the same scope in ChatGPT.') : t('settings_disabled', 'Setting access revoked');
    } catch (error) { result.textContent = error.message; }
    finally { button.disabled = false; render(); }
  };
  render(); section.append(help, button, result); container.append(section);
}

export function validSettingCommand(command, now = Date.now()) {
  return command && /^[A-Za-z0-9_-]{43}$/.test(command.id) &&
    ['read', 'prepare', 'commit', 'cancel'].includes(command.operation) &&
    Number.isFinite(command.expires_at) && command.expires_at > now && command.expires_at - now <= 30000;
}

export async function executeSettingCommand(command) {
  if (!validSettingCommand(command)) return null;
  if (completed?.id === command.id) return completed;
  refreshCredential();
  let result;
  try {
    if (!credential) throw new Error(t('settings_required', 'Allow setting access in Carrot Web Tools > ChatGPT assistant first.'));
    const response = await local('execute', { command });
    const { ok, ...value } = response;
    result = value;
  } catch (error) {
    result = { status: 'failed', error: String(error.message).slice(0, 300) };
  }
  completed = { id: command.id, result };
  // Navigation must not turn a verified device write into a reported failure.
  if (result.status === 'applied') {
    // The destination may already be open, so navigation alone will not reload
    // its controls. Discard in-flight cached reads before applying the readback.
    const values = { [result.name]: result.current };
    try {
      globalThis.CarrotSettingsRuntime?.values?.clear();
      globalThis.CarrotSettingsRuntime?.values?.applyValues(values);
      globalThis.applyRestoredSettingValuesToRenderedItems?.(values);
    } catch {}
    try { await globalThis.openAssistantSetting?.(result.name); } catch {}
    try { globalThis.applyRestoredSettingValuesToRenderedItems?.(values); } catch {}
  }
  return completed;
}
