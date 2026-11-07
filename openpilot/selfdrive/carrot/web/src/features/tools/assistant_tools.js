const KEY = 'carrot-assistant-tools-v1';
let credential = null, completed = null;
try { credential = localStorage.getItem(KEY); } catch {}
const t = (key, fallback) => globalThis.getUIText?.(`assistant_${key}`, fallback) || fallback;

export const toolPanelButtons = Object.freeze({
  car_select: 'btnToolsCarSelect', language: 'btnToolsLanguage', web_settings: 'btnToolsWebSettings',
  device_info: 'btnDeviceInfo', bluetooth: 'btnToolsBluetooth', assistant: 'btnToolsAssistant',
  git_branch: 'btnGitBranch', git_log: 'btnGitLog', git_reset_repo: 'btnGitResetRepo',
  restore_settings: 'btnRestoreSettings', qr_backup_settings: 'btnQrBackupSettings',
  qr_restore_settings: 'btnQrRestoreSettings', copy_settings: 'btnCopySettings', view_settings: 'btnViewSettings',
});
// File/clipboard/camera APIs require a human gesture; repository reset has side effects.
const manualPanels = new Set(['git_reset_repo', 'restore_settings', 'qr_restore_settings', 'copy_settings']);

async function local(operation, body = {}, token = credential) {
  const response = await fetch(`/api/assistant/tools/${operation}`, {
    method: 'POST', headers: { 'Content-Type': 'application/json', ...(token ? { Authorization: `Bearer ${token}` } : {}) },
    body: JSON.stringify(body), signal: AbortSignal.timeout(10000),
  });
  const result = await response.json();
  if (!response.ok || !result.ok) throw Object.assign(new Error(result.error || 'Tools access failed'), { status: response.status });
  return result;
}
function remember(value) {
  credential = value; completed = null;
  try { if (value) localStorage.setItem(KEY, value); else localStorage.removeItem(KEY); } catch {}
}
export async function revokeToolsAccess() {
  if (credential) {
    try { await local('disable'); } catch (error) { if (error.status !== 403) throw error; }
  }
  remember(null);
}
export function mountToolsAccess(container) {
  const section = document.createElement('section'), help = document.createElement('p');
  help.textContent = t('tools_help', 'Optional: allow Tools panels, confirmed Git updates, restart, reset, deletion, backups, recording and diagnostic upload requests. Maintenance requires parking and disengaging. Diagnostics go to the existing Carrot upload server; ChatGPT receives the action and status. Also select Tools access when reconnecting ChatGPT.');
  const button = document.createElement('button'), result = document.createElement('p');
  button.type = 'button'; button.className = 'btn'; result.setAttribute('role', 'status');
  const render = () => { button.textContent = credential ? t('tools_disable', 'Revoke Tools access') : t('tools_enable', 'Allow Tools panels and actions'); };
  button.onclick = async () => {
    button.disabled = true;
    try {
      if (credential) await revokeToolsAccess();
      else remember((await local('enable', { confirmed: true }, null)).token);
      result.textContent = credential ? t('tools_enabled', 'Allowed here. In ChatGPT, select Installed > CarrotAssist > Reconnect and select Tools access.') : t('tools_disabled', 'Tools access revoked');
    } catch (error) { result.textContent = error.message; }
    finally { button.disabled = false; render(); }
  };
  render(); section.append(help, button, result); container.append(section);
}
export function validToolsCommand(command, now = Date.now()) {
  return command && /^[A-Za-z0-9_-]{43}$/.test(command.id) &&
    ['open', 'prepare', 'commit', 'status', 'cancel'].includes(command.operation) &&
    Number.isFinite(command.expires_at) && command.expires_at > now && command.expires_at - now <= 30000;
}
async function openNaverMap(id, expiresAt) {
  if (!/CarrotAssistPhone\/1\b/.test(navigator.userAgent)) throw new Error(t('phone_app_required', 'Open CarrotAssist 0.2.5 or later on Android.'));
  if (document.hidden || expiresAt <= Date.now()) throw new Error(t('phone_app_expired', 'Phone request expired.'));
  return new Promise((resolve, reject) => {
    const stop = () => { clearTimeout(timer); window.removeEventListener('carrot-phone-result', receive); };
    const receive = event => {
      if (event.detail?.id !== id) return;
      stop();
      if (event.detail.status === 'requested') resolve('requested');
      else reject(new Error(event.detail.status === 'not_installed' ? t('naver_missing', 'Naver Map is not installed.') : t('phone_app_failed', 'Could not open the phone app.')));
    };
    const timer = setTimeout(() => { stop(); reject(new Error(t('phone_app_unknown', 'Phone launch result unavailable. Do not retry automatically.'))); }, Math.min(3500, expiresAt - Date.now()));
    window.addEventListener('carrot-phone-result', receive);
    window.location.assign(`carrotassist://phone/naver_map/${id}?expires=${expiresAt}`);
  });
}
export async function openToolPanel(action) {
  if (document.hidden) throw new Error('Keep Carrot Web visible');
  if (document.querySelector('.app-dialog:not([hidden])')) throw new Error('Close the current dialog first');
  if (action !== 'tools' && action !== 'egpu' && !Object.hasOwn(toolPanelButtons, action)) throw new Error('Unknown Tools panel');
  globalThis.showPage('tools', true);
  globalThis.CarrotToolsRuntime?.init?.();
  if (document.getElementById('pageTools')?.hidden !== false) throw new Error('Tools page did not open');
  const button = action === 'egpu' ? document.getElementById('egpuModelCard') : document.getElementById(toolPanelButtons[action]);
  if (action === 'tools') return 'opened';
  if (!button || button.hidden || button.disabled) throw new Error('This panel is unavailable on this device');
  const group = button.closest('.tools-group');
  if (group && !group.classList.contains('is-open')) group.querySelector('.tools-group__toggle')?.click();
  button.scrollIntoView({ block: 'center' }); button.focus();
  if (action === 'egpu' || manualPanels.has(action)) return 'needs_input';
  button.click();
  // Event handlers can fetch data before opening a dialog. Do not call this an applied change.
  for (let i = 0; i < 30; i++) {
    if (document.querySelector('.app-dialog:not([hidden])') || (action === 'car_select' && document.getElementById('pageCar')?.hidden === false)) return 'opened';
    await new Promise(resolve => setTimeout(resolve, 100));
  }
  return 'needs_input';
}
export async function executeToolsCommand(command) {
  if (!validToolsCommand(command) || document.hidden) return null;
  if (completed?.id === command.id) return completed;
  let result;
  try {
    if (!credential) throw new Error(t('tools_required', 'Allow Tools access in Carrot Web > Tools > ChatGPT assistant first.'));
    const { ok, ...value } = await local('execute', { command });
    result = value;
    if (result.status === 'open_requested') result = { ...result, status: result.action === 'naver_map' ? await openNaverMap(command.id, command.expires_at) : await openToolPanel(result.action) };
    if (['running', 'requested'].includes(result.status)) {
      try { globalThis.CarrotToolsRuntime?.logNotice?.(t('tools_started', 'Voice Tools request accepted. Check the Tools history for progress.'), { label: result.action }); } catch {}
    }
  } catch (error) {
    result = { status: 'failed', error: String(error.message).slice(0, 300) };
  }
  completed = { id: command.id, result };
  return completed;
}
