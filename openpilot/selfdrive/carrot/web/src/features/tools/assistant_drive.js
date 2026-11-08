// A driving grant is separate from settings and never leaves the device origin.
const KEY = 'carrot-assistant-drive-v1';
let token = null;
const t = (key, fallback) => globalThis.getUIText?.(`assistant_drive_${key}`, fallback) || fallback;
try { token = localStorage.getItem(KEY); } catch {}
export const driveAccessEnabled = () => Boolean(token);
async function local(operation, body = {}) {
  const response = await fetch(`/api/assistant/drive/${operation}`, {
    method: 'POST', headers: { 'Content-Type': 'application/json', ...(token ? { Authorization: `Bearer ${token}` } : {}) },
    body: JSON.stringify(body), signal: AbortSignal.timeout(3000),
  });
  const result = await response.json();
  if (!response.ok || !result.ok) {
    const error = new Error(result.error || 'Driving access unavailable');
    error.status = response.status;
    throw error;
  }
  return result;
}
export async function revokeDriveAccess() {
  if (token) {
    try { await local('disable'); }
    catch (error) { if (error.status !== 403) throw error; }
  }
  token = null;
  try { localStorage.removeItem(KEY); } catch {}
}
export function mountDriveAccess(container) {
  const section = document.createElement('section'), help = document.createElement('p'), button = document.createElement('button');
  help.textContent = t('help', 'Allow voice driving commands on this device while parked and disengaged. Direct phone commands go to the device without ChatGPT registration. For commands through ChatGPT, also grant driving access in the ChatGPT connection. Both routes use the existing vehicle conditions.');
  button.type = 'button'; button.className = 'btn';
  const label = () => { button.textContent = token ? t('disable', 'Revoke driving commands') : t('enable', 'Allow voice lane-change and set-speed commands'); };
  button.onclick = async () => {
    button.disabled = true;
    try {
      if (token) await revokeDriveAccess();
      else {
        const result = await local('enable', { confirmed: true });
        token = result.token;
        try { localStorage.setItem(KEY, token); } catch {}
      }
    } catch (error) { help.textContent = error.message; }
    finally { button.disabled = false; label(); }
  };
  label(); section.append(help, button); container.append(section);
}
export async function executeDriveCommand(command) {
  if (!command || !/^[A-Za-z0-9_-]{43}$/.test(command.id) || !['execute', 'status'].includes(command.operation) ||
      !Number.isFinite(command.expires_at) || command.expires_at <= Date.now() || command.expires_at - Date.now() > 5000) return null;
  try {
    if (!token || document.hidden) throw new Error(t('required', 'Keep Carrot Web visible and allow driving commands first.'));
    const { ok, ...result } = await local('execute', { command });
    return { id: command.id, result };
  } catch (error) {
    // Uncertain local delivery must never cause automatic re-execution.
    return { id: command.id, result: { id: command.id, status: 'unverified', reason: String(error.message).slice(0, 240) } };
  }
}
