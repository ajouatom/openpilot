import { driveAccessEnabled, executeDriveCommand } from './assistant_drive.js';
import { assignSpokenSetting } from './spoken_setting.js';

// The Android wrapper calls this on the verified, visible device page. No relay/account is involved.
export function createLocalVoice(openSetting, driveEnabled, drive, now = Date.now, assign = assignSpokenSetting) {
  let current = null, lastDrive = -Infinity;
  const recent = new Map();
  function start(id, expires, run) {
    if (document.hidden || !/^[A-Za-z0-9_-]{43}$/.test(id) || !Number.isFinite(expires) ||
        expires <= now() || expires - now() > 5000) return false;
    for (const [key, prior] of recent) if (prior.expires <= now()) recent.delete(key);
    if (recent.has(id)) return true;
    if (current && !current.status) return false;
    const request = { id, expires, status: '' };
    current = request;
    recent.set(id, request);
    run().then(status => { request.status = status; }, () => { request.status = 'unverified'; });
    return true;
  }
  return Object.freeze({
    findSettings(query, id, expires) {
      if (typeof query !== 'string' || !query.trim() || query.length > 80) return false;
      return start(id, expires, async () => await globalThis.findPhoneSettings(query, id) || 'unavailable');
    },
    setResultValue(searchId, position, value, language, id, expires) {
      if (!/^[A-Za-z0-9_-]{43}$/.test(searchId) || !Number.isInteger(position) || position < 1 ||
          !Number.isSafeInteger(value)) return false;
      return start(id, expires, async () => {
        const selected = () => globalThis.phoneSearchResultName(searchId, position);
        const name = selected();
        if (!name) return { status: 'no_result' };
        return assign(name, value, id, expires, { selected, language });
      });
    },
    setValue(name, value, id, expires) {
      if (typeof name !== 'string' || !name.trim() || name.length > 100 || !Number.isSafeInteger(value)) return false;
      return start(id, expires, () => assign(name, value, id, expires));
    },
    start(action, id, expires) {
      if (!['lane_left', 'lane_right', 'blinker_settings'].includes(action)) return false;
      return start(id, expires, async () => {
        if (action === 'blinker_settings') return await openSetting('AutoBlinkerControl') === 'opened' ? 'opened' : 'unavailable';
        if (!driveEnabled()) return 'permission';
        if (now() - lastDrive < 10000) return 'duplicate';
        lastDrive = now();
        const reply = await drive({ id, action, expires_at: expires, operation: 'execute' });
        return reply?.result?.status || 'unverified';
      });
    },
    result(id) { return (recent.get(id) || (current?.id === id ? current : null))?.status ?? 'unverified'; },
  });
}

export const localVoice = createLocalVoice(name => globalThis.openAssistantSetting(name), driveAccessEnabled, executeDriveCommand);
