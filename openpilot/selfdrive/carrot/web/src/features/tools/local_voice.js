import { driveAccessEnabled, executeDriveCommand } from './assistant_drive.js';

// The Android wrapper calls this on the verified, visible device page. No relay/account is involved.
export function createLocalVoice(openSetting, driveEnabled, drive, now = Date.now) {
  let current = null, lastDrive = -Infinity;
  return Object.freeze({
    start(action, id, expires) {
      if (document.hidden || !/^[A-Za-z0-9_-]{43}$/.test(id) || !Number.isFinite(expires) ||
          expires <= now() || expires - now() > 5000 || !['lane_left', 'lane_right', 'blinker_settings'].includes(action)) return false;
      if (current?.id === id) return true;
      if (current && !current.status) return false;
      const request = { id, status: '' };
      current = request;
      (async () => {
        if (action === 'blinker_settings') return await openSetting('AutoBlinkerControl') === 'opened' ? 'opened' : 'unavailable';
        if (!driveEnabled()) return 'permission';
        if (now() - lastDrive < 10000) return 'duplicate';
        lastDrive = now();
        const reply = await drive({ id, action, expires_at: expires, operation: 'execute' });
        return reply?.result?.status || 'unverified';
      })().then(status => { request.status = status; }, () => { request.status = 'unverified'; });
      return true;
    },
    result(id) { return current?.id === id ? current.status : 'unverified'; },
  });
}

export const localVoice = createLocalVoice(name => globalThis.openAssistantSetting(name), driveAccessEnabled, executeDriveCommand);
