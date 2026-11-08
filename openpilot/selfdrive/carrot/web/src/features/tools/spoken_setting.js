import { executeLocalSetting, settingAccessToken, updateDisplayedSetting } from './assistant_settings.js';

const key = text => text.normalize('NFKC').trim().toLowerCase().replace(/\s+/g, '');
const shortTitle = title => title.split(/[:：]/, 1)[0].replace(/\s*\([^()]*\)\s*$/, '').trim();

export function resolveSpokenSetting(items, spoken) {
  const needle = key(spoken);
  const matches = new Map();
  for (const item of items) {
    if (!item?.name) continue;
    const names = [item.name, item.title, item.etitle].filter(Boolean);
    const aliases = names.flatMap(name => [name, shortTitle(name)]);
    const accel = /^CruiseMaxVals(\d+)$/.exec(item.name);
    if (accel) {
      aliases.push('acceleration setting ' + accel[1]);
      if (accel[1] === '0') aliases.push('가속도 설정 영', '가속도 설정 공', '가속도 설정 제로', 'acceleration setting zero');
    }
    if (aliases.some(name => key(name) === needle)) matches.set(item.name, item);
  }
  return matches.size === 1 ? matches.values().next().value : null;
}

async function catalog() {
  const response = await fetch('/api/settings', { signal: AbortSignal.timeout(4000) });
  if (!response.ok) throw new Error('Catalog unavailable');
  const body = await response.json();
  if (!body.ok || !body.items_by_group) throw new Error('Catalog unavailable');
  return Object.values(body.items_by_group).flat();
}

function requestId() {
  const bytes = crypto.getRandomValues(new Uint8Array(32));
  return btoa(String.fromCharCode(...bytes)).replaceAll('+', '-').replaceAll('/', '_').replaceAll('=', '');
}

export async function assignSpokenSetting(spoken, value, id, expires, deps = {}) {
  const grant = deps.grant || settingAccessToken;
  const send = deps.send || executeLocalSetting;
  const token = grant();
  if (!token) return { status: 'permission' };
  const usable = () => !document.hidden && Date.now() < expires && grant() === token && (!deps.selected || deps.selected() === spoken);
  if (!usable()) return { status: 'unverified' };
  const items = await (deps.catalog || catalog)();
  const item = deps.selected ? items.find(entry => entry.name === spoken) : resolveSpokenSetting(items, spoken);
  if (!item) return { status: 'unknown' };
  if (!Number.isSafeInteger(value)) return { status: 'unverified' };
  const { name, min, max } = item;
  const label = deps.selected ? shortTitle((deps.language === 'ko' ? item.title : item.etitle) || name) : spoken;
  if (value < min || value > max) return { status: 'range', min, max };
  const command = (operation, fields, identity = requestId()) =>
    send({ operation, id: identity, expires_at: expires, ...fields }, token);
  if (!usable()) return { status: 'unverified' };
  try {
    let proposal;
    try {
      proposal = await command('prepare', { name, mode: 'absolute', value });
    } catch (error) {
      if (!error.message.startsWith('No integer value change') || !usable()) throw error;
      const read = await command('read', { name });
      if (read.status === 'read' && read.name === name && read.current === value)
        return { status: 'unchanged', label, current: value };
      return { status: 'unverified' };
    }
    if (!usable() || proposal.status !== 'prepared' || proposal.name !== name || proposal.proposed !== value ||
        !/^[A-Za-z0-9_-]{43}$/.test(proposal.proposal_id)) return { status: 'unverified' };
    const result = await command('commit', { proposal_id: proposal.proposal_id }, id);
    if (result.status !== 'applied' || result.name !== name || result.current !== value) return { status: 'unverified' };
    (deps.display || updateDisplayedSetting)(result);
    return { status: 'applied', label, current: result.current };
  } catch (error) {
    return { status: error.status === 403 ? 'permission' : 'unverified' };
  }
}
