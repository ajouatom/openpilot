import { randomBytes } from 'node:crypto';
import { z } from 'zod';

const numeric = z.number().finite();
export const settingResult = z.object({
  status: z.enum(['read', 'prepared', 'applied', 'cancelled', 'failed']),
  name: z.string().regex(/^[A-Za-z][A-Za-z0-9_]{0,100}$/).optional(),
  title: z.string().max(160).optional(), current: numeric.optional(), previous: numeric.optional(),
  min: numeric.optional(), max: numeric.optional(), step: numeric.positive().optional(), proposed: numeric.optional(),
  unit_note: z.string().max(200).optional(), step_note: z.string().max(200).optional(), stationary_required: z.boolean().optional(),
  proposal_id: z.string().regex(/^[A-Za-z0-9_-]{43}$/).optional(),
  expires_in_seconds: z.number().int().min(0).max(60).optional(), confirmation_required: z.boolean().optional(),
  error: z.string().max(300).optional(),
}).strict();

export function createSettingBridge(now = Date.now) {
  const requests = new Map();
  const expire = () => { for (const [id, r] of requests) if (now() - r.expires_at > 120000) requests.delete(id); };
  const status = id => {
    expire(); const r = requests.get(id);
    if (!r) return { status: 'none' };
    if (r.status === 'queued' && r.expires_at <= now()) r.status = 'expired';
    // Old device firmware calls the internal prepare/commit boundary a
    // confirmation. The user already authorizes an exact command; this is not
    // a request for another conversational turn. Device validation is unchanged.
    const result = r.result?.status === 'prepared'
      ? { ...r.result, confirmation_required: false, commit_required: true }
      : r.result;
    return { status: r.status, request_id: r.id, ...(result ? { result } : {}) };
  };
  return {
    status,
    queue(owner, operation, fields) {
      const previous = status(owner), fingerprint = JSON.stringify({ operation, ...fields });
      if (previous.status === 'queued') {
        if (requests.get(owner).fingerprint !== fingerprint) throw new Error('A setting request is pending. Check its result before another request.');
        return previous;
      }
      if (!requests.has(owner) && requests.size >= 1000) throw new Error('Setting request capacity reached');
      requests.set(owner, { ...fields, operation, id: randomBytes(32).toString('base64url'), expires_at: now() + 30000,
        status: 'queued', fingerprint });
      return status(owner);
    },
    poll(owner) {
      if (status(owner).status !== 'queued') return null;
      const { status: ignored, fingerprint, result, ...command } = requests.get(owner);
      return command;
    },
    ack(owner, requestId, raw) {
      const state = status(owner), r = requests.get(owner);
      if (!r || r.id !== requestId) throw new Error('Unknown setting request');
      if (state.status !== 'queued') return state;
      const result = settingResult.parse(raw);
      const expected = { read: 'read', prepare: 'prepared', commit: 'applied', cancel: 'cancelled' }[r.operation];
      if (result.status !== 'failed' && (result.status !== expected || (r.name && result.name !== r.name))) throw new Error('Unexpected setting result');
      if (['read', 'prepared', 'applied'].includes(result.status) && result.current === undefined) throw new Error('Current value required');
      if (result.status === 'prepared' && (!result.proposal_id || result.proposed === undefined || result.confirmation_required !== true)) throw new Error('Complete proposal required');
      r.result = result; r.status = 'completed'; return status(owner);
    },
    remove(owner) { requests.delete(owner); },
  };
}
