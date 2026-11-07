import { randomBytes } from 'node:crypto';
import { z } from 'zod';

export const driveResult = z.object({
  id: z.string().regex(/^[A-Za-z0-9_-]{43}$/),
  status: z.enum(['waiting', 'running', 'applied', 'unchanged', 'completed', 'rejected', 'expired', 'interrupted', 'unverified', 'cancelled']),
  direction: z.enum(['left', 'right']).optional(), reason: z.string().max(240).optional(),
  previous: z.number().finite().optional(), current: z.number().finite().optional(),
}).strict();
const terminal = r => r && !['waiting', 'running'].includes(r.status);

export function createDriveBridge(now = Date.now) {
  const requests = new Map();
  function get(owner) {
    const r = requests.get(owner);
    if (!r) return null;
    if ((!r.result && now() >= r.expires_at) || (r.result && !terminal(r.result) && now() - r.created > 20000)) {
      r.result = { id: r.id, status: r.result ? 'unverified' : 'expired', reason: 'delivery_or_feedback_expired' };
    }
    return r;
  }
  return {
    queue(owner, fields) {
      for (const [key, value] of requests) if (now() - value.created > 120000) requests.delete(key);
      const previous = get(owner);
      if (previous && (!terminal(previous.result) || now() - previous.created < 10000)) {
        if (previous.action === fields.action && previous.value === fields.value) {
          return { ...this.status(owner), submitted: false, reason: 'duplicate', retry_allowed: false,
            message: '같은 요청이 이미 접수됐어요.' };
        }
        return { status: 'blocked', submitted: false, reason: 'active_or_recent_request', retry_allowed: false,
          message: '직전 운전 요청이 있어 이번 요청은 접수하지 않았어요.' };
      }
      if (!requests.has(owner) && requests.size >= 1000) throw new Error('Driving request capacity reached');
      requests.set(owner, { ...fields, id: randomBytes(32).toString('base64url'), created: now(), expires_at: now() + 5000 });
      return this.status(owner);
    },
    status(owner) {
      const r = get(owner);
      return r ? { status: terminal(r.result) ? 'completed' : r.result ? 'active' : 'queued', request_id: r.id, ...(r.result ? { result: r.result } : {}) }
        : { status: 'none' };
    },
    poll(owner) {
      const r = get(owner);
      if (!r || terminal(r.result)) return null;
      return { id: r.id, action: r.action, ...(r.value !== undefined ? { value: r.value } : {}),
        operation: r.result ? 'status' : 'execute', expires_at: r.result ? now() + 5000 : r.expires_at };
    },
    ack(owner, identity, raw) {
      const r = get(owner);
      if (!r || r.id !== identity) throw new Error('Unknown driving request');
      if (terminal(r.result)) return this.status(owner);
      const result = driveResult.parse(raw);
      if (result.id !== identity) throw new Error('Mismatched driving request');
      if (r.action.startsWith('lane_')) {
        if (['applied', 'unchanged'].includes(result.status) || (result.direction && r.action !== `lane_${result.direction}`)) throw new Error('Invalid lane result');
        if (result.status === 'completed' && !result.direction) throw new Error('Completed lane direction required');
      } else {
        if (['completed', 'running'].includes(result.status)) throw new Error('Invalid speed result');
        if (['applied', 'unchanged'].includes(result.status) && result.current === undefined) throw new Error('Verified set speed required');
      }
      r.result = result;
      return this.status(owner);
    },
    remove(owner) { requests.delete(owner); },
  };
}
