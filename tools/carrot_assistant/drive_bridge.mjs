import { randomBytes } from 'node:crypto';
import { z } from 'zod';

export const driveResult = z.object({
  id: z.string().regex(/^[A-Za-z0-9_-]{43}$/),
  status: z.enum(['waiting', 'running', 'applied', 'unchanged', 'completed', 'rejected', 'expired', 'interrupted', 'unverified', 'cancelled']),
  direction: z.enum(['left', 'right']).optional(), reason: z.string().max(240).optional(),
  previous: z.number().finite().optional(), current: z.number().finite().optional(),
}).strict();
export const speedReading = z.discriminatedUnion('status', [
  z.object({ status: z.literal('ok'), vehicle_speed_kph: z.number().finite().nullable(),
    set_speed_kph: z.number().finite().min(0).lt(255).nullable(), sample_age_ms: z.number().int().min(0).max(1000) }).strict()
    .refine(r => r.vehicle_speed_kph !== null || r.set_speed_kph !== null),
  z.object({ status: z.literal('unavailable'), reason: z.enum(['no_fresh_data', 'device_permission', 'connection_failed']) }).strict(),
]);
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
      const recent = (previous?.recent || []).filter(entry => now() - entry.created < 10000);
      const same = entry => entry.action === fields.action && entry.value === fields.value;
      const duplicate = recent.find(same) || (previous && !terminal(previous.result) && same(previous) ? previous : null);
      if (duplicate) {
        return { status: 'blocked', request_id: duplicate.id, submitted: false, reason: 'duplicate', retry_allowed: false,
          message: '같은 요청이 이미 접수됐어.' };
      }
      if (previous && !terminal(previous.result)) {
        return { status: 'blocked', submitted: false, reason: 'request_in_progress', retry_allowed: false,
          message: '앞선 운전 요청이 진행 중이야.' };
      }
      if (previous && ['unverified', 'expired', 'interrupted'].includes(previous.result.status) && now() - previous.created < 10000) {
        return { status: 'blocked', submitted: false, reason: 'previous_outcome_uncertain', retry_allowed: false,
          message: '앞선 요청의 종료를 확인하지 못했어.' };
      }
      if (!requests.has(owner) && requests.size >= 1000) throw new Error('Driving request capacity reached');
      const entry = { ...fields, id: randomBytes(32).toString('base64url'), created: now() };
      recent.push(entry);
      requests.set(owner, { ...entry, recent, expires_at: entry.created + 5000 });
      return this.status(owner);
    },
    status(owner) {
      const r = get(owner);
      return r ? { status: terminal(r.result) ? 'completed' : r.result ? 'active' : 'queued', request_id: r.id,
        timing_ms: { ...(r.dispatched !== undefined && { queue_to_browser_poll: r.dispatched - r.created }),
          ...(r.firstAck !== undefined && r.dispatched !== undefined && { poll_to_first_ack: r.firstAck - r.dispatched }),
          ...(r.deviceRoundTripMs !== undefined && { first_local_http: r.deviceRoundTripMs }) },
        ...(r.result ? { result: r.result } : {}) }
        : { status: 'none' };
    },
    hasPending(owner) {
      const r = get(owner);
      return Boolean(r && !terminal(r.result));
    },
    poll(owner) {
      const r = get(owner);
      if (!r || terminal(r.result)) return null;
      r.dispatched ??= now();
      return { id: r.id, action: r.action, ...(r.value !== undefined ? { value: r.value } : {}),
        operation: r.result ? 'status' : 'execute', expires_at: r.result ? now() + 5000 : r.expires_at };
    },
    ack(owner, identity, raw, deviceRoundTripMs) {
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
      r.firstAck ??= now();
      if (r.deviceRoundTripMs === undefined && Number.isFinite(deviceRoundTripMs) && deviceRoundTripMs >= 0 && deviceRoundTripMs < 10000) {
        r.deviceRoundTripMs = deviceRoundTripMs;
      }
      r.result = result;
      return this.status(owner);
    },
    remove(owner) { requests.delete(owner); },
  };
}
