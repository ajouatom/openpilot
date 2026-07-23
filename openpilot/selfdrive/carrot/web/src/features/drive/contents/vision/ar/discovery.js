/* Spatial event discovery policy.
 *
 * Discovery answers "should this real Navi event exist yet?". It is separate
 * from visual emphasis, which may still move APPROACH/PRECISE boundaries with
 * speed. Keeping this range independent prevents a stopped/slow vehicle from
 * shrinking a 300 m preview horizon to 180 m.
 */

export const AR_DISCOVERY_POLICY = Object.freeze({
  minimumRangeM: 200,
  defaultRangeM: 300,
  maximumRangeM: 300,
});

function finite(value, fallback = null) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

export function discoveryRangeM(requestedRangeM = AR_DISCOVERY_POLICY.defaultRangeM) {
  const requested = finite(requestedRangeM, AR_DISCOVERY_POLICY.defaultRangeM);
  return Math.min(
    AR_DISCOVERY_POLICY.maximumRangeM,
    Math.max(AR_DISCOVERY_POLICY.minimumRangeM, requested),
  );
}

export function isWithinDiscoveryRange(distanceM, requestedRangeM) {
  const distance = finite(distanceM);
  return distance !== null && distance >= 0 && distance <= discoveryRangeM(requestedRangeM);
}
