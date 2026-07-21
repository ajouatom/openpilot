/**
 * Temporary performance-isolation switches.
 *
 * Set a surface back to true to restore it. These flags disable runtime work,
 * not just CSS visibility, so they are suitable for an A/B driving test.
 */
export const TEMPORARY_TELEMETRY_SURFACES = Object.freeze({
  driveInsights: true,
  replayGraphs: true,
  replayForward: true,
});
