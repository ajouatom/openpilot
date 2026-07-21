import { createLatestOnlyRenderScheduler } from "../../../telemetry/render_scheduler.js";

export const DRIVE_INSIGHTS_RENDER_CADENCE_MS = Object.freeze({
  graph: 100,
  forward: 67,
});

/**
 * Low-priority latest-only scheduler for auxiliary drive surfaces.
 *
 * Drive Insights must never paint inside the compact-state delivery stack:
 * that stack also schedules the camera overlay. A timer boundary both yields
 * to the vision scheduler and coalesces a service burst into one latest paint.
 */
export function createDriveInsightsRenderScheduler(options = {}) {
  return createLatestOnlyRenderScheduler({
    cadenceMs: DRIVE_INSIGHTS_RENDER_CADENCE_MS.graph,
    ...options,
  });
}
