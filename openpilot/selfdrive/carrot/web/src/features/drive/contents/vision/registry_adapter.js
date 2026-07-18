"use strict";

import {
  getOrCreateDriveVisionContent,
  VISION_CONTENT_ID,
} from "./content.js";
import {
  finalizeDriveVisionFacade,
  getDriveVisionFacade,
} from "./facade.js";
import {
  getDriveVisionHudContent,
  installDriveVisionHudContentFacade,
} from "./hud_content.js";

export const VISION_SUPPORTED_SLOTS = Object.freeze(["primary", "secondary"]);
export const VISION_SUPPORTED_SOURCES = Object.freeze(["live", "replay"]);

export function createVisionContentDescriptor(options = {}) {
  return Object.freeze({
    id: VISION_CONTENT_ID,
    labelKey: "web_drive_layout_content_vision",
    factory(context = {}) {
      const target = options.target || globalThis;
      const renderer = options.renderer
        || getDriveVisionFacade()
        || finalizeDriveVisionFacade(target, options.facadeOptions || {});
      const hud = options.hud
        || getDriveVisionHudContent(target)
        || installDriveVisionHudContentFacade(target, options.hudOptions || {});
      if (!renderer || !hud) return null;
      return getOrCreateDriveVisionContent({
        ...options,
        target,
        context,
        slot: context.slot,
        renderer,
        hud,
      });
    },
    supportedSlots: VISION_SUPPORTED_SLOTS,
    supportedSources: VISION_SUPPORTED_SOURCES,
    singleton: true,
  });
}

export const visionContentDescriptor = createVisionContentDescriptor();
