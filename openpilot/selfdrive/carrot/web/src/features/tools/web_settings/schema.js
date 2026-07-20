"use strict";

// Web settings presentation schema. Value type, default, choices, and
// validation come only from services/web_settings.py. This file owns grouping,
// ordering, and translation keys for settings that should be user-visible.

const WEB_SETTINGS_GROUPS = [
  {
    id: "general",
    labelKey: "web_settings_general",
    items: [
      {
        id: "auto_update_git_pull",
        titleKey: "web_auto_update",
        descKey: "web_auto_update_desc",
      },
      {
        id: "start_page",
        titleKey: "web_start_page",
        descKey: "web_start_page_desc",
        options: [
          { value: "last", labelKey: "web_start_page_last" },
          { value: "carrot", labelKey: "home" },
          { value: "setting", labelKey: "setting" },
          { value: "tools", labelKey: "tools" },
          { value: "logs", labelKey: "logs" },
          { value: "terminal", labelKey: "terminal" },
        ],
      },
    ],
  },
  {
    id: "hud",
    labelKey: "web_settings_hud",
    items: [
      {
        id: "mini_hud_enabled",
        titleKey: "web_mini_hud_enabled",
        descKey: "web_mini_hud_enabled_desc",
      },
    ],
  },
  {
    id: "layout",
    labelKey: "web_settings_drive_layout",
    items: [
      {
        id: "carrot_navi_layout",
        component: "drive-layout",
      },
    ],
  },
  {
    id: "vision",
    labelKey: "web_settings_carrot_vision",
    items: [
      {
        id: "vision_fullscreen_default",
        titleKey: "web_vision_fullscreen_default",
        descKey: "web_vision_fullscreen_default_desc",
      },
    ],
  },
  {
    id: "navigation",
    labelKey: "web_settings_navigation",
    // Carrot Navi is always enabled; retain its group as a stable home for
    // navigation-specific settings added here later.
    keepWhenEmpty: true,
    items: [],
  },
  {
    id: "upload",
    labelKey: "web_settings_upload",
    items: [
      {
        id: "web_upload",
        component: "web-upload",
      },
    ],
  },
];

window.WEB_SETTINGS_GROUPS = WEB_SETTINGS_GROUPS;

export { WEB_SETTINGS_GROUPS };
