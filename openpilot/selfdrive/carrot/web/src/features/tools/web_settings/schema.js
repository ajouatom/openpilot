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
      {
        id: "vision_ar_enabled",
        titleKey: "web_vision_ar_enabled",
        descKey: "web_vision_ar_enabled_desc",
      },
      {
        id: "vision_ar_debug",
        titleKey: "web_vision_ar_debug",
        descKey: "web_vision_ar_debug_desc",
      },
    ],
  },
  {
    id: "navigation",
    labelKey: "web_settings_navigation",
    items: [
      {
        id: "carrot_navi_fullscreen_on_tap",
        titleKey: "web_navi_fullscreen_on_tap",
        descKey: "web_navi_fullscreen_on_tap_desc",
      },
      {
        id: "carrot_navi_map_appearance",
        component: "param-selects",
        fields: [
          {
            id: "carrot_navi_map_type",
            paramName: "ClusterNaviMapType",
            titleKey: "web_navi_map_type",
            descKey: "web_navi_map_type_desc",
            options: [
              { value: "0", labelKey: "web_navi_map_type_normal" },
              { value: "1", labelKey: "web_navi_map_type_satellite" },
            ],
          },
          {
            id: "carrot_navi_map_theme",
            paramName: "ClusterNaviMapTheme",
            titleKey: "web_navi_map_theme",
            descKey: "web_navi_map_theme_desc",
            options: [
              { value: "0", labelKey: "web_navi_map_theme_auto" },
              { value: "1", labelKey: "web_navi_map_theme_dark" },
              { value: "2", labelKey: "web_navi_map_theme_light" },
            ],
          },
        ],
      },
    ],
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
