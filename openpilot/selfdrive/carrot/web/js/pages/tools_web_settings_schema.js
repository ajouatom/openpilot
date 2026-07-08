"use strict";

// Web settings UI schema — groups, rows, control widget per row, and option
// labels. This is presentation only: the value TYPE, DEFAULT and validation
// live in the backend spec (services/web_settings.py) and arrive through
// window.__CARROT_BOOTSTRAP__.webSettingsSpec (see tools_web_settings_state.js).
// Adding a setting means one backend spec entry + one row here.

const WEB_SETTINGS_GROUPS = [
  {
    id: "general",
    labelKey: "web_settings_general",
    defaultLabel: "General",
    items: [
      {
        id: "auto_update_git_pull",
        type: "toggle",
        titleKey: "web_auto_update",
        defaultTitle: "Auto update",
        descKey: "web_auto_update_desc",
        defaultDesc: "Automatically run git pull when updates are available. This will not reboot.",
      },
    ],
  },
  {
    id: "display",
    labelKey: "web_settings_display",
    defaultLabel: "Display",
    items: [
      {
        id: "start_page",
        type: "select",
        titleKey: "web_start_page",
        defaultTitle: "Start menu",
        descKey: "web_start_page_desc",
        defaultDesc: "Choose which menu opens first when Carrot Web loads.",
        options: [
          { value: "last", labelKey: "web_start_page_last", defaultLabel: "Last tab" },
          { value: "carrot", labelKey: "home", defaultLabel: "Drive" },
          { value: "setting", labelKey: "setting", defaultLabel: "Setting" },
          { value: "tools", labelKey: "tools", defaultLabel: "Tools" },
          { value: "logs", labelKey: "logs", defaultLabel: "Logs" },
          { value: "terminal", labelKey: "terminal", defaultLabel: "Terminal" },
        ],
      },
      {
        id: "mini_hud_enabled",
        type: "toggle",
        titleKey: "web_mini_hud_enabled",
        defaultTitle: "Mini HUD",
        descKey: "web_mini_hud_enabled_desc",
        defaultDesc: "Show a compact driving HUD on narrow or multi-window layouts.",
      },
    ],
  },
  {
    id: "vision",
    labelKey: "web_settings_carrot_vision",
    defaultLabel: "Carrot Vision",
    items: [
      {
        id: "vision_fullscreen_default",
        type: "toggle",
        titleKey: "web_vision_fullscreen_default",
        defaultTitle: "Vision fullscreen",
        descKey: "web_vision_fullscreen_default_desc",
        defaultDesc: "Automatically enter fullscreen when Carrot Vision starts.",
      },
      {
        id: "kmap_enabled",
        type: "toggle",
        titleKey: "web_kmap_enabled",
        defaultTitle: "Carrot map",
        descKey: "web_kmap_enabled_desc",
        defaultDesc: "Show the kmap iframe on the drive vision screen.",
      },
      {
        id: "kmap_overlay_heading_up",
        type: "toggle",
        titleKey: "web_kmap_heading_up",
        defaultTitle: "Heading-up path",
        descKey: "web_kmap_heading_up_desc",
        defaultDesc: "Draw the local path relative to the vehicle heading.",
      },
      {
        id: "kmap_overlay_curvature_color",
        type: "toggle",
        titleKey: "web_kmap_curvature_color",
        defaultTitle: "Curve color",
        descKey: "web_kmap_curvature_color_desc",
        defaultDesc: "Tint local path segments more strongly on sharper bends.",
      },
      {
        id: "kmap_map_type",
        type: "select",
        titleKey: "web_kmap_map_type",
        defaultTitle: "Map type",
        descKey: "web_kmap_map_type_desc",
        defaultDesc: "Kakao base layer for the Carrot map (reloads the map on change).",
        options: [
          { value: "roadmap", labelKey: "web_kmap_map_type_roadmap", defaultLabel: "Roadmap" },
          { value: "satellite", labelKey: "web_kmap_map_type_satellite", defaultLabel: "Satellite" },
          { value: "hybrid", labelKey: "web_kmap_map_type_hybrid", defaultLabel: "Hybrid" },
        ],
      },
      {
        id: "nav_hud_enabled",
        type: "toggle",
        titleKey: "web_nav_hud_enabled",
        defaultTitle: "Nav HUD",
        descKey: "web_nav_hud_enabled_desc",
        defaultDesc: "Show the small turn-by-turn card at the top of Carrot Vision.",
      },
    ],
  },
  {
    id: "log_upload",
    labelKey: "web_settings_log_upload",
    defaultLabel: "Log upload",
    items: [
      {
        id: "log_upload_target",
        type: "select",
        titleKey: "web_log_upload_target",
        defaultTitle: "Upload server",
        descKey: "web_log_upload_target_desc",
        defaultDesc: "Where dashcam log uploads are sent.",
        options: [
          { value: "carrot", labelKey: "web_log_upload_target_carrot", defaultLabel: "Carrot server" },
          { value: "toss", labelKey: "web_log_upload_target_toss", defaultLabel: "Toss server" },
        ],
      },
      {
        id: "toss_upload_url",
        type: "text",
        titleKey: "web_toss_upload_url",
        defaultTitle: "Toss server URL",
        descKey: "web_toss_upload_url_desc",
        defaultDesc: "HTTPS upload API address, e.g. https://op.wjcloud.kr",
        placeholder: "https://",
      },
      {
        id: "toss_upload_token",
        type: "password",
        titleKey: "web_toss_upload_token",
        defaultTitle: "Toss access token",
        descKey: "web_toss_upload_token_desc",
        defaultDesc: "Bearer token issued by the toss server operator.",
      },
      {
        id: "toss_connection_test",
        type: "action",
        titleKey: "web_toss_connection_test",
        defaultTitle: "Connection test",
        descKey: "web_toss_connection_test_desc",
        defaultDesc: "Check the saved URL and token against the toss server.",
        buttonKey: "web_toss_connection_test_button",
        defaultButton: "Test",
        enabledWhen: () => getWebSettingByKey("log_upload_target", "carrot") === "toss",
        disabledHintKey: "web_toss_test_requires_toss",
        defaultDisabledHint: "Available when the toss server is selected as the upload target.",
      },
    ],
  },
];

window.WEB_SETTINGS_GROUPS = WEB_SETTINGS_GROUPS;
