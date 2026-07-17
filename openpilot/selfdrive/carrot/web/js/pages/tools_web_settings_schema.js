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
    items: [
      {
        id: "carrot_navi_enabled",
        titleKey: "web_carrot_navi_enabled",
        descKey: "web_carrot_navi_enabled_desc",
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
