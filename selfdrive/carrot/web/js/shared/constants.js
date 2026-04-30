"use strict";

const DEBUG_UI = false;

const LANG_STORAGE_KEY = "carrot_web_lang";
const LANG_EMOJI = {
  ko: "🇰🇷",
  en: "🇺🇸",
  zh: "🇨🇳",
  ja: "🇯🇵",
  fr: "🇫🇷",
};

let UNIT_CYCLE = [1, 2, 5, 10, 50, 100];
const UNIT_INDEX = {}; // per name

const PAGE_TRANSITION_CLASSES = [
  "page-transitioning",
  "page-active",
  "page-enter-from-right",
  "page-enter-from-left",
  "page-exit-to-left",
  "page-exit-to-right",
];
const PAGE_TRANSITION_MS = 280;
const SWIPE_SETTLE_MS = 220;
const SWIPE_COMMIT_RATIO = 0.22;
const SWIPE_VELOCITY_THRESHOLD = 0.45;
const SWIPE_EDGE_RESISTANCE = 0.18;

const SWIPE_PAGES = ["carrot", "setting", "tools", "logs", "terminal"];
const SETTING_BACK_EDGE_WIDTH = 32;

const QUICK_LINK_FIXED_URL = "https://man.carrotpilot.app/";
