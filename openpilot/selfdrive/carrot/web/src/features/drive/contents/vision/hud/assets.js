"use strict";

// The web server exposes the existing selfdrive assets read-only. Reusing the
// same files keeps the cluster and web HUD visually identical without copies.
const ROOT = "/shared-assets";

const CARROT_URI = `${ROOT}/icons_mici/carrot_wheel_org.png`;
const LFA_LANE_URI = `${ROOT}/icons_mici/carrot_wheel_lane.png`;
const WIFI_URI = `${ROOT}/icons_mici/settings/network/wifi_strength_full.png`;
const SPEED_BG_URI = `${ROOT}/images/speed_bg.png`;
const TRAFFIC_RED_URI = `${ROOT}/images/traffic_red.png`;
const TRAFFIC_GREEN_URI = `${ROOT}/images/traffic_green.png`;
const KAIGEN_FONT_URI = `${ROOT}/fonts/KaiGenGothicKR-Bold.ttf`;

export {
  CARROT_URI,
  LFA_LANE_URI,
  KAIGEN_FONT_URI,
  SPEED_BG_URI,
  TRAFFIC_GREEN_URI,
  TRAFFIC_RED_URI,
  WIFI_URI,
};
