import { adoptTelemetryGraphSurface } from "../telemetry/graph_surface.js";
import { createLatestOnlyRenderScheduler } from "../telemetry/render_scheduler.js";
import { TEMPORARY_TELEMETRY_SURFACES } from "../telemetry/temporary_surface_flags.js";
import { createSegmentedControl } from "../../ui/components/segmented_control/segmented_control.js";
import {
  isUserVisibleReplayEvent,
  navigationManeuverLabel,
  replayEventDisplayTitle,
} from "./navigation_event.js";

"use strict";

window.CarrotReplayInsights = window.CarrotReplayInsights || (() => {
  const stageEl = document.getElementById("carrotStage");
  const auxPaneEl = document.getElementById("carrotDriveAuxPane");
  const rootEl = document.getElementById("carrotReplayInsights");
  const railEl = document.getElementById("carrotReplayEventRail");
  const seekEl = document.getElementById("carrotReplaySeek");
  const summaryEl = document.getElementById("carrotReplayEventSummary");
  const summaryIconEl = document.getElementById("carrotReplayEventSummaryIcon");
  const summaryTitleEl = document.getElementById("carrotReplayEventSummaryTitle");
  const summaryMetaEl = document.getElementById("carrotReplayEventSummaryMeta");
  const clusterEl = document.getElementById("carrotReplayEventCluster");
  const clusterTitleEl = document.getElementById("carrotReplayEventClusterTitle");
  const clusterItemsEl = document.getElementById("carrotReplayEventClusterItems");
  const listEl = document.getElementById("carrotReplayEventList");
  const emptyEl = document.getElementById("carrotReplayEventEmpty");
  const followButtonEl = document.getElementById("btnCarrotReplayEventFollow");
  const degradedEl = document.getElementById("carrotReplayEventDegraded");
  const recordCountEl = document.getElementById("carrotReplayInsightsRecordCount");
  const serviceCountEl = document.getElementById("carrotReplayInsightsServiceCount");
  const eventCountEl = document.getElementById("carrotReplayInsightsEventCount");
  const graphCountEl = document.getElementById("carrotReplayInsightsGraphCount");
  const invalidCountEl = document.getElementById("carrotReplayInsightsInvalidCount");
  const replaySurfaceEnabled = (name) => (
    name !== "graphs" || TEMPORARY_TELEMETRY_SURFACES.replayGraphs
  ) && (
    name !== "sensors" || TEMPORARY_TELEMETRY_SURFACES.replayForward
  );
  const allTabEls = Array.from(document.querySelectorAll("[data-replay-insights-tab]"));
  const allPanelEls = Array.from(document.querySelectorAll("[data-replay-insights-panel]"));
  const tabEls = allTabEls.filter((element) => replaySurfaceEnabled(element.dataset.replayInsightsTab));
  const panelEls = allPanelEls.filter((element) => replaySurfaceEnabled(element.dataset.replayInsightsPanel));
  for (const element of allTabEls) {
    if (replaySurfaceEnabled(element.dataset.replayInsightsTab)) continue;
    element.hidden = true;
    element.setAttribute("aria-hidden", "true");
  }
  for (const element of allPanelEls) {
    if (replaySurfaceEnabled(element.dataset.replayInsightsPanel)) continue;
    element.hidden = true;
    element.setAttribute("aria-hidden", "true");
  }
  const graphEls = new Map(Array.from(document.querySelectorAll("[data-replay-graph]")).map((element) => (
    [element.dataset.replayGraph, element]
  )));
  const graphUi = new Map(Array.from(graphEls, ([name, element]) => [name, {
    cursor: element.querySelector(".carrot-replay-insights__cursor"),
    output: element.querySelector("output"),
  }]));
  adoptTelemetryGraphSurface(
    rootEl?.querySelector?.(".carrot-replay-insights__graphs"),
    Array.from(graphEls.values()),
  );
  const advancedServiceEl = document.getElementById("carrotReplayAdvancedService");
  const advancedFieldEl = document.getElementById("carrotReplayAdvancedField");
  const advancedStatusEl = document.getElementById("carrotReplayAdvancedStatus");
  const advancedTechnicalPathEl = document.getElementById("carrotReplayAdvancedTechnicalPath");
  const advancedTrackEl = document.getElementById("carrotReplayAdvancedTrack");
  const advancedCanvasEl = document.getElementById("carrotReplayAdvancedCanvas");
  const advancedCursorEl = document.getElementById("carrotReplayAdvancedCursor");
  const advancedCurrentEl = document.getElementById("carrotReplayAdvancedCurrent");
  const advancedMinEl = document.getElementById("carrotReplayAdvancedMin");
  const advancedMaxEl = document.getElementById("carrotReplayAdvancedMax");
  const advancedAverageEl = document.getElementById("carrotReplayAdvancedAverage");
  const advancedStatisticsEl = document.getElementById("carrotReplayAdvancedStatistics");
  const advancedStatisticsDetailsEl = document.getElementById("carrotReplayAdvancedStatisticsDetails");
  const advancedSourceDetailsEl = document.getElementById("carrotReplayAdvancedSourceDetails");
  const advancedCatalogEl = document.getElementById("carrotReplayAdvancedCatalog");
  const advancedSnapshotButtonEl = document.getElementById("btnCarrotReplayAdvancedSnapshot");
  const advancedSnapshotEl = document.getElementById("carrotReplayAdvancedSnapshot");

  const model = window.CarrotReplayInsightsModel;
  const dataTracks = window.CarrotReplayInsightsData;
  const rendererRegistry = window.CarrotReplayRendererRegistry;
  const sensorTopview = window.CarrotReplaySensorTopview;
  const TAB_SETTING_KEY = "replay_insights_tab";
  const TAB_NAMES = new Set(
    ["events", "graphs", "sensors", "advanced"].filter(replaySurfaceEnabled),
  );
  const POLICY = Object.freeze({
    indexChunkSize: 220,
    graphPointLimit: 560,
    summaryWindowSeconds: 2.4,
    // Wall-clock, not content time: this window exists to stabilise what the eye
    // sees, so it must not shrink to 40 ms just because playback is at 4x.
    eventHighlightSettleMs: 160,
    eventSeekJumpSeconds: 0.45,
    eventReviewLeadSeconds: 2,
    rawQueryDebounceMs: 140,
    shortLayoutMaxHeight: 560,
    eventScrollContextMaxItems: 2,
    eventScrollSafeInsetPx: 12,
    // Replay video presentation owns the frame clock. Detail canvases consume
    // the latest playback time independently and never run in that hot path.
    playbackSurfaceCadenceMs: 50,
  });
  const CATEGORY_META = Object.freeze({
    control: { labelKey: "replay_category_control", fallback: "Control" },
    driver: { labelKey: "replay_category_driver", fallback: "Driver" },
    vehicle: { labelKey: "replay_category_vehicle", fallback: "Vehicle" },
    carrot: { labelKey: "replay_category_carrot", fallback: "CarrotPilot" },
    warning: { labelKey: "replay_category_warning", fallback: "Warning" },
    turn: { labelKey: "replay_category_lane_change", fallback: "Lane change" },
    nav: { labelKey: "replay_category_navigation", fallback: "Navigation" },
  });
  const CATEGORY_COLOR_VAR = Object.freeze({
    control: "--carrot-replay-insights-control",
    driver: "--carrot-replay-insights-driver",
    vehicle: "--carrot-replay-insights-vehicle",
    carrot: "--carrot-replay-insights-carrot",
    warning: "--carrot-replay-insights-warning",
    turn: "--carrot-replay-insights-turn",
    nav: "--carrot-replay-insights-nav",
  });
  const EVENT_TITLE_META = Object.freeze({
    control_engaged: ["replay_event_control_engaged", "Driving control engaged"],
    control_disengaged: ["replay_event_control_disengaged", "Driving control disengaged"],
    lateral_control_on: ["replay_event_lateral_control_on", "Lateral control started"],
    lateral_control_off: ["replay_event_lateral_control_off", "Lateral control ended"],
    longitudinal_control_on: ["replay_event_longitudinal_control_on", "Longitudinal control started"],
    longitudinal_control_off: ["replay_event_longitudinal_control_off", "Longitudinal control ended"],
    control_override_start: ["replay_event_control_override_start", "Driver override started"],
    control_override_end: ["replay_event_control_override_end", "Driver override ended"],
    experimental_mode_on: ["replay_event_experimental_mode_on", "Experimental mode enabled"],
    experimental_mode_off: ["replay_event_experimental_mode_off", "Experimental mode disabled"],
    control_available: ["replay_event_control_available", "Driving control available"],
    control_unavailable: ["replay_event_control_unavailable", "Driving control unavailable"],
    driving_personality_changed: ["replay_event_personality_changed", "Driving personality changed"],
    cruise_enabled: ["replay_event_cruise_enabled", "Cruise control enabled"],
    cruise_disabled: ["replay_event_cruise_disabled", "Cruise control disabled"],
    cruise_cancel_requested: ["replay_event_cruise_cancel", "Cruise cancel requested"],
    cruise_resume_requested: ["replay_event_cruise_resume", "Cruise resume requested"],
    accelerator_pressed: ["replay_event_accelerator_pressed", "Accelerator pressed"],
    accelerator_released: ["replay_event_accelerator_released", "Accelerator released"],
    brake_pressed: ["replay_event_brake_pressed", "Brake pressed"],
    brake_released: ["replay_event_brake_released", "Brake released"],
    steering_override_start: ["replay_event_steering_override_start", "Steering override started"],
    steering_override_end: ["replay_event_steering_override_end", "Steering override ended"],
    left_blinker_on: ["replay_event_left_blinker", "Left turn signal"],
    right_blinker_on: ["replay_event_right_blinker", "Right turn signal"],
    gear_changed: ["replay_event_gear_changed", "Gear changed"],
    vehicle_stopped: ["replay_event_vehicle_stopped", "Vehicle stopped"],
    vehicle_moved: ["replay_event_vehicle_moved", "Vehicle moved"],
    brake_hold_on: ["replay_event_brake_hold_on", "Brake hold engaged"],
    brake_hold_off: ["replay_event_brake_hold_off", "Brake hold released"],
    parking_brake_on: ["replay_event_parking_brake_on", "Parking brake engaged"],
    parking_brake_off: ["replay_event_parking_brake_off", "Parking brake released"],
    clutch_pressed: ["replay_event_clutch_pressed", "Clutch pressed"],
    clutch_released: ["replay_event_clutch_released", "Clutch released"],
    soft_hold_changed: ["replay_event_soft_hold_changed", "Soft hold changed"],
    cruise_gap_changed: ["replay_event_cruise_gap_changed", "Cruise gap changed"],
    vehicle_button_pressed: ["replay_event_vehicle_button", "Vehicle button pressed"],
    enable_button_pressed: ["replay_event_enable_button", "Enable button pressed"],
    lead_acquired: ["replay_event_lead_acquired", "Lead vehicle detected"],
    lead_lost: ["replay_event_lead_lost", "Lead vehicle lost"],
    planned_stop_start: ["replay_event_planned_stop_start", "Planned stop started"],
    planned_stop_end: ["replay_event_planned_stop_end", "Planned stop cleared"],
    longitudinal_stopping: ["replay_event_longitudinal_stopping", "Stopping control"],
    longitudinal_starting: ["replay_event_longitudinal_starting", "Starting control"],
    driving_mode_changed: ["replay_event_driving_mode_changed", "Driving mode changed"],
    carrot_mode_changed: ["replay_event_carrot_mode_changed", "CarrotPilot mode changed"],
    carrot_cruise_changed: ["replay_event_carrot_cruise_changed", "Carrot cruise state changed"],
    carrot_command_received: ["replay_event_carrot_command", "Carrot command received"],
    carrot_speed_control_start: ["replay_event_speed_control_start", "Carrot speed control started"],
    carrot_speed_control_end: ["replay_event_speed_control_end", "Carrot speed control ended"],
    carrot_turn_control_start: ["replay_event_turn_control_start", "Turn speed control started"],
    carrot_turn_control_end: ["replay_event_turn_control_end", "Turn speed control ended"],
    lane_change_left: ["replay_event_lane_change_left", "Lane change left"],
    lane_change_right: ["replay_event_lane_change_right", "Lane change right"],
    lane_change: ["replay_event_lane_change", "Lane change"],
    lane_change_completed: ["replay_event_lane_change_completed", "Lane change completed"],
    system_alert: ["replay_event_system_alert", "System alert"],
    forward_collision_warning: ["replay_event_fcw", "Forward collision warning"],
    stock_aeb: ["replay_event_stock_aeb", "Stock AEB activated"],
    stock_fcw: ["replay_event_stock_fcw", "Stock collision warning"],
    can_timeout: ["replay_event_can_timeout", "Vehicle communication timeout"],
    can_invalid: ["replay_event_can_invalid", "Invalid vehicle communication"],
    steering_fault_temporary: ["replay_event_steering_fault_temporary", "Temporary steering fault"],
    steering_fault_permanent: ["replay_event_steering_fault_permanent", "Steering fault"],
    acc_fault: ["replay_event_acc_fault", "Cruise control fault"],
    vehicle_sensors_invalid: ["replay_event_vehicle_sensors_invalid", "Vehicle sensor error"],
    vehicle_fault: ["replay_event_vehicle_fault", "Vehicle fault detected"],
    stability_control_disabled: ["replay_event_stability_control_disabled", "Stability control disabled"],
    door_open: ["replay_event_door_open", "Door open"],
    seatbelt_unlatched: ["replay_event_seatbelt_unlatched", "Seat belt unlatched"],
    forced_deceleration: ["replay_event_forced_deceleration", "Forced deceleration"],
    low_speed_steering_alert: ["replay_event_low_speed_steering", "Low-speed steering unavailable"],
    left_blindspot_warning: ["replay_event_left_blindspot", "Left blind spot warning"],
    right_blindspot_warning: ["replay_event_right_blindspot", "Right blind spot warning"],
    driver_monitoring_lockout: ["replay_event_driver_lockout", "Driver monitoring lockout"],
    driver_attention_warning: ["replay_event_driver_attention", "Driver attention warning"],
    driver_attention_restored: ["replay_event_driver_attention_restored", "Driver attention restored"],
    driver_distracted: ["replay_event_driver_distracted", "Driver distraction detected"],
    navigation_maneuver: ["replay_event_navigation", "Navigation guidance"],
    navi_connected: ["replay_event_navi_connected", "Navigation connected"],
    navi_disconnected: ["replay_event_navi_disconnected", "Navigation disconnected"],
    navigation_session_changed: ["replay_event_navigation_session_changed", "Route session changed"],
    navigation_active: ["replay_event_navigation_active", "Route guidance active"],
    navigation_ended: ["replay_event_navigation_ended", "Route guidance ended"],
    navigation_off_route: ["replay_event_navigation_off_route", "Off route"],
    navigation_route_recovered: ["replay_event_navigation_route_recovered", "Route recovered"],
    navigation_maneuver_current: ["replay_event_navigation_current", "Current maneuver"],
    navigation_maneuver_next: ["replay_event_navigation_next", "Next maneuver"],
    navigation_maneuver_current_cleared: ["replay_event_navigation_current_cleared", "Current maneuver cleared"],
    navigation_maneuver_next_cleared: ["replay_event_navigation_next_cleared", "Next maneuver cleared"],
    navigation_approach: ["replay_event_navigation_approach", "Approaching maneuver"],
    lane_guidance_shown: ["replay_event_lane_guidance_shown", "Lane guidance shown"],
    lane_guidance_changed: ["replay_event_lane_guidance_changed", "Lane guidance changed"],
    lane_guidance_hidden: ["replay_event_lane_guidance_hidden", "Lane guidance ended"],
    speed_alert_shown: ["replay_event_speed_alert_shown", "Road speed alert"],
    speed_alert_changed: ["replay_event_speed_alert_changed", "Road speed alert changed"],
    speed_alert_cleared: ["replay_event_speed_alert_cleared", "Road speed alert ended"],
    road_speed_limit_changed: ["replay_event_road_speed_limit_changed", "Road speed limit changed"],
    section_control_started: ["replay_event_section_control_started", "Average-speed zone started"],
    section_control_ended: ["replay_event_section_control_ended", "Average-speed zone ended"],
    section_control_suspended: ["replay_event_section_control_suspended", "Average-speed guidance paused"],
    section_control_resumed: ["replay_event_section_control_resumed", "Average-speed guidance resumed"],
    section_control_off_route: ["replay_event_section_control_off_route", "Average-speed route lost"],
    section_control_recovered: ["replay_event_section_control_recovered", "Average-speed route recovered"],
    traffic_signal_shown: ["replay_event_traffic_signal_shown", "Traffic signal guidance shown"],
    traffic_signal_changed: ["replay_event_traffic_signal_changed", "Traffic signal changed"],
    traffic_signal_hidden: ["replay_event_traffic_signal_hidden", "Traffic signal guidance ended"],
    crossroad_guidance_shown: ["replay_event_crossroad_guidance_shown", "Intersection guidance shown"],
    crossroad_guidance_changed: ["replay_event_crossroad_guidance_changed", "Intersection guidance changed"],
    crossroad_guidance_hidden: ["replay_event_crossroad_guidance_hidden", "Intersection guidance ended"],
  });
  const GRAPH_FORMAT = Object.freeze({
    speed: { digits: 1, unit: " km/h" },
    steer: { digits: 2, unit: "°" },
    accel: { digits: 2, unit: " m/s²" },
    lead: { digits: 1, unit: " m" },
  });

  const state = {
    active: false,
    open: false,
    token: 0,
    durationSeconds: 0,
    recordsProcessed: 0,
    indexingProgress: 0,
    indexing: false,
    events: [],
    services: new Set(),
    series: { speed: [], steer: [], accel: [], lead: [] },
    activeTab: "events",
    selectedEventId: "",
    currentEventId: "",
    hoverEventId: "",
    passedEventIndex: -2,
    pendingEventId: "",
    pendingEventSinceMs: 0,
    lastSyncSeconds: NaN,
    previewVisible: false,
    lastAutoScrollEventId: "",
    // Auto-scroll follows playback only while the user is not reading the list.
    followEvents: true,
    expectedScrollTop: null,
    eventSource: "index",
    currentSeconds: 0,
    onSeek: null,
    onPause: null,
    onUpdate: null,
    onReady: null,
    records: [],
    decodeRecord: null,
    rawStats: null,
    rawCatalogStatus: "idle",
    rawRequestController: null,
    snapshotRequestController: null,
    rawQueryTimer: null,
    snapshotToken: 0,
    snapshotCurrent: null,
    advancedCatalog: [],
    advancedTrack: null,
    advancedToken: 0,
    summarySeriesStatus: "idle",
    summaryToken: 0,
    resizeObserver: null,
    layoutMode: "side",
    layoutRaf: 0,
    timelineCanvas: null,
    timelineGroups: [],
  };
  let tabControl = null;
  let pendingTabPreference = "";
  let tabPreferenceWriting = false;
  const playbackSurfaceScheduler = createLatestOnlyRenderScheduler({
    target: window,
    cadenceMs: POLICY.playbackSurfaceCadenceMs,
    onFlush: () => flushSyncedTime(),
  });

  function normalizeTabName(name) {
    const candidate = String(name || "").trim().toLowerCase();
    return TAB_NAMES.has(candidate) ? candidate : "events";
  }

  function preferredTabName() {
    return normalizeTabName(window.getWebSettingByKey?.(TAB_SETTING_KEY, "events"));
  }

  async function flushTabPreference() {
    if (tabPreferenceWriting || !pendingTabPreference) return;
    if (typeof window.setWebSettingByKey !== "function") {
      pendingTabPreference = "";
      return;
    }
    tabPreferenceWriting = true;
    try {
      while (pendingTabPreference) {
        const next = pendingTabPreference;
        pendingTabPreference = "";
        try { await window.setWebSettingByKey(TAB_SETTING_KEY, next); } catch {}
      }
    } finally {
      tabPreferenceWriting = false;
      if (pendingTabPreference) flushTabPreference();
    }
  }

  function persistTabPreference(name) {
    pendingTabPreference = normalizeTabName(name);
    flushTabPreference();
  }

  function text(key, fallback, vars = null) {
    return typeof getUIText === "function" ? getUIText(key, fallback, vars) : fallback;
  }

  function localeName() {
    const language = String(document.documentElement?.lang || navigator.language || "en").toLowerCase();
    if (language.startsWith("ko")) return "ko-KR";
    if (language.startsWith("zh")) return "zh-CN";
    return "en-US";
  }

  function numberText(value, digits = 1) {
    const numeric = Number(value);
    if (!Number.isFinite(numeric)) return "--";
    return new Intl.NumberFormat(localeName(), {
      minimumFractionDigits: digits,
      maximumFractionDigits: digits,
    }).format(numeric);
  }

  function formatTime(seconds) {
    const safe = Math.max(0, Number(seconds) || 0);
    const minutes = Math.floor(safe / 60);
    const remainder = Math.floor(safe % 60);
    return `${minutes}:${String(remainder).padStart(2, "0")}`;
  }

  function formatValue(value, digits, suffix) {
    return Number.isFinite(Number(value)) ? `${numberText(value, digits)}${suffix}` : `--${suffix}`;
  }

  function categoryMeta(category) {
    return CATEGORY_META[category] || { labelKey: "replay_category_event", fallback: "Event" };
  }

  const GEAR_VALUE_META = Object.freeze({
    park: ["replay_value_gear_park", "Park (P)"],
    reverse: ["replay_value_gear_reverse", "Reverse (R)"],
    neutral: ["replay_value_gear_neutral", "Neutral (N)"],
    drive: ["replay_value_gear_drive", "Drive (D)"],
    sport: ["replay_value_gear_sport", "Sport (S)"],
    low: ["replay_value_gear_low", "Low (L)"],
    brake: ["replay_value_gear_brake", "Engine brake (B)"],
    eco: ["replay_value_gear_eco", "Eco (E)"],
    manumatic: ["replay_value_gear_manual", "Manual shift (M)"],
  });
  const SERVICE_LABEL_META = Object.freeze({
    carState: ["replay_data_car_state", "Vehicle state"],
    controlsState: ["replay_data_controls_state", "Driving control"],
    deviceState: ["replay_data_device_state", "Device health"],
    peripheralState: ["replay_data_peripheral_state", "Power supply"],
    carrotMan: ["replay_data_carrot_man", "Road and navigation"],
    selfdriveState: ["replay_data_selfdrive_state", "Driving assistance"],
    gpsLocationExternal: ["replay_data_gps", "GPS location"],
    longitudinalPlan: ["replay_data_longitudinal_plan", "Speed plan"],
    modelV2: ["replay_data_model", "Road perception"],
    liveCalibration: ["replay_data_calibration", "Camera calibration"],
    roadCameraState: ["replay_data_road_camera", "Road camera"],
    lateralPlan: ["replay_data_lateral_plan", "Lane plan"],
    radarState: ["replay_data_radar", "Lead vehicles"],
    carControl: ["replay_data_car_control", "Vehicle commands"],
    liveDelay: ["replay_data_live_delay", "Control delay"],
    liveTorqueParameters: ["replay_data_torque", "Steering learning"],
    liveParameters: ["replay_data_vehicle_parameters", "Vehicle parameters"],
  });
  const FIELD_LABEL_META = Object.freeze({
    "carState.vEgo": ["replay_field_vehicle_speed", "Vehicle speed"],
    "carState.aEgo": ["replay_field_vehicle_acceleration", "Vehicle acceleration"],
    "carState.steeringAngleDeg": ["replay_field_steering_angle", "Steering angle"],
    "carState.gearShifter": ["replay_field_gear", "Gear position"],
    "carState.brakeHoldActive": ["replay_field_brake_hold", "Brake hold"],
    "controlsState.enabled": ["replay_field_control_enabled", "Control active"],
    "controlsState.vCruiseCluster": ["replay_field_set_speed", "Set speed"],
    "controlsState.desiredCurvature": ["replay_field_target_curvature", "Target curvature"],
    "controlsState.actualLateralAccel": ["replay_field_actual_lateral_accel", "Actual lateral acceleration"],
    "controlsState.desiredLateralAccel": ["replay_field_target_lateral_accel", "Target lateral acceleration"],
    "deviceState.memoryUsagePercent": ["replay_field_memory_usage", "Memory usage"],
    "deviceState.freeSpacePercent": ["replay_field_free_space", "Free storage"],
    "deviceState.cpuTempC": ["replay_field_cpu_temperature", "CPU temperature"],
    "peripheralState.voltage": ["replay_field_voltage", "Input voltage"],
    "carrotMan.nRoadLimitSpeed": ["replay_field_road_speed_limit", "Road speed limit"],
    "carrotMan.xSpdLimit": ["replay_field_target_speed_limit", "Upcoming target speed"],
    "carrotMan.xSpdDist": ["replay_field_target_speed_distance", "Distance to speed target"],
    "carrotMan.xDistToTurn": ["replay_field_turn_distance", "Distance to turn"],
    "carrotMan.szPosRoadName": ["replay_field_road_name", "Road name"],
    "carrotMan.desiredSpeed": ["replay_field_desired_speed", "Desired speed"],
    "selfdriveState.enabled": ["replay_field_assistance_enabled", "Assistance active"],
    "selfdriveState.alertType": ["replay_field_alert_type", "Alert type"],
    "gpsLocationExternal.speed": ["replay_field_gps_speed", "GPS speed"],
    "gpsLocationExternal.bearingDeg": ["replay_field_heading", "Heading"],
    "gpsLocationExternal.hasFix": ["replay_field_gps_fix", "GPS fix"],
    "longitudinalPlan.tFollow": ["replay_field_follow_time", "Following time"],
    "longitudinalPlan.desiredDistance": ["replay_field_follow_distance", "Desired following distance"],
    "modelV2.laneLineProbs": ["replay_field_lane_confidence", "Lane confidence"],
    "liveCalibration.calStatus": ["replay_field_calibration_status", "Calibration status"],
    "liveCalibration.calPerc": ["replay_field_calibration_progress", "Calibration progress"],
    "lateralPlan.laneChangeState": ["replay_field_lane_change_state", "Lane change state"],
    "lateralPlan.laneChangeDirection": ["replay_field_lane_change_direction", "Lane change direction"],
    "radarState.leadOne.dRel": ["replay_field_lead_distance", "Lead distance"],
    "radarState.leadOne.vRel": ["replay_field_relative_speed", "Relative speed"],
    "carControl.latActive": ["replay_field_lateral_active", "Steering control active"],
    "carControl.longActive": ["replay_field_longitudinal_active", "Speed control active"],
    "liveParameters.angleOffsetDeg": ["replay_field_steering_offset", "Steering angle offset"],
    "liveParameters.steerRatio": ["replay_field_steer_ratio", "Steering ratio"],
  });
  const FIELD_UNIT_META = Object.freeze({
    "carState.vCruiseCluster": "km/h", "controlsState.vCruiseCluster": "km/h",
    vEgo: "m/s", aEgo: "m/s²", steeringAngleDeg: "°", curvature: "1/m", desiredCurvature: "1/m",
    actualLateralAccel: "m/s²", desiredLateralAccel: "m/s²", memoryUsagePercent: "%", freeSpacePercent: "%",
    cpuTempC: "°C", voltage: "mV", nRoadLimitSpeed: "km/h", xSpdLimit: "km/h", xSpdDist: "m",
    xDistToTurn: "m", desiredSpeed: "km/h", latitude: "°", longitude: "°", speed: "m/s", bearingDeg: "°",
    bearingAccuracyDeg: "°", speedAccuracy: "m/s", tFollow: "s", desiredDistance: "m", dRel: "m", yRel: "m",
    vRel: "m/s", aRel: "m/s²", vLead: "m/s", aLead: "m/s²", lateralDelay: "s", angleOffsetDeg: "°",
  });
  const PERSONALITY_VALUE_META = Object.freeze({
    aggressive: ["replay_value_personality_aggressive", "Responsive"],
    standard: ["replay_value_personality_standard", "Standard"],
    relaxed: ["replay_value_personality_relaxed", "Relaxed"],
    morerelaxed: ["replay_value_personality_more_relaxed", "Very relaxed"],
  });
  const BUTTON_VALUE_META = Object.freeze({
    accelcruise: ["replay_value_button_accel", "Increase set speed"],
    decelcruise: ["replay_value_button_decel", "Decrease set speed"],
    cancel: ["replay_value_button_cancel", "Cancel cruise"],
    resumecruise: ["replay_value_button_resume", "Resume cruise"],
    setcruise: ["replay_value_button_set", "Set cruise"],
    gapadjustcruise: ["replay_value_button_gap", "Adjust following distance"],
    lkas: ["replay_value_button_lkas", "Lane assist"],
  });
  const DRIVING_MODE_META = Object.freeze({
    1: ["replay_value_driving_eco", "Efficiency first"],
    2: ["replay_value_driving_safe", "Safety first"],
    3: ["replay_value_driving_normal", "Balanced"],
    4: ["replay_value_driving_high", "Quick response"],
  });
  const CARROT_MODE_META = Object.freeze({
    0: ["replay_value_carrot_off", "Guidance off"],
    1: ["replay_value_carrot_ready", "Road guidance ready"],
    2: ["replay_value_carrot_road", "Road information guidance"],
    3: ["replay_value_carrot_decel", "Speed reduction guidance"],
    4: ["replay_value_carrot_section", "Average-speed zone guidance"],
    5: ["replay_value_carrot_bump", "Speed bump guidance"],
    6: ["replay_value_carrot_limit", "Speed-limit guidance"],
  });

  function translatedMetaValue(metadata, fallback) {
    return metadata ? text(metadata[0], metadata[1]) : fallback;
  }

  function readableToken(value) {
    return String(value ?? "")
      .replace(/([a-z0-9])([A-Z])/g, "$1 $2")
      .replace(/[_-]+/g, " ")
      .trim();
  }

  function friendlyEventValue(event, value) {
    const key = String(value ?? "").replace(/[\s_-]+/g, "").toLowerCase();
    if (event?.type === "gear_changed") {
      return translatedMetaValue(GEAR_VALUE_META[key], readableToken(value));
    }
    if (event?.type === "driving_personality_changed") {
      return translatedMetaValue(PERSONALITY_VALUE_META[key], readableToken(value));
    }
    if (event?.type === "vehicle_button_pressed") {
      return translatedMetaValue(BUTTON_VALUE_META[key], readableToken(value));
    }
    if (event?.type === "soft_hold_changed") {
      const stateKey = Number(value) >= 2
        ? ["replay_value_hold_active", "Holding the vehicle"]
        : Number(value) === 1
          ? ["replay_value_hold_ready", "Ready to hold"]
          : ["replay_value_hold_off", "Hold released"];
      return translatedMetaValue(stateKey, readableToken(value));
    }
    if (event?.type === "cruise_gap_changed") {
      return text("replay_value_gap_step", "Level {value}", { value: numberText(value, 0) });
    }
    if (event?.type === "driving_mode_changed") {
      return translatedMetaValue(DRIVING_MODE_META[Number(value)], readableToken(value));
    }
    if (event?.type === "carrot_mode_changed") {
      return translatedMetaValue(CARROT_MODE_META[Number(value)], readableToken(value));
    }
    if (event?.type === "carrot_cruise_changed") {
      return Number(value) > 0
        ? text("replay_value_cruise_on", "Cruise active")
        : text("replay_value_cruise_off", "Cruise inactive");
    }
    return readableToken(value);
  }

  function eventTitle(event) {
    if (event?.type === "system_alert") return event.sourceTitle || text("replay_event_system_alert", "Driving alert");
    if (["navigation_maneuver", "navigation_maneuver_current", "navigation_maneuver_next"].includes(event?.type)) {
      return navigationManeuverLabel(event, text)
        || event.sourceTitle
        || text(EVENT_TITLE_META[event.type]?.[0] || "replay_event_navigation", EVENT_TITLE_META[event.type]?.[1] || "Route guidance");
    }
    if (event?.type === "navigation_approach" && Number(event.params?.thresholdM) > 0) {
      return navigationManeuverLabel(event, text) || text("replay_event_navigation_approach_value", "Maneuver within {distance} m", {
        distance: numberText(event.params.thresholdM, 0),
      });
    }
    const value = event?.params?.value;
    const dynamic = {
      gear_changed: ["replay_event_gear_value", "Gear: {value}"],
      driving_personality_changed: ["replay_event_personality_value", "Driving style: {value}"],
      soft_hold_changed: ["replay_event_soft_hold_value", "Stop hold: {value}"],
      cruise_gap_changed: ["replay_event_cruise_gap_value", "Following distance: {value}"],
      driving_mode_changed: ["replay_event_driving_mode_value", "Driving mode: {value}"],
      carrot_mode_changed: ["replay_event_carrot_mode_value", "Road guidance: {value}"],
      carrot_cruise_changed: ["replay_event_carrot_cruise_value", "Carrot cruise: {value}"],
    }[event?.type];
    if (dynamic && value != null) return text(dynamic[0], dynamic[1], { value: friendlyEventValue(event, value) });
    if (event?.type === "vehicle_button_pressed" && event.params?.button) {
      return text("replay_event_button_value", "{value} pressed", {
        value: friendlyEventValue(event, event.params.button),
      });
    }
    const metadata = EVENT_TITLE_META[event?.type];
    if (metadata) return text(metadata[0], metadata[1]);
    return text("replay_event_unknown", "Driving event");
  }

  function eventDetail(event) {
    if (["navigation_maneuver", "navigation_maneuver_current", "navigation_maneuver_next", "navigation_approach"].includes(event?.type)) {
      const distance = Number(event.params?.thresholdM || event.params?.distanceM);
      const distanceDetail = distance > 0
        ? text("replay_event_distance", "{distance} m ahead", { distance: numberText(Math.round(distance), 0) })
        : "";
      const sourceText = [event.sourceTitle, event.sourceDetail]
        .map((value) => String(value || "").trim())
        .filter((value, index, values) => value && values.indexOf(value) === index)
        .join(" · ");
      return [distanceDetail, sourceText].filter(Boolean).join(" · ");
    }
    if (event?.sourceDetail) return event.sourceDetail;
    if (event?.type === "vehicle_button_pressed" && event.params?.button) {
      return "";
    }
    if (event?.params?.from != null && event.params?.value != null) {
      return text("replay_event_changed_from", "Changed from {from}", {
        from: friendlyEventValue(event, event.params.from),
      });
    }
    if (event?.params?.durationMs > 0) {
      return text("replay_event_duration", "{duration} sec", {
        duration: numberText(Number(event.params.durationMs) / 1000, 1),
      });
    }
    const details = {
      planned_stop_start: ["replay_event_planned_stop_detail", "Preparing to stop for traffic ahead."],
      planned_stop_end: ["replay_event_planned_stop_end_detail", "Continuing because stopping is no longer needed."],
      longitudinal_stopping: ["replay_event_stopping_detail", "Slowing down smoothly until stopped."],
      longitudinal_starting: ["replay_event_starting_detail", "Starting again from a stop."],
    }[event?.type];
    return details ? text(details[0], details[1]) : "";
  }

  function scheduleWork(callback) {
    if (typeof window.requestIdleCallback === "function") {
      window.requestIdleCallback(callback, { timeout: 80 });
    } else {
      window.setTimeout(() => callback({ timeRemaining: () => 8 }), 0);
    }
  }

  function dispatchOpenState() {
    window.dispatchEvent(new CustomEvent("carrot:replayinsightschange", { detail: { open: isOpen() } }));
  }

  function replayLayoutOrientation() {
    const shared = window.CarrotLayout?.orientation?.();
    if (shared === "landscape" || shared === "portrait") return shared;
    const documentRoot = document.documentElement;
    const width = Number(documentRoot?.clientWidth || window.innerWidth || 0);
    const height = Number(documentRoot?.clientHeight || window.innerHeight || 0);
    return width > height ? "landscape" : "portrait";
  }

  function syncDockOpenState() {
    const docked = state.layoutMode === "aux-pane";
    const suppressHud = docked && state.active && state.open;
    auxPaneEl?.classList.toggle("is-replay-insights-host", docked && state.active);
    auxPaneEl?.classList.toggle("is-replay-insights-open", docked && state.open);
    window.DriveVisionHudContent?.setSuppressed?.("replay-insights", suppressHud);
  }

  function syncContainerLayout() {
    const documentRoot = document.documentElement;
    const width = Number(documentRoot?.clientWidth || window.innerWidth || 0);
    const viewportHeight = Number(documentRoot?.clientHeight || window.innerHeight || 0);
    const wideLayout = typeof window.CarrotLayout?.isWide === "function"
      ? window.CarrotLayout.isWide()
      : width / Math.max(1, viewportHeight) >= 1.3 || (width >= 640 && viewportHeight >= 650);
    const useAuxPane = replayLayoutOrientation() === "portrait" && !wideLayout && Boolean(auxPaneEl);
    const layoutMode = useAuxPane ? "aux-pane" : "side";
    const target = layoutMode === "aux-pane" ? auxPaneEl : stageEl;
    const stageHeight = Number(stageEl?.clientHeight || 0);

    state.layoutMode = layoutMode;
    if (stageEl) stageEl.dataset.replayInsightsLayout = layoutMode;
    if (rootEl && target && rootEl.parentElement !== target) target.appendChild(rootEl);
    if (rootEl) rootEl.dataset.layout = layoutMode;
    rootEl?.classList.toggle(
      "is-short-layout",
      layoutMode === "side" && stageHeight > 0 && stageHeight <= POLICY.shortLayoutMaxHeight,
    );
    syncDockOpenState();
  }

  function scheduleContainerLayout() {
    if (state.layoutRaf) return;
    state.layoutRaf = window.requestAnimationFrame(() => {
      state.layoutRaf = 0;
      syncContainerLayout();
      renderRail();
      if (state.activeTab === "advanced" && state.advancedTrack) drawAdvancedTrack(state.advancedTrack);
      if (TEMPORARY_TELEMETRY_SURFACES.replayForward) sensorTopview?.resize?.();
      scrollCurrentEventIntoView();
    });
  }

  function setOpen(open) {
    state.open = Boolean(open && state.active);
    syncContainerLayout();
    stageEl?.classList.toggle("is-replay-insights-open", state.open);
    rootEl?.classList.toggle("is-open", state.open);
    rootEl?.setAttribute("aria-hidden", state.open ? "false" : "true");
    syncDockOpenState();
    if (TEMPORARY_TELEMETRY_SURFACES.replayForward) {
      sensorTopview?.setVisible?.(state.open && state.activeTab === "sensors");
    }
    if (!state.open) {
      closeCluster();
    } else {
      scrollCurrentEventIntoView();
    }
    syncFollowButton();
    dispatchOpenState();
  }

  function selectTab(name, options = {}) {
    const nextName = normalizeTabName(name);
    const changed = state.activeTab !== nextName;
    state.activeTab = nextName;
    tabEls.forEach((element) => {
      const selected = element.dataset.replayInsightsTab === nextName;
      element.classList.toggle("is-active", selected);
      element.setAttribute("aria-selected", selected ? "true" : "false");
      element.tabIndex = selected ? 0 : -1;
    });
    tabControl?.sync?.();
    panelEls.forEach((element) => {
      const selected = element.dataset.replayInsightsPanel === nextName;
      element.classList.toggle("is-active", selected);
      element.setAttribute("aria-hidden", selected ? "false" : "true");
    });
    if (nextName === "graphs") {
      ensureSummarySeries();
      graphEls.forEach((_element, graphName) => syncGraphValue(graphName, state.currentSeconds));
    }
    if (nextName === "advanced") {
      ensureAdvancedCatalog();
      syncAdvancedCurrent();
    }
    if (nextName === "events") scrollCurrentEventIntoView();
    syncFollowButton();
    if (TEMPORARY_TELEMETRY_SURFACES.replayForward) {
      sensorTopview?.setVisible?.(state.open && nextName === "sensors");
      if (nextName === "sensors") sensorTopview?.syncTime?.(state.currentSeconds);
    }
    if (options.persist && changed) persistTabPreference(nextName);
  }

  function clearSummary() {
    if (summaryEl) summaryEl.hidden = true;
  }

  function closeCluster() {
    const wasOpen = Boolean(clusterEl && !clusterEl.hidden);
    if (clusterEl) clusterEl.hidden = true;
    clusterItemsEl?.replaceChildren();
    if (wasOpen) dispatchOpenState();
  }

  function resetAdvanced() {
    state.advancedToken += 1;
    state.snapshotToken += 1;
    state.rawRequestController?.abort?.();
    state.rawRequestController = null;
    state.snapshotRequestController?.abort?.();
    state.snapshotRequestController = null;
    if (state.rawQueryTimer != null) window.clearTimeout(state.rawQueryTimer);
    state.rawQueryTimer = null;
    state.rawCatalogStatus = "idle";
    state.advancedCatalog = [];
    state.advancedTrack = null;
    advancedServiceEl?.replaceChildren();
    advancedFieldEl?.replaceChildren();
    advancedCatalogEl?.replaceChildren();
    advancedSnapshotEl?.replaceChildren();
    if (advancedSnapshotEl) advancedSnapshotEl.hidden = true;
    if (advancedSnapshotButtonEl) advancedSnapshotButtonEl.hidden = true;
    state.snapshotCurrent = null;
    advancedStatisticsEl?.replaceChildren();
    if (advancedStatisticsEl) advancedStatisticsEl.hidden = true;
    if (advancedStatisticsDetailsEl) {
      advancedStatisticsDetailsEl.hidden = true;
      advancedStatisticsDetailsEl.open = false;
    }
    if (advancedSourceDetailsEl) advancedSourceDetailsEl.open = false;
    if (advancedTechnicalPathEl) {
      advancedTechnicalPathEl.textContent = "--";
      advancedTechnicalPathEl.title = "";
    }
    setAdvancedStatus(text("replay_advanced_select", "Select a field."));
    const context = advancedCanvasEl?.getContext?.("2d");
    if (context && advancedCanvasEl) context.clearRect(0, 0, advancedCanvasEl.width, advancedCanvasEl.height);
    if (advancedTrackEl) advancedTrackEl.dataset.kind = "empty";
    [advancedCurrentEl, advancedMinEl, advancedMaxEl, advancedAverageEl].forEach((element) => {
      if (element) element.textContent = "--";
    });
  }

  function resetData() {
    playbackSurfaceScheduler?.cancel?.();
    state.recordsProcessed = 0;
    state.indexingProgress = 0;
    state.indexing = false;
    state.events = [];
    state.services = new Set();
    state.series = { speed: [], steer: [], accel: [], lead: [] };
    state.summarySeriesStatus = "idle";
    state.summaryToken += 1;
    state.activeTab = preferredTabName();
    state.selectedEventId = "";
    state.currentEventId = "";
    state.hoverEventId = "";
    state.passedEventIndex = -2;
    state.pendingEventId = "";
    state.pendingEventSinceMs = 0;
    state.lastSyncSeconds = NaN;
    state.previewVisible = false;
    state.lastAutoScrollEventId = "";
    state.followEvents = true;
    state.expectedScrollTop = null;
    state.eventSource = "index";
    state.currentSeconds = 0;
    state.onSeek = null;
    state.onPause = null;
    state.onUpdate = null;
    state.onReady = null;
    state.records = [];
    state.decodeRecord = null;
    state.rawStats = null;
    if (TEMPORARY_TELEMETRY_SURFACES.replayForward) sensorTopview?.reset?.();
    railEl?.replaceChildren();
    state.timelineCanvas = null;
    state.timelineGroups = [];
    listEl?.replaceChildren();
    closeCluster();
    clearSummary();
    resetAdvanced();
    for (const graphEl of graphEls.values()) {
      graphEl.querySelector("polyline")?.setAttribute("points", "");
      const output = graphEl.querySelector("output");
      if (output) output.textContent = "--";
      const cursor = graphEl.querySelector(".carrot-replay-insights__cursor");
      if (cursor) cursor.style.left = "0%";
    }
    syncCounts();
    selectTab(state.activeTab);
  }

  function reset() {
    state.token += 1;
    state.active = false;
    state.durationSeconds = 0;
    setOpen(false);
    resetData();
    if (rootEl) rootEl.hidden = true;
    if (railEl) railEl.hidden = true;
  }

  function setActive(active) {
    state.active = Boolean(active);
    if (rootEl) rootEl.hidden = !state.active;
    if (railEl) railEl.hidden = !state.active || !state.events.length;
    syncContainerLayout();
    if (!state.active) setOpen(false);
    else setOpen(true);
  }

  function signalReady() {
    const callback = state.onReady;
    state.onReady = null;
    callback?.();
  }

  function syncCounts() {
    const rawEventCount = Number(state.rawStats?.rawEventCount);
    const rawServices = state.rawStats?.rawServiceStats;
    const rawServiceCount = rawServices && typeof rawServices === "object" ? Object.keys(rawServices).length : NaN;
    if (recordCountEl) recordCountEl.textContent = numberText(Number.isFinite(rawEventCount) ? rawEventCount : state.recordsProcessed, 0);
    if (serviceCountEl) serviceCountEl.textContent = numberText(Number.isFinite(rawServiceCount) ? rawServiceCount : state.services.size, 0);
    if (eventCountEl) eventCountEl.textContent = numberText(state.events.length, 0);
    if (graphCountEl) {
      graphCountEl.textContent = numberText(Object.values(state.series).reduce((sum, series) => sum + series.length, 0), 0);
    }
    if (invalidCountEl) invalidCountEl.textContent = numberText(Number(state.rawStats?.rawInvalidEventCount || 0), 0);
  }

  function graphRange(name, samples) {
    let min = Math.min(...samples.map((sample) => sample.value));
    let max = Math.max(...samples.map((sample) => sample.value));
    if (name === "speed" || name === "lead") min = Math.min(0, min);
    if (name === "steer" || name === "accel") {
      const span = Math.max(1, Math.abs(min), Math.abs(max));
      min = -span;
      max = span;
    }
    if (Math.abs(max - min) < 0.001) {
      min -= 1;
      max += 1;
    }
    return { min, max };
  }

  function polylinePoints(samples, durationSeconds, name, height = 90) {
    if (!samples.length || durationSeconds <= 0) return "";
    const visible = model?.downsample?.(samples, POLICY.graphPointLimit) || samples;
    const range = graphRange(name, visible);
    const durationMs = durationSeconds * 1000;
    return visible.map((sample) => {
      const x = Math.max(0, Math.min(600, sample.timeMs / durationMs * 600));
      const y = height - 5 - ((sample.value - range.min) / (range.max - range.min)) * (height - 10);
      return `${x.toFixed(2)},${y.toFixed(2)}`;
    }).join(" ");
  }

  function renderGraph(name) {
    const graphEl = graphEls.get(name);
    const line = graphEl?.querySelector("polyline");
    if (!line) return;
    line.setAttribute("points", polylinePoints(state.series[name] || [], state.durationSeconds, name));
  }

  function renderGraphs() {
    if (!TEMPORARY_TELEMETRY_SURFACES.replayGraphs) return;
    graphEls.forEach((_element, name) => renderGraph(name));
  }

  function eventDisplayTitle(event) {
    return replayEventDisplayTitle(eventTitle(event), String(event?.sourceTag || "").trim());
  }

  function ensureSummarySeries() {
    if (!TEMPORARY_TELEMETRY_SURFACES.replayGraphs) return;
    if (state.summarySeriesStatus === "ready" || state.summarySeriesStatus === "loading") return;
    if (!state.records.length || !state.decodeRecord || typeof dataTracks?.buildSummarySeries !== "function") return;
    const token = ++state.summaryToken;
    state.summarySeriesStatus = "loading";
    graphEls.forEach((graphEl) => {
      const output = graphEl.querySelector("output");
      if (output) output.textContent = "…";
    });
    dataTracks.buildSummarySeries({
      records: state.records,
      decodeRecord: state.decodeRecord,
      shouldCancel: () => token !== state.summaryToken || !state.active,
    }).then((series) => {
      if (!series || token !== state.summaryToken || !state.active) return;
      state.series = series;
      state.summarySeriesStatus = "ready";
      renderGraphs();
      syncCounts();
      syncTime(state.currentSeconds);
    });
  }

  function eventButton(event, compact = false) {
    const button = document.createElement("button");
    button.type = "button";
    button.dataset.eventId = event.id;
    button.dataset.category = event.category;
    button.setAttribute("aria-label", `${formatTime(event.timeMs / 1000)} ${eventDisplayTitle(event)}`);
    const copy = document.createElement("span");
    if (compact) {
      const time = document.createElement("time");
      time.textContent = formatTime(event.timeMs / 1000);
      copy.textContent = eventDisplayTitle(event);
      button.append(time, copy);
    } else {
      const marker = document.createElement("span");
      marker.className = "carrot-replay-insights__timelineMarker";
      marker.setAttribute("aria-hidden", "true");
      copy.className = "carrot-replay-insights__eventCopy";
      const title = document.createElement("strong");
      title.textContent = eventDisplayTitle(event);
      const detail = document.createElement("span");
      const detailText = eventDetail(event);
      if (detailText) detail.textContent = detailText;
      copy.append(title);
      if (detailText) copy.append(detail);
      button.append(marker, copy);
    }
    button.addEventListener("click", () => selectEvent(event));
    button.addEventListener("pointerenter", (pointerEvent) => {
      if (pointerEvent.pointerType === "touch") return;
      state.hoverEventId = event.id;
      paintRail();
    });
    button.addEventListener("pointerleave", (pointerEvent) => {
      if (pointerEvent.pointerType === "touch") return;
      if (state.hoverEventId !== event.id) return;
      state.hoverEventId = "";
      paintRail();
    });
    return button;
  }

  function selectEvent(event) {
    const eventSeconds = Math.max(0, event.timeMs / 1000);
    const reviewStartSeconds = Math.max(0, eventSeconds - POLICY.eventReviewLeadSeconds);
    state.selectedEventId = event.id;
    state.currentEventId = event.id;
    state.lastAutoScrollEventId = "";
    state.pendingEventId = "";
    state.pendingEventSinceMs = 0;
    state.lastSyncSeconds = reviewStartSeconds;
    // Picking an event is an explicit "take me there", so following resumes.
    state.followEvents = true;
    closeCluster();
    syncSelectedEvent();
    scrollCurrentEventIntoView();
    syncVerticalTimeline(reviewStartSeconds);
    state.onSeek?.(reviewStartSeconds);
  }

  // Event buttons live in two places that are NOT in one subtree: the insights
  // list, and the timeline cluster inside the transport. Querying from
  // rootEl.parentElement silently lost the cluster once syncContainerLayout
  // reparented the panel into the HUD dock, so scope off the owning elements
  // instead of whatever happens to be the current parent.
  function eventNodes() {
    const nodes = [];
    if (listEl) nodes.push(...listEl.querySelectorAll("[data-event-id]"));
    if (clusterItemsEl) nodes.push(...clusterItemsEl.querySelectorAll("[data-event-id]"));
    return nodes;
  }

  function syncSelectedEvent() {
    eventNodes().forEach((element) => {
      const current = element.dataset.eventId === state.currentEventId;
      element.classList.toggle("is-selected", element.dataset.eventId === state.selectedEventId);
      element.classList.toggle("is-current", current);
      if (current) element.setAttribute("aria-current", "true");
      else element.removeAttribute("aria-current");
      element.closest?.(".carrot-replay-insights__eventItem")?.classList.toggle("is-focus-hit", current);
    });
    paintRail();
  }

  function eventsViewport() {
    return listEl?.closest?.(".carrot-replay-insights__panel") || null;
  }

  function syncEventSourceNotice() {
    if (!degradedEl) return;
    degradedEl.hidden = state.eventSource !== "fallback";
    degradedEl.textContent = text(
      "replay_events_degraded",
      "Partial event list: this segment's event index is unavailable.",
    );
  }

  // Scrolling the list is how the user says "I am reading this, stop moving it".
  // Auto-follow resumes on an explicit action (the follow button, picking an
  // event, a new segment) or once the current event is back on screen by itself.
  function setFollowEvents(follow) {
    const next = Boolean(follow);
    if (state.followEvents === next) {
      syncFollowButton();
      return;
    }
    state.followEvents = next;
    syncFollowButton();
    if (next) scrollCurrentEventIntoView();
  }

  function currentEventFullyVisible() {
    const viewport = eventsViewport();
    if (!viewport || !state.currentEventId) return false;
    const button = eventNodes().find((element) => (
      element.dataset.eventId === state.currentEventId && listEl?.contains?.(element)
    ));
    if (!button) return false;
    const viewportRect = viewport.getBoundingClientRect();
    const itemRect = button.getBoundingClientRect();
    return itemRect.top >= viewportRect.top && itemRect.bottom <= viewportRect.bottom;
  }

  function syncFollowButton() {
    if (!followButtonEl) return;
    const show = state.active && state.open && state.activeTab === "events"
      && !state.followEvents && Boolean(state.currentEventId) && !currentEventFullyVisible();
    followButtonEl.hidden = !show;
  }

  function handleEventListScroll() {
    const viewport = eventsViewport();
    // Our own scrollTop writes fire this event too. Recognise them by the
    // position we asked for rather than by racing a timer to clear a flag.
    if (viewport && state.expectedScrollTop != null
      && Math.abs(viewport.scrollTop - state.expectedScrollTop) < 1.5) {
      state.expectedScrollTop = null;
      return;
    }
    state.expectedScrollTop = null;
    // Any real gesture suspends following, and only an explicit action resumes
    // it. Auto-resuming as soon as the current event happened to be visible
    // snapped the list back under the user's finger - during playback the
    // current event is nearly always nearby, so the list read as if scrolling
    // were locked.
    if (state.followEvents) state.followEvents = false;
    syncFollowButton();
  }

  function applyEventScroll(viewport, delta) {
    const target = viewport.scrollTop + delta;
    state.expectedScrollTop = target;
    viewport.scrollTop = target;
  }

  function scrollCurrentEventIntoView() {
    if (!state.open || state.activeTab !== "events" || !state.currentEventId) return;
    if (!state.followEvents) {
      syncFollowButton();
      return;
    }
    const buttons = Array.from(listEl?.querySelectorAll?.("[data-event-id]") || []);
    const button = buttons
      .find((element) => element.dataset.eventId === state.currentEventId);
    if (!button) return;
    const viewport = eventsViewport();
    if (viewport) {
      const viewportRect = viewport.getBoundingClientRect();
      const currentIndex = buttons.indexOf(button);
      const itemRect = button.getBoundingClientRect();
      const itemHeight = Math.max(1, itemRect.height);
      const visibleSlots = Math.max(1, Math.floor(
        (viewportRect.height - POLICY.eventScrollSafeInsetPx * 2) / itemHeight,
      ));
      const contextItems = Math.min(
        POLICY.eventScrollContextMaxItems,
        Math.max(0, Math.floor((visibleSlots - 1) / 2)),
      );
      const first = buttons[Math.max(0, currentIndex - contextItems)] || button;
      const last = buttons[Math.min(buttons.length - 1, currentIndex + contextItems)] || button;
      const firstRect = first.getBoundingClientRect();
      const lastRect = last.getBoundingClientRect();
      const safeTop = viewportRect.top + POLICY.eventScrollSafeInsetPx;
      const safeBottom = viewportRect.bottom - POLICY.eventScrollSafeInsetPx;
      let delta = 0;
      if (firstRect.top < safeTop) delta = firstRect.top - safeTop;
      else if (lastRect.bottom > safeBottom) delta = lastRect.bottom - safeBottom;
      if (Math.abs(delta) <= 0.5) {
        state.lastAutoScrollEventId = state.currentEventId;
        syncFollowButton();
        return;
      }
      applyEventScroll(viewport, delta);
      state.lastAutoScrollEventId = state.currentEventId;
      syncFollowButton();
      return;
    }
    try {
      button.scrollIntoView?.({ block: "nearest", inline: "nearest", behavior: "auto" });
    } catch {
      button.scrollIntoView?.(false);
    }
    state.lastAutoScrollEventId = state.currentEventId;
    syncFollowButton();
  }

  function syncCurrentEvent(event) {
    const eventId = event?.id || "";
    if (eventId === state.currentEventId) return;
    state.currentEventId = eventId;
    syncSelectedEvent();
    if (eventId && eventId !== state.lastAutoScrollEventId) scrollCurrentEventIntoView();
    else syncFollowButton();
  }

  function eventIndexAt(seconds) {
    const timeMs = Math.max(0, Number(seconds) || 0) * 1000;
    let low = 0;
    let high = state.events.length;
    while (low < high) {
      const middle = Math.floor((low + high) / 2);
      if (state.events[middle].timeMs <= timeMs) low = middle + 1;
      else high = middle;
    }
    return low - 1;
  }

  function timelineEventAt(seconds) {
    return state.events[eventIndexAt(seconds)] || null;
  }

  function selectedReviewEventAt(seconds) {
    if (!state.selectedEventId) return null;
    const selected = state.events.find((event) => event.id === state.selectedEventId);
    if (!selected) return null;
    const eventSeconds = selected.timeMs / 1000;
    const reviewStart = Math.max(0, eventSeconds - POLICY.eventReviewLeadSeconds);
    const reviewEnd = eventSeconds + POLICY.summaryWindowSeconds;
    return seconds >= reviewStart - 0.05 && seconds <= reviewEnd ? selected : null;
  }

  function clearSelectionOutsideReview(reviewEvent) {
    if (!state.selectedEventId || reviewEvent) return;
    state.selectedEventId = "";
    syncSelectedEvent();
  }

  function nowMs() {
    const value = Number(window.performance?.now?.());
    return Number.isFinite(value) ? value : Date.now();
  }

  function stabilizedPlaybackEvent(candidate, seconds) {
    const currentSeconds = Math.max(0, Number(seconds) || 0);
    const previousSeconds = state.lastSyncSeconds;
    state.lastSyncSeconds = currentSeconds;
    const candidateId = candidate?.id || "";
    const jumped = !Number.isFinite(previousSeconds)
      || currentSeconds < previousSeconds
      || Math.abs(currentSeconds - previousSeconds) >= POLICY.eventSeekJumpSeconds;
    if (!candidateId || !state.currentEventId || candidateId === state.currentEventId || jumped) {
      state.pendingEventId = "";
      state.pendingEventSinceMs = 0;
      return candidate;
    }
    if (state.pendingEventId !== candidateId) {
      state.pendingEventId = candidateId;
      state.pendingEventSinceMs = nowMs();
      return state.events.find((event) => event.id === state.currentEventId) || candidate;
    }
    // Settle on wall-clock so the highlight is equally calm at 1x and 4x.
    if (nowMs() - state.pendingEventSinceMs < POLICY.eventHighlightSettleMs) {
      return state.events.find((event) => event.id === state.currentEventId) || candidate;
    }
    state.pendingEventId = "";
    state.pendingEventSinceMs = 0;
    return candidate;
  }

  function syncVerticalTimeline(seconds, force = false) {
    const passedIndex = eventIndexAt(seconds);
    if (!force && passedIndex === state.passedEventIndex) return;
    state.passedEventIndex = passedIndex;
    listEl?.querySelectorAll?.(".carrot-replay-insights__eventItem").forEach((item) => {
      const index = Number(item.dataset.eventIndex);
      const passed = Number.isFinite(index) && index <= passedIndex;
      item.classList.toggle("is-past", passed);
      item.querySelector("button")?.classList.toggle("is-past", passed);
    });
  }

  function clusterOpen() {
    return Boolean(clusterEl && !clusterEl.hidden);
  }

  // One surface at a time above the transport. The scrub preview and the cluster
  // are both direct responses to something the user is doing, so they outrank
  // the passive playback readout; stacking all three is what made this corner
  // unreadable.
  function showSummary(event) {
    if (!summaryEl || !event || state.previewVisible || clusterOpen()) return clearSummary();
    summaryEl.dataset.category = event.category;
    if (summaryIconEl) {
      const icon = window.CarrotMediaTransport?.createEventIcon?.(event.category);
      if (icon) summaryIconEl.replaceChildren(icon);
      else summaryIconEl.replaceChildren();
    }
    if (summaryTitleEl) summaryTitleEl.textContent = eventDisplayTitle(event);
    if (summaryMetaEl) {
      const detail = eventDetail(event);
      summaryMetaEl.textContent = [formatTime(event.timeMs / 1000), detail].filter(Boolean).join(" · ");
    }
    summaryEl.hidden = false;
  }

  function renderEventList() {
    if (!listEl) return;
    listEl.replaceChildren(...state.events.map((event, index) => {
      const item = document.createElement("li");
      item.className = "carrot-replay-insights__eventItem setting";
      item.dataset.category = event.category;
      item.dataset.eventIndex = String(index);
      item.append(eventButton(event));
      return item;
    }));
    if (emptyEl) emptyEl.hidden = state.events.length > 0;
    syncSelectedEvent();
    syncVerticalTimeline(state.currentSeconds, true);
    scrollCurrentEventIntoView();
  }

  function layoutEvents() {
    return model?.layoutEvents?.(state.events, state.durationSeconds, railEl?.clientWidth)
      || model?.clusterEvents?.(state.events, state.durationSeconds, railEl?.clientWidth)
      || [];
  }

  function openCluster(group, leftPercent) {
    if (!clusterEl || !clusterItemsEl) return;
    // The cluster owns the lane while it is open.
    clearSummary();
    clusterEl.style.setProperty("--popover-left", `${Math.max(13, Math.min(87, leftPercent))}%`);
    if (clusterTitleEl) {
      clusterTitleEl.textContent = text("replay_event_count", "{count} events", { count: numberText(group.length, 0) });
    }
    clusterItemsEl.replaceChildren(...group.map((event) => eventButton(event, true)));
    clusterEl.hidden = false;
    dispatchOpenState();
  }

  function ensureRailCanvas() {
    if (!railEl) return null;
    if (state.timelineCanvas?.isConnected) return state.timelineCanvas;
    const canvas = document.createElement("canvas");
    canvas.className = "carrot-replay-insights__eventCanvas";
    canvas.setAttribute("aria-hidden", "true");
    railEl.replaceChildren(canvas);
    state.timelineCanvas = canvas;
    return canvas;
  }

  function eventColor(category) {
    const property = CATEGORY_COLOR_VAR[category] || "--carrot-replay-insights-event";
    const resolved = window.getComputedStyle?.(railEl).getPropertyValue(property).trim();
    return resolved || "#ffad66";
  }

  function strokeEventTick(context, x, height, color, options = {}) {
    const emphasized = Boolean(options.current || options.selected || options.hovered);
    const top = options.current ? 2 : options.selected || options.hovered ? 4 : 6;
    context.beginPath();
    context.moveTo(x, top);
    context.lineTo(x, Math.min(18, top + height));
    context.lineCap = "round";
    context.lineWidth = emphasized ? 4.5 : 3.5;
    context.strokeStyle = "rgba(0, 0, 0, .72)";
    context.stroke();
    context.globalAlpha = emphasized ? 1 : 0.76;
    context.lineWidth = options.current ? 3 : options.selected || options.hovered ? 2.5 : 1.5;
    context.strokeStyle = color;
    context.stroke();
    context.globalAlpha = 1;
  }

  function paintRail() {
    const canvas = state.timelineCanvas;
    if (!canvas || !railEl || railEl.hidden) return;
    const width = Math.max(0, railEl.clientWidth);
    const height = Math.max(0, railEl.clientHeight);
    if (!width || !height) return;
    const ratio = Math.max(1, Math.min(3, Number(window.devicePixelRatio) || 1));
    const pixelWidth = Math.max(1, Math.round(width * ratio));
    const pixelHeight = Math.max(1, Math.round(height * ratio));
    if (canvas.width !== pixelWidth) canvas.width = pixelWidth;
    if (canvas.height !== pixelHeight) canvas.height = pixelHeight;
    const context = canvas.getContext("2d", { alpha: true });
    if (!context) return;
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    context.clearRect(0, 0, width, height);

    for (const group of state.timelineGroups) {
      const events = group.events || [];
      if (!events.length) continue;
      const x = Math.max(2, Math.min(width - 2, Number(group.pixel) || 0));
      const focused = events.filter((event) => (
        event.id === state.currentEventId || event.id === state.selectedEventId || event.id === state.hoverEventId
      ));
      const visible = events.slice(0, 12);
      focused.forEach((event) => {
        if (!visible.some((candidate) => candidate.id === event.id)) visible.push(event);
      });
      visible.sort((a, b) => {
        const aFocused = a.id === state.currentEventId || a.id === state.selectedEventId || a.id === state.hoverEventId;
        const bFocused = b.id === state.currentEventId || b.id === state.selectedEventId || b.id === state.hoverEventId;
        return Number(aFocused) - Number(bFocused);
      });
      const spacing = visible.length > 1 ? Math.min(2, 12 / (visible.length - 1)) : 0;
      visible.forEach((event, index) => {
        const current = event.id === state.currentEventId;
        const selected = event.id === state.selectedEventId;
        const hovered = event.id === state.hoverEventId;
        const offset = (index - (visible.length - 1) / 2) * spacing;
        const tickX = Math.max(1, Math.min(width - 1, x + offset));
        const tickHeight = current ? 16 : selected || hovered ? 13 : event.category === "warning" ? 12 : 9;
        strokeEventTick(context, tickX, tickHeight, eventColor(event.category), { current, selected, hovered });
      });
    }
  }

  function renderRail() {
    if (!railEl) return;
    state.timelineGroups = layoutEvents();
    railEl.hidden = !state.active || state.timelineGroups.length === 0;
    if (railEl.hidden) return;
    ensureRailCanvas();
    paintRail();
  }

  function selectTimelineAt(seconds, options = {}) {
    if (!state.active || !state.timelineGroups.length || state.durationSeconds <= 0 || !railEl) return false;
    const width = Math.max(1, seekEl?.clientWidth || railEl.clientWidth);
    const pointerX = Math.max(0, Math.min(width, Number(seconds) / state.durationSeconds * width));
    let nearest = null;
    let distance = Infinity;
    for (const group of state.timelineGroups) {
      const candidateDistance = Math.abs((Number(group.pixel) || 0) - pointerX);
      if (candidateDistance >= distance) continue;
      nearest = group;
      distance = candidateDistance;
    }
    const tolerance = options.pointerType === "touch" ? 16 : 9;
    if (!nearest || distance > tolerance) return false;
    if (nearest.events.length > 1) openCluster(nearest.events, nearest.leftPercent);
    else selectEvent(nearest.events[0]);
    return true;
  }

  function previewEventsAt(seconds, windowSeconds = 6) {
    if (!state.active || state.durationSeconds <= 0) return null;
    const span = Math.min(state.durationSeconds, Math.max(1, Number(windowSeconds) || 6));
    const center = Math.max(0, Math.min(state.durationSeconds, Number(seconds) || 0));
    const startSeconds = Math.max(0, Math.min(state.durationSeconds - span, center - span / 2));
    const endSeconds = startSeconds + span;
    const startMs = startSeconds * 1000;
    const endMs = endSeconds * 1000;
    let low = 0;
    let high = state.events.length;
    while (low < high) {
      const middle = Math.floor((low + high) / 2);
      if (state.events[middle].timeMs < startMs) low = middle + 1;
      else high = middle;
    }
    const nearby = [];
    for (let index = low; index < state.events.length; index += 1) {
      const event = state.events[index];
      if (event.timeMs > endMs) break;
      nearby.push({ id: event.id, category: event.category, timeSeconds: event.timeMs / 1000 });
    }
    return {
      startSeconds,
      endSeconds,
      events: nearby,
    };
  }

  function selectEventById(eventId) {
    const event = state.events.find((candidate) => candidate.id === String(eventId || ""));
    if (!event) return false;
    selectEvent(event);
    return true;
  }

  function renderAll() {
    state.events.sort((a, b) => a.timeMs - b.timeMs);
    renderEventList();
    renderRail();
    renderGraphs();
    syncCounts();
    renderAdvancedCatalog();
  }

  function syncGraphValue(name, seconds) {
    const ui = graphUi.get(name);
    if (!ui) return;
    const cursorLeft = `${state.durationSeconds > 0 ? Math.max(0, Math.min(100, seconds / state.durationSeconds * 100)) : 0}%`;
    if (ui.cursor && ui.cursor.style.left !== cursorLeft) ui.cursor.style.left = cursorLeft;
    const sample = model?.nearestSample?.(state.series[name] || [], seconds * 1000) || null;
    const format = GRAPH_FORMAT[name];
    const output = format ? formatValue(sample?.value, format.digits, format.unit) : "";
    if (ui.output && ui.output.textContent !== output) ui.output.textContent = output;
  }

  function selectedAdvancedEntry() {
    const service = String(advancedServiceEl?.value || "");
    const fieldPath = String(advancedFieldEl?.value || "");
    const serviceEntry = state.advancedCatalog.find((entry) => entry.service === service);
    const field = serviceEntry?.fields?.find((entry) => entry.path === fieldPath);
    return serviceEntry && field ? { serviceEntry, field } : null;
  }

  function serviceDisplayName(service) {
    const key = String(service || "");
    const meta = SERVICE_LABEL_META[key];
    return meta ? text(meta[0], meta[1]) : key;
  }

  function fieldDisplayName(service, field) {
    const path = String(field?.path || "");
    const meta = FIELD_LABEL_META[`${service}.${path}`];
    if (meta) return text(meta[0], meta[1]);
    return path;
  }

  function fieldUnit(field, service = "") {
    if (field?.unit) return String(field.unit);
    const scoped = `${service}.${String(field?.path || "")}`;
    if (FIELD_UNIT_META[scoped]) return FIELD_UNIT_META[scoped];
    const leaf = String(field?.path || "").split(".").pop() || "";
    return FIELD_UNIT_META[leaf] || "";
  }

  function syncAdvancedSelectionMeta() {
    if (!advancedTechnicalPathEl) return;
    const selection = selectedAdvancedEntry();
    if (!selection) {
      advancedTechnicalPathEl.textContent = "--";
      advancedTechnicalPathEl.title = "";
      return;
    }
    const { serviceEntry, field } = selection;
    const unit = fieldUnit(field, serviceEntry.service);
    const technical = `${serviceEntry.service}.${field.path} · ${field.type}${unit ? ` · ${unit}` : ""}`;
    advancedTechnicalPathEl.textContent = technical;
    advancedTechnicalPathEl.title = technical;
  }

  function setAdvancedStatus(message, status = "idle") {
    if (!advancedStatusEl) return;
    advancedStatusEl.textContent = String(message || "");
    advancedStatusEl.dataset.state = status;
  }

  function groupLabel(groupKey) {
    const group = dataTracks?.GROUPS?.find((entry) => entry.key === groupKey);
    return group ? text(group.labelKey, group.key) : groupKey;
  }

  function renderAdvancedCatalog(catalog = null, syncFields = true) {
    if (Array.isArray(catalog)) state.advancedCatalog = catalog;
    else if (state.rawCatalogStatus !== "ready") state.advancedCatalog = dataTracks?.catalog?.(state.services) || [];
    if (!advancedServiceEl || !advancedFieldEl) return;
    const previousService = advancedServiceEl.value;
    const fragment = document.createDocumentFragment();
    for (const group of dataTracks?.GROUPS || []) {
      const entries = state.advancedCatalog.filter((entry) => entry.group === group.key);
      if (!entries.length) continue;
      const optionGroup = document.createElement("optgroup");
      optionGroup.label = groupLabel(group.key);
      entries.forEach((entry) => {
        const option = document.createElement("option");
        option.value = entry.service;
        option.textContent = serviceDisplayName(entry.service);
        option.title = entry.service;
        optionGroup.append(option);
      });
      fragment.append(optionGroup);
    }
    advancedServiceEl.replaceChildren(fragment);
    if (state.advancedCatalog.some((entry) => entry.service === previousService)) {
      advancedServiceEl.value = previousService;
    } else if (state.advancedCatalog.some((entry) => entry.service === "carState")) {
      advancedServiceEl.value = "carState";
    }
    if (syncFields) syncAdvancedFields(false);

    advancedCatalogEl?.replaceChildren(...(dataTracks?.GROUPS || []).map((group) => {
      const services = state.advancedCatalog.filter((entry) => entry.group === group.key);
      if (!services.length) return null;
      const section = document.createElement("section");
      section.className = "carrot-replay-insights__catalogGroup";
      const title = document.createElement("strong");
      title.textContent = groupLabel(group.key);
      const list = document.createElement("div");
      list.className = "carrot-replay-insights__catalogItems";
      services.forEach((entry) => {
        const chip = document.createElement("span");
        chip.className = "chip";
        const count = Number(entry.stats?.count);
        chip.textContent = Number.isFinite(count)
          ? `${serviceDisplayName(entry.service)} · ${numberText(count, 0)}`
          : `${serviceDisplayName(entry.service)} · ${numberText(entry.fieldCount ?? entry.fields.length, 0)}`;
        chip.title = entry.service;
        if (entry.stats) {
          chip.title = `${entry.service} · ${text("replay_advanced_service_health", "{fields} fields · {count} entries · {hz} Hz · {coverage}% coverage · {invalid} invalid · {gap} ms max gap", {
            fields: numberText(entry.fieldCount ?? entry.fields.length, 0),
            count: numberText(entry.stats.count, 0),
            hz: numberText(entry.stats.observedHz, 1),
            coverage: numberText(entry.stats.coveragePct, 1),
            invalid: numberText(entry.stats.invalidCount, 0),
            gap: numberText(entry.stats.maxGapMs, 1),
          })}`;
        }
        list.append(chip);
      });
      section.append(title, list);
      return section;
    }).filter(Boolean));
  }

  function ensureAdvancedCatalog() {
    state.rawCatalogStatus = dataTracks?.canQueryClientRaw?.(state.rawStats) ? "client" : "compact";
    renderAdvancedCatalog();
    ensureAdvancedTrack();
  }

  async function syncAdvancedFields(loadTrack = true) {
    if (!advancedServiceEl || !advancedFieldEl) return;
    if (state.snapshotCurrent?.service !== advancedServiceEl.value) {
      state.snapshotToken += 1;
      state.snapshotRequestController?.abort?.();
      state.snapshotRequestController = null;
      state.snapshotCurrent = null;
      advancedSnapshotEl?.replaceChildren();
      if (advancedSnapshotEl) advancedSnapshotEl.hidden = true;
    }
    const serviceEntry = state.advancedCatalog.find((entry) => entry.service === advancedServiceEl.value);
    const previousField = advancedFieldEl.value;
    advancedFieldEl.replaceChildren(...(serviceEntry?.fields || []).map((field) => {
      const option = document.createElement("option");
      option.value = field.path;
      const unit = fieldUnit(field, serviceEntry?.service);
      if (unit && !field.unit) field.unit = unit;
      option.textContent = fieldDisplayName(serviceEntry?.service, field);
      option.title = `${field.path} · ${field.type}${unit ? ` · ${unit}` : ""}`;
      option.dataset.renderer = String(field.recommendedView || "raw");
      return option;
    }));
    if (serviceEntry?.fields?.some((field) => field.path === previousField)) advancedFieldEl.value = previousField;
    syncAdvancedSelectionMeta();
    if (advancedSnapshotButtonEl) {
      advancedSnapshotButtonEl.hidden = !serviceEntry || !dataTracks?.canQueryClientRaw?.(state.rawStats);
    }
    if (loadTrack && state.activeTab === "advanced") ensureAdvancedTrack();
  }

  function ensureAdvancedTrack() {
    const selection = selectedAdvancedEntry();
    const canUseClientRaw = dataTracks?.canQueryClientRaw?.(state.rawStats)
      && typeof dataTracks?.queryClientTrack === "function";
    if (!selection || (!canUseClientRaw && (!state.records.length || !state.decodeRecord))) {
      setAdvancedStatus(text("replay_advanced_select", "Select a field."));
      return;
    }
    const token = ++state.advancedToken;
    state.advancedTrack = null;
    [advancedCurrentEl, advancedMinEl, advancedMaxEl, advancedAverageEl].forEach((element) => {
      if (element) element.textContent = "--";
    });
    advancedStatisticsEl?.replaceChildren();
    if (advancedStatisticsEl) advancedStatisticsEl.hidden = true;
    if (advancedStatisticsDetailsEl) {
      advancedStatisticsDetailsEl.hidden = true;
      advancedStatisticsDetailsEl.open = false;
    }
    if (advancedTrackEl) advancedTrackEl.dataset.kind = "empty";
    const context = advancedCanvasEl?.getContext?.("2d");
    if (context && advancedCanvasEl) context.clearRect(0, 0, advancedCanvasEl.width, advancedCanvasEl.height);
    state.rawRequestController?.abort?.();
    state.rawRequestController = null;
    setAdvancedStatus(text("replay_advanced_loading", "Analyzing field..."), "loading");
    const storeTrack = (track) => {
      if (!track || token !== state.advancedToken) return;
      state.advancedTrack = track;
      renderAdvancedTrack();
      syncAdvancedCurrent();
    };
    if (canUseClientRaw) {
      const controller = new AbortController();
      state.rawRequestController = controller;
      if (state.rawQueryTimer != null) window.clearTimeout(state.rawQueryTimer);
      state.rawQueryTimer = window.setTimeout(() => {
        state.rawQueryTimer = null;
        dataTracks.queryClientTrack({
          manifest: state.rawStats,
          service: selection.serviceEntry.service,
          field: selection.field,
          signal: controller.signal,
          onProgress: (progress) => {
            if (token !== state.advancedToken) return;
            setAdvancedStatus(text("replay_advanced_loading_progress", "Building field track {percent}%", {
              percent: numberText(progress * 100, 0),
            }), "loading");
          },
        }).then(storeTrack).catch((error) => {
          if (error?.name === "AbortError" || token !== state.advancedToken) return;
          setAdvancedStatus(text("replay_advanced_failed", "Field analysis failed."), "error");
        });
      }, POLICY.rawQueryDebounceMs);
      return;
    }
    dataTracks.buildTrack({
      records: state.records,
      decodeRecord: state.decodeRecord,
      service: selection.serviceEntry.service,
      field: selection.field,
      shouldCancel: () => token !== state.advancedToken || !state.active,
      onProgress: (progress) => {
        if (token !== state.advancedToken) return;
        setAdvancedStatus(text("replay_advanced_loading_progress", "Building field track {percent}%", {
          percent: numberText(progress * 100, 0),
        }), "loading");
      },
    }).then(storeTrack).catch((error) => {
      if (error?.name === "AbortError" || token !== state.advancedToken) return;
      setAdvancedStatus(text("replay_advanced_failed", "Field analysis failed."), "error");
    });
  }

  function advancedValue(value, field, digits = null) {
    if (typeof value === "number") {
      const precision = digits == null ? (Math.abs(value) < 10 ? 2 : 1) : digits;
      const unit = fieldUnit(field);
      return `${numberText(value, precision)}${unit ? ` ${unit}` : ""}`;
    }
    if (typeof value === "boolean") return value
      ? text("replay_value_true", "True")
      : text("replay_value_false", "False");
    if (typeof value === "string") return value || text("replay_value_empty", "Empty");
    if (value && typeof value === "object" && Object.prototype.hasOwnProperty.call(value, "length")) {
      const average = Number.isFinite(Number(value.average))
        ? text("replay_value_average", " · avg {value}", { value: numberText(value.average, 2) })
        : "";
      return text("replay_value_list", "{count} values{average}", {
        count: numberText(value.length, 0),
        average,
      });
    }
    if (value && typeof value === "object" && Object.prototype.hasOwnProperty.call(value, "keys")) {
      return text("replay_value_object", "{count} fields", { count: numberText(value.keys, 0) });
    }
    return field?.type || "--";
  }

  function advancedSampleSignature(value) {
    if (value == null) return String(value);
    if (typeof value !== "object") return `${typeof value}:${String(value)}`;
    try { return JSON.stringify(value); } catch { return Object.prototype.toString.call(value); }
  }

  function canvasToken(name, fallback) {
    const value = window.getComputedStyle?.(document.documentElement)?.getPropertyValue(name)?.trim();
    return value || fallback;
  }

  function drawAdvancedTrack(track) {
    if (!advancedCanvasEl || !advancedTrackEl || !track || advancedTrackEl.hidden) return;
    const rect = advancedTrackEl.getBoundingClientRect();
    const width = Math.max(1, Math.round(rect.width));
    const height = Math.max(1, Math.round(rect.height));
    const ratio = Math.max(1, Math.min(2, Number(window.devicePixelRatio) || 1));
    const pixelWidth = Math.max(1, Math.round(width * ratio));
    const pixelHeight = Math.max(1, Math.round(height * ratio));
    if (advancedCanvasEl.width !== pixelWidth || advancedCanvasEl.height !== pixelHeight) {
      advancedCanvasEl.width = pixelWidth;
      advancedCanvasEl.height = pixelHeight;
    }
    const context = advancedCanvasEl.getContext("2d");
    if (!context) return;
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    context.clearRect(0, 0, width, height);
    const durationMs = Math.max(1, state.durationSeconds * 1000);
    const samples = Array.isArray(track.samples) ? track.samples : [];
    if (!samples.length) return;

    if (track.kind === "number") {
      let min = Number(track.stats?.min);
      let max = Number(track.stats?.max);
      if (!Number.isFinite(min) || !Number.isFinite(max)) return;
      if (Math.abs(max - min) < 1e-9) {
        min -= 1;
        max += 1;
      }
      const inset = 7;
      context.beginPath();
      samples.forEach((sample, index) => {
        const x = Math.max(0, Math.min(width, Number(sample.timeMs || 0) / durationMs * width));
        const y = height - inset - ((Number(sample.value) - min) / (max - min)) * Math.max(1, height - inset * 2);
        if (index === 0) context.moveTo(x, y);
        else context.lineTo(x, y);
      });
      context.strokeStyle = canvasToken("--md-info", "#56a8ff");
      context.lineWidth = 1.6;
      context.lineJoin = "miter";
      context.lineCap = "butt";
      context.stroke();
      return;
    }

    const colors = [
      canvasToken("--md-info", "#56a8ff"),
      canvasToken("--md-success", "#53c98b"),
      canvasToken("--md-warning", "#f4b942"),
      canvasToken("--md-primary", "#ff9d45"),
    ];
    const boundary = canvasToken("--md-outline-variant", "rgba(255,255,255,.16)");
    const colorByValue = new Map();
    samples.forEach((sample, index) => {
      const next = samples[index + 1];
      const start = Math.max(0, Math.min(width, Number(sample.timeMs || 0) / durationMs * width));
      const naturalEnd = next
        ? Math.max(start, Math.min(width, Number(next.timeMs || 0) / durationMs * width))
        : width;
      const end = advancedTrackEl.dataset.renderMode === "markers" ? Math.min(width, start + 2) : naturalEnd;
      const signature = advancedSampleSignature(sample.value);
      if (!colorByValue.has(signature)) colorByValue.set(signature, colors[colorByValue.size % colors.length]);
      context.globalAlpha = 0.28;
      context.fillStyle = colorByValue.get(signature);
      context.fillRect(start, 0, Math.max(1, end - start), height);
      context.globalAlpha = 1;
      context.fillStyle = boundary;
      context.fillRect(start, 0, 1, height);
    });
  }

  function flattenSnapshot(value, path = "", rows = []) {
    if (rows.length >= 320) return rows;
    if (value && typeof value === "object" && value.redacted === true) {
      rows.push([path, text("replay_stat_redacted", "Redacted")]);
      return rows;
    }
    if (Array.isArray(value)) {
      if (!value.length) rows.push([path, "[]"]);
      value.forEach((item, index) => flattenSnapshot(item, `${path}[${index}]`, rows));
      return rows;
    }
    if (value && typeof value === "object") {
      const entries = Object.entries(value);
      if (!entries.length) rows.push([path, "{}"]);
      entries.forEach(([key, child]) => flattenSnapshot(child, path ? `${path}.${key}` : key, rows));
      return rows;
    }
    rows.push([path, value == null ? "null" : String(value)]);
    return rows;
  }

  function renderAdvancedSnapshot(payload) {
    if (!advancedSnapshotEl || !payload?.found) return;
    const rows = [
      [text("replay_snapshot_time", "Recorded time"), formatTime(Number(payload.timeMs || 0) / 1000)],
      [text("replay_snapshot_distance", "Cursor difference"), `${numberText(payload.distanceMs, 1)} ms`],
      [text("replay_snapshot_valid", "Valid"), payload.valid
        ? text("replay_value_true", "On")
        : text("replay_value_false", "Off")],
      ...flattenSnapshot(payload.snapshot, payload.service),
    ].slice(0, 320);
    advancedSnapshotEl.replaceChildren(...rows.flatMap(([key, value]) => {
      const keyEl = document.createElement("span");
      const valueEl = document.createElement("span");
      keyEl.textContent = key || payload.service;
      valueEl.textContent = value;
      return [keyEl, valueEl];
    }));
    advancedSnapshotEl.hidden = false;
  }

  function loadAdvancedSnapshot() {
    const service = String(advancedServiceEl?.value || "");
    if (!service || !dataTracks?.canQueryClientRaw?.(state.rawStats)
        || typeof dataTracks?.queryClientSnapshot !== "function") return;
    let targetSeconds = state.currentSeconds;
    try {
      const pausedAt = state.onPause?.();
      const pausedSeconds = Number(pausedAt);
      if (pausedAt != null && Number.isFinite(pausedSeconds)) targetSeconds = Math.max(0, pausedSeconds);
    } catch {}
    syncTime(targetSeconds);
    const targetMs = Math.max(0, targetSeconds * 1000);
    const token = ++state.snapshotToken;
    state.snapshotRequestController?.abort?.();
    const controller = new AbortController();
    state.snapshotRequestController = controller;
    setAdvancedStatus(text("replay_snapshot_loading", "Loading current snapshot..."), "loading");
    dataTracks.queryClientSnapshot({
      manifest: state.rawStats,
      service,
      timeMs: targetMs,
      signal: controller.signal,
    }).then((payload) => {
      if (!payload || token !== state.snapshotToken || controller.signal.aborted) return;
      if (!payload.found) {
        setAdvancedStatus(text("replay_snapshot_empty", "No snapshot was found."), "error");
        return;
      }
      state.snapshotCurrent = payload;
      renderAdvancedSnapshot(payload);
      setAdvancedStatus(text("replay_snapshot_ready", "{service} snapshot · {time}", {
        service: serviceDisplayName(service),
        time: formatTime(Number(payload.timeMs || 0) / 1000),
      }), payload.parseStatus === "partial" ? "partial" : "ready");
    }).catch((error) => {
      if (error?.name === "AbortError" || token !== state.snapshotToken) return;
      setAdvancedStatus(text("replay_snapshot_failed", "Could not load the snapshot."), "error");
    });
  }

  function renderAdvancedStatistics(track) {
    if (!advancedStatisticsEl) return;
    const stats = track?.rawStats || {};
    const numeric = stats.numeric || {};
    const integer64 = stats.integer64 || {};
    const listLength = stats.listLength || {};
    const rows = [];
    const add = (key, value, digits = 0) => {
      if (value == null || value === "" || !Number.isFinite(Number(value))) return;
      rows.push({ key, value: numberText(value, digits) });
    };
    const addExact = (key, value) => {
      if (value == null || value === "") return;
      rows.push({ key, value: String(value) });
    };
    add("replay_stat_matching_events", stats.matchingEventCount ?? track?.stats?.count);
    add("replay_stat_values", stats.extractedValueCount ?? track?.samples?.length);
    add("replay_stat_invalid", stats.invalidEventCount);
    add("replay_stat_missing", stats.missingValueCount);
    add("replay_stat_nonfinite", stats.nonfiniteCount);
    add("replay_stat_min", numeric.min, 3);
    add("replay_stat_max", numeric.max, 3);
    add("replay_stat_mean", numeric.mean, 3);
    add("replay_stat_weighted_mean", numeric.timeWeightedMean, 3);
    add("replay_stat_stddev", numeric.stddev, 3);
    add("replay_stat_rms", numeric.rms, 3);
    add("replay_stat_nondefault", numeric.nonDefaultCount);
    add("replay_stat_max_delta", numeric.maxAbsDelta, 3);
    add("replay_stat_max_rate", numeric.maxAbsRatePerSecond, 3);
    add("replay_stat_p01", numeric.p01, 3);
    add("replay_stat_p05", numeric.p05, 3);
    add("replay_stat_p50", numeric.p50, 3);
    add("replay_stat_p95", numeric.p95, 3);
    add("replay_stat_p99", numeric.p99, 3);
    add("replay_stat_distinct", stats.distinctValueCount);
    add("replay_stat_transitions", Array.isArray(stats.stateTransitions)
      ? stats.stateTransitions.reduce((sum, item) => sum + Number(item.count || 0), 0)
      : null);
    add("replay_stat_histogram_bins", numeric.histogram?.bins?.length);
    add("replay_stat_list_min", listLength.min);
    add("replay_stat_list_max", listLength.max);
    add("replay_stat_list_mean", listLength.mean, 2);
    add("replay_stat_length_min", stats.valueLength?.min);
    add("replay_stat_length_max", stats.valueLength?.max);
    add("replay_stat_length_mean", stats.valueLength?.mean, 2);
    add("replay_stat_duplicates", numeric.sequenceQuality?.duplicateCount);
    add("replay_stat_backwards", numeric.sequenceQuality?.backwardsCount);
    addExact("replay_stat_exact_min", integer64.min);
    addExact("replay_stat_exact_max", integer64.max);
    addExact("replay_stat_exact_mean", integer64.mean);
    addExact("replay_stat_exact_p50", integer64.p50);
    add("replay_stat_nondefault", integer64.nonDefaultCount);
    add("replay_stat_duplicates", integer64.sequenceQuality?.duplicateCount);
    add("replay_stat_backwards", integer64.sequenceQuality?.backwardsCount);
    addExact("replay_stat_max_step", integer64.sequenceQuality?.maxStep);
    if (track?.redacted) rows.push({ key: "replay_stat_privacy", value: text("replay_stat_redacted", "Redacted") });
    advancedStatisticsEl.replaceChildren(...rows.map((row) => {
      const item = document.createElement("span");
      const value = document.createElement("b");
      const label = document.createElement("small");
      value.textContent = row.value;
      label.textContent = text(row.key, row.key);
      item.append(value, label);
      return item;
    }));
    advancedStatisticsEl.hidden = !rows.length;
    if (advancedStatisticsDetailsEl) advancedStatisticsDetailsEl.hidden = !rows.length;
  }

  function renderAdvancedTrack() {
    const track = state.advancedTrack;
    if (!track || !advancedTrackEl) return;
    const renderer = rendererRegistry?.resolve?.(track.renderer, track.kind) || {
      key: track.renderer || "raw",
      mode: track.kind === "number" ? "line" : "bands",
    };
    renderAdvancedStatistics(track);
    advancedTrackEl.hidden = renderer.interactiveSeries === false;
    advancedTrackEl.dataset.kind = renderer.mode === "line"
      ? "number"
      : renderer.mode === "bands" || renderer.mode === "markers"
        ? "state"
        : "summary";
    advancedTrackEl.dataset.renderer = renderer.key;
    advancedTrackEl.dataset.renderMode = renderer.mode;
    const graphRenderer = track.kind === "number" && renderer.mode === "line";
    if (graphRenderer) {
      if (advancedMinEl) advancedMinEl.textContent = advancedValue(track.stats.min, track.field, 2);
      if (advancedMaxEl) advancedMaxEl.textContent = advancedValue(track.stats.max, track.field, 2);
      if (advancedAverageEl) advancedAverageEl.textContent = advancedValue(track.stats.average, track.field, 2);
    } else {
      [advancedMinEl, advancedMaxEl, advancedAverageEl].forEach((element) => { if (element) element.textContent = "--"; });
    }
    window.requestAnimationFrame(() => drawAdvancedTrack(track));
    const ready = text("replay_advanced_ready", "{count} samples ready", {
      count: numberText(track.samples.length, 0),
    });
    setAdvancedStatus(
      track.parseStatus === "partial" ? `${ready} · ${text("replay_advanced_partial", "Partial log")}` : ready,
      track.parseStatus === "partial" ? "partial" : "ready",
    );
  }

  function syncAdvancedCurrent() {
    const track = state.advancedTrack;
    if (!track) return;
    const sample = dataTracks?.sampleAt?.(track.samples, state.currentSeconds * 1000);
    if (advancedCurrentEl) advancedCurrentEl.textContent = sample ? advancedValue(sample.value, track.field) : "--";
    if (advancedCursorEl) {
      advancedCursorEl.style.left = `${state.durationSeconds > 0 ? Math.max(0, Math.min(100, state.currentSeconds / state.durationSeconds * 100)) : 0}%`;
    }
  }

  function nearestEvent(seconds) {
    if (!state.events.length) return null;
    const timeMs = seconds * 1000;
    let low = 0;
    let high = state.events.length - 1;
    while (low < high) {
      const middle = Math.floor((low + high) / 2);
      if (state.events[middle].timeMs < timeMs) low = middle + 1;
      else high = middle;
    }
    const current = state.events[low];
    const prior = state.events[Math.max(0, low - 1)];
    return Math.abs(prior.timeMs - timeMs) <= Math.abs(current.timeMs - timeMs) ? prior : current;
  }

  function previewEventAt(seconds, toleranceSeconds = 3) {
    const event = nearestEvent(seconds);
    const tolerance = Math.max(0.1, Number(toleranceSeconds) || 3);
    if (!event || Math.abs(event.timeMs / 1000 - Number(seconds || 0)) > tolerance) return null;
    return {
      id: event.id,
      category: event.category,
      title: eventDisplayTitle(event),
      meta: eventDetail(event)
        ? `${formatTime(event.timeMs / 1000)} · ${eventDetail(event)}`
        : formatTime(event.timeMs / 1000),
      timeSeconds: event.timeMs / 1000,
    };
  }

  function setPreviewVisible(visible) {
    const next = Boolean(visible);
    if (state.previewVisible === next) return;
    state.previewVisible = next;
    if (next) clearSummary();
    else syncTime(state.currentSeconds);
  }

  function flushSyncedTime() {
    if (!state.active) return false;
    if (state.open && state.activeTab === "graphs") {
      graphEls.forEach((_element, name) => syncGraphValue(name, state.currentSeconds));
    } else if (state.open && state.activeTab === "sensors") {
      sensorTopview?.syncTime?.(state.currentSeconds);
    } else if (state.open && state.activeTab === "advanced") {
      syncAdvancedCurrent();
    }
    const reviewEvent = selectedReviewEventAt(state.currentSeconds);
    clearSelectionOutsideReview(reviewEvent);
    const currentEvent = stabilizedPlaybackEvent(
      reviewEvent || timelineEventAt(state.currentSeconds),
      state.currentSeconds,
    );
    syncCurrentEvent(currentEvent);
    syncVerticalTimeline(state.currentSeconds);
    const currentAgeSeconds = currentEvent ? state.currentSeconds - currentEvent.timeMs / 1000 : Infinity;
    if (currentEvent && currentAgeSeconds >= 0 && currentAgeSeconds <= POLICY.summaryWindowSeconds) showSummary(currentEvent);
    else clearSummary();
    return true;
  }

  function syncTime(seconds, durationSeconds = state.durationSeconds, options = {}) {
    if (!state.active) return false;
    state.currentSeconds = Math.max(0, Number(seconds) || 0);
    if (Number.isFinite(Number(durationSeconds)) && Number(durationSeconds) > 0) {
      state.durationSeconds = Number(durationSeconds);
    }
    if (!playbackSurfaceScheduler) return flushSyncedTime();
    playbackSurfaceScheduler.request(POLICY.playbackSurfaceCadenceMs, options);
    return true;
  }

  function syncLabels() {
    const assignments = [
      ["carrotReplayInsightsEventsTab", "replay_events", "Events"],
      ["carrotReplayInsightsGraphsTab", "replay_graphs", "Graphs"],
      ["carrotReplayInsightsSensorsTab", "replay_sensors", "Forward"],
      ["carrotReplayInsightsAdvancedTab", "replay_advanced", "Analysis"],
      ["carrotReplayEventEmpty", "replay_no_events", "No major events."],
      ["btnCarrotReplayEventFollow", "replay_events_follow", "Jump to current"],
      ["carrotReplaySpeedGraphLabel", "replay_graph_speed", "Speed"],
      ["carrotReplaySteerGraphLabel", "replay_graph_steering", "Steering"],
      ["carrotReplayAccelGraphLabel", "replay_graph_acceleration", "Acceleration"],
      ["carrotReplayLeadGraphLabel", "replay_graph_lead_distance", "Lead distance"],
      ["carrotReplayAdvancedServiceLabel", "replay_advanced_service", "Data type"],
      ["carrotReplayAdvancedFieldLabel", "replay_advanced_field", "Measurement"],
      ["carrotReplayAdvancedCurrentLabel", "replay_advanced_current", "Current value"],
      ["carrotReplayAdvancedMinLabel", "replay_advanced_minimum", "Minimum"],
      ["carrotReplayAdvancedMaxLabel", "replay_advanced_maximum", "Maximum"],
      ["carrotReplayAdvancedAverageLabel", "replay_advanced_average", "Average"],
      ["carrotReplayInsightsRecordCountLabel", "replay_advanced_records", "Records"],
      ["carrotReplayInsightsServiceCountLabel", "replay_advanced_services", "Services"],
      ["carrotReplayInsightsEventCountLabel", "replay_events", "Events"],
      ["carrotReplayInsightsGraphCountLabel", "replay_advanced_samples", "Samples"],
      ["carrotReplayInsightsInvalidCountLabel", "replay_advanced_invalid", "Invalid"],
      ["carrotReplayAdvancedHelp", "replay_advanced_help", "Only the selected measurement is analyzed on this device."],
      ["btnCarrotReplayAdvancedSnapshot", "replay_snapshot_button", "View current raw values"],
      ["carrotReplayAdvancedStatisticsTitle", "replay_advanced_statistics", "Detailed statistics"],
      ["carrotReplayAdvancedStatisticsHint", "replay_advanced_statistics_hint", "Distribution and data quality"],
      ["carrotReplayAdvancedSourceTitle", "replay_advanced_source_info", "Recording information"],
      ["carrotReplayAdvancedSourceHint", "replay_advanced_source_hint", "Coverage and available data"],
    ];
    assignments.forEach(([id, key, fallback]) => {
      const element = document.getElementById(id);
      if (element) element.textContent = text(key, fallback);
    });
    const detailsLabel = text("replay_details", "Details");
    rootEl?.setAttribute("aria-label", detailsLabel);
    railEl?.setAttribute("aria-label", text("replay_events", "Events"));
    rootEl?.querySelector(".carrot-replay-insights__tabs")?.setAttribute("aria-label", detailsLabel);
    const graphLabels = {
      speed: text("replay_graph_speed", "Speed"),
      steer: text("replay_graph_steering", "Steering"),
      accel: text("replay_graph_acceleration", "Acceleration"),
      lead: text("replay_graph_lead_distance", "Lead distance"),
    };
    graphEls.forEach((graphEl, name) => graphEl.setAttribute("aria-label", graphLabels[name] || name));
    const selection = selectedAdvancedEntry();
    advancedTrackEl?.setAttribute(
      "aria-label",
      selection
        ? `${fieldDisplayName(selection.serviceEntry.service, selection.field)} ${text("replay_graphs", "Graph")}`
        : text("replay_advanced", "Analysis"),
    );
    renderEventList();
    renderRail();
    renderAdvancedCatalog();
    if (state.advancedTrack) renderAdvancedTrack();
    if (state.snapshotCurrent) renderAdvancedSnapshot(state.snapshotCurrent);
    if (TEMPORARY_TELEMETRY_SURFACES.replayForward) sensorTopview?.syncLabels?.();
    syncCounts();
    syncTime(state.currentSeconds);
  }

  function load(options = {}) {
    const records = Array.isArray(options.records) ? options.records : [];
    const decodeRecord = typeof options.decodeRecord === "function" ? options.decodeRecord : () => [];
    const token = ++state.token;
    resetData();
    state.active = true;
    state.durationSeconds = Math.max(0, Number(options.durationMs || 0) / 1000);
    state.onSeek = typeof options.onSeek === "function" ? options.onSeek : null;
    state.onPause = typeof options.onPause === "function" ? options.onPause : null;
    state.onUpdate = typeof options.onUpdate === "function" ? options.onUpdate : null;
    state.onReady = typeof options.onReady === "function" ? options.onReady : null;
    state.records = records;
    state.decodeRecord = decodeRecord;
    state.rawStats = options.manifest && typeof options.manifest === "object" ? options.manifest : null;
    if (TEMPORARY_TELEMETRY_SURFACES.replayForward) {
      sensorTopview?.load?.({ records, decodeRecord, manifest: state.rawStats });
    }
    selectTab(state.activeTab);
    const hasServerEventIndex = Array.isArray(options.events) || Array.isArray(state.rawStats?.eventIndex);
    const serverEvents = Array.isArray(options.events) ? options.events : state.rawStats?.eventIndex;
    if (hasServerEventIndex) {
      state.eventSource = "index";
      syncEventSourceNotice();
      state.events = serverEvents
        .filter((event) => event && Number.isFinite(Number(event.timeMs)) && isUserVisibleReplayEvent(event))
        .map((event, index) => ({
          id: String(event.id || `event-${Math.round(Number(event.timeMs) || 0)}-${index}`),
          timeMs: Math.max(0, Number(event.timeMs) || 0),
          category: String(event.category || "event"),
          type: String(event.type || "unknown"),
          params: event.params && typeof event.params === "object" ? { ...event.params } : {},
          sourceTitle: String(event.sourceTitle || "").trim(),
          sourceDetail: String(event.sourceDetail || "").trim(),
          sourceTag: String(event.sourceTag || "").trim(),
        }))
        .sort((left, right) => left.timeMs - right.timeMs);
      state.services = new Set([
        ...Object.keys(state.rawStats?.rawServiceStats || {}),
        ...Object.keys(state.rawStats?.services || {}),
      ]);
      state.recordsProcessed = records.length;
      state.indexingProgress = 1;
      state.indexing = false;
      setActive(true, { open: true });
      syncLabels();
      renderAll();
      selectTab(state.activeTab);
      state.onUpdate?.();
      syncTime(state.currentSeconds);
      signalReady();
      return;
    }
    // Fallback path. The log worker always emits an eventIndex, so reaching here
    // means that index is missing (stale manifest, worker failure). The fallback
    // indexer covers only selfdriveState/lateralPlan/carrotMan and deliberately
    // skips carState/radarState, so it yields a strictly smaller event set than
    // the worker's. That difference must be visible rather than look like a
    // drive that simply had fewer events.
    state.eventSource = "fallback";
    syncEventSourceNotice();
    state.indexing = true;
    const indexer = model?.createIndexer?.();
    if (indexer) {
      state.events = indexer.events;
      state.services = indexer.services;
    }
    syncLabels();
    let index = 0;

    const processChunk = (deadline) => {
      if (token !== state.token || !state.active) return;
      const end = Math.min(records.length, index + POLICY.indexChunkSize);
      const chunkStart = index;
      while (index < end && (index - chunkStart < 80 || deadline.timeRemaining() > 1)) {
        const record = records[index];
        try {
          for (const frame of decodeRecord(record)) indexer?.ingest?.(frame, Number(record?.timeMs || 0));
        } catch {}
        index += 1;
      }
      state.recordsProcessed = index;
      state.indexingProgress = records.length ? index / records.length : 1;
      syncCounts();
      if (index < records.length) return scheduleWork(processChunk);
      state.indexing = false;
      setActive(true, { open: true });
      renderAll();
      selectTab(state.activeTab);
      state.onUpdate?.();
      syncTime(state.currentSeconds);
      signalReady();
    };
    scheduleWork(processChunk);
  }

  function seekFromTrack(event, element) {
    if (!state.active || !state.onSeek || state.durationSeconds <= 0) return;
    const rect = element.getBoundingClientRect();
    const ratio = Math.max(0, Math.min(1, (event.clientX - rect.left) / Math.max(1, rect.width)));
    state.onSeek(ratio * state.durationSeconds);
  }

  function bindSeekSurface(element) {
    if (!element) return;
    const touchGesture = { pointerId: null, startX: 0, startY: 0, mode: "idle" };
    const resetTouchGesture = (pointerId = null) => {
      if (pointerId != null && touchGesture.pointerId !== pointerId) return;
      touchGesture.pointerId = null;
      touchGesture.mode = "idle";
    };
    const suppressNativeDrag = (event) => event.preventDefault();
    element.setAttribute("draggable", "false");
    element.addEventListener("dragstart", suppressNativeDrag);
    element.addEventListener("selectstart", suppressNativeDrag);
    element.addEventListener("pointerdown", (event) => {
      if (event.isPrimary === false || event.button !== 0) return;
      if (event.target instanceof Element && event.target.closest("button, input, select, a")) return;
      if (event.pointerType === "touch") {
        touchGesture.pointerId = event.pointerId;
        touchGesture.startX = event.clientX;
        touchGesture.startY = event.clientY;
        touchGesture.mode = "pending";
        return;
      }
      event.preventDefault();
      try { element.setPointerCapture(event.pointerId); } catch {}
      seekFromTrack(event, element);
    }, { passive: false });
    element.addEventListener("pointermove", (event) => {
      if (event.pointerType === "touch" && touchGesture.pointerId === event.pointerId) {
        const deltaX = event.clientX - touchGesture.startX;
        const deltaY = event.clientY - touchGesture.startY;
        if (touchGesture.mode === "pending") {
          if (Math.hypot(deltaX, deltaY) < 8) return;
          if (Math.abs(deltaY) >= Math.abs(deltaX) * 0.9) {
            touchGesture.mode = "scroll";
            return;
          }
          touchGesture.mode = "seek";
          try { element.setPointerCapture(event.pointerId); } catch {}
        }
        if (touchGesture.mode !== "seek") return;
        event.preventDefault();
        seekFromTrack(event, element);
        return;
      }
      if (!element.hasPointerCapture?.(event.pointerId)) return;
      event.preventDefault();
      seekFromTrack(event, element);
    }, { passive: false });
    element.addEventListener("pointerup", (event) => {
      if (event.pointerType !== "touch" || touchGesture.pointerId !== event.pointerId) return;
      if (touchGesture.mode === "pending") seekFromTrack(event, element);
      resetTouchGesture(event.pointerId);
    }, { passive: true });
    element.addEventListener("pointercancel", (event) => resetTouchGesture(event.pointerId), { passive: true });
  }

  tabEls.forEach((element) => {
    element.addEventListener("click", () => selectTab(element.dataset.replayInsightsTab, { persist: true }));
  });
  const tabRoot = rootEl?.querySelector?.(".carrot-replay-insights__tabs");
  if (tabRoot) {
    tabControl = createSegmentedControl(tabRoot, {
      itemSelector: "[data-replay-insights-tab]",
      selectedAttribute: "aria-selected",
      activation: "automatic",
      onActivate(element) {
        selectTab(element?.dataset?.replayInsightsTab, { persist: true });
      },
    });
  }
  graphEls.forEach((graphEl) => bindSeekSurface(graphEl));
  bindSeekSurface(advancedTrackEl);
  eventsViewport()?.addEventListener("scroll", handleEventListScroll, { passive: true });
  followButtonEl?.addEventListener("click", () => setFollowEvents(true));

  advancedServiceEl?.addEventListener("change", () => syncAdvancedFields(true));
  advancedFieldEl?.addEventListener("change", () => {
    syncAdvancedSelectionMeta();
    ensureAdvancedTrack();
  });
  advancedSnapshotButtonEl?.addEventListener("click", loadAdvancedSnapshot);
  if (railEl && typeof ResizeObserver === "function") {
    state.resizeObserver = new ResizeObserver(scheduleContainerLayout);
    state.resizeObserver.observe(railEl);
    if (stageEl) state.resizeObserver.observe(stageEl);
    if (auxPaneEl) state.resizeObserver.observe(auxPaneEl);
  }
  window.addEventListener("resize", scheduleContainerLayout, { passive: true });
  window.addEventListener("orientationchange", scheduleContainerLayout, { passive: true });
  document.addEventListener("pointerdown", (event) => {
    if (clusterEl?.hidden || clusterEl?.contains(event.target) || railEl?.contains(event.target)) return;
    closeCluster();
  }, true);
  window.addEventListener("carrot:languagechange", syncLabels);

  function isOpen() {
    return state.open || Boolean(clusterEl && !clusterEl.hidden);
  }

  function hasOpenPopover() {
    return Boolean(clusterEl && !clusterEl.hidden);
  }

  reset();
  syncLabels();
  return {
    load,
    reset,
    setActive,
    syncTime,
    syncLabels,
    isOpen,
    hasOpenPopover,
    selectTimelineAt,
    previewEventsAt,
    previewEventAt,
    setPreviewVisible,
    selectEventById,
    renderStatus: () => playbackSurfaceScheduler?.status?.() || null,
    eventCount: () => state.events.length,
  };
})();
