import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import test from "node:test";
import { fileURLToPath } from "node:url";

const webRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const carrotRoot = path.resolve(webRoot, "..");
const pythonSource = fs.readFileSync(path.join(carrotRoot, "realtime", "compact_state.py"), "utf8");
const nativeSource = fs.readFileSync(path.join(carrotRoot, "realtime", "compact_state_native.cc"), "utf8");
const browserSource = fs.readFileSync(path.join(webRoot, "js", "realtime", "vision_compact.js"), "utf8");

const arContract = {
  carState: [1, [
    "vEgo", "aEgo", "vEgoCluster", "vCruiseCluster", "steeringAngleDeg",
    "brakeHoldActive", "softHoldActive", "carrotCruise", "gearStep", "useLaneLineSpeed",
    "brakeLights", "leftBlindspot", "rightBlindspot", "leftLaneLine", "rightLaneLine",
    "gearShifter", "leftBlinker", "rightBlinker", "fuelGauge", "ureaGauge", "tpms",
    // Appended for the cluster-parity EV telltale (kept at the end).
    "evModeValid", "evModeActive",
  ]],
  selfdriveState: [6, [
    "enabled", "personality", "alertStatus", "alertSize", "alertType", "alertText1", "alertText2",
  ]],
  gpsLocationExternal: [7, [
    "latitude", "longitude", "speed", "bearingDeg", "bearingAccuracyDeg", "speedAccuracy",
    "hasFix", "altitude", "horizontalAccuracy", "verticalAccuracy", "unixTimestampMillis",
  ]],
  modelV2: [9, [
    "frameId", "frameIdExtra", "position", "velocity", "laneLines", "laneLineProbs",
    "roadEdges", "roadEdgeStds", "leadsV3", "laneLineStds", "frameAge",
    "frameDropPerc", "modelExecutionTime",
  ]],
  liveCalibration: [10, ["calStatus", "calCycle", "calPerc", "validBlocks", "rpyCalib", "height"]],
  roadCameraState: [11, ["frameId", "sensor", "timestampEof"]],
  radarState: [13, [
    "leadOne", "leadTwo", "leadRight", "leadLeft", "leadsLeft", "leadsCenter", "leadsRight",
    "leadsLeft2", "leadsRight2", "leadsCutIn",
  ]],
  cameraOdometry: [19, ["frameId", "timestampEof", "trans", "rot", "transStd", "rotStd"]],
  livePose: [20, [
    "orientationNED", "velocityDevice", "accelerationDevice", "angularVelocityDevice",
    "inputsOK", "posenetOK", "sensorsOK", "timestamp",
  ]],
  carrotNavi: [21, [
    "schemaVersion", "generation", "sessionId", "publishMonoTimeNanos", "connected", "vehicle",
    "guidanceCurrent", "guidanceNext", "laneCurrent", "laneAhead", "speed", "trafficSignal", "crossroad",
    "route", "navigationStatus",
  ]],
};

const nestedContract = {
  NAVI_GUIDANCE_SCHEMA: ["present", "distanceM", "timeSec", "turnType", "roadName", "mainText", "pointValid", "latitude", "longitude"],
  NAVI_VEHICLE_SCHEMA: ["present", "latitude", "longitude", "headingDeg", "speedKph", "roadName"],
  NAVI_LANE_SCHEMA: ["present", "count", "distanceM", "visible", "available"],
  NAVI_SPEED_SCHEMA: [
    "roadLimitValid", "roadLimitKph", "sdiPresent", "sdiType", "sdiDistanceM", "sdiSpeedLimitKph",
    "sdiSectionType", "sdiBlockType", "sdiBlockSpeedKph", "sdiBlockDistanceM",
    "secondarySdiPresent", "secondarySdiType", "secondarySdiDistanceM", "secondarySdiSpeedLimitKph",
    "secondarySdiSectionType", "secondarySdiBlockType", "secondarySdiBlockSpeedKph", "secondarySdiBlockDistanceM",
    "sectionPresent", "sectionActive", "sectionSpeedLimitKph", "sectionAverageKph",
    "sectionOverallAverageKph", "sectionRemainingDistanceM", "sectionRemainingTimeSec", "sectionProgress",
    "sectionSuspended", "sectionOffRoute",
  ],
  NAVI_SIGNAL_SCHEMA: [
    "visible", "distanceM", "redValid", "redOn", "redRemainSec", "leftValid", "leftOn", "leftRemainSec",
    "greenValid", "greenOn", "greenRemainSec", "rightValid", "rightOn", "rightRemainSec",
    "uturnValid", "uturnOn", "uturnRemainSec", "uiCounterValid", "uiCounterRemainSec",
  ],
  NAVI_CROSSROAD_SCHEMA: ["visible", "distanceM", "imageCode"],
  NAVI_ROUTE_SCHEMA: ["present", "remainingDistanceM", "remainingTimeSec", "movedDistanceM", "totalDistanceM", "polyline"],
  NAVI_STATUS_SCHEMA: ["guidanceActive", "offRoute", "routePresent"],
  XYZ_MEASUREMENT_SCHEMA: ["x", "y", "z", "xStd", "yStd", "zStd", "valid"],
};

const browserNestedNames = {
  NAVI_GUIDANCE_SCHEMA: "naviGuidance",
  NAVI_VEHICLE_SCHEMA: "naviVehicle",
  NAVI_LANE_SCHEMA: "naviLane",
  NAVI_SPEED_SCHEMA: "naviSpeed",
  NAVI_SIGNAL_SCHEMA: "naviSignal",
  NAVI_CROSSROAD_SCHEMA: "naviCrossroad",
  NAVI_ROUTE_SCHEMA: "naviRoute",
  NAVI_STATUS_SCHEMA: "naviStatus",
  XYZ_MEASUREMENT_SCHEMA: "xyzMeasurement",
};

function pythonDeclarationFields(source) {
  const wireTypes = new Set([
    "bool", "i8", "u8", "i16", "u16", "i32", "u32", "u64", "f32", "f64", "text",
    "f32_list", "f32_first_list", "i16_list", "coord_list", "u16_cm_list", "i16_cm_list", "i16_mm_list",
  ]);
  return Array.from(source.matchAll(/\("([A-Za-z][A-Za-z0-9]*)",\s*"([a-z0-9_]+)"/g))
    .filter((match) => wireTypes.has(match[2]))
    .map((match) => match[1]);
}

function browserDeclarationFields(source) {
  const wireTypes = new Set([
    "bool", "i8", "u8", "i16", "u16", "i32", "u32", "u64", "f32", "f64", "text",
    "enumname", "f32list", "i16list", "coordlist", "u16cmlist", "i16cmlist", "i16mmlist",
    "struct", "structlist",
  ]);
  return Array.from(source.matchAll(/\["([A-Za-z][A-Za-z0-9]*)",\s*"([a-z0-9]+)"/g))
    .filter((match) => wireTypes.has(match[2]))
    .map((match) => match[1]);
}

function pythonServiceBlocks() {
  const markers = Array.from(pythonSource.matchAll(/^  "([A-Za-z0-9]+)": \((\d+), \($/gm));
  return new Map(markers.map((marker, index) => {
    const end = markers[index + 1]?.index ?? pythonSource.indexOf("\n}\n", marker.index);
    return [marker[1], {
      id: Number(marker[2]),
      fields: Array.from(pythonSource.slice(marker.index, end).matchAll(/^    \("([A-Za-z][A-Za-z0-9]*)"/gm), (match) => match[1]),
    }];
  }));
}

function browserServiceBlocks() {
  const markers = Array.from(browserSource.matchAll(/^    \[(\d+), \["([A-Za-z0-9]+)", \[$/gm));
  return new Map(markers.map((marker, index) => {
    const end = markers[index + 1]?.index ?? browserSource.indexOf("\n  ]);", marker.index);
    return [marker[2], {
      id: Number(marker[1]),
      fields: browserDeclarationFields(browserSource.slice(marker.index, end)),
    }];
  }));
}

function declarationBody(source, declaration) {
  const pythonDeclaration = declaration.endsWith("_SCHEMA");
  const startPattern = pythonDeclaration
    ? new RegExp(`^${declaration} = \\($`, "m")
    : new RegExp(`^  const ${declaration} = \\[`, "m");
  const start = source.search(startPattern);
  assert.notEqual(start, -1, `missing declaration ${declaration}`);
  if (pythonDeclaration) {
    const end = source.indexOf("\n)", start);
    assert.notEqual(end, -1, `unterminated declaration ${declaration}`);
    return source.slice(start, end);
  }

  const open = source.indexOf("[", start);
  let depth = 0;
  for (let index = open; index < source.length; index += 1) {
    if (source[index] === "[") depth += 1;
    if (source[index] === "]") depth -= 1;
    if (depth === 0) return source.slice(start, index + 1);
  }
  assert.fail(`unterminated declaration ${declaration}`);
}

function functionBody(source, name) {
  const signature = new RegExp(`(?:void|uint8_t) ${name}\\(`);
  const start = source.search(signature);
  assert.notEqual(start, -1, `missing native function ${name}`);
  const open = source.indexOf("{", start);
  let depth = 0;
  for (let index = open; index < source.length; index += 1) {
    if (source[index] === "{") depth += 1;
    if (source[index] === "}") depth -= 1;
    if (depth === 0) return source.slice(start, index + 1);
  }
  assert.fail(`unterminated native function ${name}`);
}

function assertTokensInOrder(source, tokens, label) {
  let cursor = 0;
  for (const token of tokens) {
    const index = source.indexOf(`"${token}"`, cursor);
    assert.notEqual(index, -1, `${label}: missing or reordered ${token}`);
    cursor = index + token.length + 2;
  }
}

test("AR compact service IDs stay identical in Python, native, and browser codecs", () => {
  const python = pythonServiceBlocks();
  const browser = browserServiceBlocks();
  const nativeIds = new Map(Array.from(
    functionBody(nativeSource, "service_id").matchAll(/service == "([A-Za-z0-9]+)"\) return (\d+);/g),
    (match) => [match[1], Number(match[2])],
  ));

  for (const [service, [id]] of Object.entries(arContract)) {
    assert.equal(python.get(service)?.id, id, `Python ID drift: ${service}`);
    assert.equal(nativeIds.get(service), id, `native ID drift: ${service}`);
    assert.equal(browser.get(service)?.id, id, `browser ID drift: ${service}`);
  }
});

test("AR compact top-level field order stays identical in Python and browser codecs", () => {
  const python = pythonServiceBlocks();
  const browser = browserServiceBlocks();
  for (const [service, [, fields]] of Object.entries(arContract)) {
    assert.deepEqual(python.get(service)?.fields, fields, `Python field drift: ${service}`);
    assert.deepEqual(browser.get(service)?.fields, fields, `browser field drift: ${service}`);
  }
});

test("AR nested pose and navigation schemas stay identical in Python and browser codecs", () => {
  for (const [pythonName, fields] of Object.entries(nestedContract)) {
    const browserName = browserNestedNames[pythonName];
    assert.deepEqual(pythonDeclarationFields(declarationBody(pythonSource, pythonName)), fields, `Python schema drift: ${pythonName}`);
    assert.deepEqual(browserDeclarationFields(declarationBody(browserSource, browserName)), fields, `browser schema drift: ${browserName}`);
  }
});

test("native AR encoders retain the shared field sequence and route limit", () => {
  const directEncoders = {
    encode_car_state: arContract.carState[1].flatMap((field) => field === "tpms" ? ["tpms", "fl", "tpms", "fr", "tpms", "rl", "tpms", "rr"] : [field]),
    encode_selfdrive_state: arContract.selfdriveState[1],
    encode_gps: arContract.gpsLocationExternal[1],
    encode_model_v2: arContract.modelV2[1].filter((field) => field !== "leadsV3"),
    encode_live_calibration: arContract.liveCalibration[1],
    encode_radar_state: arContract.radarState[1],
    encode_camera_odometry: arContract.cameraOdometry[1],
    encode_live_pose: arContract.livePose[1],
  };
  for (const [encoder, fields] of Object.entries(directEncoders)) {
    assertTokensInOrder(functionBody(nativeSource, encoder), fields, encoder);
  }

  const model = functionBody(nativeSource, "encode_model_v2");
  assert.ok(
    model.indexOf('"roadEdgeStds"') < model.indexOf("append_model_leads(out, value)")
      && model.indexOf("append_model_leads(out, value)") < model.indexOf('"laneLineStds"'),
    "encode_model_v2: leadsV3 helper moved out of wire order",
  );
  assert.match(functionBody(nativeSource, "append_model_leads"), /find_value\(reader, "leadsV3"\)/);

  assertTokensInOrder(functionBody(nativeSource, "encode_service"), arContract.roadCameraState[1], "roadCameraState");
  const navi = functionBody(nativeSource, "encode_carrot_navi");
  assertTokensInOrder(navi, arContract.carrotNavi[1], "encode_carrot_navi");
  assertTokensInOrder(functionBody(nativeSource, "append_navi_guidance"), nestedContract.NAVI_GUIDANCE_SCHEMA.slice(1), "native guidance");
  assertTokensInOrder(navi, nestedContract.NAVI_VEHICLE_SCHEMA.slice(1), "native vehicle");
  assertTokensInOrder(functionBody(nativeSource, "append_navi_lane_value"), nestedContract.NAVI_LANE_SCHEMA.slice(1), "native lane");
  assert.match(functionBody(nativeSource, "append_navi_lane_list"), /append_navi_lane_value/);
  assertTokensInOrder(navi, nestedContract.NAVI_SPEED_SCHEMA, "native speed");
  assertTokensInOrder(navi, nestedContract.NAVI_SIGNAL_SCHEMA, "native signal");
  assertTokensInOrder(navi, nestedContract.NAVI_CROSSROAD_SCHEMA, "native crossroad");
  assertTokensInOrder(navi, nestedContract.NAVI_ROUTE_SCHEMA.slice(1), "native route");
  assertTokensInOrder(navi, nestedContract.NAVI_STATUS_SCHEMA, "native status");
  assertTokensInOrder(functionBody(nativeSource, "append_xyz_measurement"), nestedContract.XYZ_MEASUREMENT_SCHEMA, "native pose measurement");

  assert.match(pythonSource, /^ROUTE_POLYLINE_LIMIT = 64$/m);
  assert.match(nativeSource, /^constexpr size_t kRoutePolylineLimit = 64;$/m);
  assert.match(browserSource, /const AR_SERVICES = \["cameraOdometry", "livePose", "carrotNavi"\];/);
});
