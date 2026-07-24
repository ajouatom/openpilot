"use strict";

window.CarrotVisionCompact = (() => {
  const decoder = new TextDecoder();
  const MAGIC = [0x43, 0x56, 0x53, 0x31]; // CVS1
  const BATCH_MAGIC = [0x43, 0x56, 0x42, 0x31]; // CVB1
  const HUD_SERVICES = [
    "carState", "controlsState", "deviceState", "peripheralState",
    "carrotMan", "selfdriveState", "gpsLocationExternal", "longitudinalPlan",
  ];
  const OVERLAY_SERVICES = [
    "modelV2", "liveCalibration", "roadCameraState", "lateralPlan",
    "radarState", "carControl", "liveDelay", "liveTorqueParameters", "liveParameters",
  ];
  // Kept out of OVERLAY_SERVICES so the always-on overlay group does not pay
  // for a full radar track list. Decoded frames still land in the overlay
  // state; only the subscription is opt-in via the "tracks" activity channel.
  const TRACK_SERVICES = ["liveTracks"];
  // AR 앵커 입력. 20Hz 라 "ar" 임대가 있을 때만 구독한다.
  const AR_SERVICES = ["cameraOdometry", "livePose", "carrotNavi"];

  const xyz = [
    ["x", "u16cmlist"],
    ["y", "i16mmlist"],
    ["z", "i16mmlist"],
  ];
  const velocity = [["x", "i16cmlist"]];
  // y/v arrive as single-sample lists (see MODEL_LEAD_SCHEMA); the wire shape
  // is identical to any other f32 list, so "f32list" reads them unchanged.
  const modelLead = [
    ["prob", "f32"],
    ["x", "u16cmlist"],
    ["y", "f32list"],
    ["v", "f32list"],
  ];
  const xyzMeasurement = [
    ["x", "f32"], ["y", "f32"], ["z", "f32"],
    ["xStd", "f32"], ["yStd", "f32"], ["zStd", "f32"], ["valid", "bool"],
  ];
  const naviGuidance = [
    ["present","bool"], ["distanceM","i32"], ["timeSec","i32"], ["turnType","i32"],
    ["roadName","text"], ["mainText","text"], ["pointValid","bool"],
    ["latitude","f64"], ["longitude","f64"],
  ];
  const naviVehicle = [
    ["present","bool"], ["latitude","f64"], ["longitude","f64"],
    ["headingDeg","f32"], ["speedKph","f32"], ["roadName","text"],
  ];
  const naviLane = [
    ["present","bool"], ["count","i16"], ["distanceM","i32"],
    ["visible","bool"], ["available","i16list"],
  ];
  const naviSpeed = [
    ["roadLimitValid","bool"], ["roadLimitKph","i16"],
    ["sdiPresent","bool"], ["sdiType","i32"], ["sdiDistanceM","i32"], ["sdiSpeedLimitKph","i16"],
    ["sdiSectionType","i32"], ["sdiBlockType","i32"], ["sdiBlockSpeedKph","i16"], ["sdiBlockDistanceM","i32"],
    ["secondarySdiPresent","bool"], ["secondarySdiType","i32"], ["secondarySdiDistanceM","i32"],
    ["secondarySdiSpeedLimitKph","i16"], ["secondarySdiSectionType","i32"], ["secondarySdiBlockType","i32"],
    ["secondarySdiBlockSpeedKph","i16"], ["secondarySdiBlockDistanceM","i32"],
    ["sectionPresent","bool"], ["sectionActive","bool"], ["sectionSpeedLimitKph","i16"],
    ["sectionAverageKph","f32"], ["sectionOverallAverageKph","f32"], ["sectionRemainingDistanceM","f32"],
    ["sectionRemainingTimeSec","i32"], ["sectionProgress","f32"], ["sectionSuspended","bool"], ["sectionOffRoute","bool"],
  ];
  const naviSignal = [
    ["visible","bool"], ["distanceM","i32"],
    ["redValid","bool"], ["redOn","bool"], ["redRemainSec","i16"],
    ["leftValid","bool"], ["leftOn","bool"], ["leftRemainSec","i16"],
    ["greenValid","bool"], ["greenOn","bool"], ["greenRemainSec","i16"],
    ["rightValid","bool"], ["rightOn","bool"], ["rightRemainSec","i16"],
    ["uturnValid","bool"], ["uturnOn","bool"], ["uturnRemainSec","i16"],
    ["uiCounterValid","bool"], ["uiCounterRemainSec","i16"],
  ];
  const naviCrossroad = [["visible","bool"], ["distanceM","i32"], ["imageCode","i32"]];
  const naviRoute = [
    ["present","bool"], ["remainingDistanceM","i32"],
    ["remainingTimeSec","i32"], ["movedDistanceM","i32"], ["totalDistanceM","i32"],
    ["polyline","coordlist"],
  ];
  const naviStatus = [["guidanceActive","bool"], ["offRoute","bool"], ["routePresent","bool"]];
  const radarPoint = [
    ["trackId", "u32"], ["dRel", "f32"], ["yRel", "f32"], ["vRel", "f32"],
    ["measured", "bool"],
    ["radarSource", "enumname", ["frontRadar", "scc", "corner235", "corner180", "corner430"]],
  ];
  const radarLead = [
    ["dRel", "f32"], ["yRel", "f32"], ["vRel", "f32"], ["aRel", "f32"],
    ["vLead", "f32"], ["aLead", "f32"], ["dPath", "f32"], ["vLat", "f32"],
    ["vLeadK", "f32"], ["aLeadK", "f32"], ["fcw", "bool"], ["status", "bool"],
    ["aLeadTau", "f32"], ["modelProb", "f32"], ["radar", "bool"],
    ["radarTrackId", "i32"], ["jLead", "f32"], ["score", "f32"],
  ];
  const tpms = [
    ["fl", "f32"], ["fr", "f32"], ["rl", "f32"], ["rr", "f32"],
  ];

  const schemas = new Map([
    [1, ["carState", [
      ["vEgo", "f32"], ["aEgo", "f32"], ["vEgoCluster", "f32"], ["vCruiseCluster", "f32"],
      ["steeringAngleDeg", "f32"], ["brakeHoldActive", "bool"], ["softHoldActive", "i16"],
      ["carrotCruise", "i16"], ["gearStep", "i16"], ["useLaneLineSpeed", "f32"],
      ["brakeLights", "bool"], ["leftBlindspot", "bool"], ["rightBlindspot", "bool"],
      ["leftLaneLine", "i16"], ["rightLaneLine", "i16"],
      ["gearShifter", "enumname", ["unknown", "park", "drive", "neutral", "reverse", "sport", "low", "brake", "eco", "manumatic"]],
      ["leftBlinker", "bool"], ["rightBlinker", "bool"],
      ["fuelGauge", "f32"], ["ureaGauge", "f32"], ["tpms", "struct", tpms],
      // Appended (must match compact_state.py / native order). Green EV telltale.
      // Decoder is append-tolerant, so shorter recorded frames stay decodable.
      ["evModeValid", "bool"], ["evModeActive", "bool"],
    ]]],
    [2, ["controlsState", [
      ["enabled", "bool"], ["vCruiseCluster", "f32"], ["activeLaneLine", "bool"],
      ["curvature", "f32"], ["desiredCurvature", "f32"], ["actualLateralAccel", "f32"],
      ["desiredLateralAccel", "f32"], ["lateralOutput", "f32"],
    ]]],
    [3, ["deviceState", [
      ["memoryUsagePercent", "i8"], ["freeSpacePercent", "f32"], ["cpuTempC", "f32list"],
      ["deviceType", "enumname", ["unknown", "neo", "chffrAndroid", "chffrIos", "tici", "pc", "tizi", "mici"]],
    ]]],
    [4, ["peripheralState", [["voltage", "u32"]]]],
    [5, ["carrotMan", [
      ["activeCarrot", "i32"], ["nRoadLimitSpeed", "i32"], ["xSpdType", "i32"],
      ["xSpdLimit", "i32"], ["xSpdDist", "i32"], ["xSpdCountDown", "i32"],
      ["xTurnInfo", "i32"], ["xDistToTurn", "i32"], ["xTurnCountDown", "i32"],
      ["atcType", "text"], ["szPosRoadName", "text"], ["szTBTMainText", "text"],
      ["desiredSpeed", "i32"], ["xPosLat", "f32"],
      ["xPosLon", "f32"], ["xPosAngle", "f32"], ["xPosSpeed", "f32"],
      ["trafficState", "i32"], ["nGoPosDist", "i32"], ["nGoPosTime", "i32"],
      ["szSdiDescr", "text"], ["naviPaths", "text"], ["desiredSource", "text"],
    ]]],
    [6, ["selfdriveState", [
      ["enabled", "bool"], ["personality", "u8"], ["alertStatus", "u8"],
      ["alertSize", "u8"], ["alertType", "text"], ["alertText1", "text"],
      ["alertText2", "text"],
    ]]],
    [7, ["gpsLocationExternal", [
      ["latitude", "f64"], ["longitude", "f64"], ["speed", "f32"],
      ["bearingDeg", "f32"], ["bearingAccuracyDeg", "f32"], ["speedAccuracy", "f32"],
      ["hasFix", "bool"],
      ["altitude", "f64"], ["horizontalAccuracy", "f32"],
      ["verticalAccuracy", "f32"], ["unixTimestampMillis", "f64"],
    ]]],
    [8, ["longitudinalPlan", [
      ["accels", "f32list"], ["speeds", "f32list"], ["jerks", "f32list"],
      ["tFollow", "f32"], ["desiredDistance", "f32"], ["myDrivingMode", "i32"],
      ["xState", "i32"], ["trafficState", "i32"], ["longitudinalPlanSource", "u8"],
      // Appended (must match compact_state.py / native order). Eco cruise override.
      ["cruiseTarget", "f32"],
    ]]],
    [9, ["modelV2", [
      ["frameId", "u32"], ["frameIdExtra", "u32"], ["position", "struct", xyz],
      ["velocity", "struct", velocity], ["laneLines", "structlist", xyz],
      ["laneLineProbs", "f32list"], ["roadEdges", "structlist", xyz],
      ["roadEdgeStds", "f32list"], ["leadsV3", "structlist", modelLead],
      ["laneLineStds", "f32list"], ["frameAge", "i32"],
      ["frameDropPerc", "f32"], ["modelExecutionTime", "f32"],
    ]]],
    [10, ["liveCalibration", [
      ["calStatus", "enumname", ["uncalibrated", "calibrated", "invalid", "recalibrating"]],
      ["calCycle", "i32"], ["calPerc", "i8"],
      ["validBlocks", "i32"], ["rpyCalib", "f32list"], ["height", "f32list"],
    ]]],
    [11, ["roadCameraState", [
      ["frameId", "u32"], ["sensor", "enumname", ["unknown", "ar0231", "ox03c10", "os04c10"]],
      ["timestampEof", "u64"],
    ]]],
    [12, ["lateralPlan", [
      ["useLaneLines", "bool"], ["latDebugText", "text"], ["position", "struct", xyz],
      ["distances", "f32list"], ["laneChangeState", "u8"], ["laneChangeDirection", "u8"],
    ]]],
    [13, ["radarState", [
      ["leadOne", "struct", radarLead], ["leadTwo", "struct", radarLead],
      ["leadRight", "struct", radarLead], ["leadLeft", "struct", radarLead],
      ["leadsLeft", "structlist", radarLead], ["leadsCenter", "structlist", radarLead],
      ["leadsRight", "structlist", radarLead],
      ["leadsLeft2", "structlist", radarLead], ["leadsRight2", "structlist", radarLead],
      ["leadsCutIn", "structlist", radarLead],
    ]]],
    [14, ["carControl", [
      ["latActive", "bool"], ["longActive", "bool"],
      ["actuators", "struct", [
        ["steeringAngleDeg", "f32"], ["accel", "f32"], ["curvature", "f32"],
      ]],
    ]]],
    [15, ["liveDelay", [["lateralDelay", "f32"], ["calPerc", "i8"]]]],
    [16, ["liveTorqueParameters", [
      ["liveValid", "bool"], ["latAccelFactorFiltered", "f32"],
      ["frictionCoefficientFiltered", "f32"], ["calPerc", "i8"],
    ]]],
    [17, ["liveParameters", [["angleOffsetDeg", "f32"], ["steerRatio", "f32"]]]],
    [18, ["liveTracks", [["points", "structlist", radarPoint]]]],
    [19, ["cameraOdometry", [
      ["frameId", "u32"], ["timestampEof", "u64"],
      ["trans", "f32list"], ["rot", "f32list"],
      ["transStd", "f32list"], ["rotStd", "f32list"],
    ]]],
    [21, ["carrotNavi", [
      ["schemaVersion","u16"], ["generation","u64"], ["sessionId","text"],
      ["publishMonoTimeNanos","u64"], ["connected","bool"],
      ["vehicle","struct",naviVehicle],
      ["guidanceCurrent","struct",naviGuidance],
      ["guidanceNext","struct",naviGuidance],
      ["laneCurrent","struct",naviLane],
      ["laneAhead","structlist",naviLane],
      ["speed","struct",naviSpeed],
      ["trafficSignal","struct",naviSignal],
      ["crossroad","struct",naviCrossroad],
      ["route","struct",naviRoute],
      ["navigationStatus","struct",naviStatus],
    ]]],
    [20, ["livePose", [
      ["orientationNED", "struct", xyzMeasurement],
      ["velocityDevice", "struct", xyzMeasurement],
      ["accelerationDevice", "struct", xyzMeasurement],
      ["angularVelocityDevice", "struct", xyzMeasurement],
      ["inputsOK", "bool"], ["posenetOK", "bool"], ["sensorsOK", "bool"],
      ["timestamp", "u64"],
    ]]],
  ]);

  class Cursor {
    constructor(bytes) {
      this.bytes = bytes;
      this.view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
      this.offset = 0;
    }

    ensure(size) {
      if (this.offset + size > this.bytes.byteLength) throw new Error("compact state frame truncated");
    }

    readI8() { this.ensure(1); return this.view.getInt8(this.offset++); }
    readU8() { this.ensure(1); return this.view.getUint8(this.offset++); }
    readI16() { this.ensure(2); const v = this.view.getInt16(this.offset, true); this.offset += 2; return v; }
    readU16() { this.ensure(2); const v = this.view.getUint16(this.offset, true); this.offset += 2; return v; }
    readI32() { this.ensure(4); const v = this.view.getInt32(this.offset, true); this.offset += 4; return v; }
    readU32() { this.ensure(4); const v = this.view.getUint32(this.offset, true); this.offset += 4; return v; }
    readU64() {
      this.ensure(8);
      // 나노초 타임스탬프는 2^53 을 넘을 수 있어 BigInt 로 읽는다.
      const v = this.view.getBigUint64(this.offset, true);
      this.offset += 8;
      return Number(v);
    }
    readF32() { this.ensure(4); const v = this.view.getFloat32(this.offset, true); this.offset += 4; return v; }
    readF64() { this.ensure(8); const v = this.view.getFloat64(this.offset, true); this.offset += 8; return v; }

    readText() {
      const length = this.readU16();
      this.ensure(length);
      const value = decoder.decode(this.bytes.subarray(this.offset, this.offset + length));
      this.offset += length;
      return value;
    }

    readFloat32List() {
      const length = this.readU16();
      this.ensure(length * 4);
      const out = new Array(length);
      for (let index = 0; index < length; index += 1) out[index] = this.readF32();
      return out;
    }

    readInt16List() {
      const length = this.readU16();
      this.ensure(length * 2);
      const out = new Array(length);
      for (let index = 0; index < length; index += 1) out[index] = this.readI16();
      return out;
    }

    readCoordList() {
      // u8 count + f64 앵커 + (count-1) x f32 델타
      const n = this.readU8();
      if (!n) return [];
      const lat0 = this.readF64(), lon0 = this.readF64();
      const out = [{ latitude: lat0, longitude: lon0 }];
      for (let i = 1; i < n; i += 1) {
        out.push({ latitude: lat0 + this.readF32(), longitude: lon0 + this.readF32() });
      }
      return out;
    }

    readQuantizedList(signed, scale) {
      const length = this.readU16();
      this.ensure(length * 2);
      const out = new Array(length);
      for (let index = 0; index < length; index += 1) {
        out[index] = (signed ? this.readI16() : this.readU16()) / scale;
      }
      return out;
    }
  }

  function readField(cursor, type, nestedSchema) {
    switch (type) {
      case "bool": return cursor.readU8() !== 0;
      case "i8": return cursor.readI8();
      case "u8": return cursor.readU8();
      case "i16": return cursor.readI16();
      case "u16": return cursor.readU16();
      case "i32": return cursor.readI32();
      case "u32": return cursor.readU32();
      case "u64": return cursor.readU64();
      case "f32": return cursor.readF32();
      case "f64": return cursor.readF64();
      case "text": return cursor.readText();
      case "enumname": return nestedSchema[cursor.readU8()] || nestedSchema[0] || "unknown";
      case "f32list": return cursor.readFloat32List();
      case "i16list": return cursor.readInt16List();
      case "coordlist": return cursor.readCoordList();
      case "u16cmlist": return cursor.readQuantizedList(false, 100);
      case "i16cmlist": return cursor.readQuantizedList(true, 100);
      case "i16mmlist": return cursor.readQuantizedList(true, 1000);
      case "struct": return readSchema(cursor, nestedSchema);
      case "structlist": {
        const count = cursor.readU8();
        const out = new Array(count);
        for (let index = 0; index < count; index += 1) out[index] = readSchema(cursor, nestedSchema);
        return out;
      }
      default: throw new Error(`unsupported compact state field ${type}`);
    }
  }

  function readSchema(cursor, schema, frameEnd) {
    const out = {};
    for (const [name, type, nestedSchema] of schema) {
      // Append-tolerant top-level decode: a frame encoded/recorded with an older
      // (shorter) schema stops exactly at its byte boundary, so newly appended
      // trailing fields are simply absent (undefined) rather than throwing.
      // Every frame is length-prefixed, so frameEnd is an exact boundary. Nested
      // structs pass no frameEnd and stay strict.
      if (frameEnd != null && cursor.offset >= frameEnd) break;
      out[name] = readField(cursor, type, nestedSchema);
    }
    return out;
  }

  function decodeFrame(data) {
    const bytes = data instanceof Uint8Array
      ? data
      : (ArrayBuffer.isView(data)
        ? new Uint8Array(data.buffer, data.byteOffset, data.byteLength)
        : new Uint8Array(data));
    if (bytes.byteLength < 8 || !MAGIC.every((value, index) => bytes[index] === value)) {
      throw new Error("invalid compact state frame");
    }
    const cursor = new Cursor(bytes);
    cursor.offset = 4;
    const serviceId = cursor.readU8();
    cursor.readU8(); // flags, reserved for a future compatible extension
    const sequence = cursor.readU16();
    const definition = schemas.get(serviceId);
    if (!definition) throw new Error(`unknown compact state service ${serviceId}`);
    const [service, schema] = definition;
    const decoded = readSchema(cursor, schema, bytes.byteLength);
    // Append-only evolution: a newer encoder may add trailing fields this decoder
    // doesn't know — ignore those extra bytes rather than rejecting the frame.
    // Mid-field truncation is still caught by Cursor.ensure().
    return { service, sequence, decoded, byteLength: bytes.byteLength };
  }

  function decodeFrames(data) {
    const bytes = data instanceof Uint8Array
      ? data
      : (ArrayBuffer.isView(data)
        ? new Uint8Array(data.buffer, data.byteOffset, data.byteLength)
        : new Uint8Array(data));
    if (bytes.byteLength >= 4 && MAGIC.every((value, index) => bytes[index] === value)) {
      return [decodeFrame(bytes)];
    }
    if (bytes.byteLength < 6 || !BATCH_MAGIC.every((value, index) => bytes[index] === value)) {
      throw new Error("invalid compact state batch");
    }

    const cursor = new Cursor(bytes);
    cursor.offset = 4;
    const count = cursor.readU16();
    const frames = new Array(count);
    for (let index = 0; index < count; index += 1) {
      const length = cursor.readU32();
      cursor.ensure(length);
      frames[index] = decodeFrame(bytes.subarray(cursor.offset, cursor.offset + length));
      cursor.offset += length;
    }
    if (cursor.offset !== bytes.byteLength) throw new Error("compact state batch trailing bytes");
    return frames;
  }

  function flattenSchema(schema, prefix = "") {
    const fields = [];
    for (const [name, type, nestedSchema] of schema) {
      const path = prefix ? `${prefix}.${name}` : name;
      if (type === "struct") {
        fields.push(...flattenSchema(nestedSchema, path));
      } else {
        fields.push({
          path,
          type,
          values: type === "enumname" ? [...nestedSchema] : [],
        });
      }
    }
    return fields;
  }

  function catalog() {
    return Array.from(schemas.values(), ([service, schema]) => ({
      service,
      fields: flattenSchema(schema),
    }));
  }

  return { HUD_SERVICES, OVERLAY_SERVICES, TRACK_SERVICES, AR_SERVICES, decodeFrame, decodeFrames, catalog };
})();
