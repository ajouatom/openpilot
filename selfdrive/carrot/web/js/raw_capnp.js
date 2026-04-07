"use strict";

const carrotRawCapnpGlobal = typeof globalThis !== "undefined"
  ? globalThis
  : (typeof self !== "undefined" ? self : window);

carrotRawCapnpGlobal.CarrotRawCapnp = (() => {
  const HUD_GEAR_NAMES = ["unknown", "park", "drive", "neutral", "reverse", "sport", "low", "brake", "eco", "manumatic"];
  const HUD_SERVICES = [
    "carState",
    "controlsState",
    "deviceState",
    "peripheralState",
    "carrotMan",
    "selfdriveState",
    "gpsLocationExternal",
    "longitudinalPlan",
  ];
  const OVERLAY_SERVICES = [
    "modelV2",
    "liveCalibration",
    "roadCameraState",
    "lateralPlan",
    "radarState",
    "carControl",
    "liveDelay",
    "liveTorqueParameters",
    "liveParameters",
  ];

  const HUD_SCHEMAS = {
    carState: {
      fields: {
        // capnp slot offsets (not ordinals) — verified via slot allocation algorithm
        vEgo: { kind: "float32", offset: 0 },             // @1  32-bit slot 0
        aEgo: { kind: "float32", offset: 7 },             // @16 32-bit slot 7
        vEgoCluster: { kind: "float32", offset: 14 },     // @44 32-bit slot 14
        vCruiseCluster: { kind: "float32", offset: 19 },  // @54 32-bit slot 19
        steeringAngleDeg: { kind: "float32", offset: 4 }, // @7  32-bit slot 4
        brakeHoldActive: { kind: "bool", offset: 356 },   // @38 bool bit 356
        softHoldActive: { kind: "int16", offset: 42 },    // @60 int16 slot 42
        carrotCruise: { kind: "int16", offset: 60 },      // @73 int16 slot 60
        gearStep: { kind: "int16", offset: 45 },          // @66 int16 slot 45
        useLaneLineSpeed: { kind: "float32", offset: 25 },// @68 32-bit slot 25
        brakeLights: { kind: "bool", offset: 68 },        // @19 bool bit 68
        gearShifter: { kind: "enum", offset: 5, values: HUD_GEAR_NAMES }, // @14 enum16 slot 5
      },
    },
    controlsState: {
      fields: {
        // capnp slot offsets (not ordinals) — verified via slot allocation algorithm
        vCruiseCluster: { kind: "float32", offset: 46 },  // @63 — may be fork-repurposed field
        activeLaneLine: { kind: "bool", offset: 716 },    // @67 bool bit 716
        curvature: { kind: "float32", offset: 34 },       // @37 32-bit slot 34
        desiredCurvature: { kind: "float32", offset: 44 },// @61 32-bit slot 44
      },
    },
    deviceState: {
      fields: {
        memoryUsagePercent: { kind: "int8", offset: 25 },
        cpuTempC: { kind: "list<float32>", offset: 1 },
      },
    },
    peripheralState: {
      fields: {
        voltage: { kind: "uint32", offset: 1 },
      },
    },
    carrotMan: {
      fields: {
        activeCarrot: { kind: "int32", offset: 0 },
        nRoadLimitSpeed: { kind: "int32", offset: 1 },
        xSpdType: { kind: "int32", offset: 2 },
        xSpdLimit: { kind: "int32", offset: 3 },
        desiredSpeed: { kind: "int32", offset: 10 },
        desiredSource: { kind: "text", offset: 4 },
      },
    },
    selfdriveState: {
      fields: {
        personality: { kind: "enum", offset: 5 },
      },
    },
    gpsLocationExternal: {
      fields: {
        latitude: { kind: "float64", offset: 1 },
        longitude: { kind: "float64", offset: 2 },
      },
    },
    longitudinalPlan: {
      fields: {
        accels: { kind: "list<float32>", offset: 3 },
        speeds: { kind: "list<float32>", offset: 4 },
        jerks: { kind: "list<float32>", offset: 5 },
        tFollow: { kind: "float32", offset: 26 },
        desiredDistance: { kind: "float32", offset: 27 },
        myDrivingMode: { kind: "int32", offset: 28 },
      },
    },
  };

  const XYZT_SCHEMA = {
    fields: {
      x: { kind: "list<float32>", offset: 0 },
      y: { kind: "list<float32>", offset: 1 },
      z: { kind: "list<float32>", offset: 2 },
      t: { kind: "list<float32>", offset: 3 },
    },
  };

  const LEAD_V3_SCHEMA = {
    fields: {
      prob: { kind: "float32", offset: 0 },
      probTime: { kind: "float32", offset: 1 },
      t: { kind: "list<float32>", offset: 0 },
      x: { kind: "list<float32>", offset: 1 },
      y: { kind: "list<float32>", offset: 3 },
      v: { kind: "list<float32>", offset: 5 },
      a: { kind: "list<float32>", offset: 7 },
    },
  };

  const RADAR_LEAD_SCHEMA = {
    fields: {
      // capnp binary layout: slots are per-type-width, NOT ordinal numbers.
      // Bool fields consume 1-bit slots starting after all prior 32-bit slots,
      // which shifts subsequent 32-bit slot numbers.
      // Verified via capnp slot allocation algorithm against log.capnp LeadData.
      dRel: { kind: "float32", offset: 0 },       // @0  32-bit slot 0
      yRel: { kind: "float32", offset: 1 },        // @1  32-bit slot 1
      vRel: { kind: "float32", offset: 2 },        // @2  32-bit slot 2
      aRel: { kind: "float32", offset: 3 },        // @3  32-bit slot 3
      vLead: { kind: "float32", offset: 4 },       // @4  32-bit slot 4
      aLead: { kind: "float32", offset: 5 },       // @5  32-bit slot 5
      dPath: { kind: "float32", offset: 6 },       // @6  32-bit slot 6
      vLat: { kind: "float32", offset: 7 },        // @7  32-bit slot 7
      vLeadK: { kind: "float32", offset: 8 },      // @8  32-bit slot 8
      aLeadK: { kind: "float32", offset: 9 },      // @9  32-bit slot 9
      fcw: { kind: "bool", offset: 320 },          // @10 bool bit 320 (byte 40 bit 0)
      status: { kind: "bool", offset: 321 },       // @11 bool bit 321 (byte 40 bit 1)
      aLeadTau: { kind: "float32", offset: 11 },   // @12 32-bit slot 11 (slot 10 blocked by bools)
      modelProb: { kind: "float32", offset: 12 },  // @13 32-bit slot 12
      radar: { kind: "bool", offset: 322 },        // @14 bool bit 322 (byte 40 bit 2)
      radarTrackId: { kind: "int32", offset: 13, xorDefault: -1 }, // @15 32-bit slot 13
      jLead: { kind: "float32", offset: 14 },      // @16 32-bit slot 14
      score: { kind: "float32", offset: 15 },      // @17 32-bit slot 15
    },
  };

  const MODEL_META_SCHEMA = {
    fields: {
      laneChangeState: { kind: "enum", offset: 9 },
      laneChangeDirection: { kind: "enum", offset: 10 },
      laneWidthLeft: { kind: "float32", offset: 6 },
      laneWidthRight: { kind: "float32", offset: 7 },
      distanceToRoadEdgeLeft: { kind: "float32", offset: 8 },
      distanceToRoadEdgeRight: { kind: "float32", offset: 9 },
      laneChangeProb: { kind: "float32", offset: 10 },
      modelTurnSpeed: { kind: "float32", offset: 11 },
    },
  };

  const OVERLAY_SCHEMAS = {
    modelV2: {
      fields: {
        frameId: { kind: "uint32", offset: 0 },
        frameIdExtra: { kind: "uint32", offset: 7 },
        position: { kind: "struct", offset: 0, schema: XYZT_SCHEMA },
        velocity: { kind: "struct", offset: 2, schema: XYZT_SCHEMA },
        laneLines: { kind: "list<struct>", offset: 4, schema: XYZT_SCHEMA },
        laneLineProbs: { kind: "list<float32>", offset: 5 },
        roadEdges: { kind: "list<struct>", offset: 6, schema: XYZT_SCHEMA },
        roadEdgeStds: { kind: "list<float32>", offset: 10 },
        leadsV3: { kind: "list<struct>", offset: 12, schema: LEAD_V3_SCHEMA },
        meta: { kind: "struct", offset: 8, schema: MODEL_META_SCHEMA },
      },
    },
    liveCalibration: {
      fields: {
        calStatus: { kind: "enum", offset: 1 },
        calCycle: { kind: "int32", offset: 1 },
        calPerc: { kind: "int8", offset: 1 },
        validBlocks: { kind: "int32", offset: 2 },
        extrinsicMatrix: { kind: "list<float32>", offset: 1 },
        rpyCalib: { kind: "list<float32>", offset: 4 },
        wideFromDeviceEuler: { kind: "list<float32>", offset: 6 },
        height: { kind: "list<float32>", offset: 7 },
      },
    },
    roadCameraState: {
      fields: {
        frameId: { kind: "uint32", offset: 0 },
        transform: { kind: "list<float32>", offset: 2 },
      },
    },
    lateralPlan: {
      fields: {
        useLaneLines: { kind: "bool", offset: 166 }, // @29 bool bit 166 (capnp slot, not ordinal)
        latDebugText: { kind: "text", offset: 9 },
        position: { kind: "struct", offset: 10, schema: XYZT_SCHEMA },
        distances: { kind: "list<float32>", offset: 11 },
      },
    },
    radarState: {
      fields: {
        // RadarState keeps older deprecated pointer fields ahead of the modern
        // lead slots, so these offsets follow the pointer-section layout.
        leadOne: { kind: "struct", offset: 1, schema: RADAR_LEAD_SCHEMA },
        leadTwo: { kind: "struct", offset: 2, schema: RADAR_LEAD_SCHEMA },
        leadRight: { kind: "struct", offset: 5, schema: RADAR_LEAD_SCHEMA },
        leadLeft: { kind: "struct", offset: 9, schema: RADAR_LEAD_SCHEMA },
      },
    },
    carControl: {
      fields: {
        latActive: { kind: "bool", offset: 2 },   // @11 bool bit 2 (capnp slot, not ordinal)
        longActive: { kind: "bool", offset: 3 },  // @12 bool bit 3
        actuators: {
          kind: "struct",
          offset: 2,
          schema: {
            fields: {
              steeringAngleDeg: { kind: "float32", offset: 3 },
              accel: { kind: "float32", offset: 4 },
              curvature: { kind: "float32", offset: 7 },
            },
          },
        },
      },
    },
    liveDelay: {
      fields: {
        lateralDelay: { kind: "float32", offset: 0 },
        calPerc: { kind: "int8", offset: 10 },  // @6 int8 slot 10 (capnp slot, not ordinal)
      },
    },
    liveTorqueParameters: {
      fields: {
        liveValid: { kind: "bool", offset: 0 },
        latAccelFactorFiltered: { kind: "float32", offset: 4 },
        frictionCoefficientFiltered: { kind: "float32", offset: 6 },
        calPerc: { kind: "int8", offset: 1 },
      },
    },
    liveParameters: {
      fields: {
        angleOffsetDeg: { kind: "float32", offset: 2 },
        steerRatio: { kind: "float32", offset: 5 },
      },
    },
  };

  function signed30(value) {
    return (value & 0x20000000) ? (value - 0x40000000) : value;
  }

  function parseMessage(data) {
    const bytes = data instanceof Uint8Array ? data : new Uint8Array(data);
    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    const segmentCount = view.getUint32(0, true) + 1;
    let cursor = 4;
    const segmentWordCounts = [];
    for (let i = 0; i < segmentCount; i++) {
      segmentWordCounts.push(view.getUint32(cursor, true));
      cursor += 4;
    }
    if (segmentCount % 2 === 0) cursor += 4;

    const segments = [];
    let segmentByteOffset = cursor;
    for (const wordCount of segmentWordCounts) {
      segments.push({
        byteOffset: segmentByteOffset,
        byteLength: wordCount * 8,
      });
      segmentByteOffset += wordCount * 8;
    }
    return { bytes, view, segments };
  }

  function getSegment(message, segmentIndex) {
    const segment = message.segments[segmentIndex];
    if (!segment) {
      throw new Error(`capnp segment ${segmentIndex} out of range`);
    }
    return segment;
  }

  function readPointerWord(message, segmentIndex, wordOffset) {
    const segment = getSegment(message, segmentIndex);
    if (wordOffset < 0 || ((wordOffset + 1) * 8) > segment.byteLength) {
      throw new Error(`capnp word offset ${wordOffset} out of range for segment ${segmentIndex}`);
    }
    const absolute = segment.byteOffset + wordOffset * 8;
    return {
      lo: message.view.getUint32(absolute, true),
      hi: message.view.getUint32(absolute + 4, true),
    };
  }

  function resolvePointerWord(message, segmentIndex, pointerWordOffset, depth = 0) {
    if (depth > 8) {
      throw new Error("capnp pointer resolution depth exceeded");
    }

    const word = readPointerWord(message, segmentIndex, pointerWordOffset);
    if (word.lo === 0 && word.hi === 0) return null;

    const pointerType = word.lo & 0x3;
    if (pointerType !== 2) {
      return {
        lo: word.lo,
        hi: word.hi,
        pointerSegmentIndex: segmentIndex,
        pointerWordOffset,
        contentSegmentIndex: null,
        contentWordOffset: null,
      };
    }

    const isDoubleFar = ((word.lo >>> 2) & 0x1) !== 0;
    const landingPadWordOffset = word.lo >>> 3;
    const landingPadSegmentIndex = word.hi >>> 0;

    if (!isDoubleFar) {
      return resolvePointerWord(message, landingPadSegmentIndex, landingPadWordOffset, depth + 1);
    }

    const landingPadFar = readPointerWord(message, landingPadSegmentIndex, landingPadWordOffset);
    if ((landingPadFar.lo & 0x3) !== 2 || ((landingPadFar.lo >>> 2) & 0x1) !== 0) {
      throw new Error("invalid capnp double-far landing pad");
    }

    const tagWord = readPointerWord(message, landingPadSegmentIndex, landingPadWordOffset + 1);
    return {
      lo: tagWord.lo,
      hi: tagWord.hi,
      pointerSegmentIndex: landingPadSegmentIndex,
      pointerWordOffset: landingPadWordOffset + 1,
      contentSegmentIndex: landingPadFar.hi >>> 0,
      contentWordOffset: landingPadFar.lo >>> 3,
    };
  }

  function readStructPointer(message, segmentIndex, pointerWordOffset) {
    const resolved = resolvePointerWord(message, segmentIndex, pointerWordOffset);
    if (!resolved) return null;
    const pointerType = resolved.lo & 0x3;
    if (pointerType !== 0) {
      throw new Error(`unsupported capnp struct pointer type ${pointerType}`);
    }

    if (resolved.contentWordOffset != null) {
      const offset = signed30(resolved.lo >>> 2);
      if (offset !== 0) {
        throw new Error(`invalid capnp double-far struct offset ${offset}`);
      }
      return {
        segmentIndex: resolved.contentSegmentIndex,
        dataWordOffset: resolved.contentWordOffset,
        dataWords: resolved.hi & 0xffff,
        pointerCount: resolved.hi >>> 16,
      };
    }

    return {
      segmentIndex: resolved.pointerSegmentIndex,
      dataWordOffset: resolved.pointerWordOffset + 1 + signed30(resolved.lo >>> 2),
      dataWords: resolved.hi & 0xffff,
      pointerCount: resolved.hi >>> 16,
    };
  }

  function readStructSlot(message, structRef, slotIndex) {
    const pointerWordOffset = structRef.dataWordOffset + structRef.dataWords + slotIndex;
    return readStructPointer(message, structRef.segmentIndex, pointerWordOffset);
  }

  function readListSlot(message, structRef, slotIndex) {
    const pointerWordOffset = structRef.dataWordOffset + structRef.dataWords + slotIndex;
    const resolved = resolvePointerWord(message, structRef.segmentIndex, pointerWordOffset);
    if (!resolved) return null;
    const pointerType = resolved.lo & 0x3;
    if (pointerType !== 1) {
      throw new Error(`unsupported capnp list pointer type ${pointerType}`);
    }

    if (resolved.contentWordOffset != null) {
      const offset = signed30(resolved.lo >>> 2);
      if (offset !== 0) {
        throw new Error(`invalid capnp double-far list offset ${offset}`);
      }
      return {
        segmentIndex: resolved.contentSegmentIndex,
        wordOffset: resolved.contentWordOffset,
        elementSizeCode: resolved.hi & 0x7,
        elementCount: resolved.hi >>> 3,
      };
    }

    return {
      segmentIndex: resolved.pointerSegmentIndex,
      wordOffset: resolved.pointerWordOffset + 1 + signed30(resolved.lo >>> 2),
      elementSizeCode: resolved.hi & 0x7,
      elementCount: resolved.hi >>> 3,
    };
  }

  function structDataByteOffset(message, structRef) {
    return getSegment(message, structRef.segmentIndex).byteOffset + structRef.dataWordOffset * 8;
  }

  function readScalar(message, structRef, kind, offset) {
    const base = structDataByteOffset(message, structRef);
    switch (kind) {
      case "int8":
        return message.view.getInt8(base + offset);
      case "int16":
        return message.view.getInt16(base + offset * 2, true);
      case "bool": {
        const byteOffset = Math.floor(offset / 8);
        const bitIndex = offset % 8;
        return ((message.view.getUint8(base + byteOffset) >>> bitIndex) & 0x1) === 1;
      }
      case "uint32":
        return message.view.getUint32(base + offset * 4, true);
      case "int32":
        return message.view.getInt32(base + offset * 4, true);
      case "float32":
        return message.view.getFloat32(base + offset * 4, true);
      case "float64":
        return message.view.getFloat64(base + offset * 8, true);
      case "enum":
        return message.view.getUint16(base + offset * 2, true);
      default:
        throw new Error(`unsupported capnp scalar kind ${kind}`);
    }
  }

  function readFloat32List(message, structRef, slotIndex) {
    const listRef = readListSlot(message, structRef, slotIndex);
    if (!listRef) return [];
    if (listRef.elementSizeCode !== 4) {
      throw new Error(`unexpected float32 list size code ${listRef.elementSizeCode}`);
    }
    const segment = getSegment(message, listRef.segmentIndex);
    const byteOffset = segment.byteOffset + listRef.wordOffset * 8;
    const out = [];
    for (let i = 0; i < listRef.elementCount; i++) {
      out.push(message.view.getFloat32(byteOffset + i * 4, true));
    }
    return out;
  }

  function readStructList(message, structRef, slotIndex) {
    const listRef = readListSlot(message, structRef, slotIndex);
    if (!listRef) return [];
    if (listRef.elementSizeCode !== 7) {
      throw new Error(`unsupported capnp struct list size code ${listRef.elementSizeCode}`);
    }

    const { lo, hi } = readPointerWord(message, listRef.segmentIndex, listRef.wordOffset);
    const pointerType = lo & 0x3;
    if (pointerType !== 0) {
      throw new Error(`unexpected composite tag pointer type ${pointerType}`);
    }

    const elementCount = lo >>> 2;
    const dataWords = hi & 0xffff;
    const pointerCount = hi >>> 16;
    const strideWords = dataWords + pointerCount;
    const firstElementWordOffset = listRef.wordOffset + 1;
    const out = [];
    for (let i = 0; i < elementCount; i++) {
      out.push({
        segmentIndex: listRef.segmentIndex,
        dataWordOffset: firstElementWordOffset + i * strideWords,
        dataWords,
        pointerCount,
      });
    }
    return out;
  }

  function readText(message, structRef, slotIndex) {
    const listRef = readListSlot(message, structRef, slotIndex);
    if (!listRef) return "";
    if (listRef.elementSizeCode !== 2) {
      throw new Error(`unexpected text list size code ${listRef.elementSizeCode}`);
    }
    const segment = getSegment(message, listRef.segmentIndex);
    const byteOffset = segment.byteOffset + listRef.wordOffset * 8;
    const bytes = message.bytes.subarray(byteOffset, byteOffset + listRef.elementCount);
    const textBytes = bytes.length && bytes[bytes.length - 1] === 0 ? bytes.subarray(0, bytes.length - 1) : bytes;
    return new TextDecoder().decode(textBytes);
  }

  function decodeStruct(message, structRef, schema) {
    const decoded = {};
    for (const [fieldName, fieldSpec] of Object.entries(schema.fields)) {
      try {
        decoded[fieldName] = decodeField(message, structRef, fieldSpec);
      } catch {
        decoded[fieldName] = null;
      }
    }
    return decoded;
  }

  function decodeField(message, structRef, fieldSpec) {
    switch (fieldSpec.kind) {
      case "bool":
      case "int8":
      case "uint32":
      case "int32":
      case "float32":
      case "float64": {
        const raw = readScalar(message, structRef, fieldSpec.kind, fieldSpec.offset);
        // capnp stores wire_value = actual XOR default; apply XOR to recover actual
        return fieldSpec.xorDefault != null ? (raw ^ fieldSpec.xorDefault) : raw;
      }
      case "enum": {
        const rawValue = readScalar(message, structRef, "enum", fieldSpec.offset);
        return Array.isArray(fieldSpec.values) ? (fieldSpec.values[rawValue] ?? rawValue) : rawValue;
      }
      case "text":
        return readText(message, structRef, fieldSpec.offset);
      case "list<float32>":
        return readFloat32List(message, structRef, fieldSpec.offset);
      case "struct": {
        const nestedRef = readStructSlot(message, structRef, fieldSpec.offset);
        return nestedRef ? decodeStruct(message, nestedRef, fieldSpec.schema) : null;
      }
      case "list<struct>": {
        const nestedRefs = readStructList(message, structRef, fieldSpec.offset);
        return nestedRefs.map((nestedRef) => decodeStruct(message, nestedRef, fieldSpec.schema));
      }
      default:
        throw new Error(`unsupported capnp field kind ${fieldSpec.kind}`);
    }
  }

  function decodeEventFields(service, data, schemas = HUD_SCHEMAS) {
    const schema = schemas[service];
    if (!schema) return null;

    const message = parseMessage(data);
    const eventRef = readStructPointer(message, 0, 0);
    if (!eventRef) return null;
    const payloadRef = readStructSlot(message, eventRef, 0);
    if (!payloadRef) return null;
    return decodeStruct(message, payloadRef, schema);
  }

  function rawHudDriveMode(state) {
    const mode = Number(state?.longitudinalPlan?.myDrivingMode);
    if (mode === 1) return { name: "Eco", kind: "eco" };
    if (mode === 2) return { name: "Safe", kind: "safe" };
    if (mode === 4) return { name: "Sport", kind: "sport" };
    return { name: "Normal", kind: "normal" };
  }

  function rawHudGap(state) {
    const personality = Number(state?.selfdriveState?.personality);
    if (!isFinite(personality)) return null;
    return personality + 1;
  }

  function rawHudSpeedLimitKph(state) {
    const xSpdLimit = Number(state?.carrotMan?.xSpdLimit);
    const xSpdType = Number(state?.carrotMan?.xSpdType);
    if (isFinite(xSpdLimit) && xSpdLimit > 0 && xSpdType !== 22) return xSpdLimit;

    const nRoadLimitSpeed = Number(state?.carrotMan?.nRoadLimitSpeed);
    if (isFinite(nRoadLimitSpeed) && nRoadLimitSpeed > 0) return nRoadLimitSpeed;

    return null;
  }

  function rawHudTemp(state) {
    const desiredSpeed = Number(state?.carrotMan?.desiredSpeed);
    const vCruise = Number(
      state?.carState?.vCruiseCluster ??
      state?.controlsState?.vCruiseCluster
    );
    const desiredSource = state?.carrotMan?.desiredSource;
    if (!isFinite(desiredSpeed)) return null;

    return {
      speed: desiredSpeed,
      source: isFinite(vCruise) && desiredSpeed >= vCruise ? (desiredSource || "") : "",
      is_decel: isFinite(vCruise) ? desiredSpeed < vCruise : false,
    };
  }

  function rawHudGear(state) {
    const raw = String(state?.carState?.gearShifter || "").trim().toLowerCase();
    if (!raw) return null;
    if (raw === "park") return "P";
    if (raw === "reverse") return "R";
    if (raw === "neutral") return "N";
    if (raw === "drive") return "D";
    if (raw === "sport") return "S";
    if (raw === "low") return "L";
    return raw.length > 2 ? raw.slice(0, 2).toUpperCase() : raw.toUpperCase();
  }

  function rawHudGearStep(state) {
    const gearShifter = String(state?.carState?.gearShifter || "").trim().toLowerCase();
    if (gearShifter !== "drive") return null;
    const gearStep = Number(state?.carState?.gearStep);
    if (!isFinite(gearStep) || gearStep <= 0) return null;
    return Math.round(gearStep);
  }

  function deriveHudPayload(state) {
    const deviceState = state?.deviceState || {};
    const peripheralState = state?.peripheralState || {};
    const carState = state?.carState || {};
    const carrotMan = state?.carrotMan || {};
    const gpsLocationExternal = state?.gpsLocationExternal || {};
    const controlsState = state?.controlsState || {};
    const tfGap = rawHudGap(state);
    const speedLimitKph = rawHudSpeedLimitKph(state);

    const cpuTemps = Array.isArray(deviceState?.cpuTempC)
      ? deviceState.cpuTempC.filter((value) => isFinite(Number(value))).map(Number)
      : [];
    const cpuTempC = cpuTemps.length ? Math.max(...cpuTemps) : null;
    const voltageMv = Number(peripheralState?.voltage);
    const vEgo = Number(
      carState?.vEgoCluster != null ? carState.vEgoCluster : carState?.vEgo
    );

    return {
      cpuTempC,
      memPct: Number(deviceState?.memoryUsagePercent),
      diskPct: null,
      voltageV: isFinite(voltageMv) ? voltageMv / 1000.0 : null,
      vEgo,
      vSetKph: Number(
        carState?.vCruiseCluster ?? controlsState?.vCruiseCluster
      ),
      temp: rawHudTemp(state),
      redDot: false,
      tlight: "off",
      tfGap,
      tfBars: tfGap,
      gear: rawHudGear(state),
      gearStep: rawHudGearStep(state),
      gpsOk: gpsLocationExternal?.latitude != null || gpsLocationExternal?.longitude != null,
      driveMode: rawHudDriveMode(state),
      speedLimitKph,
      speedLimitOver: isFinite(vEgo) && isFinite(speedLimitKph) ? (vEgo * 3.6) > speedLimitKph : false,
      speedLimitBlink: isFinite(Number(carrotMan?.xSpdLimit)) && Number(carrotMan.xSpdLimit) > 0 && Number(carrotMan?.xSpdType) !== 22 && Number(carrotMan?.xSpdType) !== 4,
      apm: (() => {
        const ac = Number(carrotMan?.activeCarrot);
        if (!isFinite(ac) || ac < 1) return "";
        return ac >= 2 ? "APN" : "APM";
      })(),
    };
  }

  return {
    HUD_SERVICES,
    OVERLAY_SERVICES,
    decodeHudEvent(service, data) {
      return decodeEventFields(service, data, HUD_SCHEMAS);
    },
    decodeOverlayEvent(service, data) {
      return decodeEventFields(service, data, OVERLAY_SCHEMAS);
    },
    deriveHudPayload,
  };
})();
