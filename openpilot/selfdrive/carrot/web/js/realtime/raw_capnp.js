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
    "cameraOdometry",
    "livePose",
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
        yawRate: { kind: "float32", offset: 9 },           // @22 32-bit slot 9
        brakeHoldActive: { kind: "bool", offset: 356 },   // @38 bool bit 356
        softHoldActive: { kind: "int16", offset: 42 },    // @60 int16 slot 42
        carrotCruise: { kind: "int16", offset: 60 },      // @73 int16 slot 60
        gearStep: { kind: "int16", offset: 45 },          // @66 int16 slot 45
        useLaneLineSpeed: { kind: "float32", offset: 25 },// @68 32-bit slot 25
        brakeLights: { kind: "bool", offset: 68 },        // @19 bool bit 68
        gasPressed: { kind: "bool", offset: 64 },
        brakePressed: { kind: "bool", offset: 65 },
        steeringPressed: { kind: "bool", offset: 66 },
        standstill: { kind: "bool", offset: 67 },
        leftBlinker: { kind: "bool", offset: 69 },
        rightBlinker: { kind: "bool", offset: 70 },
        doorOpen: { kind: "bool", offset: 72 },
        seatbeltUnlatched: { kind: "bool", offset: 73 },
        canValid: { kind: "bool", offset: 74 },
        clutchPressed: { kind: "bool", offset: 75 },
        stockAeb: { kind: "bool", offset: 77 },
        stockFcw: { kind: "bool", offset: 78 },
        espDisabled: { kind: "bool", offset: 79 },
        leftBlindspot: { kind: "bool", offset: 352 },
        rightBlindspot: { kind: "bool", offset: 353 },
        steerFaultTemporary: { kind: "bool", offset: 354 },
        steerFaultPermanent: { kind: "bool", offset: 355 },
        parkingBrake: { kind: "bool", offset: 357 },
        canTimeout: { kind: "bool", offset: 358 },
        accFaulted: { kind: "bool", offset: 359 },
        carFaultedNonCritical: { kind: "bool", offset: 362 },
        vehicleSensorsInvalid: { kind: "bool", offset: 365 },
        lowSpeedAlert: { kind: "bool", offset: 367 },
        buttonEnable: { kind: "bool", offset: 368 },
        leftLaneLine: { kind: "int16", offset: 61 },
        rightLaneLine: { kind: "int16", offset: 62 },
        pcmCruiseGap: { kind: "int16", offset: 44 },
        gearShifter: { kind: "enum", offset: 5, values: HUD_GEAR_NAMES }, // @14 enum16 slot 5
        cruiseState: {
          kind: "struct",
          offset: 2,
          schema: { fields: { enabled: { kind: "bool", offset: 0 } } },
        },
        buttonEvents: {
          kind: "list<struct>",
          offset: 3,
          schema: {
            fields: {
              pressed: { kind: "bool", offset: 0 },
              type: {
                kind: "enum",
                offset: 1,
                values: [
                  "unknown", "leftBlinker", "rightBlinker", "accelCruise", "decelCruise",
                  "cancel", "lkas", "altButton2", "mainCruise", "setCruise", "resumeCruise",
                  "gapAdjustCruise", "lfaButton", "paddleLeft", "paddleRight",
                ],
              },
            },
          },
        },
      },
    },
    controlsState: {
      fields: {
        // capnp slot offsets (not ordinals) — verified via slot allocation algorithm
        enabled: { kind: "bool", offset: 704 },
        vCruiseCluster: { kind: "float32", offset: 46 },
        activeLaneLine: { kind: "bool", offset: 716 },    // @67 bool bit 716
        curvature: { kind: "float32", offset: 34 },       // @37 32-bit slot 34
        desiredCurvature: { kind: "float32", offset: 44 },// @61 32-bit slot 44
        forceDecel: { kind: "bool", offset: 713 },
        longControlState: { kind: "enum", offset: 45, values: ["off", "pid", "stopping", "starting"] },
        torqueState: {
          kind: "struct",
          offset: 5,
          schema: {
            fields: {
              actualLateralAccel: { kind: "float32", offset: 8 },
              desiredLateralAccel: { kind: "float32", offset: 9 },
              lateralOutput: { kind: "float32", offset: 6 },
            },
          },
        },
      },
    },
    deviceState: {
      fields: {
        memoryUsagePercent: { kind: "int8", offset: 25 },
        cpuTempC: { kind: "list<float32>", offset: 1 },
        freeSpacePercent: { kind: "float32", offset: 4 },
        deviceType: { kind: "enum", offset: 41, values: ["unknown", "neo", "chffrAndroid", "chffrIos", "tici", "pc", "tizi", "mici"] },
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
        xSpdDist: { kind: "int32", offset: 4 },
        xSpdCountDown: { kind: "int32", offset: 5 },
        xTurnInfo: { kind: "int32", offset: 6 },
        xDistToTurn: { kind: "int32", offset: 7 },
        xTurnCountDown: { kind: "int32", offset: 8 },
        atcType: { kind: "text", offset: 1 },
        szPosRoadName: { kind: "text", offset: 2 },
        szTBTMainText: { kind: "text", offset: 3 },
        desiredSpeed: { kind: "int32", offset: 10 },
        xPosLat: { kind: "float32", offset: 12 },
        xPosLon: { kind: "float32", offset: 13 },
        xPosAngle: { kind: "float32", offset: 14 },
        xPosSpeed: { kind: "float32", offset: 15 },
        trafficState: { kind: "int32", offset: 16 },
        nGoPosDist: { kind: "int32", offset: 17 },
        nGoPosTime: { kind: "int32", offset: 18 },
        szSdiDescr: { kind: "text", offset: 7 },
        naviPaths: { kind: "text", offset: 8 },
        desiredSource: { kind: "text", offset: 4 },
        carrotCmdIndex: { kind: "int32", offset: 11 },
        carrotCmd: { kind: "text", offset: 5 },
        carrotArg: { kind: "text", offset: 6 },
      },
    },
    selfdriveState: {
      fields: {
        state: { kind: "enum", offset: 0, values: ["disabled", "preEnabled", "enabled", "softDisabling", "overriding"] },
        enabled: { kind: "bool", offset: 16 },
        active: { kind: "bool", offset: 17 },
        engageable: { kind: "bool", offset: 18 },
        experimentalMode: { kind: "bool", offset: 19 },
        personality: { kind: "enum", offset: 5 },
        alertStatus: { kind: "enum", offset: 2 },
        alertSize: { kind: "enum", offset: 3 },
        alertType: { kind: "text", offset: 2 },
        alertText1: { kind: "text", offset: 0 },
        alertText2: { kind: "text", offset: 1 },
      },
    },
    gpsLocationExternal: {
      fields: {
        latitude: { kind: "float64", offset: 1 },
        longitude: { kind: "float64", offset: 2 },
        speed: { kind: "float32", offset: 1 },
        bearingDeg: { kind: "float32", offset: 8 },
        bearingAccuracyDeg: { kind: "float32", offset: 13 },
        speedAccuracy: { kind: "float32", offset: 14 },
        hasFix: { kind: "bool", offset: 480 },
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
        xState: { kind: "int32", offset: 21 },
        trafficState: { kind: "int32", offset: 22 },
        longitudinalPlanSource: { kind: "enum", offset: 1 },
        hasLead: { kind: "bool", offset: 2 },
        fcw: { kind: "bool", offset: 3 },
        shouldStop: { kind: "bool", offset: 12 },
      },
    },
  };
  // LivePose.XYZMeasurement: float32 6개(슬롯 0..5) 뒤 Bool → 비트 192(=24바이트)
  // 중첩 스키마도 반드시 { fields: ... } 래퍼가 있어야 한다.
  // decodeStruct 가 schema.fields 를 순회하므로, 래퍼가 없으면 예외 없이
  // 빈 객체가 나와서 원인을 찾기 매우 어렵다(XYZT_SCHEMA 와 같은 모양).
  const XYZ_MEASUREMENT_SCHEMA = {
    fields: {
      x: { kind: "float32", offset: 0 },
      y: { kind: "float32", offset: 1 },
      z: { kind: "float32", offset: 2 },
      xStd: { kind: "float32", offset: 3 },
      yStd: { kind: "float32", offset: 4 },
      zStd: { kind: "float32", offset: 5 },
      valid: { kind: "bool", offset: 192 },
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

  const RADAR_POINT_SCHEMA = {
    fields: {
      // RadarPoint starts with a UInt64 track id. Float32 offsets therefore
      // begin at slot 2 rather than following the source field ordinals.
      trackId: { kind: "uint64", offset: 0 },
      dRel: { kind: "float32", offset: 2 },
      yRel: { kind: "float32", offset: 3 },
      vRel: { kind: "float32", offset: 4 },
      aRel: { kind: "float32", offset: 5 },
      yvRel: { kind: "float32", offset: 6 },
      measured: { kind: "bool", offset: 224 },
      vLead: { kind: "float32", offset: 8 },
      aLead: { kind: "float32", offset: 9 },
      jLead: { kind: "float32", offset: 10 },
      radarSource: {
        kind: "enum",
        offset: 15,
        values: ["frontRadar", "scc", "corner235", "corner180", "corner430"],
      },
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
    cameraOdometry: {
      // capnp 오프셋은 ordinal 순 배치 규칙으로 산출했다.
      //   @0..@3 이 pointer(List) → ptr 0..3
      //   @4 frameId(UInt32) → 32bit 슬롯 0
      //   @5 timestampEof(UInt64) → 64bit 슬롯 1 (앞 4바이트는 hole)
      // 값이 틀리면 조용히 이상해지므로 ar/replay_guard 가 범위를 검사한다.
      fields: {
        trans: { kind: "list<float32>", offset: 0 },
        rot: { kind: "list<float32>", offset: 1 },
        transStd: { kind: "list<float32>", offset: 2 },
        rotStd: { kind: "list<float32>", offset: 3 },
        frameId: { kind: "uint32", offset: 0 },
        timestampEof: { kind: "uint64", offset: 1 },
      },
    },
    livePose: {
      //   @0..@3 XYZMeasurement(struct) → ptr 0..3
      //   @4..@6 Bool → 비트 0,1,2
      //   @7 debugFilterState(struct) → ptr 4
      //   @8 timestamp(UInt64) → 64bit 슬롯 1 (슬롯 0은 bool 이 점유)
      fields: {
        orientationNED: { kind: "struct", offset: 0, schema: XYZ_MEASUREMENT_SCHEMA },
        velocityDevice: { kind: "struct", offset: 1, schema: XYZ_MEASUREMENT_SCHEMA },
        accelerationDevice: { kind: "struct", offset: 2, schema: XYZ_MEASUREMENT_SCHEMA },
        angularVelocityDevice: { kind: "struct", offset: 3, schema: XYZ_MEASUREMENT_SCHEMA },
        inputsOK: { kind: "bool", offset: 0 },
        posenetOK: { kind: "bool", offset: 1 },
        sensorsOK: { kind: "bool", offset: 2 },
        timestamp: { kind: "uint64", offset: 1 },
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
        sensor: { kind: "enum", offset: 40, values: ["unknown", "ar0231", "ox03c10", "os04c10"] },
      },
    },
    lateralPlan: {
      fields: {
        useLaneLines: { kind: "bool", offset: 166 }, // @29 bool bit 166 (capnp slot, not ordinal)
        latDebugText: { kind: "text", offset: 9 },
        position: { kind: "struct", offset: 10, schema: XYZT_SCHEMA },
        distances: { kind: "list<float32>", offset: 11 },
        laneChangeState: { kind: "enum", offset: 16 },
        laneChangeDirection: { kind: "enum", offset: 17 },
      },
    },
    radarState: {
      fields: {
        // RadarState keeps older deprecated pointer fields ahead of the modern
        // lead slots, so these offsets follow the pointer-section layout.
        leadOne: { kind: "struct", offset: 1, schema: RADAR_LEAD_SCHEMA },
        leadTwo: { kind: "struct", offset: 2, schema: RADAR_LEAD_SCHEMA },
        leadRight: { kind: "struct", offset: 6, schema: RADAR_LEAD_SCHEMA },
        leadLeft: { kind: "struct", offset: 10, schema: RADAR_LEAD_SCHEMA },
        leadsLeft: { kind: "list<struct>", offset: 8, schema: RADAR_LEAD_SCHEMA },
        leadsCenter: { kind: "list<struct>", offset: 7, schema: RADAR_LEAD_SCHEMA },
        leadsRight: { kind: "list<struct>", offset: 9, schema: RADAR_LEAD_SCHEMA },
        leadsLeft2: { kind: "list<struct>", offset: 11, schema: RADAR_LEAD_SCHEMA },
        leadsRight2: { kind: "list<struct>", offset: 12, schema: RADAR_LEAD_SCHEMA },
        leadsCutIn: { kind: "list<struct>", offset: 13, schema: RADAR_LEAD_SCHEMA },
      },
    },
    liveTracks: {
      fields: {
        points: { kind: "list<struct>", offset: 1, schema: RADAR_POINT_SCHEMA },
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
        cruiseControl: {
          kind: "struct",
          offset: 0,
          schema: {
            fields: {
              cancel: { kind: "bool", offset: 0 },
              resume: { kind: "bool", offset: 1 },
              override: { kind: "bool", offset: 2 },
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

  const DRIVER_MONITORING_SCHEMA = {
    fields: {
      lockout: { kind: "bool", offset: 0 },
      alertLevel: { kind: "enum", offset: 2, values: ["none", "one", "two", "three"] },
      visionPolicyState: {
        kind: "struct",
        offset: 1,
        schema: { fields: { isDistracted: { kind: "bool", offset: 8 } } },
      },
    },
  };

  const NAVI_ITEM_META_SCHEMA = {
    fields: {
      present: { kind: "bool", offset: 0 },
    },
  };

  const NAVI_VEHICLE_SCHEMA = {
    fields: {
      meta: { kind: "struct", offset: 0, schema: NAVI_ITEM_META_SCHEMA },
      latitude: { kind: "float64", offset: 0 },
      longitude: { kind: "float64", offset: 1 },
      headingDeg: { kind: "float32", offset: 4 },
      speedKph: { kind: "float32", offset: 5 },
      roadName: { kind: "text", offset: 1 },
    },
  };

  const NAVI_GUIDANCE_SCHEMA = {
    fields: {
      meta: { kind: "struct", offset: 0, schema: NAVI_ITEM_META_SCHEMA },
      distanceM: { kind: "int32", offset: 0 },
      timeSec: { kind: "int32", offset: 1 },
      turnType: { kind: "int32", offset: 2 },
      roadName: { kind: "text", offset: 1 },
      mainText: { kind: "text", offset: 2 },
      pointValid: { kind: "bool", offset: 96 },
      latitude: { kind: "float64", offset: 2 },
      longitude: { kind: "float64", offset: 3 },
    },
  };

  const NAVI_LANE_SCHEMA = {
    fields: {
      meta: { kind: "struct", offset: 0, schema: NAVI_ITEM_META_SCHEMA },
      count: { kind: "int16", offset: 0 },
      distanceM: { kind: "int32", offset: 1 },
      visible: { kind: "bool", offset: 16 },
      available: { kind: "list<int16>", offset: 3 },
    },
  };

  const NAVI_SPEED_SCHEMA = {
    fields: {
      roadLimitValid: { kind: "bool", offset: 32 },
      roadLimitKph: { kind: "int16", offset: 3 },
      sdiPresent: { kind: "bool", offset: 33 },
      sdiType: { kind: "int32", offset: 2 },
      sdiDistanceM: { kind: "int32", offset: 3 },
      sdiSpeedLimitKph: { kind: "int16", offset: 8 },
      sectionPresent: { kind: "bool", offset: 34 },
      sectionActive: { kind: "bool", offset: 35 },
      sectionSpeedLimitKph: { kind: "int16", offset: 9 },
      sectionAverageKph: { kind: "float32", offset: 5 },
      sectionRemainingDistanceM: { kind: "float32", offset: 7 },
      sectionProgress: { kind: "float32", offset: 9 },
    },
  };

  const NAVI_SIGNAL_SCHEMA = {
    fields: {
      visible: { kind: "bool", offset: 0 },
      distanceM: { kind: "int32", offset: 1 },
      redValid: { kind: "bool", offset: 1 },
      redOn: { kind: "bool", offset: 2 },
      leftValid: { kind: "bool", offset: 3 },
      leftOn: { kind: "bool", offset: 4 },
      greenValid: { kind: "bool", offset: 5 },
      greenOn: { kind: "bool", offset: 6 },
      rightValid: { kind: "bool", offset: 7 },
      rightOn: { kind: "bool", offset: 8 },
      uturnValid: { kind: "bool", offset: 9 },
      uturnOn: { kind: "bool", offset: 10 },
    },
  };

  const NAVI_CROSSROAD_SCHEMA = {
    fields: {
      visible: { kind: "bool", offset: 0 },
      distanceM: { kind: "int32", offset: 1 },
      imageCode: { kind: "int32", offset: 2 },
    },
  };

  const NAVI_COORDINATE_SCHEMA = {
    fields: {
      latitude: { kind: "float64", offset: 0 },
      longitude: { kind: "float64", offset: 1 },
    },
  };

  const NAVI_ROUTE_SCHEMA = {
    fields: {
      meta: { kind: "struct", offset: 0, schema: NAVI_ITEM_META_SCHEMA },
      remainingDistanceM: { kind: "int32", offset: 0 },
      remainingTimeSec: { kind: "int32", offset: 1 },
      totalDistanceM: { kind: "int32", offset: 4 },
      polyline: { kind: "list<struct>", offset: 1, schema: NAVI_COORDINATE_SCHEMA },
    },
  };

  const NAVI_STATUS_SCHEMA = {
    fields: {
      guidanceActive: { kind: "bool", offset: 0 },
      offRoute: { kind: "bool", offset: 1 },
      routePresent: { kind: "bool", offset: 2 },
    },
  };

  const CARROT_NAVI_SCHEMA = {
    fields: {
      schemaVersion: { kind: "uint16", offset: 0 },
      generation: { kind: "uint64", offset: 1 },
      sessionId: { kind: "text", offset: 0 },
      publishMonoTimeNanos: { kind: "uint64", offset: 2 },
      connected: { kind: "bool", offset: 16 },
      vehicle: { kind: "struct", offset: 1, schema: NAVI_VEHICLE_SCHEMA },
      guidanceCurrent: { kind: "struct", offset: 2, schema: NAVI_GUIDANCE_SCHEMA },
      guidanceNext: { kind: "struct", offset: 3, schema: NAVI_GUIDANCE_SCHEMA },
      laneCurrent: { kind: "struct", offset: 4, schema: NAVI_LANE_SCHEMA },
      speed: { kind: "struct", offset: 6, schema: NAVI_SPEED_SCHEMA },
      trafficSignal: { kind: "struct", offset: 7, schema: NAVI_SIGNAL_SCHEMA },
      crossroad: { kind: "struct", offset: 8, schema: NAVI_CROSSROAD_SCHEMA },
      route: { kind: "struct", offset: 9, schema: NAVI_ROUTE_SCHEMA },
      navigationStatus: { kind: "struct", offset: 10, schema: NAVI_STATUS_SCHEMA },
    },
  };

  const ONROAD_EVENT_SCHEMA = {
    fields: {
      // Keep this numeric. Summary policy owns the small set of EventName
      // ordinals it groups instead of duplicating the full presentation enum.
      name: { kind: "enum", offset: 0 },
    },
  };

  const REPLAY_SCHEMAS = {
    ...HUD_SCHEMAS,
    ...OVERLAY_SCHEMAS,
    initData: { fields: { wallTimeNanos: { kind: "uint64", offset: 1 } } },
    clocks: { fields: { wallTimeNanos: { kind: "uint64", offset: 3 } } },
    carParams: {
      fields: {
        wheelbase: { kind: "float32", offset: 5 },
        steerRatio: { kind: "float32", offset: 7 },
      },
    },
    onroadEvents: { rootKind: "list<struct>", schema: ONROAD_EVENT_SCHEMA },
    carrotNavi: CARROT_NAVI_SCHEMA,
    driverMonitoringState: DRIVER_MONITORING_SCHEMA,
    qRoadEncodeIdx: {
      fields: {
        frameId: { kind: "uint32", offset: 0 },
        segmentNum: { kind: "int32", offset: 3 },
        segmentId: { kind: "uint32", offset: 4 },
        timestampSof: { kind: "uint64", offset: 3 },
      },
    },
  };

  // cereal Event union discriminants from the checked-in log.capnp schema.
  // These are wire discriminants, not the explicit @field ordinals.
  const EVENT_SERVICE_BY_DISCRIMINANT = new Map([
    [0, "initData"],
    [1, "roadCameraState"],
    [5, "deviceState"],
    [6, "controlsState"],
    [12, "radarState"],
    [18, "liveCalibration"],
    [21, "carState"],
    [22, "carControl"],
    [23, "longitudinalPlan"],
    [34, "clocks"],
    [47, "gpsLocationExternal"],
    [60, "liveParameters"],
    [62, "cameraOdometry"],
    [63, "lateralPlan"],
    [67, "carParams"],
    [73, "modelV2"],
    [78, "peripheralState"],
    [88, "qRoadEncodeIdx"],
    [92, "liveTorqueParameters"],
    [105, "carrotMan"],
    [106, "carrotNavi"],
    // 판별자는 @N 이 아니라 "union 멤버를 @N 순으로 정렬했을 때의 순번"이다.
    // 삭제된 멤버가 남긴 @N 구멍 때문에 둘이 어긋난다(livePose 는 @129 인데 127).
    [127, "livePose"],
    [128, "selfdriveState"],
    [129, "liveTracks"],
    [132, "onroadEvents"],
    [144, "liveDelay"],
    [149, "driverMonitoringState"],
  ]);

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

  function framedMessageByteLength(data) {
    const bytes = data instanceof Uint8Array ? data : new Uint8Array(data);
    if (bytes.byteLength < 4) return 0;
    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    const segmentCount = view.getUint32(0, true) + 1;
    if (segmentCount <= 0 || segmentCount > 512) throw new Error("invalid capnp segment count");
    const tableBytes = 4 + segmentCount * 4 + (segmentCount % 2 === 0 ? 4 : 0);
    if (bytes.byteLength < tableBytes) return 0;
    let wordCount = 0;
    for (let index = 0; index < segmentCount; index += 1) {
      wordCount += view.getUint32(4 + index * 4, true);
      if (wordCount > 64 * 1024 * 1024) throw new Error("capnp message too large");
    }
    const byteLength = tableBytes + wordCount * 8;
    return bytes.byteLength >= byteLength ? byteLength : 0;
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
      case "uint16":
        return message.view.getUint16(base + offset * 2, true);
      case "bool": {
        const byteOffset = Math.floor(offset / 8);
        const bitIndex = offset % 8;
        return ((message.view.getUint8(base + byteOffset) >>> bitIndex) & 0x1) === 1;
      }
      case "uint32":
        return message.view.getUint32(base + offset * 4, true);
      case "uint64": {
        const byteOffset = base + offset * 8;
        const lo = message.view.getUint32(byteOffset, true);
        const hi = message.view.getUint32(byteOffset + 4, true);
        return hi * 0x100000000 + lo;
      }
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

  function readInt16List(message, structRef, slotIndex) {
    const listRef = readListSlot(message, structRef, slotIndex);
    if (!listRef) return [];
    if (listRef.elementSizeCode !== 3) {
      throw new Error(`unexpected int16 list size code ${listRef.elementSizeCode}`);
    }
    const segment = getSegment(message, listRef.segmentIndex);
    const byteOffset = segment.byteOffset + listRef.wordOffset * 8;
    const out = [];
    for (let i = 0; i < listRef.elementCount; i++) {
      out.push(message.view.getInt16(byteOffset + i * 2, true));
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
      case "int16":
      case "uint16":
      case "uint32":
      case "uint64":
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
      case "list<int16>":
        return readInt16List(message, structRef, fieldSpec.offset);
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
    const decoded = decodeStruct(message, payloadRef, schema);
    if (service === "controlsState" && decoded?.torqueState) {
      Object.assign(decoded, decoded.torqueState);
      delete decoded.torqueState;
    }
    return decoded;
  }

  function decodeReplayEvent(data, expectedService = "", excludedService = "", includedServices = null) {
    const message = parseMessage(data);
    const eventRef = readStructPointer(message, 0, 0);
    if (!eventRef) return null;
    const base = structDataByteOffset(message, eventRef);
    if (eventRef.dataWords < 2) return null;
    const discriminant = message.view.getUint16(base + 8, true);
    const service = EVENT_SERVICE_BY_DISCRIMINANT.get(discriminant);
    if (!service) return null;
    if (expectedService && service !== expectedService) return null;
    if (excludedService && service === excludedService) return null;
    if (includedServices?.has && !includedServices.has(service)) return null;
    const logMonoTime = readScalar(message, eventRef, "uint64", 0);
    // Event.valid has a wire default of true, so the stored bit is inverted.
    const valid = !readScalar(message, eventRef, "bool", 80);
    const schema = REPLAY_SCHEMAS[service];
    let decoded;
    if (schema?.rootKind === "list<struct>") {
      decoded = readStructList(message, eventRef, 0)
        .map((itemRef) => decodeStruct(message, itemRef, schema.schema));
    } else {
      const payloadRef = readStructSlot(message, eventRef, 0);
      if (!payloadRef) return null;
      decoded = decodeStruct(message, payloadRef, schema);
    }
    if (service === "controlsState" && decoded?.torqueState) {
      Object.assign(decoded, decoded.torqueState);
      delete decoded.torqueState;
    }
    if (service === "carrotNavi") {
      for (const key of ["vehicle", "guidanceCurrent", "guidanceNext", "laneCurrent", "route"]) {
        const value = decoded?.[key];
        if (!value || typeof value !== "object") continue;
        value.present = value.meta?.present === true;
        delete value.meta;
      }
    }
    return { service, logMonoTime, valid, decoded };
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
      // LFA(측면제어) 활성 = openpilot engaged 신호. 신규 HUD 당근 아이콘용.
      lfaActive: Boolean(state?.selfdriveState?.enabled ?? controlsState?.enabled),
      steeringAngleDeg: Number(carState?.steeringAngleDeg), // 클러스터: 당근 아이콘 조향각 회전
      aEgo: Number(carState?.aEgo),                          // 종가속(accel 게이지)
      steerOutput: Number(controlsState?.torqueState?.lateralOutput), // 조향출력 -1..1 (steer 게이지)
      leftBlinker: Boolean(carState?.leftBlinker),           // 방향지시등
      rightBlinker: Boolean(carState?.rightBlinker),
      trafficState: Number(carrotMan?.trafficState),         // 신호등 1=red 2=green
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
    framedMessageByteLength,
    decodeReplayEvent,
    deriveHudPayload,
  };
})();
