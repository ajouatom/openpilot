#include "selfdrive/carrot/realtime/compact_state_native.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>

#include <capnp/dynamic.h>
#include <capnp/serialize.h>
#include <kj/array.h>
#include <kj/exception.h>

#include "cereal/gen/cpp/log.capnp.h"

namespace {

using DynamicReader = capnp::DynamicStruct::Reader;
using DynamicValue = capnp::DynamicValue::Reader;

template <typename T>
void append_scalar(std::string &out, T value) {
  out.append(reinterpret_cast<const char *>(&value), sizeof(value));
}

float finite_float(double value) {
  const float converted = static_cast<float>(value);
  return std::isfinite(converted) ? converted : 0.0f;
}

double finite_double(double value) {
  return std::isfinite(value) ? value : 0.0;
}

std::optional<DynamicValue> find_value(const DynamicReader &reader, const char *name) {
  try {
    return reader.get(reader.getSchema().getFieldByName(name));
  } catch (const kj::Exception &) {
    // The only expected miss is an inactive union member, such as a lateral
    // controller other than torqueState. Normal fields must still be read so
    // Cap'n Proto schema defaults (for example radarTrackId=-1) are preserved.
    return std::nullopt;
  }
}

template <typename... Rest>
std::optional<DynamicValue> find_value(const DynamicReader &reader, const char *name, Rest... rest) {
  const auto value = find_value(reader, name);
  if (!value.has_value()) return std::nullopt;
  return find_value(value->template as<capnp::DynamicStruct>(), rest...);
}

template <typename... Names>
std::optional<DynamicReader> find_struct(const DynamicReader &reader, Names... names) {
  const auto value = find_value(reader, names...);
  if (!value.has_value()) return std::nullopt;
  return value->template as<capnp::DynamicStruct>();
}

template <typename... Names>
void append_bool(std::string &out, const DynamicReader &reader, Names... names) {
  const auto value = find_value(reader, names...);
  append_scalar(out, static_cast<uint8_t>(value.has_value() && value->template as<bool>()));
}

template <typename... Names>
void append_i8(std::string &out, const DynamicReader &reader, Names... names) {
  const auto value = find_value(reader, names...);
  const int64_t number = value.has_value() ? value->template as<int64_t>() : 0;
  append_scalar(out, static_cast<int8_t>(std::clamp<int64_t>(number, INT8_MIN, INT8_MAX)));
}

template <typename... Names>
void append_i16(std::string &out, const DynamicReader &reader, Names... names) {
  const auto value = find_value(reader, names...);
  const int64_t number = value.has_value() ? value->template as<int64_t>() : 0;
  append_scalar(out, static_cast<int16_t>(std::clamp<int64_t>(number, INT16_MIN, INT16_MAX)));
}

template <typename... Names>
void append_i32(std::string &out, const DynamicReader &reader, Names... names) {
  const auto value = find_value(reader, names...);
  const int64_t number = value.has_value() ? value->template as<int64_t>() : 0;
  append_scalar(out, static_cast<int32_t>(std::clamp<int64_t>(number, INT32_MIN, INT32_MAX)));
}

template <typename... Names>
void append_u32(std::string &out, const DynamicReader &reader, Names... names) {
  const auto value = find_value(reader, names...);
  const uint64_t number = value.has_value() ? value->template as<uint64_t>() : 0;
  append_scalar(out, static_cast<uint32_t>(std::min<uint64_t>(number, UINT32_MAX)));
}

template <typename... Names>
void append_f32(std::string &out, const DynamicReader &reader, Names... names) {
  const auto value = find_value(reader, names...);
  append_scalar(out, finite_float(value.has_value() ? value->template as<double>() : 0.0));
}

template <typename... Names>
void append_f64(std::string &out, const DynamicReader &reader, Names... names) {
  const auto value = find_value(reader, names...);
  append_scalar(out, finite_double(value.has_value() ? value->template as<double>() : 0.0));
}

template <typename... Names>
void append_enum(std::string &out, const DynamicReader &reader, Names... names) {
  const auto value = find_value(reader, names...);
  const uint16_t raw = value.has_value() ? value->template as<capnp::DynamicEnum>().getRaw() : 0;
  append_scalar(out, static_cast<uint8_t>(std::min<uint16_t>(raw, UINT8_MAX)));
}

template <typename... Names>
void append_text(std::string &out, const DynamicReader &reader, Names... names) {
  const auto value = find_value(reader, names...);
  if (!value.has_value()) {
    append_scalar(out, static_cast<uint16_t>(0));
    return;
  }
  const auto text = value->template as<capnp::Text>();
  const uint16_t size = std::min<size_t>(text.size(), UINT16_MAX);
  append_scalar(out, size);
  out.append(text.begin(), size);
}

void append_f32_list_value(std::string &out, const std::optional<DynamicValue> &value, bool first_only = false) {
  if (!value.has_value()) {
    append_scalar(out, static_cast<uint16_t>(0));
    return;
  }
  const auto list = value->as<capnp::DynamicList>();
  const uint16_t size = std::min<size_t>(list.size(), first_only ? 1 : UINT16_MAX);
  append_scalar(out, size);
  for (uint16_t i = 0; i < size; ++i) append_scalar(out, finite_float(list[i].as<double>()));
}

template <typename... Names>
void append_f32_list(std::string &out, const DynamicReader &reader, Names... names) {
  append_f32_list_value(out, find_value(reader, names...));
}

template <typename... Names>
void append_first_f32_list(std::string &out, const DynamicReader &reader, Names... names) {
  append_f32_list_value(out, find_value(reader, names...), true);
}

void append_quantized_list_value(std::string &out, const std::optional<DynamicValue> &value,
                                 float scale, int minimum, int maximum, bool is_signed) {
  if (!value.has_value()) {
    append_scalar(out, static_cast<uint16_t>(0));
    return;
  }
  const auto list = value->as<capnp::DynamicList>();
  const uint16_t size = std::min<size_t>(list.size(), UINT16_MAX);
  append_scalar(out, size);
  for (uint16_t i = 0; i < size; ++i) {
    const int quantized = std::clamp(static_cast<int>(std::nearbyint(finite_float(list[i].as<double>()) * scale)),
                                     minimum, maximum);
    if (is_signed) append_scalar(out, static_cast<int16_t>(quantized));
    else append_scalar(out, static_cast<uint16_t>(quantized));
  }
}

void append_empty_xyz(std::string &out) {
  append_scalar(out, static_cast<uint16_t>(0));
  append_scalar(out, static_cast<uint16_t>(0));
  append_scalar(out, static_cast<uint16_t>(0));
}

void append_xyz(std::string &out, const DynamicReader &value) {
  append_quantized_list_value(out, find_value(value, "x"), 100.0f, 0, UINT16_MAX, false);
  append_quantized_list_value(out, find_value(value, "y"), 1000.0f, INT16_MIN, INT16_MAX, true);
  append_quantized_list_value(out, find_value(value, "z"), 1000.0f, INT16_MIN, INT16_MAX, true);
}

void append_xyz_field(std::string &out, const DynamicReader &reader, const char *name) {
  const auto value = find_struct(reader, name);
  if (value.has_value()) append_xyz(out, *value);
  else append_empty_xyz(out);
}

void append_xyz_list(std::string &out, const DynamicReader &reader, const char *name) {
  const auto value = find_value(reader, name);
  if (!value.has_value()) {
    append_scalar(out, static_cast<uint8_t>(0));
    return;
  }
  const auto list = value->as<capnp::DynamicList>();
  const uint8_t size = std::min<size_t>(list.size(), UINT8_MAX);
  append_scalar(out, size);
  for (uint8_t i = 0; i < size; ++i) append_xyz(out, list[i].as<capnp::DynamicStruct>());
}

void append_model_leads(std::string &out, const DynamicReader &reader) {
  const auto value = find_value(reader, "leadsV3");
  if (!value.has_value()) {
    append_scalar(out, static_cast<uint8_t>(0));
    return;
  }
  const auto list = value->as<capnp::DynamicList>();
  const uint8_t size = std::min<size_t>(list.size(), UINT8_MAX);
  append_scalar(out, size);
  for (uint8_t i = 0; i < size; ++i) {
    const auto lead = list[i].as<capnp::DynamicStruct>();
    append_f32(out, lead, "prob");
    append_quantized_list_value(out, find_value(lead, "x"), 100.0f, 0, UINT16_MAX, false);
    // Mirrors MODEL_LEAD_SCHEMA in compact_state.py: sample [0] only.
    append_first_f32_list(out, lead, "y");
    append_first_f32_list(out, lead, "v");
  }
}

void append_radar_points(std::string &out, const DynamicReader &reader) {
  const auto value = find_value(reader, "points");
  if (!value.has_value()) {
    append_scalar(out, static_cast<uint8_t>(0));
    return;
  }
  const auto list = value->as<capnp::DynamicList>();
  const uint8_t size = std::min<size_t>(list.size(), UINT8_MAX);
  append_scalar(out, size);
  for (uint8_t i = 0; i < size; ++i) {
    const auto point = list[i].as<capnp::DynamicStruct>();
    append_u32(out, point, "trackId");
    append_f32(out, point, "dRel");
    append_f32(out, point, "yRel");
    append_f32(out, point, "vRel");
    append_bool(out, point, "measured");
    append_enum(out, point, "radarSource");
  }
}

void append_radar_lead(std::string &out, const DynamicReader &lead) {
  append_f32(out, lead, "dRel");
  append_f32(out, lead, "yRel");
  append_f32(out, lead, "vRel");
  append_f32(out, lead, "aRel");
  append_f32(out, lead, "vLead");
  append_f32(out, lead, "aLead");
  append_f32(out, lead, "dPath");
  append_f32(out, lead, "vLat");
  append_f32(out, lead, "vLeadK");
  append_f32(out, lead, "aLeadK");
  append_bool(out, lead, "fcw");
  append_bool(out, lead, "status");
  append_f32(out, lead, "aLeadTau");
  append_f32(out, lead, "modelProb");
  append_bool(out, lead, "radar");
  append_i32(out, lead, "radarTrackId");
  append_f32(out, lead, "jLead");
  append_f32(out, lead, "score");
}

void append_radar_lead_field(std::string &out, const DynamicReader &reader, const char *name) {
  const auto lead = find_struct(reader, name);
  if (lead.has_value()) append_radar_lead(out, *lead);
  else {
    for (int i = 0; i < 10; ++i) append_scalar(out, 0.0f);
    append_scalar(out, static_cast<uint8_t>(0));
    append_scalar(out, static_cast<uint8_t>(0));
    append_scalar(out, 0.0f);
    append_scalar(out, 0.0f);
    append_scalar(out, static_cast<uint8_t>(0));
    append_scalar(out, static_cast<int32_t>(0));
    append_scalar(out, 0.0f);
    append_scalar(out, 0.0f);
  }
}

void append_radar_lead_list(std::string &out, const DynamicReader &reader, const char *name) {
  const auto value = find_value(reader, name);
  if (!value.has_value()) {
    append_scalar(out, static_cast<uint8_t>(0));
    return;
  }
  const auto list = value->as<capnp::DynamicList>();
  const uint8_t size = std::min<size_t>(list.size(), UINT8_MAX);
  append_scalar(out, size);
  for (uint8_t i = 0; i < size; ++i) append_radar_lead(out, list[i].as<capnp::DynamicStruct>());
}

void encode_car_state(std::string &out, const DynamicReader &value) {
  append_f32(out, value, "vEgo");
  append_f32(out, value, "aEgo");
  append_f32(out, value, "vEgoCluster");
  append_f32(out, value, "vCruiseCluster");
  append_f32(out, value, "steeringAngleDeg");
  append_bool(out, value, "brakeHoldActive");
  append_i16(out, value, "softHoldActive");
  append_i16(out, value, "carrotCruise");
  append_i16(out, value, "gearStep");
  append_f32(out, value, "useLaneLineSpeed");
  append_bool(out, value, "brakeLights");
  append_bool(out, value, "leftBlindspot");
  append_bool(out, value, "rightBlindspot");
  append_i16(out, value, "leftLaneLine");
  append_i16(out, value, "rightLaneLine");
  append_enum(out, value, "gearShifter");
  append_bool(out, value, "leftBlinker");
  append_bool(out, value, "rightBlinker");
  append_f32(out, value, "fuelGauge");
  append_f32(out, value, "ureaGauge");
  append_f32(out, value, "tpms", "fl");
  append_f32(out, value, "tpms", "fr");
  append_f32(out, value, "tpms", "rl");
  append_f32(out, value, "tpms", "rr");
}

void encode_controls_state(std::string &out, const DynamicReader &value) {
  append_bool(out, value, "deprecated", "enabled");
  append_f32(out, value, "deprecated", "vCruiseCluster");
  append_bool(out, value, "activeLaneLine");
  append_f32(out, value, "curvature");
  append_f32(out, value, "desiredCurvature");
  append_f32(out, value, "lateralControlState", "torqueState", "actualLateralAccel");
  append_f32(out, value, "lateralControlState", "torqueState", "desiredLateralAccel");
  append_f32(out, value, "lateralControlState", "torqueState", "output");
}

void encode_device_state(std::string &out, const DynamicReader &value) {
  append_i8(out, value, "memoryUsagePercent");
  append_f32(out, value, "freeSpacePercent");
  append_f32_list(out, value, "cpuTempC");
  append_enum(out, value, "deviceType");
}

void encode_carrot_man(std::string &out, const DynamicReader &value) {
  append_i32(out, value, "activeCarrot");
  append_i32(out, value, "nRoadLimitSpeed");
  append_i32(out, value, "xSpdType");
  append_i32(out, value, "xSpdLimit");
  append_i32(out, value, "xSpdDist");
  append_i32(out, value, "xSpdCountDown");
  append_i32(out, value, "xTurnInfo");
  append_i32(out, value, "xDistToTurn");
  append_i32(out, value, "xTurnCountDown");
  append_text(out, value, "atcType");
  append_text(out, value, "szPosRoadName");
  append_text(out, value, "szTBTMainText");
  append_i32(out, value, "desiredSpeed");
  append_f32(out, value, "xPosLat");
  append_f32(out, value, "xPosLon");
  append_f32(out, value, "xPosAngle");
  append_f32(out, value, "xPosSpeed");
  append_i32(out, value, "trafficState");
  append_i32(out, value, "nGoPosDist");
  append_i32(out, value, "nGoPosTime");
  append_text(out, value, "szSdiDescr");
  append_text(out, value, "naviPaths");
  append_text(out, value, "desiredSource");
}

void encode_selfdrive_state(std::string &out, const DynamicReader &value) {
  append_bool(out, value, "enabled");
  append_enum(out, value, "personality");
  append_enum(out, value, "alertStatus");
  append_enum(out, value, "alertSize");
  append_text(out, value, "alertType");
  append_text(out, value, "alertText1");
  append_text(out, value, "alertText2");
}

void encode_gps(std::string &out, const DynamicReader &value) {
  append_f64(out, value, "latitude");
  append_f64(out, value, "longitude");
  append_f32(out, value, "speed");
  append_f32(out, value, "bearingDeg");
  append_f32(out, value, "bearingAccuracyDeg");
  append_f32(out, value, "speedAccuracy");
  append_bool(out, value, "hasFix");
}

void encode_longitudinal_plan(std::string &out, const DynamicReader &value) {
  append_first_f32_list(out, value, "accels");
  append_first_f32_list(out, value, "speeds");
  append_first_f32_list(out, value, "jerks");
  append_f32(out, value, "tFollow");
  append_f32(out, value, "desiredDistance");
  append_i32(out, value, "myDrivingMode");
  append_i32(out, value, "xState");
  append_i32(out, value, "trafficState");
  append_enum(out, value, "longitudinalPlanSource");
}

void encode_model_v2(std::string &out, const DynamicReader &value) {
  append_u32(out, value, "frameId");
  append_u32(out, value, "frameIdExtra");
  append_xyz_field(out, value, "position");
  const auto velocity = find_struct(value, "velocity");
  if (velocity.has_value()) {
    append_quantized_list_value(out, find_value(*velocity, "x"), 100.0f, INT16_MIN, INT16_MAX, true);
  } else {
    append_scalar(out, static_cast<uint16_t>(0));
  }
  append_xyz_list(out, value, "laneLines");
  append_f32_list(out, value, "laneLineProbs");
  append_xyz_list(out, value, "roadEdges");
  append_f32_list(out, value, "roadEdgeStds");
  append_model_leads(out, value);
}

void encode_live_calibration(std::string &out, const DynamicReader &value) {
  append_enum(out, value, "calStatus");
  append_i32(out, value, "calCycle");
  append_i8(out, value, "calPerc");
  append_i32(out, value, "validBlocks");
  append_f32_list(out, value, "rpyCalib");
  append_f32_list(out, value, "height");
}

void encode_lateral_plan(std::string &out, const DynamicReader &value) {
  append_bool(out, value, "useLaneLines");
  append_text(out, value, "latDebugText");
  append_xyz_field(out, value, "position");
  append_f32_list(out, value, "distances");
  append_enum(out, value, "laneChangeState");
  append_enum(out, value, "laneChangeDirection");
}

void encode_radar_state(std::string &out, const DynamicReader &value) {
  append_radar_lead_field(out, value, "leadOne");
  append_radar_lead_field(out, value, "leadTwo");
  append_radar_lead_field(out, value, "leadRight");
  append_radar_lead_field(out, value, "leadLeft");
  append_radar_lead_list(out, value, "leadsLeft");
  append_radar_lead_list(out, value, "leadsCenter");
  append_radar_lead_list(out, value, "leadsRight");
  append_radar_lead_list(out, value, "leadsLeft2");
  append_radar_lead_list(out, value, "leadsRight2");
  append_radar_lead_list(out, value, "leadsCutIn");
}

void encode_car_control(std::string &out, const DynamicReader &value) {
  append_bool(out, value, "latActive");
  append_bool(out, value, "longActive");
  append_f32(out, value, "actuators", "steeringAngleDeg");
  append_f32(out, value, "actuators", "accel");
  append_f32(out, value, "actuators", "curvature");
}

uint8_t service_id(const std::string &service) {
  if (service == "carState") return 1;
  if (service == "controlsState") return 2;
  if (service == "deviceState") return 3;
  if (service == "peripheralState") return 4;
  if (service == "carrotMan") return 5;
  if (service == "selfdriveState") return 6;
  if (service == "gpsLocationExternal") return 7;
  if (service == "longitudinalPlan") return 8;
  if (service == "modelV2") return 9;
  if (service == "liveCalibration") return 10;
  if (service == "roadCameraState") return 11;
  if (service == "lateralPlan") return 12;
  if (service == "radarState") return 13;
  if (service == "carControl") return 14;
  if (service == "liveDelay") return 15;
  if (service == "liveTorqueParameters") return 16;
  if (service == "liveParameters") return 17;
  if (service == "liveTracks") return 18;
  throw std::invalid_argument("unsupported compact state service");
}

void encode_service(std::string &out, const std::string &service, const DynamicReader &value) {
  if (service == "carState") encode_car_state(out, value);
  else if (service == "controlsState") encode_controls_state(out, value);
  else if (service == "deviceState") encode_device_state(out, value);
  else if (service == "peripheralState") append_u32(out, value, "voltage");
  else if (service == "carrotMan") encode_carrot_man(out, value);
  else if (service == "selfdriveState") encode_selfdrive_state(out, value);
  else if (service == "gpsLocationExternal") encode_gps(out, value);
  else if (service == "longitudinalPlan") encode_longitudinal_plan(out, value);
  else if (service == "modelV2") encode_model_v2(out, value);
  else if (service == "liveCalibration") encode_live_calibration(out, value);
  else if (service == "roadCameraState") {
    append_u32(out, value, "frameId");
    append_enum(out, value, "sensor");
  } else if (service == "lateralPlan") encode_lateral_plan(out, value);
  else if (service == "radarState") encode_radar_state(out, value);
  else if (service == "carControl") encode_car_control(out, value);
  else if (service == "liveDelay") {
    append_f32(out, value, "lateralDelay");
    append_i8(out, value, "calPerc");
  } else if (service == "liveTorqueParameters") {
    append_bool(out, value, "liveValid");
    append_f32(out, value, "latAccelFactorFiltered");
    append_f32(out, value, "frictionCoefficientFiltered");
    append_i8(out, value, "calPerc");
  } else if (service == "liveParameters") {
    append_f32(out, value, "angleOffsetDeg");
    append_f32(out, value, "steerRatio");
  } else if (service == "liveTracks") append_radar_points(out, value);
}

}  // namespace

std::string encode_carrot_state_compact_frame(const char *service_data, size_t service_size,
                                              const char *data, size_t size, uint16_t sequence) {
  if (service_data == nullptr || service_size == 0) throw std::invalid_argument("empty compact state service");
  if (data == nullptr || size == 0) throw std::invalid_argument("empty compact state event");

  const std::string service(service_data, service_size);
  const uint8_t id = service_id(service);

  auto aligned = kj::heapArray<capnp::word>((size + sizeof(capnp::word) - 1) / sizeof(capnp::word));
  std::memset(aligned.begin(), 0, aligned.size() * sizeof(capnp::word));
  std::memcpy(aligned.begin(), data, size);
  capnp::FlatArrayMessageReader reader(kj::ArrayPtr<const capnp::word>(aligned.begin(), aligned.size()));
  const auto event = reader.getRoot<cereal::Event>();
  const capnp::DynamicStruct::Reader dynamic_event(event);
  const auto union_field = dynamic_event.getSchema().getFieldByName(service.c_str());
  if (union_field.getProto().getDiscriminantValue() != static_cast<uint16_t>(event.which())) {
    throw std::invalid_argument("compact state event/service mismatch");
  }
  const auto value = dynamic_event.get(union_field).as<capnp::DynamicStruct>();

  std::string out;
  out.reserve(service == "modelV2" ? 2048 : 512);
  out.append("CVS1", 4);
  append_scalar(out, id);
  append_scalar(out, static_cast<uint8_t>(0));
  append_scalar(out, sequence);
  encode_service(out, service, value);
  return out;
}
