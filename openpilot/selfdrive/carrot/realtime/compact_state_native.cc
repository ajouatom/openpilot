#include "selfdrive/carrot/realtime/compact_state_native.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

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
void append_u16(std::string &out, const DynamicReader &reader, Names... names) {
  const auto value = find_value(reader, names...);
  const uint64_t number = value.has_value() ? value->template as<uint64_t>() : 0;
  append_scalar(out, static_cast<uint16_t>(std::min<uint64_t>(number, UINT16_MAX)));
}

template <typename... Names>
void append_u32(std::string &out, const DynamicReader &reader, Names... names) {
  const auto value = find_value(reader, names...);
  const uint64_t number = value.has_value() ? value->template as<uint64_t>() : 0;
  append_scalar(out, static_cast<uint32_t>(std::min<uint64_t>(number, UINT32_MAX)));
}

template <typename... Names>
void append_u64(std::string &out, const DynamicReader &reader, Names... names) {
  const auto value = find_value(reader, names...);
  const uint64_t number = value.has_value() ? value->template as<uint64_t>() : 0;
  append_scalar(out, number);
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

void append_xyz_measurement(std::string &out, const DynamicReader &reader, const char *name) {
  const auto m = find_struct(reader, name);
  if (!m.has_value()) {
    for (int i = 0; i < 6; ++i) append_scalar(out, 0.0f);
    append_scalar(out, static_cast<uint8_t>(0));
    return;
  }
  for (const char *k : {"x", "y", "z", "xStd", "yStd", "zStd"}) append_f32(out, *m, k);
  append_bool(out, *m, "valid");
}

void encode_camera_odometry(std::string &out, const DynamicReader &value) {
  append_u32(out, value, "frameId");
  append_u64(out, value, "timestampEof");
  append_f32_list(out, value, "trans");
  append_f32_list(out, value, "rot");
  append_f32_list(out, value, "transStd");
  append_f32_list(out, value, "rotStd");
}

void encode_live_pose(std::string &out, const DynamicReader &value) {
  append_xyz_measurement(out, value, "orientationNED");
  append_xyz_measurement(out, value, "velocityDevice");
  append_xyz_measurement(out, value, "accelerationDevice");
  append_xyz_measurement(out, value, "angularVelocityDevice");
  append_bool(out, value, "inputsOK");
  append_bool(out, value, "posenetOK");
  append_bool(out, value, "sensorsOK");
  append_u64(out, value, "timestamp");
}

// ── carrotNavi 압축 서브셋 (compact_state.py 와 1:1) ──────────────
void append_navi_present(std::string &out, const std::optional<DynamicReader> &item) {
  bool present = false;
  if (item.has_value()) {
    const auto meta = find_struct(*item, "meta");
    if (meta.has_value()) {
      const auto v = find_value(*meta, "present");
      present = v.has_value() && v->as<bool>();
    }
  }
  append_scalar(out, static_cast<uint8_t>(present ? 1 : 0));
}

void append_navi_guidance(std::string &out, const DynamicReader &navi, const char *name) {
  const auto g = find_struct(navi, name);
  append_navi_present(out, g);
  if (!g.has_value()) {
    append_scalar(out, static_cast<int32_t>(0));   // distanceM
    append_scalar(out, static_cast<int32_t>(0));   // timeSec
    append_scalar(out, static_cast<int32_t>(0));   // turnType
    append_scalar(out, static_cast<uint16_t>(0));  // roadName
    append_scalar(out, static_cast<uint16_t>(0));  // mainText
    append_scalar(out, static_cast<uint8_t>(0));   // pointValid
    append_scalar(out, 0.0);                       // latitude
    append_scalar(out, 0.0);                       // longitude
    return;
  }
  append_i32(out, *g, "distanceM");
  append_i32(out, *g, "timeSec");
  append_i32(out, *g, "turnType");
  append_text(out, *g, "roadName");
  append_text(out, *g, "mainText");
  append_bool(out, *g, "pointValid");
  append_f64(out, *g, "latitude");
  append_f64(out, *g, "longitude");
}

void append_i16_list(std::string &out, const DynamicReader &reader, const char *name) {
  const auto value = find_value(reader, name);
  if (!value.has_value()) {
    append_scalar(out, static_cast<uint16_t>(0));
    return;
  }
  const auto list = value->as<capnp::DynamicList>();
  const uint16_t size = std::min<size_t>(list.size(), UINT16_MAX);
  append_scalar(out, size);
  for (uint16_t i = 0; i < size; ++i) {
    const auto raw = list[i].as<int64_t>();
    append_scalar(out, static_cast<int16_t>(std::clamp<int64_t>(raw, INT16_MIN, INT16_MAX)));
  }
}

void append_navi_lane_value(std::string &out, const std::optional<DynamicReader> &lane) {
  append_navi_present(out, lane);
  if (!lane.has_value()) {
    append_scalar(out, static_cast<int16_t>(0));
    append_scalar(out, static_cast<int32_t>(0));
    append_scalar(out, static_cast<uint8_t>(0));
    append_scalar(out, static_cast<uint16_t>(0));
    return;
  }
  append_i16(out, *lane, "count");
  append_i32(out, *lane, "distanceM");
  append_bool(out, *lane, "visible");
  append_i16_list(out, *lane, "available");
}

void append_navi_lane(std::string &out, const DynamicReader &navi, const char *name) {
  append_navi_lane_value(out, find_struct(navi, name));
}

void append_navi_lane_list(std::string &out, const DynamicReader &navi, const char *name) {
  const auto value = find_value(navi, name);
  if (!value.has_value()) {
    append_scalar(out, static_cast<uint8_t>(0));
    return;
  }
  const auto list = value->as<capnp::DynamicList>();
  const uint8_t size = static_cast<uint8_t>(std::min<size_t>(list.size(), UINT8_MAX));
  append_scalar(out, size);
  for (uint8_t i = 0; i < size; ++i) {
    const std::optional<DynamicReader> lane = list[i].as<capnp::DynamicStruct>();
    append_navi_lane_value(out, lane);
  }
}

// 경로 폴리라인. compact_state.py 의 coord_list 와 1:1.
// u8 count + f64 앵커(lat,lon) + (count-1) x f32 델타.
constexpr size_t kRoutePolylineLimit = 64;

void append_coord_list(std::string &out, const DynamicReader &reader, const char *name) {
  const auto value = find_value(reader, name);
  std::vector<std::pair<double, double>> coords;
  if (value.has_value()) {
    const auto list = value->as<capnp::DynamicList>();
    const size_t n = std::min<size_t>(list.size(), kRoutePolylineLimit);
    coords.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      const auto c = list[i].as<capnp::DynamicStruct>();
      const auto la = find_value(c, "latitude");
      const auto lo = find_value(c, "longitude");
      if (!la.has_value() || !lo.has_value()) continue;
      const double lat = la->as<double>(), lon = lo->as<double>();
      if (std::isfinite(lat) && std::isfinite(lon)) coords.emplace_back(lat, lon);
    }
  }
  append_scalar(out, static_cast<uint8_t>(coords.size()));
  if (coords.empty()) return;
  append_scalar(out, coords[0].first);
  append_scalar(out, coords[0].second);
  for (size_t i = 1; i < coords.size(); ++i) {
    append_scalar(out, static_cast<float>(coords[i].first - coords[0].first));
    append_scalar(out, static_cast<float>(coords[i].second - coords[0].second));
  }
}

void encode_carrot_navi(std::string &out, const DynamicReader &value) {
  append_u16(out, value, "schemaVersion");
  append_u64(out, value, "generation");
  append_text(out, value, "sessionId");
  append_u64(out, value, "publishMonoTimeNanos");
  append_bool(out, value, "connected");

  // vehicle
  const auto veh = find_struct(value, "vehicle");
  append_navi_present(out, veh);
  if (veh.has_value()) {
    append_f64(out, *veh, "latitude");
    append_f64(out, *veh, "longitude");
    append_f32(out, *veh, "headingDeg");
    append_f32(out, *veh, "speedKph");
    append_text(out, *veh, "roadName");
  } else {
    append_scalar(out, 0.0); append_scalar(out, 0.0);
    append_scalar(out, 0.0f); append_scalar(out, 0.0f);
    append_scalar(out, static_cast<uint16_t>(0));
  }

  append_navi_guidance(out, value, "guidanceCurrent");
  append_navi_guidance(out, value, "guidanceNext");
  append_navi_lane(out, value, "laneCurrent");
  append_navi_lane_list(out, value, "laneAhead");

  // speed
  const auto sp = find_struct(value, "speed");
  if (sp.has_value()) {
    append_bool(out, *sp, "roadLimitValid");
    append_i16(out, *sp, "roadLimitKph");
    append_bool(out, *sp, "sdiPresent");
    append_i32(out, *sp, "sdiType");
    append_i32(out, *sp, "sdiDistanceM");
    append_i16(out, *sp, "sdiSpeedLimitKph");
    append_i32(out, *sp, "sdiSectionType");
    append_i32(out, *sp, "sdiBlockType");
    append_i16(out, *sp, "sdiBlockSpeedKph");
    append_i32(out, *sp, "sdiBlockDistanceM");
    append_bool(out, *sp, "secondarySdiPresent");
    append_i32(out, *sp, "secondarySdiType");
    append_i32(out, *sp, "secondarySdiDistanceM");
    append_i16(out, *sp, "secondarySdiSpeedLimitKph");
    append_i32(out, *sp, "secondarySdiSectionType");
    append_i32(out, *sp, "secondarySdiBlockType");
    append_i16(out, *sp, "secondarySdiBlockSpeedKph");
    append_i32(out, *sp, "secondarySdiBlockDistanceM");
    append_bool(out, *sp, "sectionPresent");
    append_bool(out, *sp, "sectionActive");
    append_i16(out, *sp, "sectionSpeedLimitKph");
    append_f32(out, *sp, "sectionAverageKph");
    append_f32(out, *sp, "sectionOverallAverageKph");
    append_f32(out, *sp, "sectionRemainingDistanceM");
    append_i32(out, *sp, "sectionRemainingTimeSec");
    append_f32(out, *sp, "sectionProgress");
    append_bool(out, *sp, "sectionSuspended");
    append_bool(out, *sp, "sectionOffRoute");
  } else {
    append_scalar(out, static_cast<uint8_t>(0)); append_scalar(out, static_cast<int16_t>(0));
    append_scalar(out, static_cast<uint8_t>(0)); append_scalar(out, static_cast<int32_t>(0));
    append_scalar(out, static_cast<int32_t>(0)); append_scalar(out, static_cast<int16_t>(0));
    append_scalar(out, static_cast<int32_t>(0)); append_scalar(out, static_cast<int32_t>(0));
    append_scalar(out, static_cast<int16_t>(0)); append_scalar(out, static_cast<int32_t>(0));
    append_scalar(out, static_cast<uint8_t>(0)); append_scalar(out, static_cast<int32_t>(0));
    append_scalar(out, static_cast<int32_t>(0)); append_scalar(out, static_cast<int16_t>(0));
    append_scalar(out, static_cast<int32_t>(0)); append_scalar(out, static_cast<int32_t>(0));
    append_scalar(out, static_cast<int16_t>(0)); append_scalar(out, static_cast<int32_t>(0));
    append_scalar(out, static_cast<uint8_t>(0)); append_scalar(out, static_cast<uint8_t>(0));
    append_scalar(out, static_cast<int16_t>(0));
    append_scalar(out, 0.0f); append_scalar(out, 0.0f); append_scalar(out, 0.0f);
    append_scalar(out, static_cast<int32_t>(0)); append_scalar(out, 0.0f);
    append_scalar(out, static_cast<uint8_t>(0)); append_scalar(out, static_cast<uint8_t>(0));
  }

  // trafficSignal
  const auto sg = find_struct(value, "trafficSignal");
  if (sg.has_value()) {
    append_bool(out, *sg, "visible");
    append_i32(out, *sg, "distanceM");
    append_bool(out, *sg, "redValid"); append_bool(out, *sg, "redOn"); append_i16(out, *sg, "redRemainSec");
    append_bool(out, *sg, "leftValid"); append_bool(out, *sg, "leftOn"); append_i16(out, *sg, "leftRemainSec");
    append_bool(out, *sg, "greenValid"); append_bool(out, *sg, "greenOn"); append_i16(out, *sg, "greenRemainSec");
    append_bool(out, *sg, "rightValid"); append_bool(out, *sg, "rightOn"); append_i16(out, *sg, "rightRemainSec");
    append_bool(out, *sg, "uturnValid"); append_bool(out, *sg, "uturnOn"); append_i16(out, *sg, "uturnRemainSec");
    append_bool(out, *sg, "uiCounterValid"); append_i16(out, *sg, "uiCounterRemainSec");
  } else {
    append_scalar(out, static_cast<uint8_t>(0));
    append_scalar(out, static_cast<int32_t>(0));
    for (int i = 0; i < 5; ++i) {
      append_scalar(out, static_cast<uint8_t>(0));
      append_scalar(out, static_cast<uint8_t>(0));
      append_scalar(out, static_cast<int16_t>(0));
    }
    append_scalar(out, static_cast<uint8_t>(0));
    append_scalar(out, static_cast<int16_t>(0));
  }

  // crossroad
  const auto cr = find_struct(value, "crossroad");
  if (cr.has_value()) {
    append_bool(out, *cr, "visible");
    append_i32(out, *cr, "distanceM");
    append_i32(out, *cr, "imageCode");
  } else {
    append_scalar(out, static_cast<uint8_t>(0));
    append_scalar(out, static_cast<int32_t>(0));
    append_scalar(out, static_cast<int32_t>(0));
  }

  // route
  const auto rt = find_struct(value, "route");
  append_navi_present(out, rt);
  if (rt.has_value()) {
    append_i32(out, *rt, "remainingDistanceM");
    append_i32(out, *rt, "remainingTimeSec");
    append_i32(out, *rt, "movedDistanceM");
    append_i32(out, *rt, "totalDistanceM");
    append_coord_list(out, *rt, "polyline");
  } else {
    for (int i = 0; i < 4; ++i) append_scalar(out, static_cast<int32_t>(0));
    append_scalar(out, static_cast<uint8_t>(0));
  }

  // navigationStatus
  const auto st = find_struct(value, "navigationStatus");
  if (st.has_value()) {
    append_bool(out, *st, "guidanceActive");
    append_bool(out, *st, "offRoute");
    append_bool(out, *st, "routePresent");
  } else {
    for (int i = 0; i < 3; ++i) append_scalar(out, static_cast<uint8_t>(0));
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
  // Appended at the end to match compact_state.py / vision_compact.js (schema
  // evolution: order must stay identical across all three). Green EV telltale.
  append_bool(out, value, "evModeValid");
  append_bool(out, value, "evModeActive");
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
  append_f64(out, value, "altitude");
  append_f32(out, value, "horizontalAccuracy");
  append_f32(out, value, "verticalAccuracy");
  append_f64(out, value, "unixTimestampMillis");
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
  // Appended at the end to match compact_state.py / vision_compact.js. Eco
  // cruise override telltale.
  append_f32(out, value, "cruiseTarget");
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
  append_f32_list(out, value, "laneLineStds");
  append_i32(out, value, "frameAge");
  append_f32(out, value, "frameDropPerc");
  append_f32(out, value, "modelExecutionTime");
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
  if (service == "cameraOdometry") return 19;
  if (service == "livePose") return 20;
  if (service == "carrotNavi") return 21;
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
    append_u64(out, value, "timestampEof");
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
  else if (service == "cameraOdometry") encode_camera_odometry(out, value);
  else if (service == "livePose") encode_live_pose(out, value);
  else if (service == "carrotNavi") encode_carrot_navi(out, value);
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
