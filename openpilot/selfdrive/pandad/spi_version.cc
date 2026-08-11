#include "selfdrive/pandad/spi_version.h"

#include <algorithm>

namespace panda::spi {
namespace {

constexpr size_t kLengthSize = 2U;
constexpr size_t kMetadataSize = kVersionUidSize + 3U;
constexpr size_t kMaxVersionDataSize = 1000U;

uint16_t read_u16_le(const uint8_t *data) {
  return static_cast<uint16_t>(data[0]) |
         static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8U);
}

void discard_prefix(std::vector<uint8_t> &buffer, size_t size) {
  const size_t count = std::min(size, buffer.size());
  buffer.erase(buffer.begin(), buffer.begin() + count);
}

size_t possible_prefix_suffix(const std::vector<uint8_t> &buffer) {
  const size_t maximum = std::min(buffer.size(), kVersionRequest.size() - 1U);
  for (size_t candidate = maximum; candidate > 0U; --candidate) {
    if (std::equal(buffer.end() - candidate, buffer.end(), kVersionRequest.begin())) {
      return candidate;
    }
  }
  return 0U;
}

bool is_h7_hardware_type(uint8_t hardware_type) {
  // board_declarations.h: RED_PANDA, RED_PANDA_V2, TRES, CUATRO.
  return hardware_type >= 7U && hardware_type <= 10U;
}

}  // namespace

uint8_t version_crc8(const uint8_t *data, size_t size) {
  uint8_t crc = 0xffU;
  for (size_t index = size; index > 0U; --index) {
    crc ^= data[index - 1U];
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      crc = (crc & 0x80U) != 0U
              ? static_cast<uint8_t>((crc << 1U) ^ 0xd5U)
              : static_cast<uint8_t>(crc << 1U);
    }
  }
  return crc;
}

std::optional<VersionInfo> VersionStreamDecoder::feed(const uint8_t *data, size_t size) {
  if (data != nullptr && size != 0U) {
    buffer_.insert(buffer_.end(), data, data + size);
  } else if (size != 0U) {
    return std::nullopt;
  }

  while (true) {
    const auto prefix = std::search(buffer_.begin(), buffer_.end(),
                                    kVersionRequest.begin(), kVersionRequest.end());
    if (prefix == buffer_.end()) {
      const size_t keep = possible_prefix_suffix(buffer_);
      discard_prefix(buffer_, buffer_.size() - keep);
      return std::nullopt;
    }
    discard_prefix(buffer_, static_cast<size_t>(prefix - buffer_.begin()));

    const size_t data_offset = kVersionRequest.size() + kLengthSize;
    if (buffer_.size() < data_offset) {
      return std::nullopt;
    }
    const uint16_t data_length = read_u16_le(&buffer_[kVersionRequest.size()]);
    if (data_length < kMetadataSize || data_length > kMaxVersionDataSize) {
      discard_prefix(buffer_, 1U);
      continue;
    }

    const size_t packet_size = data_offset + data_length + 1U;
    if (buffer_.size() < packet_size) {
      return std::nullopt;
    }
    if (version_crc8(buffer_.data(), packet_size - 1U) != buffer_[packet_size - 1U]) {
      discard_prefix(buffer_, 1U);
      continue;
    }

    VersionInfo info;
    std::copy_n(buffer_.begin() + data_offset, info.uid.size(), info.uid.begin());
    info.hardware_type = buffer_[data_offset + kVersionUidSize];
    info.pid = buffer_[data_offset + kVersionUidSize + 1U];
    info.protocol_version = buffer_[data_offset + kVersionUidSize + 2U];
    discard_prefix(buffer_, packet_size);
    return info;
  }
}

ProtocolSelection select_protocol(const VersionInfo &info) {
  if (info.protocol_version == static_cast<uint8_t>(ProtocolSelection::V2)) {
    return ProtocolSelection::V2;
  }
  if (info.protocol_version == static_cast<uint8_t>(ProtocolSelection::V3) &&
      info.pid == kApplicationPid && is_h7_hardware_type(info.hardware_type)) {
    return ProtocolSelection::V3;
  }
  return ProtocolSelection::Unsupported;
}

}  // namespace panda::spi
