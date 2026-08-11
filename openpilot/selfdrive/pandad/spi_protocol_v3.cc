#include "selfdrive/pandad/spi_protocol_v3.h"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

namespace panda::spi_v3 {
namespace {

constexpr size_t kVersionOffset = 4U;
constexpr size_t kTypeOffset = 5U;
constexpr size_t kFlagsOffset = 6U;
constexpr size_t kStatusOffset = 7U;
constexpr size_t kSessionOffset = 8U;
constexpr size_t kSequenceOffset = 12U;
constexpr size_t kEndpointOffset = 16U;
constexpr size_t kPayloadLengthOffset = 18U;
constexpr size_t kMaxResponseLengthOffset = 20U;
constexpr size_t kReserved16Offset = 22U;
constexpr size_t kHeaderCrcOffset = 24U;

void append_u16_le(std::vector<uint8_t> &data, uint16_t value) {
  data.push_back(static_cast<uint8_t>(value));
  data.push_back(static_cast<uint8_t>(value >> 8U));
}

void append_u32_le(std::vector<uint8_t> &data, uint32_t value) {
  data.push_back(static_cast<uint8_t>(value));
  data.push_back(static_cast<uint8_t>(value >> 8U));
  data.push_back(static_cast<uint8_t>(value >> 16U));
  data.push_back(static_cast<uint8_t>(value >> 24U));
}

uint16_t read_u16_le(const uint8_t *data) {
  return static_cast<uint16_t>(data[0]) |
         static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8U);
}

uint32_t read_u32_le(const uint8_t *data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8U) |
         (static_cast<uint32_t>(data[2]) << 16U) |
         (static_cast<uint32_t>(data[3]) << 24U);
}

bool valid_type(uint8_t raw_type) {
  return raw_type == static_cast<uint8_t>(FrameType::Request) ||
         raw_type == static_cast<uint8_t>(FrameType::Response);
}

bool valid_status(uint8_t raw_status) {
  return raw_status <= static_cast<uint8_t>(Status::InternalError);
}

bool valid_fields(FrameType type, uint8_t flags, Status status, uint16_t endpoint,
                  uint16_t max_response_length) {
  // No flags are assigned in v3 yet. Reserving the byte now lets a future
  // version extend behavior without changing the fixed header shape.
  if (flags != 0U || endpoint > std::numeric_limits<uint8_t>::max() ||
      max_response_length > kMaxPayloadSize) {
    return false;
  }
  return type == FrameType::Request ? status == Status::Ok : max_response_length == 0U;
}

}  // namespace

bool Frame::operator==(const Frame &other) const {
  return type == other.type && flags == other.flags && status == other.status &&
         session_id == other.session_id && sequence == other.sequence &&
         endpoint == other.endpoint && max_response_length == other.max_response_length &&
         payload == other.payload;
}

uint32_t crc32c(const uint8_t *data, size_t size) {
  if (data == nullptr && size != 0U) {
    throw std::invalid_argument("crc32c received null data with non-zero size");
  }

  uint32_t crc = 0xffffffffU;
  for (size_t i = 0U; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      const uint32_t mask = 0U - (crc & 1U);
      crc = (crc >> 1U) ^ (0x82f63b78U & mask);
    }
  }
  return ~crc;
}

uint32_t crc32c(const std::vector<uint8_t> &data) {
  return crc32c(data.data(), data.size());
}

std::vector<uint8_t> encode_frame(const Frame &frame) {
  if (!valid_type(static_cast<uint8_t>(frame.type)) ||
      !valid_status(static_cast<uint8_t>(frame.status)) ||
      !valid_fields(frame.type, frame.flags, frame.status, frame.endpoint,
                    frame.max_response_length)) {
    throw std::invalid_argument("invalid SPI v3 frame fields");
  }
  if (frame.payload.size() > kMaxPayloadSize ||
      frame.payload.size() > std::numeric_limits<uint16_t>::max()) {
    throw std::invalid_argument("SPI v3 payload is too large");
  }

  std::vector<uint8_t> encoded;
  encoded.reserve(kHeaderSize + frame.payload.size() + kTrailerSize);
  encoded.insert(encoded.end(), kMagic.begin(), kMagic.end());
  encoded.push_back(kVersion);
  encoded.push_back(static_cast<uint8_t>(frame.type));
  encoded.push_back(frame.flags);
  encoded.push_back(static_cast<uint8_t>(frame.status));
  append_u32_le(encoded, frame.session_id);
  append_u32_le(encoded, frame.sequence);
  encoded.push_back(static_cast<uint8_t>(frame.endpoint));
  encoded.push_back(0U);
  append_u16_le(encoded, static_cast<uint16_t>(frame.payload.size()));
  append_u16_le(encoded, frame.max_response_length);
  append_u16_le(encoded, 0U);
  append_u32_le(encoded, crc32c(encoded));
  encoded.insert(encoded.end(), frame.payload.begin(), frame.payload.end());
  append_u32_le(encoded, crc32c(encoded));
  return encoded;
}

StreamDecoder::StreamDecoder(size_t max_payload_size) : max_payload_size_(max_payload_size) {
  if (max_payload_size_ > kMaxPayloadSize) {
    throw std::invalid_argument("decoder payload limit exceeds the wire buffer");
  }
}

void StreamDecoder::discard_prefix(size_t size) {
  const size_t discard_size = std::min(size, buffer_.size());
  stats_.discarded_bytes += discard_size;
  buffer_.erase(buffer_.begin(), buffer_.begin() + discard_size);
}

std::vector<Frame> StreamDecoder::feed(const uint8_t *data, size_t size) {
  if (data == nullptr && size != 0U) {
    throw std::invalid_argument("decoder received null data with non-zero size");
  }
  if (size != 0U) {
    buffer_.insert(buffer_.end(), data, data + size);
  }

  std::vector<Frame> frames;
  while (true) {
    const auto magic = std::search(buffer_.begin(), buffer_.end(), kMagic.begin(), kMagic.end());
    if (magic == buffer_.end()) {
      // Retain the longest suffix that may be the start of split magic.
      size_t keep = 0U;
      const size_t max_keep = std::min(buffer_.size(), kMagic.size() - 1U);
      for (size_t candidate = max_keep; candidate > 0U; --candidate) {
        if (std::equal(buffer_.end() - candidate, buffer_.end(), kMagic.begin())) {
          keep = candidate;
          break;
        }
      }
      discard_prefix(buffer_.size() - keep);
      break;
    }

    discard_prefix(static_cast<size_t>(magic - buffer_.begin()));
    if (buffer_.size() < kHeaderSize) {
      break;
    }

    const uint32_t received_header_crc = read_u32_le(&buffer_[kHeaderCrcOffset]);
    if (crc32c(buffer_.data(), kHeaderCrcOffset) != received_header_crc) {
      ++stats_.bad_header_crc;
      discard_prefix(1U);
      continue;
    }

    if (buffer_[kVersionOffset] != kVersion) {
      ++stats_.bad_version;
      discard_prefix(1U);
      continue;
    }

    const uint16_t payload_size = read_u16_le(&buffer_[kPayloadLengthOffset]);
    const uint16_t max_response_length = read_u16_le(&buffer_[kMaxResponseLengthOffset]);
    if (payload_size > max_payload_size_ || max_response_length > max_payload_size_) {
      ++stats_.bad_length;
      discard_prefix(1U);
      continue;
    }

    const uint8_t raw_type = buffer_[kTypeOffset];
    const uint8_t raw_status = buffer_[kStatusOffset];
    if (!valid_type(raw_type) || !valid_status(raw_status)) {
      ++stats_.bad_fields;
      discard_prefix(1U);
      continue;
    }
    const auto type = static_cast<FrameType>(raw_type);
    const auto status = static_cast<Status>(raw_status);
    const uint16_t endpoint = buffer_[kEndpointOffset];
    if (buffer_[kEndpointOffset + 1U] != 0U || read_u16_le(&buffer_[kReserved16Offset]) != 0U ||
        !valid_fields(type, buffer_[kFlagsOffset], status, endpoint, max_response_length)) {
      ++stats_.bad_fields;
      discard_prefix(1U);
      continue;
    }

    const size_t frame_size = kHeaderSize + payload_size + kTrailerSize;
    if (buffer_.size() < frame_size) {
      break;
    }

    const uint32_t received_frame_crc = read_u32_le(&buffer_[frame_size - kTrailerSize]);
    if (crc32c(buffer_.data(), frame_size - kTrailerSize) != received_frame_crc) {
      ++stats_.bad_frame_crc;
      discard_prefix(1U);
      continue;
    }

    Frame frame;
    frame.type = type;
    frame.flags = buffer_[kFlagsOffset];
    frame.status = status;
    frame.session_id = read_u32_le(&buffer_[kSessionOffset]);
    frame.sequence = read_u32_le(&buffer_[kSequenceOffset]);
    frame.endpoint = endpoint;
    frame.max_response_length = max_response_length;
    frame.payload.assign(buffer_.begin() + kHeaderSize,
                         buffer_.begin() + kHeaderSize + payload_size);
    frames.push_back(std::move(frame));
    buffer_.erase(buffer_.begin(), buffer_.begin() + frame_size);
  }

  return frames;
}

std::vector<Frame> StreamDecoder::feed(const std::vector<uint8_t> &data) {
  return feed(data.data(), data.size());
}

void StreamDecoder::reset() {
  buffer_.clear();
  stats_ = DecodeStats{};
}

RequestDisposition RequestReplayCache::classify(const Frame &request) const {
  if (request.type != FrameType::Request) {
    throw std::invalid_argument("replay cache requires a valid request frame");
  }
  const std::vector<uint8_t> encoded_request = encode_frame(request);
  if (!valid_ || session_id_ != request.session_id || sequence_ != request.sequence) {
    return RequestDisposition::New;
  }
  return encoded_request_ == encoded_request
           ? RequestDisposition::Replay
           : RequestDisposition::Conflict;
}

void RequestReplayCache::remember(const Frame &request, const Frame &response) {
  if (request.type != FrameType::Request) {
    throw std::invalid_argument("replay cache requires a valid request frame");
  }
  const std::vector<uint8_t> encoded_request = encode_frame(request);
  if (response.type != FrameType::Response || response.session_id != request.session_id ||
      response.sequence != request.sequence || response.endpoint != request.endpoint ||
      response.status == Status::Busy || response.payload.size() > request.max_response_length) {
    throw std::invalid_argument("response cannot be cached for this request");
  }

  session_id_ = request.session_id;
  sequence_ = request.sequence;
  encoded_request_ = encoded_request;
  encoded_response_ = encode_frame(response);
  valid_ = true;
}

void RequestReplayCache::clear() {
  valid_ = false;
  session_id_ = 0U;
  sequence_ = 0U;
  encoded_request_.clear();
  encoded_response_.clear();
}

}  // namespace panda::spi_v3
