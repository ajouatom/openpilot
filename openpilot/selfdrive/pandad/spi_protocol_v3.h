#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace panda::spi_v3 {

// Wire format (all multi-byte integers are little-endian):
//
//   0   magic[4]
//   4   version
//   5   frame type
//   6   flags
//   7   status
//   8   session id       u32
//   12  sequence         u32
//   16  endpoint         u8
//   17  reserved         u8, must be zero
//   18  payload length   u16
//   20  max response len u16 (request only; zero in a response)
//   22  reserved         u16, must be zero
//   24  header CRC32C    u32, over bytes [0, 24)
//   28  payload
//   ... frame CRC32C     u32, over the complete header and payload
//
// The four-byte magic makes the protocol self-framing. The independent header
// CRC prevents a corrupt length from making a stream decoder wait indefinitely,
// while the final CRC protects the payload. SPI idle/filler bytes (currently
// 0xcd), split reads, and arbitrary garbage can therefore be skipped safely.

inline constexpr std::array<uint8_t, 4> kMagic = {0x5aU, 0xc3U, 0x69U, 0x96U};
inline constexpr uint8_t kVersion = 3U;
inline constexpr size_t kHeaderSize = 28U;
inline constexpr size_t kTrailerSize = 4U;
inline constexpr size_t kWireBufferSize = 2048U;
inline constexpr size_t kMaxPayloadSize = kWireBufferSize - kHeaderSize - kTrailerSize;
// Bound one side-effecting CAN write so queue reservation and safety-hook work
// cannot monopolize Panda's main loop. Other endpoints retain the full wire
// payload capacity.
inline constexpr size_t kCanWriteMaxPayloadSize = 512U;

inline constexpr size_t bulk_write_chunk_size(uint8_t endpoint, size_t default_chunk_size) {
  return (endpoint == 3U && default_chunk_size > kCanWriteMaxPayloadSize) ?
           kCanWriteMaxPayloadSize : default_chunk_size;
}

enum class FrameType : uint8_t {
  Request = 1U,
  Response = 2U,
};

enum class Status : uint8_t {
  Ok = 0U,
  BadFrame = 1U,
  UnsupportedVersion = 2U,
  BadLength = 3U,
  BadEndpoint = 4U,
  Busy = 5U,
  SequenceConflict = 6U,
  InternalError = 7U,
};

struct Frame {
  FrameType type = FrameType::Request;
  uint8_t flags = 0U;
  Status status = Status::Ok;
  uint32_t session_id = 0U;
  uint32_t sequence = 0U;
  uint16_t endpoint = 0U;
  uint16_t max_response_length = 0U;
  std::vector<uint8_t> payload;

  bool operator==(const Frame &other) const;
};

// Castagnoli CRC-32C, matching the conventional init/final xor values. The
// standard "123456789" check vector is 0xe3069283.
uint32_t crc32c(const uint8_t *data, size_t size);
uint32_t crc32c(const std::vector<uint8_t> &data);

// Throws std::invalid_argument for a semantically invalid frame or a payload
// too large for Panda's 2048-byte SPI DMA buffer.
std::vector<uint8_t> encode_frame(const Frame &frame);

struct DecodeStats {
  size_t discarded_bytes = 0U;
  size_t bad_header_crc = 0U;
  size_t bad_frame_crc = 0U;
  size_t bad_version = 0U;
  size_t bad_length = 0U;
  size_t bad_fields = 0U;
};

class StreamDecoder {
public:
  explicit StreamDecoder(size_t max_payload_size = kMaxPayloadSize);

  // Each call may return zero or more complete frames. Incomplete magic,
  // headers, and frames remain buffered for the next call.
  std::vector<Frame> feed(const uint8_t *data, size_t size);
  std::vector<Frame> feed(const std::vector<uint8_t> &data);

  const DecodeStats &stats() const { return stats_; }
  size_t buffered_bytes() const { return buffer_.size(); }
  void reset();

private:
  void discard_prefix(size_t size);

  size_t max_payload_size_;
  std::vector<uint8_t> buffer_;
  DecodeStats stats_;
};

enum class RequestDisposition {
  New,
  Replay,
  Conflict,
};

// Prototype for the device-side duplicate rule used by the stop-and-wait host
// transport. The host gives every new logical request a new sequence and uses
// the exact same encoded request (same session and sequence) for every retry.
//
// Once a request reaches a terminal response, the device remembers that request
// and its encoded response. An identical duplicate is not executed again: the
// cached response is replayed byte-for-byte. The full encoded request is
// compared, rather than a hash that could collide. Reusing the same
// (session,sequence) for different request contents is a SequenceConflict and
// has no side effect.
// Busy is non-terminal and must not be remembered. A new host connection uses a
// new random session id, preventing a restarted sequence counter from colliding
// with the previous connection's final request.
class RequestReplayCache {
public:
  RequestDisposition classify(const Frame &request) const;
  void remember(const Frame &request, const Frame &response);
  const std::vector<uint8_t> &encoded_response() const { return encoded_response_; }
  void clear();

private:
  bool valid_ = false;
  uint32_t session_id_ = 0U;
  uint32_t sequence_ = 0U;
  std::vector<uint8_t> encoded_request_;
  std::vector<uint8_t> encoded_response_;
};

}  // namespace panda::spi_v3
