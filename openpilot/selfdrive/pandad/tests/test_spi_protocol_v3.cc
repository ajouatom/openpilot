#define CATCH_CONFIG_MAIN

#include <algorithm>
#include <array>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include "catch2/catch.hpp"
#include "selfdrive/pandad/spi_protocol_v3.h"

using namespace panda::spi_v3;

namespace {

Frame request(uint32_t session, uint32_t sequence, std::vector<uint8_t> payload = {},
              uint16_t max_response_length = 0U) {
  Frame frame;
  frame.type = FrameType::Request;
  frame.flags = 0U;
  frame.status = Status::Ok;
  frame.session_id = session;
  frame.sequence = sequence;
  frame.endpoint = 3U;
  frame.max_response_length = max_response_length;
  frame.payload = std::move(payload);
  return frame;
}

Frame response_for(const Frame &request_frame, Status status, std::vector<uint8_t> payload = {}) {
  Frame frame;
  frame.type = FrameType::Response;
  frame.flags = 0U;
  frame.status = status;
  frame.session_id = request_frame.session_id;
  frame.sequence = request_frame.sequence;
  frame.endpoint = request_frame.endpoint;
  frame.max_response_length = 0U;
  frame.payload = std::move(payload);
  return frame;
}

void append(std::vector<uint8_t> &destination, const std::vector<uint8_t> &source) {
  destination.insert(destination.end(), source.begin(), source.end());
}

void write_u32_le(std::vector<uint8_t> &data, size_t offset, uint32_t value) {
  data[offset] = static_cast<uint8_t>(value);
  data[offset + 1U] = static_cast<uint8_t>(value >> 8U);
  data[offset + 2U] = static_cast<uint8_t>(value >> 16U);
  data[offset + 3U] = static_cast<uint8_t>(value >> 24U);
}

}  // namespace

TEST_CASE("SPI v3 uses standard CRC32C") {
  const std::array<uint8_t, 9> check = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  REQUIRE(crc32c(check.data(), check.size()) == 0xe3069283U);
  REQUIRE(crc32c(nullptr, 0U) == 0U);
}

TEST_CASE("SPI v3 CAN writes use a bounded host chunk without shrinking other endpoints") {
  constexpr size_t default_chunk_size = 2048U - 0x40U;
  REQUIRE(kCanWriteMaxPayloadSize == 512U);
  REQUIRE(bulk_write_chunk_size(3U, default_chunk_size) == kCanWriteMaxPayloadSize);
  REQUIRE(bulk_write_chunk_size(2U, default_chunk_size) == default_chunk_size);
  REQUIRE(bulk_write_chunk_size(0x81U, default_chunk_size) == default_chunk_size);
}

TEST_CASE("SPI v3 request and response round trip") {
  const Frame tx_request = request(0x12345678U, 0x89abcdefU, {0x00U, 0xcdU, 0xffU}, 512U);
  const Frame tx_response = response_for(tx_request, Status::BadEndpoint, {0x42U});
  const auto encoded_request = encode_frame(tx_request);
  const auto encoded_response = encode_frame(tx_response);
  const std::vector<uint8_t> expected_request = {
    0x5aU, 0xc3U, 0x69U, 0x96U, 0x03U, 0x01U, 0x00U, 0x00U,
    0x78U, 0x56U, 0x34U, 0x12U, 0xefU, 0xcdU, 0xabU, 0x89U,
    0x03U, 0x00U, 0x03U, 0x00U, 0x00U, 0x02U, 0x00U, 0x00U,
    0x83U, 0x1aU, 0xd3U, 0x18U, 0x00U, 0xcdU, 0xffU, 0xd2U,
    0x3fU, 0xabU, 0xd2U,
  };

  REQUIRE(encoded_request.size() == kHeaderSize + tx_request.payload.size() + kTrailerSize);
  REQUIRE(encoded_request == expected_request);
  REQUIRE(std::equal(kMagic.begin(), kMagic.end(), encoded_request.begin()));
  REQUIRE(encoded_request[4U] == kVersion);
  REQUIRE(encoded_request[5U] == static_cast<uint8_t>(FrameType::Request));
  REQUIRE(encoded_request[7U] == static_cast<uint8_t>(Status::Ok));
  REQUIRE(encoded_request[8U] == 0x78U);
  REQUIRE(encoded_request[11U] == 0x12U);
  REQUIRE(encoded_request[12U] == 0xefU);
  REQUIRE(encoded_request[15U] == 0x89U);
  REQUIRE(encoded_request[16U] == 3U);
  REQUIRE(encoded_request[18U] == tx_request.payload.size());
  REQUIRE(encoded_request[20U] == 0x00U);
  REQUIRE(encoded_request[21U] == 0x02U);

  StreamDecoder decoder;
  auto decoded = decoder.feed(encoded_request);
  REQUIRE(decoded == std::vector<Frame>{tx_request});
  decoded = decoder.feed(encoded_response);
  REQUIRE(decoded == std::vector<Frame>{tx_response});
  REQUIRE(decoder.buffered_bytes() == 0U);
}

TEST_CASE("SPI v3 decoder tolerates filler, misalignment, and partial chunks") {
  const Frame first = request(11U, 20U, {0xcdU, 0xcdU, 0x01U});
  Frame second = response_for(first, Status::Ok, {kMagic[0], kMagic[1], kMagic[2], kMagic[3], 0xcdU});
  second.sequence = 21U;

  std::vector<uint8_t> stream(37U, 0xcdU);
  stream.push_back(kMagic[0]);
  stream.push_back(kMagic[1]);
  stream.push_back(0x00U);  // false, partially matching magic
  append(stream, encode_frame(first));
  stream.insert(stream.end(), {0x12U, 0x34U, 0xcdU});
  append(stream, encode_frame(second));
  stream.insert(stream.end(), 19U, 0xcdU);

  StreamDecoder decoder;
  std::vector<Frame> decoded;
  size_t offset = 0U;
  size_t chunk_size = 1U;
  while (offset < stream.size()) {
    const size_t count = std::min(chunk_size, stream.size() - offset);
    auto chunk_frames = decoder.feed(&stream[offset], count);
    decoded.insert(decoded.end(), chunk_frames.begin(), chunk_frames.end());
    offset += count;
    chunk_size = chunk_size == 11U ? 1U : chunk_size + 1U;
  }

  REQUIRE((decoded == std::vector<Frame>{first, second}));
  REQUIRE(decoder.stats().discarded_bytes >= 61U);
  REQUIRE(decoder.stats().bad_header_crc == 0U);
  REQUIRE(decoder.stats().bad_frame_crc == 0U);
  REQUIRE(decoder.buffered_bytes() == 0U);
}

TEST_CASE("SPI v3 decoder resynchronizes after corrupt frames") {
  const Frame corrupt_header_frame = request(1U, 1U, {0x10U});
  const Frame corrupt_payload_frame = request(1U, 2U, {0x20U, 0x21U, 0x22U});
  const Frame valid_frame = request(1U, 3U, {0x30U});

  auto bad_header = encode_frame(corrupt_header_frame);
  bad_header[8] ^= 0x80U;
  auto bad_payload = encode_frame(corrupt_payload_frame);
  bad_payload[kHeaderSize + 1U] ^= 0x01U;

  std::vector<uint8_t> stream;
  append(stream, bad_header);
  append(stream, bad_payload);
  append(stream, encode_frame(valid_frame));

  StreamDecoder decoder;
  const auto decoded = decoder.feed(stream);
  REQUIRE(decoded == std::vector<Frame>{valid_frame});
  REQUIRE(decoder.stats().bad_header_crc == 1U);
  REQUIRE(decoder.stats().bad_frame_crc == 1U);
  REQUIRE(decoder.buffered_bytes() == 0U);
}

TEST_CASE("SPI v3 decoder preserves split magic") {
  const Frame frame = request(0x11111111U, 7U, {0x01U, 0x02U});
  const auto encoded = encode_frame(frame);
  StreamDecoder decoder;

  REQUIRE(decoder.feed(encoded.data(), 1U).empty());
  REQUIRE(decoder.buffered_bytes() == 1U);
  REQUIRE(decoder.feed(encoded.data() + 1U, 2U).empty());
  REQUIRE(decoder.buffered_bytes() == 3U);
  REQUIRE(decoder.feed(encoded.data() + 3U, encoded.size() - 4U).empty());
  REQUIRE(decoder.buffered_bytes() == encoded.size() - 1U);
  REQUIRE(decoder.feed(encoded.data() + encoded.size() - 1U, 1U) == std::vector<Frame>{frame});
}

TEST_CASE("SPI v3 rejects invalid and oversized frames") {
  Frame bad_request = request(1U, 1U);
  bad_request.status = Status::Busy;
  REQUIRE_THROWS_AS(encode_frame(bad_request), std::invalid_argument);

  Frame oversized = request(1U, 2U, std::vector<uint8_t>(kMaxPayloadSize + 1U));
  REQUIRE_THROWS_AS(encode_frame(oversized), std::invalid_argument);

  Frame oversized_response = request(1U, 3U);
  oversized_response.max_response_length = static_cast<uint16_t>(kMaxPayloadSize + 1U);
  REQUIRE_THROWS_AS(encode_frame(oversized_response), std::invalid_argument);

  Frame wide_endpoint = request(1U, 4U);
  wide_endpoint.endpoint = 0x100U;
  REQUIRE_THROWS_AS(encode_frame(wide_endpoint), std::invalid_argument);

  Frame response_with_request_limit = response_for(request(1U, 5U), Status::Ok);
  response_with_request_limit.max_response_length = 1U;
  REQUIRE_THROWS_AS(encode_frame(response_with_request_limit), std::invalid_argument);
  REQUIRE_THROWS_AS(StreamDecoder(kMaxPayloadSize + 1U), std::invalid_argument);
}

TEST_CASE("SPI v3 decoder rejects a corrupt max response length and resynchronizes") {
  auto malformed = encode_frame(request(9U, 1U, {0x01U}));
  const uint16_t invalid_limit = static_cast<uint16_t>(kMaxPayloadSize + 1U);
  malformed[20U] = static_cast<uint8_t>(invalid_limit & 0xffU);
  malformed[21U] = static_cast<uint8_t>(invalid_limit >> 8U);
  write_u32_le(malformed, 24U, crc32c(malformed.data(), 24U));

  const Frame valid = request(9U, 2U, {0x02U});
  append(malformed, encode_frame(valid));

  StreamDecoder decoder;
  REQUIRE(decoder.feed(malformed) == std::vector<Frame>{valid});
  REQUIRE(decoder.stats().bad_length == 1U);
}

TEST_CASE("SPI v3 duplicate requests replay without re-execution") {
  const Frame original = request(0x10203040U, 55U, {0xaaU, 0xbbU}, 1U);
  const Frame terminal_response = response_for(original, Status::Ok, {0x99U});
  RequestReplayCache cache;

  REQUIRE(cache.classify(original) == RequestDisposition::New);
  cache.remember(original, terminal_response);
  REQUIRE(cache.classify(original) == RequestDisposition::Replay);
  REQUIRE(cache.encoded_response() == encode_frame(terminal_response));

  Frame conflict = original;
  conflict.payload[0] ^= 0x01U;
  REQUIRE(cache.classify(conflict) == RequestDisposition::Conflict);

  Frame next_sequence = original;
  ++next_sequence.sequence;
  REQUIRE(cache.classify(next_sequence) == RequestDisposition::New);

  Frame next_session = original;
  ++next_session.session_id;
  REQUIRE(cache.classify(next_session) == RequestDisposition::New);

  REQUIRE_THROWS_AS(cache.remember(original, response_for(original, Status::Busy)), std::invalid_argument);
  REQUIRE_THROWS_AS(cache.remember(original, response_for(original, Status::Ok, {0x01U, 0x02U})), std::invalid_argument);
  cache.clear();
  REQUIRE(cache.classify(original) == RequestDisposition::New);
}
