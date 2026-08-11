#define CATCH_CONFIG_MAIN

#include <algorithm>
#include <cstdint>
#include <vector>

#include "catch2/catch.hpp"
#include "selfdrive/pandad/spi_version.h"

using namespace panda::spi;

namespace {

std::vector<uint8_t> version_packet(uint8_t hardware_type, uint8_t pid,
                                    uint8_t protocol_version) {
  std::vector<uint8_t> packet(kVersionRequest.begin(), kVersionRequest.end());
  constexpr uint16_t data_length = 15U;
  packet.push_back(static_cast<uint8_t>(data_length));
  packet.push_back(static_cast<uint8_t>(data_length >> 8U));
  for (uint8_t i = 0U; i < kVersionUidSize; ++i) {
    packet.push_back(i);
  }
  packet.push_back(hardware_type);
  packet.push_back(pid);
  packet.push_back(protocol_version);
  packet.push_back(version_crc8(packet.data(), packet.size()));
  return packet;
}

}  // namespace

TEST_CASE("stable SPI VERSION decoder tolerates filler and split prefix") {
  const auto packet = version_packet(10U, kApplicationPid, 3U);
  VersionStreamDecoder decoder;

  const std::vector<uint8_t> first = {0xcdU, 0xcdU, 'V', 'E', 'R'};
  REQUIRE_FALSE(decoder.feed(first.data(), first.size()).has_value());
  REQUIRE_FALSE(decoder.feed(packet.data() + 3U, 4U).has_value());
  const auto info = decoder.feed(packet.data() + 7U, packet.size() - 7U);

  REQUIRE(info.has_value());
  REQUIRE(info->hardware_type == 10U);
  REQUIRE(info->pid == kApplicationPid);
  REQUIRE(info->protocol_version == 3U);
  for (uint8_t i = 0U; i < kVersionUidSize; ++i) {
    REQUIRE(info->uid[i] == i);
  }
}

TEST_CASE("stable SPI VERSION discovery includes request MISO and one-byte poll windows") {
  const auto packet = version_packet(7U, kApplicationPid, 3U);
  VersionStreamDecoder decoder;

  // The response starts in the final four bytes clocked by the VERSION request.
  // The remaining packet crosses more than two eight-byte polling windows.
  const std::array<uint8_t, kVersionRequest.size()> request_miso = {
    0xcdU, 0xcdU, 0xcdU, packet[0], packet[1], packet[2], packet[3],
  };
  REQUIRE_FALSE(decoder.feed(request_miso.data(), request_miso.size()).has_value());

  std::optional<VersionInfo> info;
  for (size_t index = 4U; index < packet.size(); ++index) {
    INFO("one-byte poll index " << index);
    info = decoder.feed(&packet[index], 1U);
    REQUIRE(info.has_value() == (index == packet.size() - 1U));
  }

  REQUIRE(info.has_value());
  REQUIRE(info->hardware_type == 7U);
  REQUIRE(info->pid == kApplicationPid);
  REQUIRE(info->protocol_version == 3U);
}

TEST_CASE("stable SPI VERSION poll budget drains one abandoned maximum v3 response") {
  const auto packet = version_packet(10U, kApplicationPid, 3U);
  REQUIRE(packet.size() < kVersionPollServiceMarginBytes);

  // Model a process dying before it clocks any of a maximum-sized v3 response.
  // The replacement process consumes the first seven bytes while transmitting
  // VERSION, then keeps the same decoder across exact one-byte CS polls.
  std::vector<uint8_t> stale_response(panda::spi_v3::kWireBufferSize, 0xa5U);
  std::copy(panda::spi_v3::kMagic.begin(), panda::spi_v3::kMagic.end(),
            stale_response.begin());
  stale_response[4] = panda::spi_v3::kVersion;

  VersionStreamDecoder decoder;
  REQUIRE_FALSE(decoder.feed(stale_response.data(), kVersionRequest.size()).has_value());

  size_t poll_count = 0U;
  for (size_t index = kVersionRequest.size(); index < stale_response.size(); ++index) {
    ++poll_count;
    REQUIRE_FALSE(decoder.feed(&stale_response[index], 1U).has_value());
  }

  const uint8_t filler = 0xcdU;
  const size_t service_filler = kVersionPollServiceMarginBytes - packet.size();
  for (size_t index = 0U; index < service_filler; ++index) {
    ++poll_count;
    REQUIRE_FALSE(decoder.feed(&filler, 1U).has_value());
  }

  std::optional<VersionInfo> info;
  for (size_t index = 0U; index < packet.size(); ++index) {
    ++poll_count;
    info = decoder.feed(&packet[index], 1U);
    REQUIRE(info.has_value() == (index == packet.size() - 1U));
  }

  REQUIRE(info.has_value());
  REQUIRE(info->hardware_type == 10U);
  REQUIRE(info->protocol_version == 3U);
  REQUIRE(poll_count <= kVersionMaxPollBytes);
  REQUIRE(kVersionMaxPollBytes ==
          panda::spi_v3::kWireBufferSize + kVersionPollServiceMarginBytes);
}

TEST_CASE("stable SPI VERSION decoder resynchronizes after bad CRC and length") {
  auto bad_crc = version_packet(7U, kApplicationPid, 3U);
  bad_crc.back() ^= 1U;
  std::vector<uint8_t> stream = {'V', 'E', 'R', 'S', 'I', 'O', 'N', 0xffU, 0xffU};
  stream.insert(stream.end(), bad_crc.begin(), bad_crc.end());
  const auto valid = version_packet(6U, kApplicationPid, 2U);
  stream.insert(stream.end(), valid.begin(), valid.end());

  VersionStreamDecoder decoder;
  const auto info = decoder.feed(stream.data(), stream.size());
  REQUIRE(info.has_value());
  REQUIRE(info->hardware_type == 6U);
  REQUIRE(info->protocol_version == 2U);
}

TEST_CASE("SPI protocol selection keeps bootstub and F4 on v2") {
  VersionInfo info;

  info.hardware_type = 6U;
  info.pid = kApplicationPid;
  info.protocol_version = 2U;
  REQUIRE(select_protocol(info) == ProtocolSelection::V2);

  info.hardware_type = 10U;
  info.pid = kBootstubPid;
  REQUIRE(select_protocol(info) == ProtocolSelection::V2);

  info.protocol_version = 3U;
  REQUIRE(select_protocol(info) == ProtocolSelection::Unsupported);

  info.hardware_type = 6U;
  info.pid = kApplicationPid;
  REQUIRE(select_protocol(info) == ProtocolSelection::Unsupported);

  info.hardware_type = 7U;
  REQUIRE(select_protocol(info) == ProtocolSelection::V3);
}
