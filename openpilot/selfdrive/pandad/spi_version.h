#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "selfdrive/pandad/spi_protocol_v3.h"

namespace panda::spi {

inline constexpr std::array<uint8_t, 7> kVersionRequest = {
  'V', 'E', 'R', 'S', 'I', 'O', 'N',
};
inline constexpr size_t kVersionUidSize = 12U;
inline constexpr uint8_t kApplicationPid = 0xccU;
inline constexpr uint8_t kBootstubPid = 0xeeU;
// A new host can inherit a response left in Panda's TX DMA when the previous
// v3 process died. Drain at most one complete v3 wire frame, then allow enough
// additional clocks for firmware service latency and the 25-byte VERSION
// packet. Every clock remains a one-byte CS window and discovery stops on the
// exact byte that validates VERSION's CRC.
inline constexpr size_t kVersionPollServiceMarginBytes = 64U;
inline constexpr size_t kVersionMaxPollBytes =
  panda::spi_v3::kWireBufferSize + kVersionPollServiceMarginBytes;

struct VersionInfo {
  std::array<uint8_t, kVersionUidSize> uid = {};
  uint8_t hardware_type = 0U;
  uint8_t pid = 0U;
  uint8_t protocol_version = 0U;
};

// CRC-8 used only by the stable, legacy-format VERSION discovery packet.
uint8_t version_crc8(const uint8_t *data, size_t size);

// VERSION is deliberately outside both v2 and v3. This decoder accepts filler
// and arbitrary chunk boundaries so protocol selection itself cannot depend on
// Panda ISR response timing.
class VersionStreamDecoder {
public:
  std::optional<VersionInfo> feed(const uint8_t *data, size_t size);
  void reset() { buffer_.clear(); }

private:
  std::vector<uint8_t> buffer_;
};

enum class ProtocolSelection : uint8_t {
  V2 = 2U,
  V3 = 3U,
  Unsupported = 0xffU,
};

// Only a non-bootstub H7 application may select v3. F4 and every bootstub stay
// on v2 even though all variants share the VERSION discovery format.
ProtocolSelection select_protocol(const VersionInfo &info);

}  // namespace panda::spi
