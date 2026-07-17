#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

std::string encode_carrot_state_compact_frame(const char *service, size_t service_size,
                                              const char *data, size_t size, uint16_t sequence);
