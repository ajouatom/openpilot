// Exact uint8 -> IEEE binary16 bits; no floating-point arithmetic or scratch.
#include <stddef.h>
#include <stdint.h>

void jetlink_u8_f16(const uint8_t *restrict src, uint16_t *restrict dst,
                   const uint16_t *restrict table, size_t n) {
  for (size_t i = 0; i < n; ++i) dst[i] = table[src[i]];
}
