// Exact IEEE exponent checks. No floating-point arithmetic or fast-math.
#include <stddef.h>
#include <stdint.h>
#if defined(__aarch64__)
#include <arm_neon.h>
#endif

int jetlink_finite16(const uint16_t *p, size_t n) {
  size_t i = 0;
#if defined(__aarch64__)
  uint16x8_t bad = vdupq_n_u16(0), mask = vdupq_n_u16(0x7c00);
  for (; i + 8 <= n; i += 8)
    bad = vorrq_u16(bad, vceqq_u16(vandq_u16(vld1q_u16(p + i), mask), mask));
  if (vmaxvq_u16(bad)) return 0;
#endif
  for (; i < n; ++i) if ((p[i] & 0x7c00) == 0x7c00) return 0;
  return 1;
}

int jetlink_finite32(const uint32_t *p, size_t n) {
  size_t i = 0;
#if defined(__aarch64__)
  uint32x4_t bad = vdupq_n_u32(0), mask = vdupq_n_u32(0x7f800000);
  for (; i + 4 <= n; i += 4)
    bad = vorrq_u32(bad, vceqq_u32(vandq_u32(vld1q_u32(p + i), mask), mask));
  if (vmaxvq_u32(bad)) return 0;
#endif
  for (; i < n; ++i) if ((p[i] & 0x7f800000) == 0x7f800000) return 0;
  return 1;
}
