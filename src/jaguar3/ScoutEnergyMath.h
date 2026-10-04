/* Pure register arithmetic for RtlJaguar3Device::GetRxEnergyScout: the OFDM
 * false-alarm vendor sum (phydm_fa_cnt_statistics_jgr3) and the full-dword
 * composition of the BB counter reset (phydm_reset_bb_hw_cnt jgr3:
 * 0x1eb4[25] 1->0 wrapped by 0x1d2c[31] rx-clk-gate off/on). Full dwords
 * composed in memory from the two registers' current values (read in the
 * same batched group), so the scout path never pays a masked read-modify-
 * write — which on JGR3 would also hit the double-shift gotcha
 * (jaguar3/CLAUDE.md). Kept standalone so the shipping path and
 * tests/scout_energy_selftest.cpp share one copy, not a shadow of one. */
#ifndef DEVOURER_JAGUAR3_SCOUT_ENERGY_MATH_H
#define DEVOURER_JAGUAR3_SCOUT_ENERGY_MATH_H

#include <cstdint>

namespace devourer {
namespace jgr3 {

/* Vendor OFDM false-alarm sum: parity (0x2d04 hi) + rate-illegal (0x2d08 lo)
 * + crc8 (0x2d08 hi) + mcs (0x2d10 lo) + fast-fsync (0x2d20 lo) + sb-search
 * (0x2d20 hi) + mcs-vht (0x2d10 hi) + crc8-vhta (0x2d0c lo). The full
 * GetRxEnergy read runs the identical formula through here. */
inline uint32_t fa_ofdm_sum(uint32_t r2d04, uint32_t r2d08, uint32_t r2d10,
                            uint32_t r2d20, uint32_t r2d0c) {
  return ((r2d04 >> 16) & 0xffff) + (r2d08 & 0xffff) +
         ((r2d08 >> 16) & 0xffff) + (r2d10 & 0xffff) + (r2d20 & 0xffff) +
         ((r2d20 >> 16) & 0xffff) + ((r2d10 >> 16) & 0xffff) +
         (r2d0c & 0xffff);
}

/* The four full dwords of the counter reset, in write order. */
struct ResetDwords {
  uint32_t d1d2c_off; /* rx clock gate off  (0x1d2c[31] = 0) */
  uint32_t d1eb4_on;  /* counter reset assert (0x1eb4[25] = 1) */
  uint32_t d1eb4_off; /* counter reset release (0x1eb4[25] = 0) */
  uint32_t d1d2c_on;  /* rx clock gate back on (0x1d2c[31] = 1) */
};

/* Compose the reset from the two registers' CURRENT values, so every bit
 * outside [31]/[25] round-trips untouched. Both inputs must come from a
 * read that actually succeeded — composing from a failed read's zero
 * clobbers two live BB registers and leaves the receiver permanently deaf
 * (tests/scout_batch_fail_selftest.cpp pins the caller's bail-out). */
inline ResetDwords compose_reset(uint32_t cur_1d2c, uint32_t cur_1eb4) {
  ResetDwords r;
  r.d1d2c_off = cur_1d2c & ~(1u << 31);
  r.d1eb4_on = cur_1eb4 | (1u << 25);
  r.d1eb4_off = cur_1eb4 & ~(1u << 25);
  r.d1d2c_on = cur_1d2c | (1u << 31);
  return r;
}

} // namespace jgr3
} // namespace devourer

#endif /* DEVOURER_JAGUAR3_SCOUT_ENERGY_MATH_H */
