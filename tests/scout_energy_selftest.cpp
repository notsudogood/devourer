/* Headless guard for the Jaguar3 scout energy read's register arithmetic
 * (src/jaguar3/ScoutEnergyMath.h): the vendor OFDM false-alarm sum and the
 * full-dword composition of the BB counter reset. Pure math; no hardware.
 * Guards the copy BOTH paths actually run — GetRxEnergyScout and the full
 * GetRxEnergy share fa_ofdm_sum — rather than a shadow of it. The failure
 * this exists for is silent: a dropped bit in compose_reset holds the OFDM
 * counters in permanent reset, which reads on air as a perfectly quiet
 * channel. */
#include "jaguar3/ScoutEnergyMath.h"

#include <cstdio>
#include <cstdlib>

static int fails = 0;
#define CHECK(c)                                                               \
  do {                                                                         \
    if (!(c)) {                                                                \
      std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #c);         \
      ++fails;                                                                 \
    }                                                                          \
  } while (0)

int main() {
  using namespace devourer::jgr3;
  // Vendor sum: parity(2d04 hi) + rate-illegal(2d08 lo) + crc8(2d08 hi) +
  // mcs(2d10 lo) + fast-fsync(2d20 lo) + sb-search(2d20 hi) + mcs-vht(2d10 hi)
  // + crc8-vhta(2d0c lo). Same formula as GetRxEnergy.
  CHECK(fa_ofdm_sum(0x00010000, 0x00030002, 0x00070004, 0x00060005,
                    0x00000008) == 1 + 2 + 3 + 4 + 5 + 6 + 7 + 8);
  CHECK(fa_ofdm_sum(0, 0, 0, 0, 0) == 0);
  CHECK(fa_ofdm_sum(0xffff0000, 0xffffffff, 0xffffffff, 0xffffffff,
                    0x0000ffff) == 8u * 0xffff);
  // Reset dwords: 0x1d2c[31] off, 0x1eb4[25] on, 0x1eb4[25] off, 0x1d2c[31] on,
  // every other bit of the shadow preserved.
  //
  // TWO vectors, opposite polarity on the bits that matter, so a mutant that
  // forgets to actually flip a bit (e.g. `d1eb4_off = shadow_1eb4;` with no
  // mask) cannot pass both. Vector 1 starts with 0x1d2c[31]=1 and
  // 0x1eb4[25]=0 — it pins d1d2c_off (bit cleared) and d1eb4_on (bit set),
  // but a mutant copying the shadow verbatim into d1eb4_off/d1d2c_on would
  // ALSO pass vector 1 unnoticed, because those two bits already sit at
  // their target value in this shadow (a real bug caught in review: on
  // hardware that mutant latches 0x1eb4[25]=1 forever after the first call,
  // holding the OFDM FA/CCA counters in permanent reset — fa_ofdm/cca_ofdm
  // read 0 on every call from the second one on, silently). Vector 2 starts
  // with the opposite polarity (0x1d2c[31]=0, 0x1eb4[25]=1) so it pins
  // d1eb4_off and d1d2c_on instead — the mutant fails vector 2. Together the
  // two vectors pin all four output dwords against a bit-dropped mutant.
  const ResetDwords r1 = compose_reset(0x80001234u, 0x00005678u);
  CHECK(r1.d1d2c_off == 0x00001234u);
  CHECK(r1.d1eb4_on == (0x00005678u | (1u << 25)));
  CHECK(r1.d1eb4_off == 0x00005678u);
  CHECK(r1.d1d2c_on == 0x80001234u);
  const ResetDwords r2 = compose_reset(0x00001234u, 0x02005678u);
  CHECK(r2.d1d2c_off == 0x00001234u);
  CHECK(r2.d1eb4_on == 0x02005678u);
  CHECK(r2.d1eb4_off == 0x00005678u);
  CHECK(r2.d1d2c_on == 0x80001234u);
  if (fails) return EXIT_FAILURE;
  std::puts("scout_energy_selftest: ok");
  return 0;
}
