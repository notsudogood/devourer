/* Headless guard for the active idle-noise-floor math helpers
 * (src/NoiseFloorMath.h): the 10-bit sign conversion of the 0x0FA0 debug-port
 * I/Q and the 3*log2 pwdb approximation (Jaguar1); the Jaguar3 NHM absolute
 * floor (nf::nhm_abs_floor_dbm — bucket weighting, the sample/idle-time
 * guards and the frozen-estimate peak-bucket guard); and the NhmReader.h
 * read_nhm/read_nhm_absolute register recipe against a fake JGR3 BB, which
 * pins the th[8..10] masked writes against the double-shift regression
 * (values pre-shifted into the mask read back 0). Pure math; no hardware. */
#include "NhmReader.h"
#include "NoiseFloorMath.h"

#include <cstdio>
#include <map>

using devourer::nf::kNhmAbsThDbm;
using devourer::nf::nhm_abs_floor_dbm;
using devourer::nf::nhm_th_from_dbm;
using devourer::nf::pwdb_conversion;
using devourer::nf::sign_conversion;

static int g_fail = 0;

static void check(const char *what, long got, long want) {
  if (got != want) {
    std::printf("FAIL %s: got %ld want %ld\n", what, got, want);
    ++g_fail;
  }
}

int main() {
  /* 10-bit two's-complement (rxi/rxq packing). */
  check("sign 0x000", sign_conversion(0x000, 10), 0);
  check("sign 0x1ff", sign_conversion(0x1ff, 10), 511);   /* largest positive */
  check("sign 0x200", sign_conversion(0x200, 10), -512);  /* sign bit only */
  check("sign 0x3ff", sign_conversion(0x3ff, 10), -1);    /* all ones = -1 */

  /* pwdb = 3*log2(X) - 3*decimal_bit + coarse-decimal, called as (X,20,18). */
  check("pwdb 0",       pwdb_conversion(0, 20, 18), -54);       /* X==0 -> 1 */
  check("pwdb 1",       pwdb_conversion(1, 20, 18), -54);       /* only bit0 */
  check("pwdb 4",       pwdb_conversion(4, 20, 18), -48);       /* bit2, no dec */
  check("pwdb 6",       pwdb_conversion(6, 20, 18), -46);       /* bit2 + bit1 dec */
  check("pwdb 1<<18",   pwdb_conversion(1 << 18, 20, 18), 0);   /* full scale */
  check("pwdb 1<<19",   pwdb_conversion(1 << 19, 20, 18), 3);
  /* A representative accepted idle sample: I=Q=20 -> val=800, leading bit 9. */
  check("pwdb 800",     pwdb_conversion(800, 20, 18), -25);     /* in [-27,0) */

  /* --- NHM absolute floor (Jaguar3): the phydm NHM_ACS recipe. ---
   * Threshold unit PWdB U(8,1) = 2*(dBm+110): -110 dBm is 0, -80 dBm is 60
   * (phydm's NHM_IC_NOISE_TH), -92 dBm is IGI_2_NHM_TH(0x12) = 36. */
  check("th -110", nhm_th_from_dbm(-110), 0);
  check("th -80", nhm_th_from_dbm(-80), 60);
  check("th -92", nhm_th_from_dbm(-92), 36);
  /* The table is monotonic and bottoms out below a real 20 MHz floor (-95). */
  {
    bool mono = true;
    for (int i = 1; i < 11; i++)
      if (kNhmAbsThDbm[i] <= kNhmAbsThDbm[i - 1]) mono = false;
    check("table monotonic", mono ? 1 : 0, 1);
    check("table floor below -100", kNhmAbsThDbm[0] < -100 ? 1 : 0, 1);
  }
  uint8_t th[11];
  for (int i = 0; i < 11; i++) th[i] = nhm_th_from_dbm(kNhmAbsThDbm[i]);
  /* Weighted average of the bucket midpoints (phydm_nhm_cal_wgt_avg over
   * buckets 0..10 = nhm_level), then rtw_acs's -100 to dBm. All mass in bucket
   * 3 (-98..-95 dBm) -> midpoint -96.5 -> -97 in phydm's integer arithmetic. */
  {
    uint8_t b[12] = {}; b[2] = 20; b[3] = 215; b[4] = 20; int dbm = 0;
    check("bucket3 valid", nhm_abs_floor_dbm(b, th, 500, 500, dbm) ? 1 : 0, 1);
    check("bucket3 dbm", dbm, -97);
  }
  /* Bucket 0 = below the lowest threshold: weight th[0]-2, i.e. 1 dB under it. */
  {
    uint8_t b[12] = {}; b[0] = 230; b[1] = 25; int dbm = 0;
    check("bucket0 valid", nhm_abs_floor_dbm(b, th, 500, 500, dbm) ? 1 : 0, 1);
    check("bucket0 dbm", dbm, -105);
  }
  /* A frozen power estimate (on-air 8812EU, see docs/rx-spectrum-sensing.md):
   * the BB reports one constant value, so the whole window lands in a single
   * 3 dB bucket (241-255 of 255 observed, remainder <= 15 in one neighbour),
   * whereas a live noise estimate always spreads over neighbours (max bucket
   * <= 220 observed). Reject the degenerate histogram: never a fake dBm. */
  {
    uint8_t b[12] = {}; b[7] = 255; int dbm = 0;
    check("frozen single bucket invalid", nhm_abs_floor_dbm(b, th, 500, 500, dbm) ? 1 : 0, 0);
    uint8_t c[12] = {}; c[6] = 14; c[7] = 241;
    check("frozen 241 invalid", nhm_abs_floor_dbm(c, th, 500, 500, dbm) ? 1 : 0, 0);
    uint8_t d[12] = {}; d[3] = 235; d[4] = 20;
    check("235 still live", nhm_abs_floor_dbm(d, th, 500, 500, dbm) ? 1 : 0, 1);
  }
  /* Split mass: 128 in bucket 3 (wgt 27), 127 in bucket 4 (wgt 33) -> 29 -> -96. */
  {
    uint8_t b[12] = {}; b[3] = 128; b[4] = 127; int dbm = 0; /* live: two buckets */
    check("split valid", nhm_abs_floor_dbm(b, th, 500, 500, dbm) ? 1 : 0, 1);
    check("split dbm", dbm, -96);
  }
  /* Bucket 11 (above the top threshold) is signal, not floor: excluded. */
  {
    uint8_t b[12] = {}; b[3] = 12; b[4] = 8; b[11] = 235; int dbm = 0;
    check("signal excluded valid", nhm_abs_floor_dbm(b, th, 500, 500, dbm) ? 1 : 0, 1);
    check("signal excluded dbm", dbm, -96);
  }
  /* Too few floor samples (<8 of 255) -> no floor. */
  {
    uint8_t b[12] = {}; b[3] = 4; b[11] = 251; int dbm = 0;
    check("few samples invalid", nhm_abs_floor_dbm(b, th, 500, 500, dbm) ? 1 : 0, 0);
  }
  /* Empty histogram -> no floor. */
  {
    uint8_t b[12] = {}; int dbm = 0;
    check("empty invalid", nhm_abs_floor_dbm(b, th, 500, 500, dbm) ? 1 : 0, 0);
  }
  /* Idle time (duration, CCA/TX excluded) under 10 % of the window -> no floor;
   * exactly 10 % is enough. */
  {
    uint8_t b[12] = {}; b[3] = 200; b[4] = 55; int dbm = 0;
    check("idle 4% invalid", nhm_abs_floor_dbm(b, th, 20, 500, dbm) ? 1 : 0, 0);
    check("idle 10% valid", nhm_abs_floor_dbm(b, th, 50, 500, dbm) ? 1 : 0, 1);
  }

  /* --- read_nhm_absolute register recipe against a fake JGR3 BB. --- */
  {
    std::map<uint16_t, uint32_t> regs;
    const auto r = devourer::nhm_regs_jgr3();
    regs[r.ready] = (1u << 16) | 123;          /* ready, duration 123 */
    regs[r.res0_3] = 0x00000000u;
    regs[r.res4_7] = 0x00000000u;
    regs[r.res8_11] = 0x00000000u;
    regs[r.res0_3] |= 0xffu << 24;             /* bucket 3 = 255 */
    auto read32 = [&](uint16_t a) { return regs[a]; };
    int triggers = 0;
    auto set_bb = [&](uint16_t a, uint32_t m, uint32_t v) {
      int sh = 0; while (sh < 32 && !((m >> sh) & 1u)) sh++;
      if (a == r.ctrl && m == 0x2u && v == 1) triggers++;
      regs[a] = (regs[a] & ~m) | ((v << sh) & m);
    };
    uint8_t b[12] = {}; uint16_t dur = 0;
    const bool ok = devourer::read_nhm_absolute(r, read32, set_bb, b, dur);
    check("abs read ok", ok ? 1 : 0, 1);
    check("abs read bucket3", b[3], 255);
    check("abs read duration", dur, 123);
    check("abs read triggered", triggers, 1);
    /* cfg [11:8] = (divi<<3)|(inc_tx<<2)|(inc_cca<<1)|ccx_en: idle floor means
     * EXCLUDE tx-on and EXCLUDE cca-busy -> 0b0001 (vs the background 0b0011). */
    check("abs cfg excludes cca+tx", (regs[r.ctrl] >> 8) & 0xf, 0x1);
    /* Thresholds are the absolute table, not IGI-relative. */
    check("abs th0", regs[r.th0_3] & 0xff, nhm_th_from_dbm(kNhmAbsThDbm[0]));
    check("abs th3", (regs[r.th0_3] >> 24) & 0xff, nhm_th_from_dbm(kNhmAbsThDbm[3]));
    check("abs th8", (regs[r.th8] >> r.th8_shift) & 0xff, nhm_th_from_dbm(kNhmAbsThDbm[8]));
    check("abs th10", (regs[r.ctrl] >> 24) & 0xff, nhm_th_from_dbm(kNhmAbsThDbm[10]));
    check("abs period", (regs[r.period] >> 16) & 0xffff, 500);
    /* The background (IGI-relative) recipe writes th[8..10] through the same
     * masked-write helper, which shifts the value to the mask itself: a value
     * pre-shifted by the caller lands past the mask and the register reads 0
     * (the bug this pins: th[8..10] silently zero, buckets 9-11 meaningless).
     * IGI 0x1e -> th[0] = (30-14)*2 = 32, th[i] = 32 + 4i. */
    RxEnergy e;
    devourer::read_nhm(r, 0x1e, read32, set_bb, e);
    check("bg th0", regs[r.th0_3] & 0xff, 32);
    check("bg th8", (regs[r.th8] >> r.th8_shift) & 0xff, 32 + 4 * 8);
    check("bg th9", (regs[r.ctrl] >> 16) & 0xff, 32 + 4 * 9);
    check("bg th10", (regs[r.ctrl] >> 24) & 0xff, 32 + 4 * 10);
    check("bg cfg includes cca", (regs[r.ctrl] >> 8) & 0xf, 0x3);
    /* Not-ready never returns a histogram. */
    regs[r.ready] = 123;
    check("abs not ready", devourer::read_nhm_absolute(r, read32, set_bb, b, dur) ? 1 : 0, 0);
  }

  if (g_fail) {
    std::printf("noise_floor_math: %d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("noise_floor_math: all checks passed\n");
  return 0;
}
