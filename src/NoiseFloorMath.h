/* Verbatim ports of the two phydm_math_lib helpers the active idle-noise-floor
 * measurement needs (reference/rtl8812au/hal/phydm/phydm_math_lib.c). Kept in a
 * standalone header so the per-generation noise-floor code and a selftest share
 * one copy. See docs/rx-spectrum-sensing.md.
 *
 * These drive the Jaguar1 (8812A/8821A) active-sampling path: the 0x0FA0 debug
 * port packs rx I/Q as 10-bit signed (sign_conversion), and the per-sample power
 * is 10*log10(I^2 + Q^2) approximated as 3*log2 via the leading set bit
 * (pwdb_conversion). */
#ifndef DEVOURER_NOISE_FLOOR_MATH_H
#define DEVOURER_NOISE_FLOOR_MATH_H

#include <cstdint>

namespace devourer {
namespace nf {

/* odm_pwdb_conversion: Y = 10*log10(X) ~= 3*log2(X), read off the leading set
 * bit. `total_bit` bounds the bit search; `decimal_bit` is the fixed-point
 * fractional width of X. The measurement forms S(20,18) = I^2 + Q^2, so it is
 * called with (X, 20, 18). The next-lower bit adds a coarse ~2 dB decimal. */
inline int32_t pwdb_conversion(int32_t X, uint32_t total_bit,
                               uint32_t decimal_bit) {
  int32_t integer = 0, decimal = 0;
  if (X == 0)
    X = 1; /* log2(x): x can't be 0 */
  for (uint32_t i = (total_bit - 1); i > 0; i--) {
    if (X & (int32_t{1} << i)) {
      integer = static_cast<int32_t>(i);
      decimal = (X & (int32_t{1} << (i - 1))) ? 2 : 0;
      break;
    }
  }
  return 3 * (integer - static_cast<int32_t>(decimal_bit)) + decimal;
}

/* odm_sign_conversion: interpret the low `total_bit` bits of `value` as a
 * two's-complement signed integer (rxi/rxq are 10-bit signed on the debug
 * port). */
inline int32_t sign_conversion(int32_t value, uint32_t total_bit) {
  if (value & (int32_t{1} << (total_bit - 1)))
    value -= (int32_t{1} << total_bit);
  return value;
}

/* --- NHM absolute floor (Jaguar3 8822C/8822E) ---
 * The Jaguar3 parts have no vendor idle-noise report (phydm_noisemonitor.c
 * dispatches only to 8812/8821/8814A and 8822B/8821C). The vendor's own
 * channel-select (hal_dm_acs.c, RTK_ACS_VERSION 3) derives a dBm floor for them
 * from the NHM histogram instead: ABSOLUTE thresholds (not IGI-relative),
 * tx-on and cca-busy samples EXCLUDED so only idle air is binned, then the
 * weighted average of the occupied bucket midpoints (phydm_nhm_cal_wgt +
 * phydm_nhm_cal_wgt_avg over buckets 0..10 = nhm_level, the top bucket being
 * signal) converted with NTH_TH_2_RSSI (th/2 - 10) and the ACS's -100.
 *
 * Threshold unit is PWdB U(8,1): th = 2*(dBm + 110). phydm's 11k table starts
 * at -92 dBm, which clips a real 20 MHz 5 GHz floor (~-95); devourer's table
 * below reaches -104 with the same 3 dB steps (phydm's NHM_DBG 1-dB mode shows
 * the thresholds are free), 5 dB steps above -80, top bucket = above -70. */
inline constexpr uint8_t nhm_th_from_dbm(int dbm) {
  return static_cast<uint8_t>(2 * (dbm + 110));
}

inline constexpr int8_t kNhmAbsThDbm[11] = {-104, -101, -98, -95, -92, -89,
                                             -86,  -83,  -80, -75, -70};

inline void nhm_abs_thresholds(uint8_t th[11]) {
  for (int i = 0; i < 11; i++)
    th[i] = nhm_th_from_dbm(kNhmAbsThDbm[i]);
}

/* buckets[12] + th[11] from one absolute-threshold NHM window, `duration` the
 * counted (idle) time and `period` the window, both in 4 us units. Returns
 * false — never a fake dBm — when fewer than 8 of the samples landed in the
 * floor buckets 0..10, under 10 % of the window was idle (a saturated
 * channel has no measurable floor), or the histogram is degenerate: on air
 * the 8812EU BB spends the first seconds after bring-up, and 2.4 GHz almost
 * permanently, reporting a FROZEN power estimate — one constant value, so
 * 241..255 of the 255 samples land in a single 3 dB bucket (a fixed −85 dBm,
 * 7-10 dB above the passive floor) — whereas a live noise estimate always
 * spreads over neighbouring buckets (max bucket <= 220 observed on two
 * 8812EU units — a thin 21-sample margin to the frozen 241). See
 * docs/rx-spectrum-sensing.md. The idle-time guard is inert on Jaguar3,
 * which reports the full window as duration. */
constexpr int kNhmFrozenBucket = 236;
inline bool nhm_abs_floor_dbm(const uint8_t buckets[12], const uint8_t th[11],
                              uint16_t duration, uint16_t period, int &dbm) {
  if (period == 0 || static_cast<uint32_t>(duration) * 10 < period)
    return false;
  int wgt[12];
  wgt[0] = th[0] >= 2 ? th[0] - 2 : 0;
  for (int i = 1; i < 11; i++)
    wgt[i] = (th[i - 1] + th[i]) >> 1;
  wgt[11] = th[10] + 2;
  int n_sum = 0, acc = 0, peak = 0;
  for (int i = 0; i < 11; i++) { /* bucket 11 = above the top threshold = signal */
    n_sum += buckets[i];
    acc += buckets[i] * wgt[i];
    if (buckets[i] > peak)
      peak = buckets[i];
  }
  if (n_sum < 8 || peak >= kNhmFrozenBucket)
    return false;
  const int avg_th = acc / n_sum;
  dbm = (avg_th >> 1) - 10 - 100;
  return true;
}

/* Same, over devourer's kNhmAbsThDbm table (what read_nhm_absolute wrote). */
inline bool nhm_abs_floor_dbm(const uint8_t buckets[12], uint16_t duration,
                              uint16_t period, int &dbm) {
  uint8_t th[11];
  nhm_abs_thresholds(th);
  return nhm_abs_floor_dbm(buckets, th, duration, period, dbm);
}

} // namespace nf
} // namespace devourer

#endif /* DEVOURER_NOISE_FLOOR_MATH_H */
