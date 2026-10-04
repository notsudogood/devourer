#pragma once

/* Decoded view of the 3-byte radiotap MCS field (known, flags, index) on the
 * TX/inject path — the one shared reading for all three generations'
 * send_packet parsers (J1/J2/J3), so the HT LDPC/STBC known/flag bits can't
 * silently diverge again. Bit positions per the radiotap MCS spec and
 * RadiotapBuilder::build_ht. */

#include <cstdint>

extern "C" {
#include "ieee80211_radiotap.h"
}

namespace devourer {

/* Devourer-private TX_FLAGS bit: "never aggregate this frame". With an A-MPDU
 * session on (SetAmpduMode), every frame is otherwise AGG_EN on one queue and
 * the MAC folds consecutive frames into one PPDU aired at its FIRST MPDU's
 * rate and bandwidth — a frame's own MCS/BW/LDPC/STBC are silently dropped
 * (measured on the 8812EU: 62 % of MCS0 frames aired at MCS5 when alternated
 * with MCS5 at saturation, ~40 % of 20 MHz frames aired at 40 MHz). A
 * no_agg frame is written AGG_EN=0 + BK=1 (break), the vendor rtl8822eu
 * xmit recipe for frames that must go out alone (EAPOL/ARP/DHCP), so it airs
 * as its own PPDU with its own rate/bw. Radiotap defines TX_FLAGS bits
 * 0x0001..0x0020; this one sits well above them. Honoured by Jaguar3 only;
 * other generations ignore it. */
constexpr uint16_t kRadiotapTxFlagNoAgg = 0x0100;

inline bool radiotap_tx_no_agg(uint16_t tx_flags) {
  return (tx_flags & kRadiotapTxFlagNoAgg) != 0;
}

struct RadiotapMcsField {
  bool have_mcs = false;
  uint8_t mcs = 0;   /* HT MCS index 0..31, valid when have_mcs */
  bool bw40 = false; /* MCS BW subfield == 40 MHz */
  uint8_t sgi = 0;
  uint8_t ldpc = 0;
  uint8_t stbc = 0;  /* STBC stream count 0..3 */
};

inline RadiotapMcsField decode_radiotap_mcs_field(const uint8_t *arg) {
  RadiotapMcsField f;
  const uint8_t known = arg[0];
  const uint8_t flags = arg[1];
  if ((flags & IEEE80211_RADIOTAP_MCS_BW_MASK) == IEEE80211_RADIOTAP_MCS_BW_40)
    f.bw40 = true;
  f.sgi = (flags & IEEE80211_RADIOTAP_MCS_SGI) ? 1 : 0;
  if ((known & IEEE80211_RADIOTAP_MCS_HAVE_FEC) &&
      (flags & IEEE80211_RADIOTAP_MCS_FEC_LDPC))
    f.ldpc = 1;
  if (known & IEEE80211_RADIOTAP_MCS_HAVE_STBC) /* flags [6:5] = stream count */
    f.stbc = (flags & IEEE80211_RADIOTAP_MCS_STBC_MASK) >>
             IEEE80211_RADIOTAP_MCS_STBC_SHIFT;
  if ((known & IEEE80211_RADIOTAP_MCS_HAVE_MCS) && arg[2] <= 31) {
    f.have_mcs = true;
    f.mcs = arg[2];
  }
  return f;
}

}  // namespace devourer
