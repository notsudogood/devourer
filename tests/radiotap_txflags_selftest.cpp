/* Headless wire-contract guard for the radiotap TX-flag path: what
 * build_stream_radiotap() emits must decode back to the TxMode that went in.
 * Guards the J2/J3 regression where HT LDPC/STBC known/flag bits were
 * silently ignored (only the VHT case honoured them). No device is opened. */
#include <cstdio>
#include <cstring>
#include <vector>

#include "RadiotapBuilder.h"
#include "RadiotapPeek.h"
#include "RadiotapTxFlags.h"
#include "ieee80211_radiotap.h"
#if defined(DEVOURER_HAVE_JAGUAR3)
#include "jaguar3/FrameParserJaguar3.h"
#endif

static int failures = 0;

#define CHECK_EQ(expr, want)                                                   \
  do {                                                                         \
    long got_ = long(expr);                                                    \
    if (got_ != long(want)) {                                                  \
      std::fprintf(stderr, "FAIL %s:%d: %s = %ld, want %ld\n", __FILE__,       \
                   __LINE__, #expr, got_, long(want));                         \
      failures++;                                                              \
    }                                                                          \
  } while (0)

/* Walk a built radiotap header and return the decoded MCS field. */
static devourer::RadiotapMcsField decode_ht(const std::vector<uint8_t> &rt) {
  struct ieee80211_radiotap_iterator it;
  auto *hdr = reinterpret_cast<struct ieee80211_radiotap_header *>(
      const_cast<uint8_t *>(rt.data()));
  devourer::RadiotapMcsField f;
  if (ieee80211_radiotap_iterator_init(&it, hdr, rt.size(), nullptr) != 0) {
    std::fprintf(stderr, "FAIL: iterator init\n");
    failures++;
    return f;
  }
  while (ieee80211_radiotap_iterator_next(&it) == 0) {
    if (it.this_arg_index == IEEE80211_RADIOTAP_MCS)
      f = devourer::decode_radiotap_mcs_field(it.this_arg);
  }
  return f;
}

/* Walk a built radiotap header and return its TX_FLAGS field (0xFFFF = absent). */
static uint16_t find_tx_flags(const std::vector<uint8_t> &rt) {
  struct ieee80211_radiotap_iterator it;
  auto *hdr = reinterpret_cast<struct ieee80211_radiotap_header *>(
      const_cast<uint8_t *>(rt.data()));
  if (ieee80211_radiotap_iterator_init(&it, hdr, rt.size(), nullptr) != 0)
    return 0xFFFF;
  while (ieee80211_radiotap_iterator_next(&it) == 0) {
    if (it.this_arg_index == IEEE80211_RADIOTAP_TX_FLAGS)
      return static_cast<uint16_t>(it.this_arg[0] | (it.this_arg[1] << 8));
  }
  return 0xFFFF;
}

int main() {
  /* HT MCS0/20, LDPC+STBC (mabur's MAX_RANGE control mode). */
  devourer::TxMode m;
  m.mode = devourer::TxMode::Mode::HT;
  m.ht_mcs = 0;
  m.bw_mhz = 20;
  m.ldpc = true;
  m.stbc = true;
  auto f = decode_ht(devourer::build_stream_radiotap(m));
  CHECK_EQ(f.have_mcs, 1);
  CHECK_EQ(f.mcs, 0);
  CHECK_EQ(f.bw40, 0);
  CHECK_EQ(f.sgi, 0);
  CHECK_EQ(f.ldpc, 1);
  CHECK_EQ(f.stbc, 1);

  /* HT MCS7/40/SGI, no LDPC/STBC. */
  m = devourer::TxMode{};
  m.mode = devourer::TxMode::Mode::HT;
  m.ht_mcs = 7;
  m.bw_mhz = 40;
  m.sgi = true;
  f = decode_ht(devourer::build_stream_radiotap(m));
  CHECK_EQ(f.have_mcs, 1);
  CHECK_EQ(f.mcs, 7);
  CHECK_EQ(f.bw40, 1);
  CHECK_EQ(f.sgi, 1);
  CHECK_EQ(f.ldpc, 0);
  CHECK_EQ(f.stbc, 0);

  /* Raw 3-byte MCS field (bypasses the builder): BW code 20U (not 40 MHz),
   * STBC stream count 2, FEC known + LDPC, GI not short, MCS 5. Pins the
   * decoder's bit positions independently of what build_stream_radiotap can
   * emit. */
  {
    const uint8_t raw[3] = {
        /* known */ IEEE80211_RADIOTAP_MCS_HAVE_BW |
            IEEE80211_RADIOTAP_MCS_HAVE_MCS | IEEE80211_RADIOTAP_MCS_HAVE_FEC |
            IEEE80211_RADIOTAP_MCS_HAVE_STBC,
        /* flags */ IEEE80211_RADIOTAP_MCS_BW_20U |
            IEEE80211_RADIOTAP_MCS_FEC_LDPC |
            (2 << IEEE80211_RADIOTAP_MCS_STBC_SHIFT),
        /* index */ 5};
    const auto g = devourer::decode_radiotap_mcs_field(raw);
    CHECK_EQ(g.have_mcs, 1);
    CHECK_EQ(g.mcs, 5);
    CHECK_EQ(g.bw40, 0);
    CHECK_EQ(g.sgi, 0);
    CHECK_EQ(g.ldpc, 1);
    CHECK_EQ(g.stbc, 2);
  }

  /* VHT wire contract: pin the byte positions the device parsers read
   * (known bit0 = STBC, arg[2] bit0 = STBC flag, arg[8] bit0 = LDPC). */
  m = devourer::TxMode{};
  m.mode = devourer::TxMode::Mode::VHT;
  m.vht_mcs = 3;
  m.vht_nss = 1;
  m.bw_mhz = 80;
  m.ldpc = true;
  m.stbc = true;
  auto rt = devourer::build_stream_radiotap(m);
  /* 22-byte VHT radiotap: header(8) + TX_FLAGS(2) + VHT info(12) at off 10. */
  CHECK_EQ(rt.size(), 22);
  CHECK_EQ(rt[10] & 0x01, 0x01); /* known: STBC        */
  CHECK_EQ(rt[12] & 0x01, 0x01); /* flags: STBC        */
  CHECK_EQ(rt[18] & 0x01, 0x01); /* coding[u0]: LDPC   */
  CHECK_EQ(rt[13], 4);           /* bw code: 80 MHz    */

  /* TxMode::no_agg rides the TX_FLAGS field (devourer-private bit) on every
   * builder, and a default TxMode stays byte-identical (NoAck only). The
   * device parsers read it back with radiotap_tx_no_agg(). */
  {
    const devourer::TxMode::Mode modes[4] = {
        devourer::TxMode::Mode::Legacy, devourer::TxMode::Mode::HT,
        devourer::TxMode::Mode::VHT, devourer::TxMode::Mode::HE};
    for (const auto mode : modes) {
      devourer::TxMode t;
      t.mode = mode;
      const uint16_t plain = find_tx_flags(devourer::build_stream_radiotap(t));
      CHECK_EQ(plain, IEEE80211_RADIOTAP_F_TX_NOACK);
      CHECK_EQ(devourer::radiotap_tx_no_agg(plain), 0);
      t.no_agg = true;
      const auto rt_na = devourer::build_stream_radiotap(t);
      const uint16_t na = find_tx_flags(rt_na);
      CHECK_EQ(na, IEEE80211_RADIOTAP_F_TX_NOACK | devourer::kRadiotapTxFlagNoAgg);
      CHECK_EQ(devourer::radiotap_tx_no_agg(na), 1);
      /* Length is part of the J3 HT-vs-VHT contract (13 = HT/legacy). */
      t.no_agg = false;
      CHECK_EQ(rt_na.size(), devourer::build_stream_radiotap(t).size());
    }
  }

  /* TxMode::hw_queue rides the TX_FLAGS queue field ([11:9]) on every
   * builder, composes with no_agg, keeps the radiotap length (the J3
   * HT-vs-VHT length contract), and Default stays byte-identical. The batch
   * packer's peek reads back the same field the device parser does. */
  {
    const devourer::TxMode::Mode modes[4] = {
        devourer::TxMode::Mode::Legacy, devourer::TxMode::Mode::HT,
        devourer::TxMode::Mode::VHT, devourer::TxMode::Mode::HE};
    const devourer::HwQueue queues[6] = {
        devourer::HwQueue::BK, devourer::HwQueue::BE, devourer::HwQueue::VI,
        devourer::HwQueue::VO, devourer::HwQueue::Mgmt, devourer::HwQueue::High};
    for (const auto mode : modes) {
      devourer::TxMode t;
      t.mode = mode;
      const auto rt_plain = devourer::build_stream_radiotap(t);
      CHECK_EQ(devourer::radiotap_tx_queue_code(find_tx_flags(rt_plain)), 0);
      for (const auto q : queues) {
        t.hw_queue = q;
        t.no_agg = (static_cast<int>(q) % 2) == 0;
        const auto rt = devourer::build_stream_radiotap(t);
        const uint16_t f = find_tx_flags(rt);
        CHECK_EQ(devourer::radiotap_tx_queue_code(f), static_cast<int>(q));
        CHECK_EQ(devourer::radiotap_tx_no_agg(f), t.no_agg ? 1 : 0);
        CHECK_EQ(f & IEEE80211_RADIOTAP_F_TX_NOACK, IEEE80211_RADIOTAP_F_TX_NOACK);
        CHECK_EQ(rt.size(), rt_plain.size());
        std::vector<uint8_t> pkt = rt; /* the peek needs room for an MPDU */
        pkt.resize(rt.size() + 24, 0);
        CHECK_EQ(devourer::radiotap_peek_tx_flags(pkt.data(), pkt.size()), f);
      }
    }
    /* Code 7 is reserved and reads as default; the field never leaks into
     * the no-aggregation bit or the radiotap-defined bits. */
    CHECK_EQ(devourer::radiotap_tx_queue_code(devourer::radiotap_tx_queue_flags(7)), 0);
    CHECK_EQ(devourer::radiotap_tx_queue_flags(7) & (devourer::kRadiotapTxFlagNoAgg | 0x00FF), 0);
    CHECK_EQ(devourer::radiotap_tx_queue_code(devourer::kRadiotapTxFlagNoAgg), 0);
  }

#if defined(DEVOURER_HAVE_JAGUAR3)
  /* Jaguar3 QSEL per queue code: the vendor QSLT values (hal/hal_com.h);
   * default and the reserved code leave the caller's QSEL alone. */
  CHECK_EQ(jaguar3::qsel_for_queue_code_8822c(0), -1);
  CHECK_EQ(jaguar3::qsel_for_queue_code_8822c(1), 0x02);
  CHECK_EQ(jaguar3::qsel_for_queue_code_8822c(2), 0x00);
  CHECK_EQ(jaguar3::qsel_for_queue_code_8822c(3), 0x05);
  CHECK_EQ(jaguar3::qsel_for_queue_code_8822c(4), 0x07);
  CHECK_EQ(jaguar3::qsel_for_queue_code_8822c(5), 0x12);
  CHECK_EQ(jaguar3::qsel_for_queue_code_8822c(6), 0x11);
  CHECK_EQ(jaguar3::qsel_for_queue_code_8822c(7), -1);
  /* Only the TID queues aggregate: a frame moved to MGMT/HIGH must leave
   * with AGG_EN cleared (AGG_EN on MGMT wedges the queue). */
  for (uint8_t code = 1; code <= 4; ++code)
    CHECK_EQ(jaguar3::qsel_aggregates_8822c(
                 static_cast<uint8_t>(jaguar3::qsel_for_queue_code_8822c(code))), 1);
  CHECK_EQ(jaguar3::qsel_aggregates_8822c(0x12), 0);
  CHECK_EQ(jaguar3::qsel_aggregates_8822c(0x11), 0);
#endif

  if (failures) {
    std::fprintf(stderr, "%d failure(s)\n", failures);
    return 1;
  }
  std::printf("radiotap_txflags: OK\n");
  return 0;
}
