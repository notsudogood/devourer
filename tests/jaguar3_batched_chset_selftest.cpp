/* Headless guard: RadioManagementJaguar3::set_channel_bwmode_batched must put
 * EXACTLY the plain set_channel_bwmode's write sequence on the bus -- same
 * addresses, same values, same widths, same order -- and leave the same
 * register state, for every move it is used for (a width switch 20/40/80 at
 * one primary, 20 <-> 10 MHz narrowband, a hop into and out of an 8822E spur
 * channel, a primary move),
 * on both dies.
 *
 * Two identical fake chips run the same steps; chip A takes every step on the
 * plain path, chip B on the batched path after a plain boot set. The fake
 * reads back what was last written (a configuration register), and an
 * address never written reads a fixed per-address pseudo-random value, so a
 * read-modify-write that composed from the wrong value would show as a
 * different write. What it cannot model: a hardware register whose readback
 * is not its last write -- those are the batched path's uncached set, which
 * always reads the chip, so both chips see the same reads there too.
 *
 * Also pins the point of the exercise: the batched path's blocking transfers
 * per step are a small fraction of the plain path's. */
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <tuple>
#include <vector>

#include "DeviceConfig.h"
#include "RtlAdapter.h"
#include "Transport.h"
#include "jaguar3/ChipVariant.h"
#include "jaguar3/RadioManagementJaguar3.h"
#include "logger.h"

namespace {

struct FakeChip : devourer::ITransport {
  std::map<uint16_t, uint8_t> bytes; /* byte-addressed register file */
  std::vector<std::tuple<uint16_t, int, uint32_t>> writes; /* addr, width, value */
  uint64_t blocking = 0; /* transfers that each paid a round trip */

  uint8_t byte_at(uint16_t a) {
    auto it = bytes.find(a);
    if (it != bytes.end())
      return it->second;
    /* Deterministic, address-keyed "power-on" value -- not stored, so the
     * register file holds only what was written (the batched path reads
     * registers the plain one never does: its priming set). */
    const uint32_t h = (uint32_t(a) * 2654435761u) ^ 0x5bd1e995u;
    return static_cast<uint8_t>(h >> ((a & 3) * 8));
  }
  uint32_t get(uint16_t a, int w) {
    uint32_t v = 0;
    for (int i = 0; i < w; ++i)
      v |= uint32_t(byte_at(static_cast<uint16_t>(a + i))) << (8 * i);
    return v;
  }
  void put(uint16_t a, int w, uint32_t v) {
    for (int i = 0; i < w; ++i)
      bytes[static_cast<uint16_t>(a + i)] = static_cast<uint8_t>(v >> (8 * i));
    writes.emplace_back(a, w, v);
  }

  uint32_t read32(uint16_t a) override { ++blocking; return get(a, 4); }
  uint16_t read16(uint16_t a) override { ++blocking; return static_cast<uint16_t>(get(a, 2)); }
  uint8_t read8(uint16_t a) override { ++blocking; return static_cast<uint8_t>(get(a, 1)); }
  bool write32(uint16_t a, uint32_t v) override { ++blocking; put(a, 4, v); return true; }
  bool write16(uint16_t a, uint16_t v) override { ++blocking; put(a, 2, v); return true; }
  bool write8(uint16_t a, uint8_t v) override { ++blocking; put(a, 1, v); return true; }
  /* A batch pays one round trip per group, not per op. */
  bool ctrl_batch(std::vector<devourer::CtrlOp> &ops) override {
    ++blocking;
    for (devourer::CtrlOp &op : ops) {
      if (op.write)
        put(op.addr, 4, op.value);
      else
        op.value = get(op.addr, 4);
    }
    return true;
  }
  bool write_bytes(uint16_t, const uint8_t *, size_t) override { return true; }
  bool is_usb() const override { return false; }
  bool tx_async(uint8_t, uint8_t *, size_t, unsigned) override { return false; }
  int tx_sync(uint8_t, uint8_t *, size_t, int) override { return -1; }
  void rx_loop(int, int, const std::function<void(const uint8_t *, int)> &,
               const std::function<bool()> &) override {}
};

int fails = 0;

} // namespace

#define CHECK(c)                                                               \
  do {                                                                         \
    if (!(c)) {                                                                \
      std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #c);         \
      ++fails;                                                                 \
    }                                                                          \
  } while (0)

int main() {
  auto logger = std::make_shared<Logger>();
  devourer::DeviceConfig cfg;

  struct Step {
    uint8_t ch, off;
    ChannelWidth_t bw;
    const char *what;
  };
  const std::vector<Step> steps = {
      {132, 1, CHANNEL_WIDTH_40, "132 80->40 (width down)"},
      {132, 0, CHANNEL_WIDTH_20, "132 40->20"},
      {132, 0, CHANNEL_WIDTH_10, "132 20->10 (narrowband)"},
      {132, 0, CHANNEL_WIDTH_20, "132 10->20"},
      {132, 1, CHANNEL_WIDTH_40, "132 20->40"},
      {132, 0, CHANNEL_WIDTH_80, "132 40->80 (width up)"},
      {36, 0, CHANNEL_WIDTH_80, "hop 132->36 at 80"},
      {149, 0, CHANNEL_WIDTH_80, "hop into spur block 149/80 (centre 155)"},
      {149, 1, CHANNEL_WIDTH_40, "spur 149/40 (centre 151)"},
      {153, 0, CHANNEL_WIDTH_20, "spur 153/20"},
      {132, 0, CHANNEL_WIDTH_80, "out of spur back to 132/80"},
      {140, 1, CHANNEL_WIDTH_40, "140+144"},
      {140, 0, CHANNEL_WIDTH_10, "140 40->10 (narrowband)"},
      {140, 1, CHANNEL_WIDTH_40, "140 10->40"},
  };

  for (auto variant : {jaguar3::ChipVariant::C8822E, jaguar3::ChipVariant::C8822C}) {
    auto a = std::make_shared<FakeChip>();
    auto b = std::make_shared<FakeChip>();
    jaguar3::RadioManagementJaguar3 ra(RtlAdapter(a, logger, cfg), logger, variant, cfg);
    jaguar3::RadioManagementJaguar3 rb(RtlAdapter(b, logger, cfg), logger, variant, cfg);
    /* Boot: both plain, so both start from the same tuned state. */
    ra.set_channel_bwmode(132, 0, CHANNEL_WIDTH_80);
    rb.set_channel_bwmode(132, 0, CHANNEL_WIDTH_80);
    CHECK(a->writes == b->writes);
    CHECK(a->bytes == b->bytes);

    for (const Step &s : steps) {
      a->writes.clear();
      b->writes.clear();
      a->blocking = 0;
      b->blocking = 0;
      ra.set_channel_bwmode(s.ch, s.off, s.bw);
      const bool ok = rb.set_channel_bwmode_batched(s.ch, s.off, s.bw);
      CHECK(ok);
      const bool same_seq = a->writes == b->writes;
      const bool same_state = a->bytes == b->bytes;
      if (!same_seq || !same_state)
        std::fprintf(stderr, "  %s %s: plain %zu writes, batched %zu\n",
                     variant == jaguar3::ChipVariant::C8822E ? "8822E" : "8822C",
                     s.what, a->writes.size(), b->writes.size());
      CHECK(same_seq);
      CHECK(same_state);
      std::printf("%s %-42s plain %4llu round trips, batched %3llu (%llu ops batched)\n",
                  variant == jaguar3::ChipVariant::C8822E ? "8822E" : "8822C", s.what,
                  (unsigned long long)a->blocking, (unsigned long long)b->blocking,
                  (unsigned long long)rb.last_batched_ops());
      /* The point: a fraction of the plain path's blocking transfers. The
       * first batched step may still learn reads the seed missed. What is
       * left on the 8822E is the reads that must reach the chip (the DAC
       * write-confirm loop, the CSI-mask table port): ~60 against ~190 on a
       * spur-free move, ~90 against ~215 on a spur one. */
      if (&s != &steps.front())
        CHECK(b->blocking * 2 < a->blocking);
    }
  }

  /* Any 20 of an 80 MHz block may be the primary: the RF tunes the block
   * centre, the BB pri-ch nibbles and REG_DATA_SC carry the slot as the
   * vendor's VHT_DATA_SC code (lowest 4, lower 2, upper 1, upperst 3; the
   * 40-in-80 half 10 = lower, 9 = upper). */
  {
    using RM = jaguar3::RadioManagementJaguar3;
    CHECK(RM::vht80_block_lowest(132) == 132 && RM::vht80_block_lowest(144) == 132);
    CHECK(RM::vht80_block_lowest(48) == 36 && RM::vht80_block_lowest(52) == 52);
    CHECK(RM::vht80_block_lowest(161) == 149 && RM::vht80_block_lowest(173) == 165);
    CHECK(RM::vht80_block_lowest(6) == 6 && RM::vht80_block_lowest(70) == 70);
    struct Slot { uint8_t ch; uint8_t sc20; uint8_t sc40; };
    for (Slot t : {Slot{132, 4, 10}, Slot{136, 2, 10}, Slot{140, 1, 9}, Slot{144, 3, 9}}) {
      auto chip = std::make_shared<FakeChip>();
      RM rm(RtlAdapter(chip, logger, cfg), logger, jaguar3::ChipVariant::C8822E, cfg);
      rm.set_channel_bwmode(t.ch, 0, CHANNEL_WIDTH_80);
      CHECK((chip->get(0x3c60, 4) & 0xff) == 138);                 /* RF18 = centre */
      CHECK(((chip->get(0x9b0, 4) >> 8) & 0xff) == uint32_t(t.sc20 | (t.sc20 << 4)));
      CHECK(chip->get(0x483, 1) == uint32_t(t.sc20 | (t.sc40 << 4)));
    }
  }

  if (fails)
    std::fprintf(stderr, "%d failure(s)\n", fails);
  else
    std::printf("jaguar3 batched channel set: parity OK\n");
  return fails ? EXIT_FAILURE : EXIT_SUCCESS;
}
