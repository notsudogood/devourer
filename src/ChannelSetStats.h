#ifndef DEVOURER_CHANNEL_SET_STATS_H
#define DEVOURER_CHANNEL_SET_STATS_H

#include <array>
#include <cstdint>

namespace devourer {

/* What the last channel set or width switch cost, and how it ran
 * (IRtlRadio::LastChannelSetStats). Diagnostic: it exists so a caller on
 * real hardware can see where a switch spends its time -- the offline
 * parity test counts round trips against a fake chip that answers
 * instantly, which cannot. Times are microseconds, wall clock. */
struct ChannelSetStats {
  enum class Path : uint8_t { None, Plain, Batched, Lean };
  Path path = Path::None;
  uint32_t total_us = 0;      /* the whole call, lock wait included */
  uint32_t lock_us = 0;       /* waiting for the device register lock */
  uint32_t prime_us = 0;      /* batched/lean: the priming read */
  uint32_t flush_us = 0;      /* batched/lean: write batches, summed */
  uint32_t flushes = 0;
  uint32_t batched_writes = 0;
  uint32_t sync_us = 0;       /* batched/lean: one-round-trip transfers, summed */
  uint32_t sync_ops = 0;      /* reads that reached the chip, 8/16-bit writes */
  uint32_t post_us = 0;       /* after the set: TX-power / CCA / bank re-asserts */
  uint32_t xfers = 0;         /* control transfers the transport counted */
  uint32_t dack_loops = 0;    /* AFE write-confirm iterations (8 per DAC reset
                               * when none repeated; 8822E at 40/80 only) */
  /* Lean width switch (tuning.lean_width): why it declined, null when it
   * ran or was not asked for; then its readback check. */
  const char *lean_skip = nullptr;
  uint32_t lean_writes = 0;
  uint32_t verify_regs = 0, verify_bad = 0;
  struct Miss {
    uint16_t addr = 0;
    uint32_t want = 0, got = 0, mask = 0;
  };
  std::array<Miss, 4> misses{};
};

inline const char *to_string(ChannelSetStats::Path p) {
  switch (p) {
  case ChannelSetStats::Path::Plain: return "plain";
  case ChannelSetStats::Path::Batched: return "batched";
  case ChannelSetStats::Path::Lean: return "lean";
  default: return "none";
  }
}

} // namespace devourer

#endif /* DEVOURER_CHANNEL_SET_STATS_H */
