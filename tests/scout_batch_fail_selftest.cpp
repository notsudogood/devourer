/* Headless guard: RtlJaguar3Device::GetRxEnergyScout must NOT write a
 * composed BB reset built out of a FAILED read.
 *
 * WHY THIS TEST IS MANDATORY AND THE OTHERS ARE NOT. The scout's reset dwords
 * 0x1d2c (RX clock gate) and 0x1eb4 are read, bit-twiddled and written back as
 * FULL DWORDS. While they were read through rtw_read, a dead register bus
 * threw and nothing was written. Reading them inside a ctrl_batch changed the
 * failure mode from "throw" to "returns false, leaves op.value at the 0 it was
 * constructed with" — so one cancelled, timed-out or short read on either op
 * would have composed 0x00000000/0x80000000 into 0x1d2c and
 * 0x02000000/0x00000000 into 0x1eb4, clobbering every other bit in two live BB
 * registers. The card goes deaf and STAYS deaf, because the next scout call
 * reads the corrupted value and writes it straight back.
 *
 * And it is invisible: valid_fa=false merely drops the sample, and
 * scout_read_bench's counter-plausibility tripwire is itself gated on
 * valid_fa, so the bench cannot see it either. A silent corruption with no
 * observable symptom short of a deaf radio is exactly the case that has to be
 * pinned by a test rather than by a comment.
 *
 * The fake transport overrides ctrl_batch to reproduce the ONE UsbTransport
 * failure semantic this test turns on: a failed read op keeps the value it was
 * constructed with, and the call returns false. It deliberately does not model
 * the rest — UsbTransport also skips an op whose submit failed and skips the
 * remainder of a chunk after a wait gives up, and it can finish a remainder
 * synchronously. Immaterial here (an all-read group, and the bail-out keys on
 * the return value alone), but the fake is not a stand-in for the transport. */
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "DeviceConfig.h"
#include "RtlAdapter.h"
#include "Transport.h"
#include "jaguar3/ChipVariant.h"
#include "jaguar3/RtlJaguar3Device.h"
#include "logger.h"

namespace {

struct FakeTransport : devourer::ITransport {
  std::map<uint16_t, uint32_t> regs;
  std::set<uint16_t> fail_read; /* a batched read of these reports failure */
  std::vector<std::pair<uint16_t, uint32_t>> writes; /* what reached the bus */

  bool ctrl_batch(std::vector<devourer::CtrlOp> &ops) override {
    bool ok = true;
    for (devourer::CtrlOp &op : ops) {
      if (op.write) {
        writes.emplace_back(op.addr, op.value);
        regs[op.addr] = op.value;
      } else if (fail_read.count(op.addr)) {
        ok = false; /* value left exactly as constructed — as UsbTransport does */
      } else {
        op.value = regs[op.addr];
      }
    }
    return ok;
  }

  uint32_t read32(uint16_t a) override { return regs[a]; }
  bool write32(uint16_t a, uint32_t v) override {
    writes.emplace_back(a, v);
    regs[a] = v;
    return true;
  }
  bool is_usb() const override { return false; }
  uint8_t read8(uint16_t) override { return 0; }
  uint16_t read16(uint16_t) override { return 0; }
  bool write8(uint16_t, uint8_t) override { return true; }
  bool write16(uint16_t, uint16_t) override { return true; }
  bool write_bytes(uint16_t, const uint8_t *, size_t) override { return true; }
  bool tx_async(uint8_t, uint8_t *, size_t, unsigned) override { return false; }
  int tx_sync(uint8_t, uint8_t *, size_t, int) override { return -1; }
  void rx_loop(int, int, const std::function<void(const uint8_t *, int)> &,
               const std::function<bool()> &) override {}
};

int fails = 0;

/* Live-ish resting values, the ones the real 8822EU reports. */
constexpr uint32_t k1d2c = 0xc0000000;
constexpr uint32_t k1eb4 = 0x31800002;

bool wrote(const FakeTransport &t, size_t i, uint16_t addr, uint32_t val) {
  return i < t.writes.size() && t.writes[i].first == addr &&
         t.writes[i].second == val;
}

int count_writes(const FakeTransport &t, uint16_t addr) {
  int n = 0;
  for (const auto &w : t.writes)
    if (w.first == addr)
      ++n;
  return n;
}

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

  /* ---- case 1: healthy reads -> the reset IS written, composed correctly */
  {
    auto fake = std::make_shared<FakeTransport>();
    fake->regs[0x1d2c] = k1d2c;
    fake->regs[0x1eb4] = k1eb4;
    fake->regs[0x2c08] = 0x00070003;
    RtlAdapter adapter(fake, logger, cfg);
    RtlJaguar3Device dev(adapter, logger, jaguar3::ChipVariant::C8822E, cfg);

    const RxEnergy e = dev.GetRxEnergyScout();
    CHECK(e.valid_fa);
    CHECK(e.cca_ofdm == 0x0007);
    CHECK(fake->writes.size() == 4);
    CHECK(count_writes(*fake, 0x1d2c) == 2);
    CHECK(count_writes(*fake, 0x1eb4) == 2);
    /* Composed from the values read, every other bit preserved. */
    CHECK(wrote(*fake, 0, 0x1d2c, k1d2c & ~(1u << 31)));
    CHECK(wrote(*fake, 1, 0x1eb4, k1eb4 | (1u << 25)));
    CHECK(wrote(*fake, 2, 0x1eb4, k1eb4 & ~(1u << 25)));
    CHECK(wrote(*fake, 3, 0x1d2c, k1d2c | (1u << 31)));
  }

  /* ---- case 2: a failed read of EITHER reset register must produce NO
   * writes at all. Not "a different write" — none. A composed dword built on
   * a zero that was never read is what bricks the receiver. */
  for (uint16_t bad : {uint16_t(0x1d2c), uint16_t(0x1eb4)}) {
    auto fake = std::make_shared<FakeTransport>();
    fake->regs[0x1d2c] = k1d2c;
    fake->regs[0x1eb4] = k1eb4;
    fake->fail_read.insert(bad);
    RtlAdapter adapter(fake, logger, cfg);
    RtlJaguar3Device dev(adapter, logger, jaguar3::ChipVariant::C8822E, cfg);

    const RxEnergy e = dev.GetRxEnergyScout();
    CHECK(!e.valid_fa);
    CHECK(fake->writes.empty());
    /* And the live registers are untouched, which is the actual damage. */
    CHECK(fake->regs[0x1d2c] == k1d2c);
    CHECK(fake->regs[0x1eb4] == k1eb4);
  }

  /* ---- case 3: a failed COUNTER read also suppresses the reset. The sample
   * is invalid either way, and skipping the reset is harmless — the counters
   * keep accumulating and the next successful call reports a wider delta. */
  {
    auto fake = std::make_shared<FakeTransport>();
    fake->regs[0x1d2c] = k1d2c;
    fake->regs[0x1eb4] = k1eb4;
    fake->fail_read.insert(0x2d08);
    RtlAdapter adapter(fake, logger, cfg);
    RtlJaguar3Device dev(adapter, logger, jaguar3::ChipVariant::C8822E, cfg);

    const RxEnergy e = dev.GetRxEnergyScout();
    CHECK(!e.valid_fa);
    CHECK(fake->writes.empty());
  }

  if (fails)
    return EXIT_FAILURE;
  std::puts("scout_batch_fail_selftest: ok");
  return 0;
}
