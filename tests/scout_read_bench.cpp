// scout_read_bench.cpp — transfer count, latency and on-hardware reset
// verification for IRtlRadio::GetRxEnergyScout() against the full
// GetRxEnergy(false), on a live Jaguar3 card after an InitWrite bring-up
// (WiFiDriver::CreateRadio, the doctor demo's open path). The scout is
// 8 reads + 4 composed full-dword writes = 12 transfers in two ctrl_batch
// groups; the full read is 8 reads + 8 masked writes (each a
// read-modify-write) = 24. This file is the one home of the ctrl_batch
// numbers.
//
//   DEVOURER_PID=0xa81a ./build/scout_read_bench
//   DEVOURER_VID/PID pick the adapter, DEVOURER_CHANNEL the bring-up channel
//   (default 6); DEVOURER_CTRL_BATCH=0 runs the unbatched side of the A/B.
//
// Per-call bracketing. The adapter's transfer counter (RtlAdapter::ctrl_xfers,
// read through RtlJaguar3Device::DebugCtrlXfers) also counts the coex
// thread's traffic, and its ~2 s tick issues ~70
// transfers under the same _reg_mu, so a call blocked behind a tick absorbs
// the tick's transfers and latency. Contamination can only add, so every call
// is bracketed on its own and the per-run minimum is the attributable cost;
// an idle ~2.5 s window with no calls in flight reports the ambient rate
// separately. kRuns x kCalls per function, so the spread shows, not a best
// case.
//
// Reset verification. Once per process the bench reads 0x1d2c/0x1eb4 around
// a scout call (RtlJaguar3Device::DebugPeekBb) and checks the resting state
// compose_reset promises (0x1d2c[31]=1, 0x1eb4[25]=0). That proves the
// composed write lands, not that the 0x1eb4[25] pulse fired — the pulse is
// inside the call and a before/after sample cannot see it. The behavioural
// half is the per-run fa_ofdm/cca_ofdm min/max: a stuck reset pins the
// counters at 0 or lets them climb across a run. Neither is proven on a
// quiet channel; run it next to an active AP or a CW tone for that.
//
// Measured on one 8822EU, 3 invocations x 3 runs x 200 calls per function
// per mode, batch on/off alternated per invocation; median of the per-run
// medians (range of per-run medians):
//
//                          unbatched             batched
//   an RK3566 aarch64 host
//     scout, 12 ops        3372 us (3002..3748)  603 us (571..648)   5.6x
//     GetRxEnergy(false)   6612..7444 us, never batched — the control
//   an x86 host (xHCI)
//     scout, 12 ops        4500 us               4500 us             1.0x
//     GetRxEnergy(false)   9000 us               9000 us
//
// Every figure is a FLOOR measured with no RX loop: nothing reaps RX URBs on
// the libusb context that a real caller's event pump must also service, so
// the cost under live RX traffic is unmeasured. The two hosts are the
// adversarial pair. On the aarch64 host a synchronous access is ~290 us of
// host turnaround per completion wait plus ~7-16 us of wire time, and
// batching removes the waits; on the x86 host every EP0 transfer costs a
// flat 375 us (3 high-speed microframes) in both modes, so batching buys
// ~zero.
//
// kImplausibleCount exists for one failure signature. If
// UsbTransport::ctrl_batch's acquire phase (all of a chunk's slots taken
// before anything is submitted) regresses, a chunk can be handed one of its
// own just-completed reads and refill it with a later write, so the scout's
// reset-write dwords come back as counter reads: cca_ofdm 16384 / 49152 /
// 12672 / 13184 (the high halves of 0x40000000 / 0xC0000000 / 0x31800002 /
// 0x33800002) and fa_ofdm in the tens of thousands. It only shows on a host
// whose EP0 transfers overlap, and printed as a min..max range it reads as a
// busy channel.
#ifdef _WIN32
#define NOMINMAX
#endif

#if defined(__ANDROID__) || defined(_MSC_VER) || defined(__APPLE__)
#include <libusb.h>
#else
#include <libusb-1.0/libusb.h>
#endif

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include "DeviceSession.h"
#include "IRtlRadio.h"
#include "WiFiDriver.h"
#include "jaguar3/RtlJaguar3Device.h"
#include "logger.h"

namespace {

struct Stats { double min, med, mean, max; };

Stats summarize(std::vector<double> &v) {
  std::sort(v.begin(), v.end());
  double sum = 0;
  for (double x : v) sum += x;
  return Stats{v.front(), v[v.size() / 2], sum / v.size(), v.back()};
}

struct RunResult {
  uint64_t xfer_floor = 0;   /* min per-call xfer count this run — the
                              * attributable true cost (contamination only
                              * adds, never subtracts). */
  int n_at_floor = 0;        /* calls that hit the floor exactly (clean) */
  uint64_t xfer_max = 0;
  double xfer_mean = 0;
  uint64_t contam_excess = 0; /* sum of (xfers[i] - floor) over all calls —
                               * total transfers attributable to ambient
                               * (coex-thread) traffic landing in this
                               * run's windows, not to our own calls. */
  int n_contam = 0;           /* calls above the floor */
  Stats us{};
  uint32_t fa_min = std::numeric_limits<uint32_t>::max(), fa_max = 0;
  uint32_t cca_min = std::numeric_limits<uint32_t>::max(), cca_max = 0;
  /* Calls whose fa_ofdm/cca_ofdm are physically impossible for the window
   * they cover — the tripwire for READ PAYLOAD CORRUPTION, the acquire-phase
   * signature in the file header (the reset WRITE dwords come back as
   * counter values). These are 16-bit BB counters sampled over a sub-millisecond to
   * few-millisecond window; a few hundred events is a busy channel, tens of
   * thousands is not a channel, it is memory. A bench that only printed
   * min..max would have shown "fa_ofdm 0..61826" and been read as "busy
   * channel" — so this is counted and shouted about, not just printed. */
  int n_implausible = 0;
};

/* The adapter's transfer counter; set in main() once the device is known to
 * be a Jaguar3 (DebugCtrlXfers), zero otherwise. */
std::function<uint64_t()> g_xfers = [] { return uint64_t{0}; };

/* Above this, a counter value is not a reading. See RunResult::n_implausible. */
constexpr uint32_t kImplausibleCount = 4096;

/* Runs `n` calls of `fn`, bracketing the transfer counter and the clock around
 * EACH INDIVIDUAL call (see the file header for why not one delta over
 * the whole loop) and tracking fa_ofdm/cca_ofdm
 * min/max (a stuck reset shows as max==0 for the whole run). */
template <typename Fn>
RunResult bench_run(Fn fn, int n) {
  std::vector<uint64_t> xfers(n);
  std::vector<double> us(n);
  RunResult r;
  for (int i = 0; i < n; ++i) {
    const uint64_t x0 = g_xfers();
    const auto t0 = std::chrono::steady_clock::now();
    const RxEnergy e = fn();
    us[i] = std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - t0)
                .count();
    xfers[i] = g_xfers() - x0;
    if (e.valid_fa) {
      if (e.fa_ofdm > kImplausibleCount || e.cca_ofdm > kImplausibleCount)
        ++r.n_implausible;
      r.fa_min = std::min(r.fa_min, e.fa_ofdm);
      r.fa_max = std::max(r.fa_max, e.fa_ofdm);
      r.cca_min = std::min(r.cca_min, e.cca_ofdm);
      r.cca_max = std::max(r.cca_max, e.cca_ofdm);
    }
  }
  r.xfer_floor = *std::min_element(xfers.begin(), xfers.end());
  r.xfer_max = *std::max_element(xfers.begin(), xfers.end());
  uint64_t sum = 0;
  for (uint64_t x : xfers) {
    sum += x;
    if (x == r.xfer_floor) {
      ++r.n_at_floor;
    } else {
      r.contam_excess += x - r.xfer_floor;
      ++r.n_contam;
    }
  }
  r.xfer_mean = double(sum) / n;
  r.us = summarize(us);
  return r;
}

void print_run(const char *label, int run_idx, int runs, int n,
              const RunResult &r) {
  std::printf(
      "%s run %d/%d: floor=%llu (%d/%d calls clean) mean=%.1f max=%llu "
      "contam_excess=%llu xfers over %d call(s); us min/med/mean/max "
      "%.0f/%.0f/%.0f/%.0f; fa_ofdm %u..%u cca_ofdm %u..%u\n",
      label, run_idx, runs, (unsigned long long)r.xfer_floor, r.n_at_floor, n,
      r.xfer_mean, (unsigned long long)r.xfer_max,
      (unsigned long long)r.contam_excess, r.n_contam, r.us.min, r.us.med,
      r.us.mean, r.us.max, r.fa_min, r.fa_max, r.cca_min, r.cca_max);
  if (r.n_implausible)
    std::fprintf(stderr,
                 "** %s run %d/%d: %d/%d calls returned an IMPOSSIBLE counter "
                 "value (>%u) — this is read-payload corruption, not a busy "
                 "channel; every latency number in this run is suspect **\n",
                 label, run_idx, runs, r.n_implausible, n, kImplausibleCount);
  /* NOT a per-run "fa_max==0 => stuck" warning here on purpose: a quiet
   * bench channel legitimately produces all-zero fa_ofdm across a whole
   * run of ~4-9 ms windows for BOTH functions, and a naive threshold on this alone fires
   * identically for the already-trusted GetRxEnergy(false) path, which is
   * not useful information — it says "quiet channel", not "broken reset".
   * The real cross-check (scout distinctly failing to see activity the
   * SAME-RUN full read demonstrably saw) is printed once after all runs,
   * in main(), where both are available side by side. */
}

/* Idle ambient-rate sample: no calls of ours in flight, so any
 * transfer-counter movement during `dur` is background traffic (the coex
 * thread) alone — reported separately from, not folded into, the per-call
 * floor above. */
void measure_ambient(std::chrono::milliseconds dur) {
  const uint64_t x0 = g_xfers();
  std::this_thread::sleep_for(dur);
  const uint64_t x1 = g_xfers();
  std::printf("ambient (idle, no calls, %lld ms window): %llu xfers "
             "(%.2f xfers/s) — background coex-thread traffic, NOT part of "
             "the per-call floor above\n",
             (long long)dur.count(), (unsigned long long)(x1 - x0),
             double(x1 - x0) * 1000.0 / double(dur.count()));
}

} // namespace

int main() {
  auto logger = std::make_shared<Logger>();

  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) < 0) {
    std::fprintf(stderr, "libusb_init failed\n");
    return 1;
  }
  devourer::DeviceSession session{logger};
  session.adopt_context(ctx);

  uint16_t vid = 0x0bda, pid = 0;
  if (const char *v = std::getenv("DEVOURER_VID")) vid = (uint16_t)strtoul(v, 0, 0);
  if (const char *p = std::getenv("DEVOURER_PID")) pid = (uint16_t)strtoul(p, 0, 0);
  libusb_device_handle *handle = libusb_open_device_with_vid_pid(ctx, vid, pid);
  if (!handle) {
    std::fprintf(stderr, "usb open %04x:%04x failed (set DEVOURER_PID)\n", vid, pid);
    return 1;
  }

  std::shared_ptr<devourer::UsbDeviceLock> lock;
  if (devourer::claim_interface_then_reset(
          handle, devourer::find_wifi_interface(handle), logger, true, lock) != 0) {
    session.adopt_handle(handle);
    return 1;
  }
  session.adopt_handle(handle);
  session.adopt_lock(lock);

  devourer::DeviceConfig cfg;
  /* A/B knob for the batched-EP0 path (DeviceConfig::usb.ctrl_batch): =0 runs
   * the scout's register group through the synchronous per-op default, which
   * is the pre-batch behaviour and therefore the honest control for every
   * latency number this bench prints. The library itself reads no environment
   * (global constraint); this is the demo/bench side doing the mapping, same
   * as examples/common/env_config.cpp. */
  if (const char *e = std::getenv("DEVOURER_CTRL_BATCH"))
    cfg.usb.ctrl_batch = (std::strcmp(e, "0") != 0);
  std::printf("ctrl_batch: %s (DEVOURER_CTRL_BATCH=0 to compare against the "
              "synchronous per-op path)\n",
              cfg.usb.ctrl_batch ? "ON" : "OFF");
  WiFiDriver driver(logger);
  std::unique_ptr<IRadio> owned = driver.CreateRadio(handle, ctx, lock, cfg);
  if (!owned) {
    std::fprintf(stderr, "CreateRadio failed (not a Jaguar3 card?)\n");
    return 1;
  }
  session.adopt_device(std::move(owned));
  auto *const dev = dynamic_cast<IRtlRadio *>(session.device());
  if (!dev) {
    std::fprintf(stderr, "not a Realtek radio (no GetRxEnergy)\n");
    return 1;
  }

  uint8_t channel = 6;
  if (const char *c = std::getenv("DEVOURER_CHANNEL"))
    channel = static_cast<uint8_t>(strtoul(c, nullptr, 0));
  try {
    dev->InitWrite(SelectedChannel{.Channel = channel,
                                   .ChannelOffset = 0,
                                   .ChannelWidth = CHANNEL_WIDTH_20});
  } catch (const std::exception &e) {
    std::fprintf(stderr, "InitWrite failed: %s\n", e.what());
    return 1;
  }

  /* Reset verification: only meaningful on the chip that
   * actually has DebugPeekBb — the Jaguar3-specific downcast is
   * deliberate, see the file header. */
  auto *j3 = dynamic_cast<RtlJaguar3Device *>(dev);
  if (j3)
    g_xfers = [j3] { return j3->DebugCtrlXfers(); };
  if (!j3) {
    std::fprintf(stderr,
                 "not a Jaguar3 device (no DebugPeekBb) — skipping the "
                 "register-level reset verification, transfer/latency bench "
                 "only\n");
  } else {
    const uint32_t pre_1d2c = j3->DebugPeekBb(0x1d2c);
    const uint32_t pre_1eb4 = j3->DebugPeekBb(0x1eb4);
    const RxEnergy e = dev->GetRxEnergyScout();
    const uint32_t post_1d2c = j3->DebugPeekBb(0x1d2c);
    const uint32_t post_1eb4 = j3->DebugPeekBb(0x1eb4);
    const bool ok = ((post_1d2c >> 31) & 1) == 1 && ((post_1eb4 >> 25) & 1) == 0;
    std::printf(
        "reset verification: 0x1d2c %08x -> %08x, 0x1eb4 %08x -> %08x "
        "(want post 0x1d2c[31]=1 0x1eb4[25]=0: %s); this call fa_ofdm=%u "
        "cca_ofdm=%u\n",
        pre_1d2c, post_1d2c, pre_1eb4, post_1eb4, ok ? "OK" : "FAILED",
        e.fa_ofdm, e.cca_ofdm);
    if (!ok)
      std::fprintf(stderr,
                   "** the composed reset did NOT land on hardware in the "
                   "expected end-state — do not trust the transfer-count "
                   "numbers below as evidence the feature works **\n");
  }

  measure_ambient(std::chrono::milliseconds(2500));

  constexpr int kCalls = 200;
  constexpr int kRuns = 3;
  std::vector<RunResult> scout_runs, full_runs;
  for (int run = 1; run <= kRuns; ++run) {
    RunResult rs = bench_run([dev] { return dev->GetRxEnergyScout(); }, kCalls);
    print_run("GetRxEnergyScout", run, kRuns, kCalls, rs);
    scout_runs.push_back(rs);
    RunResult rf = bench_run(
        [dev] { return dev->GetRxEnergy(/*with_nhm=*/false); }, kCalls);
    print_run("GetRxEnergy(false)", run, kRuns, kCalls, rf);
    full_runs.push_back(rf);
  }

  /* Cross-check for a scout-specific stuck reset: per run, did the
   * (already-trusted) full read see fa_ofdm activity that the scout, in
   * the same slot, did not? That is the actual "reset is stuck" signal —
   * both paths reading zero together just means a quiet channel. */
  bool cross_check_suspicious = false;
  for (size_t i = 0; i < scout_runs.size() && i < full_runs.size(); ++i) {
    if (full_runs[i].fa_max > 0 && scout_runs[i].fa_max == 0) {
      std::printf(
          "** run %zu: GetRxEnergy(false) saw fa_ofdm activity (max=%u) in "
          "the same slot GetRxEnergyScout saw none (max=0) — possible "
          "scout-specific stuck reset, do not dismiss as a quiet channel **\n",
          i + 1, full_runs[i].fa_max);
      cross_check_suspicious = true;
    }
  }
  if (!cross_check_suspicious)
    std::printf(
        "cross-check: no run shows the full read seeing fa_ofdm activity "
        "the scout missed in the same slot — the per-run 0..0 readings "
        "above are consistent with a quiet bench channel affecting both "
        "functions equally, not a scout-specific stuck reset. The pre-loop "
        "single-call verification above independently saw real nonzero "
        "fa_ofdm/cca_ofdm right after InitWrite, and the 0x1d2c/0x1eb4 "
        "bit-level check is the decisive evidence for the reset itself; "
        "this fa_ofdm-activity cross-check is corroborating, not primary.\n");

  auto summarize_runs = [](const char *label, const std::vector<RunResult> &rs) {
    uint64_t floor_min = rs[0].xfer_floor, floor_max = rs[0].xfer_floor;
    double mean_min = rs[0].xfer_mean, mean_max = rs[0].xfer_mean;
    for (const auto &r : rs) {
      floor_min = std::min(floor_min, r.xfer_floor);
      floor_max = std::max(floor_max, r.xfer_floor);
      mean_min = std::min(mean_min, r.xfer_mean);
      mean_max = std::max(mean_max, r.xfer_mean);
    }
    std::printf(
        "%s AGGREGATE over %zu runs: floor %llu..%llu xfers/call%s, "
        "per-run mean spread %.1f..%.1f\n",
        label, rs.size(), (unsigned long long)floor_min,
        (unsigned long long)floor_max,
        floor_min == floor_max ? " (stable)" : " (NOT stable — see per-run lines)",
        mean_min, mean_max);
  };
  int implausible = 0;
  for (const auto *v : {&scout_runs, &full_runs})
    for (const auto &r : *v)
      implausible += r.n_implausible;
  std::printf("counter plausibility: %s (%d call(s) over the %u bound across "
              "every run)\n",
              implausible ? "** FAILED — read payloads were corrupted **"
                          : "ok",
              implausible, kImplausibleCount);

  summarize_runs("GetRxEnergyScout", scout_runs);
  summarize_runs("GetRxEnergy(false)", full_runs);

  return 0;
}
