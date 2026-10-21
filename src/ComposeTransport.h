#ifndef DEVOURER_COMPOSE_TRANSPORT_H
#define DEVOURER_COMPOSE_TRANSPORT_H

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Transport.h"

namespace devourer {

/* An ITransport decorator that turns a sequence of blocking register
 * read-modify-writes into composed, batched transfers -- without touching the
 * code that issues them.
 *
 * A register path written as phy_set_bb_reg calls (read32 + write32 each) pays
 * a full EP0 round trip per field. Run the same code over this transport and:
 *   - 32-bit writes are queued as CtrlOps (in order) and also update a dword
 *     cache, so a later read-modify-write of the same register composes from
 *     what was just written instead of reading the chip again;
 *   - 32-bit reads of PRIMED registers (prime(): one ctrl_batch of reads at
 *     the start) are served from the cache;
 *   - anything else -- an unprimed read, an 8/16-bit access, write_bytes, the
 *     wide window, a bulk transfer -- first FLUSHES the queue, then goes to the
 *     real transport, so ordering against the queued writes is exactly the
 *     plain path's. An unprimed 32-bit read is cached afterwards and reported
 *     by missed_reads(), so a caller can prime it next time.
 *   - reads of `uncached_reads` addresses (write-confirm loops, table data
 *     ports whose readback is not the last value written) always flush and
 *     read the chip; writes to them still queue.
 *
 * The write SEQUENCE reaching the chip is identical to the plain path's (same
 * addresses, same values, same order) as long as every cached register reads
 * back what was last written to it and nothing else writes it meanwhile --
 * true for configuration registers under the caller's device register lock.
 * tests/compose_transport_selftest.cpp pins the transport; the Jaguar3
 * channel-set parity case pins the write sequence against the plain path.
 *
 * Failure: prime() reports a failed read and caches NOTHING -- a composed
 * dword built on a read that never happened is exactly what deafens a
 * receiver (tests/scout_batch_fail_selftest.cpp). A failed write batch makes
 * finish() return false; the chip may then be half-configured and the caller
 * must resync through its plain path. Single-threaded: one instance per
 * register sequence, used under the caller's device register lock. */
class ComposeTransport final : public ITransport {
public:
  explicit ComposeTransport(std::shared_ptr<ITransport> real,
                            std::set<uint16_t> uncached_reads = {})
      : _real(std::move(real)), _uncached(std::move(uncached_reads)) {}

  /* One batched read of `addrs` into the cache. False (and nothing cached)
   * when any read failed. */
  bool prime(const std::vector<uint16_t> &addrs) {
    const auto t0 = std::chrono::steady_clock::now();
    struct Acc {
      uint64_t &us;
      std::chrono::steady_clock::time_point t0;
      ~Acc() { us += since(t0); }
    } acc{_prime_us, t0};
    std::vector<CtrlOp> ops;
    ops.reserve(addrs.size());
    for (uint16_t a : addrs)
      if (!_uncached.count(a) && !_cache.count(a))
        ops.push_back(CtrlOp{false, a, 0});
    if (ops.empty())
      return true;
    if (!_real->ctrl_batch(ops))
      return false;
    for (const CtrlOp &op : ops)
      _cache[op.addr] = op.value;
    return true;
  }

  /* Flushes what is queued. True when every batch (and every pass-through
   * write) since construction succeeded. */
  bool finish() {
    flush();
    return !_failed;
  }

  /* 32-bit reads that missed the cache and went to the chip (excluding the
   * uncached set), in first-miss order: what to prime next time. */
  const std::vector<uint16_t> &missed_reads() const { return _missed; }
  /* Register transfers that waited on their own round trip (pass-through
   * reads/writes), and ops that went out in batches. */
  uint64_t passthrough_ops() const { return _passthrough; }
  uint64_t batched_ops() const { return _batched; }
  /* Wall-clock spent in the priming read, in write batches and in the
   * transfers that waited on their own round trip -- what a switch costs
   * on real hardware, where the round-trip count alone does not say. */
  uint64_t prime_us() const { return _prime_us; }
  uint64_t flush_us() const { return _flush_us; }
  uint64_t flushes() const { return _flushes; }
  uint64_t passthrough_us() const { return _pass_us; }

  /* ---- register plane ---- */
  uint32_t read32(uint16_t reg) override {
    if (!_uncached.count(reg)) {
      auto it = _cache.find(reg);
      if (it != _cache.end())
        return it->second;
    }
    flush();
    ++_passthrough;
    const auto t0 = std::chrono::steady_clock::now();
    const uint32_t v = _real->read32(reg);
    _pass_us += since(t0);
    if (!_uncached.count(reg)) {
      _cache[reg] = v;
      _missed.push_back(reg);
    }
    return v;
  }
  bool write32(uint16_t reg, uint32_t v) override {
    _cache[reg] = v;
    _ops.push_back(CtrlOp{true, reg, v});
    if (_ops.size() >= kMaxQueued)
      flush();
    return true;
  }
  uint8_t read8(uint16_t reg) override {
    flush();
    ++_passthrough;
    const auto t0 = std::chrono::steady_clock::now();
    const uint8_t v = _real->read8(reg);
    _pass_us += since(t0);
    return v;
  }
  uint16_t read16(uint16_t reg) override {
    flush();
    ++_passthrough;
    const auto t0 = std::chrono::steady_clock::now();
    const uint16_t v = _real->read16(reg);
    _pass_us += since(t0);
    return v;
  }
  bool write8(uint16_t reg, uint8_t v) override {
    flush();
    forget(reg, 1);
    const auto t0 = std::chrono::steady_clock::now();
    const bool ok = _real->write8(reg, v);
    _pass_us += since(t0);
    return pass(ok);
  }
  bool write16(uint16_t reg, uint16_t v) override {
    flush();
    forget(reg, 2);
    const auto t0 = std::chrono::steady_clock::now();
    const bool ok = _real->write16(reg, v);
    _pass_us += since(t0);
    return pass(ok);
  }
  bool write_bytes(uint16_t reg, const uint8_t *p, size_t n) override {
    flush();
    forget(reg, n);
    return pass(_real->write_bytes(reg, p, n));
  }
  bool write32_wide(uint32_t addr, uint32_t v) override {
    flush();
    if (addr < 0x10000)
      forget(static_cast<uint16_t>(addr), 4);
    return pass(_real->write32_wide(addr, v));
  }
  uint32_t read32_wide(uint32_t addr) override {
    flush();
    ++_passthrough;
    return _real->read32_wide(addr);
  }
  bool ctrl_batch(std::vector<CtrlOp> &ops) override {
    flush();
    for (const CtrlOp &op : ops)
      if (op.write)
        forget(op.addr, 4);
    const bool ok = _real->ctrl_batch(ops);
    _failed = _failed || !ok;
    return ok;
  }

  /* ---- everything else: flush, then the real transport ---- */
  bool is_usb() const override { return _real->is_usb(); }
  void write_batch_begin() override {
    flush();
    _real->write_batch_begin();
  }
  bool write_batch_end() override {
    flush();
    return _real->write_batch_end();
  }
  void flush_writes() override {
    flush();
    _real->flush_writes();
  }
  uint64_t ctrl_xfers() const override { return _real->ctrl_xfers(); }
  bool tx_async(uint8_t ep, uint8_t *buf, size_t len, unsigned timeout_ms) override {
    flush();
    return _real->tx_async(ep, buf, len, timeout_ms);
  }
  int tx_sync(uint8_t ep, uint8_t *buf, size_t len, int timeout_ms) override {
    flush();
    return _real->tx_sync(ep, buf, len, timeout_ms);
  }
  int tx_sync_data(uint8_t ep, uint8_t *buf, size_t len, int timeout_ms) override {
    flush();
    return _real->tx_sync_data(ep, buf, len, timeout_ms);
  }
  void rx_loop(int buf_size, int n_xfers,
               const std::function<void(const uint8_t *, int)> &on_data,
               const std::function<bool()> &should_stop) override {
    flush();
    _real->rx_loop(buf_size, n_xfers, on_data, should_stop);
  }
  int rx_raw(uint8_t *buf, int len, int timeout_ms) override {
    flush();
    return _real->rx_raw(buf, len, timeout_ms);
  }
  void clear_halt(uint8_t ep) override {
    flush();
    _real->clear_halt(ep);
  }
  void quiesce_tx() override {
    flush();
    _real->quiesce_tx();
  }
  void hci_setup() override {
    flush();
    _real->hci_setup();
  }
  UsbLinkInfo usb_info() const override { return _real->usb_info(); }
  TxStats tx_stats() const override { return _real->tx_stats(); }

private:
  static constexpr size_t kMaxQueued = 64;

  static uint64_t since(std::chrono::steady_clock::time_point t0) {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                     std::chrono::steady_clock::now() - t0)
                                     .count());
  }

  void flush() {
    if (_ops.empty())
      return;
    _batched += _ops.size();
    ++_flushes;
    const auto t0 = std::chrono::steady_clock::now();
    if (!_real->ctrl_batch(_ops))
      _failed = true;
    _flush_us += since(t0);
    _ops.clear();
  }
  bool pass(bool ok) {
    ++_passthrough;
    _failed = _failed || !ok;
    return ok;
  }
  /* A narrower or bulk write changed bytes of these dwords behind the cache. */
  void forget(uint16_t reg, size_t n) {
    const uint32_t first = reg & ~3u;
    const uint32_t last = (static_cast<uint32_t>(reg) + std::max<size_t>(n, 1) - 1) & ~3u;
    for (uint32_t a = first; a <= last && a <= 0xfffcu; a += 4)
      _cache.erase(static_cast<uint16_t>(a));
  }

  std::shared_ptr<ITransport> _real;
  std::set<uint16_t> _uncached;
  std::unordered_map<uint16_t, uint32_t> _cache;
  std::vector<CtrlOp> _ops;
  std::vector<uint16_t> _missed;
  uint64_t _passthrough = 0;
  uint64_t _batched = 0;
  uint64_t _prime_us = 0, _flush_us = 0, _flushes = 0, _pass_us = 0;
  bool _failed = false;
};

} // namespace devourer

#endif /* DEVOURER_COMPOSE_TRANSPORT_H */
