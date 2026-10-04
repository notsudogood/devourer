#pragma once

/* ITransport — the bus seam. USB (libusb) and PCIe (vfio) are independent
 * transports implementing this one interface; RtlAdapter (the copyable value
 * type every HAL holds) owns a shared_ptr to one of them and forwards. Nothing
 * here depends on libusb or vfio.
 *
 * Two planes, mirroring how the chips are built:
 *  - register plane: 8/16/32-bit accesses into the 0x0000..0xFFFF MAC/BB
 *    register space (USB: vendor control transfers; PCIe: BAR2 MMIO).
 *  - frame plane: TX submissions and the blocking RX delivery loop
 *    (USB: bulk endpoints; PCIe: 88xx buffer-descriptor DMA rings).
 *
 * hci_setup() is the rtw88-style pre-power hook: programming that must happen
 * before the power-on sequence each bring-up attempt (PCIe TRX ring
 * registers; nothing on USB). */

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "TxStats.h"

namespace devourer {

/* USB-descriptor-derived link facts consumed by the HALs (queue mapping,
 * RX-aggregation sizing). A PCIe transport returns the defaults. */
/* libusb_speed numeric values, mirrored so HAL code comparing link speed does
 * not need libusb.h (the whole point of the transport seam). */
constexpr int kUsbSpeedHigh = 3;  /* LIBUSB_SPEED_HIGH  (480 MBit/s) */
constexpr int kUsbSpeedSuper = 4; /* LIBUSB_SPEED_SUPER (5 GBit/s)   */

struct UsbLinkInfo {
  bool valid = false;    /* true only on the USB transport */
  int speed = 0;         /* libusb_speed numeric value */
  uint16_t vid = 0, pid = 0;
  uint8_t bulk_in_ep = 0x81;
  std::vector<uint8_t> bulk_out_eps; /* descriptor order */
};

/* ---- batched control transfers -------------------------------------------
 * One register access in a group submitted to ITransport::ctrl_batch.
 * `value` is the payload for a write and the RESULT slot for a read (filled
 * in place, in this op's own element — see the contract on ctrl_batch).
 * 16-bit MAC/BB space only; the wIndex=1 window is not batchable.
 * Namespace scope, not nested in ITransport: the selftest, RtlAdapter's
 * forwarder and every call site name it `devourer::CtrlOp`. */
struct CtrlOp {
  bool write;
  uint16_t addr;
  uint32_t value;
};

class ITransport {
public:
  virtual ~ITransport() = default;

  virtual bool is_usb() const = 0;

  /* ---- register plane ---- */
  virtual uint8_t read8(uint16_t reg) = 0;
  virtual uint16_t read16(uint16_t reg) = 0;
  virtual uint32_t read32(uint16_t reg) = 0;
  virtual bool write8(uint16_t reg, uint8_t v) = 0;
  virtual bool write16(uint16_t reg, uint16_t v) = 0;
  virtual bool write32(uint16_t reg, uint32_t v) = 0;
  virtual bool write_bytes(uint16_t reg, const uint8_t *p, size_t n) = 0;
  /* 32-bit *address* register write. Realtek splits the register address as
   * (wIndex << 16) | wValue over USB, so addresses >= 0x10000 (the halbb/halrf
   * BB register window at addr + 0x10000) need the high half in wIndex — the
   * plain 16-bit write forces wIndex=0 and would corrupt the MAC/system space.
   * The default forwards to the 16-bit path (correct for addr < 0x10000); the
   * USB transport overrides it to carry the full address. */
  virtual bool write32_wide(uint32_t addr, uint32_t v) {
    return write32(static_cast<uint16_t>(addr), v);
  }
  /* 32-bit-address register read (the read counterpart of write32_wide;
   * needed for masked BB read-modify-write and RF-window reads). */
  virtual uint32_t read32_wide(uint32_t addr) {
    return read32(static_cast<uint16_t>(addr));
  }

  /* ---- pipelined register writes ----
   * Inside a write batch, register writes are submitted as asynchronous
   * in-order transfers and only a read (or a bulk transfer, or flush_writes)
   * waits for them. The Jaguar3 bring-up is ~14k EP0 round trips; on the
   * one unit and host measured (an RTL8812EU on an ssc338q) a synchronous
   * write cost ~80 us and a pipelined one ~27 us at depth >= 8 — one device,
   * one host, so a scale rather than a number to plan by; the measured
   * bring-up figures and their limits are in src/jaguar3/CLAUDE.md.
   * Correctness rests on EP0 completing URBs in submission order, so a read
   * that follows a write still sees it. Single-threaded by contract: open a
   * batch only while no other thread touches the transport (the Jaguar3
   * InitWrite/Init bring-up), and close it before any worker thread starts.
   * write_batch_end drains and reports whether every queued write completed
   * (a failed or short completion is only known after the fact); the caller
   * decides what an incomplete batch means. Defaults are no-ops (PCIe). */
  virtual void write_batch_begin() {}
  virtual bool write_batch_end() { return true; }
  virtual void flush_writes() {}
  /* Register transfers (reads + writes) this transport instance has issued
   * so far — the unit a USB bring-up is paid in. InitTimer differences it
   * per stage. Per instance, never process-wide. 0 where the notion does
   * not apply (PCIe MMIO). */
  virtual uint64_t ctrl_xfers() const { return 0; }

  /* ---- batched control transfers ----
   * A group of EP0 register reads/writes submitted back to back, paying one
   * completion wait per chunk of at most the transport's slot depth instead
   * of one host round trip per op.
   *
   * CONTRACT, owed by every implementation:
   *   - ops execute in submission order, so a read after a write to the same
   *     address in one batch sees that write (on USB this is EP0's in-order
   *     completion, the property the pipelined write batch already rests on);
   *   - each read's value lands in ITS OWN op's `value`;
   *   - a write op's `value` is an input and comes back untouched;
   *   - returns false if an op failed, and a failure does NOT abort the rest
   *     of the batch (a half-issued hop leaves a 3-wire bracket open or a BB
   *     reset asserted — finishing and reporting beats bailing out). The
   *     default below reports write failures only; a failed read leaves by
   *     the transport's read idiom (exception on USB — read32 has no failure
   *     return). UsbTransport's own ctrl_batch reports a failed read as false.
   *     A read-then-compose-then-write caller must allow for both; see
   *     RtlJaguar3Device::GetRxEnergyScout.
   * tests/ctrl_batch_selftest.cpp pins all four against a fake transport.
   *
   * Serialized by the CALLER (it holds its device register lock for the
   * whole group; the transport additionally takes its own mutex so a bring-up
   * write_batch_* and a runtime ctrl_batch can never interleave over the same
   * slot pool). Default: synchronous, in order — correct on PCIe and on any
   * transport that has nothing to pipeline.
   *
   * HAZARD, USB: the implementation pumps libusb events from the CALLING
   * thread while the RX bulk pump may be doing the same on its own thread.
   * libusb permits that, but either thread may reap either thread's
   * completions, so an RX on_data callback can run on the caller's thread
   * inside this call, and vice versa. Hence: nothing reachable from on_data
   * may take a lock a ctrl_batch caller holds (the device register lock),
   * and the wait loop keys on the per-slot `done` flag the completion
   * callback sets — never on "I pumped, therefore mine finished".
   * See UsbTransport::ctrl_batch. */
  virtual bool ctrl_batch(std::vector<CtrlOp> &ops) {
    bool ok = true;
    for (CtrlOp &op : ops) {
      if (op.write)
        ok = write32(op.addr, op.value) && ok;
      else
        op.value = read32(op.addr);
    }
    return ok;
  }

  /* ---- frame plane ---- */
  /* Fire-and-forget data TX (the send_packet hot path). `ep` is the USB
   * bulk-OUT endpoint choice; the PCIe transport ignores it (the ring is
   * chosen from the descriptor's QSEL). */
  virtual bool tx_async(uint8_t ep, uint8_t *buf, size_t len,
                        unsigned timeout_ms) = 0;
  /* Synchronous TX that blocks until the transport confirms consumption
   * (USB: bulk completion; PCIe: hardware read-pointer / BCN kick). Returns
   * bytes submitted or a negative error. */
  virtual int tx_sync(uint8_t ep, uint8_t *buf, size_t len, int timeout_ms) = 0;
  /* tx_sync for a DATA frame (never firmware download or a reserved-page
   * write): the one bulk-OUT a transport may send with a relaxed timeout
   * policy (USB: DeviceConfig::Tx::no_cancel_multipkt). Default: tx_sync. */
  virtual int tx_sync_data(uint8_t ep, uint8_t *buf, size_t len,
                           int timeout_ms) {
    return tx_sync(ep, buf, len, timeout_ms);
  }
  /* Blocking RX delivery loop until should_stop(). buf_size/n_xfers are USB
   * URB-queue tuning; the PCIe ring depth is fixed at transport creation. */
  virtual void rx_loop(int buf_size, int n_xfers,
                       const std::function<void(const uint8_t *, int)> &on_data,
                       const std::function<bool()> &should_stop) = 0;
  /* Single-shot raw RX read (USB bulk-IN); unsupported (-1) on PCIe. */
  virtual int rx_raw(uint8_t *buf, int len, int timeout_ms) {
    (void)buf; (void)len; (void)timeout_ms;
    return -1;
  }
  virtual void clear_halt(uint8_t ep) { (void)ep; }
  /* Stop the TX engine and return only once nothing is in flight: cancel every
   * outstanding transfer, reap the completions, and refuse further sends. The
   * caller-owned bus context (libusb here) must still be alive, so this has to
   * run BEFORE any of it is torn down — which is exactly why it is an explicit
   * call and not destructor work. Idempotent. Default no-op: a transport whose
   * TX is synchronous has nothing outstanding by construction. */
  virtual void quiesce_tx() {}

  /* ---- lifecycle / info ---- */
  /* Pre-power-on HCI programming, re-run per bring-up attempt (rtw88's
   * rtw_hci_setup slot: PCIe TRX buffer-descriptor ring registers; no-op on
   * USB). */
  virtual void hci_setup() {}
  virtual UsbLinkInfo usb_info() const { return {}; }
  virtual TxStats tx_stats() const { return {}; }
};

} /* namespace devourer */
