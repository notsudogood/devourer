#ifndef DEVOURER_DEVICE_PROBE_H
#define DEVOURER_DEVICE_PROBE_H

#include <cstdint>

struct libusb_device;

#include "AdapterCaps.h"
#include "kestrel/KestrelUsbIds.h"
#include "mt7612u/Mt7612uUsbIds.h"
#include "rtl8733b/Rtl8733bUsbIds.h"

/* Pre-open device probe: "would CreateRadio know what to do with this?"
 * answered WITHOUT claiming the interface or resetting the device.
 *
 * Motivation: a host with several adapters (a diversity ground station) has to
 * decide which USB devices are its radios. Doing that by opening each one the
 * normal way is destructive -- devourer::claim_interface_then_reset detaches
 * the kernel driver and resets the port, which is not something to do to a
 * card reader that happens to share the Realtek VID. This path costs one
 * device-recipient vendor control read, which an unrelated device either STALLs
 * or leaves unanswered until USB_TIMEOUT (500 ms; the MT7612U does the latter,
 * see WiFiDriver.cpp) — slow in that case, but otherwise untouched.
 *
 * The dispatch below MUST mirror WiFiDriver::CreateRadio: same ids, same
 * order (PID-gated generations first -- on AX silicon 0x00FC is a different
 * register -- then the SYS_CFG2 byte). tests/device_probe_selftest.cpp is the
 * guard against the two drifting.
 *
 * One deliberate difference: the factory falls back to Jaguar1 for an
 * unrecognised chip-id, which is right when a caller has already decided this
 * device IS its radio. A probe asking "is this a radio at all" must not adopt
 * every byte it doesn't recognise, so Unknown means unknown here. */
namespace devourer {

/* SYS_CFG2 (0x00FC) chip-id -> generation. Unknown for a byte no backend
 * claims, including 0 (the value a failed or STALLed control read leaves). */
inline ChipGeneration generation_for_chip_id(uint8_t chip_id) {
  switch (chip_id) {
  case 0x04: /* 8812A */
  case 0x05: /* 8821A */
  case 0x08: /* 8814A */
    return ChipGeneration::Jaguar1;
  case 0x09: /* 8821C -- a HalMAC/phydm Jaguar2 chip despite the name */
  case 0x0a: /* 8822B */
  case 0x50: /* 8822B, cold-VBUS transient before the chip settles */
    return ChipGeneration::Jaguar2;
  case 0x13: /* 8822C */
  case 0x17: /* 8822E = RTL8812EU / RTL8822EU */
    return ChipGeneration::Jaguar3;
  case rtl8733b::kChipId: /* 0x16 */
    return ChipGeneration::Rtl8733b;
  default:
    return ChipGeneration::Unknown;
  }
}

/* Generations dispatched by USB id rather than by register read. The MT7612U
 * is a MediaTek part (no Realtek register map at all), checked first as the
 * factory does. Kestrel (11ax) is here because 0x00FC is R_AX_SYS_CHIPINFO on
 * that silicon and the 8852A die-id collides with the 8822B cold transient
 * above. */
inline ChipGeneration generation_for_usb_id(uint16_t vid, uint16_t pid) {
  if (mt7612u::is_usb_id(vid, pid))
    return ChipGeneration::Mt7612u;
  if (kestrel::variant_for_usb_id(vid, pid))
    return ChipGeneration::Kestrel;
  return ChipGeneration::Unknown;
}

/* Is this device worth one control transfer? Every Realtek-VID device is (the
 * 11ac families are only distinguishable by the register read), plus the
 * seller-branded ids in the PID tables, whose VIDs are not Realtek's. Anything
 * else is somebody else's hardware and is never addressed. */
inline bool is_probe_candidate(uint16_t vid, uint16_t pid) {
  constexpr uint16_t kRealtekVid = 0x0bda;
  return vid == kRealtekVid || generation_for_usb_id(vid, pid) != ChipGeneration::Unknown ||
         rtl8733b::is_usb_id(vid, pid);
}

/* Bus half: identify the chip behind `dev` without disturbing it. Costs one
 * control transfer for an 11ac candidate and nothing at all for a PID-
 * dispatched one. Unknown = not a devourer-supported radio (including "the
 * device refused to answer"), which is the answer a scanner wants. Thin on
 * purpose: every decision it makes lives in the pure functions above, which
 * the selftest covers. */
ChipGeneration probe_generation(libusb_device *dev);

} /* namespace devourer */

#endif /* DEVOURER_DEVICE_PROBE_H */
