/* Headless guard for the pre-open device probe (src/DeviceProbe.h): the
 * question "would CreateRadio know what to do with this device?" asked
 * BEFORE claiming or resetting it, so a multi-card host can pick its radios
 * out of a bus without disturbing anything else plugged into it.
 *
 * The dispatch encoded here must stay in step with WiFiDriver::CreateRadio
 * -- same ids, same order (PID-gated generations first, then SYS_CFG2). This
 * selftest is what fails when the two drift. */
#include <cstdio>

#include "DeviceProbe.h"

static int g_fail = 0;

static void expect(const char *what, bool ok) {
  if (ok)
    return;
  ++g_fail;
  std::printf("FAIL: %s\n", what);
}

int main() {
  using namespace devourer;

  /* --- SYS_CFG2 (0x00FC) chip-id -> generation, mirroring the factory --- */
  expect("0x04 = 8812A -> Jaguar1",
         generation_for_chip_id(0x04) == ChipGeneration::Jaguar1);
  expect("0x05 = 8821A -> Jaguar1",
         generation_for_chip_id(0x05) == ChipGeneration::Jaguar1);
  expect("0x08 = 8814A -> Jaguar1",
         generation_for_chip_id(0x08) == ChipGeneration::Jaguar1);
  expect("0x09 = 8821C -> Jaguar2 (NOT Jaguar1, despite the name)",
         generation_for_chip_id(0x09) == ChipGeneration::Jaguar2);
  expect("0x0a = 8822B -> Jaguar2",
         generation_for_chip_id(0x0a) == ChipGeneration::Jaguar2);
  expect("0x50 = 8822B cold-boot transient -> Jaguar2",
         generation_for_chip_id(0x50) == ChipGeneration::Jaguar2);
  expect("0x13 = 8822C -> Jaguar3",
         generation_for_chip_id(0x13) == ChipGeneration::Jaguar3);
  expect("0x17 = 8822E (RTL8812EU) -> Jaguar3",
         generation_for_chip_id(0x17) == ChipGeneration::Jaguar3);
  expect("0x16 = 8733B -> Rtl8733b",
         generation_for_chip_id(0x16) == ChipGeneration::Rtl8733b);

  /* A failed control transfer reads 0, and a non-Realtek device that answers
   * the vendor request with junk must not be adopted. The factory's Jaguar1
   * fallback is deliberately NOT reproduced here: this predicate answers
   * "recognised", and an unrecognised byte is not a radio. */
  expect("0x00 (failed/STALLed read) -> Unknown",
         generation_for_chip_id(0x00) == ChipGeneration::Unknown);
  expect("0xff -> Unknown", generation_for_chip_id(0xff) == ChipGeneration::Unknown);
  expect("0x42 -> Unknown", generation_for_chip_id(0x42) == ChipGeneration::Unknown);

  /* --- PID-gated generations, decided before any register read --- */
  expect("0bda:b832 (RTL8852B) -> Kestrel by USB id",
         generation_for_usb_id(0x0bda, 0xb832) == ChipGeneration::Kestrel);
  expect("35bc:0101 (Archer TX50UH, RTL8852C) -> Kestrel by USB id",
         generation_for_usb_id(0x35bc, 0x0101) == ChipGeneration::Kestrel);
  expect("0e8d:7612 (MT7612U) -> Mt7612u by USB id",
         generation_for_usb_id(0x0e8d, 0x7612) == ChipGeneration::Mt7612u);
  expect("0e8d:7612 (MT7612U) is a probe candidate",
         is_probe_candidate(0x0e8d, 0x7612));
  expect("0bda:a81a (RTL8812EU) is not PID-dispatchable -> Unknown",
         generation_for_usb_id(0x0bda, 0xa81a) == ChipGeneration::Unknown);

  /* --- candidate gate: which devices are worth one control transfer --- */
  expect("any Realtek VID is a candidate",
         is_probe_candidate(0x0bda, 0xa81a) && is_probe_candidate(0x0bda, 0x8812));
  expect("a seller-branded Kestrel is a candidate despite a foreign VID",
         is_probe_candidate(0x35bc, 0x0101));
  expect("an unrelated vendor is left alone",
         !is_probe_candidate(0x046d, 0xc52b));

  if (g_fail) {
    std::printf("%d FAILURE(S)\n", g_fail);
    return 1;
  }
  std::printf("device_probe selftest OK\n");
  return 0;
}
