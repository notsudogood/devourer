#include "DeviceProbe.h"

#include <libusb.h>

#include "UsbTransport.h" /* REALTEK_USB_VENQT_READ, USB_TIMEOUT */

namespace devourer {

ChipGeneration probe_generation(libusb_device *dev) {
  if (dev == nullptr)
    return ChipGeneration::Unknown;
  libusb_device_descriptor desc{};
  if (libusb_get_device_descriptor(dev, &desc) != 0)
    return ChipGeneration::Unknown;
  if (!is_probe_candidate(desc.idVendor, desc.idProduct))
    return ChipGeneration::Unknown;

  /* PID-dispatched generations answer without touching the device. */
  const ChipGeneration by_usb_id =
      generation_for_usb_id(desc.idVendor, desc.idProduct);
  if (by_usb_id != ChipGeneration::Unknown)
    return by_usb_id;

  /* One device-recipient vendor read of SYS_CFG2. No interface claim (this
   * recipient does not need one, and claiming would fight whatever driver is
   * bound), and emphatically no reset: a device that is not ours must be
   * left as it was found. A non-Realtek responder STALLs -> LIBUSB_ERROR_PIPE
   * -> Unknown, which is the ordinary "not a radio" path, not an error. */
  libusb_device_handle *h = nullptr;
  if (libusb_open(dev, &h) != 0 || h == nullptr)
    return ChipGeneration::Unknown;
  uint8_t chip_id = 0;
  const int rc =
      libusb_control_transfer(h, REALTEK_USB_VENQT_READ, 5, 0x00FC, 0,
                              &chip_id, sizeof(chip_id), USB_TIMEOUT);
  libusb_close(h);
  if (rc != static_cast<int>(sizeof(chip_id)))
    return ChipGeneration::Unknown;
  return generation_for_chip_id(chip_id);
}

} /* namespace devourer */
