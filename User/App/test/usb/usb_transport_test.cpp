#include "app_test.hpp"

#if APP_TEST_USB_TRANSPORT_ENABLED

#  include "FreeRTOS.h"
#  include "task.h"

#  include <stdint.h>

#  include "bsp_cfg.hpp"

extern "C" void usb_transport_test_step(void)
{
  static TickType_t last_send_tick = xTaskGetTickCount();
#  if DM_MC02_USB_CLASS_HID
  static uint8_t  pending_echo[BspUsb::HID_REPORT_SIZE] = {};
  static uint32_t pending_echo_len                      = 0U;
#  endif
  const TickType_t now = xTaskGetTickCount();

  if (!bsp_usb.initialized())
  {
    return;
  }

#  if DM_MC02_USB_CLASS_HID
  if ((pending_echo_len == 0U) && (bsp_usb.hid_available() > 0U))
  {
    pending_echo_len = bsp_usb.hid_read(pending_echo, sizeof(pending_echo));
  }
  if (pending_echo_len > 0U)
  {
    if (bsp_usb.hid_write(pending_echo, pending_echo_len))
    {
      pending_echo_len = 0U;
    }
    return;
  }
#  endif

  if ((now - last_send_tick) < pdMS_TO_TICKS(1000U))
  {
    return;
  }
  last_send_tick = now;

#  if DM_MC02_USB_CLASS_CDC
  static const uint8_t message[] = "DM_MC02 TinyUSB CDC OK\r\n";
  (void)bsp_usb.cdc_write(message, sizeof(message) - 1U);
#  else
  static const uint8_t message[] = "DM_MC02 TinyUSB HID OK\r\n";
  (void)bsp_usb.hid_write(message, sizeof(message) - 1U);
#  endif
}

#endif
