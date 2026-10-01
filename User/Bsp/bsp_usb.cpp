#include "bsp_usb.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "external_flash.h"
#include "stm32h7xx_hal.h" // IWYU pragma: keep
#include "tusb.h"          // IWYU pragma: keep

#include <string.h>

#define USB_VID 0xCAFEU
#if DM_MC02_USB_CLASS_CDC
#  define USB_PID 0x4001U
#else
#  define USB_PID 0x4002U
#endif
#define USB_BCD 0x0200U

#define EPNUM_CDC_NOTIF 0x81U
#define EPNUM_CDC_OUT 0x02U
#define EPNUM_CDC_IN 0x82U
#define EPNUM_HID_OUT 0x01U
#define EPNUM_HID_IN 0x81U

#define CDC_RX_POLL_CHUNK 64U
#define USB_PROVISION_REQUEST_VALUE 0x55535031UL /* "USP1" */
#if DM_MC02_USB_CLASS_CDC
#  define USB_XIP_MAGIC_VALUE 0x55424331UL /* "UBC1": USB CDC payload v1 */
#else
#  define USB_XIP_MAGIC_VALUE 0x55424831UL /* "UBH1": USB HID payload v1 */
#endif
#define USB_XIP_TEXT __attribute__((section(".usb_xip_text"), noinline))
#define USB_XIP_RODATA __attribute__((section(".usb_xip_rodata"), used))

#if DM_MC02_USB_CLASS_CDC
#  define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)
#else
#  define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN)
#endif

extern "C" __attribute__((section(".usb_xip_header"), used)) volatile const uint32_t dm_mc02_usb_xip_magic = USB_XIP_MAGIC_VALUE;

/* OpenOCD writes this NOLOAD mailbox before starting the provisioning boot. */
extern "C"
{
  __attribute__((section(".usb_provision_mailbox"), used)) volatile uint32_t dm_mc02_usb_provision_request = 0U;
}

/** @brief USB OTG HS 中断入口，转交 TinyUSB DCD 处理。 */
extern "C" void OTG_HS_IRQHandler(void)
{
  tud_int_handler(0);
}

enum
{
#if DM_MC02_USB_CLASS_CDC
  ITF_NUM_CDC = 0,
  ITF_NUM_CDC_DATA,
#else
  ITF_NUM_HID = 0,
#endif
  ITF_NUM_TOTAL
};

BspUsb &BspUsb::instance()
{
  static BspUsb s_instance;
  return s_instance;
}

void BspUsb::init()
{
  ext_flash_info_t info = {};
  const bool       provision_requested =
    dm_mc02_usb_provision_request == USB_PROVISION_REQUEST_VALUE;

  dm_mc02_usb_provision_request = 0U;
  __DSB();

  _initialized = false;
  if (ext_flash_init(&info) != EXT_FLASH_OK)
  {
    return;
  }
  if (provision_requested)
  {
    if (ext_flash_memory_mapped_spi_enable() == EXT_FLASH_OK)
    {
      (void)ext_flash_memory_mapped_access_enable(0);
    }
    return;
  }
  if (ext_flash_memory_mapped_enable() != EXT_FLASH_OK)
  {
    return;
  }
  if (ext_flash_memory_mapped_access_enable(1) != EXT_FLASH_OK)
  {
    (void)ext_flash_memory_mapped_disable();
    return;
  }

  SCB_InvalidateICache();
  __DSB();
  __ISB();
  if (dm_mc02_usb_xip_magic != USB_XIP_MAGIC_VALUE)
  {
    ext_flash_memory_mapped_access_disable();
    (void)ext_flash_memory_mapped_disable();
    if (ext_flash_memory_mapped_spi_enable() == EXT_FLASH_OK)
    {
      (void)ext_flash_memory_mapped_access_enable(0);
    }
    return;
  }

  HAL_NVIC_SetPriority(OTG_HS_IRQn,
                       configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
                       0U);
  _initialized = tud_init(0);
}

void BspUsb::task()
{
  if (!_initialized)
  {
    return;
  }
  tud_task_ext(0, false);
  _process_rx();
}

BspUsb::DeviceClass BspUsb::active_class() const
{
#if DM_MC02_USB_CLASS_CDC
  return DeviceClass::CDC;
#else
  return DeviceClass::HID;
#endif
}

bool BspUsb::initialized() const
{
  return _initialized;
}

bool BspUsb::is_ready() const
{
  if (!_initialized || !tud_mounted())
  {
    return false;
  }

#if DM_MC02_USB_CLASS_CDC
  if (_require_dtr)
  {
    return tud_cdc_connected();
  }
#else
  if (!tud_hid_ready())
  {
    return false;
  }
#endif
  return true;
}

void BspUsb::set_require_dtr(bool enable)
{
  _require_dtr = enable;
}

bool BspUsb::mounted() const
{
  return _initialized && tud_mounted();
}

bool BspUsb::cdc_write(const uint8_t *data, uint32_t len)
{
#if DM_MC02_USB_CLASS_CDC
  if ((data == nullptr) || (len == 0U) || !is_ready())
  {
    return false;
  }

  uint32_t written = tud_cdc_write(data, len);
  tud_cdc_write_flush();
  return written == len;
#else
  (void)data;
  (void)len;
  return false;
#endif
}

uint32_t BspUsb::cdc_read(uint8_t *data, uint32_t len)
{
#if DM_MC02_USB_CLASS_CDC
  if ((data == nullptr) || (len == 0U) || !mounted())
  {
    return 0U;
  }
  return tud_cdc_read(data, len);
#else
  (void)data;
  (void)len;
  return 0U;
#endif
}

uint32_t BspUsb::cdc_available() const
{
#if DM_MC02_USB_CLASS_CDC
  return mounted() ? tud_cdc_available() : 0U;
#else
  return 0U;
#endif
}

bool BspUsb::hid_write(const uint8_t *data, uint32_t len)
{
#if DM_MC02_USB_CLASS_HID
  if ((data == nullptr) || (len == 0U) || (len > HID_REPORT_SIZE) || !is_ready())
  {
    return false;
  }

  uint8_t report[HID_REPORT_SIZE] = {};
  memcpy(report, data, len);
  return tud_hid_report(0U, report, sizeof(report));
#else
  (void)data;
  (void)len;
  return false;
#endif
}

uint32_t BspUsb::hid_read(uint8_t *data, uint32_t len)
{
#if DM_MC02_USB_CLASS_HID
  if ((data == nullptr) || (len == 0U))
  {
    return 0U;
  }

  uint32_t read = 0U;
  while ((read < len) && (_hid_rx_tail != _hid_rx_head))
  {
    data[read++] = _hid_rx_buffer[_hid_rx_tail];
    _hid_rx_tail = static_cast<uint16_t>((_hid_rx_tail + 1U) % HID_RX_BUFFER_SIZE);
  }
  return read;
#else
  (void)data;
  (void)len;
  return 0U;
#endif
}

uint32_t BspUsb::hid_available() const
{
#if DM_MC02_USB_CLASS_HID
  const uint16_t head = _hid_rx_head;
  const uint16_t tail = _hid_rx_tail;
  return head >= tail ? static_cast<uint32_t>(head - tail) : static_cast<uint32_t>(HID_RX_BUFFER_SIZE - tail + head);
#else
  return 0U;
#endif
}

void BspUsb::accept_hid_report(const uint8_t *data, uint32_t len)
{
#if DM_MC02_USB_CLASS_HID
  if (data == nullptr)
  {
    return;
  }
  for (uint32_t index = 0U; index < len; ++index)
  {
    const uint16_t next = static_cast<uint16_t>((_hid_rx_head + 1U) % HID_RX_BUFFER_SIZE);
    if (next == _hid_rx_tail)
    {
      break;
    }
    _hid_rx_buffer[_hid_rx_head] = data[index];
    _hid_rx_head                 = next;
  }
#else
  (void)data;
  (void)len;
#endif
}

void BspUsb::set_rx_callback(RxCallback cb, void *user_ctx)
{
  _rx_callback = cb;
  _rx_user_ctx = user_ctx;
}

void BspUsb::_process_rx()
{
  if ((_rx_callback == nullptr) || !mounted())
  {
    return;
  }

  uint8_t rx_buf[CDC_RX_POLL_CHUNK];
#if DM_MC02_USB_CLASS_CDC
  while (tud_cdc_available() > 0U)
  {
    const uint32_t read_len = tud_cdc_read(rx_buf, sizeof(rx_buf));
    if (read_len == 0U)
    {
      break;
    }
    _rx_callback(rx_buf, read_len, _rx_user_ctx);
  }
#else
  while (hid_available() > 0U)
  {
    const uint32_t read_len = hid_read(rx_buf, sizeof(rx_buf));
    if (read_len == 0U)
    {
      break;
    }
    _rx_callback(rx_buf, read_len, _rx_user_ctx);
  }
#endif
}

#if DM_MC02_USB_CLASS_CDC
static USB_XIP_RODATA const tusb_desc_device_t desc_device = {
  sizeof(tusb_desc_device_t), TUSB_DESC_DEVICE, USB_BCD, TUSB_CLASS_MISC, MISC_SUBCLASS_COMMON, MISC_PROTOCOL_IAD, CFG_TUD_ENDPOINT0_SIZE, USB_VID, USB_PID, 0x0100U, 0x01U, 0x02U, 0x03U, 0x01U};

static USB_XIP_RODATA const uint8_t desc_configuration[] = {
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0, 100),
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, CFG_TUD_CDC_EP_BUFSIZE)};
#else
static USB_XIP_RODATA const tusb_desc_device_t desc_device = {
  sizeof(tusb_desc_device_t), TUSB_DESC_DEVICE, USB_BCD, 0x00U, 0x00U, 0x00U, CFG_TUD_ENDPOINT0_SIZE, USB_VID, USB_PID, 0x0100U, 0x01U, 0x02U, 0x03U, 0x01U};

static USB_XIP_RODATA const uint8_t desc_hid_report[] = {
  TUD_HID_REPORT_DESC_GENERIC_INOUT(BspUsb::HID_REPORT_SIZE)};

static USB_XIP_RODATA const uint8_t desc_configuration[] = {
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0, 100),
  TUD_HID_INOUT_DESCRIPTOR(ITF_NUM_HID, 4, HID_ITF_PROTOCOL_NONE, sizeof(desc_hid_report), EPNUM_HID_OUT, EPNUM_HID_IN, CFG_TUD_HID_EP_BUFSIZE, 1)};
#endif

extern "C" USB_XIP_TEXT uint8_t const *tud_descriptor_device_cb(void)
{
  return reinterpret_cast<uint8_t const *>(&desc_device);
}

extern "C" USB_XIP_TEXT uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
  (void)index;
  return desc_configuration;
}

#if DM_MC02_USB_CLASS_HID
extern "C" USB_XIP_TEXT uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
  (void)instance;
  return desc_hid_report;
}

extern "C" USB_XIP_TEXT uint16_t tud_hid_get_report_cb(
  uint8_t           instance,
  uint8_t           report_id,
  hid_report_type_t report_type,
  uint8_t          *buffer,
  uint16_t          reqlen)
{
  (void)instance;
  (void)report_id;
  (void)report_type;
  (void)buffer;
  (void)reqlen;
  return 0U;
}

extern "C" USB_XIP_TEXT void tud_hid_set_report_cb(
  uint8_t           instance,
  uint8_t           report_id,
  hid_report_type_t report_type,
  uint8_t const    *buffer,
  uint16_t          bufsize)
{
  (void)instance;
  (void)report_id;
  if (report_type == HID_REPORT_TYPE_OUTPUT)
  {
    BspUsb::instance().accept_hid_report(buffer, bufsize);
  }
}
#endif

enum
{
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
#if DM_MC02_USB_CLASS_CDC
  STRID_INTERFACE
#endif
};

static USB_XIP_RODATA const char string_langid[]       = {0x09, 0x04};
static USB_XIP_RODATA const char string_manufacturer[] = "RoboMaster";
#if DM_MC02_USB_CLASS_CDC
static USB_XIP_RODATA const char string_product[]   = "TY H723 TinyUSB CDC";
static USB_XIP_RODATA const char string_interface[] = "TinyUSB CDC";
#else
static USB_XIP_RODATA const char string_product[] = "TY H723 TinyUSB HID";
#endif
static USB_XIP_RODATA const char string_serial[] = "0001";

static USB_XIP_RODATA const char *const string_desc_arr[] = {
  string_langid,
  string_manufacturer,
  string_product,
  string_serial,
#if DM_MC02_USB_CLASS_CDC
  string_interface
#endif
};

static uint16_t desc_string[32];

extern "C" USB_XIP_TEXT uint16_t const *tud_descriptor_string_cb(uint8_t  index,
                                                                 uint16_t langid)
{
  (void)langid;
  uint8_t chr_count;

  if (index == STRID_LANGID)
  {
    desc_string[1] = 0x0409U;
    chr_count      = 1U;
  }
  else
  {
    if (index >= (sizeof(string_desc_arr) / sizeof(string_desc_arr[0])))
    {
      return nullptr;
    }
    const char *str = string_desc_arr[index];
    chr_count       = static_cast<uint8_t>(strlen(str));
    if (chr_count > 31U)
    {
      chr_count = 31U;
    }
    for (uint8_t i = 0U; i < chr_count; ++i)
    {
      desc_string[1U + i] = static_cast<uint16_t>(str[i]);
    }
  }

  desc_string[0] = static_cast<uint16_t>((TUSB_DESC_STRING << 8U) | (2U * chr_count + 2U));
  return desc_string;
}
