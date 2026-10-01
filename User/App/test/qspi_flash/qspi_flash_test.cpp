#include "app_test.hpp"

#if APP_TEST_QSPI_FLASH_ENABLED

#  include "FreeRTOS.h"
#  include "task.h"

#  include <stdint.h>

#  include "external_flash.h"
#  include "external_flash_test.h"
#  include "external_flash_xip.h"

volatile uint32_t           qspi_flash_test_stage           = 0U;
volatile uint32_t           qspi_flash_test_passed          = 0U;
volatile ext_flash_result_t qspi_flash_test_init_result     = EXT_FLASH_ERROR_IO;
volatile ext_flash_result_t qspi_flash_test_data_result     = EXT_FLASH_ERROR_IO;
volatile ext_flash_result_t qspi_flash_test_mapped_result   = EXT_FLASH_ERROR_IO;
volatile ext_flash_result_t qspi_flash_test_xip_result      = EXT_FLASH_ERROR_IO;
volatile uint8_t            qspi_flash_test_manufacturer_id = 0U;
volatile uint8_t            qspi_flash_test_memory_type     = 0U;
volatile uint8_t            qspi_flash_test_capacity_id     = 0U;
volatile uint32_t           qspi_flash_test_mapped_mismatch = UINT32_MAX;
ext_flash_test_report_t     qspi_flash_test_data_report;
ext_flash_xip_report_t      qspi_flash_test_xip_report;

namespace
{
constexpr uint32_t mapped_test_address =
  FLASH_DEVICE_SIZE - FLASH_DEVICE_SECTOR_SIZE - 128U;
constexpr uint32_t mapped_test_length = 257U;

void fail(uint32_t stage)
{
  qspi_flash_test_stage = 0xE0U | stage;
  vTaskDelete(nullptr);
}
} // namespace

extern "C" void qspi_flash_test_task(void *argument)
{
  (void)argument;
  ext_flash_info_t        info        = {};
  ext_flash_test_report_t data_report = {};
  ext_flash_xip_report_t  xip_report  = {};

  vTaskDelay(pdMS_TO_TICKS(250U));

  qspi_flash_test_stage           = 1U;
  qspi_flash_test_init_result     = ext_flash_init(&info);
  qspi_flash_test_manufacturer_id = info.manufacturer_id;
  qspi_flash_test_memory_type     = info.memory_type;
  qspi_flash_test_capacity_id     = info.capacity_id;
  if (qspi_flash_test_init_result != EXT_FLASH_OK)
  {
    fail(1U);
    return;
  }

  qspi_flash_test_stage       = 2U;
  qspi_flash_test_data_result = ext_flash_run_data_tests(&data_report);
  qspi_flash_test_data_report = data_report;
  if (qspi_flash_test_data_result != EXT_FLASH_OK)
  {
    fail(2U);
    return;
  }

  qspi_flash_test_stage         = 3U;
  qspi_flash_test_mapped_result = ext_flash_memory_mapped_enable();
  if (qspi_flash_test_mapped_result == EXT_FLASH_OK)
  {
    qspi_flash_test_mapped_result = ext_flash_memory_mapped_access_enable(0);
  }
  if (qspi_flash_test_mapped_result != EXT_FLASH_OK)
  {
    fail(3U);
    return;
  }

  const volatile uint8_t *mapped = reinterpret_cast<const volatile uint8_t *>(
    FLASH_DEVICE_MAPPED_BASE + mapped_test_address);
  for (uint32_t index = 0U; index < mapped_test_length; ++index)
  {
    if (mapped[index] != static_cast<uint8_t>(index))
    {
      qspi_flash_test_mapped_mismatch = index;
      qspi_flash_test_mapped_result   = EXT_FLASH_ERROR_IO;
      break;
    }
  }
  ext_flash_memory_mapped_access_disable();
  if (ext_flash_memory_mapped_disable() != EXT_FLASH_OK)
  {
    qspi_flash_test_mapped_result = EXT_FLASH_ERROR_IO;
  }
  if (qspi_flash_test_mapped_result != EXT_FLASH_OK)
  {
    fail(4U);
    return;
  }

  qspi_flash_test_stage      = 4U;
  qspi_flash_test_xip_result = ext_flash_run_xip_test(&xip_report);
  qspi_flash_test_xip_report = xip_report;
  if (qspi_flash_test_xip_result != EXT_FLASH_OK)
  {
    fail(5U);
    return;
  }

  qspi_flash_test_passed = 1U;
  qspi_flash_test_stage  = 5U;
  vTaskDelete(nullptr);
}

#endif // APP_TEST_QSPI_FLASH_ENABLED
