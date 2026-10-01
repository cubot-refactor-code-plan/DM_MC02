#include "external_flash_xip.h"

#include <stddef.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

/* 由 STM32H723xG_flash.ld 导出：VMA 起止地址和内部 Flash 加载镜像地址。 */
extern const uint8_t __qspi_text_start__[];
extern const uint8_t __qspi_text_end__[];
extern const uint8_t __qspi_text_load_start__[];

/**
 * @brief 实际放入 `.qspi_text` 并从外置 Flash 执行的无依赖测试函数。
 * @param[in] value 用于验证参数传递和代码执行的输入值。
 * @return 确定性变换结果，测试输入 0x12345678 对应 0xB8B45189。
 */
__attribute__((section(".qspi_text"), noinline, used)) static uint32_t ext_flash_xip_function(uint32_t value)
{
  return (value ^ 0xA5A55A5AU) + 0x01234567U;
}

/**
 * @brief 擦除镜像覆盖的全部扇区并把内部加载镜像写入外置 Flash。
 * @param[in] flash_offset 镜像在 W25Q64JV 内部的起始偏移。
 * @param[in] image 内部 Flash 中的加载镜像。
 * @param[in] image_size 镜像字节数。
 * @return EXT_FLASH_OK 或首个擦写错误。
 */
static ext_flash_result_t ext_flash_program_xip_image(uint32_t       flash_offset,
                                                      const uint8_t *image,
                                                      uint32_t       image_size)
{
  uint32_t erase_start = flash_offset & ~(FLASH_DEVICE_SECTOR_SIZE - 1U);
  uint32_t erase_end   = (flash_offset + image_size + FLASH_DEVICE_SECTOR_SIZE - 1U) & ~(FLASH_DEVICE_SECTOR_SIZE - 1U);

  for (uint32_t address = erase_start; address < erase_end;
       address += FLASH_DEVICE_SECTOR_SIZE)
  {
    ext_flash_result_t result = ext_flash_erase_sector(address);
    if (result != EXT_FLASH_OK)
    {
      return result;
    }
  }
  return ext_flash_write(flash_offset, image, image_size);
}

ext_flash_result_t ext_flash_run_xip_test(ext_flash_xip_report_t *report)
{
  typedef uint32_t (*xip_function_t)(uint32_t);
  uintptr_t          mapped_start = (uintptr_t)__qspi_text_start__;
  uint32_t           image_size   = (uint32_t)(__qspi_text_end__ - __qspi_text_start__);
  uint32_t           flash_offset = (uint32_t)(mapped_start - FLASH_DEVICE_MAPPED_BASE);
  ext_flash_result_t result;

  if (report == NULL || image_size == 0U || (flash_offset + image_size) > FLASH_DEVICE_SIZE)
  {
    return EXT_FLASH_ERROR_IO;
  }
  *report = (ext_flash_xip_report_t) {
    .result           = EXT_FLASH_ERROR_IO,
    .flash_offset     = flash_offset,
    .image_size       = image_size,
    .function_address = (uint32_t)(uintptr_t)ext_flash_xip_function,
    .argument         = 0x12345678U,
    .expected         = 0xB8B45189U};

  result = ext_flash_program_xip_image(flash_offset,
                                       __qspi_text_load_start__,
                                       image_size);
  if (result != EXT_FLASH_OK)
  {
    report->result = result;
    return result;
  }
  result = ext_flash_memory_mapped_enable();
  if (result != EXT_FLASH_OK)
  {
    report->result = result;
    return result;
  }
  result = ext_flash_memory_mapped_access_enable(1);
  if (result != EXT_FLASH_OK)
  {
    (void)ext_flash_memory_mapped_disable();
    report->result = result;
    return result;
  }

  SCB_InvalidateICache();
  __DSB();
  __ISB();
  xip_function_t function = ext_flash_xip_function;
  report->actual          = function(report->argument);
  report->result          = report->actual == report->expected ? EXT_FLASH_OK : EXT_FLASH_ERROR_IO;

  ext_flash_memory_mapped_access_disable();
  if (ext_flash_memory_mapped_disable() != EXT_FLASH_OK)
  {
    report->result = EXT_FLASH_ERROR_IO;
  }
  return report->result;
}
