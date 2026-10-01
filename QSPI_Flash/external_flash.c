#include "external_flash.h"

#include <stdint.h>

#include "flash_port_stm32h7.h"

/* 公共接口的使用方法和参数约束见 external_flash.h。 */

ext_flash_result_t ext_flash_init(ext_flash_info_t *info)
{
  flash_port_prepare();
  if (flash_device_reset() != 0 || flash_device_read_info(info) != 0)
  {
    return EXT_FLASH_ERROR_IO;
  }
  if (!flash_device_is_expected(info))
  {
    return EXT_FLASH_ERROR_DEVICE_ID;
  }
  return flash_device_enable_quad() == 0 ? EXT_FLASH_OK : EXT_FLASH_ERROR_IO;
}

ext_flash_result_t ext_flash_read_info(ext_flash_info_t *info)
{
  return flash_device_read_info(info) == 0 ? EXT_FLASH_OK : EXT_FLASH_ERROR_IO;
}

ext_flash_result_t ext_flash_write_enable(void)
{
  return flash_device_write_enable() == 0 ? EXT_FLASH_OK : EXT_FLASH_ERROR_IO;
}

ext_flash_result_t ext_flash_wait_ready(uint32_t timeout_ms)
{
  int result = flash_device_wait_ready(timeout_ms);

  if (result == -2)
  {
    return EXT_FLASH_ERROR_TIMEOUT;
  }
  return result == 0 ? EXT_FLASH_OK : EXT_FLASH_ERROR_IO;
}

/**
 * @brief 检查一个非空访问区间是否完全落在 8 MiB 器件内。
 * @param[in] address Flash 内部起始偏移。
 * @param[in] length 访问长度，0 被视为非法。
 * @return 合法返回非 0，否则返回 0。
 */
static int ext_flash_range_valid(uint32_t address, size_t length)
{
  return length > 0U && length <= FLASH_DEVICE_SIZE && address < FLASH_DEVICE_SIZE && length <= (FLASH_DEVICE_SIZE - address);
}

ext_flash_result_t ext_flash_read(uint32_t address, void *data, size_t length)
{
  if (data == NULL || !ext_flash_range_valid(address, length))
  {
    return EXT_FLASH_ERROR_IO;
  }
  return flash_device_read(address, data, (uint32_t)length) == 0 ? EXT_FLASH_OK : EXT_FLASH_ERROR_IO;
}

ext_flash_result_t ext_flash_write(uint32_t address, const void *data, size_t length)
{
  const uint8_t *source = data;

  if (data == NULL || !ext_flash_range_valid(address, length))
  {
    return EXT_FLASH_ERROR_IO;
  }
  while (length > 0U)
  {
    uint32_t page_remaining = FLASH_DEVICE_PAGE_SIZE - (address & (FLASH_DEVICE_PAGE_SIZE - 1U));
    uint32_t chunk          = length < page_remaining ? (uint32_t)length : page_remaining;

    if (flash_device_program_page(address, source, chunk) != 0)
    {
      return EXT_FLASH_ERROR_IO;
    }
    address += chunk;
    source += chunk;
    length -= chunk;
  }
  return EXT_FLASH_OK;
}

ext_flash_result_t ext_flash_erase_sector(uint32_t address)
{
  if (address >= FLASH_DEVICE_SIZE || (address & (FLASH_DEVICE_SECTOR_SIZE - 1U)) != 0U)
  {
    return EXT_FLASH_ERROR_IO;
  }
  return flash_device_erase_sector(address) == 0 ? EXT_FLASH_OK : EXT_FLASH_ERROR_IO;
}

ext_flash_result_t ext_flash_erase_chip(void)
{
  return flash_device_erase_chip() == 0 ? EXT_FLASH_OK : EXT_FLASH_ERROR_IO;
}

ext_flash_result_t ext_flash_read_mdma(uint32_t address, void *data, size_t length)
{
  if (data == NULL || !ext_flash_range_valid(address, length))
  {
    return EXT_FLASH_ERROR_IO;
  }
  return flash_device_read_mdma(address, data, (uint32_t)length) == 0 ? EXT_FLASH_OK : EXT_FLASH_ERROR_IO;
}

ext_flash_result_t ext_flash_write_mdma(uint32_t address, const void *data, size_t length)
{
  const uint8_t *source = data;

  if (data == NULL || !ext_flash_range_valid(address, length))
  {
    return EXT_FLASH_ERROR_IO;
  }
  while (length > 0U)
  {
    uint32_t page_remaining = FLASH_DEVICE_PAGE_SIZE - (address & (FLASH_DEVICE_PAGE_SIZE - 1U));
    uint32_t chunk          = length < page_remaining ? (uint32_t)length : page_remaining;

    if (flash_device_program_page_mdma(address, source, chunk) != 0)
    {
      return EXT_FLASH_ERROR_IO;
    }
    address += chunk;
    source += chunk;
    length -= chunk;
  }
  return EXT_FLASH_OK;
}

ext_flash_result_t ext_flash_memory_mapped_enable(void)
{
  return flash_device_memory_mapped_enable() == 0 ? EXT_FLASH_OK : EXT_FLASH_ERROR_IO;
}

ext_flash_result_t ext_flash_memory_mapped_spi_enable(void)
{
  return flash_device_memory_mapped_spi_enable() == 0 ? EXT_FLASH_OK : EXT_FLASH_ERROR_IO;
}

ext_flash_result_t ext_flash_memory_mapped_disable(void)
{
  return flash_device_memory_mapped_disable() == 0 ? EXT_FLASH_OK : EXT_FLASH_ERROR_IO;
}

ext_flash_result_t ext_flash_memory_mapped_access_enable(int executable)
{
  return flash_port_memory_mapped_access_enable(executable) == 0 ? EXT_FLASH_OK : EXT_FLASH_ERROR_IO;
}

void ext_flash_memory_mapped_access_disable(void)
{
  flash_port_memory_mapped_access_disable();
}
