#include "flash_device.h"

#include "flash_port_stm32h7.h"

#define W25Q_CMD_ENABLE_RESET 0x66U
#define W25Q_CMD_RESET_DEVICE 0x99U
#define W25Q_CMD_READ_JEDEC_ID 0x9FU
#define W25Q_CMD_READ_SR1 0x05U
#define W25Q_CMD_READ_SR2 0x35U
#define W25Q_CMD_READ_SR3 0x15U
#define W25Q_CMD_WRITE_ENABLE 0x06U
#define W25Q_CMD_READ 0x03U
#define W25Q_CMD_PROGRAM 0x02U
#define W25Q_CMD_ERASE_4K 0x20U
#define W25Q_CMD_WRITE_SR2 0x31U
#define W25Q_CMD_QUAD_READ 0x6BU
#define W25Q_CMD_QUAD_PROGRAM 0x32U
#define W25Q_QUAD_READ_DUMMY_CYCLES 8U
#define W25Q_SR2_QUAD_ENABLE 0x02U
#define W25Q_CMD_CHIP_ERASE 0xC7U

/* 本文件实现 W25Q64JV 命令层；所有地址均为器件内部字节偏移。 */

int flash_device_reset(void)
{
  if (flash_port_command(W25Q_CMD_ENABLE_RESET) != 0)
  {
    return -1;
  }
  if (flash_port_command(W25Q_CMD_RESET_DEVICE) != 0)
  {
    return -1;
  }
  flash_port_delay_ms(1U);
  return 0;
}

int flash_device_read_info(flash_device_info_t *info)
{
  uint8_t jedec_id[3];

  if (info == NULL)
  {
    return -1;
  }
  if (flash_port_receive(W25Q_CMD_READ_JEDEC_ID, jedec_id, sizeof(jedec_id)) != 0)
  {
    return -1;
  }
  if (flash_port_receive(W25Q_CMD_READ_SR1, &info->status_register_1, 1U) != 0 || flash_port_receive(W25Q_CMD_READ_SR2, &info->status_register_2, 1U) != 0 || flash_port_receive(W25Q_CMD_READ_SR3, &info->status_register_3, 1U) != 0)
  {
    return -1;
  }

  info->manufacturer_id = jedec_id[0];
  info->memory_type     = jedec_id[1];
  info->capacity_id     = jedec_id[2];
  return 0;
}

int flash_device_is_expected(const flash_device_info_t *info)
{
  return info != NULL && info->manufacturer_id == FLASH_DEVICE_EXPECTED_MANUFACTURER_ID && info->memory_type == FLASH_DEVICE_EXPECTED_MEMORY_TYPE && info->capacity_id == FLASH_DEVICE_EXPECTED_CAPACITY_ID;
}

int flash_device_read_status1(uint8_t *status)
{
  return flash_port_receive(W25Q_CMD_READ_SR1, status, 1U);
}

int flash_device_write_enable(void)
{
  uint8_t status;

  if (flash_port_command(W25Q_CMD_WRITE_ENABLE) != 0 || flash_device_read_status1(&status) != 0)
  {
    return -1;
  }
  return (status & FLASH_DEVICE_SR1_WEL) != 0U ? 0 : -1;
}

int flash_device_wait_ready(uint32_t timeout_ms)
{
  uint32_t start = flash_port_get_tick_ms();
  uint8_t  status;

  do
  {
    if (flash_device_read_status1(&status) != 0)
    {
      return -1;
    }
    if ((status & FLASH_DEVICE_SR1_BUSY) == 0U)
    {
      return 0;
    }
  } while ((flash_port_get_tick_ms() - start) < timeout_ms);

  return -2;
}

int flash_device_read(uint32_t address, void *data, uint32_t length)
{
  return flash_port_address_receive(W25Q_CMD_READ, address, data, length);
}

int flash_device_program_page(uint32_t address, const void *data, uint32_t length)
{
  /* Page Program 不能跨越 256 字节页边界，上层负责按页拆分。 */
  if (length == 0U || length > FLASH_DEVICE_PAGE_SIZE || ((address & (FLASH_DEVICE_PAGE_SIZE - 1U)) + length) > FLASH_DEVICE_PAGE_SIZE)
  {
    return -1;
  }
  if (flash_device_write_enable() != 0 || flash_port_address_transmit(W25Q_CMD_PROGRAM, address, data, length) != 0)
  {
    return -1;
  }
  return flash_device_wait_ready(1000U);
}

int flash_device_erase_sector(uint32_t address)
{
  if ((address & (FLASH_DEVICE_SECTOR_SIZE - 1U)) != 0U)
  {
    return -1;
  }
  if (flash_device_write_enable() != 0 || flash_port_address_command(W25Q_CMD_ERASE_4K, address) != 0)
  {
    return -1;
  }
  return flash_device_wait_ready(5000U);
}

int flash_device_erase_chip(void)
{
  if (flash_device_write_enable() != 0 || flash_port_command(W25Q_CMD_CHIP_ERASE) != 0)
  {
    return -1;
  }
  return flash_device_wait_ready(300000U);
}

int flash_device_enable_quad(void)
{
  uint8_t status2;

  if (flash_port_receive(W25Q_CMD_READ_SR2, &status2, 1U) != 0)
  {
    return -1;
  }
  /* QE 已经置位时不重复写状态寄存器，以减少非易失写入。 */
  if ((status2 & W25Q_SR2_QUAD_ENABLE) != 0U)
  {
    return 0;
  }
  status2 |= W25Q_SR2_QUAD_ENABLE;
  if (flash_device_write_enable() != 0 || flash_port_transmit(W25Q_CMD_WRITE_SR2, &status2, 1U) != 0 || flash_device_wait_ready(1000U) != 0)
  {
    return -1;
  }
  if (flash_port_receive(W25Q_CMD_READ_SR2, &status2, 1U) != 0)
  {
    return -1;
  }
  return (status2 & W25Q_SR2_QUAD_ENABLE) != 0U ? 0 : -1;
}

int flash_device_read_mdma(uint32_t address, void *data, uint32_t length)
{
  return flash_port_address_receive_mdma(W25Q_CMD_READ, address, data, length, 0U, FLASH_PORT_DATA_1_LINE);
}

int flash_device_program_page_mdma(uint32_t address, const void *data, uint32_t length)
{
  if (length == 0U || length > FLASH_DEVICE_PAGE_SIZE || ((address & (FLASH_DEVICE_PAGE_SIZE - 1U)) + length) > FLASH_DEVICE_PAGE_SIZE)
  {
    return -1;
  }
  if (flash_device_write_enable() != 0 || flash_port_address_transmit_mdma(W25Q_CMD_PROGRAM, address, data, length, FLASH_PORT_DATA_1_LINE) != 0)
  {
    return -1;
  }
  return flash_device_wait_ready(1000U);
}

int flash_device_memory_mapped_enable(void)
{
  return flash_port_memory_mapped_enable(W25Q_CMD_QUAD_READ,
                                         W25Q_CMD_QUAD_PROGRAM,
                                         W25Q_QUAD_READ_DUMMY_CYCLES);
}

int flash_device_memory_mapped_spi_enable(void)
{
  return flash_port_memory_mapped_spi_enable(W25Q_CMD_READ,
                                             W25Q_CMD_PROGRAM);
}

int flash_device_memory_mapped_disable(void)
{
  return flash_port_memory_mapped_disable();
}
