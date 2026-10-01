#include <stddef.h>
#include <stdint.h>

#include "external_flash.h"
#include "loader_board.h"

#define LOADER_FLASH_BASE FLASH_DEVICE_MAPPED_BASE                         /**< 烧录工具使用的外置存储绝对基址。 */
#define LOADER_BUFFER_SIZE 256U                                            /**< Verify 分块读取缓冲区大小。 */
#define LOADER_API __attribute__((section(".loader_api"), used, noinline)) /**< 保留 Loader ABI 入口。 */

/** @brief Verify() 从外置 Flash 分块读取时使用的静态缓冲区。 */
static uint8_t verify_buffer[LOADER_BUFFER_SIZE];
/** @brief Init() 阶段诊断值，供 GDB 检查：1=板级前，2=Flash 前，3=完成。 */
volatile int loader_last_stage;
/** @brief 最近一次 ext_flash_init() 返回值，供 Loader 调试。 */
volatile ext_flash_result_t loader_flash_init_result = EXT_FLASH_ERROR_IO;
/** @brief Init() 读到的 JEDEC ID 和状态寄存器快照。 */
volatile ext_flash_info_t loader_flash_info;

/* 链接脚本导出的 BSS 边界；烧录工具直接跳入 Init，不会执行常规启动代码。 */
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;

/** @brief 清零 Loader 的 BSS，使每次 Init() 调用都从确定状态开始。 */
static void loader_clear_bss(void)
{
  for (uint32_t *word = &__bss_start__; word < &__bss_end__; ++word)
  {
    *word = 0U;
  }
}

/**
 * @brief 将烧录工具传入的映射绝对地址转换为 W25Q64JV 内部偏移。
 * @param[in] address 0x70000000 起的绝对地址。
 * @param[in] size 本次操作字节数；允许为 0。
 * @param[out] offset 转换后的器件内部偏移。
 * @return 地址区间合法返回 0，否则返回 -1。
 */
static int loader_offset(uint32_t address, uint32_t size, uint32_t *offset)
{
  if (address < LOADER_FLASH_BASE || size > FLASH_DEVICE_SIZE || address - LOADER_FLASH_BASE > FLASH_DEVICE_SIZE - size)
  {
    return -1;
  }
  *offset = address - LOADER_FLASH_BASE;
  return 0;
}

/**
 * @brief External Loader 初始化入口。
 * @param[in] configure_memory_mapped_mode 非 0 时初始化后进入只读映射模式；
 *            常规擦写流程应传 0。
 * @return 1 成功，0 失败。
 * @note 烧录工具必须先调用本函数，再调用其他入口。
 */
LOADER_API int Init(uint8_t configure_memory_mapped_mode)
{
  ext_flash_info_t info;

  loader_clear_bss();
  loader_last_stage = 1;
  if (loader_board_init() != 0)
  {
    loader_last_stage = -1;
    return 0;
  }
  loader_last_stage        = 2;
  loader_flash_init_result = ext_flash_init(&info);
  loader_flash_info        = info;
  if (loader_flash_init_result != EXT_FLASH_OK)
  {
    loader_last_stage = -2;
    return 0;
  }
  if (configure_memory_mapped_mode != 0U)
  {
    if (ext_flash_memory_mapped_enable() != EXT_FLASH_OK || ext_flash_memory_mapped_access_enable(0) != EXT_FLASH_OK)
    {
      return 0;
    }
  }
  loader_last_stage = 3;
  return 1;
}

/**
 * @brief External Loader 读取入口。
 * @param[in] address 0x70000000~0x707FFFFF 范围内的绝对地址。
 * @param[in] size 读取字节数。
 * @param[out] buffer 烧录工具提供的 RAM 接收缓冲区。
 * @return 1 成功，0 表示参数或读取失败。
 */
LOADER_API int Read(uint32_t address, uint32_t size, uint8_t *buffer)
{
  uint32_t offset;

  if (buffer == NULL || loader_offset(address, size, &offset) != 0)
  {
    return 0;
  }
  return ext_flash_read(offset, buffer, size) == EXT_FLASH_OK ? 1 : 0;
}

/**
 * @brief External Loader 写入入口，内部自动按 256 字节页拆分。
 * @param[in] address 外置映射绝对地址。
 * @param[in] size 写入字节数。
 * @param[in] buffer 烧录工具提供的源数据缓冲区。
 * @return 1 成功，0 表示参数或写入失败。
 * @note 调用者应先通过 SectorErase() 擦除目标区域。
 */
LOADER_API int Write(uint32_t address, uint32_t size, uint8_t *buffer)
{
  uint32_t offset;

  if (buffer == NULL || loader_offset(address, size, &offset) != 0)
  {
    return 0;
  }
  return ext_flash_write(offset, buffer, size) == EXT_FLASH_OK ? 1 : 0;
}

/**
 * @brief 擦除覆盖给定闭区间的全部 4 KiB 扇区。
 * @param[in] erase_start_address 起始绝对地址，可不对齐。
 * @param[in] erase_end_address 结束绝对地址，包含该地址，可不对齐。
 * @return 1 成功，0 表示越界或擦除失败。
 */
LOADER_API int SectorErase(uint32_t erase_start_address,
                           uint32_t erase_end_address)
{
  uint32_t start;
  uint32_t end;

  if (loader_offset(erase_start_address, 1U, &start) != 0 || loader_offset(erase_end_address, 1U, &end) != 0)
  {
    return 0;
  }
  start &= ~(FLASH_DEVICE_SECTOR_SIZE - 1U);
  end &= ~(FLASH_DEVICE_SECTOR_SIZE - 1U);
  for (uint32_t address = start; address <= end;
       address += FLASH_DEVICE_SECTOR_SIZE)
  {
    if (ext_flash_erase_sector(address) != EXT_FLASH_OK)
    {
      return 0;
    }
  }
  return 1;
}

/**
 * @brief 擦除整颗 W25Q64JV。
 * @return 1 成功，0 失败。
 * @warning 会清除全部 8 MiB 内容，最坏等待时间约 300 秒。
 */
LOADER_API int MassErase(void)
{
  return ext_flash_erase_chip() == EXT_FLASH_OK ? 1 : 0;
}

/**
 * @brief 逐字节比较外置 Flash 与 RAM，并累计 Flash 数据校验和。
 * @param[in] memory_address 待校验外置存储绝对地址。
 * @param[in] ram_buffer_address 期望数据在目标 RAM 中的地址。
 * @param[in] size 校验字节数。
 * @param[in] miss_alignment 烧录工具 ABI 保留参数，当前实现不使用。
 * @return 高 32 位为截至返回位置的累加和；低 32 位为首个错误绝对地址，
 *         全部匹配时为 memory_address + size。
 */
LOADER_API uint64_t Verify(uint32_t memory_address, uint32_t ram_buffer_address, uint32_t size, uint32_t miss_alignment)
{
  const uint8_t *expected = (const uint8_t *)(uintptr_t)ram_buffer_address;
  uint32_t       offset;
  uint32_t       checksum = 0U;

  (void)miss_alignment;
  if (loader_offset(memory_address, size, &offset) != 0)
  {
    return memory_address;
  }
  for (uint32_t done = 0U; done < size;)
  {
    uint32_t chunk = (size - done) < LOADER_BUFFER_SIZE ? (size - done) : LOADER_BUFFER_SIZE;
    if (ext_flash_read(offset + done, verify_buffer, chunk) != EXT_FLASH_OK)
    {
      return memory_address + done;
    }
    for (uint32_t i = 0U; i < chunk; ++i)
    {
      checksum += verify_buffer[i];
      if (verify_buffer[i] != expected[done + i])
      {
        return ((uint64_t)checksum << 32) | (memory_address + done + i);
      }
    }
    done += chunk;
  }
  return ((uint64_t)checksum << 32) | (memory_address + size);
}
