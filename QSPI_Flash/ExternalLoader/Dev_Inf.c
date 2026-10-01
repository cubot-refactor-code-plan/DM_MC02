#include "Dev_Inf.h"

/**
 * @brief W25Q64JV 外部存储布局描述符。
 *
 * 烧录工具从 `.Dev_Info` 段读取该全局常量：映射基址为 0x70000000，
 * 总容量 8 MiB，页大小 256 字节，共 2048 个 4 KiB 擦除扇区。
 */
__attribute__((section(".Dev_Info"), used))
const StorageInfoType StorageInfo = {
  "W25Q64JV_STM32H723_OSPI2",
  NOR_FLASH,
  0x70000000UL,
  0x00800000UL,
  0x00000100UL,
  0xFFU,
  {{2048U, 0x00001000UL},
   {0U, 0U}}};
