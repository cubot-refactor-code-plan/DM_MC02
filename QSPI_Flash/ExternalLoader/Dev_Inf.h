#ifndef DEV_INF_H
#define DEV_INF_H

#include <stdint.h>

#define NOR_FLASH 1U /**< STM32 External Loader 约定的 NOR Flash 类型。 */

/** @brief 一种擦除扇区布局的数量与大小。 */
typedef struct
{
  uint32_t SectorNum;  /**< 此布局包含的扇区数量。 */
  uint32_t SectorSize; /**< 每个扇区的字节数。 */
} SectorInfo;

/** @brief 外部烧录工具用于识别存储器布局的固定描述结构。 */
typedef struct
{
  char       DeviceName[100];    /**< 在烧录工具中显示的 Loader 名称。 */
  uint16_t   DeviceType;         /**< 存储器类型，本工程为 NOR_FLASH。 */
  uint32_t   DeviceStartAddress; /**< Cortex-M7 映射窗口首地址。 */
  uint32_t   DeviceSize;         /**< 存储器总容量，单位字节。 */
  uint32_t   PageSize;           /**< 最小 Page Program 大小。 */
  uint8_t    EraseValue;         /**< 擦除后的字节值，NOR Flash 为 0xFF。 */
  SectorInfo sectors[10];        /**< 擦除布局，以 {0,0} 结束。 */
} StorageInfoType;

/** @brief 位于 `.Dev_Info` 段中的 W25Q64JV 固定描述符。 */
extern const StorageInfoType StorageInfo;

#endif
