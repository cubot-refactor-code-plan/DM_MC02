#ifndef FLASH_DEVICE_H
#define FLASH_DEVICE_H

#include <stdint.h>

#define FLASH_DEVICE_EXPECTED_MANUFACTURER_ID 0xEFU /**< Winbond 厂商 ID。 */
#define FLASH_DEVICE_EXPECTED_MEMORY_TYPE 0x40U     /**< W25Q 系列类型 ID。 */
#define FLASH_DEVICE_EXPECTED_CAPACITY_ID 0x17U     /**< 64 Mbit 容量 ID。 */
#define FLASH_DEVICE_SR1_BUSY 0x01U                 /**< SR1 写入/擦除忙标志。 */
#define FLASH_DEVICE_SR1_WEL 0x02U                  /**< SR1 写使能锁存标志。 */
#define FLASH_DEVICE_SIZE 0x00800000UL              /**< 容量：8 MiB。 */
#define FLASH_DEVICE_MAPPED_BASE 0x70000000UL       /**< STM32H723 OCTOSPI2 映射基址。 */
#define FLASH_DEVICE_PAGE_SIZE 256U                 /**< Page Program 页大小。 */
#define FLASH_DEVICE_SECTOR_SIZE 4096U              /**< 最小擦除扇区大小。 */

/** @brief W25Q64JV JEDEC 标识和三个状态寄存器的快照。 */
typedef struct
{
  uint8_t manufacturer_id;   /**< JEDEC 厂商 ID，W25Q64JV 应为 0xEF。 */
  uint8_t memory_type;       /**< JEDEC 器件类型，W25Q64JV 应为 0x40。 */
  uint8_t capacity_id;       /**< JEDEC 容量码，W25Q64JV 应为 0x17。 */
  uint8_t status_register_1; /**< SR1：BUSY、WEL、保护位等。 */
  uint8_t status_register_2; /**< SR2：包含 Quad Enable 位。 */
  uint8_t status_register_3; /**< SR3：驱动能力等配置。 */
} flash_device_info_t;

/** @brief 依次发送 0x66/0x99，把器件复位到已知状态。 @return 0 成功，-1 失败。 */
int flash_device_reset(void);

/** @brief 读取 JEDEC ID 和 SR1/SR2/SR3。 @param[out] info 接收信息，不可为 NULL。 @return 0 成功，-1 失败。 */
int flash_device_read_info(flash_device_info_t *info);

/** @brief 检查信息是否对应本驱动支持的 W25Q64JV。 @param[in] info 待检查信息。 @return 匹配返回非 0，否则返回 0。 */
int flash_device_is_expected(const flash_device_info_t *info);

/** @brief 读取状态寄存器 1。 @param[out] status 接收 SR1。 @return 0 成功，-1 失败。 */
int flash_device_read_status1(uint8_t *status);

/** @brief 发送 WREN 并确认 WEL。 @return 0 成功，-1 失败。 */
int flash_device_write_enable(void);

/** @brief 等待 SR1.BUSY 清零。 @param[in] timeout_ms 超时毫秒数。 @return 0 成功，-2 超时，-1 传输失败。 */
int flash_device_wait_ready(uint32_t timeout_ms);

/** @brief 用 3-byte Read 命令读取。 @param[in] address 器件内部偏移。 @param[out] data 接收缓冲区。 @param[in] length 字节数。 @return 0 成功，-1 失败。 */
int flash_device_read(uint32_t address, void *data, uint32_t length);

/** @brief 用 3-byte Page Program 写一页内的数据。 @param[in] address 起始偏移。 @param[in] data 源数据。 @param[in] length 1~256 字节且不得跨页。 @return 0 成功，负值失败。 */
int flash_device_program_page(uint32_t address, const void *data, uint32_t length);

/** @brief 用 3-byte 4 KiB Erase 擦除对齐扇区。 @param[in] address 4 KiB 对齐偏移。 @return 0 成功，负值失败。 */
int flash_device_erase_sector(uint32_t address);

/** @brief 发送 Chip Erase 并等待最多 300 秒。 @return 0 成功，负值失败。 */
int flash_device_erase_chip(void);

/** @brief 检查并按需设置 SR2.QE。 @return 0 表示 QE 有效，-1 失败。 */
int flash_device_enable_quad(void);

/** @brief 使用 MDMA 和 3-byte Read 读取。参数含义同 flash_device_read()。 @return 0 成功，-1 失败。 */
int flash_device_read_mdma(uint32_t address, void *data, uint32_t length);

/** @brief 使用 MDMA 写一页内的数据。参数约束同 flash_device_program_page()。 @return 0 成功，负值失败。 */
int flash_device_program_page_mdma(uint32_t address, const void *data, uint32_t length);

/** @brief 配置 3-byte Quad Output Fast Read Memory-Mapped 模式。 @return 0 成功，-1 失败。 */
int flash_device_memory_mapped_enable(void);

/** @brief 进入供外部烧录器使用的 0x03/0x02 单线映射模式。 */
int flash_device_memory_mapped_spi_enable(void);

/** @brief 中止 Memory-Mapped 模式。 @return 0 成功，-1 失败。 */
int flash_device_memory_mapped_disable(void);

#endif
