#ifndef FLASH_PORT_STM32H7_H
#define FLASH_PORT_STM32H7_H

#include <stddef.h>
#include <stdint.h>

typedef enum
{
  FLASH_PORT_DATA_1_LINE, /**< 数据阶段使用 IO0。 */
  FLASH_PORT_DATA_4_LINES /**< 数据阶段并行使用 IO0~IO3。 */
} flash_port_data_mode_t;

/** @brief 建立 OCTOSPI2 MPU 保护，并在配置 MDMA 时开启外设 IRQ。 */
void flash_port_prepare(void);

/** @brief 发送无地址、无数据命令。 @param[in] instruction 8 位指令码。 @return 0 成功，-1 失败。 */
int flash_port_command(uint8_t instruction);

/** @brief 发送指令后轮询接收数据。 @param[in] instruction 指令码。 @param[out] data 接收缓冲区。 @param[in] length 字节数。 @return 0 成功，-1 失败。 */
int flash_port_receive(uint8_t instruction, void *data, size_t length);

/** @brief 发送指令后轮询发送数据。 @param[in] instruction 指令码。 @param[in] data 源缓冲区。 @param[in] length 字节数。 @return 0 成功，-1 失败。 */
int flash_port_transmit(uint8_t instruction, const void *data, size_t length);

/** @brief 发送带 24 位地址、无数据命令。 @param[in] instruction 指令码。 @param[in] address 器件内部地址。 @return 0 成功，-1 失败。 */
int flash_port_address_command(uint8_t instruction, uint32_t address);

/** @brief 发送指令和 24 位地址后轮询接收。 @param[in] instruction 指令码。 @param[in] address 器件内部地址。 @param[out] data 接收缓冲区。 @param[in] length 字节数。 @return 0 成功，-1 失败。 */
int flash_port_address_receive(uint8_t instruction, uint32_t address, void *data, size_t length);

/** @brief 发送指令和 24 位地址后轮询发送。 @param[in] instruction 指令码。 @param[in] address 器件内部地址。 @param[in] data 源缓冲区。 @param[in] length 字节数。 @return 0 成功，-1 失败。 */
int flash_port_address_transmit(uint8_t instruction, uint32_t address, const void *data, size_t length);

/**
 * @brief 发送带地址命令并通过 MDMA 接收。
 * @param[in] instruction 指令码。
 * @param[in] address 器件内部地址。
 * @param[out] data MDMA 可访问的接收缓冲区。
 * @param[in] length 接收字节数。
 * @param[in] dummy_cycles 地址与数据之间的空周期数。
 * @param[in] data_mode 数据阶段使用单线或四线。
 * @return 0 成功，-1 失败或超时。
 */
int flash_port_address_receive_mdma(uint8_t instruction, uint32_t address, void *data, size_t length, uint8_t dummy_cycles, flash_port_data_mode_t data_mode);

/**
 * @brief 发送带地址命令并通过 MDMA 发送数据。
 * @param[in] instruction 指令码。
 * @param[in] address 器件内部地址。
 * @param[in] data MDMA 可访问的源缓冲区。
 * @param[in] length 发送字节数。
 * @param[in] data_mode 数据阶段使用单线或四线。
 * @return 0 成功，-1 失败或超时。
 */
int flash_port_address_transmit_mdma(uint8_t instruction, uint32_t address, const void *data, size_t length, flash_port_data_mode_t data_mode);

/** @brief 配置 OCTOSPI2 Memory-Mapped 读写命令。 @param[in] read_instruction 读取指令。 @param[in] write_instruction 写入指令。 @param[in] dummy_cycles 读取空周期数。 @return 0 成功，-1 失败。 */
int flash_port_memory_mapped_enable(uint8_t read_instruction,
                                    uint8_t write_instruction,
                                    uint8_t dummy_cycles);

/** @brief 配置供烧录器探测使用的单线 Memory-Mapped 读写命令。 */
int flash_port_memory_mapped_spi_enable(uint8_t read_instruction,
                                        uint8_t write_instruction);

/** @brief 通过 HAL_OSPI_Abort() 退出 Memory-Mapped 模式。 @return 0 成功，-1 失败。 */
int flash_port_memory_mapped_disable(void);

/** @brief 配置 MPU Region 7 开放 8 MiB 窗口。 @param[in] executable 非 0 允许取指，0 禁止取指。 @return 0。 */
int flash_port_memory_mapped_access_enable(int executable);

/** @brief 禁用 MPU Region 7，恢复 Region 6 的 No-Access 保护。 */
void flash_port_memory_mapped_access_disable(void);

/** @brief 平台毫秒延时。 @param[in] milliseconds 延时毫秒数。 */
void flash_port_delay_ms(uint32_t milliseconds);

/** @brief 获取 HAL 毫秒节拍。 @return 自 HAL_Init() 后的毫秒计数。 */
uint32_t flash_port_get_tick_ms(void);

#endif
