#ifndef EXTERNAL_FLASH_H
#define EXTERNAL_FLASH_H

#include <stddef.h>

#include "flash_device.h"

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @file external_flash.h
   * @brief DM-MC-Board02 板载 W25Q64JV 的公共接口。
   *
   * 典型的间接读写调用顺序如下：
   * @code
   * ext_flash_info_t info;
   * uint8_t data[32];
   * if (ext_flash_init(&info) == EXT_FLASH_OK) {
   *     ext_flash_erase_sector(0U);       // address 必须按 4 KiB 对齐
   *     ext_flash_write(0U, data, sizeof(data));
   *     ext_flash_read(0U, data, sizeof(data));
   * }
   * @endcode
   *
   * Memory-Mapped 数据访问必须按“外设映射 → MPU 开放 → CPU 读取 → MPU
   * 关闭 → 外设退出”的顺序配对调用。擦除或编程前必须先退出映射模式。
   */

  /** @brief 外置 Flash 公共 API 的统一返回值。 */
  typedef enum
  {
    EXT_FLASH_OK              = 0,  /**< 操作成功。 */
    EXT_FLASH_ERROR_IO        = -1, /**< 参数非法或 QSPI/HAL 传输失败。 */
    EXT_FLASH_ERROR_DEVICE_ID = -2, /**< JEDEC ID 与 W25Q64JV 不匹配。 */
    EXT_FLASH_ERROR_TIMEOUT   = -3  /**< 等待 Flash 就绪超时。 */
  } ext_flash_result_t;

  /** @brief 对外暴露的器件信息，与器件层信息结构一致。 */
  typedef flash_device_info_t ext_flash_info_t;

  /**
   * @brief 复位、识别并初始化 W25Q64JV。
   *
   * 应在 `MX_OCTOSPI2_Init()` 之后、任何读写操作之前调用一次。函数会
   * 保护未建立的映射窗口、复位器件、读取 JEDEC ID 和状态寄存器，
   * 并确保 QE 位有效。
   *
   * @param[out] info 保存 JEDEC ID 和三个状态寄存器；不可为 NULL。
   * @return EXT_FLASH_OK 表示初始化完成，其他值表示失败原因。
   */
  ext_flash_result_t ext_flash_init(ext_flash_info_t *info);

  /**
   * @brief 重新读取器件标识和状态寄存器。
   * @param[out] info 接收当前器件信息；不可为 NULL。
   * @return EXT_FLASH_OK 或 EXT_FLASH_ERROR_IO。
   */
  ext_flash_result_t ext_flash_read_info(ext_flash_info_t *info);

  /**
   * @brief 发送 Write Enable 命令并确认 SR1.WEL 已置位。
   * @return EXT_FLASH_OK 或 EXT_FLASH_ERROR_IO。
   */
  ext_flash_result_t ext_flash_write_enable(void);

  /**
   * @brief 轮询 SR1.BUSY，直到器件就绪或超时。
   * @param[in] timeout_ms 最大等待时间，单位毫秒。
   * @return EXT_FLASH_OK、EXT_FLASH_ERROR_TIMEOUT 或 EXT_FLASH_ERROR_IO。
   */
  ext_flash_result_t ext_flash_wait_ready(uint32_t timeout_ms);

  /**
   * @brief 使用轮询方式从 Flash 读取数据。
   * @param[in] address Flash 内部偏移，范围 0 到 FLASH_DEVICE_SIZE-1。
   * @param[out] data 接收数据的缓冲区；不可为 NULL。
   * @param[in] length 读取字节数；必须大于 0 且不能越过器件末尾。
   * @return EXT_FLASH_OK 或 EXT_FLASH_ERROR_IO。
   */
  ext_flash_result_t ext_flash_read(uint32_t address, void *data, size_t length);

  /**
   * @brief 使用轮询方式写入任意长度的数据。
   *
   * 函数自动按 256 字节页边界拆分 Page Program；调用者必须预先擦除目标
   * 区域，因为 NOR Flash 编程只能把位从 1 改为 0。
   *
   * @param[in] address Flash 内部偏移。
   * @param[in] data 待写入数据；不可为 NULL。
   * @param[in] length 写入字节数；必须大于 0 且不能越界。
   * @return EXT_FLASH_OK 或 EXT_FLASH_ERROR_IO。
   */
  ext_flash_result_t ext_flash_write(uint32_t address, const void *data, size_t length);

  /**
   * @brief 擦除一个 4 KiB 扇区。
   * @param[in] address 扇区首地址，必须按 FLASH_DEVICE_SECTOR_SIZE 对齐。
   * @return EXT_FLASH_OK 或 EXT_FLASH_ERROR_IO。
   */
  ext_flash_result_t ext_flash_erase_sector(uint32_t address);

  /**
   * @brief 擦除整颗 8 MiB Flash。
   * @warning 会不可逆地清除外置 Flash 全部内容，最长等待 300 秒。
   * @return EXT_FLASH_OK 或 EXT_FLASH_ERROR_IO。
   */
  ext_flash_result_t ext_flash_erase_chip(void);

  /**
   * @brief 使用 OCTOSPI MDMA 从 Flash 读取数据。
   * @param[in] address Flash 内部偏移。
   * @param[out] data MDMA 可访问的接收缓冲区；本工程建议放在 AXI SRAM。
   * @param[in] length 读取字节数；必须大于 0 且不能越界。
   * @return EXT_FLASH_OK 或 EXT_FLASH_ERROR_IO。
   * @note 未配置 MDMA 句柄时安全回退为轮询读取；启用 D-Cache 时，调用者
   *       负责在读取后 invalidate 接收缓冲区。
   */
  ext_flash_result_t ext_flash_read_mdma(uint32_t address, void *data, size_t length);

  /**
   * @brief 使用 OCTOSPI MDMA 写入数据，并自动按页拆分。
   * @param[in] address Flash 内部偏移。
   * @param[in] data MDMA 可访问的源缓冲区；本工程建议放在 AXI SRAM。
   * @param[in] length 写入字节数；必须大于 0 且不能越界。
   * @return EXT_FLASH_OK 或 EXT_FLASH_ERROR_IO。
   * @note 未配置 MDMA 句柄时安全回退为轮询写入；调用前应擦除目标区域。
   */
  ext_flash_result_t ext_flash_write_mdma(uint32_t address, const void *data, size_t length);

  /**
   * @brief 让 OCTOSPI2 进入 0x70000000 Memory-Mapped 读取模式。
   * @return EXT_FLASH_OK 或 EXT_FLASH_ERROR_IO。
   * @note 此函数只配置外设；CPU 访问前还必须调用
   *       ext_flash_memory_mapped_access_enable() 开放 MPU Region 7。
   */
  ext_flash_result_t ext_flash_memory_mapped_enable(void);

  /**
   * @brief 进入供 OpenOCD stmqspi 探测和烧录使用的单线映射模式。
   * @note 仅用于部署外置载荷；正常数据访问和 XIP 应使用四线接口。
   */
  ext_flash_result_t ext_flash_memory_mapped_spi_enable(void);

  /**
   * @brief 终止 OCTOSPI2 Memory-Mapped 模式并恢复间接操作能力。
   * @return EXT_FLASH_OK 或 EXT_FLASH_ERROR_IO。
   * @note 应先调用 ext_flash_memory_mapped_access_disable() 关闭 CPU 访问。
   */
  ext_flash_result_t ext_flash_memory_mapped_disable(void);

  /**
   * @brief 动态开放实际 8 MiB OCTOSPI2 地址窗口的 MPU Region 7。
   * @param[in] executable 非 0 时允许取指，用于 XIP；0 时只允许数据访问。
   * @return 当前实现配置成功后返回 EXT_FLASH_OK。
   */
  ext_flash_result_t ext_flash_memory_mapped_access_enable(int executable);

  /**
   * @brief 禁用 MPU Region 7，使 Region 6 的 OCTOSPI2 No-Access 保护重新生效。
   */
  void ext_flash_memory_mapped_access_disable(void);

#ifdef __cplusplus
}
#endif

#endif
