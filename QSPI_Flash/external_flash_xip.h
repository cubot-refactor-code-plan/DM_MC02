#ifndef EXTERNAL_FLASH_XIP_H
#define EXTERNAL_FLASH_XIP_H

#include <stdint.h>

#include "external_flash.h"

#ifdef __cplusplus
extern "C"
{
#endif

  /** @brief XIP 测试使用的地址、输入和结果报告。 */
  typedef struct
  {
    ext_flash_result_t result;           /**< XIP 测试结果。 */
    uint32_t           flash_offset;     /**< 测试镜像在 W25Q64JV 内部的偏移。 */
    uint32_t           image_size;       /**< 从内部 Flash 搬运的代码字节数。 */
    uint32_t           function_address; /**< 带 Thumb 位的外置函数调用地址。 */
    uint32_t           argument;         /**< 传给测试函数的输入值。 */
    uint32_t           expected;         /**< 预先计算的期望返回值。 */
    uint32_t           actual;           /**< 从外置 Flash 执行后得到的返回值。 */
  } ext_flash_xip_report_t;

  /**
   * @brief 编程并执行链接到 `.qspi_text` 的独立测试函数。
   *
   * 函数擦除 XIP 镜像所在扇区，将链接器提供的内部 Flash 加载镜像写到
   * W25Q64JV，进入 Memory-Mapped 模式，开放可执行 MPU 区域并实际调用它。
   *
   * @param[out] report 接收镜像位置、函数地址和比较结果；不可为 NULL。
   * @return EXT_FLASH_OK 表示外置执行结果正确，否则返回失败原因。
   * @warning 会改写外置 Flash 偏移 0x00100000 附近的 XIP 镜像区。
   */
  ext_flash_result_t ext_flash_run_xip_test(ext_flash_xip_report_t *report);

#ifdef __cplusplus
}
#endif

#endif
