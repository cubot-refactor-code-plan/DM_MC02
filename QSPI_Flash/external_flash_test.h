#ifndef EXTERNAL_FLASH_TEST_H
#define EXTERNAL_FLASH_TEST_H

#include <stdint.h>

#include "external_flash.h"

#ifdef __cplusplus
extern "C"
{
#endif

  /** @brief 自动数据测试当前执行到的阶段，便于在 GDB 中定位失败步骤。 */
  typedef enum
  {
    EXT_FLASH_TEST_STAGE_NONE = 0,     /**< 尚未开始。 */
    EXT_FLASH_TEST_STAGE_ERASE,        /**< 正在擦除测试扇区。 */
    EXT_FLASH_TEST_STAGE_ERASE_VERIFY, /**< 正在验证擦除值 0xFF。 */
    EXT_FLASH_TEST_STAGE_WRITE,        /**< 正在写入测试模式。 */
    EXT_FLASH_TEST_STAGE_READ,         /**< 正在读回测试数据。 */
    EXT_FLASH_TEST_STAGE_COMPARE,      /**< 正在逐字节比较。 */
    EXT_FLASH_TEST_STAGE_COMPLETE      /**< 全部测试通过。 */
  } ext_flash_test_stage_t;

  /** @brief 自动数据测试的诊断报告。 */
  typedef struct
  {
    ext_flash_result_t     result;                /**< 总体结果。 */
    ext_flash_test_stage_t stage;                 /**< 返回时所在阶段。 */
    uint32_t               case_index;            /**< 当前或最后一个测试用例的零基索引。 */
    uint32_t               passed_cases;          /**< 已通过的测试用例数。 */
    uint32_t               address;               /**< 当前用例使用的 Flash 内部偏移。 */
    uint32_t               first_mismatch_offset; /**< 首个错误相对当前地址的偏移。 */
    uint8_t                expected;              /**< 首个错误处的期望字节。 */
    uint8_t                actual;                /**< 首个错误处的实际字节。 */
  } ext_flash_test_report_t;

  /**
   * @brief 在外置 Flash 最后两个扇区执行破坏性数据测试。
   *
   * 覆盖擦除校验、固定/递增/全零/全 FF/伪随机模式、不同长度、跨页和
   * 跨扇区写入。应在 ext_flash_init() 成功后调用。
   *
   * @param[out] report 接收过程与失败诊断；不可为 NULL。
   * @return EXT_FLASH_OK 表示全部用例通过，否则返回首个失败结果。
   * @warning 会改写 Flash 偏移 0x007FE000 到 0x007FFFFF。
   */
  ext_flash_result_t ext_flash_run_data_tests(ext_flash_test_report_t *report);

#ifdef __cplusplus
}
#endif

#endif
