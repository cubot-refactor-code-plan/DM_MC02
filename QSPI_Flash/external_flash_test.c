#include "external_flash_test.h"

#include <stddef.h>

#define TEST_BUFFER_SIZE 512U                                                   /**< 单个测试用例支持的最大数据长度。 */
#define TEST_SECTOR_FIRST (FLASH_DEVICE_SIZE - (2U * FLASH_DEVICE_SECTOR_SIZE)) /**< 专用测试区首偏移。 */

/** @brief 自动生成测试数据时使用的模式。 */
typedef enum
{
  TEST_PATTERN_FIXED,
  TEST_PATTERN_INCREMENTING,
  TEST_PATTERN_ZERO,
  TEST_PATTERN_ERASED,
  TEST_PATTERN_PSEUDORANDOM
} test_pattern_t;

/** @brief 单个数据测试用例的地址、长度与数据模式。 */
typedef struct
{
  uint32_t       address; /**< W25Q64JV 内部起始偏移。 */
  uint32_t       length;  /**< 测试字节数，不超过 TEST_BUFFER_SIZE。 */
  test_pattern_t pattern; /**< 写入缓冲区的生成方式。 */
} test_case_t;

/** @brief 轮询写入使用的静态工作缓冲区。 */
static uint8_t test_write_buffer[TEST_BUFFER_SIZE];
/** @brief 擦除校验和轮询读回共用的静态工作缓冲区。 */
static uint8_t test_read_buffer[TEST_BUFFER_SIZE];

/**
 * @brief 七个破坏性测试用例。
 *
 * 地址均限制在最后两个 4 KiB 扇区；其中包含页前/页后边界和跨扇区
 * 场景。每个用例执行前都会重新擦除其覆盖的扇区。
 */
static const test_case_t test_cases[] = {
  {TEST_SECTOR_FIRST + 0x100U, 1U, TEST_PATTERN_FIXED},
  {TEST_SECTOR_FIRST + 0x200U, 255U, TEST_PATTERN_INCREMENTING},
  {TEST_SECTOR_FIRST + 0x400U, 256U, TEST_PATTERN_ZERO},
  {TEST_SECTOR_FIRST + 0x6F0U, 257U, TEST_PATTERN_PSEUDORANDOM},
  {TEST_SECTOR_FIRST + 0x900U, 256U, TEST_PATTERN_ERASED},
  {TEST_SECTOR_FIRST + 0xAF0U, 512U, TEST_PATTERN_FIXED},
  {TEST_SECTOR_FIRST + FLASH_DEVICE_SECTOR_SIZE - 128U,
   257U,
   TEST_PATTERN_INCREMENTING}};

/**
 * @brief 根据模式和字节索引生成可重复的测试值。
 * @param[in] pattern 数据模式。
 * @param[in] index 当前字节在用例中的偏移。
 * @return 应写入该位置的字节。
 */
static uint8_t test_pattern_byte(test_pattern_t pattern, uint32_t index)
{
  switch (pattern)
  {
    case TEST_PATTERN_FIXED:
      return (index & 1U) == 0U ? 0x5AU : 0xA5U;
    case TEST_PATTERN_INCREMENTING:
      return (uint8_t)index;
    case TEST_PATTERN_ZERO:
      return 0x00U;
    case TEST_PATTERN_ERASED:
      return 0xFFU;
    case TEST_PATTERN_PSEUDORANDOM:
      return (uint8_t)((index * 73U + 41U) ^ (index >> 1U));
    default:
      return 0U;
  }
}

/**
 * @brief 擦除一个闭区间内的所有 4 KiB 扇区并逐字节确认 0xFF。
 * @param[in] first_sector 第一个扇区的对齐偏移。
 * @param[in] last_sector 最后一个扇区的对齐偏移，包含在测试中。
 * @param[in,out] report 更新阶段、地址和首个错误信息。
 * @return EXT_FLASH_OK 或首个擦除/读取/比较错误。
 */
static ext_flash_result_t test_erase_and_verify(uint32_t                 first_sector,
                                                uint32_t                 last_sector,
                                                ext_flash_test_report_t *report)
{
  for (uint32_t sector = first_sector; sector <= last_sector;
       sector += FLASH_DEVICE_SECTOR_SIZE)
  {
    report->stage             = EXT_FLASH_TEST_STAGE_ERASE;
    report->address           = sector;
    ext_flash_result_t result = ext_flash_erase_sector(sector);
    if (result != EXT_FLASH_OK)
    {
      return result;
    }

    report->stage = EXT_FLASH_TEST_STAGE_ERASE_VERIFY;
    for (uint32_t offset = 0U; offset < FLASH_DEVICE_SECTOR_SIZE;
         offset += TEST_BUFFER_SIZE)
    {
      result = ext_flash_read(sector + offset, test_read_buffer, TEST_BUFFER_SIZE);
      if (result != EXT_FLASH_OK)
      {
        return result;
      }
      for (uint32_t i = 0U; i < TEST_BUFFER_SIZE; ++i)
      {
        if (test_read_buffer[i] != 0xFFU)
        {
          report->first_mismatch_offset = offset + i;
          report->expected              = 0xFFU;
          report->actual                = test_read_buffer[i];
          return EXT_FLASH_ERROR_IO;
        }
      }
    }
  }
  return EXT_FLASH_OK;
}

/**
 * @brief 完整执行一个擦除、写入、读取和比较用例。
 * @param[in] test_case 用例描述，不可为 NULL。
 * @param[in,out] report 测试过程和错误报告。
 * @return EXT_FLASH_OK 或首个失败结果。
 */
static ext_flash_result_t test_run_case(const test_case_t       *test_case,
                                        ext_flash_test_report_t *report)
{
  uint32_t           first_sector = test_case->address & ~(FLASH_DEVICE_SECTOR_SIZE - 1U);
  uint32_t           last_address = test_case->address + test_case->length - 1U;
  uint32_t           last_sector  = last_address & ~(FLASH_DEVICE_SECTOR_SIZE - 1U);
  ext_flash_result_t result       = test_erase_and_verify(first_sector, last_sector, report);

  if (result != EXT_FLASH_OK)
  {
    return result;
  }
  for (uint32_t i = 0U; i < test_case->length; ++i)
  {
    test_write_buffer[i] = test_pattern_byte(test_case->pattern, i);
    test_read_buffer[i]  = 0U;
  }

  report->stage   = EXT_FLASH_TEST_STAGE_WRITE;
  report->address = test_case->address;
  result          = ext_flash_write(test_case->address, test_write_buffer, test_case->length);
  if (result != EXT_FLASH_OK)
  {
    return result;
  }

  report->stage = EXT_FLASH_TEST_STAGE_READ;
  result        = ext_flash_read(test_case->address, test_read_buffer, test_case->length);
  if (result != EXT_FLASH_OK)
  {
    return result;
  }

  report->stage = EXT_FLASH_TEST_STAGE_COMPARE;
  for (uint32_t i = 0U; i < test_case->length; ++i)
  {
    if (test_write_buffer[i] != test_read_buffer[i])
    {
      report->first_mismatch_offset = i;
      report->expected              = test_write_buffer[i];
      report->actual                = test_read_buffer[i];
      return EXT_FLASH_ERROR_IO;
    }
  }
  return EXT_FLASH_OK;
}

ext_flash_result_t ext_flash_run_data_tests(ext_flash_test_report_t *report)
{
  if (report == NULL)
  {
    return EXT_FLASH_ERROR_IO;
  }
  *report = (ext_flash_test_report_t) {
    .result = EXT_FLASH_ERROR_IO,
    .stage  = EXT_FLASH_TEST_STAGE_NONE};

  for (uint32_t i = 0U; i < (sizeof(test_cases) / sizeof(test_cases[0])); ++i)
  {
    report->case_index        = i;
    ext_flash_result_t result = test_run_case(&test_cases[i], report);
    if (result != EXT_FLASH_OK)
    {
      report->result = result;
      return result;
    }
    ++report->passed_cases;
  }
  report->stage  = EXT_FLASH_TEST_STAGE_COMPLETE;
  report->result = EXT_FLASH_OK;
  return EXT_FLASH_OK;
}
