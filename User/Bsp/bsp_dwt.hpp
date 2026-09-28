/**
 * @file bsp_dwt.hpp
 * @author Rh
 * @brief DWT 计时驱动 —— Cortex-M7 内核 CYCCNT（本工程 1.818 ns / 计数）
 * @version 0.1
 * @date 2026-09-28
 *
 * @copyright Copyright (c) 2026
 *
 * @details CYCCNT 是内核里的 32 位自由运行计数器：不占外设、不依赖中断，所以
 *          关中断期间（临界区）照样能计时、能延时（HAL_Delay 做不到）；
 *          分辨率 = 1 个 CPU 周期，比 HAL_GetTick() 的 1 ms 高 5~6 个数量级。
 *
 * @note 用法（bsp_init() 里已统一 init，之后随处可用）：
 *
 *      uint32_t last = 0;
 *      (void)bsp_dwt.delta_s(&last);        // 丢弃第一次，取基准
 *      ...                                  // 被测代码
 *      double dt = bsp_dwt.delta_s(&last);  // 间隔（秒）
 *
 *      double t0 = bsp_dwt.time_s();        // 绝对时间轴（秒）
 *      bsp_dwt.delay(0.0005);               // 忙等 500 us（临界区可用）
 *
 * @warning 32 位计数器每 7.809 s 回绕一次（本工程 550 MHz）：
 *          - delta_s() 用无符号差值，两次间隔 < 7.809 s 时永远正确；
 *          - 绝对时间轴靠"回绕累加"续命，所以两次 time_*() 之间同样不能超过
 *            7.809 s，否则会少算圈数。
 * @warning CPU 进入 sleep/stop 时 CYCCNT 停走 ⇒ 它不是墙钟，别拿它记真实时间。
 * @warning delta_s() 无状态、可在 ISR 中调用；time_*() 会推进内部累加量，
 *          只应在任务上下文（单一位置）调用。
 */

#ifndef __BSP_DWT_HPP__
#define __BSP_DWT_HPP__

#include "main.h"     // IWYU pragma: keep（DWT / CoreDebug / RCC / HAL）
#include "status.hpp" // 统一状态码

#include <stdint.h>

/**
 * @brief DWT 计时类
 *
 * @note CPU 频率由 init() 自动从 RCC 算出，没有配置项，故不设 Config，也只需一个实例。
 */
class BspDwt
{
public:
  // ---------------- 公有接口 ----------------

  BspDwt() = default;

  /**
   * @brief 使能 CYCCNT 周期计数（幂等，可重复调用）
   *
   * @note 顺序：DEMCR.TRCENA（总开关，脱机运行时必须自己置位，不能指望调试器）
   *            → DWT.LAR 解锁（Cortex-M7 的 CoreSight 软件锁）
   *            → CYCCNT 清零 → CYCCNTENA 使能 → 等约 8 周期稳定 → 再清零
   *
   * @note 时间基准 = CPU 时钟 = SYSCLK / D1CPRE（自动从 RCC 算）。
   *       ⚠ 不能用 SystemCoreClock：STM32H7 的 HAL 把它定义成 HCLK（D2 时钟），
   *         本工程 HCLK 只有 CPU 时钟的一半，拿它换算会差 2 倍。
   *
   * @return Status OK=可用；IO_ERROR=本内核无 CYCCNT（CTRL.NOCYCCNT=1）或使能未生效
   */
  Status init();

  ///< 是否已初始化且 CYCCNT 可用
  bool available() const;

  ///< 实际使用的 CPU 时钟（Hz），即 CYCCNT 计数频率（便于核对换算）
  uint32_t cpu_hz() const;

  /**
   * @brief 与上一次调用之间的时间间隔（秒）
   *
   * @param last 时间戳，首次调用传入的值无意义（第一次结果要丢弃）
   * @return 间隔（秒）；两次调用间隔 < 7.886 s 时结果严格正确
   *
   * @note 无状态、回绕安全（无符号差值），可在 ISR / 临界区中调用
   */
  double delta_s(uint32_t *last) const;

  ///< 当前绝对时间（秒，自 init() 起算）
  double time_s();

  ///< 当前绝对时间（毫秒）
  double time_ms();

  ///< 当前绝对时间（微秒）
  double time_us();

  /**
   * @brief 忙等指定时长（不依赖任何中断，临界区可用）
   *
   * @param seconds 时长（秒）；<= 0 立即返回，超过 7.809 s 按 32 位上限处理
   * @note 纯忙等不让出 CPU：毫秒级以上请用 vTaskDelay()
   */
  void delay(double seconds) const;

  // ----------------
private:
  // ---------------- 私有实现 ----------------

  ///< 回绕检测 + 累加，返回本次读到的 CYCCNT（绝对时间轴唯一的状态更新点）
  uint32_t update_timeline();

  ///< 按 RCC 算出 CPU 时钟（Hz）= SYSCLK / D1CPRE 分频
  static uint32_t cpu_clock_hz();

  // ---------------- 成员变量 ----------------

  uint32_t _cpu_hz   = 0U;    ///< CPU 时钟（Hz），即 CYCCNT 计数频率
  double   _round_s  = 0.0;   ///< 一圈回绕对应的秒数（2^32 / _cpu_hz）
  double   _base_s   = 0.0;   ///< 已完成回绕的累计秒数
  uint32_t _last_cnt = 0U;    ///< 上次回绕检测时的 CYCCNT 快照
  bool     _inited   = false; ///< 初始化成功标志

  // ----------------
};

#endif // __BSP_DWT_HPP__
