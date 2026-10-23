/**
 * @file bsp_dwt.hpp
 * @author Rh
 * @brief DWT 计时驱动 —— Cortex-M 内核 CYCCNT 自由运行计数器
 * @version 0.2
 * @date 2026-10-23
 *
 * @copyright Copyright (c) 2026
 *
 * @note CYCCNT 不占外设、不依赖中断，关中断期间仍可计时；分辨率 = 1 个 CPU 周期。
 *       CPU 频率由 init() 从 RCC 自动推算，不设 Config。
 *       回绕周期、精度对比与调用约束见 bsp_dwt.md。
 *
 * @warning get_delta_s() 的时间戳由调用方保存，同一个 last 不要跨任务共用。
 * @warning CPU 进入 sleep/stop 时停止计数；32 位计数器会周期性回绕。
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

  /** @brief 默认构造（未初始化，须自行调 init()） */
  BspDwt() = default;

  /**
   * @brief 使能 CYCCNT 周期计数（幂等，可重复调用）
   *
   * @note 顺序：DEMCR.TRCENA（总开关，脱机运行时要自己置位，不能依赖调试器）
   *            → DWT.LAR 解锁（Cortex-M7 的 CoreSight 软件锁）
   *            → CYCCNT 清零 → CYCCNTENA 使能 → 等约 8 周期稳定 → 再清零
   *
   * @note 时间基准 = CPU 时钟 = SYSCLK / D1CPRE（由 init() 从 RCC 读出）。
   *
   * @return Status OK=可用；IO_ERROR=本内核无 CYCCNT（CTRL.NOCYCCNT=1）或使能未生效
   */
  Status init();

  /**
   * @brief 与上一次调用之间的时间间隔（秒）
   *
   * @param last 时间戳，首次调用传入的值无意义（第一次结果要丢弃）
   * @return 间隔（秒）；两次调用间隔 < 7.809 s 时结果严格正确
   *
   * @note 无状态、回绕安全（无符号差值），可在 ISR / 临界区中调用
   */
  double get_delta_s(uint32_t *last) const;

  /** @brief 当前绝对时间（秒，自 init() 起算）—— 会推进内部累加量，仅任务上下文调用 */
  double get_time_s();

  /** @brief 当前绝对时间（毫秒）—— 会推进内部累加量，仅任务上下文调用 */
  double get_time_ms();

  /** @brief 当前绝对时间（微秒）—— 会推进内部累加量，仅任务上下文调用 */
  double get_time_us();

  /**
   * @brief 忙等指定纳秒（纯忙等，不让出 CPU）
   * @param ns 纳秒
   * @note 分辨率 = 1 个 CPU 周期（本工程 1.818 ns）：传入不足 1 个周期时不会等待
   */
  void delay_ns(double ns) const;

  /**
   * @brief 忙等指定微秒（纯忙等，不让出 CPU）
   * @param us 微秒
   */
  void delay_us(double us) const;

  /**
   * @brief 忙等指定毫秒（纯忙等，不让出 CPU）
   * @param ms 毫秒
   * @note 毫秒级以上的等待建议改用 vTaskDelay()，不要占着 CPU 空转
   */
  void delay_ms(double ms) const;

  /**
   * @brief 忙等指定秒（纯忙等，不让出 CPU）
   * @param s 秒
   * @note 单次上限 ≈7.809 s（32 位计数器一圈）；这么长的等待应该用 vTaskDelay()
   */
  void delay_s(double s) const;

  // ----------------
private:
  // ---------------- 私有实现 ----------------

  /** @brief 回绕检测 + 累加，返回本次读到的 CYCCNT（绝对时间轴唯一的状态更新点） */
  uint32_t update_timeline();

  /** @brief 四个 delay_* 的公共实现：忙等 cycles 个 CPU 周期 */
  void wait_cycles(double cycles) const;

  /** @brief 按 RCC 算出 CPU 时钟（Hz）= SYSCLK / D1CPRE 分频 */
  static uint32_t get_cpu_clock_hz();

  // 成员变量

  uint32_t _cpu_hz   = 0U;    ///< CPU 时钟（Hz），即 CYCCNT 计数频率
  double   _round_s  = 0.0;   ///< 一圈回绕对应的秒数（2^32 / _cpu_hz）
  double   _base_s   = 0.0;   ///< 已完成回绕的累计秒数
  uint32_t _last_cnt = 0U;    ///< 上次回绕检测时的 CYCCNT 快照
  bool     _inited   = false; ///< 初始化成功标志

  // ----------------
};

#endif // __BSP_DWT_HPP__
