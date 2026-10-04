/**
 * @file bsp_pwm.hpp
 * @author Rh
 * @brief PWM 通道驱动 —— 参数全部由 Config 手动传入（驱动内无板级常量），上电默认 0% 占空比
 * @version 0.1
 * @date 2026-10-02
 *
 * @copyright Copyright (c) 2026
 *
 * @details 一个实例 = 一个定时器的一个 PWM 通道，只负责「设占空比 / 设脉宽 / 设频率 / 关断」。
 *          定时器时钟、PSC、ARR 全部由使用方从 CubeMX 生成的配置里抄进 Config，
 *          驱动本身不写死任何硬件常量，换板子只改实例化处的参数。
 *
 * @note 本板实例（见 bsp_cfg.cpp，参数取自 CubeMX；定时器时钟 275 MHz = APB1/APB2 137.5 MHz × 2）：
 *
 *       bsp_pwm1        TIM1_CH3   PE13   排针预留（舵机）
 *       bsp_pwm2        TIM1_CH1   PE9    排针预留（舵机）
 *       bsp_pwm3        TIM2_CH3   PA2    排针预留（舵机）
 *       bsp_pwm4        TIM2_CH1   PA0    排针预留（舵机）
 *       bsp_pwm_gyro    TIM3_CH4   PB1    陀螺仪
 *       bsp_pwm_buzzer  TIM12_CH2  PB15   无源蜂鸣器（由 Device 层 DeviceBuzzer 使用）
 *
 * @note 用法（占空比是 0~100 的浮点值；脉宽接口对舵机更直观）：
 *
 *       bsp_pwm1.set_duty(7.5f);         // 占空比 7.5%
 *       bsp_pwm1.set_pulse_us(1500.0f);  // 舵机中位：1.5 ms 高电平
 *       bsp_pwm1.off();                  // 关断（CCR=0，引脚保持低电平）
 *
 * @warning 同一定时器的多个通道共用计数器与 ARR：set_freq() 会同时改变该定时器
 *          所有通道的频率（例如 TIM1 的 PE9 与 PE13 必须同频）。
 * @warning PSC 由 CubeMX 的 MX_TIMx_Init() 写入，本类只用 Config::prescaler 做换算、
 *          不改 PSC 寄存器；该值必须与 CubeMX 保持一致，否则脉宽/频率换算是错的。
 * @warning 内部含浮点运算（M7 有 FPU）：只在任务上下文调用，不要放进 ISR。
 */

#ifndef __BSP_PWM_HPP__
#define __BSP_PWM_HPP__

#include "status.hpp" // 统一状态码
#include "tim.h"      // IWYU pragma: keep（TIM_HandleTypeDef / TIM_CHANNEL_x）

#include <stdint.h>

/**
 * @brief PWM 通道驱动类
 *
 * @note 只碰 CCR / ARR 两个寄存器：不创建 RTOS 对象，不使用中断。
 */
class BspPwm
{
public:
  // ---------------- 配置 ----------------

  /**
   * @brief PWM 通道配置（全部是可手动传入的 CubeMX 参数，无隐含默认硬件）
   */
  struct Config
  {
    /**
     * @brief 按序构造配置（参数顺序 = 字段顺序，可匿名传入）
     */
    Config(TIM_HandleTypeDef *htim = nullptr, uint32_t channel = 0U, uint32_t timer_clk_hz = 0U, uint32_t prescaler = 0U, uint32_t period = 0U, float duty_pct = 0.0f) : htim(htim), channel(channel), timer_clk_hz(timer_clk_hz), prescaler(prescaler), period(period), duty_pct(duty_pct)
    {
    }

    TIM_HandleTypeDef *htim;         ///< 定时器句柄
    uint32_t           channel;      ///< PWM 通道（TIM_CHANNEL_x）
    uint32_t           timer_clk_hz; ///< 定时器输入时钟 (Hz)：计数器时钟 = timer_clk_hz / (prescaler + 1)
    uint32_t           prescaler;    ///< PSC 寄存器值（CubeMX 值，分频 = prescaler + 1）
    uint32_t           period;       ///< ARR 寄存器值（CubeMX 值，一个周期 = period + 1 个计数）
    float              duty_pct;     ///< 上电占空比 (%)，0~100，默认 0
  };

  // ----------------
  // ---------------- 公共接口 ----------------

  /** @brief 默认构造：未绑定通道，须再调 init() */
  BspPwm() = default;

  /** @brief 只保存配置，硬件操作全部推迟到 init() */
  explicit BspPwm(const Config &cfg);

  /** @brief 默认析构 */
  ~BspPwm() = default;

  /**
   * @brief 先写 CCR=0（保证上电 0% 占空比）再启动 PWM 输出
   *
   * @note 幂等；按 Config::period 同步一次 ARR，之后由 set_freq() 动态调整。
   *
   * @return OK=成功；BAD_ARG=配置非法（句柄/通道/时钟为空，PSC 超范围）；IO_ERROR=PWM 启动失败
   */
  Status init();

  /**
   * @brief 设置占空比
   * @param duty_pct 占空比 (%)，0~100 的浮点值，超出自动限幅
   */
  void set_duty(float duty_pct);

  /**
   * @brief 按高电平脉宽设置占空比（舵机等按时序表达的器件更直观）
   * @param width_us 高电平脉宽 (us)，超过一个周期时限幅到 100%
   */
  void set_pulse_us(float width_us);

  /**
   * @brief 动态改变频率（通过 ARR 实现，PSC 不动）
   * @param freq_hz 目标频率 (Hz)；0 或未初始化时忽略
   *
   * @note 会改变该定时器所有通道的频率；占空比按新周期等比例保持。
   */
  void set_freq(uint32_t freq_hz);

  /** @brief 关断输出（CCR=0，PWM 仍在运行、引脚保持低电平） */
  void off();

  /** @brief 当前生效的占空比 (%) */
  float duty_pct() const;

  /** @brief 计数器时钟 (Hz) = timer_clk_hz / (prescaler + 1) */
  uint32_t counter_clk_hz() const;

  /** @brief 当前一个周期的计数个数（ARR + 1） */
  uint32_t period_cnt() const;

  /** @brief 当前频率 (Hz) = 计数器时钟 / 周期计数 */
  uint32_t freq_hz() const;

  // ----------------
private:
  // ---------------- 私有方法 ----------------

  /** @brief 把 _duty_pct 换算成 CCR 并写寄存器（含限幅与实际值回写） */
  void _apply_duty();

  // ----------------
  // ---------------- 成员变量 ----------------

  Config   _config;              ///< 通道配置（CubeMX 参数）
  uint32_t _counter_clk_hz = 0U; ///< 计数器时钟（init() 算一次）
  uint32_t _period = 0U;         ///< 当前 ARR（set_freq() 会改）
  float    _duty_pct = 0.0f;     ///< 当前生效占空比 (%)
  bool     _initialized = false; ///< 是否已成功 init()

  // ----------------
};

#endif // __BSP_PWM_HPP__
