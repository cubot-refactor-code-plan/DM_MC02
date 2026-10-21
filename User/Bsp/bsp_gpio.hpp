/**
 * @file bsp_gpio.hpp
 * @author Rh
 * @brief GPIO 输出引脚封装
 * @version 0.5
 * @date 2026-09-10
 *
 * @copyright Copyright (c) 2026
 *
 * @details 仅封装电平控制；引脚方向/上下拉由 CubeMX 的 MX_GPIO_Init() 配置。
 *          构造时绑定端口+引脚，无需 init()。
 *
 * @note 接口：set() / reset() / toggle() / set_level(bool) / get_level()。
 *       实例见 bsp_cfg.cpp，引脚对应关系见 Docs/hardware/引脚分配与冲突.md。
 */

#ifndef __BSP_GPIO_HPP__
#define __BSP_GPIO_HPP__

#include "main.h" // IWYU pragma: keep
#include <stdint.h>


/**
 * @brief GPIO 输出引脚封装类
 *
 * @note 硬件引脚方向/上下拉由 CubeMX 的 MX_GPIO_Init() 配置，本类不做初始化。
 */
class BspGpio
{
public:
  // ---------------- 公有接口 ----------------

  /**
   * @brief GPIO 引脚配置（匿名按序传入：{port, pin}）
   */
  struct Config
  {
    /** @brief 按序构造配置（参数顺序 = 字段顺序） */
    Config(GPIO_TypeDef *port = nullptr, uint16_t pin = 0U) : port(port), pin(pin) {}

    GPIO_TypeDef *port; ///< GPIO 端口 (GPIOA / GPIOB / ...)
    uint16_t      pin;  ///< 引脚掩码 (GPIO_PIN_x)
  };

  /** @brief 构造函数：只做赋值，硬件已由 CubeMX 的 MX_GPIO_Init() 初始化 */
  BspGpio(const Config &cfg) : _port(cfg.port), _pin(cfg.pin) {}

  /** @brief 输出高电平 */
  void set() const { HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_SET); }

  /** @brief 输出低电平 */
  void reset() const { HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_RESET); }

  /** @brief 翻转电平 */
  void toggle() const { HAL_GPIO_TogglePin(_port, _pin); }

  /** @brief 输出指定电平 */
  void set_level(bool state) const { HAL_GPIO_WritePin(_port, _pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET); }

  /** @brief 读取当前引脚电平（true=高电平） */
  bool get_level() const { return (HAL_GPIO_ReadPin(_port, _pin) != GPIO_PIN_RESET); }

  // ----------------
private:
  // ---------------- 私有实现 ----------------

  GPIO_TypeDef *_port = nullptr; ///< GPIO 端口指针
  uint16_t      _pin  = 0U;      ///< 引脚掩码

  // ----------------
};

#endif // __BSP_GPIO_HPP__
