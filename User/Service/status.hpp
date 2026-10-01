/**
 * @file status.hpp
 * @author Rh
 * @brief 统一状态码 + 系统状态标志（0.3）
 * @version 0.3
 * @date 2026-10-01
 *
 * @copyright Copyright (c) 2026
 *
 * @details 所有对外 API 一律返回 Status（或返回数据 + Status）；
 *          void 只留给纯查询/纯动作（fire-and-forget）。
 *          调用处直接调用即可，不强制检查返回值。
 *
 * @note 系统状态标志用 FreeRTOS 事件组（原生 API）实现，不用 CMSIS-RTOS2。
 *       sys_flag_init() 必须最先调用；在此之前注册节点/创建任务都会失败。
 */

#ifndef __SERVICE_STATUS_HPP__
#define __SERVICE_STATUS_HPP__

#include <stdint.h>

#include "FreeRTOS.h" // IWYU pragma: keep
#include "event_groups.h"

extern EventGroupHandle_t sys_event; ///< 系统状态事件组（初始化前为 nullptr）

#define SYS_FLAG_INIT_FAIL_BIT (1U << 23) ///< 初始化失败标志位
#define SYS_FLAG_RUNNING_BIT   (1U << 22) ///< 系统已进入运行态标志位

/**
 * @brief 统一状态码
 */
enum class Status : uint8_t
{
  OK = 0,        ///< 成功
  BUSY,          ///< 资源被占用（DMA 正在发送等）
  TIMEOUT,       ///< 等待超时
  FULL,          ///< 缓冲满
  IO_ERROR,      ///< 硬件/传输错误
  BAD_ARG,       ///< 参数非法
  NOT_INIT,      ///< 资源未初始化（未调用 init 或创建失败）
  NOT_SUPPORTED, ///< 不支持的操作/模式
};

/**
 * @brief 创建系统状态事件组
 * @return OK=成功；FULL=创建失败
 */
Status sys_flag_init(void);

/**
 * @brief 置位某个失败状态并清掉 OK 位
 * @param statu 失败状态码（不允许传 OK）
 * @return OK=成功；BAD_ARG=传了 OK；NOT_INIT=事件组未创建
 */
Status sys_flag_set(Status statu);

/** @brief 初始化全部成功：置运行位（已有失败标志时不动） */
void sys_complete_init(void);

/** @brief 记一次初始化失败：清运行位 + 置失败位 */
void sys_init_error(void);

/**
 * @brief 记一次初始化失败并附上状态码
 * @param statu 失败状态码
 * @return sys_flag_set() 的结果
 */
Status sys_init_error(Status statu);

/**
 * @brief 等待某个状态位（不清标志）
 * @param statu 要等的状态码
 * @param timeout 等待时间（ticks），0=不等待，portMAX_DELAY=一直等
 * @return 等待结束时的状态位
 */
uint32_t sys_flag_wait(Status statu, uint32_t timeout);

/** @brief 阻塞等待系统进入运行态；事件组缺失或等待出错则结束当前任务 */
void sys_flag_wait_running(void);

/** @brief 当前是否已进入运行态 */
bool sys_flag_running(void);

#endif // __SERVICE_STATUS_HPP__
