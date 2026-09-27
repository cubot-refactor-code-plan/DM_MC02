/**
 * @file status.hpp
 * @author Rh
 * @brief 统一状态码（0.2）
 * @version 0.2
 * @date 2026-08-14
 *
 * @copyright Copyright (c) 2026
 *
 * @details 所有对外 API 一律返回 Status（或返回数据 + Status）；
 *          void 只留给纯查询/纯动作（fire-and-forget）。
 *          调用处直接调用即可，不强制检查返回值。
 */

#ifndef __SERVICE_STATUS_HPP__
#define __SERVICE_STATUS_HPP__

#include <stdint.h>

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


#endif // __SERVICE_STATUS_HPP__
