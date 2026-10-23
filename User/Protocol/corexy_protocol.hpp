/**
 * @file corexy_protocol.hpp
 * @author Rh
 * @brief CoreXY 上位机串口协议：命令队列 + 帧编解码 + 应答/事件上报
 * @version 1.0
 * @date 2026-10-23
 *
 * @copyright Copyright (c) 2026
 *
 * @note 帧：`A5 5A | CMD | LEN | PAYLOAD | SUM`，SUM = CMD ^ LEN ^ payload 各字节。
 *       应答用 `CMD | 0x80`，事件用 `0x7F`。命令表与时序见 `corexy.md` §6。
 * @note 本文件的上行发送直接绑定 `bsp_uart1`（协议适配层，不追求可移植）。
 */

#ifndef __COREXY_PROTOCOL_HPP__
#define __COREXY_PROTOCOL_HPP__

#include "status.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "queue.h"

#include <stddef.h>
#include <stdint.h>


// ---------------- 命令（队列元素，取值 = 协议命令码） ----------------

/** @brief 上位机可下发的命令；枚举值直接就是帧里的 CMD */
enum class CorexyCmdType : uint8_t
{
  Enable     = 0x01, ///< p0 = 1 使能 / 0 失能
  MoveAbs    = 0x02, ///< a,b = 目标 X/Y（mm）
  MoveRel    = 0x03, ///< a,b = 增量 X/Y（mm）
  Zero       = 0x04, ///< 当前位置记为原点
  Stop       = 0x05, ///< 立即停止并清故障
  SetSpeed   = 0x06, ///< p0 = rpm，p1 = acc
  QuickParam = 0x07, ///< addr，p0 = rpm，p1 = acc，mode = 0 相对 / 1 绝对
  QuickPos   = 0x08, ///< addr，p0 = 脉冲
  Limits     = 0x09, ///< 查询理论 / 安全限位
  MoveMode   = 0x0A  ///< p0 = 0 位置帧 / 1 快速位置帧
};

/** @brief 队列元素：语义命令 */
struct CorexyCmd
{
  CorexyCmdType type = CorexyCmdType::Stop;
  float         a    = 0.0f;  ///< X（mm）
  float         b    = 0.0f;  ///< Y（mm）
  int32_t       p0   = 0;     ///< 开关 / rpm / 脉冲
  int32_t       p1   = 0;     ///< 加速度
  uint8_t       addr = 0U;    ///< 电机地址
  uint8_t       mode = 0U;    ///< 快速位置：0 相对 / 1 绝对
};

extern QueueHandle_t corexy_cmd_queue;

///< 创建命令队列（在 all_init() 里、建任务之前调一次）
bool corexy_cmd_init();


// ----------------
// ---------------- 帧 ----------------

namespace cxy
{
constexpr uint8_t HEAD0       = 0xA5U; ///< 帧头 1
constexpr uint8_t HEAD1       = 0x5AU; ///< 帧头 2
constexpr uint8_t EVT_MARK    = 0x7FU; ///< 事件帧命令码
constexpr size_t  PAYLOAD_MAX = 24U;   ///< 请求负载上限

/** @brief 事件码 */
enum Event : uint8_t
{
  BOOT  = 0x01, ///< 上电自检完成
  DONE  = 0x02, ///< 一段移动完成
  FAULT = 0x03  ///< 进入 Fault（status 带原因）
};

///< 喂字节：解析出的命令投递到 corexy_cmd_queue
void feed(const uint8_t *data, size_t size);

///< 回一帧应答：CMD|0x80，负载 [status][x][y]
void reply(uint8_t cmd, Status st, float x, float y);

///< 限位应答：CMD|0x80，负载 [status][理论 4×i16][安全 4×i16]
void reply_limits(Status st, const float th[4], const float sf[4]);

///< 上报事件：0x7F，负载 [evt][status][x][y]
void event(uint8_t evt, Status st, float x, float y);

} // namespace cxy


// ----------------

#endif // __COREXY_PROTOCOL_HPP__
