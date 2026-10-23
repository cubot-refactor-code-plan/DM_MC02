/**
 * @file corexy_protocol.cpp
 * @author Rh
 * @brief CoreXY 上位机串口协议实现：收帧 → 投递命令；应答 / 事件 → 发帧
 * @version 1.0
 * @date 2026-10-23
 *
 * @copyright Copyright (c) 2026
 */

#include "corexy_protocol.hpp"

#include "bsp_cfg.hpp" // bsp_uart1

#include "FreeRTOS.h" // IWYU pragma: keep
#include "queue.h"
#include "task.h" // IWYU pragma: keep

#include <string.h>


QueueHandle_t corexy_cmd_queue = nullptr;


bool corexy_cmd_init()
{
  corexy_cmd_queue = xQueueCreate(8U, sizeof(CorexyCmd));
  return corexy_cmd_queue != nullptr;
}


namespace
{
// ---------------- 收帧状态机 ----------------

uint8_t s_buf[cxy::PAYLOAD_MAX + 5U] = {}; ///< 头2 + CMD + LEN + 负载 + SUM
size_t  s_len                        = 0U;


uint16_t rd_u16(const uint8_t *p)
{
  return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

int16_t rd_i16(const uint8_t *p)
{
  return static_cast<int16_t>(rd_u16(p));
}

int32_t rd_i32(const uint8_t *p)
{
  return static_cast<int32_t>((static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
                              (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]));
}

///< 帧里的 centi（0.01 mm）→ mm
float to_mm(int32_t centi)
{
  return static_cast<float>(centi) / 100.0f;
}

///< 帧里的 mm → centi（四舍五入）
int16_t to_centi(float v)
{
  return static_cast<int16_t>((v >= 0.0f) ? (v * 100.0f + 0.5f) : (v * 100.0f - 0.5f));
}


/**
 * @brief 一帧完整请求 → 语义命令并投递
 * @note 负载长度不符的帧直接丢弃，不做补零容错。
 */
void dispatch(uint8_t cmd, const uint8_t *p, size_t len)
{
  CorexyCmd c;
  c.type = static_cast<CorexyCmdType>(cmd);

  switch (c.type)
  {
    case CorexyCmdType::Enable:
      if (len < 1U) { return; }
      c.p0 = (p[0] != 0U) ? 1 : 0;
      break;

    case CorexyCmdType::MoveAbs:
    case CorexyCmdType::MoveRel:
      if (len < 4U) { return; }
      c.a = to_mm(rd_i16(p));
      c.b = to_mm(rd_i16(p + 2U));
      break;

    case CorexyCmdType::SetSpeed:
      if (len < 3U) { return; }
      c.p0 = static_cast<int32_t>(rd_u16(p));
      c.p1 = static_cast<int32_t>(p[2]);
      break;

    case CorexyCmdType::QuickParam:
      if (len < 5U) { return; }
      c.addr = p[0];
      c.p0   = static_cast<int32_t>(rd_u16(p + 1U));
      c.p1   = static_cast<int32_t>(p[3]);
      c.mode = p[4];
      break;

    case CorexyCmdType::QuickPos:
      if (len < 5U) { return; }
      c.addr = p[0];
      c.p0   = rd_i32(p + 1U);
      break;

    case CorexyCmdType::Zero:
    case CorexyCmdType::Stop:
    case CorexyCmdType::Limits:
      break;

    case CorexyCmdType::MoveMode:
      if (len < 1U) { return; }
      c.p0 = static_cast<int32_t>(p[0]);
      break;

    default:
      return; // 未知命令：安静丢弃
  }

  (void)xQueueSend(corexy_cmd_queue, &c, 0);
}


// ----------------
// ---------------- 发帧 ----------------

///< 发一帧到 bsp_uart1
void send_frame(uint8_t cmd, const uint8_t *payload, size_t len)
{
  uint8_t      buf[cxy::PAYLOAD_MAX + 5U];
  const size_t total = len + 5U;

  buf[0] = cxy::HEAD0;
  buf[1] = cxy::HEAD1;
  buf[2] = cmd;
  buf[3] = static_cast<uint8_t>(len);

  memcpy(buf + 4U, payload, len);

  uint8_t sum = static_cast<uint8_t>(cmd ^ static_cast<uint8_t>(len));
  for (size_t i = 0U; i < len; i++)
  {
    sum = static_cast<uint8_t>(sum ^ payload[i]);
  }
  buf[total - 1U] = sum;

  (void)bsp_uart1.send(buf, total, pdMS_TO_TICKS(20U), nullptr);
}

} // namespace


namespace cxy
{

void feed(const uint8_t *data, size_t size)
{
  if ((data == nullptr) || (corexy_cmd_queue == nullptr))
  {
    return;
  }

  for (size_t i = 0U; i < size; i++)
  {
    const uint8_t b = data[i];

    if (s_len == 0U)
    {
      if (b == HEAD0) { s_buf[s_len++] = b; }
      continue;
    }

    if (s_len == 1U)
    {
      if (b == HEAD1) { s_buf[s_len++] = b; }
      else if (b != HEAD0) { s_len = 0U; } // 帧头错位，重新同步
      continue;
    }

    s_buf[s_len++] = b;

    if (s_len == 4U)
    {
      if (s_buf[3] > PAYLOAD_MAX) { s_len = 0U; } // 长度非法
      continue;
    }

    if (s_len != (static_cast<size_t>(s_buf[3]) + 5U))
    {
      continue; // 还没收满
    }

    const uint8_t cmd = s_buf[2];
    const size_t  len = s_buf[3];

    uint8_t sum = static_cast<uint8_t>(cmd ^ static_cast<uint8_t>(len));
    for (size_t k = 0U; k < len; k++)
    {
      sum = static_cast<uint8_t>(sum ^ s_buf[4U + k]);
    }

    if (sum == s_buf[s_len - 1U])
    {
      dispatch(cmd, s_buf + 4U, len);
    }

    s_len = 0U;
  }
}


void reply(uint8_t cmd, Status st, float x, float y)
{
  const int16_t xi = to_centi(x);
  const int16_t yi = to_centi(y);

  const uint8_t payload[5] = {
    static_cast<uint8_t>(st),
    static_cast<uint8_t>(static_cast<uint16_t>(xi) >> 8),
    static_cast<uint8_t>(static_cast<uint16_t>(xi)),
    static_cast<uint8_t>(static_cast<uint16_t>(yi) >> 8),
    static_cast<uint8_t>(static_cast<uint16_t>(yi))};

  send_frame(static_cast<uint8_t>(cmd | 0x80U), payload, sizeof(payload));
}


void reply_limits(Status st, const float th[4], const float sf[4])
{
  uint8_t payload[17];
  payload[0] = static_cast<uint8_t>(st);

  for (size_t i = 0U; i < 4U; i++)
  {
    const int16_t t = to_centi(th[i]);
    const int16_t s = to_centi(sf[i]);

    payload[1U + (2U * i)] = static_cast<uint8_t>(static_cast<uint16_t>(t) >> 8);
    payload[2U + (2U * i)] = static_cast<uint8_t>(static_cast<uint16_t>(t));
    payload[9U + (2U * i)] = static_cast<uint8_t>(static_cast<uint16_t>(s) >> 8);
    payload[10U + (2U * i)] = static_cast<uint8_t>(static_cast<uint16_t>(s));
  }

  send_frame(static_cast<uint8_t>(static_cast<uint8_t>(CorexyCmdType::Limits) | 0x80U), payload, sizeof(payload));
}


void event(uint8_t evt, Status st, float x, float y)
{
  const int16_t xi = to_centi(x);
  const int16_t yi = to_centi(y);

  const uint8_t payload[6] = {
    evt,
    static_cast<uint8_t>(st),
    static_cast<uint8_t>(static_cast<uint16_t>(xi) >> 8),
    static_cast<uint8_t>(static_cast<uint16_t>(xi)),
    static_cast<uint8_t>(static_cast<uint16_t>(yi) >> 8),
    static_cast<uint8_t>(static_cast<uint16_t>(yi))};

  send_frame(EVT_MARK, payload, sizeof(payload));
}

} // namespace cxy


// ----------------
