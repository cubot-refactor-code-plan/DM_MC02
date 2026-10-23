/**
 * @file corexy_console_task.cpp
 * @author Rh
 * @brief CoreXY 命令码终端任务：USART1 输入 ASCII 命令 → 投递到命令队列
 * @version 1.0
 * @date 2026-10-23
 *
 * @copyright Copyright (c) 2026
 *
 * @note 本任务**不碰电机总线**，只做「收行 → 解析 → 投递」，执行由 `corexy_io_task` 负责。
 *       与二进制协议任务 `corexy_task` 二选一，开关是 `app_task.hpp` 的 `APP_COREXY_PROTOCOL`。
 * @note 行结束符：CR、LF 或 `;`。`.` 就是小数点，不再兼作结束符。
 */

#include "app_task.hpp"

#include "bsp_cfg.hpp" // bsp_uart1
#include "corexy_protocol.hpp"
#include "device_cfg.hpp" // emm_group（诊断用）
#include "module_cfg.hpp"
#include "service_cfg.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>


namespace
{
constexpr size_t   LINE_MAX  = 64U;  ///< 单行上限
constexpr uint32_t PERIOD_MS = 10U;  ///< 收字符周期

char   g_line[LINE_MAX];
size_t g_len = 0U;


// ----------------
// ---------------- 解析小工具 ----------------

///< 取下一个空白分隔 token，无则返回 false
bool tok(const char *&p, char *out, size_t cap)
{
  while ((*p == ' ') || (*p == '\t'))
  {
    p++;
  }

  if (*p == '\0')
  {
    return false;
  }

  size_t n = 0U;
  while ((*p != ' ') && (*p != '\t') && (*p != '\0'))
  {
    if ((n + 1U) < cap)
    {
      out[n++] = *p;
    }
    p++;
  }

  out[n] = '\0';
  return true;
}

///< 整串十进制，必须完整合法
bool to_int(const char *s, long &v)
{
  char      *end = nullptr;
  const long t   = strtol(s, &end, 10);

  if ((end == s) || (end == nullptr) || (*end != '\0'))
  {
    return false;
  }

  v = t;
  return true;
}

///< 毫米（可带两位小数）→ 0.01 mm
bool to_centi(const char *s, int32_t &v)
{
  char        *end = nullptr;
  const double d   = strtod(s, &end);

  if ((end == s) || (end == nullptr) || (*end != '\0'))
  {
    return false;
  }

  v = static_cast<int32_t>((d * 100.0) + ((d >= 0.0) ? 0.5 : -0.5));
  return true;
}

void post(CorexyCmdType type, float a = 0.0f, float b = 0.0f, int32_t p0 = 0, int32_t p1 = 0, uint8_t addr = 0U, uint8_t mode = 0U)
{
  CorexyCmd cmd;
  cmd.type = type;
  cmd.a    = a;
  cmd.b    = b;
  cmd.p0   = p0;
  cmd.p1   = p1;
  cmd.addr = addr;
  cmd.mode = mode;

  if (xQueueSend(corexy_cmd_queue, &cmd, 0) != pdPASS)
  {
    bsp_uart1.printf("[CXY] busy: queue full\r\n");
  }
}

void usage(const char *text)
{
  bsp_uart1.printf("[CXY] usage: %s\r\n", text);
}


// ----------------
// ---------------- 命令表 ----------------

void print_help()
{
  bsp_uart1.printf("[CXY] m <x> <y>       绝对移动 (mm，最多两位小数)\r\n");
  bsp_uart1.printf("[CXY] j <dx> <dy>     相对移动 (mm)\r\n");
  bsp_uart1.printf("[CXY] g <0..7> [mm]   八方向单步 (0=+X，逆时针，默认 10 mm)\r\n");
  bsp_uart1.printf("[CXY] pos | lim | home | stop | dump\r\n");
  bsp_uart1.printf("[CXY] en <0|1>        失能 / 使能两台电机\r\n");
  bsp_uart1.printf("[CXY] spd <rpm> <acc> 速度 (1..5000, 0..255)\r\n");
  bsp_uart1.printf("[CXY] mode <0|1>      0=位置帧(0xFD) 1=快速位置帧(0xFC)\r\n");
  bsp_uart1.printf("[CXY] qp <1|2> <rel|abs> <rpm> <acc>   快速位置参数(0xF1)\r\n");
  bsp_uart1.printf("[CXY] q <1|2> <pulses>                 快速位置(0xFC，带符号)\r\n");
}

///< 打印坐标与状态
void print_pos()
{
  const CorexyModule::Point p = corexy.position();

  bsp_uart1.printf("[CXY] pos=(%.2f, %.2f) c1=%ld c2=%ld state=%u seg=%u/%u err=%u origin=%u\r\n", p.x, p.y,
                   static_cast<long>(corexy.raw_counts(0U)), static_cast<long>(corexy.raw_counts(1U)),
                   static_cast<unsigned>(corexy.state()), static_cast<unsigned>(corexy.segment_index()),
                   static_cast<unsigned>(corexy.segment_count()), static_cast<unsigned>(corexy.last_error()),
                   corexy.origin_verified() ? 1U : 0U);
}

///< 打印最近一条 MMCL 帧与两台在线状态（诊断“帧发出去没 / 谁没应答”）
void print_dump()
{
  size_t         len = 0U;
  const uint8_t *f   = emm_group.last_multi_frame(len);

  bsp_uart1.printf("[CXY] mmcl(%u):", static_cast<unsigned>(len));

  for (size_t i = 0U; i < len; i++)
  {
    bsp_uart1.printf(" %02X", f[i]);
  }

  bsp_uart1.printf("\r\n[CXY] online 1=%u 2=%u\r\n", emm_group.is_online(1U) ? 1U : 0U,
                   emm_group.is_online(2U) ? 1U : 0U);
}

///< 解析并投递一条命令行
void console_exec(const char *text)
{
  const char *p = text;
  char        c[8];

  if (!tok(p, c, sizeof(c)))
  {
    return;
  }

  if ((strcmp(c, "?") == 0) || (strcmp(c, "help") == 0))
  {
    print_help();
    return;
  }

  if (strcmp(c, "pos") == 0)
  {
    print_pos();
    return;
  }

  if (strcmp(c, "dump") == 0)
  {
    print_dump();
    return;
  }

  if (strcmp(c, "lim") == 0)
  {
    post(CorexyCmdType::Limits);
    return;
  }

  if ((strcmp(c, "home") == 0) || (strcmp(c, "zero") == 0))
  {
    post(CorexyCmdType::Zero);
    return;
  }

  if (strcmp(c, "stop") == 0)
  {
    post(CorexyCmdType::Stop);
    return;
  }

  if ((strcmp(c, "m") == 0) || (strcmp(c, "j") == 0))
  {
    char    sx[16];
    char    sy[16];
    int32_t cx = 0;
    int32_t cy = 0;

    if (!tok(p, sx, sizeof(sx)) || !tok(p, sy, sizeof(sy)) || !to_centi(sx, cx) || !to_centi(sy, cy))
    {
      usage((strcmp(c, "m") == 0) ? "m <x> <y>" : "j <dx> <dy>");
      return;
    }

    post((strcmp(c, "m") == 0) ? CorexyCmdType::MoveAbs : CorexyCmdType::MoveRel,
         static_cast<float>(cx) / 100.0f, static_cast<float>(cy) / 100.0f);
    return;
  }

  if (strcmp(c, "g") == 0)
  {
    // 0=+X，逆时针每档 45°
    static const float DX[8] = {1.0f, 0.7071f, 0.0f, -0.7071f, -1.0f, -0.7071f, 0.0f, 0.7071f};
    static const float DY[8] = {0.0f, 0.7071f, 1.0f, 0.7071f, 0.0f, -0.7071f, -1.0f, -0.7071f};

    char    sd[8];
    char    sm[16];
    long    dir = 0;
    int32_t cmm = 1000; // 默认 10 mm

    if (!tok(p, sd, sizeof(sd)) || !to_int(sd, dir) || (dir < 0) || (dir > 7))
    {
      usage("g <0..7> [mm]");
      return;
    }

    if (tok(p, sm, sizeof(sm)) && (!to_centi(sm, cmm)))
    {
      usage("g <0..7> [mm]");
      return;
    }

    const float d = static_cast<float>(cmm) / 100.0f;
    post(CorexyCmdType::MoveRel, d * DX[dir], d * DY[dir]);
    return;
  }

  if (strcmp(c, "spd") == 0)
  {
    char sr[16];
    char sa[16];
    long rpm = 0;
    long acc = 0;

    if (!tok(p, sr, sizeof(sr)) || !tok(p, sa, sizeof(sa)) || !to_int(sr, rpm) || !to_int(sa, acc) || (rpm < 1) || (rpm > 5000) || (acc < 0) || (acc > 255))
    {
      usage("spd <rpm 1..5000> <acc 0..255>");
      return;
    }

    post(CorexyCmdType::SetSpeed, 0.0f, 0.0f, static_cast<int32_t>(rpm), static_cast<int32_t>(acc));
    return;
  }

  if (strcmp(c, "en") == 0)
  {
    char son[8];
    long on = 0;

    if (!tok(p, son, sizeof(son)) || !to_int(son, on) || ((on != 0) && (on != 1)))
    {
      usage("en <0|1>");
      return;
    }

    post(CorexyCmdType::Enable, 0.0f, 0.0f, static_cast<int32_t>(on));
    return;
  }

  if (strcmp(c, "mode") == 0)
  {
    char sm[8];
    long m = 0;

    if (!tok(p, sm, sizeof(sm)) || !to_int(sm, m) || (m < 0) || (m > 1))
    {
      usage("mode <0|1>  (0=位置帧 1=快速位置帧)");
      return;
    }

    post(CorexyCmdType::MoveMode, 0.0f, 0.0f, static_cast<int32_t>(m));
    return;
  }

  if (strcmp(c, "qp") == 0)
  {
    char sa[8];
    char sm[8];
    char sr[16];
    char sc[16];
    long addr = 0;
    long rpm  = 0;
    long acc  = 0;

    if (!tok(p, sa, sizeof(sa)) || !tok(p, sm, sizeof(sm)) || !tok(p, sr, sizeof(sr)) || !tok(p, sc, sizeof(sc)) || !to_int(sa, addr) || !to_int(sr, rpm) || !to_int(sc, acc) || (addr < 1) || (addr > 2))
    {
      usage("qp <1|2> <rel|abs> <rpm> <acc>");
      return;
    }

    uint8_t mode = 0U;
    if (strcmp(sm, "rel") == 0)
    {
      mode = 0U;
    }
    else if (strcmp(sm, "abs") == 0)
    {
      mode = 1U;
    }
    else
    {
      usage("qp <1|2> <rel|abs> <rpm> <acc>");
      return;
    }

    post(CorexyCmdType::QuickParam, 0.0f, 0.0f, static_cast<int32_t>(rpm), static_cast<int32_t>(acc),
         static_cast<uint8_t>(addr), mode);
    return;
  }

  if (strcmp(c, "q") == 0)
  {
    char sa[8];
    char sp[16];
    long addr   = 0;
    long pulses = 0;

    if (!tok(p, sa, sizeof(sa)) || !tok(p, sp, sizeof(sp)) || !to_int(sa, addr) || !to_int(sp, pulses) || (addr < 1) || (addr > 2))
    {
      usage("q <1|2> <pulses>");
      return;
    }

    post(CorexyCmdType::QuickPos, 0.0f, 0.0f, static_cast<int32_t>(pulses), 0, static_cast<uint8_t>(addr));
    return;
  }

  bsp_uart1.printf("[CXY] unknown: %s (try ?)\r\n", c);
}


// ----------------
// ---------------- 行读取 ----------------

///< 提交当前行
void flush_line()
{
  if (g_len == 0U)
  {
    return;
  }

  g_line[g_len] = '\0';
  g_len         = 0U;

  bsp_uart1.printf("\r\n");
  console_exec(g_line);
}

///< 非阻塞收字符：回显、退格、按行提交
void read_console()
{
  uint8_t chunk[16];

  for (uint32_t round = 0U; round < 4U; round++)
  {
    size_t n = 0U;

    if ((bsp_uart1.receive(chunk, sizeof(chunk), 0U, &n) != Status::OK) || (n == 0U))
    {
      return;
    }

    for (size_t i = 0U; i < n; i++)
    {
      const char ch = static_cast<char>(chunk[i]);

      if ((ch == '\r') || (ch == '\n') || (ch == ';'))
      {
        flush_line();
      }
      else if ((ch == '\b') || (ch == 0x7F))
      {
        if (g_len > 0U)
        {
          g_len--;
          bsp_uart1.printf("\b \b");
        }
      }
      else if ((ch >= 0x20) && (ch < 0x7F))
      {
        if ((g_len + 1U) < LINE_MAX)
        {
          g_line[g_len++] = ch;
          bsp_uart1.printf("%c", ch);
        }
      }
      else
      {
        // 其他控制字符忽略
      }
    }
  }
}

} // namespace


extern "C" void corexy_console_task(void *argument)
{
  (void)argument;
  sys_state.wait_running();

  vTaskDelay(pdMS_TO_TICKS(500U));
  bsp_uart1.printf("\r\n[CXY] console ready, type ? for help\r\n");

  for (;;)
  {
    read_console();
    vTaskDelay(pdMS_TO_TICKS(PERIOD_MS));
  }
}
