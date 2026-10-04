#include "app_test.hpp"

#if APP_TEST_CAN3_DEVICE_ENABLED
#include "bsp_cfg.hpp"     // bsp_can3.diagnostics / bsp_uart1
#include "device_cfg.hpp"  // m2006 / gm6020
#include "main.h"          // IWYU pragma: keep (PI)
#include "service_cfg.hpp" // bus_can3
#include "status.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include <cmath>

// CAN3 电机：M2006 开环正弦电流往复（本轮 GM6020 不转，指令恒 0）
//
//   上电 ──► M2006 按正弦电流正反往复（GM6020 保持 0）──► 串口输入任意字符停止
//                                                                 │
//                                                                 └──► 打印汇总，再输入重新开始
//
// 开环：电流指令直接按正弦给出，**不用转速反馈**：
//
//   cmd(t) = AMP · sin(2π·f·t)     ← 电流正负交替，电机随之正反往复
//
// 只看两件事：电机能不能转（`rpm` 是否有明显摆动）、方向是否跟着电流符号走。
// 转速只做兜底（1000 rpm，等于“不限制”），不做跟踪。

namespace
{
// ---------------- 测试参数 ----------------

constexpr int16_t CMD_AMP      = 3000;   ///< 开环正弦电流幅值（M2006 原始指令，满限幅 10000 的 30%）
constexpr float   CMD_SINE_HZ  = 0.25f;  ///< 正弦频率 (Hz)：4 s 一个来回（电流正负交替）
constexpr int16_t CMD_SLEW_PER_MS = 20;  ///< 电流指令每 ms 最大变化量（抑制阶跃）
constexpr float RPM_LIMIT_OUT  = 1000.0f;///< 输出轴转速兜底 (rpm)：设得很大即“不限制”
constexpr uint32_t PRINT_GAP_MS = 5U;    ///< 每次打印后留出的排空时间 (ms)

/** @brief 打印后留出排空时间（串口 TX 流缓冲小，连续快速 printf 会挤爆缓冲） */
void print_gap()
{
  vTaskDelay(pdMS_TO_TICKS(PRINT_GAP_MS));
}

/**
 * @brief 电流指令斜率限制：每步最多变化 step
 *
 * @note 抑制正弦过零 / 换向那一瞬的电流阶跃。
 */
int16_t slew_limit(int16_t target, int16_t prev, int16_t step)
{
  int32_t delta = static_cast<int32_t>(target) - static_cast<int32_t>(prev);
  if (delta > step)
    delta = step;
  if (delta < -step)
    delta = -step;

  return static_cast<int16_t>(static_cast<int32_t>(prev) + delta);
}

/** @brief 输出轴角速度 (rad/s) → 输出轴转速 (rpm) */
float rad_s_to_rpm(float w)
{
  return w * 60.0f / (2.0f * PI);
}

/** @brief 一次采样的快照 */
struct Snapshot
{
  float   m2006_rpm;
  float   gm6020_rpm;
  int16_t m2006_given;
  int16_t gm6020_given;
  uint8_t m2006_temp;
  uint8_t gm6020_temp;
  bool    m2006_online;
  bool    gm6020_online;
};

/** @brief 取一次观测快照（临界区取数据，浮点换算放外面） */
Snapshot sample()
{
  Snapshot s = {};

  float m2006_w  = 0.0f;
  float gm6020_w = 0.0f;

  taskENTER_CRITICAL();
  m2006_w        = m2006.data().radian_data.velocity;
  gm6020_w       = gm6020.data().radian_data.velocity;
  s.m2006_given  = m2006.raw_data().torque_current;
  s.gm6020_given = gm6020.raw_data().torque_current;
  s.m2006_temp   = m2006.raw_data().temperature;
  s.gm6020_temp  = gm6020.raw_data().temperature;
  taskEXIT_CRITICAL();

  s.m2006_rpm     = rad_s_to_rpm(m2006_w);
  s.gm6020_rpm    = rad_s_to_rpm(gm6020_w);
  s.m2006_online  = (m2006.online().is_online() == Status::OK);
  s.gm6020_online = (gm6020.online().is_online() == Status::OK);

  return s;
}

/** @brief 同时给两个电机下指令（0 = 撤电流） */
bool output_all(int16_t m2006_current, int16_t gm6020_current)
{
  const bool ok_m2006  = (m2006.fill_data(m2006_current) == Status::OK);
  const bool ok_gm6020 = (gm6020.fill_data(gm6020_current) == Status::OK);

  return ok_m2006 && ok_gm6020;
}

/** @brief 取走回退缓冲里的帧（分发未命中的帧会落到这里） */
void drain_fallback()
{
  CanRxMsg rx = {};
  for (uint32_t i = 0U; i < 8U && bus_can3.receive(&rx, 0) == Status::OK; ++i)
  {
  }
}

/** @brief 本轮旋转的统计 */
struct RoundStat
{
  uint32_t ms;        ///< 运行时长 (ms)
  uint32_t limit_hits;///< 触发 60 rpm 保护次数
  float    m_rpm_min;
  float    m_rpm_max;
  float    g_rpm_min;
  float    g_rpm_max;
  int16_t  m_given_min;
  int16_t  m_given_max;
  int16_t  g_given_min;
  int16_t  g_given_max;
  uint8_t  m_temp;
  uint8_t  g_temp;
};
} // namespace

// ---------------- 任务 ----------------

extern "C" void can3_device_test_task(void *argument)
{
  (void)argument;
  static_assert(configTICK_RATE_HZ == 1000U, "CAN3 device test requires 1 kHz tick");

  sys_flag_wait_running();

  // 上电先归零
  (void)output_all(0, 0);

  bsp_uart1.printf("\r\n[CAN3-MOTOR] M2006 open-loop sine: cmd %+d..%+d, %.2f Hz, slew %d/ms (GM6020 held at 0)\r\n",
                   -static_cast<int>(CMD_AMP), static_cast<int>(CMD_AMP), static_cast<double>(CMD_SINE_HZ),
                   static_cast<int>(CMD_SLEW_PER_MS));
  print_gap();

  if (m2006.statu() != Status::OK || gm6020.statu() != Status::OK)
  {
    bsp_uart1.printf("[CAN3-MOTOR] init FAIL: m2006=%u gm6020=%u\r\n", static_cast<unsigned>(m2006.statu()),
                     static_cast<unsigned>(gm6020.statu()));
    print_gap();
  }

  uint8_t ch = 0U;
  size_t  n  = 0U;

  for (;;)
  {
    // 丢掉上一轮残留的串口输入，避免"刚启动就被停"
    while (bsp_uart1.receive(&ch, 1U, 0U, &n) == Status::OK && n > 0U)
    {
    }

    bsp_uart1.printf("[CAN3-MOTOR] ===== spinning, send any char to stop =====\r\n");
    print_gap();

    RoundStat st = {};
    st.m_rpm_min = 1.0e9f;
    st.m_rpm_max = -1.0e9f;
    st.g_rpm_min = 1.0e9f;
    st.g_rpm_max = -1.0e9f;
    st.m_given_min = 32767;
    st.m_given_max = -32768;
    st.g_given_min = 32767;
    st.g_given_max = -32768;

    bool     stop_req = false;
    uint32_t stop_fail = 0U;
    int16_t  m_cmd_prev = 0;

    TickType_t wake = xTaskGetTickCount();

    while (!stop_req)
    {
      const Snapshot s = sample();
      drain_fallback();

      // 反馈必须一直在：掉线就停
      if (!s.m2006_online || !s.gm6020_online)
      {
        stop_fail = 2U;
        break;
      }
      if (bsp_can3.diagnostics.bus_off_events != 0U)
      {
        stop_fail = 7U;
        break;
      }

      // 开环：直接给正弦电流（不用转速反馈），电流正负交替 → 电机正反往复
      const float   t         = static_cast<float>(st.ms) * 0.001f;
      const float   phase     = 2.0f * PI * CMD_SINE_HZ * t;
      const int16_t m_cmd_raw = static_cast<int16_t>(CMD_AMP * sinf(phase));
      int16_t       m_cmd     = slew_limit(m_cmd_raw, m_cmd_prev, CMD_SLEW_PER_MS);
      m_cmd_prev              = m_cmd;
      const int16_t g_cmd     = 0;

      // 转速兜底（现在设得很大 = 不限制）：任何一路超过就撤电流，只记次数
      if ((std::fabs(s.m2006_rpm) > RPM_LIMIT_OUT) || (std::fabs(s.gm6020_rpm) > RPM_LIMIT_OUT))
      {
        ++st.limit_hits;
        m_cmd = 0; // g_cmd 恒 0，不用撤
      }

      if (!output_all(m_cmd, g_cmd))
      {
        stop_fail = 3U;
        break;
      }

      // 统计
      if (s.m2006_rpm < st.m_rpm_min)
        st.m_rpm_min = s.m2006_rpm;
      if (s.m2006_rpm > st.m_rpm_max)
        st.m_rpm_max = s.m2006_rpm;
      if (s.gm6020_rpm < st.g_rpm_min)
        st.g_rpm_min = s.gm6020_rpm;
      if (s.gm6020_rpm > st.g_rpm_max)
        st.g_rpm_max = s.gm6020_rpm;
      if (s.m2006_given < st.m_given_min)
        st.m_given_min = s.m2006_given;
      if (s.m2006_given > st.m_given_max)
        st.m_given_max = s.m2006_given;
      if (s.gm6020_given < st.g_given_min)
        st.g_given_min = s.gm6020_given;
      if (s.gm6020_given > st.g_given_max)
        st.g_given_max = s.gm6020_given;
      st.m_temp = s.m2006_temp;
      st.g_temp = s.gm6020_temp;

      ++st.ms;

      // 每 10 ms 看一次串口：收到任意字符就停
      if ((st.ms % 10U) == 0U)
      {
        if (bsp_uart1.receive(&ch, 1U, 0U, &n) == Status::OK && n > 0U)
          stop_req = true;
      }

      vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
    }

    // 停机：撤电流
    (void)output_all(0, 0);

    bsp_uart1.printf("[CAN3-MOTOR] stopped: ran=%u ms, limit_hits=%u, fail=%u\r\n",
                     static_cast<unsigned>(st.ms), static_cast<unsigned>(st.limit_hits), static_cast<unsigned>(stop_fail));
    print_gap();
    bsp_uart1.printf("[CAN3-MOTOR] M2006 : rpm[%+.1f..%+.1f] given[%+d..%+d] temp=%u\r\n",
                     static_cast<double>(st.m_rpm_min), static_cast<double>(st.m_rpm_max), st.m_given_min, st.m_given_max,
                     static_cast<unsigned>(st.m_temp));
    print_gap();
    bsp_uart1.printf("[CAN3-MOTOR] GM6020: rpm[%+.1f..%+.1f] given[%+d..%+d] temp=%u\r\n",
                     static_cast<double>(st.g_rpm_min), static_cast<double>(st.g_rpm_max), st.g_given_min, st.g_given_max,
                     static_cast<unsigned>(st.g_temp));
    print_gap();
    bsp_uart1.printf("[CAN3-MOTOR] can: BO=%u EW=%u EP=%u TXd=%u TXf=%u RXd=%u RXl=%u\r\n",
                     static_cast<unsigned>(bsp_can3.diagnostics.bus_off_events), static_cast<unsigned>(bsp_can3.diagnostics.err_warning),
                     static_cast<unsigned>(bsp_can3.diagnostics.err_passive), static_cast<unsigned>(bsp_can3.diagnostics.tx_dropped),
                     static_cast<unsigned>(bsp_can3.diagnostics.tx_buf_full), static_cast<unsigned>(bsp_can3.diagnostics.rx_dropped),
                     static_cast<unsigned>(bsp_can3.diagnostics.rx_lost));
    print_gap();
    bsp_uart1.printf("[CAN3-MOTOR] send any char to spin again\r\n");
    print_gap();
  }
}
#endif
