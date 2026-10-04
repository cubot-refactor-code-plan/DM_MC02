#include "app_test.hpp"


#if APP_TEST_DJI_GROUP_ENABLED

// 依赖只在测试启用时有意义：放进 #if，避免关闭时整个 TU 为空、被 include-cleaner 判成多余
#  include "bsp_cfg.hpp"
#  include "device_cfg.hpp"  // can3_dji_group
#  include "service_cfg.hpp" // sys_state
#  include "task.h"

#  include <stdint.h>


/** @brief 开环恒定指令（原始值）；没有闭环，直接把这一档电流一直给着让它转 */
constexpr int16_t CMD_AMP = 300;
/** @brief 串口汇报周期，单位 ms；每次只打一个位置，轮着来 */
constexpr uint32_t REPORT_PERIOD_MS = 200U;

namespace
{
/** @brief 汇报一行：位置 / 反馈标识符 / 在线 / 指令 / 输出轴角速度与累计角度 */
void report(uint8_t index)
{
  const MotorData &md = can3_dji_group.data(index);
  bsp_uart1.printf("[DJIGRP] pos=%u fb=0x%03X online=%u cmd=%d vel=%d mrad/s ang=%d mrad\r\n",
                   static_cast<unsigned>(index),
                   static_cast<unsigned>(DjiMotorGroup::RX_ID_BASE + index),
                   can3_dji_group.is_online(index) ? 1U : 0U,
                   static_cast<int>(CMD_AMP),
                   static_cast<int>(md.radian_data.velocity * 1000.0f),
                   static_cast<int>(md.radian_data.angle_multi_round * 1000.0f));
}

/** @brief 打印成员表里挂了几个位置 */
void report_layout()
{
  uint32_t used = 0U;
  for (uint8_t index = 0U; index < DjiMotorGroup::MOTOR_NUM; ++index)
  {
    if (can3_dji_group.has_motor(index))
    {
      ++used;
    }
  }
  bsp_uart1.printf("[DJIGRP] CAN3 open-loop constant: %u motor(s), cmd=%d\r\n",
                   static_cast<unsigned>(used),
                   static_cast<int>(CMD_AMP));
}
} // namespace

extern "C" void dji_motor_group_test_task(void *argument)
{
  (void)argument;
  sys_state.wait_running();
  static_assert(configTICK_RATE_HZ == 1000U, "DJI motor group test requires a 1 kHz tick");

  if (can3_dji_group.init() != Status::OK)
  {
    bsp_uart1.printf("[DJIGRP] can3_dji_group.init() failed\r\n");
    vTaskDelete(NULL);
    return;
  }
  report_layout();

  TickType_t wake_time = xTaskGetTickCount();
  uint32_t   tick      = 0U;

  for (;;)
  {
    // 1) 收：把 CAN3 上的帧全部交给组，由组按反馈标识符分派到对应位置
    CanRxMsg rx = {};
    for (uint32_t i = 0U; i < 8U && bsp_can3.receive(&rx, 0U) == Status::OK; ++i)
    {
      (void)can3_dji_group.update(rx);
    }

    // 2) 算：开环恒定指令，只给挂了电机的位置写
    for (uint8_t index = 0U; index < DjiMotorGroup::MOTOR_NUM; ++index)
    {
      if (can3_dji_group.has_motor(index))
      {
        (void)can3_dji_group.set_output(index, CMD_AMP);
      }
    }

    // 3) 发：有成员的控制帧一次性发出
    (void)can3_dji_group.poll();

    // 4) 报：每次只打一个位置，避免刷爆串口
    if ((tick % REPORT_PERIOD_MS) == 0U)
    {
      const uint8_t index = static_cast<uint8_t>((tick / REPORT_PERIOD_MS) % DjiMotorGroup::MOTOR_NUM);
      if (can3_dji_group.has_motor(index))
      {
        report(index);
      }
    }

    ++tick;
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(1U));
  }
}
#endif
