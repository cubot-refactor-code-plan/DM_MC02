/**
 * @file dji_motor_group_test.cpp
 * @author Rh
 * @brief CAN3 DJI 电机组自检：开环恒定指令、1 kHz 收发、每 200 ms 轮着打印一个位置
 * @date 2026-10-03
 *
 * 打印：[DJIGRP] pos=4 fb=0x204 online=1 cmd=350 vel=1234 mrad/s ang=5678 mrad
 */

#include "app_test.hpp"


#if APP_TEST_DJI_GROUP_ENABLED

// 依赖只在测试启用时有意义：放进 #if，避免关闭时整个 TU 为空、被 include-cleaner 判成多余
#  include "bsp_cfg.hpp"
#  include "device_cfg.hpp"  // can3_dji_group
#  include "service_cfg.hpp" // sys_state
#  include "task.h"

#  include <stdint.h>


/**
 * @brief 开环恒定指令（原始值），下标 = 位置；只给挂了电机的位置写
 * @note 位置 4 和 9 各给一档，方便从反馈里分辨是哪一台在响应。
 */
constexpr int16_t CMD_BY_POS[DjiMotorGroup::MOTOR_NUM] = {
  0, 0, 0, 0,    // 0~3：未挂电机
  350, 0, 0, 0,  // 4 = 挂一台，5~7 未挂
  0, 350, 0,     // 8 空，9 = 挂一台，10 空
};

/** @brief 汇报周期 (ms)；每次只打一个位置，轮着来 */
constexpr uint32_t REPORT_PERIOD_MS = 200U;


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

  // 开跑前先把成员表打出来：谁挂在哪个位置、给多少指令
  uint32_t used = 0U;
  for (uint8_t index = 0U; index < DjiMotorGroup::MOTOR_NUM; ++index)
  {
    if (!can3_dji_group.has_motor(index))
    {
      continue;
    }

    ++used;
    bsp_uart1.printf("[DJIGRP] pos=%u fb=0x%03X cmd=%d\r\n",
                     static_cast<unsigned>(index),
                     static_cast<unsigned>(DjiMotorGroup::RX_ID_BASE + index),
                     static_cast<int>(CMD_BY_POS[index]));
  }
  bsp_uart1.printf("[DJIGRP] CAN3 open-loop constant: %u motor(s)\r\n", static_cast<unsigned>(used));

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
        (void)can3_dji_group.set_output(index, CMD_BY_POS[index]);
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
        const MotorData &md = can3_dji_group.data(index);
        bsp_uart1.printf("[DJIGRP] pos=%u fb=0x%03X online=%u cmd=%d vel=%d mrad/s ang=%d mrad\r\n",
                         static_cast<unsigned>(index),
                         static_cast<unsigned>(DjiMotorGroup::RX_ID_BASE + index),
                         can3_dji_group.is_online(index) ? 1U : 0U,
                         static_cast<int>(CMD_BY_POS[index]),
                         static_cast<int>(md.radian_data.velocity * 1000.0f),
                         static_cast<int>(md.radian_data.angle_multi_round * 1000.0f));
      }
    }

    ++tick;
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(1U));
  }
}
#endif
