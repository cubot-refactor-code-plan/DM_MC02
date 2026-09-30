#include "app_test.hpp"
#include "bsp_cfg.hpp"
#include "dji_motor.hpp"
#include "task.h"
#include <cmath>

#if APP_TEST_DJI_MOTOR_ENABLED

// CAN1 / C620 ID2：发送 0x200 的第二槽位，反馈 0x202。
// 使用 M3508 默认减速比，接收与解包完全由 CanRxNode 驱动。
DjiMotor<Motor3508> dji_motor_test_motor(bsp_can1, 2U);

// GDB 写 arm=1 执行一次；写 abort=1 随时终止。复位后默认零输出。
volatile uint32_t dji_motor_test_arm = 0;
volatile uint32_t dji_motor_test_abort = 0;
// 阶段：1 待触发，2 等反馈，3 零输出，4 正转，5 滑停，6 反转，7 滑停，8 成功，9 失败。
volatile uint32_t dji_motor_test_stage = 0;
volatile uint32_t dji_motor_test_passed = 0;
// 失败：1 初始化，2 反馈超时，3 写入失败，4 离线，5 滑停超时，6 方向/位移，7 总线错误，8 超速，9 人工终止。
volatile uint32_t dji_motor_test_failure = 0;
volatile uint32_t dji_motor_test_ticks = 0;
volatile uint32_t dji_motor_test_online_samples = 0;
volatile uint32_t dji_motor_test_fallback_frames = 0;
volatile uint32_t dji_motor_test_rejected_feedback = 0;
volatile uint32_t dji_motor_test_last_fallback_id = 0;
volatile Status dji_motor_test_object_status = Status::NOT_INIT;
volatile Status dji_motor_test_fill_status = Status::NOT_INIT;
volatile Status dji_motor_test_online_status = Status::NOT_INIT;
volatile int16_t dji_motor_test_command = 0;
volatile int16_t dji_motor_test_speed_rpm = 0;
volatile int16_t dji_motor_test_given_current = 0;
volatile uint8_t dji_motor_test_temperature = 0;
volatile int16_t dji_motor_test_positive_peak = 0;
volatile int16_t dji_motor_test_negative_peak = 0;
volatile int16_t dji_motor_test_positive_current = 0;
volatile int16_t dji_motor_test_negative_current = 0;
volatile float dji_motor_test_angle = 0;
volatile float dji_motor_test_positive_delta = 0;
volatile float dji_motor_test_negative_delta = 0;
volatile uint32_t dji_motor_test_can_error = 0;
volatile uint32_t dji_motor_test_bus_off = 0;
volatile uint32_t dji_motor_test_tx_error_count = 0;
volatile uint32_t dji_motor_test_rx_error_count = 0;

namespace
{
void sample()
{
  taskENTER_CRITICAL();
  const auto raw = dji_motor_test_motor.raw_data();
  dji_motor_test_angle = dji_motor_test_motor.data().radian_data.angle_multi_round;
  taskEXIT_CRITICAL();
  dji_motor_test_speed_rpm = raw.rpm;
  dji_motor_test_given_current = raw.torque_current;
  dji_motor_test_temperature = raw.temperature;
  dji_motor_test_online_status = dji_motor_test_motor.online().isOnline();
  if (dji_motor_test_online_status == Status::OK) ++dji_motor_test_online_samples;
  ++dji_motor_test_ticks;
  // 仅观察回退帧，绝不再次调用解包，避免绕过 CanRxNode 掩盖分发问题。
  CanRxMsg rx = {};
  for (unsigned i = 0; i < 8 && bsp_can1.receive(&rx, 0) == Status::OK; ++i)
  {
    ++dji_motor_test_fallback_frames;
    dji_motor_test_last_fallback_id = rx.header.Identifier;
    if (rx.header.Identifier == 0x202 && rx.header.IdType == FDCAN_STANDARD_ID)
      ++dji_motor_test_rejected_feedback;
  }
  dji_motor_test_can_error = HAL_FDCAN_GetError(&hfdcan1);
  FDCAN_ProtocolStatusTypeDef protocol = {};
  if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol) == HAL_OK)
    dji_motor_test_bus_off = protocol.BusOff;
  FDCAN_ErrorCountersTypeDef errors = {};
  if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &errors) == HAL_OK)
  {
    dji_motor_test_tx_error_count = errors.TxErrorCnt;
    dji_motor_test_rx_error_count = errors.RxErrorCnt;
  }
}

bool output(int16_t current)
{
  dji_motor_test_fill_status = dji_motor_test_motor.FillData(current);
  if (dji_motor_test_fill_status != Status::OK)
  {
    dji_motor_test_failure = 3;
    return false;
  }
  dji_motor_test_command = current;
  return true;
}

bool healthy()
{
  if (dji_motor_test_abort) dji_motor_test_failure = 9;
  else if (dji_motor_test_bus_off) dji_motor_test_failure = 7;
  else if (dji_motor_test_online_status != Status::OK) dji_motor_test_failure = 4;
  else if (std::abs(static_cast<int>(dji_motor_test_speed_rpm)) > 8000) dji_motor_test_failure = 8;
  return dji_motor_test_failure == 0;
}

bool phase(uint32_t stage, int16_t current, uint32_t duration_ms, bool wait_stop = false)
{
  dji_motor_test_stage = stage;
  TickType_t wake = xTaskGetTickCount();
  const TickType_t start = wake;
  uint32_t quiet_ms = 0;
  while (xTaskGetTickCount() - start < pdMS_TO_TICKS(duration_ms))
  {
    sample();
    if (!healthy() || !output(current)) return false;
    if (stage == 4) dji_motor_test_positive_current = dji_motor_test_given_current;
    if (stage == 6) dji_motor_test_negative_current = dji_motor_test_given_current;
    if (stage == 4 && dji_motor_test_speed_rpm > dji_motor_test_positive_peak)
      dji_motor_test_positive_peak = dji_motor_test_speed_rpm;
    if (stage == 6 && dji_motor_test_speed_rpm < dji_motor_test_negative_peak)
      dji_motor_test_negative_peak = dji_motor_test_speed_rpm;
    if (wait_stop)
    {
      quiet_ms = std::abs(static_cast<int>(dji_motor_test_speed_rpm)) < 50 ? quiet_ms + 1 : 0;
      if (quiet_ms >= 200) return true;
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
  }
  if (wait_stop) dji_motor_test_failure = 5;
  return !wait_stop;
}

bool run()
{
  dji_motor_test_stage = 2;
  const TickType_t start = xTaskGetTickCount();
  do
  {
    sample();
    if (!output(0)) return false;
    if (dji_motor_test_abort) { dji_motor_test_failure = 9; return false; }
    if (dji_motor_test_online_status == Status::OK) break;
    if (xTaskGetTickCount() - start >= pdMS_TO_TICKS(3000))
    { dji_motor_test_failure = 2; return false; }
    vTaskDelay(pdMS_TO_TICKS(1));
  } while (true);
  if (!phase(3, 0, 500)) return false;
  float start_angle = dji_motor_test_angle;
  if (!phase(4, 512, 600) || !phase(5, 0, 10000, true)) return false;
  dji_motor_test_positive_delta = dji_motor_test_angle - start_angle;
  start_angle = dji_motor_test_angle;
  if (!phase(6, -512, 600) || !phase(7, 0, 10000, true)) return false;
  dji_motor_test_negative_delta = dji_motor_test_angle - start_angle;
  if (dji_motor_test_positive_peak <= 50 || dji_motor_test_negative_peak >= -50 ||
      dji_motor_test_positive_delta <= 0.01f || dji_motor_test_negative_delta >= -0.01f)
  { dji_motor_test_failure = 6; return false; }
  return true;
}
}

extern "C" void dji_motor_test_init(void)
{
  dji_motor_test_object_status = dji_motor_test_motor.init();
  if (dji_motor_test_object_status != Status::OK)
  {
    dji_motor_test_failure = 1;
    SysInitError(dji_motor_test_object_status);
  }
}

extern "C" void dji_motor_test_task(void *argument)
{
  (void)argument;
  static_assert(configTICK_RATE_HZ == 1000U, "Motor test requires 1 kHz tick");
  SysFlagWaitRunning();
  dji_motor_test_stage = 1;
  while (dji_motor_test_arm != 1 && dji_motor_test_failure == 0)
  {
    sample();
    output(0);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  dji_motor_test_arm = 0;
  dji_motor_test_passed = dji_motor_test_failure == 0 && run();
  output(0);
  dji_motor_test_stage = dji_motor_test_passed ? 8 : 9;
  for (;;)
  {
    output(0);
    sample();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
#endif
