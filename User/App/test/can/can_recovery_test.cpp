#include "app_test.hpp"
#include "bsp_cfg.hpp"
#include "can_rx_node.hpp"
#include "task.h"

#if APP_TEST_CAN_RECOVERY_ENABLED
#if APP_TEST_DJI_MOTOR_ENABLED
#error "CAN recovery test and motor motion test cannot share CAN1"
#endif

// CAN1/C620 ID2，所有控制帧均为零。arm=1 自动故障注入，arm=2 等待物理断连。
volatile uint32_t can_recovery_test_arm = 0;
volatile uint32_t can_recovery_test_stage = 0;
volatile uint32_t can_recovery_test_failure = 0;
volatile uint32_t can_recovery_test_passed = 0;
volatile uint32_t can_recovery_test_round = 0;
volatile uint32_t can_recovery_test_injection_attempts = 0;
volatile uint32_t can_recovery_test_feedback = 0;
volatile TickType_t can_recovery_test_last_rx = 0;
volatile uint32_t can_recovery_test_bo_rounds = 0;
volatile uint32_t can_recovery_test_reconnect_ticks = 0;
volatile uint32_t can_recovery_test_send_full = 0;
volatile uint32_t can_recovery_test_send_busy = 0;
volatile uint32_t can_recovery_test_rx_overflow = 0;
volatile Status can_recovery_test_init_status = Status::NOT_INIT;

namespace
{
Status feedback(void *, const CanRxMsg &rx)
{
  if (((uint16_t(rx.data[0]) << 8) | rx.data[1]) >= 8192) return Status::BAD_ARG;
  ++can_recovery_test_feedback;
  can_recovery_test_last_rx = xTaskGetTickCount();
  return Status::OK;
}

void zero()
{
  uint8_t data[8] = {};
  const Status status = bsp_can1.send(0x200, data);
  if (status == Status::FULL) ++can_recovery_test_send_full;
  if (status == Status::BUSY) ++can_recovery_test_send_busy;
}

bool wait_feedback(uint32_t timeout_ms)
{
  const uint32_t before = can_recovery_test_feedback;
  const TickType_t start = xTaskGetTickCount();
  while (xTaskGetTickCount() - start < pdMS_TO_TICKS(timeout_ms))
  {
    zero();
    if (can_recovery_test_feedback - before >= 10 && !bsp_can1.diagnostics.recovering &&
        (hfdcan1.Instance->PSR & FDCAN_PSR_BO) == 0) return true;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return false;
}

// 短临界区修改测试总线位时序；失败也恢复原 INIT 请求，不在 sys_task 中忙等。
bool timing(uint32_t nbtp)
{
  taskENTER_CRITICAL();
  SET_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT);
  uint32_t tries = 10000;
  while ((hfdcan1.Instance->CCCR & FDCAN_CCCR_INIT) == 0 && --tries) {}
  if (tries == 0) { taskEXIT_CRITICAL(); return false; }
  SET_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_CCE);
  hfdcan1.Instance->NBTP = nbtp;
  // 真正 Bus-Off 时将恢复动作留给 sys_task；普通测试配置退出初始化。
  if ((hfdcan1.Instance->PSR & FDCAN_PSR_BO) == 0)
    CLEAR_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT);
  taskEXIT_CRITICAL();
  return true;
}

bool automatic()
{
  can_recovery_test_stage = 2;
  if (!wait_feedback(3000)) { can_recovery_test_failure = 1; return false; }
  const uint32_t normal = hfdcan1.Instance->NBTP;
  const uint32_t prescalers[] = {17U, 0U, 1U, 8U};
  for (uint32_t attempt = 0; attempt < 100 && can_recovery_test_bo_rounds < 20; ++attempt)
  {
    can_recovery_test_injection_attempts = attempt + 1;
    can_recovery_test_round = can_recovery_test_bo_rounds + 1;
    const uint32_t wrong = (normal & ~FDCAN_NBTP_NBRP) |
                          (prescalers[attempt % 4] << FDCAN_NBTP_NBRP_Pos);
    can_recovery_test_stage = 3;
    const uint32_t bo = bsp_can1.diagnostics.bus_off_events;
    if (!timing(wrong)) { timing(normal); can_recovery_test_failure = 2; return false; }
    // 维持错误位时序 300 ms，验证持续故障期间系统任务仍能运行。
    for (uint32_t i = 0; i < 300; ++i) { zero(); vTaskDelay(pdMS_TO_TICKS(1)); }
    const bool observed = bsp_can1.diagnostics.bus_off_events > bo;
    can_recovery_test_stage = 4;
    if (!timing(normal)) { can_recovery_test_failure = 2; return false; }
    const TickType_t restored = xTaskGetTickCount();
    if (!wait_feedback(3000)) { can_recovery_test_failure = 3; return false; }
    can_recovery_test_reconnect_ticks = xTaskGetTickCount() - restored;
    if (observed) ++can_recovery_test_bo_rounds;
    // 错误波特率可能只产生 error-passive/ACK 错误；这种尝试不计为 BO 覆盖。
    for (uint32_t i = 0; i < 500; ++i) { zero(); vTaskDelay(pdMS_TO_TICKS(1)); }
  }
  if (can_recovery_test_bo_rounds < 20) { can_recovery_test_failure = 4; return false; }
  // 接收积压：屏蔽 CAN1 接收中断，电机仍持续发送反馈。
  can_recovery_test_stage = 5;
  const uint32_t drops = bsp_can1.diagnostics.rx_dropped;
  HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  vTaskDelay(pdMS_TO_TICKS(50));
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  if (!wait_feedback(3000)) { can_recovery_test_failure = 5; return false; }
  can_recovery_test_rx_overflow = bsp_can1.diagnostics.rx_dropped - drops;
  if (!can_recovery_test_rx_overflow) { can_recovery_test_failure = 6; return false; }
  // 连续零帧发送突发，必须触发 FULL，再确认仍能收发。
  can_recovery_test_stage = 6;
  for (uint32_t i = 0; i < 256; ++i) zero();
  if (!can_recovery_test_send_full || !wait_feedback(3000))
  { can_recovery_test_failure = 7; return false; }
  return true;
}

bool physical()
{
  can_recovery_test_stage = 40;
  if (!wait_feedback(3000)) { can_recovery_test_failure = 1; return false; }
  const uint32_t bo = bsp_can1.diagnostics.bus_off_events;
  const TickType_t start = xTaskGetTickCount();
  // 用户断连再接回。单纯缺 ACK 可能只停在 error-passive，不能声称验证了 Bus-Off。
  while (xTaskGetTickCount() - start < pdMS_TO_TICKS(60000))
  {
    zero();
    if (bsp_can1.diagnostics.bus_off_events > bo)
    {
      can_recovery_test_stage = 41;
      if (!wait_feedback(60000)) { can_recovery_test_failure = 3; return false; }
      ++can_recovery_test_bo_rounds;
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  can_recovery_test_failure = 4;
  return false;
}
}

extern "C" void can_recovery_test_init()
{
  can_recovery_test_init_status = regist(bsp_can1, 0x202, feedback) ? Status::OK : Status::IO_ERROR;
}
extern "C" void can_recovery_test_task(void *)
{
  SysFlagWaitRunning();
  can_recovery_test_stage = 1;
  while (can_recovery_test_arm == 0) { zero(); vTaskDelay(pdMS_TO_TICKS(1)); }
  const uint32_t arm = can_recovery_test_arm;
  can_recovery_test_arm = 0;
  can_recovery_test_passed = arm == 1 ? automatic() : physical();
  can_recovery_test_stage = can_recovery_test_passed ? 8 : 9;
  for (;;) { zero(); vTaskDelay(pdMS_TO_TICKS(1)); }
}
#endif
