#include "app_test.hpp"


#if APP_TEST_DM_MOTOR_ENABLED

// 依赖只在测试启用时有意义：放进 #if，避免关闭时整个 TU 为空、被 include-cleaner 判成多余
#  include "FreeRTOS.h" // IWYU pragma: keep
#  include "bsp_cfg.hpp"
#  include "dm_motor.hpp"
#  include "service_cfg.hpp" // bus_can2
#  include "task.h"

#  include <math.h>
#  include <stdint.h>
#  include <string.h>


volatile uint32_t dm_motor_test_stage              = 0U;
volatile uint32_t dm_motor_test_passed             = 0U;
volatile uint32_t dm_motor_test_mode_pass_mask     = 0U;
volatile uint32_t dm_motor_test_initial_mode       = 0U;
volatile uint32_t dm_motor_test_last_drive_state   = 0U;
volatile uint32_t dm_motor_test_parameter_failures = 0U;
volatile Status   dm_motor_test_last_status        = Status::NOT_INIT;
volatile Status   dm_motor_test_online_status      = Status::NOT_INIT;
volatile float    dm_motor_test_pmax               = 0.0f;
volatile float    dm_motor_test_vmax               = 0.0f;
volatile float    dm_motor_test_tmax               = 0.0f;
volatile float    dm_motor_test_last_position      = 0.0f;
volatile float    dm_motor_test_last_velocity      = 0.0f;
volatile float    dm_motor_test_peak_abs_velocity  = 0.0f;


namespace
{
constexpr uint16_t DM_ESC_ID        = 0x01U;
constexpr uint16_t DM_MASTER_ID     = 0x00U;
constexpr uint32_t PARAMETER_STD_ID = 0x7FFU;
constexpr uint8_t  READ_COMMAND     = 0x33U;
constexpr uint8_t  WRITE_COMMAND    = 0x55U;

constexpr uint8_t CTRL_MODE_REGISTER = 0x0AU;
constexpr uint8_t PMAX_REGISTER      = 0x15U;
constexpr uint8_t VMAX_REGISTER      = 0x16U;
constexpr uint8_t TMAX_REGISTER      = 0x17U;
constexpr uint8_t POSITION_REGISTER  = 0x50U;

constexpr uint32_t TEST_RUN_TIME_MS         = 4000U;
constexpr float    TEST_VELOCITY_RAD_S      = 1.0f;
constexpr float    TEST_POSITION_TRAVEL_RAD = 4.0f;
constexpr float    TEST_POSITION_MARGIN_RAD = 0.1f;

float make_position_target(float current_position, float position_limit)
{
  const float positive_room  = position_limit - current_position;
  const float negative_room  = current_position + position_limit;
  const bool  move_positive  = positive_room >= negative_room;
  const float available_room = move_positive ? positive_room : negative_room;
  const float travel         = fminf(TEST_POSITION_TRAVEL_RAD,
                                     fmaxf(available_room - TEST_POSITION_MARGIN_RAD, 0.0f));
  return current_position + (move_positive ? travel : -travel);
}

void drain_can2(void)
{
  CanRxMsg rx = {};
  while (bus_can2.receive(&rx, 0U) == Status::OK)
  {
  }
}

bool wait_parameter_reply(uint8_t command, uint8_t register_id, uint8_t *value)
{
  for (uint32_t attempt = 0U; attempt < 20U; ++attempt)
  {
    CanRxMsg rx = {};
    if (bus_can2.receive(&rx, pdMS_TO_TICKS(5U)) != Status::OK)
    {
      continue;
    }

    if ((rx.header.Identifier == DM_MASTER_ID) && (rx.header.IdType == FDCAN_STANDARD_ID) && (rx.header.RxFrameType == FDCAN_DATA_FRAME) && (rx.header.DataLength == FDCAN_DLC_BYTES_8) && (rx.data[0] == static_cast<uint8_t>(DM_ESC_ID)) && (rx.data[1] == static_cast<uint8_t>(DM_ESC_ID >> 8U)) && (rx.data[2] == command) && (rx.data[3] == register_id))
    {
      memcpy(value, &rx.data[4], 4U);
      return true;
    }
  }
  return false;
}

bool read_register(uint8_t register_id, uint8_t *value)
{
  drain_can2();
  uint8_t request[8] = {
    static_cast<uint8_t>(DM_ESC_ID),
    static_cast<uint8_t>(DM_ESC_ID >> 8U),
    READ_COMMAND,
    register_id,
    0U,
    0U,
    0U,
    0U,
  };

  if (bus_can2.send(PARAMETER_STD_ID, request) != Status::OK)
  {
    return false;
  }
  return wait_parameter_reply(READ_COMMAND, register_id, value);
}

bool read_register_float(uint8_t register_id, float *value)
{
  uint8_t raw[4] = {};
  if ((value == nullptr) || !read_register(register_id, raw))
  {
    return false;
  }
  memcpy(value, raw, sizeof(*value));
  return isfinite(*value);
}

bool read_register_u32(uint8_t register_id, uint32_t *value)
{
  uint8_t raw[4] = {};
  if ((value == nullptr) || !read_register(register_id, raw))
  {
    return false;
  }
  memcpy(value, raw, sizeof(*value));
  return true;
}

bool write_register_u32(uint8_t register_id, uint32_t value)
{
  drain_can2();
  uint8_t request[8] = {
    static_cast<uint8_t>(DM_ESC_ID),
    static_cast<uint8_t>(DM_ESC_ID >> 8U),
    WRITE_COMMAND,
    register_id,
    0U,
    0U,
    0U,
    0U,
  };
  memcpy(&request[4], &value, sizeof(value));

  if (bus_can2.send(PARAMETER_STD_ID, request) != Status::OK)
  {
    return false;
  }

  uint8_t reply[4] = {};
  if (!wait_parameter_reply(WRITE_COMMAND, register_id, reply))
  {
    return false;
  }

  uint32_t returned_value = 0U;
  memcpy(&returned_value, reply, sizeof(returned_value));
  return returned_value == value;
}

bool wait_enabled(DmMotor<DmMotorType::J4310_2EC> &motor)
{
  for (uint32_t elapsed = 0U; elapsed < 300U; ++elapsed)
  {
    if ((motor.enable_state() == DmEnableState::ENABLED) && (motor.online().is_online() == Status::OK))
    {
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(1U));
  }
  return false;
}

void sample_motor(DmMotor<DmMotorType::J4310_2EC> &motor)
{
  dm_motor_test_online_status    = motor.online().is_online();
  dm_motor_test_last_drive_state = static_cast<uint32_t>(motor.drive_state());
  dm_motor_test_last_position    = motor.data().radian_data.angle_multi_round;
  dm_motor_test_last_velocity    = motor.data().radian_data.velocity;

  const float absolute_velocity = fabsf(dm_motor_test_last_velocity);
  if (absolute_velocity > dm_motor_test_peak_abs_velocity)
  {
    dm_motor_test_peak_abs_velocity = absolute_velocity;
  }
}

bool run_mode_test(DmControlMode mode, const DmProtocolLimits &limits)
{
  if (!write_register_u32(CTRL_MODE_REGISTER, static_cast<uint32_t>(mode)))
  {
    ++dm_motor_test_parameter_failures;
    return false;
  }

  float current_position = 0.0f;
  if (!read_register_float(POSITION_REGISTER, &current_position))
  {
    ++dm_motor_test_parameter_failures;
    return false;
  }
  const float position_target =
    make_position_target(current_position, limits.position_max_rad);

  DmMotor<DmMotorType::J4310_2EC> motor({bus_can2,
                                         DM_ESC_ID,
                                         DM_MASTER_ID,
                                         mode,
                                         limits.position_max_rad,
                                         limits.velocity_max_rad_s,
                                         limits.torque_max_nm});
  dm_motor_test_last_status = motor.init();
  if (dm_motor_test_last_status != Status::OK)
  {
    return false;
  }

  switch (mode)
  {
    case DmControlMode::MIT:
    {
      const float safe_position = fmaxf(-limits.position_max_rad,
                                        fminf(limits.position_max_rad, current_position));
      dm_motor_test_last_status = motor.set_mit_target(safe_position, 0.0f, 0.0f, 0.5f, 0.0f);
      break;
    }
    case DmControlMode::POSITION_VELOCITY:
      dm_motor_test_last_status = motor.set_position_velocity_target(current_position,
                                                                     TEST_VELOCITY_RAD_S);
      break;
    case DmControlMode::VELOCITY:
      dm_motor_test_last_status = motor.set_velocity_target(0.0f);
      break;
    case DmControlMode::POSITION_TORQUE:
      dm_motor_test_last_status = motor.set_position_torque_target(current_position,
                                                                   TEST_VELOCITY_RAD_S,
                                                                   0.05f);
      break;
    default:
      return false;
  }

  if (dm_motor_test_last_status != Status::OK)
  {
    return false;
  }

  dm_motor_test_last_status = motor.enable();
  if ((dm_motor_test_last_status != Status::OK) || !wait_enabled(motor))
  {
    motor.disable();
    vTaskDelay(pdMS_TO_TICKS(20U));
    return false;
  }

  sample_motor(motor);
  const float start_position = motor.data().radian_data.angle_multi_round;

  switch (mode)
  {
    case DmControlMode::MIT:
    {
      const float safe_position = fmaxf(-limits.position_max_rad,
                                        fminf(limits.position_max_rad, current_position));
      dm_motor_test_last_status = motor.set_mit_target(safe_position,
                                                       TEST_VELOCITY_RAD_S,
                                                       0.0f,
                                                       0.5f,
                                                       0.0f);
      break;
    }
    case DmControlMode::POSITION_VELOCITY:
      dm_motor_test_last_status = motor.set_position_velocity_target(position_target,
                                                                     TEST_VELOCITY_RAD_S);
      break;
    case DmControlMode::VELOCITY:
      dm_motor_test_last_status = motor.set_velocity_target(TEST_VELOCITY_RAD_S);
      break;
    case DmControlMode::POSITION_TORQUE:
      dm_motor_test_last_status = motor.set_position_torque_target(position_target,
                                                                   TEST_VELOCITY_RAD_S,
                                                                   0.05f);
      break;
    default:
      dm_motor_test_last_status = Status::BAD_ARG;
      break;
  }

  for (uint32_t elapsed = 0U; elapsed < TEST_RUN_TIME_MS; ++elapsed)
  {
    sample_motor(motor);
    vTaskDelay(pdMS_TO_TICKS(1U));
  }

  const float position_delta =
    fabsf(motor.data().radian_data.angle_multi_round - start_position);
  const bool feedback_valid =
    (dm_motor_test_last_status == Status::OK) && (motor.online().is_online() == Status::OK) && (motor.drive_state() == DmDriveState::ENABLED) && isfinite(position_delta);

  if (mode == DmControlMode::MIT)
  {
    motor.set_mit_target(current_position, 0.0f, 0.0f, 0.5f, 0.0f);
  }
  else if (mode == DmControlMode::POSITION_VELOCITY)
  {
    motor.set_position_velocity_target(motor.data().radian_data.angle_multi_round,
                                       TEST_VELOCITY_RAD_S);
  }
  else if (mode == DmControlMode::VELOCITY)
  {
    motor.set_velocity_target(0.0f);
  }
  else
  {
    motor.set_position_torque_target(motor.data().radian_data.angle_multi_round,
                                     TEST_VELOCITY_RAD_S,
                                     0.05f);
  }

  vTaskDelay(pdMS_TO_TICKS(100U));
  dm_motor_test_last_status = motor.disable();
  vTaskDelay(pdMS_TO_TICKS(100U));
  sample_motor(motor);
  return feedback_valid && (dm_motor_test_last_status == Status::OK);
}
} // namespace


extern "C" void dm_motor_test_task(void *argument)
{
  (void)argument;
  vTaskDelay(pdMS_TO_TICKS(100U));
  dm_motor_test_stage = 1U;

  uint32_t   initial_mode = 0U;
  float      pmax         = 0.0f;
  float      vmax         = 0.0f;
  float      tmax         = 0.0f;
  const bool parameters_valid =
    read_register_u32(CTRL_MODE_REGISTER, &initial_mode) && read_register_float(PMAX_REGISTER, &pmax) && read_register_float(VMAX_REGISTER, &vmax) && read_register_float(TMAX_REGISTER, &tmax) && (initial_mode >= 1U) && (initial_mode <= 4U) && (pmax > 0.0f) && (vmax > 0.0f) && (tmax > 0.0f);

  dm_motor_test_initial_mode = initial_mode;
  dm_motor_test_pmax         = pmax;
  dm_motor_test_vmax         = vmax;
  dm_motor_test_tmax         = tmax;

  if (!parameters_valid)
  {
    ++dm_motor_test_parameter_failures;
    dm_motor_test_stage = 0xE1U;
    vTaskSuspend(nullptr);
    return;
  }

  const DmProtocolLimits limits(pmax, vmax, tmax);
  const DmControlMode    modes[] = {
    DmControlMode::MIT,
    DmControlMode::POSITION_VELOCITY,
    DmControlMode::VELOCITY,
    DmControlMode::POSITION_TORQUE,
  };

  dm_motor_test_stage = 2U;
  for (uint32_t index = 0U; index < 4U; ++index)
  {
    if (run_mode_test(modes[index], limits))
    {
      dm_motor_test_mode_pass_mask |= (1UL << index);
    }
    dm_motor_test_stage = 3U + index;
  }

  if (!write_register_u32(CTRL_MODE_REGISTER, initial_mode))
  {
    ++dm_motor_test_parameter_failures;
  }

  dm_motor_test_passed =
    ((dm_motor_test_mode_pass_mask == 0x0FU) && (dm_motor_test_parameter_failures == 0U))
      ? 1U
      : 0U;
  dm_motor_test_stage = 7U;
  vTaskSuspend(nullptr);
}

#endif // APP_TEST_DM_MOTOR_ENABLED
