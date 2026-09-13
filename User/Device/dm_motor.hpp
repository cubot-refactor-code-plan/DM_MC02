/**
 * @file dm_motor.hpp
 * @author ChoseB
 * @brief 达妙普通固件电机的反馈、控制目标和统一发送接口声明
 * @version 0.2
 * @date 2026-09-02
 *
 * @details 本接口面向达妙 V13/V17 普通固件的 MIT、位置速度、速度和力位混控模式。
 *          set_*_target() 只更新缓存控制帧，实际周期发送由 MotorTxManager 统一调度；
 *          enable()、disable() 等一次性动作也只提交状态请求，不绕过统一发送顺序。
 * @note 协议编码、反馈解析、注册、状态机和发送逻辑实现在 dm_motor.cpp。
 */
#ifndef __DM_MOTOR_HPP__
#define __DM_MOTOR_HPP__

#include "bsp_can.hpp"
#include "motor_definition.hpp"
#include "motor_tx_manager.hpp"
#include "online_check.hpp"

#include <stdint.h>

/**
 * @brief 支持的达妙电机型号
 */
enum class DmMotorType : uint8_t
{
  J4310_2EC,
};

/**
 * @brief 达妙普通固件控制模式及 CTRL_MODE 寄存器编码
 */
enum class DmControlMode : uint8_t
{
  MIT               = 1U,
  POSITION_VELOCITY = 2U,
  VELOCITY          = 3U,
  POSITION_TORQUE   = 4U,
};

/**
 * @brief 达妙普通反馈帧中的驱动状态
 */
enum class DmDriveState : uint8_t
{
  DISABLED          = 0x0U,
  ENABLED           = 0x1U,
  OUTPUT_CALIB_ERR  = 0x3U,
  SENSOR_ERR        = 0x4U,
  ENCODER_CALIB_ERR = 0x5U,
  OVER_VOLTAGE      = 0x8U,
  UNDER_VOLTAGE     = 0x9U,
  OVER_CURRENT      = 0xAU,
  MOS_OVER_TEMP     = 0xBU,
  MOTOR_OVER_TEMP   = 0xCU,
  COMM_LOST         = 0xDU,
  OVERLOAD          = 0xEU,
  UNKNOWN           = 0xFFU,
};

/**
 * @brief 软件侧的达妙启停发送状态
 */
enum class DmEnableState : uint8_t
{
  DISABLED,
  ENABLE_PENDING,
  WAIT_ENABLE_FEEDBACK,
  ENABLED,
  DISABLE_PENDING,
  FAULT,
};

/**
 * @brief 等待统一发送管理器处理的一次性特殊指令
 */
enum class DmPendingAction : uint8_t
{
  NONE,
  CLEAR_ERROR,
  SAVE_ZERO,
};

/**
 * @brief 与电机 PMAX、VMAX、TMAX 寄存器一致的线性映射范围
 */
struct DmProtocolLimits
{
  DmProtocolLimits(float position_max_rad = 0.0f,
                   float velocity_max_rad_s = 0.0f,
                   float torque_max_nm = 0.0f)
    : position_max_rad(position_max_rad),
      velocity_max_rad_s(velocity_max_rad_s),
      torque_max_nm(torque_max_nm)
  {
  }

  float position_max_rad;   ///< PMAX，位置绝对值上限，单位：rad
  float velocity_max_rad_s; ///< VMAX，速度绝对值上限，单位：rad/s
  float torque_max_nm;      ///< TMAX，扭矩绝对值上限，单位：N·m
};

/**
 * @brief 达妙普通反馈协议特有的数据
 *
 * @note 位置和速度换算后的通用运动学结果保存在 MotorData 中；本结构只保存
 *       协议原始量及无法被所有电机统一表达的扭矩、温度和驱动状态。
 */
struct DmMotorFeedback
{
  uint16_t     raw_position;      ///< 反馈帧 16 位位置映射值
  uint16_t     raw_velocity;      ///< 反馈帧 12 位速度映射值
  uint16_t     raw_torque;        ///< 反馈帧 12 位扭矩映射值
  float        torque;            ///< 达妙内置输出轴扭矩，单位：N·m
  uint8_t      mos_temperature;   ///< 驱动 MOS 平均温度，单位：摄氏度
  uint8_t      rotor_temperature; ///< 电机线圈平均温度，单位：摄氏度
  DmDriveState drive_state;       ///< 当前使能或故障状态
};

template <DmMotorType TYPE>
class DmMotor
{
public:
  /**
   * @brief 构造达妙电机软件对象
   * @param can_item 电机连接的物理 CAN
   * @param esc_id 电机接收 ID，范围及模式偏移后必须满足标准帧限制
   * @param master_id 电机反馈 ID，范围 0~0x7FF
   * @param mode 电机当前已经配置的普通固件控制模式
   * @param position_max_rad PMAX，位置绝对值上限，单位：rad；传入 0 使用型号默认值
   * @param velocity_max_rad_s VMAX，速度绝对值上限，单位：rad/s；传入 0 使用型号默认值
   * @param torque_max_nm TMAX，扭矩绝对值上限，单位：N·m；传入 0 使用型号默认值
   * @param ratio 达妙输出轴到最终机构输出轴的传动比
   * @param offset 达妙输出轴机械零位偏移，单位：rad
   * @param period_ticks 相邻两次发送之间的 MotorTxManager::update() 次数
   * @param phase_ticks 周期内发送相位
   * @param order 同一周期内的发送顺序，数值较小者优先
   * @note 构造函数只保存配置，不注册对象、不创建 RTOS 资源且不发送 CAN 帧。
   * @note ratio 默认为 1，表示直接使用达妙内置输出轴；若其后增加机械变速箱，
   *       ratio 设置为达妙输出轴转速与最终机构输出轴转速之比。
   * @note J4310-2EC 的型号默认值为 PMAX=12.5 rad、VMAX=30 rad/s、TMAX=10 N·m。
   */
  DmMotor(BspCan &can_item,
          uint16_t esc_id,
          uint16_t master_id,
          DmControlMode mode,
          float position_max_rad = 0.0f,
          float velocity_max_rad_s = 0.0f,
          float torque_max_nm = 0.0f,
          float ratio = 1.0f,
          float offset = 0.0f,
          uint16_t period_ticks = 1U,
          uint16_t phase_ticks = 0U,
          uint16_t order = 0U);

  /**
   * @brief 注销反馈分发和统一发送端点
   */
  ~DmMotor();

  /**
   * @brief 校验配置并注册反馈分发及统一发送端点
   * @return Status::OK 初始化成功；Status::BAD_ARG 配置非法；Status::BUSY ID 冲突；
   *         其他状态表示发送管理器注册失败。
   */
  Status init(void);

  /** @brief 获取最终机构输出轴的通用运动学数据。 */
  const MotorData &data(void) const;

  /** @brief 获取达妙协议特有的反馈数据。 */
  const DmMotorFeedback &feedback(void) const;

  /** @brief 获取在线检查对象。 */
  const Online &online(void) const;

  /** @brief 获取可选的 Luenberger 观测结果。 */
  const LuenbergerMotorData &lvbo_data(void) const;

  /** @brief 查询电机反馈的使能或故障状态。 */
  DmDriveState drive_state(void) const;

  /** @brief 查询软件侧启停发送状态机。 */
  DmEnableState enable_state(void) const;

  /** @brief 查询最近一次公开操作状态。 */
  Status status(void) const;

  /**
   * @brief 缓存 MIT 模式下一周期的控制目标
   * @return Status::OK 缓存成功；Status::NOT_SUPPORTED 当前不是 MIT 模式；
   *         Status::BAD_ARG 参数非有限值或超出协议允许范围。
   */
  Status set_mit_target(float position, float velocity, float kp, float kd, float torque);

  /**
   * @brief 缓存位置速度模式下一周期的控制目标
   * @param position 最终机构输出轴目标位置，单位：rad
   * @param velocity_limit 最终机构输出轴最大绝对速度，单位：rad/s
   */
  Status set_position_velocity_target(float position, float velocity_limit);

  /**
   * @brief 缓存速度模式下一周期的控制目标
   * @param velocity 最终机构输出轴目标速度，单位：rad/s
   */
  Status set_velocity_target(float velocity);

  /**
   * @brief 缓存力位混控模式下一周期的控制目标
   * @param position 最终机构输出轴目标位置，单位：rad
   * @param velocity_limit 最终机构输出轴最大绝对速度，单位：rad/s
   * @param current_limit_ratio 扭矩电流限制标幺值，范围 0~1
   */
  Status set_position_torque_target(float position, float velocity_limit, float current_limit_ratio);

  /**
   * @brief 将当前模式的全部控制目标清零，再提交启用请求并等待 ENABLED 反馈
   * @return Status::OK 请求已接受，并不表示电机已经启用；Status::NOT_INIT 尚未初始化
   *         Status::BUSY 已有不兼容的一次性请求；其他状态表示零目标编码失败。
   * @note 无需在 enable() 前调用 set_*_target()；即使此前缓存过目标，也会被零目标覆盖。
   * @note MIT 模式会编码位置、速度、Kp、Kd、扭矩均为 0 的协议帧，不能直接用全零字节代替。
   */
  Status enable(void);

  /**
   * @brief 立即禁止周期控制并提交失能请求
   * @return Status::OK 请求已接受；Status::NOT_INIT 尚未初始化。
   */
  Status disable(void);

  /** @brief 提交清除故障请求，由统一发送管理器发送 0xFB 特殊帧。 */
  Status clear_error(void);

  /**
   * @brief 提交将当前位置保存为电机零点的请求
   * @warning 该操作会写入电机 Flash，不得周期或频繁调用。
   */
  Status save_current_position_as_zero(void);

  /**
   * @brief 解析一帧属于本电机的普通反馈
   * @param rx 待解析标准 8 字节 CAN 帧
   * @return Status::OK 解析成功；Status::BAD_ARG 标识符或帧格式不匹配；
   *         Status::NOT_INIT 尚未初始化。
   */
  Status data_unpack(const CanRxMsg &rx);

  DmMotor(const DmMotor &) = delete;
  DmMotor &operator=(const DmMotor &) = delete;
  DmMotor(DmMotor &&) = delete;
  DmMotor &operator=(DmMotor &&) = delete;

private:
  MotorData                 _data;           ///< 最终机构输出轴通用运动学数据
  DmMotorFeedback           _feedback;       ///< 达妙普通协议特有反馈
  Online                    _online;         ///< 有效反馈在线检查
  LuenbergerMotorData       _lvbo_data;      ///< 可选状态观测结果
  BspCan                   *_can_item;       ///< 接收和发送所用物理 CAN
  uint16_t                  _esc_id;         ///< 电机接收 ID
  uint16_t                  _master_id;      ///< 电机反馈 ID
  DmControlMode             _mode;           ///< 当前配置的普通固件控制模式
  DmProtocolLimits          _limits;         ///< MIT 编解码映射范围
  DmEnableState             _enable_state;   ///< 启停状态机
  DmPendingAction           _pending_action; ///< 独立于启停状态的一次性特殊指令
  Status                    _status;         ///< 最近一次公开操作结果
  CanTxMsg                  _control_msg;    ///< 已完整编码的周期控制帧缓存
  float                     _last_velocity;  ///< 上一次最终机构输出轴速度，单位：rad/s
  bool                      _initialized;    ///< 是否已完成 init() 注册

  DmMotor *_next;        ///< 同型号反馈分发链表后继节点
  static DmMotor *_head; ///< 同型号反馈分发链表头
  static DmMotor *_tail; ///< 同型号反馈分发链表尾
};

#endif // __DM_MOTOR_HPP__
