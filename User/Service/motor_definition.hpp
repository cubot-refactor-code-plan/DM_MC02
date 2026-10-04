/**
 * @file motor_definition.hpp
 * @author ChoseB
 * @brief 电机通用运动学数据和观测器输出定义
 * @version 0.2
 * @date 2026-09-02
 *
 * @note 本文件只保存所有电机都具有的可复制数据，不包含具体反馈协议、硬件所有权、
 *       在线检查或 CAN 资源。协议原始值和协议限制由具体电机类型自行定义。
 */
#ifndef __MOTOR_DEFINITION_HPP__
#define __MOTOR_DEFINITION_HPP__

#ifndef PI
#  define PI 3.1415926f
#endif // PI

/**
 * @brief 单个电机经过机械传动换算后的通用运动学数据
 *
 * @note radian_data 统一表示最终机构输出轴；具体电机首先将协议原始值换算到
 *       上一级设备轴，再按 (设备轴角度 - offset) / ratio 换算到最终输出轴。
 */
struct MotorData
{
  /**
   * @brief 最终机构输出轴的 SI 制运动量
   */
  struct Radian
  {
    float angle_single_round; ///< 单圈机械角度，单位：rad
    float angle_multi_round;  ///< 累计多圈机械角度，单位：rad
    float velocity;           ///< 角速度，单位：rad/s
    float acceleration;       ///< 角加速度，单位：rad/s^2
  } radian_data;

  /**
   * @brief 从具体电机设备轴到最终机构输出轴的通用机械参数
   */
  struct Param
  {
    float offset; ///< 设备轴机械零位偏移，单位：rad
    float ratio;  ///< 设备轴转速与最终机构输出轴转速之比，必须大于 0
  } param;
};

/**
 * @brief 可选的 Luenberger 电机状态观测结果
 *
 * @note 该结构属于可选能力，并非每个电机实例都必须启用或使用观测结果。
 */
struct LuenbergerMotorData
{
  float angle;        ///< 观测得到的累计机械角度，单位：rad
  float velocity;     ///< 观测得到的角速度，单位：rad/s
  float acceleration; ///< 观测得到的角加速度，单位：rad/s^2
};

#endif // __MOTOR_DEFINITION_HPP__
