/**
 * @file pid.hpp
 * @author ChoseB (ChoseB@cumt.edu.cn)
 * @brief PID 算法的封装
 * @version 0.2
 * @date 2026-03-07
 *
 * @copyright Copyright (c) 2026
 *
 * @details 使用示例（只有一个构造函数：系数 + 限幅，限幅可省略）：
 *
 *      // 只给 kp / ki / kd，不限幅
 *      Pid motor1_pid({0.8f, 0.0f, 0.0f});
 *
 *      // 给全参数：{kp, ki, kd} + {out_max, p_max, i_max, d_max, f_max, i_separation}
 *      Pid motor2_pid({0.8f, 1.0f, 1.0f}, {10000.0f, 0.0f, 3000.0f, 0.0f, 0.0f, 50.0f});
 *
 *      // 复制构造：批量起一组同参数对象
 *      Pid motors_pid[4] = {motor2_pid, motor2_pid, motor2_pid, motor2_pid};
 *
 *      // 有必要才禁用微分先行
 *      motor2_pid.switch_mode_diff_calc(PidDiffCalcMode::ERROR);
 *
 * @note 使用（假设已有 Pid 对象 pid；calc 的第三个参数可省略）：
 *
 *      pid.feed_forward(friction_ff());        // optional
 *      pid.feed_forward(follow_ff(d_target));  // optional
 *      pid.calc(tar, motor.angle, motor.speed);
 */

#ifndef __PID_HPP__
#define __PID_HPP__

/** @brief PID 系数 */
struct PidParam
{
  float kp; ///< 比例项系数
  float ki; ///< 积分项系数
  float kd; ///< 微分项系数

  PidParam();
  PidParam(float kp, float ki, float kd);
};

/** @brief 各项限幅（写 0 表示关闭该项限制） */
struct PidLimitation
{
  float out_max;      ///< 总输出最大限制，写 0 则不限制
  float p_max;        ///< 比例项最大限制，写 0 则关闭
  float i_max;        ///< 积分项最大限制，写 0 则跟随 out_max
  float d_max;        ///< 微分项最大限制，写 0 则关闭
  float f_max;        ///< 前馈项最大限制，写 0 则关闭
  float i_separation; ///< 积分分离阈值，写 0 则关闭积分分离

  PidLimitation();
  PidLimitation(float out_max, float p_max, float i_max, float d_max, float f_max, float i_separation);
};

/** @brief 各项计算值 */
struct PidTerm
{
  float p_term; ///< 比例项计算值
  float i_term; ///< 积分项计算值
  float d_term; ///< 微分项计算值
  float f_term; ///< 前馈项计算值

  PidTerm();
};

/** @brief 输入量与中间量 */
struct PidInput
{
  float target;   ///< 当前目标值
  float feedback; ///< 当前反馈值
  float error;    ///< 当前误差值，error = target - feedback

  float last_target; ///< 上一目标值
  float last_error;  ///< 上一误差值

  float delta_target; ///< 目标值微分；也可由外部直接传入
  float delta_error;  ///< 误差值微分；也可由外部直接传入
};

/** @brief 微分项计算方式 */
enum class PidDiffCalcMode
{
  TARGET  = 0x01, ///< 使用 delta_target 计算微分项（微分先行）
  ERROR   = 0x02, ///< 使用 delta_error 计算微分项（常规微分）
  DISABLE = 0x00, ///< 不计算微分项
};

/** @brief PID 控制器 */
class Pid
{
protected:
  PidParam        _param;          ///< 系数
  PidLimitation   _lim;            ///< 限幅
  PidTerm         _term;           ///< 各项计算值
  PidDiffCalcMode _diff_calc_mode; ///< 微分项计算模式

  /** @brief 根据传入的 target / feedback 计算误差与微分 */
  virtual void calc_input(float target, float feedback);

private:
  /** @brief 用当前 input 与前馈项算一次输出并限幅（两个 calc 重载共用） */
  float _calc_output();

public:
  PidInput input;  ///< 传入
  float    output; ///< 传出

  /**
   * @brief 构造
   * @param param 系数 {kp, ki, kd}
   * @param limitation 限幅 {out_max, p_max, i_max, d_max, f_max, i_separation}；
   *                   可省略，省略等于全部不限幅
   */
  Pid(PidParam param, PidLimitation limitation = PidLimitation());

  /**
   * @brief 切换微分项计算方式（通常在不适用于微分先行的动态系统中补充初始化）
   * @param mode 微分项计算方式；用 PidDiffCalcMode::ERROR 禁用微分先行
   */
  void switch_mode_diff_calc(PidDiffCalcMode mode);

  /**
   * @brief 将某一前馈函数预测的值引入计算
   * @param feedforward 前馈函数返回的数值
   * @return float 前馈项总和
   */
  float feed_forward(float feedforward);

  /**
   * @brief PID 计算
   * @param target 目标值
   * @param feedback 反馈值
   * @return float 输出
   */
  float calc(float target, float feedback);

  /**
   * @brief PID 计算（一阶导数由外部导入，精度高于内部差分）
   * @param target 目标值
   * @param feedback 反馈值
   * @param df_dt 一阶导数；作用到哪个量由微分模式决定
   *              （TARGET → input.delta_target，ERROR → input.delta_error）
   * @note 被控量为位移 x 时，df_dt 填速度 v（微分先行 / 常规微分都适用），
   *       避免内部再差分一次把反馈噪声放大
   * @return float 输出
   */
  float calc(float target, float feedback, float df_dt);

  /** @brief 快速打印 target 和 feedback 到 vofa [暂未实现] */
  void print();
};

#endif // __PID_HPP__
