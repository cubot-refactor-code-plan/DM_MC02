/**
 * @file pid.hpp
 * @author ChoseB (ChoseB@cumt.edu.cn) 被修改过
 * @brief 位置式 PID 控制器（比例 + 积分 + 微分 + 前馈，逐项限幅后再总限幅）
 * @version 0.3
 * @date 2026-10-06
 *
 * @copyright Copyright (c) 2026
 *
 * @details 每个实例独立保存自己的运行状态（积分累加值、上一次目标/误差、前馈累加值），
 *          互不干扰。一次 calc() 的计算顺序固定为：
 *
 *          1. 误差       error = target - feedback
 *          2. 一阶差分   delta_target = target - 上一拍 target；
 *                       delta_error = error - 上一拍 error
 *          3. 各分项     P = kp·error；
 *                       I = 上一拍 I + ki·error；
 *                       D = kd·(delta_target 或 delta_error)；
 *                       F = 前馈累加值（由 feed_forward() 提前给出）
 *          4. 逐项限幅   P→p_max、I→i_max、D→d_max、F→f_max
 *          5. 求和总限幅 (P + I + D + F) → out_max，即为本拍输出
 *
 * @note 微分先行(D)：默认取目标值的差分量（微分先行，delta_target）：设定值突变时 D 项不会像
 *       取误差差分那样给出巨大冲击。需要常规微分时把 Config::diff_mode 改成 ERROR。
 *       kd 为 0 时微分项恒为 0，构造时自动把微分项关掉。
 * @note 积分分离：i_separation > 0 且 |error| ≥ i_separation 时把积分**清零**
 *       （大误差下抑制超调）；填 0 表示始终积分。
 * @note 前馈：feed_forward() 累加的值只在下一拍 calc() 里生效一次，
 *       要持续给前馈就得每拍都调一次。这样前馈与 PID 三个分项同拍使用、不会残留。 
 * @note 限幅一律「填 0 = 不限制」：Config::limit 的 out_max / p_max / i_max / d_max / f_max
 *       写 0 都表示关闭该项限制；其中 i_max 写 0 时自动跟随 out_max（见 Limit 构造函数里的说明）。
 *
 * @details 使用示例（参数按「系数 → 限幅 → 微分方式」三段给，后两段可省略）：
 *
 * @code{.cpp}
 *      // 只给三项系数，限幅与微分方式用默认值
 *      Pid speed_pid({{0.8f, 0.0f, 0.0f}});                                    
 *      // 给上 {kp, ki, kd} 与 {out_max, p_max, i_max, d_max, 前馈f_max, 积分分离阈值i_separation}
 *      Pid angle_pid({{0.8f, 1.0f, 1.0f}, {10000.0f, 0.0f, 3000.0f}});
 *
 *      // 系数 + 限幅 + 改微分方式
 *      Pid current_pid({{0.8f, 1.0f, 1.0f}, {10000.0f, 0.0f, 3000.0f}, PidDiffCalcMode::ERROR});
 *
 *      // 复制构造：批量起一组同参数对象
 *      Pid motors_pid[3] = {angle_pid, speed_pid, current_pid};
 *
 *      // 每拍调用；第三个参数（一阶导数）可省略
 *      motors_pid[0].feed_forward(friction_ff());                   // optional：本拍前馈
 *      float u = motors_pid[0].calc(tar, motor.angle, motor.speed); // 外部给速度，省一次差分
 * @endcode
 */

#ifndef __PID_HPP__
#define __PID_HPP__

/** @brief 微分项的计算方式（填进 Pid::Config::diff_mode） */
enum class PidDiffCalcMode
{
  DISABLE = 0x00, ///< 不计算微分项（kd 为 0 时自动进入此模式）
  TARGET  = 0x01, ///< 微分先行：用目标值差分算 D，设定值突变时冲击小（默认）
  ERROR   = 0x02, ///< 常规微分：用误差差分算 D，跟随更紧，但设定值突变时冲击大
};

/**
 * @brief PID 控制器
 *
 * @note 同一实例不是线程安全的：约定由同一个任务按拍调用 calc()。
 */
class Pid
{
public:
  // ---------------- 配置 ----------------

  /** @brief 三项系数（PID 的 kp / ki / kd） */
  struct Gain
  {
    /** @brief 按序构造（参数顺序 = 字段顺序，全部可省略） */
    Gain(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f) : kp(kp), ki(ki), kd(kd)
    {
    }

    float kp; ///< 比例系数
    float ki; ///< 积分系数
    float kd; ///< 微分系数；为 0 时不计算微分项
  };

  /**
   * @brief 限幅与积分分离（限幅类参数一律「填 0 = 不限制」）
   *
   * @note i_max 填 0 时在构造里被置为 out_max：让积分项也不会超过总输出上限，
   *       避免输出一直被限幅、积分迟迟卸不下来。
   */
  struct Limit
  {
    /** @brief 按序构造（参数顺序 = 字段顺序，全部可省略） */
    Limit(float out_max = 0.0f, float p_max = 0.0f, float i_max = 0.0f, float d_max = 0.0f, float f_max = 0.0f, float i_separation = 0.0f) : out_max(out_max), p_max(p_max), i_max(i_max == 0.0f ? out_max : i_max), d_max(d_max), f_max(f_max), i_separation(i_separation)
    {
    }

    float out_max;      ///< 总输出限幅（绝对值），0 = 不限制
    float p_max;        ///< 比例项限幅（绝对值），0 = 不限制
    float i_max;        ///< 积分项限幅（绝对值），构造时 0 → 跟随 out_max
    float d_max;        ///< 微分项限幅（绝对值），0 = 不限制
    float f_max;        ///< 前馈项限幅（绝对值），0 = 不限制
    float i_separation; ///< 积分分离阈值，0 = 始终积分
  };

  /** @brief 控制器全部参数 = 系数 + 限幅 + 微分方式（可直接用匿名 {...} 按段传入） */
  struct Config
  {
    /** @brief 按序构造（参数顺序 = 字段顺序，后两段可省略） */
    Config(Gain gain = Gain(), Limit limit = Limit(), PidDiffCalcMode diff_mode = PidDiffCalcMode::TARGET) : gain(gain), limit(limit), diff_mode(diff_mode)
    {
    }

    Gain            gain;      ///< 三项系数
    Limit           limit;     ///< 限幅与积分分离
    PidDiffCalcMode diff_mode; ///< 微分项取值方式；kd 为 0 时被构造改为 DISABLE
  };

  // ----------------
  // ---------------- 公有接口 ----------------

  /**
   * @brief 构造控制器
   * @param config 全部参数；可省略，省略 = 系数与限幅全为 0（输出恒为 0）
   */
  Pid(Config config = Config());

  /**
   * @brief 计算一次输出（微分量由内部对目标/误差做一阶差分得到）
   * @param target 目标值
   * @param feedback 反馈值
   * @return float 本拍限幅后的输出
   */
  float calc(float target, float feedback);

  /**
   * @brief 计算一次输出（微分量由外部直接给，避免内部再差分一次放大噪声）
   * @param target 目标值
   * @param feedback 反馈值
   * @param df_dt 一阶导数（如位移环里直接给速度），作用于哪个量由 diff_mode 决定
   * @return float 本拍限幅后的输出
   */
  float calc(float target, float feedback, float df_dt);

  /**
   * @brief 叠加一份前馈量（只在下一拍 calc() 里生效一次）
   * @param feedforward 前馈值，一拍内可多次调用累加
   * @return float 当前累计的前馈总和
   */
  float feed_forward(float feedforward);

  // ----------------
  // ---------------- 私有实现 ----------------

  /** @brief 记录本拍 target / feedback，算出误差与两个一阶差分量（供 _calc_output() 使用） */
  void _calc_input(float target, float feedback);

  /** @brief 用当前误差、积分累加值、微分与前馈算一次输出并限幅（两个 calc 重载共用） */
  float _calc_output();

  /** @brief 对称限幅：|value| 超过 limit 就截到 ±limit；limit <= 0 表示不限制（与 Limit 的约定一致） */
  static float _clamp_sym(float value, float limit);

  // ----------------
  // ---------------- 成员变量 ----------------

  Config _cfg;          ///< 参数（构造时若 kd == 0 则把 diff_mode 改成 DISABLE）
  float  _error;        ///< 本拍误差 error = target - feedback
  float  _last_error;   ///< 上一拍误差（算 delta_error 用）
  float  _last_target;  ///< 上一拍目标值（算 delta_target 用）
  float  _delta_target; ///< 本拍目标一阶差分（可被三参数 calc 覆盖）
  float  _delta_error;  ///< 本拍误差一阶差分（可被三参数 calc 覆盖）
  float  _i_term;       ///< 积分项累加值（唯一需要跨拍保留的分项）
  float  _f_term;       ///< 前馈累加值（每次输出后清零）

  // ----------------
};

#endif // __PID_HPP__
