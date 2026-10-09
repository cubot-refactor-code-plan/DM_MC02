/**
 * @file device_servo.hpp
 * @author Rh
 * @brief 舵机设备 —— 在 BspPwm 之上提供「角度」语义，并带限速与加速度规划
 * @version 0.1
 * @date 2026-10-08
 *
 * @copyright Copyright (c) 2026
 *
 * @details 一个实例 = 一路 PWM 上的一个位置舵机。BspPwm 只认「脉宽 / 占空比」，而
 *          「0 ~ 180° 对应 500 ~ 2500 µs」这类换算是器件语义，所以放这里；
 *          DeviceBuzzer 之于 BspPwm 也是同一种做法（那边是音调，这边是角度）。
 *
 * @note 目前只覆盖 PWM 位置舵机（180° / 270°）：
 *       每实例的行程由 Config::range_deg 一句话描述：180° 舵机填 180、270° 填 270。
 *
 * @note 换算链（线性）：angle ∈ [0, range_deg] → t = angle / range_deg →
 *       pulse = pulse_min_us + t × (pulse_max_us − pulse_min_us) + trim_us；
 *       reversed = true 时先取 t = 1 − t，把行程两头对调（用于舵机装反的场合）。
 *       算例：range_deg = 180、脉宽 500 ~ 2500 µs ⇒ 90° → t = 0.5 → 1500 µs。
 *
 * @note 上电不输出：init() 只校验配置、不动硬件。BspPwm 上电是 0% 占空比
 *       （引脚恒低、舵机收不到脉冲），要它动就得先 set_angle()，**并且周期调用 update()**。
 *
 * @note 限速与加速度（Config::max_speed_dps / max_accel_dps2，都为 0 表示不限制）：
 *       限速开启后 set_angle() 只记目标，实际角度由 update() 按梯形速度规划推进。
 *       update() 用 tick 差值算 dt，因此**与调用周期无关**（1 ms 调或 10 ms 调都准），
 *       但调得越勤轨迹越平滑。标准用法（同一任务里串起来，顺序不能反）：
 *
 *       servo.set_angle(120.0f);  // 想去的角度，随时可改
 *       servo.update();           // 按时间推进实际角度并写脉宽
 *
 * @note 线程模型（按"单写者"设计：跨任务的变量只用单条 32 位存储，所以不需要加锁、也没有 RTOS 对象）：
 *
 *       | 接口              | 谁调                      | 说明                                        |
 *       | ---------------- | ------------------------ | ------------------------------------------- |
 *       | `update()`       | 固定一个任务，周期调用       | 唯一的 PWM 写入者；不调它就没有输出              |
 *       | `set_pulse_us()` | 须与 update() 同任务       | 它要立即接管输出，没法用"下一拍"表达             |
 *       | `set_angle()`    | 任意任务                   | 只写"目标角度 + 要输出"两个标志                 |
 *       | `off()`          | 任意任务                   | 只清"要输出"标志，真正关断由 update() 下一拍执行 |
 *       | `get_*()`        | 任意任务                   | 各自 32 位原子读，但可能比 update() 晚一拍      |
 *
 *       成立前提：Cortex-M7 单核、32 位对齐的读写在单条 `LDR/STR` 里完成，所以不会读到半截值；
 *       D-Cache 本工程未开，也没有一致性问题。
 *
 * @warning 每条 PWM 通道只由本类一个实例驱动；同一定时器的通道共用 ARR，本类不改频率。
 * @warning 开环器件没有位置反馈：写出去的是"希望它到哪"，不是"它真的在哪"。所以需要自己观测是否到达给延时
 *          堵转、限矩、掉电都不会被本类发现，所以它不注册 Online、不做在线判定。
 */

#ifndef __DEVICE_SERVO_HPP__
#define __DEVICE_SERVO_HPP__

#include "bsp_pwm.hpp" // 被驱动的 PWM 通道
#include "status.hpp"

#include <stdint.h>


/**
 * @brief 舵机设备类
 *
 * @note 不拥有 PWM 通道、不创建 RTOS 资源、不注册 Online（开环器件无从判断在不在线）。
 */
class DeviceServo
{
public:
  // ---------------- 配置 ----------------

  /**
   * @brief 舵机配置（脉宽区间与行程按具体型号填，驱动内不写死硬件常量）
   */
  struct Config
  {
    /** @brief 按序构造配置（参数顺序 = 字段顺序，可匿名传参） */
    Config(BspPwm *pwm = nullptr, float pulse_min_us = 500.0f, float pulse_max_us = 2500.0f, float range_deg = 180.0f, float max_speed_dps = 0.0f, float max_accel_dps2 = 0.0f, bool reversed = false, float trim_us = 0.0f)

      : pwm(pwm),
        pulse_min_us(pulse_min_us),
        pulse_max_us(pulse_max_us),
        range_deg(range_deg),
        max_speed_dps(max_speed_dps),
        max_accel_dps2(max_accel_dps2),
        reversed(reversed),
        trim_us(trim_us)
    {
    }

    BspPwm *pwm;            ///< 绑定的 PWM 通道（须已配成 50 Hz / 20 ms，见 bsp_cfg.cpp 的 bsp_pwm1）
    float   pulse_min_us;   ///< 角度 0 端对应的脉宽 (µs)，常见 500 或 1000
    float   pulse_max_us;   ///< 角度 range_deg 端对应的脉宽 (µs)，常见 2500 或 2000
    float   range_deg;      ///< 行程 (度)：180° 舵机填 180，270° 舵机填 270
    float   max_speed_dps;  ///< 限速 (度/秒)，0 = 不限制（set_angle() 直接到位）
    float   max_accel_dps2; ///< 加速度上限 (度/秒²)，0 = 不限制（速度瞬变到上限）
    bool    reversed;       ///< true = 行程反向（角度 0 走 pulse_max 端）
    float   trim_us;        ///< 脉宽微调 (µs)：机械中位 / 齿轮装配偏差的补偿
  };

  // ----------------
  // ---------------- 构造与析构 ----------------

  /** @brief 必须绑定一路 PWM 通道，禁用默认构造 */
  DeviceServo() = delete;

  /** @brief 只保存配置，不碰硬件（PWM 由 BSP 层 init()） */
  explicit DeviceServo(const Config &cfg);

  /** @brief 默认析构（不持有任何需要释放的资源） */
  ~DeviceServo() = default;

  // 内部持有角度与速度状态，复制会让两份状态各自推进、输出互相打架
  DeviceServo(const DeviceServo &)            = delete;
  DeviceServo &operator=(const DeviceServo &) = delete;
  DeviceServo(DeviceServo &&)                 = delete;
  DeviceServo &operator=(DeviceServo &&)      = delete;

  // ----------------
  // ---------------- 生命周期 ----------------

  /**
   * @brief 校验配置并复位运动状态
   *
   * @return OK=配置合法；BAD_ARG=配置非法（句柄为空、脉宽区间不成立、行程非正、限速/加速度为负、
   *         脉宽超出一个 PWM 周期）
   *
   * @note 只碰软件状态：PWM 通道保持 BSP 层上电时的 0% 占空比。
   *       重复调用相当于复位：输出回到"未开始"，下次还得 set_angle()。
   */
  Status init(void);

  /**
   * @brief 推进实际角度、执行开关输出（**唯一的 PWM 写入者**，固定单任务周期调用）
   *
   * @note 必须周期调用：所有脉宽写入都在这里，不调它既没有输出、也不会执行 off()。
   * @note 周期不敏感 —— dt 取两次调用之间的 tick 差值；但两次调用间隔被限到 100 ms 上限，
   *       避免任务卡住很久之后一口气补一大段角度（那一下的电流冲击很可观）。
   * @note 限速关闭（max_speed_dps = 0）时本函数把脉宽一拍落到目标；限速开启时按梯形规划。
   */
  void update(void);

  // ----------------
  // ---------------- 控制接口 ----------------

  /**
   * @brief 设定目标角度
   * @param deg 目标角度 (度)，超出 [0, range_deg] 自动限幅；非有限值按 0 处理
   *
   * @note 只记录请求，实际动作由 update() 推进 —— 所以"调用 set_angle() 之后立刻读
   *       get_angle_deg()"拿到的是**旧值**，这是刻意的。
   * @note 本函数只写两个跨任务的 32 位变量（目标角度、要输出），因此跨任务调用不需要加锁。
   */
  void set_angle(float deg);

  /**
   * @brief 直接写脉宽（标定脉宽端点、排查接线时用；**须与 update() 同任务**）
   * @param width_us 高电平脉宽 (µs)，超出 [pulse_min_us + trim, pulse_max_us + trim] 自动限幅
   *
   * @note 绕过角度换算与限速，写什么就是什么，并把内部角度状态同步成等价值，
   *       免得之后 update() 从旧角度突然跳回来。
   * @note 它必须立即接管输出，没法用"下一拍"表达，所以是唯一要求与 update() 同任务的写接口。
   */
  void set_pulse_us(float width_us);

  /**
   * @brief 请求停止输出：脉宽 0、引脚保持低电平，舵机因此失カ
   *
   * @note 只是清掉"要输出"请求，真正关断由 update() 下一拍执行（1 kHz 调用时 ≤1 ms）；
   *       角度状态保留，下次 set_angle() 会让输出重新打开。
   */
  void off(void);

  // ----------------
  // ---------------- 开环查询接口 ----------------
  // 下面三个都是"设定/输出侧的估计值"，不是实测值（开环器件没有回读）；
  // 可从任意任务读（各自 32 位原子读），但可能比 update() 晚一拍

  /** @brief 目标角度 (度) */
  float get_target_deg(void) const;

  /** @brief 当前实际输出对应的角度 (度)：限速过程中落后于 target_deg() */
  float get_angle_deg(void) const;

  /** @brief 当前实际输出的脉宽 (µs)，off() 之后为 0 */
  float get_pulse_us(void) const;

  // ----------------
private:
  // ---------------- 私有实现 ----------------

  /**
   * @brief 两次 update() 之间最多按这么长的时间积分 (s)
   *
   * @note 任务被卡住 / 调试断点之后第一次 update()，tick 差值会很大；照实积分会让角度
   *       一步冲出去（电流冲击、机械冲击）。按 100 ms 封顶，宁可走得慢一点也不暴走。
   */
  static constexpr float MAX_DT_S = 0.1f;

  /** @brief 把角度限幅到 [0, range_deg]，并挡住 NaN（!(x > 0) 对 NaN 成立） */
  float _clamp_angle(float deg) const;

  /** @brief 脉宽 → 角度（set_pulse_us() 的逆换算，含 reversed 与 trim 的还原） */
  float _deg_of_pulse(float width_us) const;

  /** @brief 把 _angle_deg 换算成脉宽并写进 PWM 通道（唯一的下行出口） */
  void _write_pulse(void);

  // ----------------
  // ---------------- 成员变量 ----------------

  // ----------------
  // ---------------- 跨任务变量 ----------------
  // 只允许单条 32 位存储的直接赋值（见文件头的"线程模型"），多加一步就可能被读到中间态

  float _target_deg  = 0.0f;  ///< 跨任务：目标角度，set_angle() 写、update() 读
  bool  _enabled_req = false; ///< 跨任务：要输出，set_angle() 置真、off() 置假、update() 执行

  // ----------------
  // ---------------- 成员变量 ----------------

  Config   _cfg;               ///< 脉宽区间、行程与限速参数（init() 之后只读，跨任务读安全）
  float    _angle_deg = 0.0f;  ///< 当前实际输出角度（只有 update() 改）
  float    _speed_dps = 0.0f;  ///< 当前规划速度 (度/秒)，带符号（只有 update() 改）
  float    _pulse_us  = 0.0f;  ///< 最近一次写出去的脉宽 (µs)（update() / set_pulse_us() 改）
  uint32_t _last_tick = 0U;    ///< 上次 update() 的 tick，用来算 dt（只有 update() 改）
  bool     _inited    = false; ///< 是否已成功 init()（init() 之后只读）
  bool     _enabled   = false; ///< 输出当前是否真的开着（只有 update() 改）

  // ----------------
};

#endif // __DEVICE_SERVO_HPP__
