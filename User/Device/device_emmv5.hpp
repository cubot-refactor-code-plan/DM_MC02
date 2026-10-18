/**
 * @file device_emmv5.hpp
 * @author Rh
 * @brief 张大头 EmmV5.0 闭环步进电机驱动（单串口单电机）
 * @version 1.0
 * @date 2026-10-18
 *
 * @copyright Copyright (c) 2026
 *
 * @details 一个实例 = 一路串口上的一个电机地址。串口由 BSP 层初始化，本类只负责：
 *
 *          1. 调 emmv5 协议层生成命令帧；
 *          2. 把帧写进 BspUart 的发送流缓冲区；
 *          3. 从 BspUart 读字节，识别「到位返回帧」（地址 + 0xFD 0x9F 0x6B）。
 *
 *          它不创建任务、不注册全局表、不管理信号量，和 BspCan / DjiMotorGroup 的
 *          「设备只做设备的事」保持一致。
 *
 * @note 适用场景：**一路串口只接一台电机**。此时本类是这路串口唯一的接收消费者，
 *       可以直接调 update() 轮询。如果一路串口要挂多台电机（地址不同、需要 MMCL
 *       多机命令），请改用 DeviceEmmv5Group。
 *
 * @note 初始化（实例一般写在 device_cfg.cpp）：
 *
 *       DeviceEmmv5 motor({bsp_uart1, 1});   // 地址 1，地址必须 1~255
 *       motor.init();
 *
 * @note 运动控制：
 *
 *       motor.enable(true);                                    // 使能
 *       motor.set_velocity(emmv5::Direction::CW, 1000, 50);   // CW, 1000 RPM, 加速度 50
 *       motor.set_position(emmv5::Direction::CW, 500, 50, 3200,
 *                          emmv5::PositionMode::Absolute);    // 绝对位置，走 3200 个脉冲
 *       motor.stop();
 *
 * @note 周期接收（在一个任务里调用，非阻塞）：
 *
 *       if (motor.update())
 *       {
 *         // 本次读到了到位帧
 *       }
 *
 * @note 改 ID、细分、保护阈值用对应的语义方法；不要在 App 层拼协议帧。
 *       其余不常用的配置命令，需要时在设备层补一个方法即可。
 *
 * @warning update() 只能由同一个任务调用，不能并发；
 *          否则同一个流缓冲区会被两个消费者抢字节。
 * @warning 命令帧的发送失败会通过 Status 返回；上层如果只关心控制周期，可以忽略返回值，
 *          失败计数在 BspUart::tx_diag() 里查。
 */

#ifndef __DEVICE_EMMV5_HPP__
#define __DEVICE_EMMV5_HPP__

#include "bsp_uart.hpp"
#include "emmv5_protocol.hpp"
#include "status.hpp"

#include <stddef.h>
#include <stdint.h>


/**
 * @brief EmmV5.0 单电机驱动（一路串口一台电机）
 *
 * @note 未 init() 时所有发送接口返回 NOT_INIT，update() 返回 false。
 */
class DeviceEmmv5
{
public:
  // ---------------- 类型与配置 ----------------

  using Uart = BspUart<128>; ///< 本工程统一使用的串口类型

  /**
   * @brief 电机配置（可匿名按序传入）
   */
  struct Config
  {
    /**
     * @brief 按序构造配置
     * @param uart 电机所接的串口
     * @param addr 电机地址（1~255；0 是广播地址，不能用于实例）
     */
    Config(Uart &uart, uint8_t addr) : uart(uart), addr(addr)
    {
    }

    Uart   &uart; ///< bsp_uart 实例引用
    uint8_t addr; ///< 电机地址（1~255）
  };

  // ----------------
  // ---------------- 构造与析构 ----------------

  /**
   * @brief 构造函数
   * @param cfg 电机配置
   * @note 只做赋值；地址校验在 init() 里做。
   */
  explicit DeviceEmmv5(const Config &cfg);

  /** @brief 不持有需要释放的资源 */
  ~DeviceEmmv5() = default;

  // 内部有接收窗口状态，复制会让两份状态各自推进
  DeviceEmmv5(const DeviceEmmv5 &)            = delete;
  DeviceEmmv5 &operator=(const DeviceEmmv5 &) = delete;
  DeviceEmmv5(DeviceEmmv5 &&)                 = delete;
  DeviceEmmv5 &operator=(DeviceEmmv5 &&)      = delete;

  // ----------------
  // ---------------- 生命周期 ----------------

  /**
   * @brief 校验地址并复位接收窗口
   * @return OK=可用；BAD_ARG=地址为 0
   * @note 幂等，不发送任何命令。
   */
  Status init();

  ///< 是否已成功 init
  bool is_ready() const
  {
    return _ready;
  }

  ///< 电机地址
  uint8_t addr() const
  {
    return _addr;
  }

  // ----------------
  // ---------------- 常用命令 ----------------

  /**
   * @brief 使能/关闭电机
   * @param on   true=使能
   * @param sync 多机同步标志（单机一般填 false）
   */
  Status enable(bool on, bool sync = false);

  /**
   * @brief 速度模式
   * @param dir   方向
   * @param rpm   速度（0~5000 RPM）
   * @param accel 加速度（0~255，0=直接启动）
   * @param sync  多机同步标志
   */
  Status set_velocity(emmv5::Direction dir, uint16_t rpm, uint8_t accel, bool sync = false);

  /**
   * @brief 位置模式
   * @param dir    方向
   * @param rpm    速度（0~5000 RPM）
   * @param accel  加速度（0~255）
   * @param pulses 脉冲数
   * @param mode   位置模式
   * @param sync   多机同步标志
   */
  Status set_position(emmv5::Direction dir, uint16_t rpm, uint8_t accel, uint32_t pulses, emmv5::PositionMode mode, bool sync = false);

  /**
   * @brief 设置快速位置模式的运动参数
   * @param rpm   速度（0~5000 RPM）
   * @param accel 加速度（0~255）
   * @param mode  位置模式
   * @param sync  多机同步标志
   */
  Status set_quick_position_params(uint16_t rpm, uint8_t accel, emmv5::PositionMode mode, bool sync = false);

  /**
   * @brief 快速位置控制
   * @param pulses 带符号脉冲数，16 细分下 ±3200 = ±1 圈
   */
  Status set_quick_position(int32_t pulses);

  /**
   * @brief 立即停止
   * @param sync 多机同步标志
   */
  Status stop(bool sync = false);

  ///< 触发多机同步开始运动
  Status sync_start();

  ///< 触发编码器校准
  Status calibrate_encoder();

  ///< 将当前位置清零
  Status zero_position();

  ///< 解除堵转保护
  Status clear_stall();

  ///< 重启电机（Y42）
  Status reset();

  ///< 恢复出厂设置
  Status restore_defaults();

  /**
   * @brief 触发回零
   * @param mode 回零模式
   * @param sync 多机同步标志
   */
  Status home(emmv5::HomingMode mode, bool sync = false);

  ///< 强制中断并退出回零
  Status abort_home();

  /**
   * @brief 读取一次系统参数
   * @param param 要读取的参数
   * @note 回复由上层从串口读取；本驱动只负责把命令发出去。
   */
  Status read(emmv5::SystemParam param);

  /**
   * @brief 定时自动返回系统参数（Y42）
   * @param param     要返回的参数
   * @param period_ms 返回周期（ms）
   */
  Status auto_report(emmv5::SystemParam param, uint16_t period_ms);

  /**
   * @brief 修改 PID 参数
   * @param store true = 存进电机
   * @param kp    比例系数
   * @param ki    积分系数
   * @param kd    微分系数
   */
  Status set_pid(bool store, uint32_t kp, uint32_t ki, uint32_t kd);

  // ----------------
  // ---------------- 配置命令 ----------------

  /**
   * @brief 修改电机 ID
   * @param store true = 存进电机
   * @param id    新地址（1~255）
   */
  Status set_id(bool store, uint8_t id);

  /**
   * @brief 修改细分值
   * @param store      true = 存进电机
   * @param microstep  细分值（1~255，0=256 细分）
   */
  Status set_microstep(bool store, uint8_t microstep);

  /**
   * @brief 修改过热过流保护阈值（Y42）
   * @param store           true = 存进电机
   * @param over_temp_c     过热保护阈值（℃）
   * @param over_current_ma 过流保护阈值（mA）
   * @param detect_ms       检测时间（ms）
   */
  Status set_protection(bool store, uint16_t over_temp_c, uint16_t over_current_ma, uint16_t detect_ms);

  // ----------------
  // ---------------- 接收 ----------------

  /**
   * @brief 把串口里当前攒的字节全部读完，并识别到位帧（非阻塞）
   * @return true=本次读到至少一个完整到位帧
   *
   * @note 每次最多读 8 块 × 16 字节，避免数据持续到达时把调用任务卡在这里。
   * @note 只能由唯一一个任务调用，不能并发。
   */
  bool update();

  // ----------------
private:
  // ---------------- 私有实现 ----------------

  /**
   * @brief 发送一帧协议命令（内部使用）
   * @param frame 由 emmv5 协议层生成的帧
   * @return OK=整帧已入发送缓冲；NOT_INIT=未 init；BAD_ARG=空帧；
   *         TIMEOUT=发送缓冲长时间没腾出空间；IO_ERROR=串口未启用发送
   *
   * @note 使用 portMAX_DELAY 入队：单条命令最长约 20 字节，宁可短暂等待，
   *       也不要把半条命令留在缓冲区里造成协议错位。
   */
  Status _send(const emmv5::Frame &frame);

  /**
   * @brief 把一块字节喂进 4 字节滑动窗口（内部使用）
   * @param data 数据
   * @param size 字节数
   * @return true=本次凑齐了完整到位帧
   */
  bool feed_rx(const uint8_t *data, size_t size);
  // ----------------

  // ---------------- 成员变量 ----------------

  static constexpr size_t RX_CHUNK_SIZE  = 16U; ///< 单次从流缓冲区读多少字节
  static constexpr size_t RX_MAX_CHUNKS  = 8U;  ///< update() 每次最多读几块

  Uart    &_uart; ///< 被驱动的串口（不拥有）
  uint8_t  _addr; ///< 电机地址

  bool    _ready = false;                  ///< init() 是否成功
  emmv5::InPositionScanner _scanner;      ///< 到位帧识别（协议层实现）

  // ----------------
};

#endif // __DEVICE_EMMV5_HPP__
