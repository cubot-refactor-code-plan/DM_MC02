/**
 * @file device_emmv5_group.hpp
 * @author Rh
 * @brief 张大头 EmmV5.0 多电机组（单串口多电机 + MMCL 多机命令）
 * @version 1.1
 * @date 2026-10-18
 *
 * @copyright Copyright (c) 2026
 *
 * @details 一个实例 = 一路串口 + 若干个电机地址。它只做四件事：
 *
 *          1. 从串口读字节，按地址分发给组内各电机；
 *          2. 把多条子命令攒成 MMCL 总帧，通过广播地址一次发出；
 *          3. 汇总到位位图、0x36 位置、0x3A 状态；
 *          4. 每台电机维护一个 Online 节点，判定“最近是否收到过有效应答”。
 *
 *          它不创建全局表、不管理信号量，但会在 init() 时创建一个**内部 RX 任务**：
 *          该任务每 10 ms 调一次 update()，按地址把字节分发给组内各电机。
 *          任务入口是本类独有的静态函数，地址与 DeviceEmmv5 不同。
 *
 * @note 扫描器与超时模型：
 *
 *       - `update()` 只做“读字节 + 按地址分发”，非阻塞；由内部 RX 任务推进；
 *       - 每台电机各自有到位帧 / 0x36 / 0x3A 三个扫描器，命中后锁存；
 *       - 位置/状态用 `take_position()` / `take_status()` 取走，取后清有效标志；
 *       - 任意一台收到有效应答都会刷新它自己的 Online 节点；
 *       - Online 阈值 **100 ms**，计时由 sys_task 里的 `Online::update()` 统一推进；
 *       - 查询周期应小于 100 ms，否则“没被问”的电机也会显示离线。
 *
 * @note 适用场景：**一路串口接多台电机**。一路串口只接一台时用 DeviceEmmv5。
 *
 * @note 初始化（实例一般写在 device_cfg.cpp）：
 *
 *       DeviceEmmv5Group group({bsp_uart4, {1, 2, 3}});  // 串口 + 地址表
 *       group.init();
 *
 * @note 普通命令（直接发给某个地址）：
 *
 *       group.enable(1, true);
 *       group.set_velocity(2, emmv5::Direction::CW, 800, 50);
 *
 * @note MMCL 多机命令：先装子命令，再统一广播：
 *
 *       group.multi_clear();
 *       group.queue_enable(1, true);
 *       group.queue_enable(2, true);
 *       group.queue_velocity(1, emmv5::Direction::CW, 500, 50);
 *       group.queue_velocity(2, emmv5::Direction::CCW, 500, 50);
 *       group.multi_send();
 *
 * @note 周期接收由内部 RX 任务完成，业务层直接取结果即可：
 *
 *       uint32_t latch = group.take_in_position();   // 取走累计到位并清零
 *
 *       int32_t position = 0;
 *       if (group.take_position(1, position)) { ... } // 新到的 0x36 位置
 *
 * @warning update() 只能由内部 RX 任务调用；本类必须是这路串口唯一的接收消费者。
 *          take_* / 发送接口可从其他任务调用，但 **MMCL 缓冲（multi_clear、queue_ 系列、multi_send）
 *          必须由同一个任务串行操作**，否则子命令会互相拼接。
 * @warning 本对象必须是全局/静态生命周期；不要在 RX 任务自身里销毁对象。
 * @warning MMCL 子命令总字节数最多 123（BspUart<128> 的整帧上限）；
 *          queue_* 放不下时返回 FULL，不会留下半条子命令。
 */

#ifndef __DEVICE_EMMV5_GROUP_HPP__
#define __DEVICE_EMMV5_GROUP_HPP__

#include "bsp_uart.hpp"
#include "emmv5_protocol.hpp"
#include "online_check.hpp"
#include "status.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include <initializer_list>
#include <stddef.h>
#include <stdint.h>


/**
 * @brief 单串口多电机组
 *
 * @note 地址表在构造时一次定完，运行期不增删；地址重复或为 0 时 init() 返回 BAD_ARG。
 * @note 未 init() 时所有发送接口返回 NOT_INIT，update() 返回 0。
 */
class DeviceEmmv5Group
{
private:
  // ---------------- 内部常量 ----------------

  static constexpr size_t UART_BUFFER_SIZE  = 128U; ///< 与 bsp_cfg.cpp 里的 BspUart<128> 保持一致
  static constexpr size_t MAX_MOTORS        = 8U;   ///< 组内最大电机数
  static constexpr uint16_t ONLINE_TIMEOUT_MS = 100U; ///< 超过 100 ms 无有效应答判离线

  ///< MMCL 子命令总字节上限：整帧不能超过串口发送缓冲
  static constexpr size_t MULTI_PAYLOAD_MAX = UART_BUFFER_SIZE - emmv5::MULTI_OVERHEAD;

  // ----------------
public:
  // ---------------- 类型与配置 ----------------

  using Uart = BspUart<UART_BUFFER_SIZE>;

  /**
   * @brief 组配置
   *
   * @note 地址表用初始化列表传，例如 `{bsp_uart4, {1, 2, 3}}`。
   *       超过 MAX_MOTORS 的部分不保存，init() 会返回 BAD_ARG。
   */
  struct Config
  {
    /**
     * @brief 按序构造配置
     * @param uart  本组使用的串口
     * @param addrs 电机地址表（每个地址 1~255，不能重复）
     */
    Config(Uart &uart, std::initializer_list<uint8_t> addrs) : uart(uart), count(addrs.size())
    {
      for (size_t i = 0; i < MAX_MOTORS; i++)
      {
        addr[i] = 0;
      }

      size_t i = 0;
      for (uint8_t a : addrs)
      {
        if (i < MAX_MOTORS)
        {
          addr[i++] = a;
        }
      }
    }

    Uart   &uart;             ///< bsp_uart 实例引用
    uint8_t addr[MAX_MOTORS]; ///< 地址表（前 count 个有效）
    size_t  count;            ///< 传入的地址数（可能大于 MAX_MOTORS，由 init() 拒绝）
  };

  // ----------------
  // ---------------- 构造与析构 ----------------

  /**
   * @brief 构造函数
   * @param cfg 组配置
   * @note 只做赋值；地址校验在 init() 里做。
   */
  explicit DeviceEmmv5Group(const Config &cfg);

  /** @brief 停止内部 RX 任务；各电机的 Online 节点随成员析构自动注销 */
  ~DeviceEmmv5Group();

  // 内部有接收窗口与 MMCL 缓冲，复制会让两份状态互相打架
  DeviceEmmv5Group(const DeviceEmmv5Group &)            = delete;
  DeviceEmmv5Group &operator=(const DeviceEmmv5Group &) = delete;
  DeviceEmmv5Group(DeviceEmmv5Group &&)                 = delete;
  DeviceEmmv5Group &operator=(DeviceEmmv5Group &&)      = delete;

  // ----------------
  // ---------------- 生命周期与查询 ----------------

  /**
   * @brief 校验地址表、复位接收状态，并创建内部 RX 任务
   * @return OK=可用；BAD_ARG=地址表为空、超过 MAX_MOTORS、地址为 0 或地址重复；
   *         FULL=内部 RX 任务创建失败（堆/TCB 不足）
   * @note 幂等：重复 init 不会重复创建任务。
   * @warning 应在调度器启动前调用；运行期重复 init() 会复位扫描器，可能丢掉正在接收的半帧。
   */
  Status init();

  ///< 是否已成功 init
  bool is_ready() const
  {
    return _ready;
  }

  ///< 组内电机数（未 init 时返回 0）
  size_t count() const
  {
    return _ready ? _count : 0U;
  }

  // ----------------
  // ---------------- 直接命令 ----------------

  ///< 使能/关闭某个电机
  Status enable(uint8_t addr, bool on);

  ///< 速度模式（rpm 0~5000，accel 0~255）
  Status set_velocity(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel);

  ///< 位置模式（pulses 带符号；绝对模式按 32 位补码写入）
  Status set_position(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel, int32_t pulses, emmv5::PositionMode mode);

  ///< 设置快速位置模式的运动参数
  Status set_quick_position_params(uint8_t addr, uint16_t rpm, uint8_t accel, emmv5::PositionMode mode);

  ///< 快速位置控制
  Status set_quick_position(uint8_t addr, int32_t pulses);

  ///< 立即停止某个电机
  Status stop(uint8_t addr);

  ///< 广播立即停止（地址 0）
  Status stop_all();

  ///< 触发回零
  Status home(uint8_t addr, emmv5::HomingMode mode);

  ///< 将某个电机当前位置清零
  Status zero_position(uint8_t addr);

  ///< 触发编码器校准
  Status calibrate_encoder(uint8_t addr);

  ///< 解除堵转保护
  Status clear_stall(uint8_t addr);

  ///< 重启电机
  Status reset(uint8_t addr);

  ///< 恢复出厂设置
  Status restore_defaults(uint8_t addr);

  ///< 强制中断并退出回零
  Status abort_home(uint8_t addr);

  ///< 修改细分值
  Status set_microstep(uint8_t addr, bool store, uint8_t microstep);

  ///< 修改电机运动正方向
  Status set_motor_direction(uint8_t addr, bool store, emmv5::Direction dir);

  ///< 修改开环/闭环控制模式
  Status set_control_mode(uint8_t addr, bool store, emmv5::ControlMode mode);

  ///< 修改位置到达窗口（单位 0.1°）
  Status set_position_window(uint8_t addr, bool store, uint16_t window);

  ///< 修改过热过流保护阈值
  Status set_protection(uint8_t addr, bool store, uint16_t over_temp_c, uint16_t over_current_ma, uint16_t detect_ms);

  /**
   * @brief 读取一次系统参数
   * @note 本类只解析 `CurrentPosition`（0x36）与 `StatusFlag`（0x3A）；
   *       其余参数的应答会被 update() 丢弃。
   */
  Status read(uint8_t addr, emmv5::SystemParam param);

  ///< 请求一次 0x36 实时位置
  Status request_position(uint8_t addr);

  ///< 请求一次 0x3A 状态标志
  Status request_status(uint8_t addr);

  // ----------------
  // ---------------- MMCL 多机命令 ----------------

  ///< 清空 MMCL 缓冲
  void multi_clear();

  /**
   * @brief 把 MMCL 缓冲封成一帧并发送
   * @param target 总帧地址，默认广播 0
   * @return OK=已发送并清空缓冲；NOT_INIT=未 init；BAD_ARG=组帧失败；
   *         TIMEOUT/IO_ERROR=发送失败（缓冲同样清空，避免重发“半帧 + 完整帧”）
   *
   * @note 缓冲为空时直接返回 OK，不占用串口。
   */
  Status multi_send(uint8_t target = emmv5::BROADCAST);

  ///< 追加「使能」子命令
  Status queue_enable(uint8_t addr, bool on);

  ///< 追加「速度模式」子命令
  Status queue_velocity(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel);

  ///< 追加「位置模式」子命令
  Status queue_position(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel, int32_t pulses, emmv5::PositionMode mode);

  ///< 追加「快速位置参数」子命令
  Status queue_quick_position_params(uint8_t addr, uint16_t rpm, uint8_t accel, emmv5::PositionMode mode);

  ///< 追加「快速位置」子命令
  Status queue_quick_position(uint8_t addr, int32_t pulses);

  ///< 追加「立即停止」子命令
  Status queue_stop(uint8_t addr);

  ///< 追加「回零」子命令
  Status queue_home(uint8_t addr, emmv5::HomingMode mode);

  // ----------------
  // ---------------- 接收 ----------------

  /**
   * @brief 取走累计的到位位图并清零
   * @return bit i = 下标 i 的电机自上次取走后至少到位过一次
   */
  uint32_t take_in_position();

  /**
   * @brief 查询某个电机是否累计到位（不清零）
   * @param index 下标（0 ~ count()-1，顺序与 Config 里的地址表一致）
   * @return true=自上次 take_in_position() 后至少到位过一次
   */
  bool in_position(size_t index) const;

  /**
   * @brief 取走某台电机最近一次 0x36 位置
   * @param addr 电机地址
   * @param out  位置，单位 counts（65536/圈，带符号）
   * @return true=有新数据；false=没有新数据或地址不在组内
   *
   * @note 读后清有效标志；数值与标志在短临界区内一起拷贝，允许在其他任务调用。
   */
  bool take_position(uint8_t addr, int32_t &out);

  /**
   * @brief 取走某台电机最近一次 0x3A 状态字节
   * @param addr 电机地址
   * @param out  状态字节（位定义见说明书 6.3.4）
   * @return true=有新数据；false=没有新数据或地址不在组内
   */
  bool take_status(uint8_t addr, uint8_t &out);

  /**
   * @brief 某台电机最近 100 ms 内是否收到过有效应答
   * @param addr 电机地址
   * @return true=在线；false=超时或地址不在组内
   *
   * @note 计时由 sys_task 里的 Online::update() 推进；查询周期应小于 100 ms。
   */
  bool is_online(uint8_t addr) const;

  // ----------------
private:
  // ---------------- 内部实现 ----------------

  /**
   * @brief 把串口里当前攒的字节全部读完并分发给组内电机（非阻塞）
   * @return 本次命中的电机位图：bit i = 下标 i 的电机收到到位帧
   *
   * @note 只由内部 RX 任务调用；业务层不要直接调。
   *       每次最多读 8 块 × 16 字节，数据持续到达时也会把控制权还给其他任务。
   */
  uint32_t update();

  /**
   * @brief 单个电机的接收状态
   *
   * @note 每个 Motor 自带三个扫描器和一个 Online 节点；节点在静态构造时自动挂到
   *       Online 链表，在线计时由 sys_task 的 Online::update() 统一推进。
   */
  struct Motor
  {
    Motor() : online(ONLINE_TIMEOUT_MS)
    {
    }

    uint8_t                  addr = 0;            ///< 电机地址
    emmv5::InPositionScanner inpos_scanner;       ///< 到位帧识别
    emmv5::PositionScanner   position_scanner;    ///< 0x36 位置应答
    emmv5::StatusScanner     status_scanner;      ///< 0x3A 状态应答
    Online                   online;              ///< 在线判定节点
    bool                     in_position = false; ///< 累计到位标志
    int32_t                  position = 0;        ///< 最近一次位置
    uint8_t                  status = 0;          ///< 最近一次状态
    bool                     position_valid = false; ///< 位置是否未被取走
    bool                     status_valid = false;   ///< 状态是否未被取走
  };

  static constexpr size_t RX_CHUNK_SIZE = 16U; ///< 单次从流缓冲区读多少字节
  static constexpr size_t RX_MAX_CHUNKS = 8U;  ///< update() 每次最多读几块

  static constexpr uint32_t    RX_TASK_PERIOD_MS   = 10U; ///< 内部 RX 任务周期
  static constexpr UBaseType_t RX_TASK_PRIORITY    = tskIDLE_PRIORITY + 6U; ///< 低于 sys_task(+7)
  static constexpr uint32_t    RX_TASK_STACK_WORDS = 384U; ///< 384 words = 1.5 KB

  /**
   * @brief 内部 RX 任务入口
   * @param argument 本对象指针（this）
   *
   * @note 这是 DeviceEmmv5Group 独有的入口地址，与 DeviceEmmv5 的入口不同。
   *       同类多实例共享入口地址，但各自有独立 TCB、栈和 this。
   */
  static void _rx_task_entry(void *argument);

  ///< 内部接收循环：固定周期调用 update()
  void _run_rx_loop();

  ///< rpm 合法范围（说明书：0~5000）
  static bool _valid_rpm(uint16_t rpm);

  ///< 方向枚举合法性
  static bool _valid_direction(emmv5::Direction dir);

  ///< 回零模式合法性（0~3）
  static bool _valid_homing(emmv5::HomingMode mode);

  ///< 控制模式合法性（0~3）
  static bool _valid_control_mode(emmv5::ControlMode mode);

  ///< 发布位置（短临界区）
  void _publish_position(size_t index, int32_t position);

  ///< 发布状态（短临界区）
  void _publish_status(size_t index, uint8_t status);

  ///< 地址 → 下标；不在组内返回 _count
  size_t index_of(uint8_t addr) const;

  ///< 地址是否在组内
  bool _is_member(uint8_t addr) const;

  ///< 直接发送一帧（校验未初始化、地址、帧有效性）
  Status _send_for(uint8_t addr, const emmv5::Frame &frame);

  ///< 追加一帧到 MMCL 缓冲（校验未初始化、地址、帧有效性、空间）
  Status _queue_for(uint8_t addr, const emmv5::Frame &frame);

  ///< 把一块字节分发给组内所有电机
  uint32_t feed_rx(const uint8_t *data, size_t size);

  // ----------------
  // ---------------- 成员变量 ----------------

  Uart  &_uart;              ///< 被驱动的串口（不拥有）
  Motor  _motor[MAX_MOTORS]; ///< 组内电机状态
  size_t _count = 0;         ///< 组内电机数
  bool   _ready = false;     ///< init() 是否成功

  uint8_t _multi[MULTI_PAYLOAD_MAX + emmv5::MULTI_OVERHEAD] = {}; ///< MMCL 总帧缓冲
  size_t  _multi_payload = 0;                                    ///< 已装载的子命令字节数

  TaskHandle_t _task_handle = nullptr; ///< 内部 RX 任务句柄；init() 创建，析构时删除

  // ----------------
};

#endif // __DEVICE_EMMV5_GROUP_HPP__
