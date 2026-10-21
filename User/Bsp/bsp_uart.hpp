/**
 * @file bsp_uart.hpp
 * @author Rh
 * @brief 串口驱动（DMA 与中断两种收发方式）
 * @version 0.5
 * @date 2026-10-20
 *
 * @todo 收上来的数据要交给应用层分发（app_message 或上层总线注册等等）
 *
 * @copyright Copyright (c) 2026
 *
 * @details 本文件只提供一个通用模板 BspUartT<Transport, RX_SIZE, TX_SIZE>：
 *          流缓冲区、发送状态机、诊断计数与恢复流程共用，底层 DMA / 中断由 Transport 选择。
 *          具体实例放在 bsp_cfg.hpp / bsp_cfg.cpp。
 *          收发时序、诊断表与新增串口步骤见同目录 README.md。
 *
 * @note 用法：DMA 和 IT 模式共用，外部调用没有区别。接收是字节流（不保证整帧），
 *       发送返回 OK 只表示整帧已进流缓冲区；所有超时参数单位都是毫秒，portMAX_DELAY 表示一直等。
 *
 * @note 多任务同时写同一个串口是安全的：send() 用 _tx_lock 串行化「判忙 → 取包 → 启动发送」，
 *       printf() 另有 _printf_lock 保护格式化缓冲。
 */

#ifndef __BSP_UART_HPP__
#define __BSP_UART_HPP__

#include "FreeRTOS.h" // IWYU pragma: keep
#include "semphr.h"   // IWYU pragma: keep (TX 启动锁)
#include "stream_buffer.h"
#include "task.h"  // IWYU pragma: keep
#include "usart.h" // IWYU pragma: keep

#include "status.hpp" // 统一状态码


/** @brief 串口底层传输策略：只封装 DMA / 中断的 HAL 调用差异，不持有状态 */
namespace uart_detail
{
struct DmaTransport
{
  static bool arm_rx(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size);
  static bool start_tx(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size);

  static bool rx_ok(const UART_HandleTypeDef *huart);
  static bool rx_stopped(const UART_HandleTypeDef *huart);

  static size_t tx_sent(const UART_HandleTypeDef *huart, size_t inflight);
  static void abort_rx(UART_HandleTypeDef *huart);
  static void abort_tx(UART_HandleTypeDef *huart);
};

struct ItTransport
{
  static bool arm_rx(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size);
  static bool start_tx(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size);

  static bool rx_ok(const UART_HandleTypeDef *huart);
  static bool rx_stopped(const UART_HandleTypeDef *huart);

  static size_t tx_sent(const UART_HandleTypeDef *huart, size_t inflight);
  static void abort_rx(UART_HandleTypeDef *huart);
  static void abort_tx(UART_HandleTypeDef *huart);
};
}


/** @brief 串口收发模式 */
enum class UartMode : uint8_t
{
  RxOnly = 0, ///< 只接收
  TxOnly,     ///< 只发送
  TxRx,       ///< 收发都启用
};


/**
 * @brief 串口驱动（IDLE 中断 + DMA/中断收发 + FreeRTOS 流缓冲区）
 *
 * @tparam Transport DMA 或中断传输策略
 * @tparam RX_SIZE 接收流缓冲区与接收暂存区大小（字节）
 * @tparam TX_SIZE 发送流缓冲区、发送暂存区与 printf 缓冲大小（字节）
 *
 * @note 接收：IDLE/TC 中断把收到的数据投进流缓冲区，然后立即重新启动接收，准备接下一帧。
 *       半传输（HT）事件也会进回调，但它不是帧边界，直接忽略。
 *       发送：数据先写进发送流缓冲区，再由任务或 TX Complete 中断交给 Transport。
 */
template <typename Transport, size_t RX_SIZE, size_t TX_SIZE>
class BspUartT
{
  static_assert(RX_SIZE > 0U && RX_SIZE <= 65535U, "RX_SIZE 必须在 1~65535 之间");
  static_assert(TX_SIZE > 0U && TX_SIZE <= 65535U, "TX_SIZE 必须在 1~65535 之间");

public:
  // ---------------- 公有接口 ----------------

  /** @brief 串口配置（可匿名按序传入：{huart, mode, baudrate}） */
  struct Config
  {
    /** @brief 按序构造（参数顺序 = 字段顺序） */
    Config(UART_HandleTypeDef *huart = nullptr, UartMode mode = UartMode::TxRx, uint32_t baudrate = 115200U) : huart(huart), mode(mode), baudrate(baudrate)
    {
    }

    UART_HandleTypeDef *huart;  ///< UART 句柄
    UartMode            mode;   ///< 收发模式
    uint32_t            baudrate; ///< 波特率 (bit/s)，TX 卡死判定要用它算一包发完要多久
  };

  /**
   * @brief RX 的健康状态
   *
   * @note 只信 HAL 的 RxState 是不够的：底层通道可能已经停了，而 HAL 状态还没更新。
   *       具体检查项由 Transport 决定：DMA 看 CR3.DMAR 与 DMA EN；
   *       中断看 IDLEIE 加上 RXNEIE（FIFO 模式则是 CR3.RXFTIE）。
   */
  enum class RxHealth : uint8_t
  {
    OK = 0,       ///< 在收
    STOPPED,      ///< 已停（没启动过或主动停的，不算故障）
    INCONSISTENT, ///< 状态不一致，需要重建
  };

  /** @brief RX 诊断计数：只增不减，一切正常时应全为 0 */
  struct RxDiag
  {
    uint32_t arm_fail_cnt; ///< 启动接收失败的次数（包括 init 时的第一次和运行期重启）
    uint32_t drop_bytes;   ///< 接收流缓冲区满，被丢掉的字节数
    uint32_t hw_error_cnt; ///< HAL 错误回调里的 RX 硬件错误次数（ORE/FE/NE/PE/RTO）
    uint32_t error_cnt;    ///< rx_recover() 真正重建 RX 的次数
  };

  /** @brief TX 诊断计数：只增不减，一切正常时应全为 0 */
  struct TxDiag
  {
    uint32_t drop_bytes;        ///< 入队失败（缓冲区满或超时）丢掉的字节数
    uint32_t start_fail_cnt;    ///< 启动发送失败的次数
    uint32_t isr_skip_cnt;      ///< 中断里发现通道不空闲，跳过续传的次数
    uint32_t stall_recover_cnt; ///< 巡检发现卡死并恢复成功的次数
    uint32_t error_cnt;         ///< TX 触发错误回调的次数（最终由 tx_recover() 恢复）
  };

  /** @brief 构造函数（只做赋值，FreeRTOS 资源创建推迟到 init()） */
  BspUartT(const Config &cfg);

  /**
   * @brief 按 UartMode 创建流缓冲区/锁，并启动接收
   *
   * @note 必须在调度器启动之后调用，因为里面要创建 FreeRTOS 对象。
   *       可以重复调用：开头会先把上一次的资源释放掉。
   *
   * @return OK=成功；IO_ERROR=资源创建失败或接收没能启动
   */
  Status init();

  /** @brief 析构：停掉 RX/TX 通道并释放所有软件资源 */
  ~BspUartT();

  /**
   * @brief 把数据写进发送流缓冲区，并启动发送
   *
   * @param data       要发的数据
   * @param size       字节数；超过发送流缓冲区容量会直接返回 BAD_ARG，需要调用方自己分包
   * @param timeout_ms 入队最多等多久（毫秒）；portMAX_DELAY 表示一直等到全部写进去
   * @param written    返回实际写进去的字节数，不需要可以传 nullptr
   * @return OK=全部入队；TIMEOUT=只进去一部分；BAD_ARG=参数非法；IO_ERROR=没启用发送
   *
   * @note 返回 OK 只说明数据进了缓冲区，此时底层可能才刚开始发。
   */
  Status send(const uint8_t *data, size_t size, uint32_t timeout_ms = portMAX_DELAY, size_t *written = nullptr);

  /**
   * @brief 格式化输出，用法和 printf 一样（只能在任务里调用，超长会截断）
   *
   * @note 格式化用的缓冲区是这个实例共用的，所以加了 _printf_lock，
   *       多个任务同时调用也不会互相覆盖。
   * @note 入队等待固定 10 ms，不像 send() 那样给 portMAX_DELAY：万一发送卡住，
   *       不至于把调用任务一直阻塞住。没全部写进去就返回 TIMEOUT，已经进去的部分照常发。
   */
  Status printf(const char *fmt, ...);

  /**
   * @brief 从接收流缓冲区读数据
   *
   * @note 读到的不保证是完整的一帧 —— 驱动只负责把字节搬过来，怎么切帧是协议层的事
   *       （长度前缀 / 定界符 / 校验和都可以）。除非流缓冲区满，否则不会丢字节。
   *
   * @param buffer   收数据的地方
   * @param size     最多读多少字节
   * @param timeout  最多等多久（毫秒）；portMAX_DELAY 表示一直等
   * @param received 返回实际读到的字节数，不需要可以传 nullptr
   * @return OK=读到了；TIMEOUT=没有数据；BAD_ARG=参数非法；IO_ERROR=接收缓冲区没创建
   */
  Status receive(uint8_t *buffer, size_t size, uint32_t timeout = portMAX_DELAY, size_t *received = nullptr);

  /**
   * @brief TX 卡死后的恢复：还有在途或待发数据，却长时间没有推进，就复位通道重发
   *
   * @note 多久算"长时间"：取一包发完所需时间的 2 倍，按 Config::baudrate 算出来。
   *       正常发送时每发一包都会刷新 _tx_last_activity_tick，所以不会把正常发送误判成卡死。
   * @note 重发前先用 Transport 提供的剩余计数算出在途数据已经发出去多少，只补后面的部分，
   *       既不重复也不丢；在途数据如果已经发完，就直接从流缓冲区取新数据。
   * @note 取锁不等待，可以放进周期任务里调用（比如 sys_task 的 10 ms 巡检）。
   *
   * @return true=这次确实恢复成功了一次
   */
  bool tx_recover();

  /**
   * @brief RX 异常时的恢复：复位接收通道，再重新启动
   *
   * @note 触发条件：本该在收（_rx_should_run 为 true），但 Transport 报告的状态不是 OK。
   *       一切正常时本函数只读一次状态就返回，不会打断正在收的数据。
   * @note 必须放在任务里：复位接收要调阻塞版的 HAL_UART_AbortReceive()，中断里不能用。
   *       所以中断里出错只累加计数，真正的修复由本函数在 10 ms 内完成。
   *
   * @return true=这次确实重建了一次（累计次数见 get_rx_diag().error_cnt）
   */
  bool rx_recover();
  // ----------------
  // ---------------- ISR 入口 ----------------

  /**
   * @brief 收到一帧数据（IDLE/TC 中断触发）
   *
   * @param size 这一帧的字节数
   * @param pxHigherPriorityTaskWoken 中断里需要唤醒的高优先级任务
   */
  void on_idle_isr(uint16_t size, BaseType_t *pxHigherPriorityTaskWoken);

  /**
   * @brief 上一批发完了，接着发缓冲区里的下一批（TX Complete 中断触发）
   *
   * @param pxHigherPriorityTaskWoken 需初始化为 pdFALSE，唤醒了高优先级任务则置 pdTRUE
   */
  void start_transmission_from_isr(BaseType_t *pxHigherPriorityTaskWoken);

  /**
   * @brief UART 错误回调（ISR）
   *
   * @note 这里只累加计数，不做恢复：HAL 的 Abort 接口内部可能阻塞，
   *       中断里不能用。RX 交给任务侧的 rx_recover()，TX 交给 tx_recover()。
   */
  void on_error_isr();
  // ----------------
  // ---------------- 查询接口 ----------------

  /** @brief 发送流缓冲区还剩多少空间（字节） */
  size_t get_tx_free_space();

  /** @brief 接收流缓冲区现在有多少可读字节 */
  size_t get_rx_available_data();

  /**
   * @brief 校验 CubeMX 写入的串口帧格式
   *
   * @note 只读 huart->Init，不修改任何配置；协议层用它确认串口参数与设备一致。
   */
  bool check_format(uint32_t baudrate, uint32_t word_length, uint32_t parity, uint32_t stop_bits) const;

  /**
   * @brief 查 RX 是不是真的在收（逐项核对 Transport 的健康判据）
   *
   * @note 本该在收的时候返回非 OK，说明出了故障；本来就没在收（主动停的）不算。
   */
  RxHealth get_rx_health() const;

  /** @brief 读 RX 诊断计数 */
  RxDiag get_rx_diag() const;

  /** @brief 读 TX 诊断计数 */
  TxDiag get_tx_diag() const;
  // ----------------
private:
  // ---------------- 成员变量 ----------------

  UART_HandleTypeDef  *_huart;                      ///< UART 句柄
  StreamBufferHandle_t _rx_stream_buffer = nullptr; ///< 接收流缓冲区
  StreamBufferHandle_t _tx_stream_buffer = nullptr; ///< 发送流缓冲区
  SemaphoreHandle_t    _tx_lock          = nullptr; ///< 发送启动锁：保证「判忙 → 取包 → 启动发送」互斥
  SemaphoreHandle_t    _printf_lock      = nullptr; ///< printf 锁：保护实例共用的格式化缓冲区

  bool     _rx_should_run = true;       ///< 是否希望 RX 在收；某次失败不会把它永久置成 false
  UartMode _mode;                       ///< 收发模式
  uint32_t _baudrate;                   ///< 波特率 (bit/s)，TX 卡死判定要用它算阈值
  uint8_t  _rx_buffer[RX_SIZE];         ///< 接收暂存区
  uint8_t  _tx_buffer[TX_SIZE];         ///< 发送暂存区
  char     _printf_buffer[TX_SIZE];     ///< printf 格式化用的缓冲区

  // RX 诊断量：中断里累加，任务里读取
  volatile uint32_t _rx_arm_fail_cnt = 0; ///< 启动接收失败的次数
  volatile uint32_t _rx_drop_bytes   = 0; ///< 流缓冲区满而丢掉的字节数
  volatile uint32_t _rx_hw_error_cnt = 0; ///< HAL 错误回调里的 RX 硬件错误次数
  volatile uint32_t _rx_error_cnt    = 0; ///< 重建 RX 的次数

  // TX 诊断量：中断里累加，任务里读取
  volatile uint32_t _tx_drop_bytes     = 0; ///< 入队失败而丢掉的字节数
  volatile uint32_t _tx_start_fail_cnt = 0; ///< 启动发送失败的次数（任务和中断合计）
  volatile uint32_t _tx_isr_skip_cnt   = 0; ///< ISR 因通道不空闲跳过续传的次数
  volatile uint32_t _tx_stall_rec_cnt  = 0; ///< 巡检发现卡死并恢复成功的次数
  volatile uint32_t _tx_error_cnt      = 0; ///< TX 进错误回调的次数

  volatile TickType_t _tx_last_activity_tick = 0; ///< 最近一次成功启动发送的时刻，tx_recover() 用它判断卡死

  volatile size_t _tx_inflight_len = 0; ///< 当前交给 Transport 在传的字节数，卡死后据此续发

  // ----------------
  // ---------------- 内部实现 ----------------

  /** @brief 当前配置是否启用发送 */
  bool _tx_enabled() const
  {
    return _mode != UartMode::RxOnly;
  }

  /** @brief 当前配置是否启用接收 */
  bool _rx_enabled() const
  {
    return _mode != UartMode::TxOnly;
  }

  /**
   * @brief 启动接收（只管 RX；调用前 RxState 必须是 READY）
   *
   * @note 启动前先清掉残留的 UART 错误标志（ORE/FE/NE/PE），
   *       否则 HAL 会因为存在挂起的错误而拒绝启动。
   *
   * @return true=成功；false=HAL 拒绝（BUSY/ERROR），已计入 arm_fail_cnt
   */
  bool arm_reception();

  /** @brief 中止 RX 通道并复位接收状态（只动 RX，不碰 TX） */
  void abort_reception();

  /** @brief 释放已创建的 FreeRTOS 资源（两个流缓冲区、两把锁） */
  void cleanup_resources();

  /** @brief 从发送流缓冲区取一包交给 Transport；wait=0 表示取不到锁就立刻返回 */
  bool start_transmission(TickType_t wait = portMAX_DELAY);
  // ----------------
};


#endif // __BSP_UART_HPP__
