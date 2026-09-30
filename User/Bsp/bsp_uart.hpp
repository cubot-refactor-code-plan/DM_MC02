/**
 * @file bsp_uart.hpp
 * @author Rh
 * @brief 串口驱动（FreeRTOS 流缓冲区 + IDLE 中断 + DMA）
 * @version 0.3
 * @date 2026-09-10
 *
 * @todo 收上来的数据要交给应用层分发（app_message 或上层总线注册）
 *
 * @copyright Copyright (c) 2026
 *
 * @details 接收端用 IDLE 中断判断一帧收完，发送端由 TX Complete 中断接着发下一批，
 *          收发都走 DMA，不需要 CPU 逐字节搬运。
 *
 * @note 模板参数是缓冲区大小（字节）。DMA 收发缓冲区与流缓冲区都用这个尺寸，
 *       所以单次 send() 也不能超过它。
 *
 * @note 用法：
 *
 *   // 1) 模板实例化在 bsp_uart.cpp
 *   template class BspUart<128>;
 *
 *   // 2) 全局实例在 bsp_cfg.cpp：句柄 + 是否启用发送 + 波特率
 *   __attribute__((section(".dma_buffer"))) BspUart<128> bsp_uart1({&huart1, true, 115200});
 *   // 3) init() 在 bsp_init() 里统一调用（必须在调度器启动之后）
 *   bsp_uart1.init();
 *
 *   // 4) 收发
 *   bsp_uart1.send(buffer, 8);           // 写进发送流缓冲区，DMA 自动发出去
 *   bsp_uart1.printf("val=%d\r\n", 42);  // 格式化输出（非阻塞，DMA 发送）
 *   bsp_uart1.receive(buffer, 8);        // 从接收流缓冲区读，读走的字节会从缓冲区移除
 *
 * @note 多个任务同时写同一个串口是安全的：send() 用 _tx_lock 把「判忙 → 取包 → 启动 DMA」
 *       串起来，printf() 另有 _printf_lock 保护格式化缓冲，两者不会互相覆盖。
 *
 * @note 所有超时参数单位都是毫秒；传 portMAX_DELAY 表示一直等。
 */

#ifndef __BSP_UART_HPP__
#define __BSP_UART_HPP__

#include "FreeRTOS.h" // IWYU pragma: keep
#include "semphr.h"   // IWYU pragma: keep (TX 启动锁)
#include "stream_buffer.h"
#include "task.h"  // IWYU pragma: keep
#include "usart.h" // IWYU pragma: keep

#include "status.hpp" // 统一状态码


/**
 * @brief 串口驱动（IDLE 中断 + DMA + FreeRTOS 流缓冲区）
 *
 * @tparam BUFFER_SIZE 收发缓冲区大小（字节），也是流缓冲区容量
 *
 * @note 接收：IDLE/TC 中断把收到的数据投进流缓冲区，然后立即重新启动接收，准备接下一帧。
 *       半传输（HT）事件也会进回调，但它不是帧边界，直接忽略。
 *       发送：数据先写进发送流缓冲区，再由任务或 TX Complete 中断交给 DMA。
 */
template <size_t BUFFER_SIZE = 256>
class BspUart
{
public:
  // ---------------- 公有接口 ----------------

  /** @brief 串口配置（可匿名按序传入：{huart, transmit_enable, baudrate}） */
  struct Config
  {
    /** @brief 按序构造（参数顺序 = 字段顺序） */
    Config(UART_HandleTypeDef *huart = nullptr, bool transmit_enable = true, uint32_t baudrate = 115200U)

      : huart(huart),
        transmit_enable(transmit_enable),
        baudrate(baudrate)
    {
    }

    UART_HandleTypeDef *huart;           ///< UART 句柄
    bool                transmit_enable; ///< 是否启用发送
    uint32_t            baudrate;        ///< 波特率 (bit/s)，TX 卡死判定要用它算一包发完要多久
  };

  /**
   * @brief RX 的健康状态
   *
   * @note 只信 HAL 的 RxState 是不够的：它可能还停在 BUSY_RX，而 DMA 早就停了。
   *       所以这里同时看三处 —— HAL 的 RxState、UART 的 CR3.DMAR、DMA 的 CR.EN。
   */
  enum class RxHealth : uint8_t
  {
    OK = 0,       ///< 三处一致：在收
    STOPPED,      ///< 三处一致：停了（没启动过或主动停的，不算故障）
    INCONSISTENT, ///< 三处不一致：RX 有问题，需要重启
  };

  /** @brief RX 诊断计数：只增不减，一切正常时应全为 0 */
  struct RxDiag
  {
    uint32_t arm_fail_cnt; ///< 启动接收失败的次数（包括 init 时的第一次和运行期重启）
    uint32_t drop_bytes;   ///< 接收流缓冲区满，被丢掉的字节数
    uint32_t error_cnt;    ///< rx_recover() 真正重建 RX 的次数
  };

  /** @brief TX 诊断计数：只增不减，一切正常时应全为 0 */
  struct TxDiag
  {
    uint32_t drop_bytes;        ///< 入队失败（缓冲区满或超时）丢掉的字节数
    uint32_t start_fail_cnt;    ///< HAL_UART_Transmit_DMA 启动失败的次数
    uint32_t isr_skip_cnt;      ///< 中断里发现通道不空闲，跳过续传的次数
    uint32_t stall_recover_cnt; ///< 巡检发现卡死并恢复成功的次数
    uint32_t error_cnt;         ///< TX 触发错误回调的次数（最终由 tx_recover() 恢复）
  };

  /** @brief 构造函数（只做赋值，FreeRTOS 资源创建推迟到 init()） */
  BspUart(const Config &cfg);

  /**
   * @brief 创建流缓冲区和两把锁，然后启动 IDLE 接收
   *
   * @note 必须在调度器启动之后调用，因为里面要创建 FreeRTOS 对象。
   *       可以重复调用：开头会先把上一次的资源释放掉。
   *
   * @return OK=成功；IO_ERROR=资源创建失败或接收没能启动
   */
  Status init();

  /** @brief 析构：停掉 RX/TX DMA 并释放所有软件资源 */
  ~BspUart();

  /**
   * @brief 把数据写进发送流缓冲区，并启动 DMA 发出去
   *
   * @param data       要发的数据
   * @param size       字节数；超过流缓冲区容量会直接返回 BAD_ARG，需要调用方自己分包
   * @param timeout_ms 入队最多等多久（毫秒）；portMAX_DELAY 表示一直等到全部写进去
   * @param written    返回实际写进去的字节数，不需要可以传 nullptr
   * @return OK=全部入队；TIMEOUT=只进去一部分；BAD_ARG=参数非法；IO_ERROR=没启用发送
   *
   * @note 返回 OK 只说明数据进了缓冲区，此时 DMA 可能才刚开始发。
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
   * @brief TX 卡死后的恢复：缓冲区里还有数据，却长时间没有推进，就复位通道重发
   *
   * @note 多久算"长时间"：取一包发完所需时间的 2 倍，按 Config::baudrate 算出来。
   *       正常发送时每发一包都会刷新 _tx_last_activity_tick，所以不会把正常发送误判成卡死。
   * @note 重发前先用 DMA 剩余计数算出在途数据已经发出去多少，只补后面的部分，
   *       既不重复也不丢；在途数据如果已经发完，就直接从流缓冲区取新数据。
   * @note 取锁不等待，可以放进周期任务里调用（比如 sys_task 的 10 ms 巡检）。
   *
   * @return true=这次确实恢复成功了一次
   */
  bool tx_recover();

  /**
   * @brief RX 异常时的恢复：复位接收通道，再重新启动
   *
   * @note 触发条件：本该在收（_rx_should_run 为 true），但三处状态不一致。
   *       一切正常时本函数只读一次状态就返回，不会打断正在收的数据。
   * @note 必须放在任务里：复位接收要调阻塞版的 HAL_UART_AbortReceive()，中断里不能用。
   *       所以中断里出错只累加计数，真正的修复由本函数在 10 ms 内完成。
   *
   * @return true=这次确实重建了一次（累计次数见 rx_diag().error_cnt）
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
   * @note 这里只累加计数，不做恢复：HAL 的 Abort 接口内部是阻塞版 HAL_DMA_Abort，
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
   * @brief 查 RX 是不是真的在收（核对 RxHealth 里说的那三处状态）
   *
   * @note 本该在收的时候返回非 OK，说明出了故障；本来就没在收（主动停的）不算。
   */
  RxHealth rx_health() const;

  /** @brief 读 RX 诊断计数 */
  RxDiag rx_diag() const;

  /** @brief 读 TX 诊断计数 */
  TxDiag tx_diag() const;

  // ----------------

private:
  // ---------------- 私有实现 ----------------

  // 成员变量

  UART_HandleTypeDef  *_huart;                      ///< UART 句柄
  StreamBufferHandle_t _rx_stream_buffer = nullptr; ///< 接收流缓冲区
  StreamBufferHandle_t _tx_stream_buffer = nullptr; ///< 发送流缓冲区
  SemaphoreHandle_t    _tx_lock          = nullptr; ///< 发送启动锁：保证「判忙 → 取包 → 启动 DMA」互斥
  SemaphoreHandle_t    _printf_lock      = nullptr; ///< printf 锁：保护实例共用的格式化缓冲区

  bool     _rx_should_run = true;       ///< 是否希望 RX 在收；某次失败不会把它永久置成 false
  bool     _transmit_enable;            ///< 是否启用发送
  uint32_t _baudrate;                   ///< 波特率 (bit/s)，TX 卡死判定要用它算阈值
  uint8_t  _rx_dma_buffer[BUFFER_SIZE]; ///< DMA 接收缓冲区
  uint8_t  _tx_dma_buffer[BUFFER_SIZE]; ///< DMA 发送缓冲区
  char     _printf_buffer[BUFFER_SIZE]; ///< printf 格式化用的缓冲区

  // RX 诊断量：中断里累加，任务里读取
  volatile uint32_t _rx_arm_fail_cnt = 0; ///< 启动接收失败的次数
  volatile uint32_t _rx_drop_bytes   = 0; ///< 流缓冲区满而丢掉的字节数
  volatile uint32_t _rx_error_cnt    = 0; ///< 重建 RX 的次数

  // TX 诊断量：中断里累加，任务里读取
  volatile uint32_t _tx_drop_bytes     = 0; ///< 入队失败而丢掉的字节数
  volatile uint32_t _tx_start_fail_cnt = 0; ///< 启动 DMA 失败的次数（任务和中断合计）
  volatile uint32_t _tx_isr_skip_cnt   = 0; ///< ISR 因通道不空闲跳过续传的次数
  volatile uint32_t _tx_stall_rec_cnt  = 0; ///< 巡检发现卡死并恢复成功的次数
  volatile uint32_t _tx_error_cnt      = 0; ///< TX 进错误回调的次数

  volatile TickType_t _tx_last_activity_tick = 0; ///< 最近一次成功启动发送的时刻，tx_recover() 用它判断卡死

  volatile size_t _tx_inflight_len = 0; ///< 当前交给 DMA 在传的字节数，卡死后据此续发

  // 内部实现

  /**
   * @brief 启动 DMA 接收（只管 RX；调用前 RxState 必须是 READY）
   *
   * @note 启动前先清掉残留的 UART 错误标志（ORE/FE/NE/PE），
   *       否则 HAL 会因为存在挂起的错误而拒绝启动。
   *
   * @return true=成功；false=HAL 拒绝（BUSY/ERROR），已计入 arm_fail_cnt
   */
  bool arm_reception();

  /** @brief 中止 RX 通道并复位接收状态（只动 RX，不碰 TX DMA） */
  void abort_reception();

  /** @brief 释放已创建的 FreeRTOS 资源（两个流缓冲区、两把锁） */
  void cleanup_resources();

  /** @brief 从发送流缓冲区取一包交给 DMA；wait=0 表示取不到锁就立刻返回 */
  bool start_transmission(TickType_t wait = portMAX_DELAY);

  // ----------------
};


#endif // __BSP_UART_HPP__
