/**
 * @file bsp_uart.c
 * @author Rh (qq:750920400)
 * @brief 串口驱动实现。很简陋，无锁。接收是中断逐字节接收。发送是阻塞发送
 * @version 0.1
 * @date 2026-05-25
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "bsp_uart.h"
#include "hardware_config.h"

#pragma section all "cpu0_dsram"

// 串口设备定义
struct BspUart bsp_uart0;
struct BspUart bsp_uart1;
struct BspUart bsp_uart2;
struct BspUart bsp_uart3;

#pragma section all restore

/*==============================================================================
 * 内部函数声明
 *============================================================================*/
static void bsp_uart_rx_interrupt_enable(uart_index_enum uart_n);

uint8 dat = 0;

/**
 * @brief UART接收中断处理函数
 * @param uart 串口结构体指针
 */
void bsp_uart_rx_isr_handler(BspUart *uart)
{
  // 读取所有可用的数据并写入逐飞FIFO
  while (uart_query_byte(uart->uart_index, &dat))
  {
    // 写失败说明FIFO已满，字节被丢弃，累计溢出计数（诊断丢包）
    if (fifo_write_buffer(&uart->rx_fifo, &dat, 1) != FIFO_SUCCESS)
    {
      uart->rx_overflow_count++;
    }
  }

  // FIFO非空即唤醒协议任务：任务会循环消费到空；二值信号量多次give自动合并
  if (uart == &NUC_MCU_UART && fifo_used(&uart->rx_fifo) >= 1)
  {
    // 给出接收完成信号量
    xSemaphoreGiveFromISR(uart->rx_sem, NULL);
  }
}

/**
 * @brief 启用串口接收中断
 * @param uart_n 串口模块号
 */
static void bsp_uart_rx_interrupt_enable(uart_index_enum uart_n)
{
  uart_rx_interrupt(uart_n, 1);
}

/*==============================================================================
 * 公共函数实现
 *============================================================================*/

/**
 * @brief 初始化串口成员变量
 * @param uart 串口结构体指针
 * @param uart_index 串口模块号
 * @param baud 波特率
 * @param tx_pin 发送引脚
 * @param rx_pin 接收引脚
 * @param rx_buf_size FIFO缓冲区大小
 * @param uart_cfg 帧格式配置（UART_CFG_8N1 默认 / UART_CFG_8E2 SBUS协议）
 */
void bsp_uart_init(BspUart *uart, uart_index_enum uart_index, uint32 baud, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin, size_t rx_buf_size, UartConfig_enum uart_cfg)
{
  uart->uart_index = uart_index;
  uart->tx_pin     = tx_pin;
  uart->rx_pin     = rx_pin;
  uart->baudrate   = baud;

  // 初始化逐飞FIFO用于接收数据
  fifo_init(&uart->rx_fifo, FIFO_DATA_8BIT, uart->rx_buffer, (uint32)rx_buf_size);

  // 创建二值信号量
  uart->rx_sem = xSemaphoreCreateBinary();
  configASSERT(uart->rx_sem != NULL);

  // 创建发送互斥锁：防止多个任务同时阻塞发送导致帧字节交错
  uart->tx_mutex = xSemaphoreCreateMutex();
  configASSERT(uart->tx_mutex != NULL);

  // 清零溢出计数
  uart->rx_overflow_count = 0;

  // 初始化逐飞串口驱动（根据帧格式选择不同的初始化函数）
  if (uart_cfg == UART_CFG_9E2)
  {
    // SBUS 模式：帧格式配置在 init 前完成，确保波特率分频器按 12bits/帧 正确计算
    uart_sbus_init(uart_index, baud, tx_pin, rx_pin);
  }
  else
  {
    uart_init(uart_index, baud, tx_pin, rx_pin);
    // 启用接收中断
  }
  bsp_uart_rx_interrupt_enable(uart_index);
}

/**
 * @brief 析构函数，逐飞FIFO无需显式释放资源
 * @param uart 串口结构体指针
 */
void bsp_uart_deinit(BspUart *uart)
{
  (void)uart; // 逐飞FIFO无需释放
}

/**
 * @brief 阻塞发送一个字节
 * @param uart 串口结构体指针
 * @param dat 需要发送的字节
 */
void bsp_uart_send_byte(BspUart *uart, uint8 dat)
{
  // 互斥锁串行化发送：防止与响应帧/上报帧字节交错
  xSemaphoreTake(uart->tx_mutex, portMAX_DELAY);
  uart_write_byte(uart->uart_index, dat);
  xSemaphoreGive(uart->tx_mutex);
}

/**
 * @brief 阻塞发送字符串
 * @param uart 串口结构体指针
 * @param str 需要发送的字符串
 */
void bsp_uart_send_string(BspUart *uart, const char *str)
{
  xSemaphoreTake(uart->tx_mutex, portMAX_DELAY);
  uart_write_string(uart->uart_index, str);
  xSemaphoreGive(uart->tx_mutex);
}

/**
 * @brief 阻塞发送缓冲区数据
 * @param uart 串口结构体指针
 * @param buff 需要发送的数据缓冲区
 * @param len 数据长度
 */
void bsp_uart_send_buffer(BspUart *uart, const uint8 *buff, uint32 len)
{
  xSemaphoreTake(uart->tx_mutex, portMAX_DELAY);
  uart_write_buffer(uart->uart_index, buff, len);
  xSemaphoreGive(uart->tx_mutex);
}

/**
 * @brief 从FIFO读取数据（非阻塞）
 * @param uart 串口结构体指针
 * @param buff 接收数据缓冲区
 * @param len 要读取的数据长度
 * @return 实际读取的字节数
 */
uint32 bsp_uart_recv(BspUart *uart, uint8 *buff, uint32 len)
{
  uint32  read_len = len;
  boolean state    = disableInterrupts(); // 短临界区：防止RX中断与读FIFO并发修改size/指针

  fifo_read_buffer(&uart->rx_fifo, buff, &read_len, FIFO_READ_AND_CLEAN);

  restoreInterrupts(state);
  return read_len;
}

/**
 * @brief 检查FIFO中可读取的数据长度
 * @param uart 串口结构体指针
 * @return FIFO中可读取的字节数
 */
uint32 bsp_uart_available(BspUart *uart)
{
  return fifo_used(&uart->rx_fifo);
}

/**
 * @brief 等待接收信号量
 * @param uart 串口结构体指针
 * @param timeout 超时时间
 * @return pdTRUE收到信号量，pdFALSE超时
 */
BaseType_t bsp_uart_wait(BspUart *uart, TickType_t timeout)
{
  return xSemaphoreTake(uart->rx_sem, timeout);
}

/**
 * @brief 总初始化数据
 */
void bsp_uart_all_init(void)
{
  // 串口初始化 - 使用默认配置示例
  // UART0: 波特率115200, TX=P14_0, RX=P14_1, FIFO缓冲区64字节
  bsp_uart_init(&bsp_uart0, UART_0, 115200, UART0_TX_P14_0, UART0_RX_P14_1, 64, UART_CFG_8N1);

  // UART1: 波特率100000, TX=P15_0, RX=P15_1, FIFO缓冲区128字节
  bsp_uart_init(&bsp_uart1, UART_1, 100000, UART1_TX_P15_0, UART1_RX_P15_1, 256, UART_CFG_9E2);

  // UART2: 波特率115200, TX=P02_0, RX=P02_1, FIFO缓冲区64字节
  bsp_uart_init(&bsp_uart2, UART_2, 115200, UART2_TX_P14_2, UART2_RX_P14_3, 64, UART_CFG_8N1);

  // UART3: 波特率921600, TX=P00_0, RX=P00_1, FIFO缓冲区256字节（16帧缓冲）
  bsp_uart_init(&bsp_uart3, UART_3, 921600, UART3_TX_P00_0, UART3_RX_P00_1, 256, UART_CFG_8N1);
}
