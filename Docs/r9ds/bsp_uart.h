#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "FreeRTOS.h"
#include "semphr.h"
#include "zf_common_fifo.h"
#include "zf_common_headfile.h"
#include "zf_driver_uart.h"

// 前向声明
struct BspUart;

// 串口设备定义
extern struct BspUart bsp_uart0;
extern struct BspUart bsp_uart1;
extern struct BspUart bsp_uart2;
extern struct BspUart bsp_uart3;

/*==============================================================================
 * 串口帧格式配置枚举
 * @brief 用于指定串口的数据位/校验位/停止位组合
 *============================================================================*/
typedef enum
{
  UART_CFG_8N1 = 0, // 8数据位 无校验 1停止位（默认）
  UART_CFG_9E2 = 1, // 8数据位 偶校验  2停止位（SBUS协议）
} UartConfig_enum;

/*==============================================================================
 * 串口结构体
 * @brief 负责串口通信，使用逐飞FIFO阻塞发送和接收
 * @details 发送时使用逐飞库的FIFO实现阻塞发送，接收时通过中断将数据
 *          存入逐飞FIFO缓冲区，任务可以从FIFO读取数据
 *============================================================================*/
typedef struct BspUart
{
  uart_index_enum   uart_index;        // 串口模块号
  uart_tx_pin_enum  tx_pin;            // 发送引脚
  uart_rx_pin_enum  rx_pin;            // 接收引脚
  uint32            baudrate;          // 波特率
  fifo_struct       rx_fifo;           // 逐飞FIFO结构体，用于接收数据
  uint8             rx_buffer[256];    // FIFO挂载的缓冲区（各串口初始化时传入的buf_size不得超过256）
  SemaphoreHandle_t rx_sem;            // 接收完成信号量
  SemaphoreHandle_t tx_mutex;          // 发送互斥锁：防止多任务并发发送导致帧字节交错
  volatile uint32   rx_overflow_count; // RX FIFO写满丢弃字节计数（诊断丢包用）
} BspUart;


/**
 * @brief 串口中断回调
 * @param uart 串口结构体指针
 *
 */
void bsp_uart_rx_isr_handler(BspUart *uart);

/**
 * @brief 初始化串口设备
 * @param uart 串口结构体指针
 * @param uart_index 串口模块号
 * @param baud 波特率
 * @param tx_pin 发送引脚
 * @param rx_pin 接收引脚
 * @param rx_buf_size FIFO缓冲区大小
 * @param uart_cfg 帧格式配置（UART_CFG_8N1 默认 / UART_CFG_8E2 SBUS协议）
 */
void bsp_uart_init(BspUart *uart, uart_index_enum uart_index, uint32 baud, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin, size_t rx_buf_size, UartConfig_enum uart_cfg);

/**
 * @brief 析构函数，释放FIFO资源（逐飞FIFO无需显式释放）
 * @param uart 串口结构体指针
 */
void bsp_uart_deinit(BspUart *uart);

/**
 * @brief 阻塞发送一个字节
 * @param uart 串口结构体指针
 * @param dat 需要发送的字节
 */
void bsp_uart_send_byte(BspUart *uart, uint8 dat);

/**
 * @brief 阻塞发送字符串
 * @param uart 串口结构体指针
 * @param str 需要发送的字符串
 */
void bsp_uart_send_string(BspUart *uart, const char *str);

/**
 * @brief 阻塞发送缓冲区数据
 * @param uart 串口结构体指针
 * @param buff 需要发送的数据缓冲区
 * @param len 数据长度
 */
void bsp_uart_send_buffer(BspUart *uart, const uint8 *buff, uint32 len);

/**
 * @brief 从FIFO读取数据（非阻塞）
 * @param uart 串口结构体指针
 * @param buff 接收数据缓冲区
 * @param len 要读取的数据长度
 * @return 实际读取的字节数
 */
uint32 bsp_uart_recv(BspUart *uart, uint8 *buff, uint32 len);

/**
 * @brief 检查FIFO中可读取的数据长度
 * @param uart 串口结构体指针
 * @return FIFO中可读取的字节数
 */
uint32 bsp_uart_available(BspUart *uart);

/**
 * @brief 等待接收信号量
 * @param uart 串口结构体指针
 * @param timeout 超时时间
 * @return pdTRUE收到信号量，pdFALSE超时
 */
BaseType_t bsp_uart_wait(BspUart *uart, TickType_t timeout);

/**
 * @brief 总初始化数据
 */
void bsp_uart_all_init(void);

#endif // __BSP_UART_H__
