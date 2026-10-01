/**
 * @file protocol_uart.hpp
 * @author Rh
 * @brief 串口协议解析头文件，半成品
 * @version 0.1
 * @date 2026-02-17
 *
 * @todo 优化协议处理方式
 *
 * @copyright Copyright (c) 2026
 *
 * @details 基于bsp_usart的串口协议解析，支持帧头帧尾自定义，但是没写很好的处理
 *
 * @note 使用示例：
 *
 *       - 先实例化bsp_uart（在bsp文件中）
 *       - 全局类对象实例化：ProtocolUart protocol_uart_1({bsp_uart1, 1});
 *       - 初始化协议：protocol_uart_1.init();
 */

#ifndef __PROTOCOL_UART_HPP__
#define __PROTOCOL_UART_HPP__


#include "FreeRTOS.h" // IWYU pragma: keep
#include "stddef.h"
#include "stdint.h"
#include "task.h" // IWYU pragma: keep

#include "status.hpp" // 统一状态码


/* USER CODE BEGIN */

// ---------------- 外部声明 ----------------

template <size_t BUFFER_SIZE>
class BspUart;

class ProtocolUart;
/**
 * @brief 全局串口协议实例
 */
extern ProtocolUart protocol_uart_1;

/* USER CODE END */


/**
 * @brief 协议帧结构体定义
 *
 * @note 对应上位机：AA 55 CMD LEN DATA... SUM 0D（帧头帧尾可自定义）
 */
#pragma pack(1)
typedef struct
{
  uint8_t header1;  ///< 帧头1（默认0xAA）
  uint8_t header2;  ///< 帧头2（默认0x55）
  uint8_t cmd;      ///< 指令码
  uint8_t len;      ///< 有效负载长度
  uint8_t data[64]; ///< 数据负载（最大64字节）
  uint8_t checksum; ///< 校验和
  uint8_t tail;     ///< 帧尾（默认0x0C）
} ProtocolFrame;
#pragma pack()


/**
 * @brief 串口协议类
 */
class ProtocolUart
{
private:
  // ----------------
  // ---------------- 私有成员变量 ----------------

  ProtocolFrame _rx_frame;      ///< 接收用结构体
  BspUart<128> &_uart_instance; ///< 使用的串口驱动实例
  uint8_t       _header1;       ///< 自定义帧头1
  uint8_t       _header2;       ///< 自定义帧头2
  uint8_t       _tail;          ///< 自定义帧尾
  uint8_t       _instance_name; ///< 实例名称编号
  char          _task_name[32]; ///< 任务名称
  uint32_t      _stack_size;    ///< 堆栈大小
  uint32_t      _priority;      ///< 任务优先级

  // ----------------
  // ---------------- 私有成员函数 ----------------

  /**
   * @brief 计算校验和
   * @param data 数据缓冲区
   * @param len 长度
   * @return uint8_t 校验结果
   */
  uint8_t _calculate_checksum(uint8_t *data, uint8_t len);

  /**
   * @brief 逻辑分发：根据指令执行具体动作
   */
  void _protocol_handle_cmd();


public:
  // ----------------
  // ---------------- 构造函数与析构函数 ----------------

  /**
   * @brief 协议配置结构体（可匿名按序传入）
   */
  struct Config
  {
    /**
     * @brief 按序构造配置（参数顺序 = 字段顺序）
     */
    Config(BspUart<128> &uart, uint8_t name, uint8_t h1 = 0xAA, uint8_t h2 = 0x55, uint8_t t = 0x0C) : uart(uart),
                                                                                                       name(name),
                                                                                                       h1(h1),
                                                                                                       h2(h2),
                                                                                                       t(t)
    {
    }

    BspUart<128> &uart; ///< 串口实例引用
    uint8_t       name; ///< 实例名称编号
    uint8_t       h1;   ///< 帧头1
    uint8_t       h2;   ///< 帧头2
    uint8_t       t;    ///< 帧尾
  };

  /**
   * @brief 构造函数
   * @param cfg 协议配置（串口实例/名称/帧头帧尾，可匿名按序传入）
   */
  ProtocolUart(const Config &cfg);


  // ----------------
  // ---------------- 公共接口 ----------------

  /**
   * @brief 协议处理初始化（创建协议解析任务）
   * @return Status OK=任务创建成功，IO_ERROR=任务创建失败
   */
  Status init();

  /**
   * @brief 协议解析任务主体：按帧头帧尾同步、校验和验证，通过后分发指令
   *
   * @note 内部是死循环（阻塞在串口读上），且不返回；只由 init() 创建出来的
   *       协议任务调用（C 入口 uart_protocol_task_entry 做一次类型转换后转发）。
   *       类外代码请用 send() / get_rx_*() 交互，无需也不可能直接碰私有成员。
   */
  void task();

  /**
   * @brief 发送协议数据包给上位机
   * @param cmd 指令码
   * @param data 数据指针
   * @param len 数据长度
   * @return Status OK=发送成功，BAD_ARG=参数非法，其余同 BspUart::send()
   */
  Status send(uint8_t cmd, uint8_t *data, uint8_t len);

  /**
   * @brief 获取接收到的帧命令码
   * @return uint8_t 命令码
   */
  uint8_t get_rx_cmd()
  {
    return _rx_frame.cmd;
  }

  /**
   * @brief 获取接收到的帧数据长度
   * @return uint8_t 数据长度
   */
  uint8_t get_rx_len()
  {
    return _rx_frame.len;
  }

  /**
   * @brief 获取接收到的帧数据指针
   * @return uint8_t* 数据指针
   */
  uint8_t *get_rx_data()
  {
    return _rx_frame.data;
  }

  // ----------------
};


#endif // __PROTOCOL_UART_HPP__
