/**
 * @file ProtocolUart.hpp
 * @author Rhprotocol_frame
 * @brief 串口协议解析头文件
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
 *       - 先实例化bsp_usart（在bsp文件中）
 *       - 全局类对象实例化：ProtocolUart protocal_uart_6(&bsp_usart6, 6);
 *       - 初始化协议：protocal_uart_6.init();
 */

#ifndef __PROTOCOL_UART_HPP__
#define __PROTOCOL_UART_HPP__


#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"     // IWYU pragma: keep
#include "stddef.h"
#include "stdint.h"


/* USER CODE BEGIN */

/* ==================== 外部声明 ==================== */

template <size_t BUFFER_SIZE, size_t MSG_SIZE>
class BspUart;

class ProtocolUart;
/**
 * @brief 全局串口协议实例
 */
extern ProtocolUart protocal_usart_1;

/* USER CODE END */


/**
 * @brief 协议帧结构体定义
 *
 * @note 对应上位机：AA 55 CMD LEN DATA... SUM 0D（帧头帧尾可自定义）
 */
#pragma pack(1)
typedef struct ProtocolFrame
{
  uint8_t header1;  ///< 帧头1（默认0xAA）
  uint8_t header2;  ///< 帧头2（默认0x55）
  uint8_t cmd;      ///< 指令码
  uint8_t len;      ///< 有效负载长度
  uint8_t data[64]; ///< 数据负载（最大64字节）
  uint8_t checksum; ///< 校验和
  uint8_t tail;     ///< 帧尾（默认0x0D）
} protocol_frame_t;
#pragma pack()


/**
 * @brief 串口协议类
 */
class ProtocolUart
{
public:
  /* ==================== 任务入口访问面（uart_protocol_task_entry 直接读写） ==================== */

  ProtocolFrame _rx_frame;      ///< 接收用结构体
  BspUart<128> &_uart_instance; ///< 使用的串口驱动实例
  uint8_t       _header1;       ///< 自定义帧头1
  uint8_t       _header2;       ///< 自定义帧头2
  uint8_t       _tail;          ///< 自定义帧尾

  /**
   * @brief 逻辑分发：根据指令执行具体动作
   */
  void protocol_handle_cmd();


private:
  /* ==================== 私有成员变量 ==================== */

  uint8_t  _instance_name; ///< 实例名称编号
  char     _task_name[32]; ///< 任务名称
  uint32_t _stack_size;    ///< 堆栈大小
  uint32_t _priority;      ///< 任务优先级

  /* ==================== 私有成员函数 ==================== */

  /**
   * @brief 计算校验和
   * @param data 数据缓冲区
   * @param len 长度
   * @return uint8_t 校验结果
   */
  uint8_t calculate_checksum(uint8_t* data, uint8_t len);


public:
  /* ==================== 构造函数与析构函数 ==================== */

  /**
   * @brief 构造函数
   * @param uart_ptr 串口实例指针
   * @param name 实例名称编号
   * @param h1 帧头1（默认0xAA）
   * @param h2 帧头2（默认0x55）
   * @param t 帧尾（默认0x0D）
   */
  ProtocolUart(BspUart<64, 8>& uart_ptr, uint8_t name, uint8_t h1 = 0xAA, uint8_t h2 = 0x55, uint8_t t = 0x0C);


  /* ==================== 公共接口 ==================== */

  /**
   * @brief 协议处理初始化
   */
  void init();

  /**
   * @brief 发送协议数据包给上位机
   * @param cmd 指令码
   * @param data 数据指针
   * @param len 数据长度
   */
  void send(uint8_t cmd, uint8_t* data, uint8_t len);

  /**
   * @brief 获取接收到的帧命令码
   * @return uint8_t 命令码
   */
  uint8_t get_rx_cmd()
  {
    return rx_frame.cmd;
  }

  /**
   * @brief 获取接收到的帧数据长度
   * @return uint8_t 数据长度
   */
  uint8_t get_rx_len()
  {
    return rx_frame.len;
  }

  /**
   * @brief 获取接收到的帧数据指针
   * @return uint8_t* 数据指针
   */
  uint8_t* get_rx_data()
  {
    return rx_frame.data;
  }
};


#endif // __PROTOCOL_USART_HPP__
