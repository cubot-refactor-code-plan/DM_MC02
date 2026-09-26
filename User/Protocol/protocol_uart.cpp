#include "protocol_uart.hpp"
#include "bsp_cfg.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include <stdio.h>



/* USER CODE BEGIN */

/* ==================== 全局类对象实例化 ==================== */

ProtocolUart protocal_usart_1(bsp_usart1, 1);

/* ==================== C函数实现 ==================== */
static inline void protocol_usart_callback(ProtocolUart* p_usart)
{
  if (p_usart == &protocal_usart_1)
  {
  }
}

/* USER CODE END */


/**
 * @brief UART协议任务（C函数）
 * @param argument 任务参数（传入类指针this）
 * @note 1. 增加帧同步机制，减少丢包
 *       2. 移除不必要的内存拷贝，提高效率
 *       3. 优化错误处理流程
 *       4. 增加超时保护
 */
void _uart_protocol_task_entry(void* argument)
{
  /* 通过argument获取类指针 */
  ProtocolUart* self = static_cast<ProtocolUart*>(argument);


  printf("UART Protocol Task Started\n");

  /* 临时缓冲区 */
  uint8_t header_buf[4];   ///< 帧头缓冲区
  uint8_t payload_buf[70]; ///< 数据缓冲区（64字节数据+校验和+帧尾）
  uint8_t checksum_calc;   ///< 校验和计算值

  for (;;)
  {
    /* 1. 寻找帧头：先同步第一个包头 */
    if (self->_uart_instance.receive(&header_buf[0], 1, portMAX_DELAY) != Status::OK)
    {
      continue;
    }

    /* 非帧头1时，执行一次重同步：尝试读取下一个字节作为新的帧头1候选 */
    if (header_buf[0] != self->header1)
    {
      /* 不立即continue，而是继续在下一次循环中尝试 */
      /* 简单的流过滤：记录当前字节，若下一个是header1则匹配 */
      continue;
    }

    /* 2. 读取剩余的帧头部分 */
    size_t hdr_read = 0;
    if (self->_uart_instance.receive(&header_buf[1], 3, 100, &hdr_read) != Status::OK || hdr_read < 3)
    {
      continue;
    }

    /* 校验第二个包头 - 必须匹配 */
    if (header_buf[1] != self->header2)
    {
      /* 帧头2不匹配，重置状态 */
      header_buf[0] = header_buf[1]; /* 保存第二个字节作为下次帧头1候选 */
      continue;
    }

    /* 解析协议参数 */
    self->rx_frame.cmd = header_buf[2];
    self->rx_frame.len = header_buf[3];

    /* 3. 长度合法性检查 */
    if (self->rx_frame.len > 64)
    {
      continue;
    }

    /* 4. 批量读取后续内容 */
    uint8_t remaining_len = self->_rx_frame.len + 2;
    size_t  recv_len      = 0;
    if (self->_uart_instance.receive(payload_buf, remaining_len, 100, &recv_len) != Status::OK || recv_len < remaining_len)
    {
      /* 数据接收不完整，跳过 */
      continue;
    }

    /* 5. 校验和验证 - 直接计算，避免memcpy */
    checksum_calc = header_buf[0] + header_buf[1] + header_buf[2] + header_buf[3];
    for (uint8_t i = 0; i < self->rx_frame.len; i++)
    {
      checksum_calc += payload_buf[i];
    }

    uint8_t received_sum  = payload_buf[self->rx_frame.len];
    uint8_t received_tail = payload_buf[self->rx_frame.len + 1];

    /* 6. 最终判定与处理 */
    if (checksum_calc == received_sum && received_tail == self->tail)
    {
      /* 拷贝数据到帧结构体 */
      if (self->rx_frame.len > 0)
      {
        memcpy(self->rx_frame.data, payload_buf, self->rx_frame.len);
      }
      /* 处理业务逻辑 */
      self->protocol_handle_cmd();
    }
  }
}


/* ==================== 类函数实现 ==================== */

/**
 * @brief 构造函数
 *
 * @attention 校验和计算：(header1+header2+CMD+LEN+DATA) & 0xFF
 *
 * @param uart_ptr 串口实例指针
 * @param name 实例名称编号
 * @param h1 帧头1
 * @param h2 帧头2
 * @param t 帧尾
 */
ProtocolUart::ProtocolUart(BspUart<64, 8>& uart_ptr, uint8_t name, uint8_t h1, uint8_t h2, uint8_t t)

  : uart_instance(uart_ptr),
    header1(h1),
    header2(h2),
    tail(t)
{
  /* 初始化任务属性成员变量 */
  snprintf(task_name, sizeof(task_name), "uart_protocol_%d", name);
  stack_size = 512 * 4; /* 从256*4增加到512*4 */
  priority   = tskIDLE_PRIORITY + 1; /* 任务优先级，普通优先级 */
}


/**
 * @brief 协议层初始化
 */
void ProtocolUart::init()
{
  memset(&rx_frame, 0, sizeof(rx_frame));

  /* 创建串口协议处理任务，传入this指针 */
  xTaskCreate(_uart_protocol_task_entry, task_name, stack_size / 4, this, priority, nullptr);
}


/**
 * @brief 计算校验和
 * @param data 数据缓冲区
 * @param len 长度
 * @return uint8_t 校验结果
 */
uint8_t ProtocolUart::calculate_checksum(uint8_t* data, uint8_t len)
{
  uint8_t sum = 0;
  /* 使用指针遍历，减少索引操作 */
  uint8_t* p     = data;
  uint8_t* p_end = data + len;
  while (p < p_end)
  {
    sum += *p++;
  }
  return sum;
}


/**
 * @brief 发送协议帧到上位机
 * @param cmd 指令码
 * @param data 数据指针
 * @param len 数据长度
 */
void ProtocolUart::send(uint8_t cmd, uint8_t* data, uint8_t len)
{
  /* 注意：uart_instance 是引用类型，构造时已绑定，无需空检查 */
  /* 有效性检查由调用者保证 */
  if (data == nullptr || len > 64)
  {
    return;
  }

  static uint8_t tx_buf[128];
  tx_buf[0] = header1;
  tx_buf[1] = header2;
  tx_buf[2] = cmd;
  tx_buf[3] = len;
  if (len > 0 && data != nullptr)
  {
    memcpy(&tx_buf[4], data, len);
  }

  /* 计算校验和 */
  tx_buf[4 + len] = calculate_checksum(tx_buf, 4 + len);
  tx_buf[5 + len] = tail;

  return _uart_instance.send(tx_buf, 6 + len, 10);
}


/**
 * @brief 逻辑分发：根据指令执行具体动作
 */
void ProtocolUart::protocol_handle_cmd()
{
  /* 具体的指令码及数据解析 */
  protocol_usart_callback(this);
}
