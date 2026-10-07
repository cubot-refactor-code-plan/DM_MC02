/**
 * @file device_r9ds.c
 * @author Rh (qq:750920400)
 * @brief R9DS 遥控器接收机设备驱动（SBUS协议解析）
 * @version 0.1
 * @date 2026-05-27
 *
 * @note SBUS协议：100000波特率、8E2（8数据位、偶校验、2停止位）
 *       帧格式：25字节，帧头0x0F，帧尾0x00，16通道×11bit
 *       由 FreeRTOS 任务周期性调用 device_r9ds_update 进行帧同步与解析
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "device_r9ds.h"

#pragma section all "cpu0_dsram"

/*==============================================================================
 * 全局设备实例
 *============================================================================*/
struct DeviceR9DS g_r9ds;

#pragma section all restore

/*==============================================================================
 * 私有宏定义
 *============================================================================*/

// SBUS 帧常量
#define SBUS_FRAME_SIZE 25U      // SBUS 帧长度（字节）
#define SBUS_HEADER 0x0FU        // 帧头标识
#define SBUS_FOOTER 0x00U        // 帧尾标识
#define SBUS_CHANNEL_COUNT 16U   // SBUS 通道数量
#define SBUS_SIGNAL_MASK 0x07FFU // 11-bit 通道值掩码
#define SBUS_CHANNEL_MID 1000U   // 摇杆中位值

// 零飘死区范围（在此范围内归一化到中位值）
#define DEADZONE_CH0_LOW 950U
#define DEADZONE_CH0_HIGH 1060U
#define DEADZONE_CH1_LOW 996U
#define DEADZONE_CH1_HIGH 1004U
#define DEADZONE_CH2_LOW 994U
#define DEADZONE_CH2_HIGH 1002U
#define DEADZONE_CH3_LOW 950U
#define DEADZONE_CH3_HIGH 1060U

// 3段开关阈值
#define SWITCH_3POS_LOW 500U
#define SWITCH_3POS_MID 600U
#define SWITCH_3POS_HIGH 1200U

/*==============================================================================
 * 私有函数声明
 *============================================================================*/

/**
 * @brief SBUS 帧位提取（16通道 × 11-bit）
 * @param frame 25字节原始帧数据
 * @param channels 输出的16通道数组
 */
static void r9ds_sbus_extract_channels(const uint8 *frame, uint16 *channels);

/**
 * @brief 将SBUS通道值映射到R9DS通道布局，含偏移和死区处理
 * @param dev 设备指针
 * @param channels 16通道原始值
 */
static void r9ds_map_channels(DeviceR9DS *dev, const uint16 *channels);

/**
 * @brief 零飘死区检测与归一化
 * @param val 当前通道值
 * @param low 死区下限
 * @param high 死区上限
 * @return 处理后的通道值
 */
static uint16 r9ds_deadzone(uint16 val, uint16 low, uint16 high);

/**
 * @brief 在线状态检测
 * @param dev 设备指针
 * @param flags_byte SBUS 帧的第24字节（flags）
 */
static void r9ds_check_online(DeviceR9DS *dev, uint8 flags_byte);

/*==============================================================================
 * 私有函数实现
 *============================================================================*/

/**
 * @brief SBUS 帧位提取
 * @details SBUS 16个通道共176位（22字节），每个通道11位，小端排列
 *          提取算法与原始 R9DS 代码完全一致
 *
 *  frame[1]  frame[2]  frame[3]  ...
 *  [7...0]   [7...0]   [7...0]   ...
 *  |         |         |
 *  ch0[7:0]  ch0[10:8] ch1[4:0]  ...
 *            ch1[10:5]
 */
static void r9ds_sbus_extract_channels(const uint8 *frame, uint16 *channels)
{
  // 通道 0..7 从 frame[1]~frame[11] 提取
  channels[0] = (((uint16)frame[1] >> 0) | ((uint16)frame[2] << 8)) & SBUS_SIGNAL_MASK;
  channels[1] = (((uint16)frame[2] >> 3) | ((uint16)frame[3] << 5)) & SBUS_SIGNAL_MASK;
  channels[2] = (((uint16)frame[3] >> 6) | ((uint16)frame[4] << 2) | ((uint16)frame[5] << 10)) & SBUS_SIGNAL_MASK;
  channels[3] = (((uint16)frame[4] >> 1) | ((uint16)frame[6] << 7)) & SBUS_SIGNAL_MASK;
  channels[4] = (((uint16)frame[6] >> 4) | ((uint16)frame[7] << 4)) & SBUS_SIGNAL_MASK;
  channels[5] = (((uint16)frame[7] >> 7) | ((uint16)frame[8] << 1) | ((uint16)frame[9] << 9)) & SBUS_SIGNAL_MASK;
  channels[6] = (((uint16)frame[9] >> 2) | ((uint16)frame[10] << 6)) & SBUS_SIGNAL_MASK;
  channels[7] = (((uint16)frame[10] >> 5) | ((uint16)frame[11] << 3)) & SBUS_SIGNAL_MASK;

  // 通道 8..15 从 frame[12]~frame[22] 提取
  channels[8]  = (((uint16)frame[12] << 0) | ((uint16)frame[13] << 8)) & SBUS_SIGNAL_MASK;
  channels[9]  = (((uint16)frame[13] >> 3) | ((uint16)frame[14] << 5)) & SBUS_SIGNAL_MASK;
  channels[10] = (((uint16)frame[14] >> 6) | ((uint16)frame[15] << 2) | ((uint16)frame[18] << 10)) & SBUS_SIGNAL_MASK;
  channels[11] = (((uint16)frame[16] >> 1) | ((uint16)frame[17] << 7)) & SBUS_SIGNAL_MASK;
  channels[12] = (((uint16)frame[17] >> 4) | ((uint16)frame[18] << 4)) & SBUS_SIGNAL_MASK;
  channels[13] = (((uint16)frame[18] >> 7) | ((uint16)frame[19] << 1) | ((uint16)frame[20] << 9)) & SBUS_SIGNAL_MASK;
  channels[14] = (((uint16)frame[20] >> 2) | ((uint16)frame[21] << 6)) & SBUS_SIGNAL_MASK;
  channels[15] = (((uint16)frame[21] >> 5) | ((uint16)frame[22] << 3)) & SBUS_SIGNAL_MASK;
}

/**
 * @brief 零飘死区检测
 */
static uint16 r9ds_deadzone(uint16 val, uint16 low, uint16 high)
{
  if (val > low && val < high)
  {
    return SBUS_CHANNEL_MID;
  }
  return val;
}

/**
 * @brief 在线状态检测
 */
static void r9ds_check_online(DeviceR9DS *dev, uint8 flags_byte)
{
  // frame[23] == 0x00 表示在线（无丢帧、无失控保护激活）
  if (flags_byte == 0x00)
  {
    dev->is_online         = 1;
    dev->offline_check_cnt = 0;
  }
  else
  {
    dev->offline_check_cnt++;
    if (dev->offline_check_cnt > 10)
    {
      dev->is_online = 0;
    }
  }
}

/**
 * @brief R9DS 通道映射
 * @details 将 SBUS 16通道原始值映射到 R9DS 板卡的实际通道布局：
 *          ch0 = SBUS_CH4 (右摇杆水平) + offset
 *          ch1 = SBUS_CH3 (右摇杆垂直) + offset
 *          ch2 = SBUS_CH1 (左摇杆垂直) + offset
 *          ch3 = SBUS_CH2 (左摇杆水平) + offset
 *          sw1 = SBUS_CH6 (VR旋钮)
 *          sw2 = SBUS_CH8
 *          sw3 = SBUS_CH7
 *          s1  = SBUS_CH10 (2段开关, <500→1, else→2)
 *          s2  = SBUS_CH9  (2段开关)
 *          s3  = SBUS_CH5  (3段开关, <500→1, 600~1200→2, else→3)
 */
static void r9ds_map_channels(DeviceR9DS *dev, const uint16 *channels)
{
  // 摇杆通道（含偏移校准 + 零飘死区）
  dev->rc.ch0 = r9ds_deadzone(channels[3] + dev->ch_offset[0], DEADZONE_CH0_LOW, DEADZONE_CH0_HIGH);
  dev->rc.ch1 = r9ds_deadzone(channels[2] + dev->ch_offset[1], DEADZONE_CH1_LOW, DEADZONE_CH1_HIGH);
  dev->rc.ch2 = r9ds_deadzone(channels[0] + dev->ch_offset[2], DEADZONE_CH2_LOW, DEADZONE_CH2_HIGH);
  dev->rc.ch3 = r9ds_deadzone(channels[1] + dev->ch_offset[3], DEADZONE_CH3_LOW, DEADZONE_CH3_HIGH);

  // VR 旋钮通道（无死区、无偏移）
  dev->rc.sw1 = channels[5];
  dev->rc.sw2 = channels[7];
  dev->rc.sw3 = channels[6];

  // 2段开关 s1（SBUS_CH10）
  if (channels[9] < SWITCH_3POS_LOW)
  {
    dev->rc.s1 = 1;
  }
  else
  {
    dev->rc.s1 = 2;
  }

  // 2段开关 s2（SBUS_CH9）
  if (channels[8] < SWITCH_3POS_LOW)
  {
    dev->rc.s2 = 1;
  }
  else
  {
    dev->rc.s2 = 2;
  }

  // 3段开关 s3（SBUS_CH5）
  if (channels[4] < SWITCH_3POS_LOW)
  {
    dev->rc.s3 = 1;
  }
  else if (channels[4] < SWITCH_3POS_HIGH && channels[4] > SWITCH_3POS_MID)
  {
    dev->rc.s3 = 2;
  }
  else
  {
    dev->rc.s3 = 3;
  }
}

/*==============================================================================
 * 公共函数实现
 *============================================================================*/

/**
 * @brief 初始化 R9DS 设备
 */
void device_r9ds_init(DeviceR9DS *dev, BspUart *uart, const uint16 *ch_offset)
{
  dev->uart = uart;

  // 通道偏移量初始化
  if (ch_offset != NULL)
  {
    dev->ch_offset[0] = ch_offset[0];
    dev->ch_offset[1] = ch_offset[1];
    dev->ch_offset[2] = ch_offset[2];
    dev->ch_offset[3] = ch_offset[3];
  }
  else
  {
    dev->ch_offset[0] = 0;
    dev->ch_offset[1] = 0;
    dev->ch_offset[2] = 0;
    dev->ch_offset[3] = 0;
  }

  // 通道默认值（中位）
  dev->rc.ch0 = SBUS_CHANNEL_MID;
  dev->rc.ch1 = SBUS_CHANNEL_MID;
  dev->rc.ch2 = SBUS_CHANNEL_MID;
  dev->rc.ch3 = SBUS_CHANNEL_MID;
  dev->rc.s1  = 1;
  dev->rc.s2  = 1;
  dev->rc.s3  = 2;
  dev->rc.sw1 = 200;
  dev->rc.sw2 = 200;
  dev->rc.sw3 = 200;

  // 离线状态初始化
  dev->is_online         = 0;
  dev->offline_check_cnt = 0;
  dev->offline_tick      = 1000; // 初始视为离线（未收到任何帧）
}

/**
 * @brief 解析 SBUS 帧数据
 * @note 采用帧头搜索方式：先确保 FIFO 中至少有 25 字节再开始逐字节搜索 0x0F，
 *       不足 25 字节时立即返回等待下次中断补充数据，避免假帧头导致丢字节错位
 */
void device_r9ds_update(DeviceR9DS *dev)
{
  // 每 1ms 递增离线计时器（由 1ms ISR 调用），上限 2000 防止溢出
  if (dev->offline_tick < 10000)
  {
    dev->offline_tick++;
  }

  // 必须有至少一帧的数据才动手，不够就等下次中断
  if (bsp_uart_available(dev->uart) < SBUS_FRAME_SIZE)
  {
    return;
  }

  // 有足够数据，逐字节搜索帧头，搜到不足 25 字节时自动停止
  while (bsp_uart_available(dev->uart) >= SBUS_FRAME_SIZE)
  {
    uint8 byte = 0;
    bsp_uart_recv(dev->uart, &byte, 1);

    // 找到帧头候选
    if (byte == SBUS_HEADER)
    {
      uint8 frame[SBUS_FRAME_SIZE];
      frame[0] = SBUS_HEADER;

      // 已确保 FIFO >= 25 字节（含刚读的 0x0F），剩余至少 24 字节可直接读取
      bsp_uart_recv(dev->uart, &frame[1], SBUS_FRAME_SIZE - 1);

      // 验证帧尾
      if (frame[SBUS_FRAME_SIZE - 1] == SBUS_FOOTER)
      {
        // 有效帧 — 解析
        uint16 channels[SBUS_CHANNEL_COUNT];
        r9ds_sbus_extract_channels(frame, channels);
        r9ds_map_channels(dev, channels);
        r9ds_check_online(dev, frame[23]);
        dev->offline_tick = 0; // 收到有效帧，重置离线计时器
        return;                // 成功解析一帧，退出
      }
      // 帧尾不匹配：25 字节已消耗，while 条件会判断是否还有 >= 25 字节继续搜
    }
    // 非 0x0F：1 字节已消耗，继续循环搜索下一个
  }
  // FIFO 不足 25 字节，等待下次中断补充数据
}

/**
 * @brief 获取在线状态（基于 1.5s 无数据超时）
 * @note 由 1ms ISR 驱动 offline_tick 递增，收到有效帧时清零。
 *       连续 1.5s（1500ms）未收到有效 SBUS 帧即判定为离线。
 */
uint8 device_r9ds_is_online(DeviceR9DS *dev)
{
  return (dev->offline_tick < 1500U) ? 1 : 0;
}

/**
 * @brief R9DS 设备总初始化
 * @note 默认使用 UART2（P14_2/P14_3）作为 SBUS 接收端口，
 *
 */
void device_r9ds_all_init(void)
{
  const uint16 r9ds_offset[] = R9DS_CH_OFFSET_DEFAULT;
  device_r9ds_init(&g_r9ds, &bsp_uart1, r9ds_offset);
}
