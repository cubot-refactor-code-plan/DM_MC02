#ifndef __DEVICE_R9DS_H__
#define __DEVICE_R9DS_H__

#include "bsp_uart.h"
#include "zf_common_headfile.h"

/*==============================================================================
 * R9DS 通道默认偏移量
 * @brief 用于校准各通道的零飘，{ch0, ch1, ch2, ch3}
 *============================================================================*/
#define R9DS_CH_OFFSET_DEFAULT {100, 0, 0, 10}

/*==============================================================================
 * 遥控器通道数据结构体
 * @brief 存储解析后的 SBUS 通道值与开关状态
 *============================================================================*/
typedef struct
{
  int16 ch0; // 右摇杆水平（ch4）
  int16 ch1; // 右摇杆垂直（ch3）
  int16 ch2; // 左摇杆垂直（ch1）
  int16 ch3; // 左摇杆水平（ch2）
  int8  s1;  // 2段开关
  int8  s2;  // 2段开关
  int8  s3;  // 3段开关
  int16 sw1; // VR旋钮1
  int16 sw2; // VR旋钮2
  int16 sw3; // VR旋钮3
} R9DS_Channels;

/*==============================================================================
 * R9DS 遥控器接收机设备定义
 * @brief 负责 SBUS 协议帧解析与通道映射，使用 BspUart 层接收数据，
 *        由 FreeRTOS 任务周期性调用 device_r9ds_update 进行帧解析
 *============================================================================*/
typedef struct DeviceR9DS
{
  BspUart           *uart;              // 串口设备指针（需配置为 SBUS 模式 100000/8E2）
  R9DS_Channels      rc;                // 遥控器通道数据
  uint16             ch_offset[4];      // 通道零飘偏移量 {ch0, ch1, ch2, ch3}
  uint8              is_online;         // 遥控器在线标志（保留兼容）
  uint32             offline_check_cnt; // 离线检测计数（保留兼容）
  uint32 offline_tick;      // 距上次有效帧的毫秒计数，≥1000=离线
} DeviceR9DS;

/*==============================================================================
 * 公共函数声明
 *============================================================================*/

/**
 * @brief 初始化 R9DS 设备
 * @param dev 设备指针
 * @param uart 已配置为 SBUS 模式（100000/8E2）的 BspUart 指针
 * @param ch_offset 通道偏移量数组（4元素：ch0~ch3），传 NULL 则使用默认全零
 */
void device_r9ds_init(DeviceR9DS *dev, BspUart *uart, const uint16 *ch_offset);

/**
 * @brief 解析 SBUS 帧数据（由 FreeRTOS 任务周期性调用）
 * @param dev 设备指针
 * @note 内部使用帧头搜索方式从 FIFO 中提取 25 字节 SBUS 帧，
 *       验证帧头 0x0F + 帧尾 0x00 后调用位提取与通道映射
 */
void device_r9ds_update(DeviceR9DS *dev);

/**
 * @brief 获取遥控器在线状态
 * @param dev 设备指针
 * @return 1：在线  0：离线
 */
uint8 device_r9ds_is_online(DeviceR9DS *dev);

/*==============================================================================
 * 全局设备实例（外部可访问）
 *============================================================================*/
extern struct DeviceR9DS g_r9ds;

/**
 * @brief R9DS 设备总初始化（含 UART 配置）
 */
void device_r9ds_all_init(void);

#endif // __DEVICE_R9DS_H__
