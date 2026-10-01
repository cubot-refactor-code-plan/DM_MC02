#include "dm_imu.hpp"
#include "FreeRTOS.h" // IWYU pragma: keep
#include "semphr.h"
#include <stdio.h>
#include <string.h>


// ---------------- 构造函数与析构函数 ----------------

/**
 * @brief 构造函数
 * @param cfg IMU 配置（CAN 接口/设备ID/主机ID，可匿名按序传入）
 */
DmImu::DmImu(const Config& cfg)

  : _can_bus(cfg.can_bus),
    _device_id(cfg.device_id),
    _master_id(cfg.master_id)
{
  // 构造函数只做赋值；数据区初始化推迟到 init()
}


/**
 * @brief 析构函数
 */
DmImu::~DmImu()
{
  /* 删除互斥锁 */
  if (_data_mutex_handle != nullptr)
  {
    vSemaphoreDelete(_data_mutex_handle);
    _data_mutex_handle = nullptr;
  }
}


// ----------------
// ---------------- 公共接口实现 ----------------

/**
 * @brief 初始化IMU
 */
Status DmImu::init()
{
  snprintf(_name, sizeof(_name), "IMU_Data_Mutex");

  /* 初始化内部数据结构 */
  memset(&_imu_data, 0, sizeof(_imu_data));
  _imu_data.can_id = _device_id;
  _imu_data.mst_id = _master_id;

  /* 创建互斥锁 */
  _data_mutex_handle = xSemaphoreCreateMutex();

  return (_data_mutex_handle != nullptr) ? Status::OK : Status::IO_ERROR;
}


/**
 * @brief 写寄存器
 * @param reg_id 寄存器ID
 * @param data 写入数据
 */
void DmImu::_write_register(RegId reg_id, uint32_t data)
{
  uint8_t buf[8] = {0xCC, static_cast<uint8_t>(reg_id), CMD_WRITE, 0xDD, 0, 0, 0, 0};
  memcpy(buf + 4, &data, 4);

  _can_bus.send(_device_id, buf);
}


/**
 * @brief 读寄存器
 * @param reg_id 寄存器ID
 */
void DmImu::_read_register(RegId reg_id)
{
  uint8_t buf[8] = {0xCC, static_cast<uint8_t>(reg_id), CMD_READ, 0xDD, 0, 0, 0, 0};

  _can_bus.send(_device_id, buf);
}


/**
 * @brief 重启IMU
 */
void DmImu::reboot()
{
  _write_register(RegId::REBOOT_IMU, 0);
}


/**
 * @brief 加速度计校准
 */
void DmImu::accel_calibration()
{
  _write_register(RegId::ACCEL_CALI, 0);
}


/**
 * @brief 陀螺仪校准
 */
void DmImu::gyro_calibration()
{
  _write_register(RegId::GYRO_CALI, 0);
}


/**
 * @brief 更改通信端口
 * @param port 通信端口
 */
void DmImu::change_com_port(ImuComPort port)
{
  _write_register(RegId::CHANGE_COM, static_cast<uint8_t>(port));
}


/**
 * @brief 设置主动模式延时
 * @param delay 延时时间
 */
void DmImu::set_active_mode_delay(uint32_t delay)
{
  _write_register(RegId::SET_DELAY, delay);
}


/**
 * @brief 切换到主动模式
 */
void DmImu::change_to_active()
{
  _write_register(RegId::CHANGE_ACTIVE, 1);
}


/**
 * @brief 切换到请求模式
 */
void DmImu::change_to_request()
{
  _write_register(RegId::CHANGE_ACTIVE, 0);
}


/**
 * @brief 设置波特率
 * @param baud 波特率
 */
void DmImu::set_baud(ImuBaudrate baud)
{
  _write_register(RegId::SET_BAUD, static_cast<uint8_t>(baud));
}


/**
 * @brief 设置CAN ID
 * @param can_id CAN ID
 */
void DmImu::set_can_id(uint8_t can_id)
{
  _write_register(RegId::SET_CAN_ID, can_id);
}


/**
 * @brief 设置主机ID
 * @param mst_id 主机ID
 */
void DmImu::set_mst_id(uint8_t mst_id)
{
  _write_register(RegId::SET_MST_ID, mst_id);
}


/**
 * @brief 保存参数
 */
void DmImu::save_parameters()
{
  _write_register(RegId::SAVE_PARAM, 0);
}


/**
 * @brief 恢复设置
 */
void DmImu::restore_settings()
{
  _write_register(RegId::RESTORE_SETTING, 0);
}


/**
 * @brief 请求欧拉角数据
 */
void DmImu::request_euler()
{
  _read_register(RegId::EULER_DATA);
}


/**
 * @brief 请求四元数数据
 */
void DmImu::request_quat()
{
  _read_register(RegId::QUAT_DATA);
}


/**
 * @brief 获取IMU数据（线程安全）
 * @return ImuData IMU数据
 */
ImuData DmImu::get_imu_data()
{
  ImuData data_copy;

  if (_data_mutex_handle != nullptr)
  {
    xSemaphoreTake(_data_mutex_handle, portMAX_DELAY);
  }

  /* 复制数据 */
  data_copy = _imu_data;

  /* 退出临界区：释放互斥锁 */
  if (_data_mutex_handle != nullptr)
  {
    xSemaphoreGive(_data_mutex_handle);
  }

  return data_copy;
}


/**
 * @brief 设置IMU数据（线程安全）
 * @param data IMU数据
 */
void DmImu::set_imu_data(const ImuData& data)
{
  if (_data_mutex_handle != nullptr)
  {
    xSemaphoreTake(_data_mutex_handle, portMAX_DELAY);
  }

  /* 更新数据 */
  _imu_data = data;

  /* 退出临界区：释放互斥锁并恢复中断 */
  if (_data_mutex_handle != nullptr)
  {
    xSemaphoreGive(_data_mutex_handle);
  }
}


// ----------------
// ---------------- 私有辅助函数实现 ----------------

/**
 * @brief 浮点数转整数
 *
 * @param value 浮点数值
 * @param min 最小值
 * @param max 最大值
 * @param bits 位数
 * @return int 转换后的整数
 */
int DmImu::_float_to_int(float value, float min, float max, int bits)
{
  /* 将浮点数按给定范围与位数映射到整数 */
  float span   = max - min;
  float offset = min;
  return (int)((value - offset) * ((float)((1 << bits) - 1)) / span);
}


/**
 * @brief 整数转浮点数
 *
 * @param value 整数值
 * @param min 最小值
 * @param max 最大值
 * @param bits 位数
 * @return float 转换后的浮点数
 */
float DmImu::_uint_to_float(int value, float min, float max, int bits)
{
  /* 将整数按给定范围与位数映射回浮点数 */
  float span   = max - min;
  float offset = min;
  return ((float)value) * span / ((float)((1 << bits) - 1)) + offset;
}


/**
 * @brief 更新欧拉角数据
 * @param data 数据数组引用
 */
void DmImu::_update_euler(const uint8_t (&data)[8])
{
  int16_t euler[3];

  euler[0] = static_cast<int16_t>((data[3] << 8) | data[2]);
  euler[1] = static_cast<int16_t>((data[5] << 8) | data[4]);
  euler[2] = static_cast<int16_t>((data[7] << 8) | data[6]);

  if (_data_mutex_handle != nullptr)
  {
    xSemaphoreTake(_data_mutex_handle, portMAX_DELAY);
  }

  _imu_data.pitch = _uint_to_float(euler[0], PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
  _imu_data.yaw   = _uint_to_float(euler[1], YAW_CAN_MIN, YAW_CAN_MAX, 16);
  _imu_data.roll  = _uint_to_float(euler[2], ROLL_CAN_MIN, ROLL_CAN_MAX, 16);

  /* 退出临界区：释放互斥锁并恢复中断 */
  if (_data_mutex_handle != nullptr)
  {
    xSemaphoreGive(_data_mutex_handle);
  }
}


/**
 * @brief 更新四元数数据
 * @param data 数据数组引用
 */
void DmImu::_update_quaternion(const uint8_t (&data)[8])
{
  int w = data[1] << 6 | ((data[2] & 0xF8) >> 2);
  int x = (data[2] & 0x03) << 12 | (data[3] << 4) | ((data[4] & 0xF0) >> 4);
  int y = (data[4] & 0x0F) << 10 | (data[5] << 2) | ((data[6] & 0xC0) >> 6);
  int z = (data[6] & 0x3F) << 8 | data[7];

  if (_data_mutex_handle != nullptr)
  {
    xSemaphoreTake(_data_mutex_handle, portMAX_DELAY);
  }

  _imu_data.q[0] = _uint_to_float(w, QUATERNION_MIN, QUATERNION_MAX, 14);
  _imu_data.q[1] = _uint_to_float(x, QUATERNION_MIN, QUATERNION_MAX, 14);
  _imu_data.q[2] = _uint_to_float(y, QUATERNION_MIN, QUATERNION_MAX, 14);
  _imu_data.q[3] = _uint_to_float(z, QUATERNION_MIN, QUATERNION_MAX, 14);

  /* 退出临界区：释放互斥锁并恢复中断 */
  if (_data_mutex_handle != nullptr)
  {
    xSemaphoreGive(_data_mutex_handle);
  }
}


/**
 * @brief CAN消息回调处理
 * @param rx_msg CAN接收消息
 *
 * @note 本函数只按 data[0] 判帧类型，不校验 CAN ID；
 *       由上层分发（CAN 接收任务）先按帧 ID 过滤后调用。
 */
void DmImu::on_can_message(const CanRxMsg& rx_msg)
{
  if (rx_msg.data[0] == 0x03)
  {
    _update_euler(rx_msg.data);
  }
  else if (rx_msg.data[0] == 0x04)
  {
    _update_quaternion(rx_msg.data);
  }
}

// ----------------
