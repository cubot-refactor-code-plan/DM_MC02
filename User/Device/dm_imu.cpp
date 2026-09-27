#include "dm_imu.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>


/* USER CODE BEGIN */

/* ==================== 全局对象实例化 ==================== */

DmImu imu_bmi088(bsp_can1, 0x58, 0x59);

/* USER CODE END */


/* ==================== 寄存器ID定义 ==================== */

typedef enum RegId_e
{
  REBOOT_IMU = 0,        ///< 重启IMU
  ACCEL_DATA,            ///< 加速度数据
  GYRO_DATA,             ///< 陀螺仪数据
  EULER_DATA,            ///< 欧拉角数据
  QUAT_DATA,             ///< 四元数数据
  SET_ZERO,              ///< 设置零点
  ACCEL_CALI,            ///< 加速度计校准
  GYRO_CALI,             ///< 陀螺仪校准
  MAG_CALI,              ///< 磁力计校准
  CHANGE_COM,            ///< 更改通信端口
  SET_DELAY,             ///< 设置延时
  CHANGE_ACTIVE,         ///< 更改激活状态
  SET_BAUD,              ///< 设置波特率
  SET_CAN_ID,            ///< 设置CAN ID
  SET_MST_ID,            ///< 设置主ID
  DATA_OUTPUT_SELECTION, ///< 数据输出选择
  SAVE_PARAM      = 254, ///< 保存参数
  RESTORE_SETTING = 255  ///< 恢复设置
} RegId_e;


/* ==================== 构造函数与析构函数 ==================== */

/**
 * @brief 构造函数
 *
 * @param can_bus 使用的bsp_can地址
 * @param device_id 设备ID
 * @param master_id 主机ID
 */
DmImu::DmImu(BspCan& can_bus, uint8_t device_id, uint8_t master_id)

  : _device_id(device_id),
    _master_id(master_id),
    _can_bus(can_bus)
{
  /* 初始化内部数据结构 */
  memset(&_imu_data, 0, sizeof(_imu_data));
  _imu_data.can_id = device_id;
  _imu_data.mst_id = master_id;
}


/**
 * @brief 析构函数
 */
DmImu::~DmImu()
{
  /* 删除互斥锁 */
  if (_data_mutex_handle != NULL)
  {
    vSemaphoreDelete(_data_mutex_handle);
    _data_mutex_handle = NULL;
  }
}


/* ==================== 公共接口实现 ==================== */

/**
 * @brief 初始化IMU
 */
void DmImu::init()
{
  snprintf(name, sizeof(name), "IMU_Data_Mutex");

  /* 创建互斥锁 */
  _data_mutex_handle = xSemaphoreCreateMutex();
}


/**
 * @brief 写寄存器
 * @param reg_id 寄存器ID
 * @param data 写入数据
 */
void DmImu::write_register(uint8_t reg_id, uint32_t data)
{
  uint8_t buf[8] = {0xCC, reg_id, CMD_WRITE, 0xDD, 0, 0, 0, 0};
  memcpy(buf + 4, &data, 4);

  _can_bus.send(_device_id, buf);
}


/**
 * @brief 读寄存器
 * @param reg_id 寄存器ID
 */
void DmImu::read_register(uint8_t reg_id)
{
  uint8_t buf[8] = {0xCC, reg_id, CMD_READ, 0xDD, 0, 0, 0, 0};

  _can_bus.send(_device_id, buf);
}


/**
 * @brief 重启IMU
 */
void DmImu::reboot()
{
  write_register(REBOOT_IMU, 0);
}


/**
 * @brief 加速度计校准
 */
void DmImu::accel_calibration()
{
  write_register(ACCEL_CALI, 0);
}


/**
 * @brief 陀螺仪校准
 */
void DmImu::gyro_calibration()
{
  write_register(GYRO_CALI, 0);
}


/**
 * @brief 更改通信端口
 * @param port 通信端口
 */
void DmImu::change_com_port(imu_com_port_e port)
{
  write_register(CHANGE_COM, static_cast<uint8_t>(port));
}


/**
 * @brief 设置主动模式延时
 * @param delay 延时时间
 */
void DmImu::set_active_mode_delay(uint32_t delay)
{
  write_register(SET_DELAY, delay);
}


/**
 * @brief 切换到主动模式
 */
void DmImu::change_to_active()
{
  write_register(CHANGE_ACTIVE, 1);
}


/**
 * @brief 切换到请求模式
 */
void DmImu::change_to_request()
{
  write_register(CHANGE_ACTIVE, 0);
}


/**
 * @brief 设置波特率
 * @param baud 波特率
 */
void DmImu::set_baud(imu_baudrate_e baud)
{
  write_register(SET_BAUD, static_cast<uint8_t>(baud));
}


/**
 * @brief 设置CAN ID
 * @param can_id CAN ID
 */
void DmImu::set_can_id(uint8_t can_id)
{
  write_register(SET_CAN_ID, can_id);
}


/**
 * @brief 设置主机ID
 * @param mst_id 主机ID
 */
void DmImu::set_mst_id(uint8_t mst_id)
{
  write_register(SET_MST_ID, mst_id);
}


/**
 * @brief 保存参数
 */
void DmImu::save_parameters()
{
  write_register(SAVE_PARAM, 0);
}


/**
 * @brief 恢复设置
 */
void DmImu::restore_settings()
{
  write_register(RESTORE_SETTING, 0);
}


/**
 * @brief 请求欧拉角数据
 */
void DmImu::request_euler()
{
  read_register(EULER_DATA);
}


/**
 * @brief 请求四元数数据
 */
void DmImu::request_quat()
{
  read_register(QUAT_DATA);
}


/**
 * @brief 获取IMU数据（线程安全）
 * @return imu_data IMU数据
 */
imu_data DmImu::get_imu_data()
{
  imu_data data_copy;

  if (_data_mutex_handle != NULL)
  {
    xSemaphoreTake(_data_mutex_handle, portMAX_DELAY);
  }

  /* 复制数据 */
  data_copy = _imu_data;

  /* 退出临界区：释放互斥锁 */
  if (_data_mutex_handle != NULL)
  {
    xSemaphoreGive(_data_mutex_handle);
  }

  return data_copy;
}


/**
 * @brief 设置IMU数据（线程安全）
 * @param data IMU数据
 */
void DmImu::set_imu_data(const imu_data& data)
{
  if (_data_mutex_handle != NULL)
  {
    xSemaphoreTake(_data_mutex_handle, portMAX_DELAY);
  }

  /* 更新数据 */
  _imu_data = data;

  /* 退出临界区：释放互斥锁并恢复中断 */
  if (_data_mutex_handle != NULL)
  {
    xSemaphoreGive(_data_mutex_handle);
  }
}


/* ==================== 私有辅助函数实现 ==================== */

/**
 * @brief 浮点数转无符号整数
 *
 * @param x_float 浮点数值
 * @param x_min 最小值
 * @param x_max 最大值
 * @param bits 位数
 * @return int 转换后的无符号整数
 */
int DmImu::float_to_uint(float x_float, float x_min, float x_max, int bits)
{
  /* Converts a float to an unsigned int, given range and number of bits */
  float span   = x_max - x_min;
  float offset = x_min;
  return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}


/**
 * @brief 无符号整数转浮点数
 *
 * @param x_int 无符号整数值
 * @param x_min 最小值
 * @param x_max 最大值
 * @param bits 位数
 * @return float 转换后的浮点数
 */
float DmImu::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  /* converts unsigned int to float, given range and number of bits */
  float span   = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}


/**
 * @brief 更新欧拉角数据
 * @param pData 数据指针
 */
void DmImu::update_euler(const uint8_t (&data)[8])
{
  int16_t euler[3];

  euler[0] = static_cast<int16_t>((data[3] << 8) | data[2]);
  euler[1] = static_cast<int16_t>((data[5] << 8) | data[4]);
  euler[2] = static_cast<int16_t>((data[7] << 8) | data[6]);

  if (_data_mutex_handle != NULL)
  {
    xSemaphoreTake(_data_mutex_handle, portMAX_DELAY);
  }

  _imu_data.pitch = uint_to_float(euler[0], PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
  _imu_data.yaw   = uint_to_float(euler[1], YAW_CAN_MIN, YAW_CAN_MAX, 16);
  _imu_data.roll  = uint_to_float(euler[2], ROLL_CAN_MIN, ROLL_CAN_MAX, 16);

  /* 退出临界区：释放互斥锁并恢复中断 */
  if (_data_mutex_handle != NULL)
  {
    xSemaphoreGive(_data_mutex_handle);
  }
}


/**
 * @brief 更新四元数数据
 * @param pData 数据指针
 */
void DmImu::update_quaternion(const uint8_t (&data)[8])
{
  int w = data[1] << 6 | ((data[2] & 0xF8) >> 2);
  int x = (data[2] & 0x03) << 12 | (data[3] << 4) | ((data[4] & 0xF0) >> 4);
  int y = (data[4] & 0x0F) << 10 | (data[5] << 2) | ((data[6] & 0xC0) >> 6);
  int z = (data[6] & 0x3F) << 8 | data[7];

  if (_data_mutex_handle != NULL)
  {
    xSemaphoreTake(_data_mutex_handle, portMAX_DELAY);
  }

  _imu_data.q[0] = uint_to_float(w, Quaternion_MIN, Quaternion_MAX, 14);
  _imu_data.q[1] = uint_to_float(x, Quaternion_MIN, Quaternion_MAX, 14);
  _imu_data.q[2] = uint_to_float(y, Quaternion_MIN, Quaternion_MAX, 14);
  _imu_data.q[3] = uint_to_float(z, Quaternion_MIN, Quaternion_MAX, 14);

  /* 退出临界区：释放互斥锁并恢复中断 */
  if (_data_mutex_handle != NULL)
  {
    xSemaphoreGive(_data_mutex_handle);
  }
}


/**
 * @brief CAN消息回调处理
 * @param rx_msg CAN接收消息
 */
void DmImu::on_can_message(const CanRxMsg& rx_msg)
{
  if (rx_msg.data[0] == 0x03)
  {
    update_euler(rx_msg.data);
  }
  else if (rx_msg.data[0] == 0x04)
  {
    update_quaternion(rx_msg.data);
  }
}
