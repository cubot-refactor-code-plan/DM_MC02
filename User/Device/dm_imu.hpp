/**
 * @file dm_imu.hpp
 * @author Rh
 * @brief 使用bsp_can接口封装的DM IMU类
 * @version 0.1
 * @date 2026-03-02
 *
 * @todo 达妙的IMU经常漂
 *
 * @copyright Copyright (c) 2026
 *
 * @note 实例化与初始化（全局实例 imu_bmi088 在 device_cfg.cpp 中定义）
 *
 *       imu_bmi088.init();
 *
 * @note CAN接收处理
 *
 *       imu_bmi088.on_can_message(rx_msg); // 在app_message中的can消息分发中调用
 *
 * @note 数据获取
 *
 *       imu_bmi088.change_to_active();                     // 主动发送数据
 *       ImuData user_imu_data = imu_bmi088.get_imu_data()  // 读取数据
 *
 */

#ifndef __DM_IMU_HPP__
#define __DM_IMU_HPP__

#include "FreeRTOS.h"  // IWYU pragma: keep
#include "bsp_cfg.hpp" // IWYU pragma: keep
#include "semphr.h"

#include "status.hpp" // 统一状态码

// ---------------- CAN接口参数定义 ----------------

#define ACCEL_CAN_MAX (235.2f)  ///< CAN接口加速度计最大值（单位：m/s²）
#define ACCEL_CAN_MIN (-235.2f) ///< CAN接口加速度计最小值（单位：m/s²）
#define GYRO_CAN_MAX (34.88f)   ///< CAN接口陀螺仪最大值（单位：rad/s）
#define GYRO_CAN_MIN (-34.88f)  ///< CAN接口陀螺仪最小值（单位：rad/s）
#define PITCH_CAN_MAX (90.0f)   ///< CAN接口俯仰角（Pitch）最大值（单位：度）
#define PITCH_CAN_MIN (-90.0f)  ///< CAN接口俯仰角（Pitch）最小值（单位：度）
#define ROLL_CAN_MAX (180.0f)   ///< CAN接口横滚角（Roll）最大值（单位：度）
#define ROLL_CAN_MIN (-180.0f)  ///< CAN接口横滚角（Roll）最小值（单位：度）
#define YAW_CAN_MAX (180.0f)    ///< CAN接口偏航角（Yaw）最大值（单位：度）
#define YAW_CAN_MIN (-180.0f)   ///< CAN接口偏航角（Yaw）最小值（单位：度）
#define TEMP_MIN (0.0f)         ///< 温度传感器最小值（单位：摄氏度）
#define TEMP_MAX (60.0f)        ///< 温度传感器最大值（单位：摄氏度）
#define QUATERNION_MIN (-1.0f)  ///< 四元数最小值
#define QUATERNION_MAX (1.0f)   ///< 四元数最大值

#define CMD_READ 0  ///< 读取命令标识符
#define CMD_WRITE 1 ///< 写入命令标识符

/**
 * @brief 通信端口枚举
 */
enum class ImuComPort
{
  COM_USB = 0, ///< USB通信端口
  COM_RS485,   ///< RS485通信端口
  COM_CAN,     ///< CAN通信端口
  COM_VOFA     ///< VOFA通信端口
};


/**
 * @brief CAN速率枚举
 */
enum class ImuBaudrate
{
  CAN_BAUD_1M = 0, ///< CAN波特率 1Mbps
  CAN_BAUD_500K,   ///< CAN波特率 500kbps
  CAN_BAUD_400K,   ///< CAN波特率 400kbps
  CAN_BAUD_250K,   ///< CAN波特率 250kbps
  CAN_BAUD_200K,   ///< CAN波特率 200kbps
  CAN_BAUD_100K,   ///< CAN波特率 100kbps
  CAN_BAUD_50K,    ///< CAN波特率 50kbps
  CAN_BAUD_25K     ///< CAN波特率 25kbps
};


/**
 * @brief IMU数据结构体
 */
struct ImuData
{
  uint8_t can_id;   ///< CAN ID
  uint8_t mst_id;   ///< 主机ID
  float   pitch;    ///< 俯仰角（度）
  float   roll;     ///< 横滚角（度）
  float   yaw;      ///< 偏航角（度）
  float   q[4];     ///< 四元数
  float   cur_temp; ///< 当前温度（摄氏度）
};


/**
 * @brief DM IMU类
 *
 * @note 使用bsp_can接口封装，支持欧拉角和四元数输出
 */
class DmImu
{
public:
  // ----------------
  // ---------------- 公共接口 ----------------

  /**
   * @brief 获取设备 ID
   * @return uint8_t 设备 ID
   */
  uint8_t get_device_id() const
  {
    return _device_id;
  }

  /**
   * @brief 获取主机 ID
   * @return uint8_t 主机 ID
   */
  uint8_t get_master_id() const
  {
    return _master_id;
  }


  // ----------------
  // ---------------- 构造与析构 ----------------

  /**
   * @brief IMU 配置结构体（可匿名按序传入）
   */
  struct Config
  {
    /**
     * @brief 按序构造配置（参数顺序 = 字段顺序）
     */
    Config(BspCan &can_bus, uint8_t device_id, uint8_t master_id = 0) : can_bus(can_bus),
                                                                        device_id(device_id),
                                                                        master_id(master_id)
    {
    }

    BspCan &can_bus;   ///< CAN 接口引用
    uint8_t device_id; ///< 设备 ID
    uint8_t master_id; ///< 主机 ID
  };

  /**
   * @brief 构造函数
   * @param cfg IMU 配置（CAN 接口/设备ID/主机ID，可匿名按序传入）
   */
  DmImu(const Config &cfg);

  /**
   * @brief 析构函数
   */
  ~DmImu();


  // ----------------
  // ---------------- 公共接口 ----------------

  /**
   * @brief 初始化IMU（创建数据互斥锁）
   * @return Status OK=成功，IO_ERROR=互斥锁创建失败
   */
  Status init();

  /**
   * @brief 重启IMU
   */
  void reboot();

  /**
   * @brief 加速度计校准
   */
  void accel_calibration();

  /**
   * @brief 陀螺仪校准
   */
  void gyro_calibration();

  /**
   * @brief 更改通信端口
   * @param port 通信端口
   */
  void change_com_port(ImuComPort port);

  /**
   * @brief 设置主动模式延时
   * @param delay 延时时间
   */
  void set_active_mode_delay(uint32_t delay);

  /**
   * @brief 切换到主动模式
   */
  void change_to_active();

  /**
   * @brief 切换到请求模式
   */
  void change_to_request();

  /**
   * @brief 设置波特率
   * @param baud 波特率
   */
  void set_baud(ImuBaudrate baud);

  /**
   * @brief 设置CAN ID
   * @param can_id CAN ID
   */
  void set_can_id(uint8_t can_id);

  /**
   * @brief 设置主机ID
   * @param mst_id 主机ID
   */
  void set_mst_id(uint8_t mst_id);

  /**
   * @brief 保存参数
   */
  void save_parameters();

  /**
   * @brief 恢复设置
   */
  void restore_settings();

  /**
   * @brief 请求欧拉角数据
   */
  void request_euler();

  /**
   * @brief 请求四元数数据
   */
  void request_quat();

  /**
   * @brief 获取IMU数据（线程安全）
   * @return ImuData IMU数据
   */
  ImuData get_imu_data();

  /**
   * @brief 设置IMU数据（线程安全）
   * @param data IMU数据
   */
  void set_imu_data(const ImuData &data);

  /**
   * @brief CAN消息回调处理
   * @param rx_msg CAN接收消息
   */
  void on_can_message(const CanRxMsg &rx_msg);


private:
  // ----------------
  // ---------------- 寄存器ID枚举 ----------------

  /**
   * @brief IMU寄存器ID
   */
  enum class RegId
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
  };


  // ----------------
  // ---------------- 私有成员函数 ----------------

  /**
   * @brief 写寄存器
   * @param reg_id 寄存器ID
   * @param data 写入数据
   */
  void _write_register(RegId reg_id, uint32_t data);

  /**
   * @brief 读寄存器
   * @param reg_id 寄存器ID
   */
  void _read_register(RegId reg_id);

  /**
   * @brief 浮点数转整数
   */
  int _float_to_int(float value, float min, float max, int bits);

  /**
   * @brief 整数转浮点数
   */
  float _uint_to_float(int value, float min, float max, int bits);

  /**
   * @brief 更新欧拉角数据
   * @param data 数据数组引用
   */
  void _update_euler(const uint8_t (&data)[8]);

  /**
   * @brief 更新四元数数据
   * @param data 数据数组引用
   */
  void _update_quaternion(const uint8_t (&data)[8]);

  // ----------------
  // ---------------- 私有成员变量 ----------------

  BspCan &_can_bus;   ///< CAN总线接口引用
  ImuData _imu_data;  ///< IMU数据
  uint8_t _device_id; ///< 设备ID
  uint8_t _master_id; ///< 主机ID
  char    _name[32];  ///< 互斥锁名字

  SemaphoreHandle_t _data_mutex_handle; ///< 用于保护_imu_data的互斥锁

  // ----------------
};


#endif // __DM_IMU_HPP__
