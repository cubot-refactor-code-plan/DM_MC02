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
 * @details 使用示例：
 *          - 类的实例化及初始化需要在FreeRTOS内核启动后进行
 *
 * @note 实例化与初始化
 *       DmImu imu_bmi088(bsp_can1, 0x58, 0x59); // 全局实例化类
 *       imu_bmi088.init();                        // 需要在freertos内核开启之后去init
 *
 * @note CAN接收处理
 *       imu_bmi088.on_can_message(rx_msg); // 类似中断回调
 *
 * @note 数据获取
 *       imu_bmi088.change_to_active();                     // 主动发送数据
 *       imu_data user_imu_data = imu_bmi088.get_imu_data() // 读取数据
 *
 */

#ifndef __DM_IMU_HPP__
#define __DM_IMU_HPP__

#include "bsp_cfg.hpp" // IWYU pragma: keep
#include "FreeRTOS.h"
#include "semphr.h"

/* USER CODE BEGIN */

/* ==================== 外部声明 ==================== */

// 前向声明
class DmImu;

extern DmImu imu_bmi088; ///< 全局IMU实例

/* USER CODE END */


/* ==================== CAN接口参数定义 ==================== */

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
#define Quaternion_MIN (-1.0f)  ///< 四元数最小值
#define Quaternion_MAX (1.0f)   ///< 四元数最大值

#define CMD_READ 0  ///< 读取命令标识符
#define CMD_WRITE 1 ///< 写入命令标识符

/**
 * @brief 通信端口枚举
 */
typedef enum imu_com_port_e
{
  COM_USB = 0, ///< USB通信端口
  COM_RS485,   ///< RS485通信端口
  COM_CAN,     ///< CAN通信端口
  COM_VOFA     ///< VOFA通信端口
} imu_com_port_e;


/**
 * @brief CAN速率枚举
 */
typedef enum imu_baudrate_e
{
  CAN_BAUD_1M = 0, ///< CAN波特率 1Mbps
  CAN_BAUD_500K,   ///< CAN波特率 500kbps
  CAN_BAUD_400K,   ///< CAN波特率 400kbps
  CAN_BAUD_250K,   ///< CAN波特率 250kbps
  CAN_BAUD_200K,   ///< CAN波特率 200kbps
  CAN_BAUD_100K,   ///< CAN波特率 100kbps
  CAN_BAUD_50K,    ///< CAN波特率 50kbps
  CAN_BAUD_25K     ///< CAN波特率 25kbps
} imu_baudrate_e;


/**
 * @brief IMU数据结构体
 */
struct imu_data
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
  uint8_t _device_id; ///< 设备ID
  uint8_t _master_id; ///< 主机ID


  /* ==================== 构造与析构 ==================== */

  /**
   * @brief 构造函数
   *
   * @param can_bus bsp_can实例引用
   * @param device_id 设备ID
   * @param master_id 主机ID
   */
  DmImu(BspCan& can_bus, uint8_t device_id, uint8_t master_id = 0);

  /**
   * @brief 析构函数
   */
  ~DmImu();


  /* ==================== 公共接口 ==================== */

  /**
   * @brief 初始化IMU
   */
  void init();

  /**
   * @brief 写寄存器
   * @param reg_id 寄存器ID
   * @param data 写入数据
   */
  void write_register(uint8_t reg_id, uint32_t data);

  /**
   * @brief 读寄存器
   * @param reg_id 寄存器ID
   */
  void read_register(uint8_t reg_id);

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
  void change_com_port(imu_com_port_e port);

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
  void set_baud(imu_baudrate_e baud);

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
   * @brief 尝试接收数据
   * @param timeout_ms 超时时间（毫秒）
   * @return 如果成功接收到数据则返回true
   */
  bool try_receive_data(uint32_t timeout_ms = 100);

  /**
   * @brief 获取IMU数据（线程安全）
   * @return imu_data IMU数据
   */
  imu_data get_imu_data();

  /**
   * @brief 设置IMU数据（线程安全）
   * @param data IMU数据
   */
  void set_imu_data(const imu_data& data);

  /**
   * @brief CAN消息回调处理
   * @param rx_msg CAN接收消息
   */
  void on_can_message(const CanRxMsg& rx_msg);


private:
  /* ==================== 私有成员函数 ==================== */

  /**
   * @brief 浮点数转无符号整数
   */
  int float_to_uint(float x_float, float x_min, float x_max, int bits);

  /**
   * @brief 无符号整数转浮点数
   */
  float uint_to_float(int x_int, float x_min, float x_max, int bits);

  /**
   * @brief 更新欧拉角数据
   * @param data 数据数组引用
   */
  void update_euler(const uint8_t (&data)[8]);

  /**
   * @brief 更新四元数数据
   * @param data 数据数组引用
   */
  void update_quaternion(const uint8_t (&data)[8]);

  /**
   * @brief 处理接收数据
   * @param pData 数据指针
   */
  void process_received_data(uint8_t* pData);


  /* ==================== 私有成员变量 ==================== */

  BspCan&  _can_bus;  ///< CAN总线接口引用
  imu_data _imu_data; ///< IMU数据
  char     name[32];  ///< 互斥锁名字

  SemaphoreHandle_t _data_mutex_handle; ///< 用于保护_imu_data的互斥锁
};


#endif // __DM_IMU_HPP__
