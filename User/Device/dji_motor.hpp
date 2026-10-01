/**
 * @file dji_motor.hpp
 * @author ChoseB
 * @brief DJI 电机反馈解析、在线状态和控制帧聚合框架
 * @version 0.1
 * @date 2026-09-01
 *
 * @details init() 将控制帧槽位注册到 CanTxNode，将反馈 ID 注册到 CanRxNode。
 *          CanBus 的分发任务自动调用反馈解包，并在成功后刷新在线状态。
 * @note 构造阶段仅校验参数；成功初始化的对象必须存活至系统停止。
 */
#ifndef __DJI_MOTOR_HPP__
#define __DJI_MOTOR_HPP__

#include "can_bus.hpp"
#include "motor_definition.hpp"
#include "online_check.hpp"

#include <stdint.h>

/** @todo 将以下临时协议常量迁移到按 MotorType 特化的 traits，随后删除宏。 */
// #define K_ECD_TO_ANGLE 0.043945f       ///< 编码器计数到机械角度的临时换算系数，单位：度/count
// #define ECD_RANGE_FOR_3508 8191        ///< M3508 编码器原始值上限（包含 0，共 8192 个计数）
// #define CURRENT_LIMIT_FOR_3508 16000   ///< M3508/C620 软件限制使用的原始电流指令绝对值
// #define ECD_RANGE_FOR_6020 8191        ///< GM6020 编码器原始值上限（包含 0，共 8192 个计数）
// #define CURRENT_LIMIT_FOR_6020 16384   ///< GM6020 电流模式原始控制指令绝对值上限
// #define VOLTAGE_LIMIT_FOR_6020 25000   ///< GM6020 电压模式原始控制指令绝对值上限
// #define ECD_RANGE_FOR_2006 8191        ///< M2006 编码器原始值上限（包含 0，共 8192 个计数）
// #define CURRENT_LIMIT_FOR_2006 10000   ///< M2006/C610 软件限制使用的原始电流指令绝对值

/**
 * @brief 框架支持的 DJI 电机型号
 *
 * @note 电机型号用于选择反馈格式、合法 ID 范围、控制帧 ID、槽位和默认减速比。
 */
enum MotorType
{
  Motor3508 = 0x00U, ///< M3508 电机配合 C620 电调
  Motor6020 = 0x01U, ///< GM6020 电机
  Motor2006 = 0x02U, ///< M2006 电机配合 C610 电调
};

/**
 * @brief GM6020 控制帧模式
 *
 * @note M3508/C620 和 M2006/C610 只有固定的电流控制协议，此参数对它们无效。
 */
enum class DjiMotorControlMode : uint8_t
{
  VOLTAGE = 0, ///< GM6020 电压控制：0x1FF/0x2FF
  CURRENT,     ///< GM6020 电流控制：0x1FE/0x2FE，需在电机固件中启用电流环
};

/**
 * @brief DJI 电调反馈协议特有的原始数据
 */
struct DjiMotorRawData
{
  int16_t ecd;            ///< 编码器原始位置计数
  int16_t rpm;            ///< 电调反馈的有符号转速，单位：rpm
  int16_t torque_current; ///< 电调反馈的有符号原始力矩电流
  uint8_t temperature;    ///< 电调反馈温度，单位：摄氏度；无温度反馈的型号保持为 0
};

/**
 * @brief DJI 协议解析及控制限幅参数
 */
struct DjiMotorParam
{
  uint16_t ecd_full_range; ///< 编码器一圈的计数总数，例如 8192
  uint16_t current_limit;  ///< 原始控制指令允许的绝对值上限
};

template <MotorType type>
class DjiMotor;

/**
 * @brief 单个 DJI 电机设备对象
 *
 * @tparam type 电机型号，决定合法 ID、反馈解析、控制帧映射、限幅和默认减速比
 *
 * @details CanTxNode 聚合发送槽位，CanRxNode 按总线和反馈 ID 分发接收帧。
 * @note 对象不可复制或移动，以保证在线检查节点和控制帧槽位的身份稳定。
 */
template <MotorType type>
class DjiMotor
{
private:
  MotorData           _data;        ///< 最终机构输出轴的通用运动学数据
  DjiMotorRawData     _raw_data;    ///< DJI 电调反馈协议原始数据
  DjiMotorParam       _param;       ///< DJI 编码器和原始控制限幅参数
  Online              _online;      ///< 由有效反馈帧刷新的在线检查对象
  LuenbergerMotorData _lvbo_data;   ///< 可选的 Luenberger 观测结果

  CanBus            *_can_item;     ///< 接收反馈、下发控制帧所用的 CAN 总线
  uint8_t            _motor_id;     ///< 电调配置的 DJI 协议 ID
  uint8_t            _bus_slot;     ///< 当前电机在控制帧中的槽位，范围 0~3
  CanTxNode       *_tx_node;
  CanRxNode       *_rx_node;
  static Status _rx_callback(void *context, const CanRxMsg &rx);
  DjiMotorControlMode _control_mode; ///< GM6020 控制模式；其他型号忽略
  Status             _statu;        ///< 构造校验或初始化状态，运行期收发结果由函数返回

  SemaphoreHandle_t data_mutex_headler; ///< 保护解析结果与原始数据的互斥量

  int64_t _total_ecd;      ///< 跨零展开后的累计转子编码器计数
  bool    _feedback_ready; ///< 是否已接收过至少一帧有效反馈


public:
  /**
   * @brief 获取只读电机数据
   * @return 内部 MotorData 的常量引用。
   * @warning 返回实时引用而非快照；调用方须保证读取期间不并发执行 data_unpack()。
   */
  const MotorData &data(void) const;

  /**
   * @brief 获取 DJI 电调反馈协议原始数据
   * @return 内部 DjiMotorRawData 的常量引用。
   * @warning 返回实时引用而非快照；调用方须保证读取期间不并发执行 data_unpack()。
   */
  const DjiMotorRawData &raw_data(void) const;

  /**
   * @brief 获取 DJI 编码器和原始控制限幅参数
   * @return 内部 DjiMotorParam 的常量引用，构造完成后不再修改。
   */
  const DjiMotorParam &param(void) const;

  /**
   * @brief 获取本电机的在线检查对象
   * @return Online 的常量引用，可用于调用 is_online() 查询状态。
   */
  const Online &online(void) const;

  /**
   * @brief 获取可选的 Luenberger 观测数据
   * @return 内部 LuenbergerMotorData 的常量引用。
   * @note 观测器未启用时，该数据不应作为有效控制依据。
   */
  const LuenbergerMotorData &lvboData(void) const;

  /**
   * @brief 查询对象构造校验或初始化状态
   * @return Status::OK 表示初始化成功；其他值表示未初始化或初始化错误。
   * @note 收发操作结果由 data_unpack() 和 fill_data() 的返回值提供，不更新此状态。
   * @warning 不得与 init() 并发调用。
   */
  Status statu(void) const;

  /**
   * @brief 电机配置结构体（可匿名按序传入）
   */
  struct Config
  {
    /**
     * @brief 按序构造配置（参数顺序 = 字段顺序）
     *
     * @param can          电机连接的物理 CAN 总线
     * @param motor_id     电调设置的 DJI 协议 ID
     * @param ratio        电机转子到最终机构输出轴的减速比；传 0 时按型号选择默认值：
     *                     M3508 为 3591/187，GM6020 为 1，M2006 为 36
     * @param offset       电机转子机械零位偏移，单位：rad
     * @param control_mode GM6020 的控制模式；其他电机型号忽略该参数
     */
    Config(CanBus        &can,
           uint8_t        motor_id,
           float          ratio = 0.0f,
           float          offset = 0.0f,
           DjiMotorControlMode control_mode = DjiMotorControlMode::VOLTAGE)
      : can(can),
        motor_id(motor_id),
        ratio(ratio),
        offset(offset),
        control_mode(control_mode)
    {
    }

    CanBus              &can;          ///< 电机连接的物理 CAN 总线
    uint8_t              motor_id;     ///< 电调设置的 DJI 协议 ID
    float                ratio;        ///< 减速比；0 = 按型号取默认值
    float                offset;       ///< 机械零位偏移，单位：rad
    DjiMotorControlMode  control_mode; ///< GM6020 控制模式；其他型号忽略
  };

  /**
   * @brief 构造单个 DJI 电机对象并校验参数
   *
   * @param cfg 电机配置（总线 + ID + 减速比 + 零位 + 控制模式，可匿名按序传入）
   *
   * @note 构造后调用 init() 注册收发节点；错误可通过 statu() 查询。
   * @note 构造阶段不会调用 CanBus / BspCan 的 init()，也不发送 CAN 帧。
   */
  DjiMotor(const Config &cfg);

  /**
   * @brief 初始化阶段注销收发节点并销毁电机对象。
   * @note 接收节点注销成功后，释放发送槽位和互斥量。
   * @warning 成功初始化后不支持运行期销毁；接收注册表冻结后析构会触发断言/终止。
   */
  ~DjiMotor();

  /** @brief 系统运行前创建互斥量并注册发送槽位、标准帧反馈节点；重复成功初始化无副作用。 */
  Status init(void);

  /**
   * @brief 解析一帧属于本电机的 DJI CAN 反馈
   *
   * @param rx 待解析的 CAN 标准帧
   * @return Status::OK 反馈有效且数据已更新；Status::BAD_ARG 标识符或帧格式不匹配；
   *         Status::NOT_INIT 电机未成功完成内部注册。
   *
   * @note 由 CanRxNode 在接收任务中调用；失败帧进入 BSP 回退缓冲。
   *       互斥锁不可用时返回 Status::BUSY，不更新反馈。
   * @note 只有完整且有效的反馈才能刷新 _online。相邻反馈转子位移必须小于半圈。
   * @note 首帧以绝对编码器位置初始化多圈角度；单圈角度为输出轴角度对 2π 取模。
   * @note 不计算角加速度；通用数据中的 acceleration 保持初始零值，不代表实际角加速度。
   */
  Status data_unpack(const CanRxMsg &rx);

  /**
   * @brief 设置下一周期发送的原始控制量
   *
   * @param output 有符号原始控制指令；实现时应按具体型号的限制进行饱和限幅
   * @return Status::OK 写入成功；Status::NOT_INIT 未成功初始化；
   *         Status::BAD_ARG 或其他状态表示参数或发送槽位操作失败。
   *
   * @note 本函数只更新发送槽位，实际发送由 can_tx_task（Service 层）统一完成。
   */
  Status fill_data(int16_t output);

  // 禁止复制/移动
  DjiMotor(const DjiMotor &) = delete;
  DjiMotor &operator=(const DjiMotor &) = delete;
  DjiMotor(DjiMotor &&) = delete;
  DjiMotor &operator=(DjiMotor &&) = delete;

private:

};

#endif // __DJI_MOTOR_HPP__
