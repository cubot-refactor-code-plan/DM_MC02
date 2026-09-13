/**
 * @file dji_motor.hpp
 * @author ChoseB
 * @brief DJI 电机反馈解析、在线状态和控制帧聚合框架
 * @version 0.1
 * @date 2026-09-01
 *
 * @details 一个 DjiMotor 实例表示一台电机。用户只需提供所用 CAN 和电机 ID；
 *          类内部根据电机型号计算控制帧 ID 与帧内槽位，并从固定池中复用或启用
 *          对应的 DjiMotorBus。Bus 以 (BspCan*, std_id) 为唯一键，因此采用相同
 *          控制帧协议的不同型号电机可以共享一帧发送。
 *
 * @note 构造阶段只登记软件对象关系，不初始化或访问 CAN 硬件。
 * @warning FillData() 与 MotorTxManager::update() 若运行在不同任务或中断上下文，实现时
 *          必须保护共享输出槽位，避免 C++ 数据竞争。
 * @note 协议映射、反馈解析、固定池管理和发送逻辑实现在 dji_motor.cpp。
 */
#ifndef __DJI_MOTOR_HPP__
#define __DJI_MOTOR_HPP__

#include "bsp_can.hpp"
#include "motor_definition.hpp"
#include "motor_tx_manager.hpp"
#include "online_check.hpp"

#include <stdint.h>

/** @todo 将以下临时协议常量迁移到按 MotorType 特化的 traits，随后删除宏。 */
#define K_ECD_TO_ANGLE 0.043945f       ///< 编码器计数到机械角度的临时换算系数，单位：度/count
#define ECD_RANGE_FOR_3508 8191        ///< M3508 编码器原始值上限（包含 0，共 8192 个计数）
#define CURRENT_LIMIT_FOR_3508 16000   ///< M3508/C620 软件限制使用的原始电流指令绝对值
#define ECD_RANGE_FOR_6020 8191        ///< GM6020 编码器原始值上限（包含 0，共 8192 个计数）
#define CURRENT_LIMIT_FOR_6020 16384   ///< GM6020 电流模式原始控制指令绝对值上限
#define VOLTAGE_LIMIT_FOR_6020 25000   ///< GM6020 电压模式原始控制指令绝对值上限
#define ECD_RANGE_FOR_2006 8191        ///< M2006 编码器原始值上限（包含 0，共 8192 个计数）
#define CURRENT_LIMIT_FOR_2006 10000   ///< M2006/C610 软件限制使用的原始电流指令绝对值

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
 * @brief 发送固定池中所有已启用的 DJI 电机控制帧
 *
 * @return Status::OK 所有控制帧均成功交给 CAN BSP；其他状态表示至少一帧发送失败。
 *
 * @note 保留本函数用于兼容直接发送；正常周期发送由 MotorTxManager::update() 统一完成。
 * @note 未占用的帧槽位必须保持为零，防止向不存在的电机输出指令。
 */
Status DjiMotorSendAll(void);

/**
 * @brief 判断指定 CAN 是否注册了 DJI 电机
 * @param can_item 待查询的 CAN BSP 对象
 * @return true 至少有一个电机使用该 CAN；false 没有注册电机。
 * @note 本函数只查询注册关系，不访问 CAN 硬件。
 */
bool dji_motor_uses_can(const BspCan &can_item);

/**
 * @brief 将一帧 CAN 数据分发给匹配的 DJI 电机对象
 * @param can_item 接收到该帧的 CAN BSP 对象
 * @param rx       待分发的 CAN 帧
 * @return Status::OK 找到目标电机并成功解析；Status::BAD_ARG 帧不属于已注册电机。
 * @note 调用方应先从对应 CAN 接收队列取出完整帧，再调用本函数。
 */
Status dji_motor_dispatch_rx(BspCan &can_item, const CanRxMsg &rx);

/**
 * @brief 检查指定 CAN 上的反馈标识符是否已被任意 DJI 电机对象占用
 * @param can_item        待查询的 CAN BSP 对象
 * @param feedback_std_id 待查询的反馈帧标准标识符
 * @return true 标识符已被占用；false 标识符尚未被占用。
 * @note 供 DjiMotor 构造注册时进行跨型号冲突检查，业务代码通常无需调用。
 */
bool dji_motor_feedback_in_use(const BspCan &can_item, uint32_t feedback_std_id);

/**
 * @brief 单个 DJI 电机设备对象
 *
 * @tparam type 电机型号，决定合法 ID、反馈解析、控制帧映射、限幅和默认减速比
 *
 * @details 对象负责保存单电机反馈和目标输出。多个电机的目标输出由内部
 *          DjiMotorBus 聚合后注册到 MotorTxManager 周期发送。
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

  BspCan            *_can_item;     ///< 接收反馈所用的物理 CAN
  uint8_t            _motor_id;     ///< 电调配置的 DJI 协议 ID
  uint8_t            _bus_slot;     ///< 当前电机在控制帧中的槽位，范围 0~3
  DjiMotorControlMode _control_mode; ///< GM6020 控制模式；其他型号忽略
  Status             _statu;        ///< 构造注册或最近一次公开操作的状态

  int16_t _last_ecd;       ///< 上一次反馈的编码器原始值，用于检测跨零
  float   _total_angle;    ///< 已完成跨零展开的电机转子累计角度，单位：rad
  float   _last_velocity;  ///< 上一次输出轴角速度，用于计算角加速度
  bool    _feedback_ready; ///< 是否已接收过至少一帧有效反馈

  DjiMotor *_next;        ///< 同型号电机分发链表的后继节点
  static DjiMotor *_head; ///< 同型号电机分发链表头
  static DjiMotor *_tail; ///< 同型号电机分发链表尾

public:
  /**
   * @brief 获取只读电机数据
   * @return 内部 MotorData 的常量引用。
   * @warning 返回的是实时引用；若反馈更新与读取不在同一任务，实现时需提供同步或快照机制。
   */
  const MotorData &data(void) const;

  /**
   * @brief 获取 DJI 电调反馈协议原始数据
   */
  const DjiMotorRawData &raw_data(void) const;

  /**
   * @brief 获取 DJI 编码器和原始控制限幅参数
   */
  const DjiMotorParam &param(void) const;

  /**
   * @brief 获取本电机的在线检查对象
   * @return Online 的常量引用，可用于调用 isOnline() 查询状态。
   */
  const Online &online(void) const;

  /**
   * @brief 获取可选的 Luenberger 观测数据
   * @return 内部 LuenbergerMotorData 的常量引用。
   * @note 观测器未启用时，该数据不应作为有效控制依据。
   */
  const LuenbergerMotorData &lvboData(void) const;

  /**
   * @brief 查询对象最近一次初始化或操作状态
   * @return Status::OK 表示内部 Bus 注册成功且最近操作成功；其他值表示具体错误。
   */
  Status statu(void) const;

  /**
   * @brief 构造单个 DJI 电机对象并注册控制帧槽位
   *
   * @param can_item   电机连接的物理 CAN BSP 对象
   * @param motor_id   电调设置的 DJI 协议 ID
   * @param ratio      电机转子到最终机构输出轴的减速比；传 0 时按型号选择默认值：
   *                  M3508 为 3591/187，GM6020 为 1，M2006 为 36
   * @param offset     电机转子机械零位偏移，单位：rad
   * @param control_mode GM6020 的控制模式；其他电机型号忽略该参数
   *
   * @note 构造函数不能返回错误；ID 非法、槽位冲突或池耗尽可通过 statu() 查询。
   * @note 构造阶段不会调用 BspCan::init() 或发送 CAN 帧。
   */
  DjiMotor(BspCan &can_item,
           uint8_t motor_id,
           float ratio = 0,
           float offset = 0.0f,
           DjiMotorControlMode control_mode = DjiMotorControlMode::VOLTAGE);

  /**
   * @brief 注销控制帧槽位并销毁电机对象
   * @note 注销时应先将输出清零，避免回收后保留旧控制量。
   */
  ~DjiMotor();

  /**
   * @brief 解析一帧属于本电机的 DJI CAN 反馈
   *
   * @param rx 待解析的 CAN 标准帧
   * @return Status::OK 反馈有效且数据已更新；Status::BAD_ARG 标识符或帧格式不匹配；
   *         Status::NOT_INIT 电机未成功完成内部注册。
   *
   * @note 只有完整且有效的反馈才能刷新 _online。
   */
  Status DataUnpack(CanRxMsg rx);

  /**
   * @brief 设置下一周期发送的原始控制量
   *
   * @param output 有符号原始控制指令；实现时应按具体型号的限制进行饱和限幅
   * @return Status::OK 写入成功；Status::NOT_INIT 未取得内部 Bus；
   *         Status::BAD_ARG 或其他状态表示参数/Bus 操作失败。
   *
   * @note 本函数只更新内部 Bus 槽位，实际发送由 MotorTxManager::update() 统一完成。
   */
  Status FillData(int16_t output);

  // 禁止复制/移动
  DjiMotor(const DjiMotor &) = delete;
  DjiMotor &operator=(const DjiMotor &) = delete;
  DjiMotor(DjiMotor &&) = delete;
  DjiMotor &operator=(DjiMotor &&) = delete;

private:

};

#endif // __DJI_MOTOR_HPP__
