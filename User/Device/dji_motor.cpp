#include "dji_motor.hpp"
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <cstdlib>

template<MotorType type>
DjiMotor<type>::DjiMotor(BspCan &can_item,
        uint8_t motor_id,
        float ratio,
        float offset,
        DjiMotorControlMode control_mode) : 
_data(),
_raw_data(),
_param(),
_lvbo_data(),
_can_item(&can_item),
_motor_id(motor_id),
_bus_slot((motor_id - 1U) % 4),
_tx_node(nullptr),
_rx_node(nullptr),
_control_mode(control_mode),
_statu(Status::NOT_INIT),
data_mutex_headler(NULL),
data_mutex_attr{},
_total_ecd(0),
_feedback_ready(false)
{
  if (sysEvent != nullptr && (osEventFlagsGet(sysEvent) & SYS_FLAG_RUNNING_BIT) != 0)  // 只允许在初始化期使用
  {
    return;
  }
  // 参数校验
  if ( motor_id > 8 || motor_id == 0){
    this->_statu = Status::BAD_ARG;
    return;
  }
  else if ( motor_id == 8 && type == Motor6020 )  // 6020无ID8
  {
    this->_statu = Status::BAD_ARG;
    return;
  }
  // 在默认减速比处理前拒绝非有限值，避免将负无穷误当作默认配置。
  if (!std::isfinite(ratio) || !std::isfinite(offset))
  {
    _statu = Status::BAD_ARG;
    return;
  }
  if (ratio <= 0)   // 按照说明书中的减速比配置默认减速比
  {
    switch (type) {
      case Motor2006:{
        ratio = 36;
        break;
      }
      case Motor3508:{
        ratio = 3591.0f/187.0f;
        break;
      }
      case Motor6020:{
        ratio = 1;
        break;
      }
    }
  }

  _data.param.ratio = ratio;
  _data.param.offset = offset;
  _param.ecd_full_range = 8192;
  switch (type) {
    case Motor3508:
      _param.current_limit = 16000;
      snprintf(data_mutex_name, 24, "Mutex_DjiMotor_M3508_%d",motor_id);
      break;
    case Motor2006:
      _param.current_limit = 10000;
      snprintf(data_mutex_name, 24, "Mutex_DjiMotor_M2006_%d",motor_id);
      break;
    case Motor6020:
      _param.current_limit = _control_mode == DjiMotorControlMode::CURRENT ? 16384 : 25000;
      snprintf(data_mutex_name, 24, "Mutex_DjiMotor_GM6020_%d",motor_id);
      break;
  }

  data_mutex_attr.name = data_mutex_name;
}

template <MotorType type>
DjiMotor<type>::~DjiMotor()
{
  // 接收节点持有 this；必须在上下文销毁之前成功注销。
  if (_rx_node != nullptr)
  {
    const Status result = unregist(*_can_item, _rx_node);
    configASSERT(result == Status::OK);
    if (result != Status::OK)
    {
      // 运行期注册表已冻结，继续析构会留下悬空回调。
      std::abort();
    }
    _rx_node = nullptr;
  }
  if (_tx_node != nullptr)
  {
    const Status result = unregist(_tx_node, _bus_slot);
    configASSERT(result == Status::OK);
    if (result != Status::OK)
    {
      SysFlagSet(result);
    }
    _tx_node = nullptr;
  }

  // 调用方须先停止本对象的收发和读取，避免销毁仍被使用的互斥量。
  if (data_mutex_headler != nullptr)
  {
    const osStatus_t result = osMutexDelete(data_mutex_headler);
    configASSERT(result == osOK);
    (void)result;
    data_mutex_headler = nullptr;
  }
  // _online 是成员对象，其析构函数会自动注销在线检查节点。
}

template<MotorType type>
Status DjiMotor<type>::init()
{
  // 系统事件未就绪时保留对象状态，允许初始化流程稍后重试。
  if (sysEvent == nullptr)
  {
    return Status::NOT_INIT;
  }
  if ((osEventFlagsGet(sysEvent) & SYS_FLAG_RUNNING_BIT) != 0)  // 只允许在初始化期使用
  {
    return Status::NOT_SUPPORTED;
  }
  // 初始化期重复调用成功对象，不重复申请资源，也不记录系统错误。
  if (_statu == Status::OK)
  {
    return Status::OK;
  }
  if (_statu != Status::NOT_INIT)
  {
    SysInitError(_statu);
    return _statu;
  }

  // 确定发送can id
  uint32_t tx_can_id = 0x00;
  switch (type) {
    case Motor2006:
    case Motor3508:
    {
      if (_motor_id <= 4 )
      {
        tx_can_id = 0x200;
      }
      else
      {
        tx_can_id = 0x1FF;
      }
      break;
    }
    case Motor6020:
    {
      switch (_control_mode) {
        case DjiMotorControlMode::CURRENT:
        {
          if (_motor_id <= 4)
          {
            tx_can_id = 0x1FE;
          }
          else  
          {
            tx_can_id = 0x2FE;
          }
          break;
        }
        case DjiMotorControlMode::VOLTAGE:
        {
          if (_motor_id <= 4)
          {
            tx_can_id = 0x1FF;
          }
          else  
          {
            tx_can_id = 0x2FF;
          }
          break;
        }
        default:
        {
          _statu = Status::BAD_ARG;
          SysInitError(_statu);
          return _statu;
        }
      }
      break;
    }
    default:
    {
      _statu = Status::BAD_ARG;
      SysInitError(_statu);
      return _statu;
    }
  }

  // 互斥量创建
  data_mutex_headler = osMutexNew(&data_mutex_attr);
  if (data_mutex_headler == NULL)
  {
    _statu = Status::FULL;
    SysInitError(_statu);
    return _statu;
  }

  // 注册
  _tx_node = regist(*_can_item,tx_can_id,4,_bus_slot);
  if ( _tx_node == nullptr )
  {
    osMutexDelete(data_mutex_headler);
    data_mutex_headler = nullptr;
    _statu = Status::FULL;
    SysInitError(_statu);
    return _statu;
  }

  const uint32_t feedback_id = (type == Motor6020 ? 0x204U : 0x200U) + _motor_id;
  _rx_node = regist(*_can_item, feedback_id, &DjiMotor<type>::rx_callback, this);
  if (_rx_node == nullptr)
  {
    // 注册器已记录具体冲突/资源错误；撤销本次初始化取得的发送槽位和锁。
    const Status result = unregist(_tx_node, _bus_slot);
    configASSERT(result == Status::OK);
    (void)result;
    _tx_node = nullptr;
    osMutexDelete(data_mutex_headler);
    data_mutex_headler = nullptr;
    _statu = Status::IO_ERROR;
    return _statu;
  }

  _statu = Status::OK;
  return _statu;
}

template <MotorType type>
Status DjiMotor<type>::rx_callback(void *context, const CanRxMsg &rx)
{
  return static_cast<DjiMotor<type> *>(context)->DataUnpack(rx);
}

template <MotorType type>
Status DjiMotor<type>::DataUnpack(const CanRxMsg &rx)
{
  if (_tx_node == nullptr || data_mutex_headler == nullptr)
  {
    return Status::NOT_INIT;
  }
  if (rx.header.Identifier != ((type == Motor6020 ? 0x204U : 0x200U) + _motor_id) || rx.header.IdType != FDCAN_STANDARD_ID || rx.header.RxFrameType != FDCAN_DATA_FRAME || rx.header.FDFormat != FDCAN_CLASSIC_CAN || rx.header.DataLength != FDCAN_DLC_BYTES_8)
  {
    return Status::BAD_ARG;
  }

  const uint16_t ecd = (static_cast<uint16_t>(rx.data[0]) << 8) | rx.data[1];
  if (ecd >= _param.ecd_full_range)
  {
    return Status::BAD_ARG;
  }
  // 接收队列在任务上下文中解析；不等待锁，避免阻塞 CAN 反馈处理。
  if (osMutexAcquire(data_mutex_headler, 0U) != osOK)
  {
    return Status::BUSY;
  }

  _raw_data.rpm              = static_cast<int16_t>((rx.data[2] << 8) | rx.data[3]);
  _raw_data.torque_current   = static_cast<int16_t>((rx.data[4] << 8) | rx.data[5]);
  _raw_data.temperature      = type == Motor2006 ? 0U : rx.data[6];
  _data.radian_data.velocity = _raw_data.rpm / _data.param.ratio * (PI / 30.0f);

  if (!_feedback_ready)
  {
    _total_ecd = ecd;
  }
  else
  {
    // 前后编码器差值超过半圈表示跨过零点，按方向补偿一圈计数。
    int32_t delta = static_cast<int32_t>(ecd) - _raw_data.ecd;
    if (delta > _param.ecd_full_range / 2)
    {
      delta -= _param.ecd_full_range;
    }
    else if (delta < -static_cast<int32_t>(_param.ecd_full_range / 2))
    {
      delta += _param.ecd_full_range;
    }
    _total_ecd += delta;
  }

  _raw_data.ecd = static_cast<int16_t>(ecd);

  // 保留双精度中间角度，避免多圈累计后单圈取模损失精度。
  const double angle                   = (static_cast<double>(_total_ecd) * (2.0 * PI) / _param.ecd_full_range - _data.param.offset) / _data.param.ratio;
  _data.radian_data.angle_multi_round  = static_cast<float>(angle);
  _data.radian_data.angle_single_round = static_cast<float>(std::fmod(angle, 2.0 * PI));
  if (_data.radian_data.angle_single_round < 0.0f)
  {
    _data.radian_data.angle_single_round += 2.0f * PI;
  }

  _feedback_ready = true;
  _online.refresh_task();
  osMutexRelease(data_mutex_headler);
  return Status::OK;
}

template<MotorType type>
Status DjiMotor<type>::FillData(int16_t output)
{
  if (_statu != Status::OK )
  {
    return _statu;
  }
  if (output > _param.current_limit)
  {
    output = _param.current_limit;
  }
  if (output < -_param.current_limit)
  {
    output = -_param.current_limit;
  }

  uint8_t tx_data[2];
  tx_data[0] = static_cast<uint8_t>(static_cast<uint16_t>(output)>>8);
  tx_data[1] = static_cast<uint8_t>(output);

  return _tx_node->filldata(tx_data, _bus_slot);

}

template <MotorType type>
const MotorData &DjiMotor<type>::data(void) const
{
  return _data;
}

template <MotorType type>
const DjiMotorRawData &DjiMotor<type>::raw_data(void) const
{
  return _raw_data;
}

template <MotorType type>
const DjiMotorParam &DjiMotor<type>::param(void) const
{
  return _param;
}

template <MotorType type>
const Online &DjiMotor<type>::online(void) const
{
  return _online;
}

template <MotorType type>
const LuenbergerMotorData &DjiMotor<type>::lvboData(void) const
{
  return _lvbo_data;
}

template <MotorType type>
Status DjiMotor<type>::statu(void) const
{
  return _statu;
}

// 为支持的电机型号生成已定义成员的符号，供其他编译单元链接。
template class DjiMotor<Motor3508>;
template class DjiMotor<Motor6020>;
template class DjiMotor<Motor2006>;
