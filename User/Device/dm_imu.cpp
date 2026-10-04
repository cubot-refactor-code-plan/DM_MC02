#include "FreeRTOS.h" // IWYU pragma: keep (临界区)
#include "dm_imu.hpp"
#include "task.h" // IWYU pragma: keep (taskENTER_CRITICAL / taskEXIT_CRITICAL)

#include <cstdlib>  // std::abort
#include <string.h> // memcpy

namespace
{
/**
 * @brief 映射值 → 浮点（手册：映射值小端序，v * span / (2^bits - 1) + min）
 *
 * @note 映射值是**无符号**的：零位在中间，正角度落在高半区
 *       （如 +0.4° ↔ 0x804B）。若按有符号解释，正值会整体偏 -360°。
 */
float uint_to_float(uint16_t value, float min, float max, int bits)
{
  const float span = max - min;
  return static_cast<float>(value) * span / static_cast<float>((1U << bits) - 1U) + min;
}
} // namespace


// ---------------- 构造与析构 ----------------


/** @brief 只保存配置；节点注册与主动模式在 init() 里做 */
DmImu::DmImu(const Config &cfg)

  : _can(cfg.can),
    _device_id(cfg.device_id),
    _master_id(cfg.master_id),
    _device_id_alt(cfg.device_id_alt),
    _master_id_alt(cfg.master_id_alt),
    _active_delay_ms(cfg.active_delay_ms),
    _save_params(cfg.save_params),
    _rx_node(nullptr),
    _rx_node_alt(nullptr),
    _online(cfg.timeout_ms),
    _imu_data(),
    _statu(Status::NOT_INIT),
    _data_frames(0U),
    _ack_id(0xFFU),
    _ack_reg(0xFFU),
    _ack_code(0xFFU)
{
  _imu_data.can_id = _device_id;
  _imu_data.mst_id = _master_id;
}

/** @brief 注销接收节点（主 + 备）；注册表冻结后无法注销，此时只能上报 */
DmImu::~DmImu()
{
  if (_rx_node != nullptr)
  {
    const Status result = unregist(_can, _rx_node);
    configASSERT(result == Status::OK);
    if (result != Status::OK)
    {
      // 注册表已冻结，继续销毁会留下悬空回调。
      std::abort();
    }
    _rx_node = nullptr;
  }

  if (_rx_node_alt != nullptr)
  {
    const Status result = unregist(_can, _rx_node_alt);
    configASSERT(result == Status::OK);
    if (result != Status::OK)
    {
      std::abort();
    }
    _rx_node_alt = nullptr;
  }
}


// ----------------
// ---------------- 公共接口 ----------------


/**
 * @brief 注册接收节点（MST_ID）并打开主动模式
 *
 * @note 注册失败时不发任何指令：节点没挂上，发出去也收不回来。
 */
Status DmImu::init()
{
  // 幂等：成功过就直接返回
  if (_statu == Status::OK)
    return Status::OK;

  // 主接收节点：模块发出的数据帧用 MST_ID（手册：应答帧 ID = 上位机设置的 MST_ID）
  _rx_node = regist({&_can, _master_id, &DmImu::_rx_callback, this});
  if (_rx_node == nullptr)
  {
    // 具体原因（重复 ID / 节点超限 / 注册表已冻结）由注册器记录到系统状态
    _statu = Status::FULL;
    sys_init_error(_statu);
    return _statu;
  }

  // 备用接收节点：模块真实 ID 不确定时，把另一组候选也收进来
  if (_master_id_alt != 0U)
  {
    _rx_node_alt = regist({&_can, _master_id_alt, &DmImu::_rx_callback, this});
    if (_rx_node_alt == nullptr)
    {
      _statu = Status::FULL;
      sys_init_error(_statu);
      return _statu;
    }
  }

  // 先设间隔再开主动模式，避免以模块旧间隔先跑一段
  if (_active_delay_ms != 0U)
  {
    (void)set_active_mode_delay(_active_delay_ms);
  }

  Status result = change_to_active();
  if (result != Status::OK)
  {
    _statu = result;
    sys_init_error(_statu);
    return _statu;
  }

  // 写进模块内部 flash：掉电后配置仍在。模块 flash 有擦写寿命，
  // 所以只在配置期打开（Config::save_params），不要每次上电都写。
  if (_save_params)
  {
    result = save_parameters();
    if (result != Status::OK)
    {
      _statu = result;
      sys_init_error(_statu);
      return _statu;
    }

    // 保存后必须重启才真正生效（手册注：重启指令通常没有应答帧，故不看返回值）。
    // 重启期间模块离线 1~3 s，sys_task 的在线检查已相应延后。
    (void)reboot();
  }

  _statu = Status::OK;
  return _statu;
}

/** @brief 帧校验 → 分发解析；应答帧只记录，数据帧刷新在线 */
Status DmImu::data_unpack(const CanRxMsg &rx)
{
  if (_rx_node == nullptr)
    return Status::NOT_INIT;

  // 主 / 备 ID 都接受（两组候选各注册了一个接收节点）
  const bool id_ok = (rx.header.Identifier == _master_id) || ((_master_id_alt != 0U) && (rx.header.Identifier == _master_id_alt));

  // 与 DjiMotor 一致：ID、帧类型、格式、长度全部核对（本类只收标准经典 8 字节帧）
  if (!id_ok || rx.header.IdType != FDCAN_STANDARD_ID || rx.header.RxFrameType != FDCAN_DATA_FRAME || rx.header.FDFormat != FDCAN_CLASSIC_CAN || rx.header.DataLength != FDCAN_DLC_BYTES_8)
  {
    return Status::BAD_ARG;
  }

  // 应答帧：CC RID DD 应答码 …（只记录，不刷新在线）
  if (rx.data[0] == 0xCCU)
  {
    _ack_id   = static_cast<uint8_t>(rx.header.Identifier);
    _ack_reg  = rx.data[1];
    _ack_code = rx.data[3];
    return Status::OK;
  }

  switch (rx.data[0])
  {
    case 0x01U: // 加速度 + 温度
    {
      _update_accel(rx.data);
      break;
    }
    case 0x02U: // 角速度
    {
      _update_gyro(rx.data);
      break;
    }
    case 0x03U: // 欧拉角
    {
      _update_euler(rx.data);
      break;
    }
    case 0x04U: // 四元数
    {
      _update_quaternion(rx.data);
      break;
    }
    default: // 01~04 以外的未知类型：交回退缓冲，便于上层观察
    {
      return Status::BAD_ARG;
    }
  }

  _online.refresh_task();
  ++_data_frames;
  return Status::OK;
}


// ----------------
// ---------------- 寄存器指令（一次性单发） ----------------


/** @brief 重启模块 */
Status DmImu::reboot()
{
  return _write_register(RegId::REBOOT, 0U);
}

/** @brief 启动加计六面校准 */
Status DmImu::accel_calibration()
{
  return _write_register(RegId::ACCEL_CALI, 0U);
}

/** @brief 启动陀螺静态校准 */
Status DmImu::gyro_calibration()
{
  return _write_register(RegId::GYRO_CALI, 0U);
}

/** @brief 切换通信端口 */
Status DmImu::change_com_port(ImuComPort port)
{
  return _write_register(RegId::CHANGE_COM, static_cast<uint8_t>(port));
}

/** @brief 设置主动模式发送间隔（ms） */
Status DmImu::set_active_mode_delay(uint32_t delay_ms)
{
  return _write_register(RegId::SET_DELAY, delay_ms);
}

/** @brief 打开主动模式 */
Status DmImu::change_to_active()
{
  return _write_register(RegId::CHANGE_ACTIVE, 1U);
}

/** @brief 切回应答模式 */
Status DmImu::change_to_request()
{
  return _write_register(RegId::CHANGE_ACTIVE, 0U);
}

/** @brief 设置 CAN 波特率 */
Status DmImu::set_baud(ImuBaudrate baud)
{
  return _write_register(RegId::SET_BAUD, static_cast<uint8_t>(baud));
}

/** @brief 设置 CAN_ID */
Status DmImu::set_can_id(uint8_t can_id)
{
  return _write_register(RegId::SET_CAN_ID, can_id);
}

/** @brief 设置 MST_ID */
Status DmImu::set_mst_id(uint8_t mst_id)
{
  return _write_register(RegId::SET_MST_ID, mst_id);
}

/** @brief 保存参数 */
Status DmImu::save_parameters()
{
  return _write_register(RegId::SAVE_PARAM, 0U);
}

/** @brief 恢复出厂设置 */
Status DmImu::restore_settings()
{
  return _write_register(RegId::RESTORE_SETTING, 0U);
}

/** @brief 请求欧拉角数据 */
Status DmImu::request_euler()
{
  return _read_register(RegId::EULER_DATA);
}

/** @brief 请求四元数数据 */
Status DmImu::request_quat()
{
  return _read_register(RegId::QUAT_DATA);
}

/** @brief 向任意 ID 发一帧读请求（探测模块真实 CAN_ID 用，不影响本对象配置） */
Status DmImu::probe_read(uint32_t can_id)
{
  const uint8_t buf[8] = {0xCCU, static_cast<uint8_t>(RegId::EULER_DATA), CMD_READ, 0xDD, 0U, 0U, 0U, 0U};

  return _can.send(can_id, buf);
}


// ----------------
// ---------------- 查询 ----------------


/** @brief 取数据快照（临界区保护） */
ImuData DmImu::get_imu_data()
{
  ImuData snapshot = {};

  taskENTER_CRITICAL();
  snapshot = _imu_data;
  taskEXIT_CRITICAL();

  return snapshot;
}

/** @brief 构造校验 / 初始化状态 */
Status DmImu::statu() const
{
  return _statu;
}

/** @brief 在线检查对象 */
const Online &DmImu::online() const
{
  return _online;
}


// ----------------
// ---------------- 私有方法 ----------------


/**
 * @brief 向主 / 备 CAN_ID 各发一帧
 *
 * @note 模块真实 CAN_ID 不确定时（例如两组候选），两条都发：寄存器写是幂等的，
 *       同一指令收到两次结果相同，没有副作用。
 */
Status DmImu::_send(const uint8_t *buf)
{
  Status result = _can.send(_device_id, buf);

  if (_device_id_alt != 0U)
  {
    const Status alt = _can.send(_device_id_alt, buf);
    if (result != Status::OK)
      result = alt; // 主 ID 没成功时看备用 ID
  }

  return result;
}

/**
 * @brief 写寄存器：`CC RID 01 DD + 4 字节数据`（小端）
 *
 * @note 单发（不注册 CanTxNode），避免被保底心跳周期性重发。
 */
Status DmImu::_write_register(RegId reg_id, uint32_t data)
{
  uint8_t buf[8] = {0xCCU, static_cast<uint8_t>(reg_id), CMD_WRITE, 0xDD, 0U, 0U, 0U, 0U};
  memcpy(buf + 4, &data, sizeof(data)); // Cortex-M 为小端，与模块一致

  return _send(buf);
}

/** @brief 读寄存器：`CC RID 00 DD + 4 字节 0` */
Status DmImu::_read_register(RegId reg_id)
{
  const uint8_t buf[8] = {0xCCU, static_cast<uint8_t>(reg_id), CMD_READ, 0xDD, 0U, 0U, 0U, 0U};

  return _send(buf);
}

/** @brief 加速度帧：data[1]=温度（8 位映射），data[2..3]/[4..5]/[6..7]=Acc X/Y/Z（小端 16 位映射） */
void DmImu::_update_accel(const uint8_t (&data)[8])
{
  const uint16_t ax = static_cast<uint16_t>((data[3] << 8) | data[2]);
  const uint16_t ay = static_cast<uint16_t>((data[5] << 8) | data[4]);
  const uint16_t az = static_cast<uint16_t>((data[7] << 8) | data[6]);

  // 浮点换算放在临界区外，临界区里只做赋值
  const float fx   = uint_to_float(ax, ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
  const float fy   = uint_to_float(ay, ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
  const float fz   = uint_to_float(az, ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
  const float temp = uint_to_float(data[1], TEMP_MIN, TEMP_MAX, 8);

  taskENTER_CRITICAL();
  _imu_data.accel[0] = fx;
  _imu_data.accel[1] = fy;
  _imu_data.accel[2] = fz;
  _imu_data.cur_temp = temp;
  taskEXIT_CRITICAL();
}

/** @brief 角速度帧：data[2..3]/[4..5]/[6..7]=Gyro X/Y/Z（小端 16 位映射） */
void DmImu::_update_gyro(const uint8_t (&data)[8])
{
  const uint16_t gx = static_cast<uint16_t>((data[3] << 8) | data[2]);
  const uint16_t gy = static_cast<uint16_t>((data[5] << 8) | data[4]);
  const uint16_t gz = static_cast<uint16_t>((data[7] << 8) | data[6]);

  const float fx = uint_to_float(gx, GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
  const float fy = uint_to_float(gy, GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
  const float fz = uint_to_float(gz, GYRO_CAN_MIN, GYRO_CAN_MAX, 16);

  taskENTER_CRITICAL();
  _imu_data.gyro[0] = fx;
  _imu_data.gyro[1] = fy;
  _imu_data.gyro[2] = fz;
  taskEXIT_CRITICAL();
}

/** @brief 欧拉角帧：data[2..3]=Pitch, data[4..5]=Yaw, data[6..7]=Roll（小端 16 位映射值） */
void DmImu::_update_euler(const uint8_t (&data)[8])
{
  const uint16_t pitch_raw = static_cast<uint16_t>((data[3] << 8) | data[2]);
  const uint16_t yaw_raw   = static_cast<uint16_t>((data[5] << 8) | data[4]);
  const uint16_t roll_raw  = static_cast<uint16_t>((data[7] << 8) | data[6]);

  // 浮点换算放在临界区外，临界区里只做赋值
  const float pitch = uint_to_float(pitch_raw, PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
  const float yaw   = uint_to_float(yaw_raw, YAW_CAN_MIN, YAW_CAN_MAX, 16);
  const float roll  = uint_to_float(roll_raw, ROLL_CAN_MIN, ROLL_CAN_MAX, 16);

  taskENTER_CRITICAL();
  _imu_data.pitch = pitch;
  _imu_data.yaw   = yaw;
  _imu_data.roll  = roll;
  taskEXIT_CRITICAL();
}

/** @brief 四元数帧：14 位映射值按手册位序拼接 */
void DmImu::_update_quaternion(const uint8_t (&data)[8])
{
  const int w = (data[1] << 6) | ((data[2] & 0xF8U) >> 2);
  const int x = ((data[2] & 0x03U) << 12) | (data[3] << 4) | ((data[4] & 0xF0U) >> 4);
  const int y = ((data[4] & 0x0FU) << 10) | (data[5] << 2) | ((data[6] & 0xC0U) >> 6);
  const int z = ((data[6] & 0x3FU) << 8) | data[7];

  const float qw = uint_to_float(w, QUATERNION_MIN, QUATERNION_MAX, 14);
  const float qx = uint_to_float(x, QUATERNION_MIN, QUATERNION_MAX, 14);
  const float qy = uint_to_float(y, QUATERNION_MIN, QUATERNION_MAX, 14);
  const float qz = uint_to_float(z, QUATERNION_MIN, QUATERNION_MAX, 14);

  taskENTER_CRITICAL();
  _imu_data.q[0] = qw;
  _imu_data.q[1] = qx;
  _imu_data.q[2] = qy;
  _imu_data.q[3] = qz;
  taskEXIT_CRITICAL();
}

/** @brief CanRxNode 回调：静态转成员 */
Status DmImu::_rx_callback(void *context, const CanRxMsg &rx)
{
  return static_cast<DmImu *>(context)->data_unpack(rx);
}

// ----------------
