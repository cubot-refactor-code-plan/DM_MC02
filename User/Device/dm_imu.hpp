/**
 * @file dm_imu.hpp
 * @author Rh
 * @brief 达妙 DM-IMU-L1 六轴 IMU：注册到 CanBus，主动模式下自动接收姿态数据
 * @version 0.2
 * @date 2026-10-02
 *
 * @copyright Copyright (c) 2026
 *
 * @details 模块对外只有一条 8 字节标准帧通道（手册《DM-IMU-L1 V1.2》）：
 *
 *          | 方向 | 帧 ID | 数据域 |
 *          | --- | --- | --- |
 *          | 本机 → IMU | `CAN_ID`（本板 0x58） | `CC RID 读/写 DD + 数据`，请求 / 配置帧 |
 *          | IMU → 本机 | `MST_ID`（本板 0x59） | 数据帧：`data[0]` = 01 加速度 / 02 角速度 / 03 欧拉 / 04 四元数 |
 *          | IMU → 本机 | `MST_ID` | 应答帧：`data[0]` = CC，`data[1]` = RID，`data[3]` = 应答码 |
 *
 *          init() 只做两件事：把 `MST_ID` 注册为 CanBus 的接收节点（之后由 can_rx_task 自动
 *          分发），再打开主动模式。本类不需要任何外部轮询，也没有 on_can_message() 这类
 *          需要上层手工调用的入口。
 *
 * @note 用法（全局实例见 device_cfg.cpp，由 device_init() 初始化）：
 *
 *       dm_imu.init();                                 // 注册接收节点 + 打开主动模式
 *       const ImuData d = dm_imu.get_imu_data();       // 线程安全的数据快照
 *       if (dm_imu.online().is_online() == Status::OK) // 数据流是否正常
 *
 * @warning init() 必须在调度器启动前（device_init() 中）调用：CanBus 的节点注册表在
 *          can_rx_task 启动后立即冻结，之后再注册一律失败。
 * @warning 寄存器指令是"一次性事件"（重启 / 校准 / 改 ID），所以走 CanBus::send() 单发，
 *          **不注册 CanTxNode** —— 否则会被 tx_poll() 的 5 ms 保底心跳周期性重发，
 *          等于反复重启、反复校准。
 * @warning 改波特率 / 切换通信端口这类指令会让本机与模块直接失联，只在明确知道
 *          自己在做什么时才调用。
 * @warning 主动发送间隔最小 1 ms（手册：输出频率 100~1000 Hz 可调）。间隔越小，同一
 *          总线的负载越高，请按需设置。
 */

#ifndef __DM_IMU_HPP__
#define __DM_IMU_HPP__

#include "can_bus.hpp"      // CanBus / CanRxNode / CanRxMsg
#include "online_check.hpp" // 在线检查
#include "status.hpp"       // 统一状态码

#include <stdint.h>

// ---------------- 量程与映射范围（手册附录） ----------------

#define ACCEL_CAN_MAX (235.2f)  ///< 加速度计最大值（m/s²）
#define ACCEL_CAN_MIN (-235.2f) ///< 加速度计最小值（m/s²）
#define GYRO_CAN_MAX (34.88f)   ///< 陀螺仪最大值（rad/s）
#define GYRO_CAN_MIN (-34.88f)  ///< 陀螺仪最小值（rad/s）
#define PITCH_CAN_MAX (90.0f)   ///< 俯仰角 Pitch 最大值（度）
#define PITCH_CAN_MIN (-90.0f)  ///< 俯仰角 Pitch 最小值（度）
#define ROLL_CAN_MAX (180.0f)   ///< 横滚角 Roll 最大值（度）
#define ROLL_CAN_MIN (-180.0f)  ///< 横滚角 Roll 最小值（度）
#define YAW_CAN_MAX (180.0f)    ///< 偏航角 Yaw 最大值（度）
#define YAW_CAN_MIN (-180.0f)   ///< 偏航角 Yaw 最小值（度）
#define TEMP_MIN (0.0f)         ///< 温度最小值（摄氏度）
#define TEMP_MAX (60.0f)        ///< 温度最大值（摄氏度）
#define QUATERNION_MIN (-1.0f)  ///< 四元数最小值
#define QUATERNION_MAX (1.0f)   ///< 四元数最大值

#define CMD_READ 0  ///< 请求帧的"读"操作码
#define CMD_WRITE 1 ///< 请求帧的"写"操作码

/**
 * @brief 通信端口枚举（用于 change_com_port()）
 */
enum class ImuComPort
{
  COM_USB = 0, ///< USB 虚拟串口
  COM_RS485,   ///< RS485
  COM_CAN,     ///< CAN
  COM_VOFA,    ///< VOFA（Justfloat）
};

/**
 * @brief CAN 波特率序号枚举（用于 set_baud()，序号含义见手册）
 */
enum class ImuBaudrate
{
  CAN_BAUD_1M = 0, ///< 1 Mbps
  CAN_BAUD_500K,   ///< 500 kbps
  CAN_BAUD_400K,   ///< 400 kbps
  CAN_BAUD_250K,   ///< 250 kbps
  CAN_BAUD_200K,   ///< 200 kbps
  CAN_BAUD_100K,   ///< 100 kbps
  CAN_BAUD_50K,    ///< 50 kbps
  CAN_BAUD_25K,    ///< 25 kbps
};

/**
 * @brief IMU 数据（四类数据帧全部解析）
 */
struct ImuData
{
  uint8_t can_id;   ///< 设备 CAN_ID（构造时写入）
  uint8_t mst_id;   ///< 主机 MST_ID（构造时写入）
  float   accel[3]; ///< 加速度 (m/s²)：x, y, z（来自 0x01 帧）
  float   gyro[3];  ///< 角速度 (rad/s)：x, y, z（来自 0x02 帧）
  float   pitch;    ///< 俯仰角（度）（来自 0x03 帧）
  float   roll;     ///< 横滚角（度）（来自 0x03 帧）
  float   yaw;      ///< 偏航角（度）（来自 0x03 帧）
  float   q[4];     ///< 四元数 w, x, y, z（来自 0x04 帧）
  float   cur_temp; ///< 温度（摄氏度）（来自 0x01 帧的 data[1]）
};

/**
 * @brief 达妙 DM-IMU-L1 设备类
 *
 * @note 接收由 CanBus 分发任务回调驱动；发送为单发，不占用 CanTxNode 槽位。
 *       数据与解析结果用 taskENTER_CRITICAL 保护（无需互斥量）。
 */
class DmImu
{
public:
  // ---------------- 配置 ----------------

  /**
   * @brief IMU 配置结构体（可匿名按序传入）
   */
  struct Config
  {
    /**
     * @brief 按序构造配置（参数顺序 = 字段顺序）
     *
     * @param can              所在 CAN 总线
     * @param device_id        CAN_ID（主）：本机发送请求 / 配置帧的目标 ID
     * @param master_id        MST_ID（主）：模块发出数据帧所用的 ID，注册接收节点时使用
     * @param timeout_ms       在线判定阈值（ms），超时未收到数据即判离线
     * @param active_delay_ms  主动模式发送间隔（ms），0 = 不改（沿用模块当前设置）
     * @param save_params      init() 是否把上述配置写进**模块内部 flash**（掉电不丢）
     * @param device_id_alt    CAN_ID（备）：0 = 不启用；非 0 时指令会主备各发一次
     * @param master_id_alt    MST_ID（备）：0 = 不启用；非 0 时额外注册一个接收节点
     */
    Config(CanBus &can, uint8_t device_id, uint8_t master_id = 0U, uint16_t timeout_ms = 100U, uint32_t active_delay_ms = 0U, bool save_params = false, uint8_t device_id_alt = 0U, uint8_t master_id_alt = 0U)

      : can(can),
        device_id(device_id),
        master_id(master_id),
        timeout_ms(timeout_ms),
        active_delay_ms(active_delay_ms),
        save_params(save_params),
        device_id_alt(device_id_alt),
        master_id_alt(master_id_alt)
    {
    }

    CanBus  &can;             ///< 所在 CAN 总线
    uint8_t  device_id;       ///< CAN_ID（主发送目标）
    uint8_t  master_id;       ///< MST_ID（主接收节点 ID）
    uint16_t timeout_ms;      ///< 在线判定阈值（ms）
    uint32_t active_delay_ms; ///< 主动模式发送间隔（ms），0 = 不改
    bool     save_params;     ///< 是否把配置写进模块 flash（模块 flash 有擦写寿命，默认不写）
    uint8_t  device_id_alt;   ///< CAN_ID（备），0 = 不启用
    uint8_t  master_id_alt;   ///< MST_ID（备），0 = 不启用
  };

  // ----------------
  // ---------------- 构造与析构 ----------------

  /**
   * @brief 构造并绑定总线（只做赋值，不注册节点、不发帧）
   * @param cfg 配置（总线 / CAN_ID / MST_ID / 超时 / 主动间隔）
   */
  DmImu(const Config &cfg);

  /**
   * @brief 析构：注销接收节点
   * @warning 注册表在调度器启动后冻结，那时无法注销；成功初始化过的对象须存活至系统停止。
   */
  ~DmImu();

  DmImu(const DmImu &)            = delete;
  DmImu &operator=(const DmImu &) = delete;

  // ----------------
  // ---------------- 公共接口 ----------------

  /**
   * @brief 注册接收节点并打开主动模式（幂等）
   *
   * @return OK=成功；FULL=接收节点注册失败（重复 ID / 节点超限 / 注册表已冻结）；
   *         其他=打开主动模式的发送结果（BUSY/FULL/NOT_INIT）
   *
   * @note 必须在调度器启动前调用（device_init()）。
   * @note 发送成功只代表"帧已入队并可上线"，不代表模块已接受；真正的确认看 online()。
   */
  Status init();

  /**
   * @brief 解析一帧属于本 IMU 的 CAN 帧（由 CanRxNode 回调调用）
   *
   * @param rx 待解析的标准帧
   * @return OK=已消费（01~04 数据帧已更新数据并刷新在线；应答帧已记录应答码）；
   *         BAD_ARG=ID / 帧格式 / 长度不匹配，或不是 01~04 的未知帧（会进回退缓冲）
   *
   * @note 应答帧（data[0]=CC）只记录 RID 与应答码，**不刷新在线状态**：应答只能证明
   *       模块响应了指令，不能证明主动数据流正常。
   */
  Status data_unpack(const CanRxMsg &rx);

  // ---------------- 寄存器指令（一次性单发，全部返回发送结果） ----------------

  /** @brief 重启模块 */
  Status reboot();

  /** @brief 启动加速度计六面校准 */
  Status accel_calibration();

  /** @brief 启动陀螺仪静态校准 */
  Status gyro_calibration();

  /** @brief 切换通信端口（警告：切走后本机将失联） */
  Status change_com_port(ImuComPort port);

  /** @brief 设置主动模式发送间隔（ms） */
  Status set_active_mode_delay(uint32_t delay_ms);

  /** @brief 打开主动模式（模块按间隔自动上报数据） */
  Status change_to_active();

  /** @brief 切回应答模式（本类不支持应答式读取姿态数据，一般不用） */
  Status change_to_request();

  /** @brief 设置 CAN 波特率（警告：与本机 FDCAN 配置不一致会失联） */
  Status set_baud(ImuBaudrate baud);

  /** @brief 设置 CAN_ID（警告：改完须用新 ID 发送，并保存参数） */
  Status set_can_id(uint8_t can_id);

  /** @brief 设置 MST_ID（警告：改完须同步改 Config::master_id，并保存参数） */
  Status set_mst_id(uint8_t mst_id);

  /** @brief 保存参数到模块 Flash */
  Status save_parameters();

  /** @brief 恢复出厂设置（警告：会清掉 ID 等配置） */
  Status restore_settings();

  /** @brief 请求一帧欧拉角数据（应答模式用；主动模式下不需要） */
  Status request_euler();

  /** @brief 请求一帧四元数数据（应答模式用；主动模式下不需要） */
  Status request_quat();

  // ---------------- 诊断 ----------------

  /**
   * @brief 向任意 CAN_ID 发一帧"读欧拉角"请求（不改变本对象的配置）
   *
   * @param can_id 目标 CAN_ID（11 位标准帧 ID）
   * @return 发送结果（OK=已入队）
   *
   * @note 用途：模块真实 CAN_ID 未知时逐个 ID 探测。读操作一定会回应答帧，
   *       所以观察 last_ack_id() 的变化就能反推出模块的 CAN_ID 与 MST_ID。
   */
  Status probe_read(uint32_t can_id);

  // ---------------- 查询 ----------------

  /** @brief 取数据快照（临界区保护，任务 / ISR 上下文均可） */
  ImuData get_imu_data();

  /** @brief 构造校验 / 初始化状态 */
  Status statu() const;

  /** @brief 在线检查对象（is_online() == OK 表示数据流正常） */
  const Online &online() const;

  /** @brief 设备 CAN_ID */
  uint8_t get_device_id() const
  {
    return _device_id;
  }

  /** @brief 主机 MST_ID（主） */
  uint8_t get_master_id() const
  {
    return _master_id;
  }

  /**
   * @brief 最近一次应答帧的 ID
   * @return 应答帧 ID；0xFF = 还没收到过任何应答
   * @note 用来确认模块真实的 MST_ID（在双候选 ID 下判断到底哪个生效）
   */
  uint8_t last_ack_id() const
  {
    return _ack_id;
  }

  /** @brief 最近一次应答帧的寄存器号；0xFF = 还没收到过应答 */
  uint8_t last_ack_reg() const
  {
    return _ack_reg;
  }

  /** @brief 最近一次应答帧的应答码（0x00=成功）；0xFF = 还没收到过应答 */
  uint8_t last_ack_code() const
  {
    return _ack_code;
  }

  /**
   * @brief 已收到的数据帧总数（加速度 / 角速度 / 欧拉角 / 四元数，不含应答帧）
   * @note 两次读数之差 ÷ 间隔秒数 = 实际帧率，用来验证主动上报的频率是否真的生效。
   */
  uint32_t data_frames() const
  {
    return _data_frames;
  }

  // ----------------
private:
  // ---------------- 寄存器表（手册 CAN 通道，RID） ----------------

  enum class RegId : uint8_t
  {
    REBOOT = 0x00,                ///< 重启
    ACCEL_DATA = 0x01,            ///< 加速度数据
    GYRO_DATA = 0x02,             ///< 角速度数据
    EULER_DATA = 0x03,            ///< 欧拉角数据
    QUAT_DATA = 0x04,             ///< 四元数数据
    SET_ZERO = 0x05,              ///< 角度置零
    ACCEL_CALI = 0x06,            ///< 加计六面校准
    GYRO_CALI = 0x07,             ///< 陀螺静态校准
    MAG_CALI = 0x08,              ///< 磁计椭球校准
    CHANGE_COM = 0x09,            ///< 切换通信模式
    SET_DELAY = 0x0A,             ///< 设置主动发送间隔
    CHANGE_ACTIVE = 0x0B,         ///< 切换主被动模式
    SET_BAUD = 0x0C,              ///< 修改波特率
    SET_CAN_ID = 0x0D,            ///< CAN_ID
    SET_MST_ID = 0x0E,            ///< MST_ID
    DATA_OUTPUT_SELECTION = 0x0F, ///< 输出数据选择（当前固件不支持修改）
    SAVE_PARAM = 0xFE,            ///< 保存参数
    RESTORE_SETTING = 0xFF,       ///< 恢复出厂设置
  };

  // ----------------
  // ---------------- 私有方法 ----------------

  /** @brief 向主 / 备 CAN_ID 各发一帧（备 ID 为 0 时只发主） */
  Status _send(const uint8_t *buf);

  /** @brief 写寄存器：`CC RID 01 DD + 4 字节数据`（小端） */
  Status _write_register(RegId reg_id, uint32_t data);

  /** @brief 读寄存器：`CC RID 00 DD + 4 字节 0` */
  Status _read_register(RegId reg_id);

  /** @brief 解析加速度帧（含温度）并写入 _imu_data */
  void _update_accel(const uint8_t (&data)[8]);

  /** @brief 解析角速度帧并写入 _imu_data */
  void _update_gyro(const uint8_t (&data)[8]);

  /** @brief 解析欧拉角帧并写入 _imu_data */
  void _update_euler(const uint8_t (&data)[8]);

  /** @brief 解析四元数帧并写入 _imu_data */
  void _update_quaternion(const uint8_t (&data)[8]);

  /** @brief CanRxNode 回调：转调 data_unpack() */
  static Status _rx_callback(void *context, const CanRxMsg &rx);

  // ----------------
  // ---------------- 成员变量 ----------------

  CanBus   &_can;             ///< 所在 CAN 总线
  uint8_t   _device_id;       ///< CAN_ID（主发送目标）
  uint8_t   _master_id;       ///< MST_ID（主接收节点 ID）
  uint8_t   _device_id_alt;   ///< CAN_ID（备），0 = 不启用
  uint8_t   _master_id_alt;   ///< MST_ID（备），0 = 不启用
  uint32_t  _active_delay_ms; ///< 主动模式发送间隔（init() 用）
  bool      _save_params;     ///< init() 是否写模块 flash（写 flash 有寿命，默认不写）
  CanRxNode *_rx_node;        ///< 主接收节点（注册成功后由 CanBus 持有）
  CanRxNode *_rx_node_alt;    ///< 备用接收节点，未启用时为 nullptr
  Online    _online;          ///< 在线检查（由数据帧刷新）
  ImuData   _imu_data;        ///< 姿态数据（taskENTER_CRITICAL 保护）
  Status    _statu;           ///< 构造校验 / 初始化状态

  volatile uint32_t _data_frames; ///< 收到的数据帧计数（01~04），用于估算帧率
  volatile uint8_t  _ack_id;      ///< 最近一次应答帧的 ID（调试用）
  volatile uint8_t  _ack_reg;     ///< 最近一次应答帧的 RID（调试用）
  volatile uint8_t  _ack_code;    ///< 最近一次应答帧的应答码（00=成功）

  // ----------------
};

#endif // __DM_IMU_HPP__
