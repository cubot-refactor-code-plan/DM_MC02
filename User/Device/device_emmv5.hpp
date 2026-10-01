/**
 * @file device_emmv5.hpp
 * @author Rh
 * @brief Emm_V5.0 步进闭环电机驱动（UART 协议封装）
 * @version 0.1
 * @date 2026-02-20
 *
 * @copyright Copyright (c) 2026
 *
 * @details 基于张大头 Emm_V5.0 步进闭环控制协议，通过 bsp_uart 发送指令。
 *          支持单电机控制、多电机命令（MMCL）、多机同步等功能。
 *          协议格式：地址 + 功能码 + [辅助码/数据...] + 校验字节(0x6B)
 *
 * @note 初始化示例
 *
 *       DeviceEmmV5 motor_1({bsp_uart1, 1});     // 地址为1的电机
 *       motor_1.init();                          // 初始化
 *
 * @note 单电机控制示例
 *
 *       motor_1.en_control(true, false);                 // 使能电机
 *       motor_1.vel_control(0, 1000, 50, false);         // CW, 1000RPM, 加速度50
 *       motor_1.pos_control(0, 500, 50, 3200, 1, false); // 绝对位置模式
 *       motor_1.stop_now(false);                         // 立即停止
 *
 * @note 多电机命令（MMCL）示例（同一个串口，缓冲为实例成员，多机命令用同一实例加载（不同也行））
 *
 *       motor_1.mmcl_clear();                            // 清空本实例 MMCL 缓冲区
 *       motor_1.mmcl_vel_control(0, 500, 50, false);     // 加载电机1速度命令
 *       motor_1.mmcl_vel_control(1, 800, 50, false);     // 加载电机2速度命令
 *       motor_1.send_multi_motor_cmd(0);                 // 广播发送缓冲中的命令
 *
 */

#ifndef __DEVICE_EMMV5_HPP__
#define __DEVICE_EMMV5_HPP__


#include "bsp_cfg.hpp" // IWYU pragma: keep
#include "FreeRTOS.h"  // IWYU pragma: keep
#include "semphr.h"

#include "status.hpp" // 统一状态码


// ---------------- 常量定义 ----------------

#define EMMV5_MMCL_LEN (512)   ///< 多电机命令缓冲区大小（字节）
#define EMMV5_CHECKSUM (0x6B)  ///< 固定校验字节
#define EMMV5_BROADCAST (0x00) ///< 广播地址

// ----------------
// ---------------- 到位返回帧特征 ----------------

#define EMMV5_INPOS_MARK1 (0xFD) ///< 到位帧第 2 字节
#define EMMV5_INPOS_MARK2 (0x9F) ///< 到位帧第 3 字节
#define EMMV5_INPOS_MARK3 (0x6B) ///< 到位帧校验字节


// ----------------
// ---------------- 无状态帧构建助手 ----------------

/**
 * @brief Emm_V5 帧构建助手（无状态，编译期内联，零成本）
 *
 * @note 每条命令变成 3~5 行、一眼可对协议手册：
 * @code
 *   EmmFrame f;
 *   f.u8(_addr); f.u8(0xF6); f.u8(dir); f.u16(vel); f.u8(acc); f.u8(snF); f.finish();
 *   _send_cmd(f.buf, f.len);
 * @endcode
 */
struct EmmFrame
{
  uint8_t buf[32]; ///< 帧缓冲区（最大命令约 20 字节）
  size_t  len = 0; ///< 当前帧长度

  ///< 写入 1 字节（超界保护）
  void u8(uint8_t v)
  {
    if (len < sizeof(buf))
      buf[len++] = v;
  }

  ///< 写入 2 字节（大端）
  void u16(uint16_t v)
  {
    u8(uint8_t(v >> 8));
    u8(uint8_t(v));
  }

  ///< 写入 4 字节（大端）
  void u32(uint32_t v)
  {
    u8(uint8_t(v >> 24));
    u8(uint8_t(v >> 16));
    u8(uint8_t(v >> 8));
    u8(uint8_t(v));
  }

  ///< 追加固定校验字节
  void finish()
  {
    u8(EMMV5_CHECKSUM);
  }
};


// ----------------
// ---------------- 枚举定义 ----------------

/**
 * @brief 系统参数类型枚举
 *
 * @note 用于 read_sys_params() 和 auto_return_sys_params_timed()
 */
enum class EmmSysParam
{
  S_VBUS  = 5,  ///< 读取总线电压
  S_CBUS  = 6,  ///< 读取总线电流
  S_CPHA  = 7,  ///< 读取相电流
  S_ENCO  = 8,  ///< 读取编码器原始值
  S_CLKC  = 9,  ///< 读取实时脉冲数
  S_ENCL  = 10, ///< 读取经过线性化校准后的编码器值
  S_CLKI  = 11, ///< 读取输入脉冲数
  S_TPOS  = 12, ///< 读取电机目标位置
  S_SPOS  = 13, ///< 读取电机实时设定的目标位置
  S_VEL   = 14, ///< 读取电机实时转速
  S_CPOS  = 15, ///< 读取电机实时位置
  S_PERR  = 16, ///< 读取电机位置误差
  S_VBAT  = 17, ///< 读取多圈编码器电池电压（Y42）
  S_TEMP  = 18, ///< 读取电机实时温度（Y42）
  S_FLAG  = 19, ///< 读取电机状态标志位
  S_OFLAG = 20, ///< 读取回零状态标志位
  S_OAF   = 21, ///< 读取电机状态标志位 + 回零状态标志位（Y42）
  S_PIN   = 22, ///< 读取引脚状态（Y42）
};


/**
 * @brief 位置运动模式枚举
 */
enum class EmmPosMode
{
  POS_REL_PREV = 0, ///< 相对上一输入目标位置进行相对位置运动
  POS_ABS      = 1, ///< 绝对值运动
  POS_REL_CUR  = 2, ///< 相对当前电机实时位置进行相对位置运动
};


/**
 * @brief 回零模式枚举
 */
enum class EmmOriginMode
{
  ORIGIN_NEAR  = 0, ///< 单圈就近回零
  ORIGIN_DIR   = 1, ///< 单圈方向回零
  ORIGIN_COL   = 2, ///< 多圈无限位碰撞回零
  ORIGIN_LIMIT = 3, ///< 多圈有限位开关回零
};


// ----------------
// ---------------- 数据结构 ----------------

/**
 * @brief Emm_V5 电机反馈数据结构体
 */
struct EmmV5Data
{
  int32_t  encoder_raw   = 0; ///< 编码器原始值
  int32_t  encoder_cal   = 0; ///< 线性化校准后的编码器值
  int32_t  pulse_count   = 0; ///< 实时脉冲数
  int32_t  input_pulse   = 0; ///< 输入脉冲数
  int32_t  target_pos    = 0; ///< 电机目标位置
  int32_t  set_pos       = 0; ///< 电机实时设定的目标位置
  int32_t  cur_pos       = 0; ///< 电机实时位置
  int32_t  pos_error     = 0; ///< 电机位置误差
  int16_t  speed         = 0; ///< 电机实时转速（RPM）
  uint16_t bus_voltage   = 0; ///< 总线电压（mV）
  uint16_t bus_current   = 0; ///< 总线电流（mA）
  uint16_t phase_current = 0; ///< 相电流（mA）
  uint16_t battery_volt  = 0; ///< 多圈编码器电池电压（mV, Y42）
  uint8_t  temperature   = 0; ///< 电机实时温度（℃, Y42）
  uint8_t  status_flag   = 0; ///< 电机状态标志位
  uint8_t  origin_flag   = 0; ///< 回零状态标志位
  uint8_t  pin_status    = 0; ///< 引脚状态（Y42）
};


/**
 * @brief Emm_V5.0 步进闭环电机驱动类
 *
 * @note 通过 bsp_uart 发送控制指令，支持单电机控制和多电机命令。
 *       MMCL 缓冲区为实例成员，各实例独立缓冲。
 */
class DeviceEmmV5
{
public:
  // ----------------
  // ---------------- 构造与析构 ----------------

  /**
   * @brief 电机配置结构体（可匿名按序传入）
   */
  struct Config
  {
    /**
     * @brief 按序构造配置（参数顺序 = 字段顺序）
     */
    Config(BspUart<128> &uart, uint8_t addr = 0) : uart(uart),
                                                   addr(addr)
    {
    }

    BspUart<128> &uart; ///< bsp_uart 实例引用
    uint8_t          addr; ///< 电机地址（1~255，0为广播地址）
  };

  /**
   * @brief 构造函数
   * @param cfg 电机配置（uart 实例 + 地址，可匿名按序传入）
   */
  DeviceEmmV5(const Config &cfg);

  /**
   * @brief 析构函数
   */
  ~DeviceEmmV5();


  // ----------------
  // ---------------- 初始化 ----------------

  /**
   * @brief 初始化电机驱动
   * @return Status OK=初始化完成
   */
  Status init();


  // ----------------
  // ---------------- 触发动作命令 ----------------

  ///< 触发编码器校准
  void trig_encoder_cal();

  ///< 重启电机（Y42）
  void reset_motor();

  ///< 将当前位置清零
  void reset_curpos_to_zero();

  ///< 解除堵转保护
  void reset_clog_pro();

  ///< 恢复出厂设置
  void restore_motor();


  // ----------------
  // ---------------- 运动控制命令 ----------------

  /**
   * @brief 电机使能控制
   * @param state 使能状态，true=使能，false=关闭
   * @param snF   多机同步标志，false=不启用，true=启用
   */
  void en_control(bool state, bool snF = false);

  /**
   * @brief 速度模式控制
   * @param dir  方向，0=CW，其余值=CCW
   * @param vel  速度（0~5000 RPM）
   * @param acc  加速度（0~255，0=直接启动）
   * @param snF  多机同步标志
   */
  void vel_control(uint8_t dir, uint16_t vel, uint8_t acc, bool snF = false);

  /**
   * @brief 位置模式控制
   * @param dir  方向，0=CW，其余值=CCW
   * @param vel  速度（0~5000 RPM）
   * @param acc  加速度（0~255，0=直接启动）
   * @param clk  脉冲数（0 ~ 2^32-1）
   * @param raF  运动模式，0=相对上一目标，1=绝对值，2=相对当前位置
   * @param snF  多机同步标志
   */
  void pos_control(uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF, bool snF = false);

  /**
   * @brief 设置快速位置模式的运动参数
   * @param vel  速度（0~5000 RPM）
   * @param acc  加速度（0~255）
   * @param raF  运动模式
   * @param snF  多机同步标志
   */
  void set_qpos_params(uint16_t vel, uint8_t acc, uint8_t raF, bool snF = false);

  /**
   * @brief 快速位置模式控制（带符号脉冲数）
   * @param clk  脉冲数（带符号），默认16细分下±3200=±1圈
   */
  void qpos_control(int32_t clk);

  /**
   * @brief 立即停止运动
   * @param snF  多机同步标志
   */
  void stop_now(bool snF = false);

  /**
   * @brief 触发多机同步开始运动
   */
  void synchronous_motion();


  // ----------------
  // ---------------- 原点回零命令 ----------------

  /**
   * @brief 设置单圈回零的零点位置
   * @param svF  是否存储，false=不存储，true=存储
   */
  void origin_set_o(bool svF);

  /**
   * @brief 触发回零
   * @param o_mode 回零模式
   * @param snF    多机同步标志
   */
  void origin_trigger_return(uint8_t o_mode, bool snF = false);

  ///< 强制中断并退出回零
  void origin_interrupt();

  ///< 读取回零参数
  void origin_read_params();

  /**
   * @brief 修改回零参数
   * @param svF    是否存储
   * @param o_mode 回零模式
   * @param o_dir  回零方向（0=CW）
   * @param o_vel  回零速度（RPM）
   * @param o_tm   回零超时时间（ms）
   * @param sl_vel 碰撞回零检测转速（RPM）
   * @param sl_ma  碰撞回零检测电流（mA）
   * @param sl_ms  碰撞回零检测时间（ms）
   * @param potF   上电自动触发回零
   */
  void origin_modify_params(bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);

  ///< 读取碰撞回零返回角度（X42S/Y42）
  void origin_read_sl_rp();

  /**
   * @brief 修改碰撞回零返回角度（X42S/Y42）
   * @param svF   是否存储
   * @param sl_rp 返回角度（单位0.1°）
   */
  void origin_modify_sl_rp(bool svF, uint16_t sl_rp);


  // ----------------
  // ---------------- 读取系统参数命令 ----------------

  /**
   * @brief 定时返回系统参数（Y42）
   * @param s       系统参数类型
   * @param time_ms 定时时间（ms）
   */
  void auto_return_sys_params_timed(EmmSysParam s, uint16_t time_ms);

  /**
   * @brief 读取系统参数
   * @param s  系统参数类型
   */
  void read_sys_params(EmmSysParam s);


  // ----------------
  // ---------------- 读写驱动参数命令 ----------------

  /**
   * @brief 修改电机ID地址
   * @param svF  是否存储
   * @param id   新ID（1~255，0为广播地址）
   */
  void modify_motor_id(bool svF, uint8_t id);

  /**
   * @brief 修改细分值
   * @param svF    是否存储
   * @param mstep  细分值（1~255，0=256细分）
   */
  void modify_micro_step(bool svF, uint8_t mstep);

  /**
   * @brief 修改掉电标志
   * @param pdf  掉电标志
   */
  void modify_pd_flag(bool pdf);

  ///< 读取选项参数状态（Y42）
  void read_opt_param_sta();

  /**
   * @brief 修改电机类型（Y42）
   * @param svF     是否存储
   * @param mottype 0=1.8°步进电机，1=0.9°步进电机
   */
  void modify_motor_type(bool svF, bool mottype);

  /**
   * @brief 修改固件类型（Y42）
   * @param svF    是否存储
   * @param fwtype 0=X固件，1=Emm固件
   */
  void modify_firmware_type(bool svF, bool fwtype);

  /**
   * @brief 修改开环/闭环控制模式（Y42）
   * @param svF       是否存储
   * @param ctrl_mode 0=开环模式，1=闭环FOC模式
   */
  void modify_ctrl_mode(bool svF, bool ctrl_mode);

  /**
   * @brief 修改电机运动正方向（Y42）
   * @param svF  是否存储
   * @param dir  0=CW，1=CCW
   */
  void modify_motor_dir(bool svF, bool dir);

  /**
   * @brief 修改锁定按键功能（Y42）
   * @param svF     是否存储
   * @param lockbtn 0=Disable，1=Enable
   */
  void modify_lock_btn(bool svF, bool lockbtn);

  /**
   * @brief 修改命令速度值是否缩小10倍输入（Y42）
   * @param svF   是否存储
   * @param s_vel 0=Disable，1=Enable
   */
  void modify_s_vel(bool svF, bool s_vel);

  /**
   * @brief 修改开环模式工作电流
   * @param svF   是否存储
   * @param om_ma 电流值（mA）
   */
  void modify_om_ma(bool svF, uint16_t om_ma);

  /**
   * @brief 修改闭环模式最大电流
   * @param svF    是否存储
   * @param foc_ma 电流值（mA）
   */
  void modify_foc_ma(bool svF, uint16_t foc_ma);

  ///< 读取PID参数
  void read_pid_params();

  /**
   * @brief 修改PID参数
   * @param svF  是否存储
   * @param kp   比例系数
   * @param ki   积分系数
   * @param kd   微分系数
   */
  void modify_pid_params(bool svF, uint32_t kp, uint32_t ki, uint32_t kd);

  ///< 读取DMX512协议参数（Y42）
  void read_dmx512_params();

  /**
   * @brief 修改DMX512协议参数（Y42）
   * @param svF      是否存储
   * @param tch      总通道数
   * @param nch      每电机通道数（1=单通道，2=双通道）
   * @param mode     运动模式（0=相对位置，1=绝对坐标）
   * @param vel      单通道运动速度（RPM）
   * @param acc      加速度
   * @param vel_step 双通道速度步长
   * @param pos_step 双通道运动步长
   */
  void modify_dmx512_params(bool svF, uint16_t tch, uint8_t nch, uint8_t mode, uint16_t vel, uint16_t acc, uint16_t vel_step, uint32_t pos_step);

  ///< 读取位置到达窗口（Y42）
  void read_pos_window();

  /**
   * @brief 修改位置到达窗口（Y42）
   * @param svF  是否存储
   * @param prw  窗口值（默认8，即0.8°）
   */
  void modify_pos_window(bool svF, uint16_t prw);

  ///< 读取过热过流保护检测阈值（Y42）
  void read_otocp();

  /**
   * @brief 修改过热过流保护检测阈值（Y42）
   * @param svF     是否存储
   * @param otp     过热保护阈值（℃）
   * @param ocp     过流保护阈值（mA）
   * @param time_ms 检测时间（ms）
   */
  void modify_otocp(bool svF, uint16_t otp, uint16_t ocp, uint16_t time_ms);

  ///< 读取心跳保护功能时间（Y42）
  void read_heart_protect();

  /**
   * @brief 修改心跳保护功能时间（Y42）
   * @param svF  是否存储
   * @param hp   心跳保护时间（ms）
   */
  void modify_heart_protect(bool svF, uint32_t hp);

  ///< 读取积分限幅/刚性系数（Y42）
  void read_integral_limit();

  /**
   * @brief 修改积分限幅/刚性系数（Y42）
   * @param svF  是否存储
   * @param il   积分限幅值
   */
  void modify_integral_limit(bool svF, uint32_t il);


  // ----------------
  // ---------------- 读取所有驱动参数命令 ----------------

  ///< 读取系统状态参数
  void read_system_state_params();

  ///< 读取驱动配置参数
  void read_motor_conf_params();


  // ----------------
  // ---------------- 多电机命令（MMCL） ----------------

  /**
   * @brief 发送多电机命令（Y42）（实例方法，使用本实例的 UART 发送）
   *
   * @note 将本实例 MMCL 缓冲区中积累的所有命令一次性发送。
   *       发送后自动清空缓冲区。
   *
   * @param addr 电机地址（通常使用广播地址0）
   */
  void send_multi_motor_cmd(uint8_t addr);

  ///< 清空多电机命令缓冲区（实例方法）
  void mmcl_clear();


  // -------------
  // ---------------- MMCL 触发动作命令 ----------------

  void mmcl_trig_encoder_cal();
  void mmcl_reset_motor();
  void mmcl_reset_curpos_to_zero();
  void mmcl_reset_clog_pro();
  void mmcl_restore_motor();

  // -------------
  // ---------------- MMCL 运动控制命令 ----------------

  void mmcl_en_control(bool state, bool snF = false);
  void mmcl_vel_control(uint8_t dir, uint16_t vel, uint8_t acc, bool snF = false);
  void mmcl_pos_control(uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF, bool snF = false);
  void mmcl_set_qpos_params(uint16_t vel, uint8_t acc, uint8_t raF, bool snF = false);
  void mmcl_qpos_control(int32_t clk);
  void mmcl_stop_now(bool snF = false);
  void mmcl_synchronous_motion();

  // -------------
  // ---------------- MMCL 原点回零命令 ----------------

  void mmcl_origin_set_o(bool svF);
  void mmcl_origin_trigger_return(uint8_t o_mode, bool snF = false);
  void mmcl_origin_interrupt();
  void mmcl_origin_modify_params(bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);
  void mmcl_origin_read_sl_rp();
  void mmcl_origin_modify_sl_rp(bool svF, uint16_t sl_rp);

  // -------------
  // ---------------- MMCL 读取系统参数命令 ----------------

  void mmcl_auto_return_sys_params_timed(EmmSysParam s, uint16_t time_ms);
  void mmcl_read_sys_params(EmmSysParam s);


  // ----------------
  // ---------------- 数据获取 ----------------

  /**
   * @brief 获取电机地址
   * @return uint8_t 电机地址
   */
  uint8_t get_addr() const
  {
    return _addr;
  }

  /**
   * @brief 读取电机返回的原始响应数据（用于调试/测试打印）
   *
   * @note 在 read_sys_params() 等查询命令之后调用，从 UART 缓冲区读取响应。
   *       需适当延时等待电机回复后再调用。
   *
   * @param buffer  接收缓冲区
   * @param size    期望读取的字节数
   * @param received 实际读取的字节数（可为 nullptr）
   * @param timeout 超时时间（ms）
   * @return Status OK=读到数据，TIMEOUT=超时/无数据，BAD_ARG=参数非法
   */
  Status receive_raw(uint8_t *buffer, size_t size, size_t *received = nullptr, uint32_t timeout = 100);


  // ----------------
  // ---------------- 接收与到位检测 ----------------

  /**
   * @brief 扫描缓冲中是否存在"到位返回帧"（地址 + 0xFD 0x9F 0x6B）
   * @param buf 接收缓冲
   * @param n   有效字节数
   * @return true=检测到到位帧
   */
  bool scan_in_position(const uint8_t *buf, int n);

  /**
   * @brief 在流缓冲区基础上做滑动窗口装配
   * @param data 流缓冲区读出的一块数据
   * @param n    有效字节数
   * @return true=凑齐完整到位帧（addr + FD 9F 6B），窗口已清空并记录相对到达时间
   */
  bool feed_rx(const uint8_t *data, size_t n);


  // ----------------
  // ---------------- 到位信号量 ----------------

  /**
   * @brief 绑定到位信号量（device_cfg 初始化时调用）
   * @param sem 到位信号量句柄（nullptr 时不 give）
   */
  void set_in_pos_sem(SemaphoreHandle_t sem);

  ///< 获取到位信号量
  SemaphoreHandle_t in_pos_sem() const { return _in_pos_sem; }


  // ----------------
  // ---------------- 静态注册表 / 统一到位接收任务 ----------------

  static constexpr size_t MAX_INSTANCES = 10; ///< 最大实例数

  ///< 当前已注册实例数
  static size_t instance_count();

  ///< 按下标取实例（越界返回 nullptr）
  static DeviceEmmV5 *get_instance_by_index(size_t i);

  ///< 统一到位接收任务入口（arg = this 实例指针）
  static void rx_task_entry(void *arg);

  ///< 遍历注册表，为每路电机创建一个到位接收任务（device_init 调用）
  static void create_rx_tasks();


  // ----------------
  // ---------------- 成员变量 ----------------

  BspUart<128> &_uart; ///< bsp_uart 实例引用
  uint8_t          _addr; ///< 电机地址


private:
  // ----------------
  // ---------------- 底层发送 ----------------

  /**
   * @brief 通过 bsp_uart 发送命令
   * @param cmd  命令字节数组
   * @param len  数据长度
   * @return Status 同 BspUart::send()
   */
  Status _send_cmd(const uint8_t *cmd, size_t len);

  /**
   * @brief 将命令追加到 MMCL 缓冲区（实例成员）
   * @param cmd  命令字节数组
   * @param len  数据长度
   */
  void _mmcl_append(const uint8_t *cmd, size_t len);

  // ----------------
  // ---------------- MMCL 成员缓冲区 ----------------

  uint8_t  _mmcl_buf[EMMV5_MMCL_LEN + 5]; ///< 多电机命令缓冲区（实例成员，可容纳完整帧，避免栈上 517B 大数组）
  uint16_t _mmcl_count = 0;               ///< MMCL 缓冲区中当前字节数

  // ----------------
  // ---------------- 静态成员（实例注册表） ----------------

  static DeviceEmmV5 *_instances[MAX_INSTANCES]; ///< 实例注册表
  static size_t       _instance_count;           ///< 已注册实例数
  Status              _register_instance();       ///< 构造时注册到注册表

  // ----------------
  // ---------------- 到位信号量 ----------------

  SemaphoreHandle_t _in_pos_sem = nullptr; ///< 到位信号量（device_cfg 绑定）

  // ----------------
  // ---------------- 到位帧滑动窗口（基于流缓冲区） ----------------

  uint8_t    _rx_assem[4];         ///< 滑动窗口（仅在流缓冲读出块内装配）
  uint8_t    _rx_assem_len    = 0; ///< 窗口当前有效字节数
  TickType_t _last_inpos_tick = 0; ///< 最近一次完整到位帧的相对到达时间（ticks）

  // ----------------
};


#endif // __DEVICE_EMMV5_HPP__
