#include "device_emmv5.hpp"

#include "semphr.h" // xSemaphoreGive
#include "task.h"   // xTaskCreate
#include <stdio.h>  // snprintf
#include <string.h> // memmove


/* ==================== 静态成员初始化 ==================== */

DeviceEmmV5 *DeviceEmmV5::_instances[DeviceEmmV5::MAX_INSTANCES] = {};
size_t       DeviceEmmV5::_instance_count                        = 0;


/* ==================== 系统参数表（表驱动） ==================== */

struct SysParamDef
{
  EmmSysParam id;   ///< 系统参数枚举
  uint8_t     code; ///< 协议功能码
};

static const SysParamDef k_sys_params[] = {
  {EmmSysParam::S_VBUS, 0x24},
  {EmmSysParam::S_CBUS, 0x26},
  {EmmSysParam::S_CPHA, 0x27},
  {EmmSysParam::S_ENCO, 0x29},
  {EmmSysParam::S_CLKC, 0x30},
  {EmmSysParam::S_ENCL, 0x31},
  {EmmSysParam::S_CLKI, 0x32},
  {EmmSysParam::S_TPOS, 0x33},
  {EmmSysParam::S_SPOS, 0x34},
  {EmmSysParam::S_VEL, 0x35},
  {EmmSysParam::S_CPOS, 0x36},
  {EmmSysParam::S_PERR, 0x37},
  {EmmSysParam::S_VBAT, 0x38},
  {EmmSysParam::S_TEMP, 0x39},
  {EmmSysParam::S_FLAG, 0x3A},
  {EmmSysParam::S_OFLAG, 0x3B},
  {EmmSysParam::S_OAF, 0x3C},
  {EmmSysParam::S_PIN, 0x3D},
};

/**
 * @brief 查表：系统参数枚举 → 协议功能码；非法参数返回 0
 */
static uint8_t sys_param_code(EmmSysParam s)
{
  for (size_t k = 0; k < sizeof(k_sys_params) / sizeof(k_sys_params[0]); k++)
  {
    if (k_sys_params[k].id == s)
    {
      return k_sys_params[k].code;
    }
  }
  return 0;
}


/* ==================== 构造函数与析构函数 ==================== */

/**
 * @brief 构造函数
 * @param cfg 电机配置（uart 实例 + 地址，可匿名按序传入）
 */
DeviceEmmV5::DeviceEmmV5(const Config &cfg)

  : _uart(cfg.uart),
    _addr(cfg.addr)
{
  // 构造函数只做赋值；实例注册（运行时逻辑）推迟到 init()
}


/**
 * @brief 析构函数
 */
DeviceEmmV5::~DeviceEmmV5()
{
}


/* ==================== 初始化 ==================== */

/**
 * @brief 初始化电机驱动
 */
Status DeviceEmmV5::init()
{
  // 注册到静态实例注册表（统一到位接收任务使用）
  register_instance();

  mmcl_clear();
  return Status::OK;
}


/* ==================== 底层发送 ==================== */

/**
 * @brief 通过 bsp_uart 发送命令
 * @param cmd  命令字节数组
 * @param len  数据长度
 */
Status DeviceEmmV5::_send_cmd(const uint8_t *cmd, size_t len)
{
  return _uart.send(cmd, len, 0);
}


/**
 * @brief 将命令追加到 MMCL 成员缓冲区
 * @param cmd  命令字节数组
 * @param len  数据长度
 */
void DeviceEmmV5::_mmcl_append(const uint8_t *cmd, size_t len)
{
  for (size_t i = 0; i < len; i++)
  {
    if (_mmcl_count < EMMV5_MMCL_LEN)
    {
      _mmcl_buf[_mmcl_count] = cmd[i];
      ++_mmcl_count;
    }
  }
}


/* ==================== 触发动作命令 ==================== */

/**
 * @brief 触发编码器校准
 */
void DeviceEmmV5::trig_encoder_cal()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x06);
  f.u8(0x45);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 重启电机（Y42）
 */
void DeviceEmmV5::reset_motor()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x08);
  f.u8(0x97);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 将当前位置清零
 */
void DeviceEmmV5::reset_curpos_to_zero()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x0A);
  f.u8(0x6D);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 解除堵转保护
 */
void DeviceEmmV5::reset_clog_pro()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x0E);
  f.u8(0x52);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 恢复出厂设置
 */
void DeviceEmmV5::restore_motor()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x0F);
  f.u8(0x5F);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/* ==================== 运动控制命令 ==================== */

/**
 * @brief 电机使能控制
 * @param state 使能状态，true=使能，false=关闭
 * @param snF   多机同步标志
 */
void DeviceEmmV5::en_control(bool state, bool snF)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0xF3);
  f.u8(0xAB);
  f.u8(static_cast<uint8_t>(state));
  f.u8(static_cast<uint8_t>(snF));
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 速度模式控制
 * @param dir  方向，0=CW，其余值=CCW
 * @param vel  速度（0~5000 RPM）
 * @param acc  加速度（0~255，0=直接启动）
 * @param snF  多机同步标志
 */
void DeviceEmmV5::vel_control(uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0xF6);
  f.u8(dir);
  f.u16(vel);
  f.u8(acc);
  f.u8(static_cast<uint8_t>(snF));
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 位置模式控制
 * @param dir  方向，0=CW，其余值=CCW
 * @param vel  速度（0~5000 RPM）
 * @param acc  加速度（0~255，0=直接启动）
 * @param clk  脉冲数（0 ~ 2^32-1）
 * @param raF  运动模式，0=相对上一目标，1=绝对值，2=相对当前位置
 * @param snF  多机同步标志
 */
void DeviceEmmV5::pos_control(uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF, bool snF)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0xFD);
  f.u8(dir);
  f.u16(vel);
  f.u8(acc);
  f.u32(clk);
  f.u8(raF);
  f.u8(static_cast<uint8_t>(snF));
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 设置快速位置模式的运动参数
 * @param vel  速度（0~5000 RPM）
 * @param acc  加速度（0~255）
 * @param raF  运动模式
 * @param snF  多机同步标志
 */
void DeviceEmmV5::set_qpos_params(uint16_t vel, uint8_t acc, uint8_t raF, bool snF)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0xF1);
  f.u16(vel);
  f.u8(acc);
  f.u8(raF);
  f.u8(static_cast<uint8_t>(snF));
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 快速位置模式控制（带符号脉冲数）
 * @param clk  脉冲数（带符号），默认16细分下±3200=±1圈
 */
void DeviceEmmV5::qpos_control(int32_t clk)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0xFC);
  f.u32(static_cast<uint32_t>(clk));
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 立即停止运动
 * @param snF  多机同步标志
 */
void DeviceEmmV5::stop_now(bool snF)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0xFE);
  f.u8(0x98);
  f.u8(static_cast<uint8_t>(snF));
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 触发多机同步开始运动
 */
void DeviceEmmV5::synchronous_motion()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0xFF);
  f.u8(0x66);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/* ==================== 原点回零命令 ==================== */

/**
 * @brief 设置单圈回零的零点位置
 * @param svF  是否存储
 */
void DeviceEmmV5::origin_set_o(bool svF)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x93);
  f.u8(0x88);
  f.u8(static_cast<uint8_t>(svF));
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 触发回零
 * @param o_mode 回零模式
 * @param snF    多机同步标志
 */
void DeviceEmmV5::origin_trigger_return(uint8_t o_mode, bool snF)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x9A);
  f.u8(o_mode);
  f.u8(static_cast<uint8_t>(snF));
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 强制中断并退出回零
 */
void DeviceEmmV5::origin_interrupt()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x9C);
  f.u8(0x48);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 读取回零参数
 */
void DeviceEmmV5::origin_read_params()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x22);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 修改回零参数
 */
void DeviceEmmV5::origin_modify_params(bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x4C);
  f.u8(0xAE);
  f.u8(static_cast<uint8_t>(svF));
  f.u8(o_mode);
  f.u8(o_dir);
  f.u16(o_vel);
  f.u32(o_tm);
  f.u16(sl_vel);
  f.u16(sl_ma);
  f.u16(sl_ms);
  f.u8(static_cast<uint8_t>(potF));
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 读取碰撞回零返回角度（X42S/Y42）
 */
void DeviceEmmV5::origin_read_sl_rp()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x3F);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 修改碰撞回零返回角度（X42S/Y42）
 * @param svF   是否存储
 * @param sl_rp 返回角度（单位0.1°）
 */
void DeviceEmmV5::origin_modify_sl_rp(bool svF, uint16_t sl_rp)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x5C);
  f.u8(0xAC);
  f.u8(static_cast<uint8_t>(svF));
  f.u16(sl_rp);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/* ==================== 读取系统参数命令 ==================== */

/**
 * @brief 定时返回系统参数（Y42）
 * @param s       系统参数类型
 * @param time_ms 定时时间（ms）
 */
void DeviceEmmV5::auto_return_sys_params_timed(EmmSysParam s, uint16_t time_ms)
{
  // 表驱动构建 addr + 0x11 0x18 + code + time_ms(2B) + 校验
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x11);
  f.u8(0x18);
  f.u8(sys_param_code(s));
  f.u16(time_ms);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 读取系统参数
 * @param s  系统参数类型
 */
void DeviceEmmV5::read_sys_params(EmmSysParam s)
{
  // 表驱动构建 addr + code + 校验
  EmmFrame f;
  f.u8(_addr);
  f.u8(sys_param_code(s));
  f.finish();
  _send_cmd(f.buf, f.len);
}


/* ==================== 数据获取 ==================== */

/**
 * @brief 读取电机返回的原始响应数据
 */
Status DeviceEmmV5::receive_raw(uint8_t *buffer, size_t size, size_t *received, uint32_t timeout)
{
  return _uart.receive(buffer, size, timeout,received);
}


/* ==================== 读写驱动参数命令 ==================== */

/**
 * @brief 修改电机ID地址
 */
void DeviceEmmV5::modify_motor_id(bool svF, uint8_t id)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0xAE);
  f.u8(0x4B);
  f.u8(static_cast<uint8_t>(svF));
  f.u8(id);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 修改细分值
 */
void DeviceEmmV5::modify_micro_step(bool svF, uint8_t mstep)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x84);
  f.u8(0x8A);
  f.u8(static_cast<uint8_t>(svF));
  f.u8(mstep);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 修改掉电标志
 */
void DeviceEmmV5::modify_pd_flag(bool pdf)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x50);
  f.u8(static_cast<uint8_t>(pdf));
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 读取选项参数状态（Y42）
 */
void DeviceEmmV5::read_opt_param_sta()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x1A);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 修改电机类型（Y42）
 */
void DeviceEmmV5::modify_motor_type(bool svF, bool mottype)
{
  uint8_t  mot_type_val = mottype ? 25 : 50;
  EmmFrame f;
  f.u8(_addr);
  f.u8(0xD7);
  f.u8(0x35);
  f.u8(static_cast<uint8_t>(svF));
  f.u8(mot_type_val);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 修改固件类型（Y42）
 */
void DeviceEmmV5::modify_firmware_type(bool svF, bool fwtype)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0xD5);
  f.u8(0x69);
  f.u8(static_cast<uint8_t>(svF));
  f.u8(static_cast<uint8_t>(fwtype));
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 修改开环/闭环控制模式（Y42）
 */
void DeviceEmmV5::modify_ctrl_mode(bool svF, bool ctrl_mode)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x46);
  f.u8(0x69);
  f.u8(static_cast<uint8_t>(svF));
  f.u8(static_cast<uint8_t>(ctrl_mode));
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 修改电机运动正方向（Y42）
 */
void DeviceEmmV5::modify_motor_dir(bool svF, bool dir)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0xD4);
  f.u8(0x60);
  f.u8(static_cast<uint8_t>(svF));
  f.u8(static_cast<uint8_t>(dir));
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 修改锁定按键功能（Y42）
 */
void DeviceEmmV5::modify_lock_btn(bool svF, bool lockbtn)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0xD0);
  f.u8(0xB3);
  f.u8(static_cast<uint8_t>(svF));
  f.u8(static_cast<uint8_t>(lockbtn));
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 修改命令速度值是否缩小10倍输入（Y42）
 */
void DeviceEmmV5::modify_s_vel(bool svF, bool s_vel)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x4F);
  f.u8(0x71);
  f.u8(static_cast<uint8_t>(svF));
  f.u8(static_cast<uint8_t>(s_vel));
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 修改开环模式工作电流
 */
void DeviceEmmV5::modify_om_ma(bool svF, uint16_t om_ma)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x44);
  f.u8(0x33);
  f.u8(static_cast<uint8_t>(svF));
  f.u16(om_ma);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 修改闭环模式最大电流
 */
void DeviceEmmV5::modify_foc_ma(bool svF, uint16_t foc_ma)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x45);
  f.u8(0x66);
  f.u8(static_cast<uint8_t>(svF));
  f.u16(foc_ma);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 读取PID参数
 */
void DeviceEmmV5::read_pid_params()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x21);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 修改PID参数
 */
void DeviceEmmV5::modify_pid_params(bool svF, uint32_t kp, uint32_t ki, uint32_t kd)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x4A);
  f.u8(0xC3);
  f.u8(static_cast<uint8_t>(svF));
  f.u32(kp);
  f.u32(ki);
  f.u32(kd);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 读取DMX512协议参数（Y42）
 */
void DeviceEmmV5::read_dmx512_params()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x49);
  f.u8(0x78);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 修改DMX512协议参数（Y42）
 */
void DeviceEmmV5::modify_dmx512_params(bool svF, uint16_t tch, uint8_t nch, uint8_t mode, uint16_t vel, uint16_t acc, uint16_t vel_step, uint32_t pos_step)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0xD9);
  f.u8(0x90);
  f.u8(static_cast<uint8_t>(svF));
  f.u16(tch);
  f.u8(nch);
  f.u8(mode);
  f.u16(vel);
  f.u16(acc);
  f.u16(vel_step);
  f.u32(pos_step);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 读取位置到达窗口（Y42）
 */
void DeviceEmmV5::read_pos_window()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x41);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 修改位置到达窗口（Y42）
 */
void DeviceEmmV5::modify_pos_window(bool svF, uint16_t prw)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0xD1);
  f.u8(0x07);
  f.u8(static_cast<uint8_t>(svF));
  f.u16(prw);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 读取过热过流保护检测阈值（Y42）
 */
void DeviceEmmV5::read_otocp()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x13);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 修改过热过流保护检测阈值（Y42）
 */
void DeviceEmmV5::modify_otocp(bool svF, uint16_t otp, uint16_t ocp, uint16_t time_ms)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0xD3);
  f.u8(0x56);
  f.u8(static_cast<uint8_t>(svF));
  f.u16(otp);
  f.u16(ocp);
  f.u16(time_ms);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 读取心跳保护功能时间（Y42）
 */
void DeviceEmmV5::read_heart_protect()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x16);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 修改心跳保护功能时间（Y42）
 */
void DeviceEmmV5::modify_heart_protect(bool svF, uint32_t hp)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x68);
  f.u8(0x38);
  f.u8(static_cast<uint8_t>(svF));
  f.u32(hp);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 读取积分限幅/刚性系数（Y42）
 */
void DeviceEmmV5::read_integral_limit()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x23);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 修改积分限幅/刚性系数（Y42）
 */
void DeviceEmmV5::modify_integral_limit(bool svF, uint32_t il)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x4B);
  f.u8(0x57);
  f.u8(static_cast<uint8_t>(svF));
  f.u32(il);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/* ==================== 读取所有驱动参数命令 ==================== */

/**
 * @brief 读取系统状态参数
 */
void DeviceEmmV5::read_system_state_params()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x43);
  f.u8(0x7A);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/**
 * @brief 读取驱动配置参数
 */
void DeviceEmmV5::read_motor_conf_params()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x42);
  f.u8(0x6C);
  f.finish();
  _send_cmd(f.buf, f.len);
}


/* ==================== 多电机命令（MMCL）成员方法 ==================== */

/**
 * @brief 发送多电机命令（Y42）（实例方法）
 *
 * @note 将 MMCL 缓冲区中积累的所有命令一次性通过本实例的 UART 发送。
 *       发送后自动清空缓冲区。
 *
 * @param addr 电机地址（通常使用广播地址0）
 */
void DeviceEmmV5::send_multi_motor_cmd(uint8_t addr)
{
  if (_mmcl_count == 0)
  {
    return;
  }

  // 多电机命令的总字节数 = MMCL数据 + 5（地址、功能码、长度高、长度低、校验）
  uint16_t len = _mmcl_count + 5;

  // 直接在成员缓冲区上构建完整帧（避免栈上 517B 大数组）
  memmove(_mmcl_buf + 4, _mmcl_buf, _mmcl_count); // MMCL 数据后移 4 字节，腾出头部

  _mmcl_buf[0]               = addr;
  _mmcl_buf[1]               = 0xAA;
  _mmcl_buf[2]               = static_cast<uint8_t>(len >> 8);
  _mmcl_buf[3]               = static_cast<uint8_t>(len >> 0);
  _mmcl_buf[4 + _mmcl_count] = EMMV5_CHECKSUM;

  // 通过本实例的 UART 发送
  _uart.send(_mmcl_buf, len, 0);

  _mmcl_count = 0; // 发送后清空缓冲区
}


/**
 * @brief 清空多电机命令缓冲区
 */
void DeviceEmmV5::mmcl_clear()
{
  _mmcl_count = 0;
}


/* ==================== MMCL 触发动作命令 ==================== */

void DeviceEmmV5::mmcl_trig_encoder_cal()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x06);
  f.u8(0x45);
  f.finish();
  _mmcl_append(f.buf, f.len);
}


void DeviceEmmV5::mmcl_reset_motor()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x08);
  f.u8(0x97);
  f.finish();
  _mmcl_append(f.buf, f.len);
}


void DeviceEmmV5::mmcl_reset_curpos_to_zero()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x0A);
  f.u8(0x6D);
  f.finish();
  _mmcl_append(f.buf, f.len);
}


void DeviceEmmV5::mmcl_reset_clog_pro()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x0E);
  f.u8(0x52);
  f.finish();
  _mmcl_append(f.buf, f.len);
}


void DeviceEmmV5::mmcl_restore_motor()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x0F);
  f.u8(0x5F);
  f.finish();
  _mmcl_append(f.buf, f.len);
}


/* ==================== MMCL 运动控制命令 ==================== */

void DeviceEmmV5::mmcl_en_control(bool state, bool snF)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0xF3);
  f.u8(0xAB);
  f.u8(static_cast<uint8_t>(state));
  f.u8(static_cast<uint8_t>(snF));
  f.finish();
  _mmcl_append(f.buf, f.len);
}


void DeviceEmmV5::mmcl_vel_control(uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0xF6);
  f.u8(dir);
  f.u16(vel);
  f.u8(acc);
  f.u8(static_cast<uint8_t>(snF));
  f.finish();
  _mmcl_append(f.buf, f.len);
}


void DeviceEmmV5::mmcl_pos_control(uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF, bool snF)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0xFD);
  f.u8(dir);
  f.u16(vel);
  f.u8(acc);
  f.u32(clk);
  f.u8(raF);
  f.u8(static_cast<uint8_t>(snF));
  f.finish();
  _mmcl_append(f.buf, f.len);
}


void DeviceEmmV5::mmcl_set_qpos_params(uint16_t vel, uint8_t acc, uint8_t raF, bool snF)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0xF1);
  f.u16(vel);
  f.u8(acc);
  f.u8(raF);
  f.u8(static_cast<uint8_t>(snF));
  f.finish();
  _mmcl_append(f.buf, f.len);
}


void DeviceEmmV5::mmcl_qpos_control(int32_t clk)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0xFC);
  f.u32(static_cast<uint32_t>(clk));
  f.finish();
  _mmcl_append(f.buf, f.len);
}


void DeviceEmmV5::mmcl_stop_now(bool snF)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0xFE);
  f.u8(0x98);
  f.u8(static_cast<uint8_t>(snF));
  f.finish();
  _mmcl_append(f.buf, f.len);
}


void DeviceEmmV5::mmcl_synchronous_motion()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0xFF);
  f.u8(0x66);
  f.finish();
  _mmcl_append(f.buf, f.len);
}


/* ==================== MMCL 原点回零命令 ==================== */

void DeviceEmmV5::mmcl_origin_set_o(bool svF)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x93);
  f.u8(0x88);
  f.u8(static_cast<uint8_t>(svF));
  f.finish();
  _mmcl_append(f.buf, f.len);
}


void DeviceEmmV5::mmcl_origin_trigger_return(uint8_t o_mode, bool snF)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x9A);
  f.u8(o_mode);
  f.u8(static_cast<uint8_t>(snF));
  f.finish();
  _mmcl_append(f.buf, f.len);
}


void DeviceEmmV5::mmcl_origin_interrupt()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x9C);
  f.u8(0x48);
  f.finish();
  _mmcl_append(f.buf, f.len);
}


void DeviceEmmV5::mmcl_origin_modify_params(bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x4C);
  f.u8(0xAE);
  f.u8(static_cast<uint8_t>(svF));
  f.u8(o_mode);
  f.u8(o_dir);
  f.u16(o_vel);
  f.u32(o_tm);
  f.u16(sl_vel);
  f.u16(sl_ma);
  f.u16(sl_ms);
  f.u8(static_cast<uint8_t>(potF));
  f.finish();
  _mmcl_append(f.buf, f.len);
}


void DeviceEmmV5::mmcl_origin_read_sl_rp()
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x3F);
  f.finish();
  _mmcl_append(f.buf, f.len);
}


void DeviceEmmV5::mmcl_origin_modify_sl_rp(bool svF, uint16_t sl_rp)
{
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x5C);
  f.u8(0xAC);
  f.u8(static_cast<uint8_t>(svF));
  f.u16(sl_rp);
  f.finish();
  _mmcl_append(f.buf, f.len);
}


/* ==================== MMCL 读取系统参数命令 ==================== */

void DeviceEmmV5::mmcl_auto_return_sys_params_timed(EmmSysParam s, uint16_t time_ms)
{
  // 表驱动构建后追加到成员 MMCL 缓冲
  EmmFrame f;
  f.u8(_addr);
  f.u8(0x11);
  f.u8(0x18);
  f.u8(sys_param_code(s));
  f.u16(time_ms);
  f.finish();
  _mmcl_append(f.buf, f.len);
}


void DeviceEmmV5::mmcl_read_sys_params(EmmSysParam s)
{
  // 表驱动构建后追加到成员 MMCL 缓冲
  EmmFrame f;
  f.u8(_addr);
  f.u8(sys_param_code(s));
  f.finish();
  _mmcl_append(f.buf, f.len);
}


/* ==================================================================
 *  接收与到位检测
 * ================================================================== */

/**
 * @brief 扫描缓冲中是否存在"到位返回帧"（地址 + 0xFD 0x9F 0x6B）
 * @param buf 接收缓冲
 * @param n   有效字节数
 * @return true=检测到到位帧
 */
bool DeviceEmmV5::scan_in_position(const uint8_t *buf, int n)
{
  for (int i = 0; i <= n - 4; i++)
  {
    if (buf[i] == _addr && buf[i + 1] == EMMV5_INPOS_MARK1
        && buf[i + 2] == EMMV5_INPOS_MARK2 && buf[i + 3] == EMMV5_INPOS_MARK3)
    {
      return true; // 到位帧
    }
  }
  return false;
}

/**
 * @brief 在流缓冲区基础上做滑动窗口装配
 * @note 只有真正凑齐完整 4 字节帧（addr + FD 9F 6B）才判定命中；
 *       命中后正确清空窗口，并记录该帧的相对到达时间。
 */
bool DeviceEmmV5::feed_rx(const uint8_t *data, size_t n)
{
  for (size_t i = 0; i < n; i++)
  {
    // 窗口已满：滚动保留后 3 字节，再写入新字节
    if (_rx_assem_len == 4)
    {
      memmove(_rx_assem, _rx_assem + 1, 3);
      _rx_assem_len = 3;
    }
    _rx_assem[_rx_assem_len++] = data[i];

    // 只有真正凑齐完整 4 字节帧才命中（addr + FD 9F 6B）
    if (_rx_assem_len == 4 && _rx_assem[0] == _addr && _rx_assem[1] == EMMV5_INPOS_MARK1 && _rx_assem[2] == EMMV5_INPOS_MARK2 && _rx_assem[3] == EMMV5_INPOS_MARK3)
    {
      _rx_assem_len    = 0;                          // 正确清空窗口
      _last_inpos_tick = xTaskGetTickCountFromISR(); // 记录相对到达时间
      return true;                                   // 命中，给信号量
    }
  }
  return false;
}


/* ==================================================================
 *  到位信号量绑定
 * ================================================================== */

/**
 * @brief 绑定到位信号量（device_cfg 初始化时调用）
 * @param sem 到位信号量句柄（nullptr 时不 give）
 */
void DeviceEmmV5::set_in_pos_sem(SemaphoreHandle_t sem)
{
  _in_pos_sem = sem;
}


/* ==================================================================
 *  静态实例注册表（仿 BspUart）
 * ================================================================== */

/**
 * @brief 构造时注册实例到注册表
 */
Status DeviceEmmV5::register_instance()
{
  if (_instance_count >= MAX_INSTANCES)
  {
    return Status::FULL; // 注册表已满
  }
  _instances[_instance_count] = this;
  _instance_count++;
  return Status::OK;
}

size_t DeviceEmmV5::instance_count()
{
  return _instance_count;
}

DeviceEmmV5 *DeviceEmmV5::get_instance_by_index(size_t i)
{
  return (i < _instance_count) ? _instances[i] : nullptr;
}


/* ==================================================================
 *  统一到位接收任务 —— 每路电机一个任务，统一入口（arg = this）
 *
 *  到位返回帧: 地址 + 0xFD 0x9F 0x6B（4 字节）
 *  检测到后 give 本实例绑定的到位信号量（device_cfg 中 set_in_pos_sem）
 * ================================================================== */

void DeviceEmmV5::rx_task_entry(void *arg)
{
  DeviceEmmV5 *m = static_cast<DeviceEmmV5 *>(arg);
  uint8_t      buf[8];

  for (;;)
  {
    size_t n = 0;
    Status s = m->receive_raw(buf, sizeof(buf), &n, portMAX_DELAY);

    // 基于流缓冲区的滑动窗口装配，真正收到完整帧才命中
    if (s == Status::OK && n > 0 && m->feed_rx(buf, n) && m->_in_pos_sem != nullptr)
    {
      xSemaphoreGive(m->_in_pos_sem); // 电机到位
    }
  }
}


/* ==================================================================
 *  创建到位接收任务（device_init 调用）
 *
 *  任务栈 256 words (1KB)，优先级 idle+5（与电机搬运任务同级）
 * ================================================================== */

void DeviceEmmV5::create_rx_tasks()
{
  char name[16];
  for (size_t i = 0; i < _instance_count; i++)
  {
    snprintf(name, sizeof(name), "m_rx%u", _instances[i]->_addr);
    xTaskCreate(rx_task_entry, name, 256, _instances[i], tskIDLE_PRIORITY + 5, NULL);
  }
}
