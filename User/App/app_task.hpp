/**
 * @file app_task.hpp
 * @author Rh
 * @brief 应用层任务声明 —— 除默认任务外的业务任务
 * @version 0.1
 * @date 2026-08-14
 *
 * @copyright Copyright (c) 2026
 *
 * @note 任务函数均以 extern "C" 声明（FreeRTOS 以 C 方式调用）。
 *       任务实现位于 User/App/task/，由 all_init() 统一创建。
 */

#ifndef __APP_TASK_HPP__
#define __APP_TASK_HPP__

/**
 * @brief CoreXY 输入源：1 = 上位机二进制协议（`corexy_task`），0 = 命令码终端（`corexy_console_task`）
 * @note 两者都占用 USART1，只能开一个。默认 0：串口协议暂不启用，用终端手工测试。
 * @note 执行侧（`corexy_io_task`）与应答上报也跟随本开关切换。
 */
#define APP_COREXY_PROTOCOL 0


#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @brief 系统级周期维护任务（10 ms）：CAN / UART 的补救巡检 + Online 离线计时
   * @param argument 任务参数（未使用，NULL）
   * @note CAN 的日常收发由各设备自己的任务负责，本任务只做 10 ms 级的补救：
   *       补丢唤醒的发送、Bus-Off 恢复、串口收发通道重建。
   * @note 由 all_init() 创建，不应由业务代码直接调用。
   */
  void sys_task(void *argument);

  /**
   * @brief 按键任务：200 ms 轮询 key_user，按事件驱动蜂鸣器提示
   * @param argument 任务参数（未使用，NULL）
   * @note 轮询周期与 key_user 的消抖 / 长按配置对应（debounce 1 → 200 ms，
   *       long_press 5 → 1 s）；beep 为阻塞调用，实际周期会多出一个响铃时长。
   * @note 由 all_init() 创建，不应由业务代码直接调用。
   */
  void key_task(void *argument);

  /**
   * @brief CoreXY 上位机协议入口（阻塞读串口，默认 10 ms 超时）
   * @param argument 任务参数（未使用，NULL）
   * @note 只做字节搬运：`bsp_uart1.receive()` → `cxy::feed()`，解析出的命令进队列。
   * @note 帧格式与命令表见 `corexy_protocol.hpp`、`corexy.md` §6。
   * @note 由 all_init() 创建，不应由业务代码直接调用。
   */
  void corexy_task(void *argument);

  /**
   * @brief CoreXY 命令码终端任务（`APP_COREXY_PROTOCOL == 0` 时使用）
   * @param argument 任务参数（未使用，NULL）
   * @note 10 ms 收 USART1 字符，按行解析 ASCII 命令，投递到 `corexy_cmd_queue`；不碰电机总线。
   * @note 行结束符 CR / LF / `;`；命令表见文件头与 `corexy.md`。
   * @note 由 all_init() 创建，不应由业务代码直接调用。
   */
  void corexy_console_task(void *argument);

  /**
   * @brief CoreXY 执行任务：独占电机总线，执行队列命令并回帧
   * @param argument 任务参数（未使用，NULL）
   * @note 上电完成停止/清零/使能 + 原点复核，之后 10 ms 循环：取队列命令 + `corexy.update()` +
   *       状态变化上报事件。
   * @note 本任务同时负责 `emm_group.update()`（这条总线唯一的接收消费者）。
   * @note 由 all_init() 创建，不应由业务代码直接调用。
   */
  void corexy_io_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif // __APP_TASK_HPP__
