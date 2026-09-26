/**
 * @file online_check.hpp
 * @author ChoseB
 * @brief 通用设备在线状态检查
 * @version 0.1
 * @date 2026-08-14
 *
 * @copyright Copyright (c) 2026
 *
 * @details 每个 Online 对象作为一个节点自动注册到内部链表。设备收到有效数据时，
 *          根据调用上下文执行 refresh_task() 或 refresh_isr()；系统周期性调用
 *          update()（当前为 10 ms，见 sys_task），由其按真实经过的时间更新在线状态。
 *
 * @note timeout_gap 的单位是【毫秒】，与 update() 的调用周期无关 —— update() 内部
 *       用 tick 差值累计真实经过的毫秒数，改变调用周期不会影响超时判定。
 *       要求 configTICK_RATE_HZ == 1000（1 tick = 1 ms）。
 *
 * @note 默认阈值 30 ms（项目内电机均使用此默认值）。实际离线发现延迟为
 *       timeout_gap ~ timeout_gap + T（T = update() 周期，10 ms 时约 30~40 ms）。
 * @warning refresh_isr()、update() 及对象构造/析构可能并发访问状态或链表，
 *          实现时必须使用与调用上下文匹配的临界区保护。
 * @note 饱和离线计时、链表维护及任务/ISR 同步逻辑实现在 online_check.cpp。
 */
#ifndef __SERVICE_ONLINE_CHECK_HPP__
#define __SERVICE_ONLINE_CHECK_HPP__

#include "status.hpp"

#include <stdint.h>

/**
 * @brief 可嵌入任意设备对象的在线状态检查节点
 *
 * @details refresh_*() 表示设备刚收到一份有效数据；update() 负责推进所有节点的
 *          离线计时。对象不可复制或移动，以保证内部链表节点地址稳定。
 */
class Online
{
private:
  uint16_t _cnt;         ///< 距上次有效刷新的毫秒数（饱和于 UINT16_MAX）
  uint16_t _timeout_gap; ///< 离线判定阈值（毫秒）
  Status   _statu;       ///< Status::OK 表示在线，Status::TIMEOUT 表示离线

  Online *_next;        ///< 内部单向链表的后继节点
  static Online *_head; ///< 在线检查链表头
  static Online *_tail; ///< 在线检查链表尾，用于常数时间追加节点

public:
  /**
   * @brief 构造在线检查节点并注册到内部链表
   *
   * @param timeout_gap 连续多少毫秒未刷新后判定离线，默认 30（即 30 ms）
   *                    —— 电机对象用的就是此默认值：30 ms 无有效反馈即判离线
   *
   * @note 新对象初始状态为 Status::TIMEOUT。
   */
  Online(uint16_t timeout_gap = 30);

  /**
   * @brief 从内部链表注销本节点
   * @warning 若系统允许在调度器启动后销毁对象，注销操作必须与 update() 互斥。
   */
  ~Online();

  /**
   * @brief 在任务上下文中记录一次有效设备数据
   * @return Status::OK 刷新成功。
   * @note 将离线计时清零并把状态更新为 Status::OK。
   */
  Status refresh_task(void);

  /**
   * @brief 在中断上下文中记录一次有效设备数据
   * @return Status::OK 刷新成功。
   * @note 必须使用 ISR 安全的临界区实现，不得调用会阻塞的 RTOS API。
   */
  Status refresh_isr(void);

  /**
   * @brief 查询最近一次计算得到的在线状态
   * @return Status::OK 设备在线；Status::TIMEOUT 设备已超时离线。
   */
  Status isOnline(void) const;

  /**
   * @brief 推进全部在线检查节点的离线计时（按真实经过的毫秒累计）
   * @return Status::OK 遍历完成；其他状态表示内部链表异常。
   * @note 由单一任务周期性调用（10 ms），不能在 ISR 中调用。
   *       内部用 tick 差值计算经过时间，因此调用周期变化不影响超时精度。
   */
  static Status update(void);

  /** @name 禁止复制和移动，保护链表节点身份 */
  /** @{ */
  Online(const Online &) = delete;
  Online &operator=(const Online &) = delete;
  Online(Online &&) = delete;
  Online &operator=(Online &&) = delete;
  /** @} */
};

#endif // __SERVICE_ONLINE_CHECK_HPP__
