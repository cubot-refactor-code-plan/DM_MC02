/**
 * @file online_check.hpp
 * @author ChoseB
 * @brief 通用设备在线状态检查
 * @version 0.2
 * @date 2026-10-04
 *
 * @copyright Copyright (c) 2026
 *
 * @details 每个 Online 对象作为节点自动挂到内部链表；设备收到有效数据时按上下文调
 *          refresh_task() / refresh_isr()，再由周期任务调 update() 推进离线计时。
 *
 * @note timeout_gap 单位是毫秒，与 update() 的调用周期无关；调用周期只决定离线判定的
 *       延迟上限，应不大于最小的 timeout_gap。
 */
#ifndef __SERVICE_ONLINE_CHECK_HPP__
#define __SERVICE_ONLINE_CHECK_HPP__

#include "FreeRTOS.h" // IWYU pragma: keep (TickType_t / pdMS_TO_TICKS)
#include "status.hpp"

#include <stdint.h>

/**
 * @brief 可嵌入任意设备对象的在线状态检查节点
 *
 * @note 对象不可复制或移动，以保证内部链表节点地址稳定。
 */
class Online
{
private:
  TickType_t _last_refresh_tick; ///< 上次有效刷新的 tick
  uint16_t   _timeout_gap;       ///< 离线判定阈值，单位：毫秒
  Status     _statu;             ///< Status::OK 表示在线，Status::TIMEOUT 表示离线

  Online        *_next; ///< 内部单向链表的后继节点
  static Online *_head; ///< 在线检查链表头
  static Online *_tail; ///< 在线检查链表尾，用于常数时间追加节点

public:
  /**
   * @brief 构造在线检查节点并注册到内部链表
   * @param timeout_gap 距上次有效刷新超过多少毫秒判定离线，默认 30
   * @note 初始状态为 Status::TIMEOUT，收到第一帧有效数据后转为在线。
   */
  Online(uint16_t timeout_gap = 30);

  /**
   * @brief 从内部链表注销本节点
   * @warning 若允许在调度器启动后销毁对象，注销必须与 update() 互斥
   */
  ~Online();

  /**
   * @brief 在任务上下文中记录一次有效设备数据
   * @return Status::OK 刷新成功
   */
  Status refresh_task(void);

  /**
   * @brief 在中断上下文中记录一次有效设备数据
   * @return Status::OK 刷新成功
   * @note 必须用 ISR 安全的临界区实现，不得调用会阻塞的 RTOS API。
   */
  Status refresh_isr(void);

  /**
   * @brief 查询最近一次计算得到的在线状态
   * @return Status::OK 设备在线；Status::TIMEOUT 设备已超时离线
   */
  Status is_online(void) const;

  /**
   * @brief 推进全部在线检查节点的离线计时
   * @return Status::OK 遍历完成
   * @note 只能在单个周期任务中调用（当前为 sys_task，10 ms 一次），不能在 ISR 中调用。
   */
  static Status update(void);

  /** @name 禁止复制和移动，保护链表节点身份 */
  /** @{ */
  Online(const Online &)            = delete;
  Online &operator=(const Online &) = delete;
  Online(Online &&)                 = delete;
  Online &operator=(Online &&)      = delete;
  /** @} */
};

#endif // __SERVICE_ONLINE_CHECK_HPP__
