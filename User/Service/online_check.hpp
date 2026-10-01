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
 *          根据调用上下文执行 refresh_task() 或 refresh_isr()；系统以固定 1 kHz
 *          调用 update()，累计各对象距上次刷新的时间并更新在线状态。
 *
 * @note timeout_gap 的单位是 update() 调用次数；在 1 kHz 更新频率下等价于毫秒。
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
  uint16_t _cnt;         ///< 距上次有效刷新的 update() 次数；实现时必须饱和递增
  uint16_t _timeout_gap; ///< 离线判定阈值，单位为 update() 调用次数
  Status   _statu;       ///< Status::OK 表示在线，Status::TIMEOUT 表示离线

  Online *_next;        ///< 内部单向链表的后继节点
  static Online *_head; ///< 在线检查链表头
  static Online *_tail; ///< 在线检查链表尾，用于常数时间追加节点

public:
  /**
   * @brief 构造在线检查节点并注册到内部链表
   *
   * @param timeout_gap 连续多少次 update() 未刷新后判定离线，默认 30；
   *                    在 1 kHz 更新频率下即 30 ms
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
  Status is_online(void) const;

  /**
   * @brief 推进全部在线检查节点的离线计时
   * @return Status::OK 遍历完成；其他状态表示内部链表异常。
   * @note 应由单一任务以固定 1 kHz 频率调用，不能在 ISR 中调用。
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
