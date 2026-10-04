/**
 * @file status.hpp
 * @author Rh
 * @brief 统一状态码 Status 与可按场景多实例的事件状态 EventState
 * @version 0.8
 * @date 2026-10-04
 *
 * @copyright Copyright (c) 2026
 *
 * @details 一个 EventState 实例 = 一个事件组 = 一份「阶段 + 错误登记」。按场景创建多个实例
 *          互不影响（整个 MCU 用 sys_state，某条总线要单独看就再建一个）。
 *
 * @note 位分配（每个实例各一份，共 24 位）：
 *       bit 0 = OK 标记，bit 1~21 = 状态码位（位号即 Status 枚举值，掩码用 status_bit()），
 *       bit 22 = RUNNING_BIT 运行位，bit 23 = INIT_FAIL_BIT 初始化失败位（粘滞，只置不清）。
 *
 * @note 两族登记接口：
 *       - error(s)　　　运行期错误：只记码 + 清 OK 位，不动运行位，running() 仍为 true；
 *       - init_error(s)　初始化错误：记码 + 置 INIT_FAIL_BIT + 清运行位，此后永不进入运行态。
 *
 * @note 正常流程（顺序不能错）：
 *       1. init() 建事件组，只由 service_init() 调一次；
 *       2. 系统内的各个组成部分初始化，任一步失败就 init_error()；
 *       3. 全部就绪后 complete_init() 置运行位（有过失败记录则返回 NOT_SUPPORTED 且不置位）；
 *       4. 任务入口第一句 wait_running()：阻塞到运行位；未 init 或等到失败位时任务自删，不返回。
 *
 * @note 未 init 时除 init() 外全部接口安全返回：登记类返回 NOT_INIT，查询类返回 false /
 *       NOT_INIT，wait_running() 删任务。
 *
 * @note 分层约定：只给「维护多个对象的系统」用；单个设备不许引用它，失败只 return Status。
 */

#ifndef __SERVICE_STATUS_HPP__
#define __SERVICE_STATUS_HPP__

#include <stdint.h>

#include "FreeRTOS.h" // IWYU pragma: keep
#include "event_groups.h"


/**
 * @brief 统一状态码
 *
 * @note 枚举值即事件组位号，因此**顺序不可随意调整**（调整会改变对外可观测行为）。
 */
enum class Status : uint8_t
{
  OK = 0,        ///< 成功
  BUSY,          ///< 资源被占用（DMA 正在发送等）
  TIMEOUT,       ///< 等待超时
  FULL,          ///< 缓冲满
  IO_ERROR,      ///< 硬件/传输错误
  BAD_ARG,       ///< 参数非法
  NOT_INIT,      ///< 资源未初始化（未调用 init 或创建失败）
  NOT_SUPPORTED, ///< 不支持的操作/模式
};

/**
 * @brief 取某个状态码对应的事件组位掩码
 * @param statu 状态码
 * @return 该状态码对应的位掩码
 */
constexpr uint32_t status_bit(Status statu)
{
  return 1U << static_cast<uint8_t>(statu);
}


/**
 * @brief 事件状态：一个实例 = 一个事件组 = 一份「阶段 + 错误登记」
 *
 * @note 实例数量不限，按场景创建；各实例持有自己的事件组，互不影响。
 * @note 必须先 init() 才能登记/等待；未 init 时所有接口安全返回，不会触碰空句柄。
 */
class EventState
{
public:
  // ---------------- 位分配 ----------------

  static constexpr uint32_t RUNNING_BIT   = 1U << 22; ///< 本实例已进入运行态
  static constexpr uint32_t INIT_FAIL_BIT = 1U << 23; ///< 本实例初始化失败过（粘滞，不清除）
  static constexpr uint32_t STATUS_MAX    = 22U;      ///< 状态码只能用 bit 0 ~ 21，不得侵入系统标志位

  // ----------------
  // ---------------- 构造与析构 ----------------

  /** @brief 默认构造：未创建事件组，须再调 init() */
  EventState() = default;

  /** @brief 释放事件组；未 init 时无副作用 */
  ~EventState();

  // 句柄所有权唯一：禁止拷贝，避免两个实例持有同一个事件组
  EventState(const EventState &)            = delete;
  EventState &operator=(const EventState &) = delete;

  // ----------------
  // ---------------- 生命周期 ----------------

  /**
   * @brief 创建本实例的事件组（幂等：已创建过则直接返回 OK）
   * @return OK=可用；FULL=创建失败
   */
  Status init(void);

  /**
   * @brief 初始化全部完成：置运行位
   * @return OK=已置运行位；NOT_SUPPORTED=此前已有初始化失败记录，运行位不置位；NOT_INIT=未 init
   */
  Status complete_init(void);

  // ----------------
  // ---------------- 错误登记 ----------------

  /**
   * @brief 记录一个错误码，不影响运行位（运行期错误）
   * @param statu 错误状态码（不允许传 OK）
   * @return OK=已记录；BAD_ARG=传了 OK；NOT_INIT=未 init
   */
  Status error(Status statu);

  /**
   * @brief 记录初始化失败：记错误码 + 置粘滞失败位 + 清运行位
   * @param statu 错误状态码（不允许传 OK）
   * @return error() 的结果；NOT_INIT=未 init
   * @note 运行位被清且失败位粘滞，本实例此后不会再进入运行态。
   */
  Status init_error(Status statu);

  // ----------------
  // ---------------- 查询与等待 ----------------

  /** @brief 事件组是否已创建；未 init 时登记/等待接口一律安全返回 */
  bool is_ready(void) const;

  /** @brief 本实例当前是否处于运行态 */
  bool running(void) const;

  /**
   * @brief 已记录的最小错误码
   * @return 错误状态码；从未记录过错误时返回 Status::OK；未 init 时返回 Status::NOT_INIT
   * @note 位号即枚举值且从小到大扫，因此多个错误并存时返回枚举值最小的那个。
   */
  Status error_code(void) const;

  /**
   * @brief 阻塞等待本实例进入运行态
   * @note **只能在任务中调用**：未 init、或等到的是「初始化失败」时，函数会删除当前任务（不会返回）。
   */
  void wait_running(void) const;

  // ----------------
private:
  // ---------------- 成员变量 ----------------

  EventGroupHandle_t _handle = nullptr; ///< 本实例的事件组，未 init 时为 nullptr

  // ----------------
};

// 状态码不得侵入 bit 22~23（系统标志位）；新增状态码越界时在编译期就报错
static_assert(static_cast<uint32_t>(Status::NOT_SUPPORTED) < EventState::STATUS_MAX, "Status 取值与系统标志位重叠");

#endif // __SERVICE_STATUS_HPP__
