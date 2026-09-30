#ifndef CAN_RX_NODE_HPP
#define CAN_RX_NODE_HPP

#include "bsp_can.hpp"

/** @brief 轮询各 CAN 原始接收缓冲，分发成功则消费，否则送入回退缓冲。 */
extern "C" void can_rx_task(void *argument);

/**
 * @brief 初始化阶段注册、运行阶段只读的 CAN 接收节点。
 * @note 注册/注销仅由单一初始化任务执行；CAN 对象和 context 必须保持有效。
 */
class CanRxNode
{
public:
  /** @note 在接收任务中同步调用，应短且不阻塞；rx 仅在回调期间有效。 */
  using Callback = Status (*)(void *context, const CanRxMsg &rx);

private:
  uint32_t   _can_id;
  uint32_t   _id_type;
  Callback   _callback;
  void      *_context;
  CanRxNode *next;

  static uint8_t       node_num;
  static bool          frozen;

  CanRxNode(uint32_t can_id, Callback callback, void *context, uint32_t id_type);
  ~CanRxNode() = default;
  // 记忆运行状态，防止运行标志清除后重新修改注册表。
  static bool is_frozen(void);

  friend CanRxNode *regist(BspCan &can_item, uint32_t can_id, Callback callback, void *context, uint32_t id_type);
  friend Status unregist(BspCan &can_item, CanRxNode *node);
  friend void can_rx_task(void *argument);

public:
  CanRxNode(const CanRxNode &) = delete;
  CanRxNode &operator=(const CanRxNode &) = delete;
  CanRxNode(CanRxNode &&) = delete;
  CanRxNode &operator=(CanRxNode &&) = delete;
};

/**
 * @brief 初始化阶段注册接收节点，相同 CAN、帧类型和 ID 不允许重复。
 * @param can_item 所用 CAN 对象。
 * @param can_id 标准帧 11 位或扩展帧 29 位 ID。
 * @param callback 非空接收回调。
 * @param context 回调对象指针，可为空，必须保持有效。
 * @param id_type FDCAN_STANDARD_ID（默认）或 FDCAN_EXTENDED_ID。
 * @return 成功返回节点；失败返回 nullptr，参数/冲突/资源错误记入系统初始化错误。
 * @note 必须在 SysFlagInit() 后、系统运行前由单一初始化任务调用，最多 50 个节点。
 */
CanRxNode *regist(BspCan &can_item, uint32_t can_id, CanRxNode::Callback callback, void *context = nullptr, uint32_t id_type = FDCAN_STANDARD_ID);

/**
 * @brief 初始化阶段注销并销毁接收节点。
 * @param can_item 注册节点所属的 CAN 对象。
 * @param node 注册返回的节点指针；成功后调用方必须置空。
 * @return OK 成功；BAD_ARG 指针无效；NOT_SUPPORTED 已冻结，节点保持有效。
 * @note 不支持运行期注销；必须在销毁回调上下文之前注销成功。
 */
Status unregist(BspCan &can_item, CanRxNode *node);

#endif // CAN_RX_NODE_HPP
