/**
 * @file can_rx_node.hpp
 * @author Rh
 * @brief CAN 接收节点：按「总线 + 帧类型 + ID」注册一个回调
 * @version 0.2
 * @date 2026-10-01
 *
 * @copyright Copyright (c) 2026
 *
 * @note 节点注册表在系统进入运行态之前构建，运行期间只读，
 *       因此注册/注销只能由单一初始化任务调用（CanBus::frozen 之后一律拒绝）。
 * @note 所有成员对 Service/App 层公开（不使用 friend）；外部请勿直接改动。
 *
 * @note 使用示例（初始化阶段，单任务调用）：
 *
 *      CanRxNode *node = regist({&bus_can1, 0x201, my_callback, this});
 */

#ifndef __CAN_RX_NODE_HPP__
#define __CAN_RX_NODE_HPP__

#include "bsp_can.hpp"

class CanBus;


/**
 * @brief CAN 接收节点
 */
class CanRxNode
{
public:
  /** @note 在接收分发任务中同步调用，应短且不阻塞；rx 仅在回调期间有效。 */
  using Callback = Status (*)(void *context, const CanRxMsg &rx);

  /**
   * @brief 接收节点配置（可匿名按序传入）
   */
  struct Config
  {
    /**
     * @brief 按序构造配置（参数顺序 = 字段顺序）
     */
    Config(CanBus  *bus      = nullptr,
           uint32_t can_id   = 0,
           Callback callback = nullptr,
           void    *context  = nullptr,
           uint32_t id_type  = FDCAN_STANDARD_ID) : bus(bus),
                                                   can_id(can_id),
                                                   callback(callback),
                                                   context(context),
                                                   id_type(id_type)
    {
    }

    CanBus  *bus;      ///< 节点所属总线
    uint32_t can_id;   ///< 关心的帧 ID（标准帧 11 位 / 扩展帧 29 位）
    Callback callback; ///< 命中后的回调，不可为空
    void    *context;  ///< 回调上下文，必须保持有效
    uint32_t id_type;  ///< FDCAN_STANDARD_ID 或 FDCAN_EXTENDED_ID
  };

  uint32_t   _can_id;   ///< 节点关心的帧 ID
  uint32_t   _id_type;  ///< FDCAN_STANDARD_ID 或 FDCAN_EXTENDED_ID
  Callback   _callback; ///< 命中后的回调
  void      *_context;  ///< 回调上下文
  CanRxNode *next;      ///< 同一总线上的下一个节点

  static uint8_t node_num; ///< 已注册节点总数（上限 50）

  /**
   * @brief 构造一个接收节点
   * @param cfg 节点配置（总线/ID/回调/上下文/帧类型）
   * @note 请用 regist() 创建并挂到总线上，不要直接 new。
   */
  explicit CanRxNode(const Config &cfg);

  ~CanRxNode() = default;

  CanRxNode(const CanRxNode &)            = delete;
  CanRxNode &operator=(const CanRxNode &) = delete;
  CanRxNode(CanRxNode &&)                 = delete;
  CanRxNode &operator=(CanRxNode &&)      = delete;
};


/**
 * @brief 注册接收节点（初始化阶段调用）
 *
 * @param cfg 节点配置：总线、标准帧 11 位或扩展帧 29 位 ID、非空回调、
 *            回调上下文（可为空，必须保持有效）、帧类型
 * @return 成功返回节点；失败返回 nullptr，并把原因记入系统初始化错误
 *
 * @note 同一总线上「帧类型 + ID」相同的节点不允许重复注册；最多 50 个节点。
 */
CanRxNode *regist(const CanRxNode::Config &cfg);

/**
 * @brief 注销并销毁接收节点（初始化阶段调用）
 *
 * @param bus  节点所属总线
 * @param node 注册时返回的节点指针；成功后调用方必须置空
 * @return OK=成功；BAD_ARG=节点不属于该总线；NOT_SUPPORTED=已冻结，节点保持有效
 * @note 运行期不支持注销；必须在销毁回调上下文之前注销成功。
 */
Status unregist(CanBus &bus, CanRxNode *node);


#endif // __CAN_RX_NODE_HPP__
