/**
 * @file can_tx_node.hpp
 * @author Rh
 * @brief CAN 发送节点：把一帧 8 字节数据切成 division 个等长槽位，供多个对象共用
 * @version 0.2
 * @date 2026-10-01
 *
 * @copyright Copyright (c) 2026
 *
 * @details 多个对象（比如同一控制帧下的 4 个电机）各自得到一个槽位，
 *          填数据后由 can_tx_task 按 1 ms 周期统一发出。
 *
 * @note 所有成员对 Service/App 层公开（不使用 friend）；外部请勿直接改动。
 *
 * @note 使用示例（初始化阶段，单任务调用）：
 *
 *      CanTxNode *node = regist({&bus_can1, 0x200, 4}, slot); // 占 4 槽位帧中的 1 个
 *      node->fill_data(tx_data, slot);
 */

#ifndef __CAN_TX_NODE_HPP__
#define __CAN_TX_NODE_HPP__

#include "bsp_can.hpp"

class CanBus;


/**
 * @brief CAN 发送节点
 */
class CanTxNode
{
public:
  /**
   * @brief 发送节点配置（可匿名按序传入）
   */
  struct Config
  {
    /**
     * @brief 按序构造配置（参数顺序 = 字段顺序）
     */
    Config(CanBus *bus = nullptr, uint32_t can_id = 0, uint8_t division = 1) :
      bus(bus), can_id(can_id), division(division)
    {
    }

    CanBus  *bus;      ///< 节点所属总线
    uint32_t can_id;   ///< 标准帧 ID（11 位）
    uint8_t  division; ///< 帧内等长槽位数（1/2/4/8）
  };

  CanTxMsg          txBuffer;       ///< 消息本体（std_id + 8 字节数据）
  uint8_t           division;       ///< 同一帧划分的等长数据槽位数（1/2/4/8）
  Status            _statu;         ///< 构造校验或初始化状态
  uint8_t           bufferRegister; ///< 槽位被注册占用情况（位图）
  uint8_t           bufferUnsend;   ///< 已填数据但尚未发出的槽位（位图）
  SemaphoreHandle_t bufferMutex;    ///< 保护槽位与数据的互斥量
  bool              _registered;    ///< 是否已加入总线的发送链表
  CanTxNode        *next;           ///< 同一总线上的下一个节点
  TickType_t        last_send_tick; ///< 上次成功发送时刻（发送任务启动时初始化）

  static uint8_t node_num; ///< 已注册节点总数（上限 50）

  /**
   * @brief 构造一个发送节点（只做参数校验与赋值）
   * @param cfg 节点配置（总线/ID/槽位数）
   * @note 请用 regist() 创建并挂到总线上，不要直接 new。
   */
  explicit CanTxNode(const Config &cfg);

  /** @brief 析构：释放互斥量。仅 init() 失败的未注册节点会被删除 */
  ~CanTxNode();

  /**
   * @brief 创建槽位锁并把本节点挂到所属总线的发送链表
   * @param cfg 节点配置（须与构造时一致）
   * @return Status::OK=成功；BAD_ARG=配置非法；BUSY=已经初始化；FULL=RTOS 资源不足
   * @note 必须在调度器启动之后、系统进入运行态之前调用。
   */
  Status init(const Config &cfg);

  /**
   * @brief 把数据填进指定槽位
   * @param data 长度为 8/division 的数据
   * @param slot 槽位号（0 ~ division-1）
   * @return OK=成功；其他=节点状态、参数或锁操作错误
   */
  Status fill_data(uint8_t data[], uint8_t slot = 0);

  CanTxNode(const CanTxNode &)            = delete;
  CanTxNode &operator=(const CanTxNode &) = delete;
  CanTxNode(CanTxNode &&)                 = delete;
  CanTxNode &operator=(CanTxNode &&)      = delete;
};


/**
 * @brief 注册发送节点（不划分槽位，占满整帧）
 */
CanTxNode *regist(const CanTxNode::Config &cfg);

/**
 * @brief 注册发送节点（初始化阶段调用）
 *
 * @param cfg  节点配置：总线、标准帧 ID（11 位）、帧内等长槽位数（1/2/4/8）
 * @param slot 本次占用的槽位号（0 ~ division-1）
 * @return 成功返回节点；失败返回 nullptr，并把原因记入系统初始化错误
 *
 * @note 同一总线上 ID 相同的节点会共用（division 必须一致且槽位未被占用）。
 */
CanTxNode *regist(const CanTxNode::Config &cfg, uint8_t slot);

/**
 * @brief 清零并释放指定发送槽位，节点保留供发送任务继续扫描
 *
 * @param node 注册时返回的节点
 * @param slot 待释放的槽位
 * @return OK=成功；其他=参数、注册状态或锁操作错误
 * @note 仅在调度器启动前或正常任务上下文调用；运行时等待节点锁。
 *       清零帧由发送任务后续发出，不保证返回时已发到总线。
 */
Status unregist(CanTxNode *node, uint8_t slot);


#endif // __CAN_TX_NODE_HPP__
