#ifndef CAN_TX_NODE_HPP
#define CAN_TX_NODE_HPP

#include "bsp_can.hpp"

/**
 * @brief 等待系统初始化完成后，按 1 ms 周期扫描 CAN 发送节点。
 * @param argument 未使用的任务参数。
 * @note 节点须在系统进入运行状态前注册；无节点时任务自行退出。
 */
extern "C" void can_tx_task(void *argument);

class CanTxNode
{
private:
  BspCan       *_bspcan;
  CanTxMsg      txBuffer; ///< 消息本身
  uint8_t       division; ///< 同一 CAN 帧划分的等长数据槽位数
  Status        _statu;
  uint8_t       bufferRegister; ///< 消息被注册占用情况
  uint8_t       bufferUnsend;
  osMutexId_t   bufferMutex;
  osMutexAttr_t bufferMutex_Attr;
  char          mutexName[40];
  bool          _registered; ///< 是否已加入发送节点链表

  static uint8_t    node_num;
  static CanTxNode *head;
  static CanTxNode *tail;
  CanTxNode        *next;
  CanTxNode        *last;

  CanTxNode(BspCan &can, uint32_t can_id, uint8_t division = 1);
  ~CanTxNode();
  /**
   * @brief 创建信号量并将节点注册到统一发送链表
   * @return Status::OK 初始化成功；Status::BAD_ARG 配置非法；
   *         Status::BUSY 已经初始化；Status::FULL 内存或 RTOS 资源不足。
   * @note 必须在 osKernelInitialize() 之后且非 ISR 上下文调用。
   */
  Status init(void);

public:
  Status filldata(uint8_t data[], uint8_t slot = 0);

  friend CanTxNode *regist(BspCan &can_item, uint32_t can_id);
  friend CanTxNode *regist(BspCan &can_item, uint32_t can_id, uint8_t division, uint8_t slot);
  friend Status     unregist(CanTxNode *node, uint8_t slot);
  // friend Status unregist(CanTxNode* node);
  // friend Status unregist(CanTxNode* node, uint8_t slot);

  CanTxNode(const CanTxNode &)            = delete;
  CanTxNode &operator=(const CanTxNode &) = delete;
  CanTxNode(CanTxNode &&)                 = delete;
  CanTxNode &operator=(CanTxNode &&)      = delete;

  friend void can_tx_task(void *argument);
};

CanTxNode *regist(BspCan &can_item, uint32_t can_id);
CanTxNode *regist(BspCan &can_item, uint32_t can_id, uint8_t division, uint8_t slot);
// Status unregist(CanTxNode* node);
// Status unregist(CanTxNode* node, uint8_t slot);
void can_tx_task(void *argument);

/**
 * @brief 清零并释放指定发送槽位，保留节点供发送任务继续扫描。
 * @param node 注册时返回的节点。
 * @param slot 待释放的槽位。
 * @return OK 表示释放成功；其他状态表示参数、注册状态或锁操作错误。
 * @note 仅在调度器启动前或正常任务上下文调用；运行时等待节点锁。
 *       清零帧由发送任务后续发送，不保证返回时已发送到总线。
 */
Status unregist(CanTxNode *node, uint8_t slot);

#endif // CAN_TX_NODE_HPP
