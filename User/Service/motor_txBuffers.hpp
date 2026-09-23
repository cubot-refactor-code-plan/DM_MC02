#ifndef MOTOR_TXBUFFERS_HPP
#define MOTOR_TXBUFFERS_HPP

#include "bsp_can.hpp"

/**
 * @brief 等待系统初始化完成后，按 1 ms 周期扫描电机发送节点。
 * @param argument 未使用的任务参数。
 * @note 节点须在系统进入运行状态前注册；无节点时任务自行退出。
 */
extern "C" void MotorTxTask(void *argument);

class MotorTxNode
{
private:
  BspCan         *_bspcan;
  CanTxMsg        txBuffer;      ///< 消息本身
  uint8_t         division;      ///< 同一节点最多被几个同类型电机共用
  Status          _statu;
  uint8_t         bufferRegister; ///< 消息被注册占用情况
  uint8_t         bufferUnsend;
  osMutexId_t     bufferMutex;
  osMutexAttr_t   bufferMutex_Attr;
  char            mutexName[40];
  bool            _registered;   ///< 是否已加入发送节点链表

  static uint8_t     node_num;
  static MotorTxNode *head;
  static MotorTxNode *tail;
  MotorTxNode        *next;
  MotorTxNode        *last;

  MotorTxNode(BspCan &can, uint32_t can_id, uint8_t division = 1);
  ~MotorTxNode();
  /**
   * @brief 创建信号量并将节点注册到统一发送链表
   * @return Status::OK 初始化成功；Status::BAD_ARG 配置非法；
   *         Status::BUSY 已经初始化；Status::FULL 内存或 RTOS 资源不足。
   * @note 必须在 osKernelInitialize() 之后且非 ISR 上下文调用。
   */
  Status init(void);

public:

  
  Status filldata(uint8_t data[], uint8_t slot = 0);

  friend MotorTxNode* regist(BspCan& can_item, uint32_t can_id);
  friend MotorTxNode* regist(BspCan& can_item, uint32_t can_id, uint8_t division, uint8_t slot);
  // friend Status unregist(MotorTxNode* node);
  // friend Status unregist(MotorTxNode* node, uint8_t slot);

  MotorTxNode(const MotorTxNode &) = delete;
  MotorTxNode &operator=(const MotorTxNode &) = delete;
  MotorTxNode(MotorTxNode &&) = delete;
  MotorTxNode &operator=(MotorTxNode &&) = delete;

  friend void MotorTxTask(void *argument);
};

MotorTxNode* regist(BspCan& can_item, uint32_t can_id);
MotorTxNode* regist(BspCan& can_item, uint32_t can_id, uint8_t division, uint8_t slot);
// Status unregist(MotorTxNode* node);
// Status unregist(MotorTxNode* node, uint8_t slot);
void MotorTxTask(void *argument);

#endif // MOTOR_TXBUFFERS_HPP
