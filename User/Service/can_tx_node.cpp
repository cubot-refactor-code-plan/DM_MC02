#include "can_tx_node.hpp"
#include <new>
#include <stdio.h>
#include <string.h>

uint8_t    CanTxNode::node_num = 0;
bool       CanTxNode::frozen   = false;

CanTxNode::CanTxNode(uint32_t can_id, uint8_t division) :
  txBuffer {},
  division(division),
  _statu(Status::NOT_INIT),
  bufferRegister(0),
  bufferUnsend(0),
  bufferMutex(nullptr),
  bufferMutex_Attr {},
  _registered(false),
  next(nullptr),
  last_send_tick(0)
{
  if ((osEventFlagsGet(sysEvent) & SYS_FLAG_RUNNING_BIT) != 0) // 只允许在初始化期使用
  {
    return;
  }
  // 参数校验
  if (division == 0 || division > 8 || 8 % division != 0)
  {
    this->division = 1;
    this->_statu   = Status::BAD_ARG;
    SysInitError(this->_statu);
    return;
  }
  if (can_id > 0x7FFU)
  {
    this->_statu = Status::BAD_ARG;
    SysInitError(this->_statu);
    return;
  }

  txBuffer.std_id = can_id;
}

Status CanTxNode::init(BspCan &can)
{
  // 状态校验
  if ((osEventFlagsGet(sysEvent) & SYS_FLAG_RUNNING_BIT) != 0) // 只允许在初始化期使用
  {
    return Status::NOT_SUPPORTED;
  }
  if (this->_statu == Status::OK)
  {
    return Status::BUSY;
  }
  if (this->_statu != Status::NOT_INIT)
  {
    SysInitError(this->_statu);
    return this->_statu;
  }

  snprintf(mutexName, 40, "Mutex_CanTxNode_%lx", static_cast<unsigned long>(this->txBuffer.std_id));
  bufferMutex_Attr.name = mutexName;
  bufferMutex           = osMutexNew(&bufferMutex_Attr);
  if (bufferMutex == NULL)
  {
    SysInitError(Status::FULL);
    return Status::FULL;
  }

  // 每条总线独立维护单向链表，仅初始化阶段修改。
  next = can.tx_head;
  can.tx_head = this;
  _registered = true;

  this->_statu = Status::OK;
  return Status::OK;
}

CanTxNode::~CanTxNode()
{
  // 仅 init() 失败的未注册节点会被删除；成功节点由 BSP 持有至系统停止。
  if (bufferMutex != nullptr)
    osMutexDelete(bufferMutex);
}

CanTxNode *regist(BspCan &can_item, uint32_t can_id)
{
  return regist(can_item, can_id, 1U, 0U);
}

CanTxNode *regist(BspCan &can_item, uint32_t can_id, uint8_t division, uint8_t slot)
{
  if (sysEvent == NULL)
  {
    return nullptr;
  }
  if (CanTxNode::frozen || (osEventFlagsGet(sysEvent) & SYS_FLAG_RUNNING_BIT) != 0U) // 只允许在初始化期注册
  {
    CanTxNode::frozen = true;
    return nullptr;
  }

  // 参数检查
  if (can_id > 0x7FFU || division == 0U || division > 8U || (8U % division) != 0U || slot >= division)
  {
    SysInitError(Status::BAD_ARG);
    return nullptr;
  }

  // 从链表中查找有无符合条件的Node
  for (CanTxNode *node = can_item.tx_head; node != nullptr; node = node->next)
  {
    if (node->txBuffer.std_id == can_id) // 有
    {
      if (node->division == division && (node->bufferRegister & (1U << slot)) == 0U) // 校验
      {
        node->bufferRegister |= (1U << slot);
        return node; // 合法，返回已有的node
      }
      else // 非法
      {
        SysInitError(Status::FULL);
        return nullptr;
      }
    }
  }

  // 无则创建一个新的Node
  if (CanTxNode::node_num >= 50U)
  {
    SysInitError(Status::FULL);
    return nullptr;
  }
  CanTxNode *node = new (std::nothrow) CanTxNode(can_id, division);
  if (node == nullptr)
  {
    SysInitError(Status::BUSY);
    return nullptr;
  }
  const Status status = node->init(can_item);
  if (status != Status::OK)
  {
    delete node;
    return nullptr;
  }

  node->bufferRegister = (1U << slot);
  CanTxNode::node_num++;
  return node;
}

Status unregist(CanTxNode *node, uint8_t slot)
{
  if (node == nullptr || slot >= node->division)
  {
    return Status::BAD_ARG;
  }
  if (!node->_registered || node->_statu != Status::OK)
  {
    return Status::NOT_INIT;
  }

  // 启动前没有发送任务竞争，且 CMSIS 互斥量获取要求调度器已运行。
  const bool running = xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED;
  if (running && osMutexAcquire(node->bufferMutex, osWaitForever) != osOK)
  {
    return Status::BUSY;
  }

  const uint8_t mask   = static_cast<uint8_t>(1U << slot);
  Status        result = Status::BAD_ARG;
  if ((node->bufferRegister & mask) != 0U)
  {
    memset(node->txBuffer.data + slot * 8U / node->division, 0, 8U / node->division);
    node->bufferRegister &= static_cast<uint8_t>(~mask);
    node->bufferUnsend &= static_cast<uint8_t>(~mask);
    result = Status::OK;
  }
  if (running)
  {
    osMutexRelease(node->bufferMutex);
  }
  return result;
}

Status CanTxNode::filldata(uint8_t data[], uint8_t slot)
{
  if (_statu != Status::OK)
  {
    return _statu;
  }
  if (_registered != true)
  {
    return Status::NOT_INIT;
  }
  if (data == nullptr || // 空指针
      slot >= division)  // 越界
  {
    return Status::BAD_ARG;
  }

  if (osMutexAcquire(bufferMutex, pdMS_TO_TICKS(3U)) != osOK)
  {
    return Status::BUSY;
  }
  // 槽位占用位图会在其他电机析构时修改，必须在节点锁内读取。
  if ((bufferRegister & (1U << slot)) == 0U)
  {
    osMutexRelease(bufferMutex);
    return Status::BAD_ARG;
  }
  memcpy(txBuffer.data + slot * 8 / division, data, 8 / division);
  bufferUnsend |= (1U << slot);
  osMutexRelease(bufferMutex);

  return Status::OK;
}
