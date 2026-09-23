#include "motor_txBuffers.hpp"
#include <new>
#include <stdio.h>
#include <string.h>

uint8_t      MotorTxNode::node_num = 0;
MotorTxNode *MotorTxNode::head = nullptr;
MotorTxNode *MotorTxNode::tail = nullptr;

MotorTxNode::MotorTxNode(BspCan &can, uint32_t can_id, uint8_t division) :
  _bspcan(&can),
  txBuffer{},
  division(division),
  _statu(Status::NOT_INIT),
  bufferRegister(0),
  bufferUnsend(0),
  bufferMutex_Attr{},
  _registered(false),
  next(nullptr),
  last(nullptr)
{
  if ((osEventFlagsGet(sysEvent) & SYS_FLAG_RUNNING_BIT) != 0)  // 只允许在初始化期使用
  {
    return;
  }
  // 参数校验
  if (division == 0 || division > 8 || 8 % division != 0)
  {
    this->division = 1;
    this->_statu = Status::BAD_ARG;
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

Status MotorTxNode::init(void)
{
  // 状态校验
  if ( (osEventFlagsGet(sysEvent) & SYS_FLAG_RUNNING_BIT) != 0 )  // 只允许在初始化期使用
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

  snprintf(mutexName, 40, "Mutex_MotorTxNode_%lx",static_cast<unsigned long>(this->txBuffer.std_id));
  bufferMutex_Attr.name = mutexName;
  bufferMutex = osMutexNew(&bufferMutex_Attr);
  if (bufferMutex == NULL) 
  {
    SysInitError(Status::FULL);
    return Status::FULL;
  }

  // 加入链表
  taskENTER_CRITICAL();
  if (head == nullptr) // 空链表
  {
    head = this;
    tail = this;
    this->last = nullptr;
  }
  else
  {
    tail->next = this;
    this->last = tail;
    tail = this;
  }
  this->_registered = true;
  taskEXIT_CRITICAL();

  this->_statu = Status::OK;
  return Status::OK;
}

MotorTxNode::~MotorTxNode()
{
  _bspcan = nullptr;
  bufferRegister = 0;
  if (bufferMutex != NULL)
  {
    osMutexDelete(bufferMutex);
  }

  if (this->_registered)
  {
    taskENTER_CRITICAL();
    if (head == tail)
    {
      head = nullptr;
      tail = nullptr;
    }
    else if (this == head)
    {
      head = this->next;
      head->last = nullptr;
    }
    else if (this == tail)
    {
      tail = this->last;
      tail->next = nullptr;
    }
    else
    {
      this->last->next = this->next;
      this->next->last = this->last;
    }
    this->_registered = false;
    taskEXIT_CRITICAL();
  }
}

extern "C" void MotorTxTask(void *argument)
{
  (void)argument;

  MotorTxNode *p;
  SysFlagWaitRunning();
  if (MotorTxNode::head == nullptr || MotorTxNode::tail == nullptr)
  {
    osThreadExit();
    return;
  }

  TickType_t *node_last_send_tick = new (std::nothrow) TickType_t[MotorTxNode::node_num];
  if (node_last_send_tick == nullptr)
  {
    SysFlagSet(Status::FULL);
    osThreadExit();
    return;
  }
  for (uint8_t i = 0; i < MotorTxNode::node_num; i ++)
  {
    node_last_send_tick[i] = xTaskGetTickCount();
  }

  const TickType_t SEND_TIMEOUT_TICKS = pdMS_TO_TICKS(5U); // 节点距上次成功入队达到 5 ms 时触发补发

  TickType_t wake_time = xTaskGetTickCount();
  for (;;)
  {
    uint8_t id = 0;
    for (p = MotorTxNode::head; p != nullptr; p = p->next)
    {
      if ( osMutexAcquire(p->bufferMutex, pdMS_TO_TICKS(1U)) == osOK )
      {   // 尝试获取互斥量
        if ( p->bufferUnsend == p->bufferRegister )
        {   // 已经准备好发送
          if ( p->_bspcan->send(p->txBuffer.std_id, p->txBuffer.data) == Status::OK )
          {   // 发送成功才更新标记
            p->bufferUnsend = 0;
            node_last_send_tick[id] = xTaskGetTickCount();
          }
        }
        else if (static_cast<TickType_t>(xTaskGetTickCount() - node_last_send_tick[id]) >= SEND_TIMEOUT_TICKS)
        {   // 发送间隔超时，直接发送
          if ( p->_bspcan->send(p->txBuffer.std_id, p->txBuffer.data) == Status::OK )
          {
            p->bufferUnsend = 0;
            node_last_send_tick[id] = xTaskGetTickCount();
          }
        }
        osMutexRelease(p->bufferMutex);
      }
      id ++;
    }
    // 按 1 ms 周期调度，避免将本轮扫描耗时累加到等待周期中。
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(1U));
  }
}

MotorTxNode* regist(BspCan& can_item, uint32_t can_id)
{
  return regist(can_item, can_id, 1U, 0U);
}

MotorTxNode* regist(BspCan& can_item, uint32_t can_id, uint8_t division, uint8_t slot)
{
  if ( sysEvent == NULL )
  {
    return nullptr;
  }
  if ( (osEventFlagsGet(sysEvent) & SYS_FLAG_RUNNING_BIT) != 0U ) // 只允许在初始化期注册
  {
    return nullptr;
  }

  // 参数检查
  if (can_id > 0x7FFU ||
      division == 0U ||
      division > 8U ||
      (8U % division) != 0U ||
      slot >= division)
  {
    SysInitError(Status::BAD_ARG);
    return nullptr;
  }

  // 从链表中查找有无符合条件的Node
  for (MotorTxNode* node = MotorTxNode::head; node != nullptr; node = node->next)
  {
    if (node->_bspcan == &can_item && node->txBuffer.std_id == can_id) // 有
    {
      if (node->division == division && 
          (node->bufferRegister & (1U << slot)) == 0U) // 校验
      {
        node->bufferRegister |= (1U << slot);
        return node;  // 合法，返回已有的node
      }
      else  // 非法
      {
        SysInitError(Status::FULL);
        return nullptr;
      }
    }
  }

  // 无则创建一个新的Node
  if (MotorTxNode:: node_num > 50)
  {
    SysInitError(Status::FULL);
    return nullptr;;
  }
  MotorTxNode *node = new (std::nothrow) MotorTxNode(can_item, can_id, division);
  if (node == nullptr)
  {
    SysInitError(Status::BUSY);
    return nullptr;
  }
  const Status status = node->init();
  if (status != Status::OK)
  {
    delete node;
    return nullptr;
  }

  node->bufferRegister = (1U << slot);
  MotorTxNode::node_num ++;
  return node;
}

Status MotorTxNode::filldata(uint8_t data[], uint8_t slot)
{
  if (_statu != Status::OK)
  {
    return _statu;
  }
  if (_registered != true || bufferRegister == 0U)
  {
    return Status::NOT_INIT;
  }
  if (data == nullptr || // 空指针
      slot >= division || // 越界
      (bufferRegister & (1U << slot)) == 0 )  // 写入未被注册的内容
  {
    return Status::BAD_ARG;
  }

  if (osMutexAcquire(bufferMutex, pdMS_TO_TICKS(3U)) != osOK)
  {
    return Status::BUSY;
  }
  memcpy(txBuffer.data + slot * 8 / division, data, 8 / division);
  bufferUnsend |= (1U << slot);
  osMutexRelease(bufferMutex);

  return Status::OK;
}
