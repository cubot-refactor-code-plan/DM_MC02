#include "can_tx_node.hpp"

#include "can_bus.hpp"

#include <new>
#include <string.h>


uint8_t CanTxNode::node_num = 0;

CanTxNode::CanTxNode(const Config &cfg) :
  txBuffer {},
  division(cfg.division),
  _statu(Status::NOT_INIT),
  bufferRegister(0),
  bufferUnsend(0),
  bufferMutex(nullptr),
  _registered(false),
  next(nullptr),
  last_send_tick(0)
{
  if (division == 0 || division > 8 || 8 % division != 0)
  {
    this->division = 1;
    this->_statu   = Status::BAD_ARG;
    return;
  }
  if (cfg.can_id > 0x7FFU)
  {
    this->_statu = Status::BAD_ARG;
    return;
  }

  txBuffer.std_id = cfg.can_id;
}

Status CanTxNode::init(const Config &cfg)
{
  // 状态校验（只允许在初始化期使用，且构造参数必须合法）
  if (sys_flag_running())
  {
    return Status::NOT_SUPPORTED;
  }
  if (cfg.bus == nullptr)
  {
    return Status::BAD_ARG;
  }
  if (this->_statu == Status::OK)
  {
    return Status::BUSY;
  }
  if (this->_statu != Status::NOT_INIT)
  {
    sys_init_error(this->_statu);
    return this->_statu;
  }

  bufferMutex = xSemaphoreCreateMutex();
  if (bufferMutex == nullptr)
  {
    sys_init_error(Status::FULL);
    return Status::FULL;
  }

  // 每条总线独立维护单向链表，仅初始化阶段修改。
  next             = cfg.bus->tx_head;
  cfg.bus->tx_head = this;
  _registered      = true;

  this->_statu = Status::OK;
  return Status::OK;
}

CanTxNode::~CanTxNode()
{
  // 仅 init() 失败的未注册节点会被删除；成功节点由 CanBus 持有至系统停止。
  if (bufferMutex != nullptr)
  {
    vSemaphoreDelete(bufferMutex);
    bufferMutex = nullptr;
  }
}

CanTxNode *regist(const CanTxNode::Config &cfg)
{
  return regist(cfg, 0U);
}

CanTxNode *regist(const CanTxNode::Config &cfg, uint8_t slot)
{
  if (sys_event == nullptr || cfg.bus == nullptr)
  {
    return nullptr;
  }
  if (CanBus::frozen || sys_flag_running()) // 只允许在初始化期注册
  {
    CanBus::frozen = true;
    return nullptr;
  }
  // 参数检查
  if (cfg.can_id > 0x7FFU || cfg.division == 0U || cfg.division > 8U || (8U % cfg.division) != 0U || slot >= cfg.division)
  {
    sys_init_error(Status::BAD_ARG);
    return nullptr;
  }

  // 从链表中查找有无符合条件的Node
  for (CanTxNode *node = cfg.bus->tx_head; node != nullptr; node = node->next)
  {
    if (node->txBuffer.std_id == cfg.can_id) // 有
    {
      if (node->division == cfg.division && (node->bufferRegister & (1U << slot)) == 0U) // 校验
      {
        node->bufferRegister |= (1U << slot);
        return node; // 合法，返回已有的node
      }
      else // 非法
      {
        sys_init_error(Status::FULL);
        return nullptr;
      }
    }
  }

  // 无则创建一个新的Node
  if (CanTxNode::node_num >= 50U)
  {
    sys_init_error(Status::FULL);
    return nullptr;
  }
  CanTxNode *node = new (std::nothrow) CanTxNode(cfg);
  if (node == nullptr)
  {
    sys_init_error(Status::BUSY);
    return nullptr;
  }
  const Status status = node->init(cfg);
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

  // 启动前没有发送任务竞争，且互斥量获取要求调度器已运行。
  const bool running = xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED;
  if (running && xSemaphoreTake(node->bufferMutex, portMAX_DELAY) != pdTRUE)
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
    xSemaphoreGive(node->bufferMutex);
  }
  return result;
}

Status CanTxNode::fill_data(uint8_t data[], uint8_t slot)
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

  if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(3U)) != pdTRUE)
  {
    return Status::BUSY;
  }
  // 槽位占用位图会在其他电机析构时修改，必须在节点锁内读取。
  if ((bufferRegister & (1U << slot)) == 0U)
  {
    xSemaphoreGive(bufferMutex);
    return Status::BAD_ARG;
  }
  memcpy(txBuffer.data + slot * 8 / division, data, 8 / division);
  bufferUnsend |= (1U << slot);
  xSemaphoreGive(bufferMutex);

  return Status::OK;
}
